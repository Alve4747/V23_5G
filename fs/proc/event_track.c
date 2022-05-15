/**
 * Copyright (C) 2019 vivo Co., Ltd.
 *
 * Event tracking for vcode, refs: fmea.c
 *
 * Version: 1.0
 * Author: xiezhulin
 * Date: May 17, 2019
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "internal.h"

#ifndef AID_SYSTEM
#define AID_SYSTEM 1000
#endif
#define PROC_FS_NAME "event_track"
#define ENTRY_FMT "%s,%s\n"

#define NAME_LEN_MAX 32
#define DATA_LEN_MAX 128
#define FIFO_SIZE_MAX 512
#define ENTRY_LEN_MAX 384


struct event_track_entry {
	size_t len;
	char data[ENTRY_LEN_MAX];
};

struct event_track {
	wait_queue_head_t event_track_waitq;
	spinlock_t lock;
	struct dentry *dent;
	DECLARE_KFIFO_PTR(event_track_fifo, struct event_track_entry *);
};

static struct event_track *gp_event_track;

static int event_track_write_entry(struct event_track *event_track, struct event_track_entry *entry)
{
	int err = 0;
	struct event_track_entry *drop_entry;

	if (!entry->len) {
		err = -EFAULT;
		goto out;
	}
	if (entry->data[entry->len-1] != '\n')
		entry->data[entry->len-1] = '\n';

	if (kfifo_is_full(&event_track->event_track_fifo)) {
		printk(KERN_WARNING "event track fifo is full, drop the oldest\n");
		if (!kfifo_get(&event_track->event_track_fifo, &drop_entry)) {
			err = -EFAULT;
			goto out;
		}
	}

	if (!kfifo_put(&event_track->event_track_fifo, entry)) {
		err = -EFAULT;
		goto out;
	}

out:
	return err;
}

/**
 * Kernel and Native 层普通埋点API
 *
 * @param module_id 模块id
 * @param event_id  事件id
 * @param params 埋点内容字串，字段分隔符为&，
 *               字符串中不允许出现此分隔符，
 *               字段由key和value组成，分隔符为:
 *               .e.g: "name:xiezhulin&age:18"
 * @return 成功返回0, 否则返回对应错误码
 */
int track_event(const char *module_id, const char *event_id, const char *params)
{
	return 0;
}
EXPORT_SYMBOL(track_event);

/**
 * Kernel and Native 层可变参数普通埋点API
 *
 * @param module_id 模块id
 * @param event_id  事件id
 * @param fmt 埋点内容字串或格式化字符串，
 *            字段分隔符为&，字符串中不允许出现此分隔符，
 *            字段由key和value组成，分隔符为:
 *            .e.g: "name:xiezhulin&age:18"
 * @param ... 可变参数列表
 *            .e.g: "%s:%s&%s:%d", "name", "xiezhulin", "age", 18
 * @return 成功返回0, 否则返回对应错误码
 */
int track_event_va(const char *module_id, const char *event_id, const char *fmt, ...)
{
	return 0;
}
EXPORT_SYMBOL(track_event_va);

static int event_track_open(struct inode *inode, struct file *file)
{
	if (!gp_event_track)
		return -ENOENT;
	file->private_data = gp_event_track;
	return nonseekable_open(inode, file);
}

static int event_track_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t event_track_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int err = 0;
	struct event_track *event_track = file->private_data;
	struct event_track_entry *entry;
	struct event_track_entry entout;

next:
	if ((file->f_flags & O_NONBLOCK) && kfifo_is_empty(&event_track->event_track_fifo)) {
		err = -EAGAIN;
		goto out;
	} else {
		err = wait_event_interruptible(event_track->event_track_waitq,
		!kfifo_is_empty(&event_track->event_track_fifo));
		if (err)
			goto out;
	}

	spin_lock_irq(&event_track->lock);
	if (kfifo_is_empty(&event_track->event_track_fifo)) {
		spin_unlock_irq(&event_track->lock);
		goto next;
	}

	if (!kfifo_peek(&event_track->event_track_fifo, &entry)) {
		err = -EFAULT;
		goto unlock;
	}

	if (count >= entry->len) {
		entout.len = entry->len;
		memcpy(entout.data, entry->data, entout.len);
		if (kfifo_get(&event_track->event_track_fifo, &entry))
			kfree(entry);
	} else {
		entout.len = count;
		memcpy(entout.data, entry->data, entout.len);
		entry->len = entry->len - entout.len;
		memmove(entry->data, entry->data + entout.len, entry->len);
	}

unlock:
	spin_unlock_irq(&event_track->lock);
	if (!err && copy_to_user(buf, entout.data, entout.len))
		err = -EFAULT;
	if (!err)
		err = entout.len;
out:
	return err;
}

static ssize_t event_track_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	int err = 0;
	size_t max = ENTRY_LEN_MAX;

	struct event_track *event_track = file->private_data;
	struct event_track_entry *entry = kzalloc(sizeof(struct event_track_entry), GFP_KERNEL);

	if (!count)
		goto error;
	if (!entry) {
		err = -ENOMEM;
		goto out;
	}
	entry->len = min(count, max);
	if (copy_from_user(entry->data, buf, entry->len)) {
		err = -EFAULT;
		goto error;
	}
	spin_lock_irq(&event_track->lock);
	err = event_track_write_entry(event_track, entry);
	spin_unlock_irq(&event_track->lock);
	if (err)
		goto error;
	else
		wake_up_interruptible(&event_track->event_track_waitq);
	return count;
error:
	kfree(entry);
out:
	return err;
}

static const struct file_operations proc_event_track_ops = {
	.open = event_track_open,
	.release = event_track_release,
	.read = event_track_read,
	.write = event_track_write,
	.llseek = noop_llseek,
};

#ifdef CONFIG_DEBUG_FS
static int event_track_debug_read(struct seq_file *m, void *v)
{
	struct event_track *event_track = m->private;

	seq_printf(m, "fifo-size:%d, fifo-len:%d, full:%d, empty:%d\n",
		kfifo_size(&event_track->event_track_fifo),
		kfifo_len(&event_track->event_track_fifo),
		kfifo_is_full(&event_track->event_track_fifo),
		kfifo_is_empty(&event_track->event_track_fifo));

	return 0;
}

static ssize_t event_track_debug_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	int err = 0;
	size_t max = ENTRY_LEN_MAX;
	size_t sz = min(count, max);
	struct event_track_entry entry;

	memset(&entry, 0, sizeof(entry));
	if (copy_from_user(entry.data, buf, sz)) {
		err = -EFAULT;
		goto out;
	}

	err = track_event("K10086", "K10086|119", "attr:et&value:168");
	err |= track_event_va("K10086", "K10086|119", "%s&%s:%d\n",
		entry.data, "value", 168);
	err |= track_event_va("K10086", "K10086|119", "%s&%s:%s&%s:%d&%s:0x%2X\n",
		entry.data, "attr", "et", "value", 168, "hello", 0xaa);
	if (!err)
		err = count;

out:
	return err;
}

static int event_track_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, event_track_debug_read, inode->i_private);
}

static const struct file_operations debug_event_track_ops = {
	.open = event_track_debug_open,
	.release = single_release,
	.read = seq_read,
	.write = event_track_debug_write,
	.llseek = seq_lseek,
};

static int event_track_init_debugfs(struct event_track *event_track)
{
	struct dentry *dent;

	event_track->dent = debugfs_create_dir(PROC_FS_NAME, NULL);
	if (IS_ERR(event_track->dent)) {
		pr_err("couldn't create debugfs dir\n");
		return -EFAULT;
	}
	dent = debugfs_create_file("debug", 0660,
			event_track->dent, event_track, &debug_event_track_ops);
	if (IS_ERR(dent))
		return -EFAULT;
	return 0;
}

static void event_track_exit_debugfs(struct event_track *event_track)
{
	if (event_track->dent) {
		debugfs_remove_recursive(event_track->dent);
		event_track->dent = NULL;
	}
}
#endif /* CONFIG_DEBUG_FS */

static int __init proc_event_track_init(void)
{
	int err;
	struct event_track *event_track;
	struct proc_dir_entry *pdent;

	gp_event_track = 0;
	event_track = kzalloc(sizeof(*event_track), GFP_KERNEL);
	if (!event_track) {
		pr_err("could not allocate event_track\n");
		return -ENOMEM;
	}
	gp_event_track = event_track;

	err = kfifo_alloc(&event_track->event_track_fifo, FIFO_SIZE_MAX, GFP_KERNEL);
	if (err) {
		pr_err("could not allocate event_track FIFO\n");
		goto out_err;
	}
	spin_lock_init(&event_track->lock);
	init_waitqueue_head(&event_track->event_track_waitq);

	pdent = proc_create(PROC_FS_NAME, 0660, NULL, &proc_event_track_ops);
	if (!pdent) {
		err = -EFAULT;
		goto out_err2;
	}

	pdent->gid.val = AID_SYSTEM;
	pdent->uid.val = AID_SYSTEM;

#ifdef CONFIG_DEBUG_FS
	if (event_track_init_debugfs(event_track)) {
		err = -EFAULT;
		if (event_track->dent)
			debugfs_remove_recursive(event_track->dent);
		remove_proc_entry(PROC_FS_NAME, NULL);
		goto out_err2;
	}
#endif

	return 0;

out_err2:
	kfifo_free(&event_track->event_track_fifo);
out_err:
	kfree(event_track);
	gp_event_track = NULL;
	return err;
}

static __exit void proc_event_track_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	event_track_exit_debugfs(gp_event_track);
#endif
	remove_proc_entry(PROC_FS_NAME, NULL);
	kfifo_free(&gp_event_track->event_track_fifo);
	kfree(gp_event_track);
	gp_event_track = NULL;
}

module_init(proc_event_track_init);
module_exit(proc_event_track_exit);

MODULE_AUTHOR("Ronny<xiezhulin@vivo.com>");
MODULE_DESCRIPTION("Event Tracking");
MODULE_LICENSE("GPL");

