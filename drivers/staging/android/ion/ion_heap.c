/*
 * drivers/staging/android/ion/ion_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/rtmutex.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/scatterlist.h>
#include <linux/vmalloc.h>
#include "ion.h"
#include "ion_priv.h"
#include <linux/cpuset.h>


#ifdef CONFIG_RSC_ION_OPT
#include <../kernel/sched/sched.h>
#include "vivo_ion_private.h"

/*100MB*/
#define RSC_MAX_HEAPDRAIN_PAGE (100 << (20 - PAGE_SHIFT))
/*10MB*/
#define RSC_MAX_HEAPDRAIN_THR_PAGE (10 << (20 - PAGE_SHIFT))

unsigned int rsc_ion_pool[RSC_ION_POOL_LEVEL] = {
	500,
	450,
	400,
	350,
};
module_param_array_named(pool, rsc_ion_pool, uint, NULL,
			 S_IRUGO | S_IWUSR);

unsigned int ion_priority[RSC_ION_POOL_LEVEL] = {
	5,//7
	7,//5
	9,//3
	11,//1
};

module_param_array_named(priority, ion_priority, uint, NULL,
			 S_IRUGO | S_IWUSR);
#endif

void *ion_heap_map_kernel(struct ion_heap *heap,
			  struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages) {
		IONMSG("%s vmalloc failed pages is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg->length) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++)
			*(tmp++) = page++;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr) {
		IONMSG("%s vmap failed vaddr is null.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	return vaddr;
}

void ion_heap_unmap_kernel(struct ion_heap *heap,
			   struct ion_buffer *buffer)
{
	vunmap(buffer->vaddr);
}

int ion_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
		      struct vm_area_struct *vma)
{
	struct sg_table *table = buffer->sg_table;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	struct scatterlist *sg;
	int i;
	int ret;

#if defined(CONFIG_MTK_IOMMU_PGTABLE_EXT) && \
	(CONFIG_MTK_IOMMU_PGTABLE_EXT > 32)
	if (heap->ops->get_table)
		heap->ops->get_table(buffer, table);
	if (!table)
		return -1;
#endif

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len = sg->length;

		if (offset >= sg->length) {
			offset -= sg->length;
			continue;
		} else if (offset) {
			page += offset / PAGE_SIZE;
			len = sg->length - offset;
			offset = 0;
		}
		len = min(len, remainder);
		ret = remap_pfn_range(vma, addr, page_to_pfn(page), len,
				vma->vm_page_prot);
		if (ret) {
			IONMSG("%s h:%d remap fail 0x%p, %lu, %lu, %lu, %d.\n",
			       __func__, heap->id, vma, addr,
			       page_to_pfn(page), len, ret);
			return ret;
		}
		addr += len;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

static int ion_heap_clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vmap(pages, num, VM_MAP, pgprot);

	if (!addr) {
		IONMSG("%s vm_map_ram failed addr is null.\n", __func__);
		return -ENOMEM;
	}
	memset(addr, 0, PAGE_SIZE * num);
	vunmap(addr);

	return 0;
}

static int ion_heap_sglist_zero(struct scatterlist *sgl, unsigned int nents,
				pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page *pages[32];

	for_each_sg_page(sgl, &piter, nents, 0) {
		pages[p++] = sg_page_iter_page(&piter);
		if (p == ARRAY_SIZE(pages)) {
			ret = ion_heap_clear_pages(pages, p, pgprot);
			if (ret) {
				IONMSG("%s ion_heap_clear_pages failed.\n",
				       __func__);
				return ret;
			}
			p = 0;
		}
	}
	if (p)
		ret = ion_heap_clear_pages(pages, p, pgprot);

	return ret;
}

int ion_heap_buffer_zero(struct ion_buffer *buffer)
{
	struct sg_table *table = buffer->sg_table;
	pgprot_t pgprot;

	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	return ion_heap_sglist_zero(table->sgl, table->nents, pgprot);
}

int ion_heap_pages_zero(struct page *page, size_t size, pgprot_t pgprot)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	return ion_heap_sglist_zero(&sg, 1, pgprot);
}

void ion_heap_freelist_add(struct ion_heap *heap, struct ion_buffer *buffer)
{
	long nice;
	size_t free_list_size = 0;
	size_t unit = 200 * 1024 * 1024; //200M

	spin_lock(&heap->free_lock);
	list_add(&buffer->list, &heap->free_list);
	heap->free_list_size += buffer->size;

	free_list_size = heap->free_list_size;
	switch (free_list_size / unit) {
	case 0:
	case 1:
	case 2:
		nice = 0;
		break;
	case 3:
	case 4:
		nice = -5;
		break;
	default:
		nice = -10;
		break;
	}
	spin_unlock(&heap->free_lock);

	if (free_list_size > unit) {
		IONMSG(
			"%s: free_size=%zu,heap_id:%u,nice:%ld\n",
			__func__, heap->free_list_size, heap->id, nice);
	}

	set_user_nice(heap->task, nice);
	wake_up(&heap->waitqueue);
}

size_t ion_heap_freelist_size(struct ion_heap *heap)
{
	size_t size;

	spin_lock(&heap->free_lock);
	size = heap->free_list_size;
	spin_unlock(&heap->free_lock);

	return size;
}

static size_t _ion_heap_freelist_drain(struct ion_heap *heap, size_t size,
				       bool skip_pools)
{
	struct ion_buffer *buffer;
	size_t total_drained = 0;

	if (ion_heap_freelist_size(heap) == 0)
		return 0;

	spin_lock(&heap->free_lock);
	if (size == 0)
		size = heap->free_list_size;

	while (!list_empty(&heap->free_list)) {
		if (total_drained >= size)
			break;
		buffer = list_first_entry(&heap->free_list, struct ion_buffer,
					  list);
		list_del(&buffer->list);
		heap->free_list_size -= buffer->size;
		if (skip_pools)
			buffer->private_flags |= ION_PRIV_FLAG_SHRINKER_FREE;
		total_drained += buffer->size;
		spin_unlock(&heap->free_lock);
		ion_buffer_destroy(buffer);
		spin_lock(&heap->free_lock);
	}
	spin_unlock(&heap->free_lock);

	return total_drained;
}

size_t ion_heap_freelist_drain(struct ion_heap *heap, size_t size)
{
	return _ion_heap_freelist_drain(heap, size, false);
}

size_t ion_heap_freelist_shrink(struct ion_heap *heap, size_t size)
{
	return _ion_heap_freelist_drain(heap, size, true);
}

static int ion_heap_deferred_free(void *data)
{
	struct ion_heap *heap = data;

	while (true) {
		struct ion_buffer *buffer;

		wait_event_freezable(heap->waitqueue,
				     ion_heap_freelist_size(heap) > 0);

		spin_lock(&heap->free_lock);
		if (list_empty(&heap->free_list)) {
			spin_unlock(&heap->free_lock);
			continue;
		}
		buffer = list_first_entry(&heap->free_list, struct ion_buffer,
					  list);
		list_del(&buffer->list);
		heap->free_list_size -= buffer->size;
		spin_unlock(&heap->free_lock);
		ion_buffer_destroy(buffer);
	}

	return 0;
}

int ion_heap_init_deferred_free(struct ion_heap *heap)
{
	struct sched_param param = { .sched_priority = 120 };

	INIT_LIST_HEAD(&heap->free_list);
	init_waitqueue_head(&heap->waitqueue);
	heap->task = kthread_run(ion_heap_deferred_free, heap,
				 "%s", heap->name);
	if (IS_ERR(heap->task)) {
		pr_err("%s: creating thread for deferred free failed\n",
		       __func__);
		return PTR_ERR_OR_ZERO(heap->task);
	}
	sched_setscheduler(heap->task, SCHED_NORMAL, &param);
	return 0;
}

#ifdef CONFIG_RSC_ION_OPT
static unsigned long ion_heap_shrink_count(struct shrinker *shrinker,
					   struct shrink_control *sc)
{
	struct ion_heap *heap = container_of(shrinker, struct ion_heap,
					     shrinker);
	int total = 0;
	int heap_free;

	total = ion_heap_freelist_size(heap) / PAGE_SIZE;

	heap_free = total;
	heap->priority = sc->priority;


	if (heap->ops->shrink)
		total += heap->ops->shrink(heap, sc->gfp_mask, 0);

	BUILD_BUG_ON(DEF_PRIORITY < 11);
	BUILD_BUG_ON(RSC_ION_POOL_LEVEL < 4);

	if (heap->rsc_reserve_ion_enable) {
		/*
		* prevent pool page too more, while system use too many io pages.
		*/
		if ((global_zone_page_state(NR_ION) > rsc_system_max_ion_page) || rsc_recovery_survival_mode) {
			printk("[RSC] ion_heap_shrink overload shrink!!! order: %d %lu > %d priority: %d total %lu\n",
				global_zone_page_state(NR_ION), rsc_system_max_ion_page, sc->priority, total);
			return total;
		}

		if (task_in_drop_caches_ota(current) || (sc->priority >= (DEF_PRIORITY - ion_priority[0]))) {
			if (total >= (rsc_ion_pool[0] << (20 - PAGE_SHIFT)))
				return total - (rsc_ion_pool[0] << (20 - PAGE_SHIFT));
			else
				goto out_drain;
		} else if (sc->priority >= (DEF_PRIORITY - ion_priority[1])) {
			if (total >= (rsc_ion_pool[1]  << (20 - PAGE_SHIFT)))
				return total - (rsc_ion_pool[1]  << (20 - PAGE_SHIFT));
			else
				goto out_drain;
		} else if (sc->priority >= (DEF_PRIORITY - ion_priority[2])) {
			if (total >= (rsc_ion_pool[2]  << (20 - PAGE_SHIFT)))
				return total - (rsc_ion_pool[2]  << (20 - PAGE_SHIFT));
			else
				goto out_drain;
		} else if (sc->priority >= (DEF_PRIORITY - ion_priority[3])) {
			if (total >= (rsc_ion_pool[3]  << (20 - PAGE_SHIFT)))
				return total - (rsc_ion_pool[3]  << (20 - PAGE_SHIFT));
			else
				goto out_drain;
		} else {
			/*
			* on first OTA update bootup,  applypatch will do drop_caches
			* echo 3 > /proc/sys/vm/drop_caches,
			* it will reclaim all ion pool pages.
			* see /bootable/recovery/applypatch/applypatch.cpp
			* WriteToPartition()
			* {
			* 	unique_fd dc(ota_open("/proc/sys/vm/drop_caches", O_WRONLY));
			* 	if (TEMP_FAILURE_RETRY(ota_write(dc, "3\n", 2)) {};
			* }
			*/
			if (total && (task_in_drop_caches(current) ||
				(global_node_page_state(NR_FILE_PAGES) > reclaim_file_cache_page))) {
				/*
				<4>[   67.353677][12-16 06:34:02]  [5:	   applypatch: 4890] [RSC] ion_heap_shrink priority very low 366110 > 655360 priority: 0 total 306007 return: 50007
				<4>[   67.364760][12-16 06:34:02]  [5:	   applypatch: 4890] [RSC] ion_heap_shrink priority very low 74951 > 655360 priority: 0 total 0 return: 0
				*/
				printk("[RSC] ion_heap_shrink priority very low ion %lu maxion %d priority: %d total %d "
						"return: %d filecache: %lu anon: %lu srec: %lu sunrec: %lu in dropcache%d ota%d mode%d\n",
					global_zone_page_state(NR_ION), rsc_system_max_ion_page, sc->priority, total,
					(total >= (rsc_ion_pool[3] << (20 - PAGE_SHIFT)))?total - (rsc_ion_pool[3] << (20 - PAGE_SHIFT)):0,
					global_node_page_state(NR_FILE_PAGES), global_node_page_state(NR_ANON_MAPPED),
					global_node_page_state(NR_SLAB_RECLAIMABLE), global_node_page_state(NR_SLAB_UNRECLAIMABLE),
					!!task_in_drop_caches(current), !!task_in_drop_caches_ota(current), rsc_recovery_survival_mode);
				if (total >= (rsc_ion_pool[3]  << (20 - PAGE_SHIFT)))
					return total - (rsc_ion_pool[3]  << (20 - PAGE_SHIFT));
				else
					goto out_drain;
			}
		}
	}

	return total;

out_drain:
	if (heap_free >= RSC_MAX_HEAPDRAIN_THR_PAGE) {
		int on_rq = heap->task?heap->task->on_rq:0xbeef;

		if (on_rq != TASK_ON_RQ_QUEUED) {
			int drain_max_size = min(heap_free, RSC_MAX_HEAPDRAIN_PAGE);

			if (heap->flags & ION_HEAP_FLAG_DEFER_FREE) {
				ion_heap_freelist_drain(heap, drain_max_size);
				printk("[RSC] ion_heap_shrink priority very low ion %lu maxion %d priority: %d total %d "
					"filecache: %lu anon: %lu srec: %lu sunrec: %lu in dropcache%d ota%d mode%d drainheap: %d -> %d onrq%x\n",
					global_zone_page_state(NR_ION), rsc_system_max_ion_page, sc->priority, total,
					global_node_page_state(NR_FILE_PAGES), global_node_page_state(NR_ANON_MAPPED),
					global_node_page_state(NR_SLAB_RECLAIMABLE), global_node_page_state(NR_SLAB_UNRECLAIMABLE),
					!!task_in_drop_caches(current), !!task_in_drop_caches_ota(current), rsc_recovery_survival_mode,
					heap_free, drain_max_size, on_rq);
			}
		}
	}

	return 0;
}
#else
static unsigned long ion_heap_shrink_count(struct shrinker *shrinker,
					   struct shrink_control *sc)
{
	struct ion_heap *heap = container_of(shrinker, struct ion_heap,
					     shrinker);
	int total = 0;

	total = ion_heap_freelist_size(heap) / PAGE_SIZE;
	if (heap->ops->shrink)
		total += heap->ops->shrink(heap, sc->gfp_mask, 0);
	return total;
}
#endif


#ifdef CONFIG_RSC_ION_OPT
static unsigned long ion_heap_shrink_scan(struct shrinker *shrinker,
					  struct shrink_control *sc)
{
	struct ion_heap *heap = container_of(shrinker, struct ion_heap,
					     shrinker);
	int freed = 0;
	int to_scan = sc->nr_to_scan;

	if (to_scan == 0)
		return 0;

	/*prevent shrink too more page!*/
	if (!rsc_recovery_survival_mode && heap->rsc_reserve_ion_enable &&
		(global_zone_page_state(NR_ION) < rsc_system_max_ion_page) &&
		((global_node_page_state(NR_FILE_PAGES) > reclaim_file_cache_page) ||
		task_in_drop_caches(current))) {
		unsigned int reserve;
		int pool = ion_page_pool_nr_pages();

		if (task_in_drop_caches_ota(current))
			reserve = rsc_ion_pool[0] << (20 - PAGE_SHIFT);
		else
			reserve = rsc_ion_pool[RSC_ION_POOL_LEVEL - 1] << (20 - PAGE_SHIFT);

		if ((to_scan +  reserve) > pool) {
			int to_scan_old = to_scan;

			if (pool > reserve)
				to_scan = pool - reserve;
			else
				to_scan = 0;

			if (sc->priority <= 3)
				printk("[RSC] ion_heap_shrink_scan overload shrink!!! ion %lu < maxion %d priority: %d to_scan %d -> %d "
					"pool: %d reservepool: %d filecache: %lu anon: %lu srec: %lu sunrec: %lu in dropcache%d ota%d mode%d\n",
					global_zone_page_state(NR_ION), rsc_system_max_ion_page, sc->priority, to_scan_old, to_scan,
					ion_page_pool_nr_pages(), reserve, global_node_page_state(NR_FILE_PAGES), global_node_page_state(NR_ANON_MAPPED),
					global_node_page_state(NR_SLAB_RECLAIMABLE), global_node_page_state(NR_SLAB_UNRECLAIMABLE),
					!!task_in_drop_caches(current), !!task_in_drop_caches_ota(current), rsc_recovery_survival_mode);
			//show_free_areas(0, NULL);
			//dump_tasks(NULL, NULL);
			if (to_scan == 0) {
				return SHRINK_STOP;
			}
		}
	}


	/*
	* shrink ion pool only use heap->ops->shrink to keep high order page in ion pool.
	*/
	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE) {
		if (heap->rsc_reserve_ion_enable && !rsc_recovery_survival_mode) {
			//_ion_heap_freelist_drain(heap, to_scan * PAGE_SIZE, false);
			ion_heap_freelist_shrink(heap, to_scan * PAGE_SIZE) ;
		} else {
			freed = ion_heap_freelist_shrink(heap, to_scan * PAGE_SIZE) ;
			to_scan -= freed;
			if (to_scan <= 0) {
				return freed;
			}
		}
	}

	if (heap->ops->shrink) {
		heap->priority = sc->priority;
		freed += heap->ops->shrink(heap, sc->gfp_mask, to_scan);
	} else {
		printk("[RSC] %s %d ERROR! ion heap shrink not implementation!!", __func__, __LINE__);
		return SHRINK_STOP;
	}

	return freed;
}
#else
static unsigned long ion_heap_shrink_scan(struct shrinker *shrinker,
					  struct shrink_control *sc)
{
	struct ion_heap *heap = container_of(shrinker, struct ion_heap,
					     shrinker);
	int freed = 0;
	int to_scan = sc->nr_to_scan;

	if (to_scan == 0)
		return 0;

	/*
	 * shrink the free list first, no point in zeroing the memory if we're
	 * just going to reclaim it. Also, skip any possible page pooling.
	 */
	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		freed = ion_heap_freelist_shrink(heap, to_scan * PAGE_SIZE) /
				PAGE_SIZE;

	to_scan -= freed;
	if (to_scan <= 0)
		return freed;

	if (heap->ops->shrink)
		freed += heap->ops->shrink(heap, sc->gfp_mask, to_scan);
	return freed;
}
#endif

void ion_heap_init_shrinker(struct ion_heap *heap)
{
	heap->shrinker.count_objects = ion_heap_shrink_count;
	heap->shrinker.scan_objects = ion_heap_shrink_scan;
	heap->shrinker.seeks = DEFAULT_SEEKS;
	heap->shrinker.batch = 0;
	register_shrinker(&heap->shrinker);
}

struct ion_heap *ion_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_heap *heap = NULL;

	switch (heap_data->type) {
	case ION_HEAP_TYPE_SYSTEM_CONTIG:
		IONMSG("%s: Heap type is disabled: %d\n",
		       __func__, heap_data->type);
		return ERR_PTR(-EINVAL);
	case ION_HEAP_TYPE_SYSTEM:
		heap = ion_system_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_CARVEOUT:
		heap = ion_carveout_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_CHUNK:
		heap = ion_chunk_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_DMA:
		heap = ion_cma_heap_create(heap_data);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap_data->type);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR_OR_NULL(heap)) {
		pr_err("%s: error creating heap %s type %d base %lu size %zu\n",
		       __func__, heap_data->name, heap_data->type,
		       heap_data->base, heap_data->size);
		return ERR_PTR(-EINVAL);
	}

	heap->name = heap_data->name;
	heap->id = heap_data->id;
	return heap;
}
EXPORT_SYMBOL(ion_heap_create);

void ion_heap_destroy(struct ion_heap *heap)
{
	if (!heap)
		return;

	switch (heap->type) {
	case ION_HEAP_TYPE_SYSTEM_CONTIG:
		IONMSG("%s: Heap type is disabled: %d\n",
		       __func__, heap->type);
		break;
	case ION_HEAP_TYPE_SYSTEM:
		ion_system_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_CARVEOUT:
		ion_carveout_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_CHUNK:
		ion_chunk_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_DMA:
		ion_cma_heap_destroy(heap);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap->type);
	}
}
EXPORT_SYMBOL(ion_heap_destroy);
