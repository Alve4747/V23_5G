/*
 * drivers/staging/android/ion/ion_mem_pool.c
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/sched/clock.h>
#include "ion_priv.h"

static unsigned long long last_alloc_ts;

/*
 * We avoid atomic_long_t to minimize cache flushes at the cost of possible
 * race which would result in a small accounting inaccuracy that we can
 * tolerate.
 */
static long nr_total_pages;

static void *ion_page_pool_alloc_pages(struct ion_page_pool *pool)
{
	unsigned long long start, end;
	struct page *page;
	unsigned int i;

	start = sched_clock();
	page = alloc_pages(pool->gfp_mask, pool->order);
	end = sched_clock();

	if ((end - start > 10000000ULL) &&
	    (end - last_alloc_ts > 1000000000ULL)) { /* unit is ns, 1s */
		IONMSG("warn: alloc pages order: %d time: %lld ns\n",
		       pool->order, end - start);
		show_free_areas(0, NULL);
		last_alloc_ts = end;
	}

	if (!page)
		return NULL;
	ion_pages_sync_for_device(g_ion_device->dev.this_device,
				  page, PAGE_SIZE << pool->order,
				  DMA_BIDIRECTIONAL);
	atomic64_add_return((1 << pool->order), &page_sz_cnt);
	for (i = 0; i < (1 << pool->order); i++)
		SetPageIommu(&page[i]);
	return page;
}

static void ion_page_pool_free_pages(struct ion_page_pool *pool,
				     struct page *page)
{
	__free_pages(page, pool->order);
	if (atomic64_sub_return((1 << pool->order), &page_sz_cnt) < 0) {
		IONMSG("underflow!, total_now[%ld]free[%lu]\n",
		       atomic64_read(&page_sz_cnt),
		       (unsigned long)(1 << pool->order));
		atomic64_set(&page_sz_cnt, 0);
	}
}

static int ion_page_pool_add(struct ion_page_pool *pool, struct page *page)
{
	mutex_lock(&pool->mutex);
	if (PageHighMem(page)) {
		list_add_tail(&page->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&page->lru, &pool->low_items);
		pool->low_count++;
	}

#ifdef CONFIG_RSC_ION_REFILL
	atomic_inc(&pool->count);
#endif


	nr_total_pages += 1 << pool->order;
	mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
							1 << pool->order);
	mutex_unlock(&pool->mutex);
	return 0;
}

#ifdef CONFIG_RSC_ION_OPT
void rsc_ion_page_pool_add(struct ion_page_pool *pool, struct page *page)
{
	ion_page_pool_add(pool, page);
}
#endif

static struct page *ion_page_pool_remove(struct ion_page_pool *pool, bool high)
{
	struct page *page;

	if (high) {
		BUG_ON(!pool->high_count);
		page = list_first_entry(&pool->high_items, struct page, lru);
		pool->high_count--;
	} else {
		BUG_ON(!pool->low_count);
		page = list_first_entry(&pool->low_items, struct page, lru);
		pool->low_count--;
	}

	list_del(&page->lru);
#ifdef CONFIG_RSC_ION_REFILL
	atomic_dec(&pool->count);
#endif


	nr_total_pages -= 1 << pool->order;
	mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
							-(1 << pool->order));
	return page;
}

#ifdef CONFIG_RSC_ION_OPT
struct page *rsc_ion_page_pool_remove(struct ion_page_pool *pool, bool high)
{
	return ion_page_pool_remove(pool, high);
}
#endif

#ifdef CONFIG_RSC_ION_PAGE_POOL_FULLBACK
#define ALTEL_POOL_PAGES_MIN_LIMIT (60 << (20 - PAGE_SHIFT))
struct page *ion_page_pool_alloc_gather(struct ion_page_pool *pool,
					struct ion_page_pool *backend_pool, bool *from_backend)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	mutex_lock(&pool->mutex);
	if (pool->high_count)
		page = ion_page_pool_remove(pool, true);
	else if (pool->low_count)
		page = ion_page_pool_remove(pool, false);
	mutex_unlock(&pool->mutex);

	if (!page) {
		unsigned long altel_pool_free = 0;

		altel_pool_free = backend_pool->high_count + backend_pool->low_count;

		if ((altel_pool_free > ALTEL_POOL_PAGES_MIN_LIMIT) &&
				mutex_trylock(&backend_pool->mutex)) {
			if (backend_pool->high_count)
				page = ion_page_pool_remove(backend_pool, true);
			else if (backend_pool->low_count)
				page = ion_page_pool_remove(backend_pool, false);
			mutex_unlock(&backend_pool->mutex);

			if (page)
				*from_backend = true;
		}
	}

	if (!page)
		page = ion_page_pool_alloc_pages(pool);

	return page;
}
#endif

struct page *ion_page_pool_alloc(struct ion_page_pool *pool)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	mutex_lock(&pool->mutex);
	if (pool->high_count)
		page = ion_page_pool_remove(pool, true);
	else if (pool->low_count)
		page = ion_page_pool_remove(pool, false);
	mutex_unlock(&pool->mutex);

	if (!page)
		page = ion_page_pool_alloc_pages(pool);

	return page;
}

void ion_page_pool_free(struct ion_page_pool *pool, struct page *page)
{
	int ret;

	if (pool->order != compound_order(page))
		IONMSG("free page = 0x%p, compound_order(page) = 0x%x",
		       page, compound_order(page));

	BUG_ON(pool->order != compound_order(page));

	ret = ion_page_pool_add(pool, page);
	if (ret)
		ion_page_pool_free_pages(pool, page);
}

static int ion_page_pool_total(struct ion_page_pool *pool, bool high)
{
	int count = pool->low_count;

	if (high)
		count += pool->high_count;

	return count << pool->order;
}

long ion_page_pool_nr_pages(void)
{
	/* Correct possible overflow caused by racing writes */
	if (nr_total_pages < 0)
		nr_total_pages = 0;
	return nr_total_pages;
}

int ion_page_pool_shrink(struct ion_page_pool *pool, gfp_t gfp_mask,
			 int nr_to_scan)
{
	int freed = 0;
	bool high;

	if (current_is_kswapd())
		high = true;
	else
		high = !!(gfp_mask & __GFP_HIGHMEM);

	if (nr_to_scan == 0)
		return ion_page_pool_total(pool, high);

	while (freed < nr_to_scan) {
		struct page *page;

		mutex_lock(&pool->mutex);
		if (pool->low_count) {
			page = ion_page_pool_remove(pool, false);
		} else if (high && pool->high_count) {
			page = ion_page_pool_remove(pool, true);
		} else {
			mutex_unlock(&pool->mutex);
			break;
		}
		mutex_unlock(&pool->mutex);
		ion_page_pool_free_pages(pool, page);
		freed += (1 << pool->order);
	}

	return freed;
}

#ifdef CONFIG_RSC_ION_REFILL
/* do a simple check to see if we are in any low memory situation */
static bool pool_refill_ok(struct ion_page_pool *pool)
{
	struct zonelist *zonelist;
	struct zoneref *z;
	struct zone *zone;
	int mark;
	enum zone_type classzone_idx = gfp_zone(pool->gfp_mask);
	s64 delta;

	/* check if we are within the refill defer window */
	delta = ktime_ms_delta(ktime_get(), pool->last_low_watermark_ktime);
	if (delta < ION_POOL_REFILL_DEFER_WINDOW_MS)
		return false;

	zonelist = node_zonelist(numa_node_id(), pool->gfp_mask);
	/*
	 * make sure that if we allocate a pool->order page from buddy,
	 * we don't put the zone watermarks go below the high threshold.
	 * This makes sure there's no unwanted repetitive refilling and
	 * reclaiming of buddy pages on the pool.
	 */
	for_each_zone_zonelist(zone, z, zonelist, classzone_idx) {
		mark = high_wmark_pages(zone);
		mark += 1 << pool->order;
		if (!zone_watermark_ok_safe(zone, pool->order, mark,
					    classzone_idx)) {
			pool->last_low_watermark_ktime = ktime_get();
			return false;
		}
	}

	return true;
}

void ion_page_pool_refill(struct ion_page_pool *pool)
{
	struct page *page;
	gfp_t gfp_refill = (pool->gfp_mask | __GFP_RECLAIM) & ~__GFP_NORETRY;
	int i;

	/* skip refilling order 0 pools */
	if (!pool->order)
		return;

	while (!pool_fillmark_reached(pool) && pool_refill_ok(pool)) {
		page = alloc_pages(gfp_refill, pool->order);

#ifdef CONFIG_RSC_MEM_STAT
	if (page)
		mod_zone_page_state(page_zone(page), NR_ION, (1 << pool->order));
#endif
		if (!page)
			break;

		ion_pages_sync_for_device(g_ion_device->dev.this_device, page,
						PAGE_SIZE << pool->order,
						DMA_BIDIRECTIONAL);
		atomic64_add_return((1 << pool->order), &page_sz_cnt);
		for (i = 0; i < (1 << pool->order); i++)
			SetPageIommu(&page[i]);
		ion_page_pool_add(pool, page);
	}
}
#endif

struct ion_page_pool *ion_page_pool_create(gfp_t gfp_mask, unsigned int order,
					   bool cached)
{
	struct ion_page_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool) {
		IONMSG("%s kmalloc failed pool is null.\n", __func__);
		return NULL;
	}
	pool->high_count = 0;
	pool->low_count = 0;
#ifdef CONFIG_RSC_ION_REFILL
	atomic_set(&pool->count, 0);
#endif
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);
	plist_node_init(&pool->list, order);
	if (cached)
		pool->cached = true;

	return pool;
}

void ion_page_pool_destroy(struct ion_page_pool *pool)
{
	kfree(pool);
}

static int __init ion_page_pool_init(void)
{
	return 0;
}
device_initcall(ion_page_pool_init);
