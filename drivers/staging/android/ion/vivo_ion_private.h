/*
 * mm/mem_big_data.h
 *
 * VIVO Kernel Monitor Engine(Memory)
 *
 * Copyright (C) 2020 VIVO Technology Co., Ltd
 * <rongqianfeng@vivo.com>
*/

#ifndef __VIVO_ION_PRIVATE_H
#define __VIVO_ION_PRIVATE_H

#include <linux/vivo_rsc/rsc_internal.h>

#ifdef CONFIG_RSC_ION_OPT
#define RSC_ION_POOL_LEVEL 4
extern unsigned int rsc_ion_pool[RSC_ION_POOL_LEVEL];
extern unsigned int ion_priority[RSC_ION_POOL_LEVEL];
extern int reclaim_file_cache_page;

/*2048MB*/
#define RSC_SYSTEM_MAX_ION_POOL_PAGE ((2048+400) << (20 - PAGE_SHIFT))
extern atomic_t rsc_ion_pool_pages;
#define RSC_MEM_ION_RECLAIM_FILECACHE_PAGE (512 << (20 - PAGE_SHIFT))

extern int __read_mostly max_page_pool_size;
extern int __read_mostly rsc_system_max_ion_page;

#define RSC_RESERVE_ION_DISABLE 0
#define RSC_RESERVE_ION_WHEN_RAMSIZE_BIGGER_SEVEN_GB 1
#define RSC_RESERVE_ION_FORCE 2
#endif

#endif