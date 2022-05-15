/*
 * Samsung Exynos SoC series NPU driver
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *	http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _LRA_WAKELOCK_H_
#define _LRA_WAKELOCK_H_


static inline struct wakeup_source *lra_wake_lock_register(struct device *dev, const char *name)
{
	struct wakeup_source *ws = NULL;

	ws = wakeup_source_register(dev, name);
	if (ws == NULL) {
		pr_err("[haptic_wakelock] %s: wakelock register fail\n", name);
		return NULL;
	}

	return ws;
}

static inline void lra_wake_lock_unregister(struct wakeup_source *ws)
{
	if (ws == NULL) {
		pr_err("[haptic_wakelock] wakelock unregister fail\n");
		return;
	}

	wakeup_source_unregister(ws);
}

static inline void lra_wake_lock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		pr_err("[haptic_wakelock] wakelock fail\n");
		return;
	}

	__pm_stay_awake(ws);
}

static inline void lra_wake_lock_timeout(struct wakeup_source *ws, long timeout)
{
	if (ws == NULL) {
		pr_err("[haptic_wakelock] wakelock timeout fail\n");
		return;
	}

	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));
}

static inline void lra_wake_unlock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		pr_err("[haptic_wakelock] wake unlock fail\n");
		return;
	}

	__pm_relax(ws);
}

static inline int lra_wake_lock_active(struct wakeup_source *ws)
{
	if (ws == NULL) {
		pr_err("[haptic_wakelock] wake unlock fail\n");
		return 0;
	}

	return ws->active;
}

#endif
