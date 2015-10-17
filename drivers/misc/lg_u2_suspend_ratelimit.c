/*
 * LG U2 Suspend Rate Limting Driver
 * Copyright (C) 2015 Alexander Lam (lambchop468 (at) gmail.com)
 *
 * The LG U2 board design is an OMAP4430 based smartphone platform. It has an
 * issue with SD cards where the user-accessible SD card slot cannot reliably
 * be used with ext4 partitions.
 *
 * When Android performs a suspend, it will try up to 10 times in quick
 * succession when the previous attempt to suspend fails. It seems that this
 * sequence of actions can frequently (up to 5 times a day with cellular data)
 * cause the SD card's ext4 partition to encounter errors upon failure to
 * suspend. These errors cause the ext4 partition to remount read-only because
 * it is mounted with the mount option errors=remount-ro .
 *
 * The reason for failure to suspend could be anything. The most troublesome
 * one is the cellular fast dormancy feature. The LG Jellybean implementation of
 * fast dormancy uses Android Alarms to wake up and check for network traffic.
 * When there's no network traffic fast dormancy is entered. The alarms cause
 * suspend attempts to frequently fail because the alarm driver will prevent
 * suspending when an alarm will go off in the near future.
 *
 * The stock android suspend-retry mechanism in the kernel will try to suspend
 * 10 times following a failure to suspend, and then it will back-off for 10
 * seconds. This driver has two modes:
 * - Upon failure to suspend, manipulate the suspend retry counter so only
 *   a limited number (as low as none) retries are allowed. This relies on the 
 *   stock back-off mechanism, the suspend_backoff wakelock.
 * - Upon failure to suspend, lock a wakelock defined by this driver. This lock
 *   period can be set with the kernel parameter. This allows us to set a
 *   shorter backoff period lower than 10 seconds so we can attempt to save
 *   more power. The stock backoff mechanism is unaffected so 10 consecutive
 *   short suspends will invoke the 10 second backoff.
 *
 * omap_hsmmc_modifier.c was a pervious attempt to solve this problem by
 * keeping the sd card powered up during suspend. It didn't work. It seems like
 * power was being shut off anyway.
 *
 * This approach actually works and if no suspend retries are allowed, the ext4
 * partition can be stable for a very long time (weeks) before encountering an
 * error, which can be resolved by rebooting.
 */

#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>

#include "symsearch/symsearch.h"

// Number of milliseconds to backoff using the custom wakelock.
static int custom_backoff_msec = 2500;
module_param(custom_backoff_msec, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(custom_backoff_msec, "If set >zero, we'll only allow "
        "one suspend attempt and backoff this many milliseconds. If set to "
        "<= 0 and > -10, then this is the number of suspend retries to allow "
        "before invoking the stock 10-second backoff timer.");

/* kernel/power/suspend.c */
/* This function is not exported so we have to symsearch it */
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, suspend_set_ops_s, const struct platform_suspend_ops *ops);

/* kernel/power/wakelock.c */
static unsigned *suspend_short_count_p = NULL;

/* arch/arm/mach-omap2/pm44xx.c */
static const struct platform_suspend_ops *omap_pm_ops_p = NULL;

static struct platform_suspend_ops omap_pm_mod_ops;
static struct wake_lock suspend_ratelimit_lock;

static void suspend_ratelimiter_suspend_recover(void) {
	if (custom_backoff_msec > 0) {
	        pr_info("suspend_ratelimit: wakelocking\n");
		wake_lock_timeout(&suspend_ratelimit_lock,
				  msecs_to_jiffies(custom_backoff_msec));
	} else {
		/* Note: Allowing one suspend attempt and no retries
		 * (i.e. custom_backoff_msec = 0) seems to be almost 100%
		 * stable, but battery life is a bit worse. (The phone probably
		 * spends at least 50% less time suspended).
		 * Allowing two suspend attemps (one retry) is less stable. The
		 * SD card will encounter an error every few days.
		 */
		int allowed_suspend_retries = -custom_backoff_msec;
		/* If the suspend_short_count counts the number of retries
		 * already performed. If it indicates we are out of retries,
		 * Then set suspend_short_count to 9 to force a backoff.
		 * Note that suspend_short_count can be incremented without
		 * invoking this function if suspend fails early due to wakelock
		 * activity. */
		if (*suspend_short_count_p >= allowed_suspend_retries) {
	                pr_info("suspend_ratelimit: clamp suspend retries\n");
			/* Set to 9 so that wakelock.c's suspend() will backoff
			 * right away. */
			*suspend_short_count_p = 9;
		}
	}
}

static int __init suspend_ratelimiter_init(void) {
	SYMSEARCH_BIND_FUNCTION_TO(omap_hsmmc_modifier, suspend_set_ops, suspend_set_ops_s);
	SYMSEARCH_BIND_POINTER_TO(suspend_ratelimiter, struct platform_suspend_ops*, omap_pm_ops, omap_pm_ops_p);
	SYMSEARCH_BIND_POINTER_TO(suspend_ratelimiter, unsigned*, suspend_short_count, suspend_short_count_p);

        wake_lock_init(&suspend_ratelimit_lock, WAKE_LOCK_SUSPEND,
                       "suspend_ratelimit");

	omap_pm_mod_ops.begin	= omap_pm_ops_p->begin;
	omap_pm_mod_ops.end	= omap_pm_ops_p->end;
	omap_pm_mod_ops.prepare	= omap_pm_ops_p->prepare;
	omap_pm_mod_ops.enter	= omap_pm_ops_p->enter;
	omap_pm_mod_ops.valid	= omap_pm_ops_p->valid;
	omap_pm_mod_ops.recover = suspend_ratelimiter_suspend_recover;
	suspend_set_ops_s(&omap_pm_mod_ops);

	pr_info("suspend_ratelimiter: successfully hijacked platform pm ops\n");

	return 0;
}

static void __exit suspend_ratelimiter_exit(void) {
	suspend_set_ops_s(omap_pm_ops_p);
        wake_lock_destroy(&suspend_ratelimit_lock);
}

module_init(suspend_ratelimiter_init);
module_exit(suspend_ratelimiter_exit);

MODULE_DESCRIPTION("LG U2 Suspend Rate-Limiter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Lam (lambchop468 (a) gmail.com)");
