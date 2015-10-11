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
 * sequence of actions can frequently (up to 5 times a day in New York City)
 * cause the SD card's ext4 partition to encounter errors upon failure to
 * suspend. These errors cause the ext4 partition to remount read-only because
 * it is mounted with the mount option errors=remount-ro .
 *
 * The reason for failure to suspend could be anything. The most troublesome
 * one is the cellular radio's baseband processor. When in a celluar dense
 * enviornment like New York City, the baseband processor will interrupt the
 * application CPU very often. This causes suspend attempts to frequently fail.
 * Other causes for suspend failure include Android alarms (set by applications
 * so that they can perform periodic work).
 *
 * The stock android suspend-retry mechanism in the kernel will try to suspend
 * 10 times following a failure to suspend, and then it will back-off for 10
 * seconds. This driver modifies the variables used for tracking the number
 * of tries and allows no retries before backing off. It also allows one retry.
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

#include "symsearch/symsearch.h"

// If true, allow only one suspend attempt. Otherwise, alow two.
static bool allow_one_suspend_attempt = false;
module_param(allow_one_suspend_attempt, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(allow_one_suspend_attempt, "If true, only one device suspend "
	"attempt is allowed before backing off. Otherwise, allow two.");

/* kernel/power/suspend.c */
/* This function is not exported so we have to symsearch it */
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, suspend_set_ops_s, const struct platform_suspend_ops *ops);

/* kernel/power/wakelock.c */
static unsigned *suspend_short_count_p = NULL;

/* arch/arm/mach-omap2/pm44xx.c */
static const struct platform_suspend_ops *omap_pm_ops_p = NULL;

static struct platform_suspend_ops omap_pm_mod_ops;

static void suspend_ratelimiter_suspend_recover(void) {
	if (allow_one_suspend_attempt) {
		/* Increment suspend_short_count by 9 so that
		 * wakelock.c's suspend() will backoff after 1 attempt to
		 * suspend, which failed during device suspend. Note that if an
		 * early suspend abort occurs (probably because a wakelock is
		 * held), we still will try to do actual suspend after that, and
		 * then we will run this code */
		/* This option appears to be almost 100% stable, but battery
		 * life is worse (The phone probably spends at least 50% less
		 * time suspended) */
		*suspend_short_count_p = 9;
	} else {
		/* Increment suspend_short_count by 8 so that wakelock.c's
		 * suspend() will backoff after 2 attempts to suspend, which
		 * might be caused by failure during device suspend, early
		 * suspend abort caused by wakelock detect. */
		/* This option is not quite 100% stable (every few days the SD
		 * card will have a detection error) */
		*suspend_short_count_p += 8;
		if (*suspend_short_count_p > 9) {
			*suspend_short_count_p = 9;
		}
	}
}

static int __init suspend_ratelimiter_init(void) {
	SYMSEARCH_BIND_POINTER_TO(suspend_ratelimiter, struct platform_suspend_ops*, omap_pm_ops, omap_pm_ops_p);
	SYMSEARCH_BIND_POINTER_TO(suspend_ratelimiter, unsigned*, suspend_short_count, suspend_short_count_p);

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
}

module_init(suspend_ratelimiter_init);
module_exit(suspend_ratelimiter_exit);

MODULE_DESCRIPTION("LG U2 Suspend Rate-Limiter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Lam (lambchop468 (a) gmail.com)");
