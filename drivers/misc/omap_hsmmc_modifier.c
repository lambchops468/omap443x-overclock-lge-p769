/*
 * OMAP HS-MMC modifier driver. 
 * Copyright (C) 2015 Alexander Lam (lambchop468 (at) gmail.com)
 *
 * Modifies properties of the omap_hsmmc devices.
 * This driver primarily exists to disable power-off of the external sdcard
 * slot.
 *
 * Why:
 * The sdcard slot seems to have issues with ext4.
 *
 * When waking up the phone, sometimes the ext4 partition will be mounted
 * read-only because of the mount option “errors=remount-ro”. This indicates
 * that something went wrong while suspending/resume the sdcard. (This shows
 * up when overclocking is not applied, so it is not overclock's fault). Adding
 * the mount option “commit=1” seems to help. The theory here is that commit=1
 * forces writes to be written out sooner (versus the ext4 default of 2
 * minutes), and that during system suspend, the mmc card isn’t trying to catch
 * up with all the writes that were deferred and now are being flushed. I guess
 * consumer SD cards these days fake when they're done with a command for
 * piplining performance, so during the suspend phase, the software thinks the
 * card is done writing to flash and shuts power off before the card's internal
 * processor has flushed all data.
 *
 * Also, see these two links - they say that leaving the power on should fix
 * this:
 * http://serverfault.com/questions/356507/safe-ext4-configuration-for-systems-running-unattended
 * http://electronics.stackexchange.com/questions/66913/has-anyone-seen-power-cycle-corruption-in-industrial-sd-cards
 *
 * This driver basically attempts to invoke line 2839 of
 * drivers/mmc/host/omap_hsmmc.c .
 *
 * The mmc slot setup of the machine is defined in mach-omap2/lge/u2/u2_mmc.c .
 * Each element in the omap2_hsmmc_info mmc[] array in
 * mach-omap2/lge/u2/u2_mmc.c gets copied by omap_hsmmc_pdata_init()
 * (in arch/arm/mach-omap2/hsmmc.c), which is called by omap_init_hsmmc().
 * (omap_init_hsmmc() also registers the omap hardware module representing
 * the mmc slotby calling omap_device_build(), which causes another copy when
 * platform_device_add_data() is called.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <plat/mmc.h>

#ifdef CONFIG_OMAP4_DPLL_CASCADING
#include <linux/notifier.h>
#endif

#include "symsearch/symsearch.h"

/* lib/kobject.c */
/* This function is not exported so we have to symsearch it */
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct kobject*, kset_find_obj_s, struct kset *kset, const char *name);

/* drivers/base/core.c */
static struct kobj_type *device_ktype_p = NULL;

/* drivers/mmc/host/omap_hsmmc.c */
/* Unfortunately the struct below is not in a header so we duplicate here */
struct omap_hsmmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*iclk;
	struct	clk		*dbclk;
#ifdef CONFIG_OMAP4_DPLL_CASCADING
	struct notifier_block	nb;
	int			dpll_entry;
	int			dpll_exit;
	spinlock_t		dpll_lock;
#endif
	/*
	 * vcc == configured supply
	 * vcc_aux == optional
	 *   -	MMC1, supply for DAT4..DAT7
	 *   -	MMC2/MMC2, external level shifter voltage supply, for
	 *	chip (SDIO, eMMC, etc) or transceiver (MMC2 only)
	 */
	struct	regulator	*vcc;
	struct	regulator	*vcc_aux;
	struct	work_struct	mmc_carddetect_work;
	void	__iomem		*base;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	unsigned int		id;
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned int		master_clock;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	u32			*buffer;
	u32			bytesleft;
	int			suspended;
	int			irq;
	int			dma_type, dma_ch;
	struct adma_desc_table	*adma_table;
	dma_addr_t		phy_adma_table;
	int			dma_line_tx, dma_line_rx;
	int			slot_id;
	int			got_dbclk;
	int			response_busy;
	int			dpm_state;
	int			vdd;
	int			protect_card;
	int			reqs_blocked;
	int			use_reg;
	int			req_in_progress;
	unsigned int		flags;
	unsigned int		errata;

	unsigned int		eject; /* eject state */

	struct	omap_mmc_platform_data	*pdata;
};
#define mmc_slot(host)		(host->pdata->slots[host->slot_id])

#define to_dev(obj) container_of(obj, struct device, kobj)
#define to_mmc_host(obj) container_of(obj, struct mmc_host, class_dev)

/* When the omap_hsmmc driver sets up the mmc platform device, it creates a
 * subdevice called "mmc1" */
#define MMC_PLATFORM_DEV_NAME "omap_hsmmc.0"
#define MMC_HOST_DEV_NAME "mmc1"

struct device *mmc_dev;
struct device *mmc_host_dev;

static int match_mmc_host(struct device *dev, void *unused) {
	return strcmp(dev_name(dev), MMC_HOST_DEV_NAME) == 0;
}

static int __init omap_hsmmc_modifier_init(void) {
	struct device dev;
	struct kset *device_kset = NULL;
	struct kobject *mmc_kobj = NULL;
	struct platform_device *mmc_pdev = NULL;
	struct omap_hsmmc_host *host = NULL;

	SYMSEARCH_BIND_FUNCTION_TO(omap_hsmmc_modifier, kset_find_obj, kset_find_obj_s);
	SYMSEARCH_BIND_POINTER_TO(omap_hsmmc_modifier, struct kobj_type*, device_ktype, device_ktype_p);

	device_initialize(&dev);
	device_kset = dev.kobj.kset;
	/* This gets the mmc device */
	mmc_kobj = kset_find_obj_s(device_kset, MMC_PLATFORM_DEV_NAME);

	if (mmc_kobj == NULL) {
		pr_err("omap_hsmmc_modifier init failed: Couldn't find "
				MMC_PLATFORM_DEV_NAME ".\n");
		return ENODEV;
	}

	if (get_ktype(mmc_kobj) != device_ktype_p) {
		pr_err("omap_hsmmc_modifier init failed: mmc_kobj is a "
				"device.\n");
		goto wrong_dev;
	}

	mmc_dev = to_dev(mmc_kobj);

	/* Verify the omap_hsmmc driver loaded */
	mmc_host_dev = device_find_child(mmc_dev, NULL, match_mmc_host);
	if (mmc_host_dev == NULL) {
		pr_err("omap_hsmmc_modifier init failed: Couldn't find "
				MMC_HOST_DEV_NAME ".\n");
		goto wrong_dev;
		return ENODEV;
	}

	mmc_pdev = to_platform_device(mmc_dev);
	host = platform_get_drvdata(mmc_pdev);

	/* Make the hell sure this isn't the eMMC chip. */
	if (mmc_slot(host).nonremovable == false &&
	    mmc_slot(host).power_saving == true) {
		mmc_slot(host).mmc_data.built_in = 1;
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
	} else {
		pr_err("omap_hsmmc_modifier failed: Found wrong slot!\n");
		goto wrong_slot;
	}
	//struct mmc_host *host = to_mmc_host(mmc_host_dev);

	return 0;

wrong_slot:
	put_device(mmc_host_dev);
wrong_dev:
	put_device(mmc_dev);
	return ENODEV;
}

static void __exit omap_hsmmc_modifier_exit(void) {
	put_device(mmc_host_dev);
	put_device(mmc_dev);
}

module_init(omap_hsmmc_modifier_init);
module_exit(omap_hsmmc_modifier_exit);

MODULE_DESCRIPTION("OMAP443X HS-MMC device modifier");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Lam (lambchop468 (a) gmail.com)");
