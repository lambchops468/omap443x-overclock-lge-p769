#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_ARCH := arm
KERNEL_CROSS_COMPILE := arm-none-linux-gnueabi-

KERNEL_DIR := $(ANDROID_BUILD_TOP)/kernel
KERNEL_OUT_FROM_TOP := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_OUT := $(ANDROID_BUILD_TOP)/$(KERNEL_OUT_FROM_TOP)
KERNEL_CONFIG := $(KERNEL_OUT)/.config
KERNEL_DEFCONFIG_FILE := $(KERNEL_DIR)/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG)
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

# VPNClient module
# KERNEL_VPNCLIENT_MODULE := $(KERNEL_OUT)/drivers/misc/vpnclient/vpnclient.ko
#define copy_vpnclient_module
#	mkdir -p $(KERNEL_MODULES_OUT)
#	-cp -f $(KERNEL_VPNCLIENT_MODULE) $(KERNEL_MODULES_OUT)
#endef

# hdcp module kibum.lee@lge.com
#KERNEL_HDCP_MODULE := $(KERNEL_OUT)/drivers/p940/mhl/hdcp/hdcp.ko
#define copy_hdcp_module
#	mkdir -p $(KERNEL_MODULES_OUT)
#	-cp -f $(KERNEL_HDCP_MODULE) $(KERNEL_MODULES_OUT)
#endef

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image
else
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/zImage
endif

KERNEL_MAKECMD := +$(MAKE) -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE)

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_DEFCONFIG_FILE) $(KERNEL_OUT)
	$(KERNEL_MAKECMD) $(KERNEL_DEFCONFIG)

$(TARGET_PREBUILT_KERNEL): $(KERNEL_CONFIG)
	$(KERNEL_MAKECMD)
#	$(copy_wifi_module)
#	$(copy_vpnclient_module)
#	$(copy_hdcp_module)

kernel: recoveryimage

ckernel:
	-@$(KERNEL_MAKECMD) distclean
	-@rm -vf $(INSTALLED_KERNEL_TARGET)
	-@rm -vf $(INSTALLED_BOOTIMAGE_TARGET)
	-@rm -vf $(INSTALLED_RECOVERYIMAGE_TARGET)

kerneltags:
	$(filter-out O=$(KERNEL_OUT), $(KERNEL_MAKECMD)) tags

# LGE_BSP 2012.03.13 [myeonggyu.son@lge.com] make kernelconfig - it makes kernel defconfig
kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) menuconfig
	cp $(KERNEL_OUT)/.config kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

.PHONY: $(TARGET_PREBUILT_KERNEL) kernel ckernel kerneltags kernelconfig

endif
