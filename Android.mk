LOCAL_PATH:= $(call my-dir)
# Android standard make file for kernel modules

FROM_LOCAL_TO_TOP := ..

define build_prebuilt_kernel_module
    # Standard Android prebuilt module
    include $$(CLEAR_VARS)
    LOCAL_MODULE := $(notdir $(strip $(1)))
    LOCAL_MODULE_TAGS := optional
    LOCAL_MODULE_PATH := $$(TARGET_OUT)/lib/modules
    LOCAL_SRC_FILES := $$(FROM_LOCAL_TO_TOP)/$(strip $(1))
    LOCAL_MODULE_CLASS := ETC
    LOCAL_REQUIRED_MODULE := $$(LOCAL_PATH)/$$(LOCAL_SRC_FILES)
    include $$(BUILD_PREBUILT)

# Complete the dependency chain
.PHONY: $$(LOCAL_REQUIRED_MODULE)
$$(LOCAL_REQUIRED_MODULE): $$(TARGET_PREBUILT_KERNEL)
endef

#LGE_CHANGE_S, moon-wifi@lge.com by wo0ngs 2012-02-14, ICS not use module
################################################################################
## Copy wireless.ko to system.img
##	Required: PRODUCT_PACKAGES += wireless.ko
ifneq ($(filter bcm4330 bcm4330_b2 bcmdhd,$(BOARD_WLAN_DEVICE)),)
#$(eval $(call build_prebuilt_kernel_module, \
#    $(KERNEL_OUT_FROM_TOP)/drivers/net/wireless/$(BOARD_WLAN_DEVICE)/wireless.ko ))
$(eval $(call build_prebuilt_kernel_module, \
     $(KERNEL_OUT_FROM_TOP)/drivers/net/wireless/$(BOARD_WLAN_DEVICE)/bcmdhd.ko ))
endif
#LGE_CHANGE_E, moon-wifi@lge.com by wo0ngs 2012-02-14, ICS not use module

###############################################################################
# Copy vpnclient.ko to system.img
#	Required: PRODUCT_PACKAGES += vpnclient.ko
#$(eval $(call build_prebuilt_kernel_module, \
#    $(KERNEL_OUT_FROM_TOP)/drivers/misc/vpnclient/vpnclient.ko ))

###############################################################################
# Copy hdcp.ko to system.img
#	Required: PRODUCT_PACKAGES += hdcp.ko
#$(eval $(call build_prebuilt_kernel_module, \
#    $(KERNEL_OUT_FROM_TOP)/drivers/p940/mhl/hdcp/hdcp.ko ))

