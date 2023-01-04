ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY), y)

# Use the current MSM_ARCH as the target config/ Makefile
# Since Kernel SI can support multiple ARCH's this allows only the current selected target ARCH
# to compile.

ifneq (,$(filter AI2201,$(ASUS_BUILD_PROJECT)))

$(warning build camera dtbo for AI2201 8450/8475 ...)
include $(CAMERA_DEVICETREE_ROOT)/config/AI2201.mk

else ifneq (,$(filter AI2202,$(ASUS_BUILD_PROJECT)))

$(warning build camera dtbo for AI2202...)
include $(CAMERA_DEVICETREE_ROOT)/config/AI2202.mk

else

include $(CAMERA_DEVICETREE_ROOT)/config/$(MSM_ARCH).mk
endif

else
$(error CONFIG_BUILD_ARM64_DT_OVERLAY is: $(CONFIG_BUILD_ARM64_DT_OVERLAY))
endif

always-y	:= $(dtbo-y) $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
