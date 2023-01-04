# ASUS_BSP : add for asus audio dtbo +++
ifneq (,$(filter AI2201,$(ASUS_BUILD_PROJECT)))
$(warning build audio dtbo for AI2201...)

dtbo-$(CONFIG_ARCH_WAIPIO) += waipio-audio.dtbo \
                 AI2201-8450-EVB-audio-overlay.dtbo \
                 AI2201-8450-PreSR-audio-overlay.dtbo \
                 AI2201-8450-SR1-audio-overlay.dtbo \
                 AI2201-8450-SR2-audio-overlay.dtbo \
                 AI2201-8450-ER1-audio-overlay.dtbo \
                 AI2201-8450-ER2-audio-overlay.dtbo \
                 AI2201-8450-PR-audio-overlay.dtbo \
                 AI2201-8450-MP-audio-overlay.dtbo

dtbo-$(CONFIG_ARCH_DIWALI) += diwali-audio.dtbo

dtbo-$(CONFIG_ARCH_CAPE) += cape-audio.dtbo \
                 AI2201-8475-EVB_ER1-audio-overlay.dtbo \
                 AI2201-8475-ER2-audio-overlay.dtbo \
                 AI2201-8475-PR-audio-overlay.dtbo \
                 AI2201-8475-PR2-audio-overlay.dtbo \
                 AI2201-8475-PR2_2-audio-overlay.dtbo \
                 AI2201-8475-MP-audio-overlay.dtbo

# ASUS_BSP : add for asus audio dtbo ---

# ASUS_BSP : add for asus AI2202 audio dtbo +++
else ifneq (,$(filter AI2202,$(ASUS_BUILD_PROJECT)))
$(warning build audio dtbo for AI2202...)

dtbo-$(CONFIG_ARCH_WAIPIO) += waipio-audio.dtbo \
                 AI2202-EVB-audio-overlay.dtbo \
                 AI2202-SR-audio-overlay.dtbo \
                 AI2202-SR2-audio-overlay.dtbo \
                 AI2202-ER-audio-overlay.dtbo \
                 AI2202-ER2-2-audio-overlay.dtbo

dtbo-$(CONFIG_ARCH_DIWALI) += diwali-audio.dtbo \
                 diwali-audio-idp.dtbo \
                 diwali-audio-idp-amoled.dtbo \
                 diwali-audio-qrd.dtbo \
                 diwali-audio-atp.dtbo \
                 diwali-audio-idp-hsp.dtbo \
                 diwali-audio-idp-usbc.dtbo

dtbo-$(CONFIG_ARCH_CAPE) += cape-audio.dtbo \
                 AI2202-8475-EVB-audio-overlay.dtbo \
                 AI2202-8475-ER2-audio-overlay.dtbo \
                 AI2202-8475-ER2-2-audio-overlay.dtbo \
                 AI2202-8475-PR1-audio-overlay.dtbo \
                 AI2202-8475-PR2-audio-overlay.dtbo \
                 AI2202-8475-PR3-audio-overlay.dtbo
# ASUS_BSP : add for asus AI2202 audio dtbo ---
else
dtbo-$(CONFIG_ARCH_WAIPIO) += waipio-audio.dtbo \
                 waipio-audio-cdp.dtbo \
                 waipio-audio-mtp.dtbo \
                 waipio-audio-qrd.dtbo \
                 waipio-audio-atp.dtbo \
                 waipio-audio-rumi.dtbo \
                 waipio-audio-hdk.dtbo

dtbo-$(CONFIG_ARCH_DIWALI) += diwali-audio.dtbo \
                 diwali-audio-idp.dtbo \
                 diwali-audio-idp-amoled.dtbo \
                 diwali-audio-qrd.dtbo \
                 diwali-audio-atp.dtbo \
                 diwali-audio-idp-hsp.dtbo \
                 diwali-audio-idp-usbc.dtbo

dtbo-$(CONFIG_ARCH_CAPE) += cape-audio.dtbo \
                 cape-audio-cdp.dtbo \
                 cape-audio-cdp-qhd.dtbo \
                 cape-audio-mtp.dtbo \
                 cape-audio-mtp-120fps.dtbo \
                 cape-audio-mtp-nodisplay.dtbo \
                 cape-audio-atp.dtbo \
                 cape-audio-qrd.dtbo
endif

 always-y    := $(dtb-y) $(dtbo-y)
 subdir-y    := $(dts-dirs)
 clean-files    := *.dtb *.dtbo
