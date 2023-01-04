dtbo-$(CONFIG_ARCH_CAPE) += cape-mtp-pm8010-overlay.dtbo
dtbo-$(CONFIG_ARCH_CAPE) += AI2202-8475-PR1-overlay.dtbo
dtbo-$(CONFIG_ARCH_CAPE) += AI2202-8475-PR2-overlay.dtbo
dtbo-$(CONFIG_ARCH_CAPE) += AI2202-8475-PR3-overlay.dtbo

cape-mtp-pm8010-overlay.dtbo-base := cape.dtb cape-v2.dtb capep.dtb
AI2202-8475-PR1-overlay.dtbo-base := cape.dtb cape-v2.dtb capep.dtb
AI2202-8475-PR2-overlay.dtbo-base := cape.dtb cape-v2.dtb capep.dtb
AI2202-8475-PR3-overlay.dtbo-base := cape.dtb cape-v2.dtb capep.dtb

dts-dirs += audio camera display mmrm video

always-y	:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
