#
# Makefile for the input core drivers.
#
# Each configuration option enables a list of files
ccflags-y += -DANDROID_GKI_VFS_EXPORT_ONLY=VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver
ccflags-y += -I$(srctree)/drivers/sensors/snt8100fsr-linux/inc
#ccflags-y += -Werror
#ccflags-y += -Idrivers/sensors/snt8100fsr-linux/inc

#obj-$(CONFIG_SENSORS_SNT8155)	:= sensors_snt8155.o
obj-y := sensors_snt8155.o
sensors_snt8155-y	+= crc.o
sensors_snt8155-y	+= customize.o
sensors_snt8155-y	+= device.o
sensors_snt8155-y	+= event.o
sensors_snt8155-y	+= file.o
sensors_snt8155-y	+= firmware.o
sensors_snt8155-y	+= hardware.o
sensors_snt8155-y	+= input_device.o
sensors_snt8155-y	+= i2c_bus.o
sensors_snt8155-y	+= i2c_wake_bus.o
sensors_snt8155-y	+= irq.o
sensors_snt8155-y	+= main.o
sensors_snt8155-y	+= memory.o
sensors_snt8155-y	+= serial_bus.o
sensors_snt8155-y	+= spi_bus.o
sensors_snt8155-y	+= sysfs.o
sensors_snt8155-y	+= utils.o
sensors_snt8155-y	+= workqueue.o
sensors_snt8155-y	+= file_control.o
sensors_snt8155-y	+= asus_init.o
sensors_snt8155-y	+= proc_node.o
DEBUG = n

ifeq ($(DEBUG),y)
        DEBFLAGS = -O -g -DGRIP_DEBUG
else
        DEBFLAGS = -O -g
endif

ccflags-y += $(DEBFLAGS)
