#
# Makefile for MELFAS MIP4 Touchscreen driver (MediaTek)
#

ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/melfas_mip4/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

#Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/melfas_mip4/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


ifeq ($(CONFIG_MTK_PLATFORM),"mt6572")
    ccflags-y += -DMT6572
endif
ifeq ($(CONFIG_MTK_PLATFORM),"mt6582")
    ccflags-y += -DMT6582
endif

obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS438) += mip4.o mip4_mod.o mip4_debug.o mip4_fw_mms400.o

