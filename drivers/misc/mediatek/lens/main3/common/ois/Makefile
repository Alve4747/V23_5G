#
# Makefile for misc devices that really don't fit anywhere else.
#
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

ifeq ($(CONFIG_MTK_CAM_PD2135), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main3/common/ois/DW9786
obj-y += DW9786/
endif

obj-y += ois_core_tele.o ois_util_tele.o
