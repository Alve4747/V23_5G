#
# chenhan add for ois devices Makefile
#
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek
ifeq ($(CONFIG_MTK_CAM_PD2083F_EX), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/sub/common/ois/LC898129
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/sub/common/ois/inc
obj-y += LC898129/
endif

obj-y += ois_core_sub.o ois_util_sub.o
