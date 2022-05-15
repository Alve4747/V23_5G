#
# chenhan add for ois devices Makefile
#
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

ifeq ($(CONFIG_MTK_CAM_PD2072), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/ois/DW9781C
obj-y += DW9781C/
endif

ifeq ($(CONFIG_MTK_CAM_PD2083F_EX), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/ois/DW9781C
obj-y += DW9781C/
endif

ifneq ($(findstring $(CONFIG_MTK_CAM_PD2120A)$(CONFIG_MTK_CAM_PD2085)$(CONFIG_MTK_CAM_PD2133)$(CONFIG_MTK_CAM_PD2135), y y y y y),)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/ois/RUMBAS4SW
obj-y += RUMBAS4SW/
endif
ifneq ($(filter y, $(CONFIG_MTK_CAM_PD2120A)$(CONFIG_MTK_CAM_PD2085)),)
$(warning "############ois：main############")
subdir-ccflags-y += -DREAR_MAIN
else ifneq ($(filter y, $(CONFIG_MTK_CAM_PD2133)$(CONFIG_MTK_CAM_PD2135)),)
$(warning "############ois：main1############")
subdir-ccflags-y += -DREAR_MAIN1
endif

obj-y += ois_core.o ois_util.o
