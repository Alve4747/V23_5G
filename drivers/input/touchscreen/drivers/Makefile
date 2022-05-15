ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif

obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o


vivo_ts-y += $(addprefix gt9886/, \
				goodix_cfg_bin.o \
				goodix_ts_core.o \
				goodix_ts_gesture.o \
				goodix_ts_i2c.o \
				goodix_ts_node.o \
				goodix_ts_spi.o \
				goodix_ts_tools.o \
				goodix_ts_update.o \
			)

vivo_ts-y += vts_init.o
