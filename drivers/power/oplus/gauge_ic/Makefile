ifeq ($(CONFIG_OPLUS_CHARGER_OPTIGA),y)
subdir-ccflags-y += -I$(srctree)/drivers/gpio
subdir-ccflags-y += -I$(srctree)/drivers/pinctrl
subdir-ccflags-y += -I$(srctree)/drivers/pinctrl/qcom
endif

obj-y	+= oplus_bq27541.o
ifeq ($(CONFIG_OPLUS_CHARGER_OPTIGA),y)
obj-y   += oplus_optiga/
endif