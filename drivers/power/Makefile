# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_POWER_RESET)	+= reset/
obj-$(CONFIG_POWER_SUPPLY)	+= supply/

#ifdef OPLUS_FEATURE_CHG_BASIC
obj-$(CONFIG_OPLUS_SM8450_CHARGER)	+= oplus/

ifeq ($(CONFIG_OPLUS_CHG_V2), y)
obj-$(CONFIG_OPLUS_SM8450_CHARGER)	+= oplus_v2/
endif
#endif
