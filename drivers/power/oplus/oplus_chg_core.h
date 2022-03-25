// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _OPLUS_CHG_CORE_H_
#define _OPLUS_CHG_CORE_H_

struct device;
struct device_type;
struct oplus_chg_mod;

#ifdef CONFIG_SYSFS

extern void oplus_chg_mod_init_attrs(struct device_type *dev_type);
extern int oplus_chg_uevent(struct device *dev, struct kobj_uevent_env *env);

#else

static inline void oplus_chg_mod_init_attrs(struct device_type *dev_type) {}
#define oplus_chg_uevent NULL

#endif /* CONFIG_SYSFS */

#endif /* _OPLUS_CHG_CORE_H_ */
