#ifndef __OPLUS_VIRTUAL_VOOC_H__
#define __OPLUS_VIRTUAL_VOOC_H__

#include "../oplus_vooc.h"

enum oplus_chg_vooc_switch_mode {
	VOOC_SWITCH_MODE_NORMAL,
	VOOC_SWITCH_MODE_VOOC,
	VOOC_SWITCH_MODE_HEADPHONE,
};

enum oplus_chg_vooc_ic_type {
	OPLUS_VOOC_IC_UNKNOWN,
	OPLUS_VOOC_IC_RK826,
	OPLUS_VOOC_IC_OP10,
	OPLUS_VOOC_IC_RT5125,
};

bool oplus_vooc_btb_temp_over(void);
bool oplus_is_power_off_charging(struct oplus_vooc_chip *vooc_chip);
bool oplus_is_charger_reboot(struct oplus_vooc_chip *vooc_chip);

#endif /* __OPLUS_VIRTUAL_VOOC_H__ */