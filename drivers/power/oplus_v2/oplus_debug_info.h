#ifndef __OPLUS_DEBUG_INFO__H
#define __OPLUS_DEBUG_INFO__H

#include "oplus_charger.h"

enum oplus_chg_debug_info_notify_flag {
	/*电量跳变标志*/
	OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP,
	OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP,
	OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_BATT_SOC_JUMP,                      //5
	/*充电慢标志*/
	OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH,
	OPLUS_NOTIFY_CHG_SLOW_NON_OPLUS_CHARGER,
	OPLUS_NOTIFY_CHG_SLOW_OVERTIME,
	OPLUS_NOTIFY_CHG_SLOW_ADAPTER_INPUT_LOW_POWER,       //线损大
	OPLUS_NOTIFY_CHG_SLOW_VOOC_NON_START,            //10
	OPLUS_NOTIFY_CHG_SLOW_VOOC_ADAPTER_NON_MAX_POWER,        //非匹配的最大功率快充充电器
	OPLUS_NOTIFY_CHG_SLOW_CHARGER_OV,
	OPLUS_NOTIFY_CHG_SLOW_CHARGER_UV,
	OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP,
	OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP,        //15
	OPLUS_NOTIFY_CHG_SLOW_FASTCHG_TO_WARM,
	OPLUS_NOTIFY_CHG_SLOW_CHG_TYPE_SDP,
	OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH,
	OPLUS_NOTIFY_CHG_SLOW_CHG_POWER_LOW,
	OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME,  //20
	OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_CAM_ON_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_CALLING_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_COOLDOWN_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_CHECK_TIME,            //25

	//充电报满
	OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP,
	OPLUS_NOTIFY_CHG_BATT_FULL_AND_100_CAP,
	OPLUS_NOTIFY_CHG_FULL,

	//电池老化
	OPLUS_NOTIFY_BATT_AGING_CAP,
	OPLUS_NOTIFY_BATT_AGING,
	OPLUS_NOTIFY_CHG_UNSUSPEND,

	/*wireless*/
	OPLUS_NOTIFY_WIRELESS_BOOTUP,
	OPLUS_NOTIFY_WIRELESS_START_CHG,
	OPLUS_NOTIFY_WIRELESS_WIRELESS_CHG_BREAK,
	OPLUS_NOTIFY_WIRELESS_WIRELESS_CHG_END,
	OPLUS_NOTIFY_WIRELESS_START_TX,
	OPLUS_NOTIFY_WIRELESS_STOP_TX,

	/*rechg check flag*/
	OPLUS_NOTIFY_CHG_BATT_RECHG,

	/*voocphy err flag*/
	OPLUS_NOTIFY_VOOCPHY_ERR,

	OPLUS_NOTIFY_CHG_MAX_CNT,
};

struct wireless_chg_debug_info {
	int boot_version;
	int rx_version;
	int tx_version;
	int dock_version;
	int adapter_type;
	bool fastchg_ing;
	int vout;
	int iout;
	int rx_temperature;
	int wpc_dischg_status;
	int work_silent_mode;
	int break_count;
	int wpc_chg_err;
	int highest_temp;
	int max_iout;
	int min_cool_down;
	int min_skewing_current;
	int wls_auth_fail;
};

enum oplus_chg_debug_info_notify_type {
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_FULL,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_BATT_FCC,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_UNSUSPEND,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_WIRELESS,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_RECHG,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_VOOCPHY_ERR,

	OPLUS_CHG_DEBUG_NOTIFY_TYPE_MAX,
};

extern int oplus_chg_debug_info_init(void);
extern int oplus_chg_debug_chg_monitor(struct oplus_chg_chip *chip);
extern int oplus_chg_debug_set_cool_down_by_user(int is_cool_down);
extern int oplus_chg_debug_get_cooldown_current(int chg_current_by_tbatt, int chg_current_by_cooldown);
extern int oplus_chg_debug_set_soc_info(struct oplus_chg_chip *chip);
extern void oplus_chg_wireless_error(int error,  struct wireless_chg_debug_info *wireless_param);
extern int oplus_chg_unsuspend_plat_pmic(struct oplus_chg_chip *chip);
extern int oplus_chg_voocphy_err(void);
#endif
