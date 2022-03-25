#ifndef __OPLUS_MMS_GAUGE_H__
#define __OPLUS_MMS_GAUGE_H__

enum gauge_topic_item {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_VOL_MIN,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
	GAUGE_ITEM_FCC,
	GAUGE_ITEM_CC,
	GAUGE_ITEM_SOH,
	GAUGE_ITEM_RM,
};

int oplus_gauge_get_batt_mvolts(void);
int oplus_gauge_get_batt_fc(void);
int oplus_gauge_get_batt_qm(void);
int oplus_gauge_get_batt_pd(void);
int oplus_gauge_get_batt_rcu(void);
int oplus_gauge_get_batt_rcf(void);
int oplus_gauge_get_batt_fcu(void);
int oplus_gauge_get_batt_fcf(void);
int oplus_gauge_get_batt_sou(void);
int oplus_gauge_get_batt_do0(void);
int oplus_gauge_get_batt_doe(void);
int oplus_gauge_get_batt_trm(void);
int oplus_gauge_get_batt_pc(void);
int oplus_gauge_get_batt_qs(void);
int oplus_gauge_get_batt_mvolts_2cell_max(void);
int oplus_gauge_get_batt_mvolts_2cell_min(void);

int oplus_gauge_get_batt_temperature(void);
int oplus_gauge_get_batt_soc(void);
int oplus_gauge_get_batt_current(void);
int oplus_gauge_get_remaining_capacity(void);
int oplus_gauge_get_device_type(void);
int oplus_gauge_get_device_type_for_vooc(void);

int oplus_gauge_get_batt_fcc(void);

int oplus_gauge_get_batt_cc(void);
int oplus_gauge_get_batt_soh(void);
bool oplus_gauge_get_batt_hmac(void);
bool oplus_gauge_get_batt_authenticate(void);
void oplus_gauge_set_batt_full(bool);
bool oplus_gauge_check_chip_is_null(void);

int oplus_gauge_update_battery_dod0(void);
int oplus_gauge_update_soc_smooth_parameter(void);
int oplus_gauge_get_battery_cb_status(void);
int oplus_gauge_get_i2c_err(void);
void oplus_gauge_clear_i2c_err(void);
int oplus_gauge_get_passedchg(int *val);
int oplus_gauge_dump_register(void);
int oplus_gauge_lock(void);
int oplus_gauge_unlock(void);
int oplus_gauge_get_batt_num(void);

#endif /* __OPLUS_MMS_GAUGE_H__ */