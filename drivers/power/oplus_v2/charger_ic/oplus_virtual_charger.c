#define pr_fmt(fmt) "[VIRTUAL_CHARGER]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/oplus_chg.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/iio/consumer.h>
#include <linux/kthread.h>
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_short.h"
#include "../oplus_wireless.h"
#include "../charger_ic/oplus_short_ic.h"
#include "../oplus_adapter.h"
#include "../oplus_configfs.h"
#include "oplus_da9313.h"
//#include "op_charge.h"
#include "../wireless_ic/oplus_nu1619.h"
#include "../oplus_debug_info.h"
#include "../oplus_chg_module.h"
#include "../hal/oplus_chg_ic.h"
#include "oplus_virtual_charger.h"

struct oplus_virtual_charger {
	struct oplus_chg_chip chg_chip;
	struct oplus_chg_ic_dev *buck_ic;
	struct oplus_chg_ic_dev *voocphy_ic;

	struct delayed_work	ccdetect_work;
	struct delayed_work	usbtemp_recover_work;
	struct delayed_work	typec_state_change_work;
	struct delayed_work	virtual_charger_init_work;
};

struct oplus_chg_chip *g_oplus_chip = NULL;
static struct task_struct *oplus_usbtemp_kthread;
struct wakeup_source *usbtemp_wakelock;

#define USB_20C 20
#define USB_40C 40
#define USB_30C 30
#define USB_50C 50
#define USB_55C 55
#define USB_57C 57
#define USB_100C 100

#define USB_50C_VOLT 467
#define USB_55C_VOLT 400
#define USB_57C_VOLT 376
#define USB_100C_VOLT 100
#define VBUS_VOLT_THRESHOLD 400

#define VBUS_MONITOR_INTERVAL 3000

#define MIN_MONITOR_INTERVAL 50
#define MAX_MONITOR_INTERVAL 50
#define RETRY_CNT_DELAY 5
#define HIGH_TEMP_SHORT_CHECK_TIMEOUT 1000

static bool oplus_usbtemp_check_is_support(struct oplus_virtual_charger *vc_dev)
{
	bool support;
	int rc;

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_USB_TEMP_CHECK_IS_SUPPORT, &support);
	if (rc < 0)
		return false;
	return support;
}

static void oplus_set_typec_sinkonly(void)
{
	struct oplus_virtual_charger *vc_dev;

	if (g_oplus_chip == NULL) {
		chg_err("g_oplus_chip is null\n");
		return;
	}
	vc_dev = container_of(g_oplus_chip, struct oplus_virtual_charger, chg_chip);

	oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE, TYPEC_PORT_ROLE_SNK);
}

void oplus_set_usb_status(int status)
{
	if (g_oplus_chip)
		g_oplus_chip->usb_status = g_oplus_chip->usb_status | status;
}

void oplus_clear_usb_status(int status)
{
	if (g_oplus_chip)
		g_oplus_chip->usb_status = g_oplus_chip->usb_status & (~status);
}

int oplus_get_usb_status(void)
{
	if (g_oplus_chip)
		return g_oplus_chip->usb_status;
	return 0;
}

static int oplus_usbtemp_dischg_action(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev;

	if (chip == NULL) {
		chg_err("chip is null\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	if (get_eng_version() != HIGH_TEMP_AGING) {
		oplus_set_usb_status(USB_TEMP_HIGH);

		if (chip->voocphy_support) {
			rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_VOOCPHY_ENABLE, false);
			if (rc < 0) {
				chg_err("disable voocphy error\n");
				return rc;
			}
		} else if (oplus_vooc_get_fastchg_started()) {
			oplus_chg_set_chargerid_switch_val(0);
			oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
			oplus_vooc_reset_mcu();
		}

		usleep_range(10000, 10000);
		chip->chg_ops->charger_suspend();
		usleep_range(10000, 10000);

		if(chip->chg_ops->set_typec_sinkonly != NULL){
			chip->chg_ops->set_typec_sinkonly();
		} else {
			chg_err("set_typec_sinkonly is null");
		}
	}

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chg_err("CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
		rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, false);
	} else {
		chg_err("set vbus down");
		rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, true);
	}

	return 0;
}

#define RETRY_COUNT		3
static void oplus_update_usbtemp_current_status(struct oplus_chg_chip *chip)
{
	static int limit_cur_cnt_r = 0;
	static int limit_cur_cnt_l = 0;
	static int recover_cur_cnt_r = 0;
	static int recover_cur_cnt_l = 0;

	if (!chip) {
		return;
	}

	if((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C)
		&& (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		chip->usbtemp_cool_down = 0;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		return;
	}

	if ((chip->usb_temp_r  - chip->temperature/10) >= 12) {
		limit_cur_cnt_r++;
		if (limit_cur_cnt_r >= RETRY_COUNT) {
			limit_cur_cnt_r = RETRY_COUNT;
		}
		recover_cur_cnt_r = 0;
	} else if ((chip->usb_temp_r  - chip->temperature/10) <= 6)  {
		recover_cur_cnt_r++;
		if (recover_cur_cnt_r >= RETRY_COUNT) {
			recover_cur_cnt_r = RETRY_COUNT;
		}
		limit_cur_cnt_r = 0;
	}

	if ((chip->usb_temp_l  - chip->temperature/10) >= 12) {
		limit_cur_cnt_l++;
		if (limit_cur_cnt_l >= RETRY_COUNT) {
			limit_cur_cnt_l = RETRY_COUNT;
		}
		recover_cur_cnt_l = 0;
	} else if ((chip->usb_temp_l  - chip->temperature/10) <= 6)  {
		recover_cur_cnt_l++;
		if (recover_cur_cnt_l >= RETRY_COUNT) {
			recover_cur_cnt_l = RETRY_COUNT;
		}
		limit_cur_cnt_l = 0;
	}

	if ((RETRY_COUNT <= limit_cur_cnt_r || RETRY_COUNT <= limit_cur_cnt_l)
			&& (chip->smart_charge_user == SMART_CHARGE_USER_OTHER)) {
		chip->smart_charge_user = SMART_CHARGE_USER_USBTEMP;
		chip->cool_down_done = true;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	} else if ((RETRY_COUNT <= recover_cur_cnt_r &&  RETRY_COUNT <= recover_cur_cnt_l)
			&& (chip->smart_charge_user == SMART_CHARGE_USER_USBTEMP)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		chip->usbtemp_cool_down = 0;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	}

	return;
}

static int oplus_get_charger_voltage_now(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int vbus_mv;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
				&vbus_mv);
	if (rc < 0) {
		chg_err("error: get vbus, rc=%d\n", rc);
		vbus_mv = 0;
	}

	return vbus_mv;
}

static int get_battery_mvolts_for_usbtemp_monitor(struct oplus_chg_chip *chip)
{
#if 0 //nick.hu TODO
	if(!is_ext_chg_ops())
		return 5000;

	if(chip->chg_ops && chip->chg_ops->get_charger_volt)
		return chip->chg_ops->get_charger_volt();
#endif

	return oplus_get_charger_voltage_now();
}

static int oplus_usbtemp_monitor_main(void *data)
{
	int delay = 0;
	int vbus_volt = 0;
	static int count = 0;
	static int total_count = 0;
	static int last_usb_temp_l = 25;
	static int current_temp_l = 25;
	static int last_usb_temp_r = 25;
	static int current_temp_r = 25;
	int retry_cnt = 3, i = 0;
	int count_r = 1, count_l = 1;
	bool condition1 = false;
	bool condition2 = false;
	struct oplus_chg_chip *chip = g_oplus_chip;
	static int log_count = 0;
	struct oplus_virtual_charger *vc_dev;

	if (chip == NULL) {
		chg_err("g_oplus_chip is null\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	chg_err("[oplus_usbtemp_monitor_main]:run first!");

	while (!kthread_should_stop()) {
		wait_event_interruptible(chip->oplus_usbtemp_wq, chip->usbtemp_check == true);
		if(chip->dischg_flag == true) {
			goto dischg;
		}
		(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
					&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
		(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP,
					&chip->usb_temp_l, &chip->usb_temp_r);
		if ((chip->usb_temp_l < USB_50C) && (chip->usb_temp_r < USB_50C)) {/*get vbus when usbtemp < 50C*/
			vbus_volt = get_battery_mvolts_for_usbtemp_monitor(chip);
		} else {
			vbus_volt = 0;
		}
		if ((chip->usb_temp_l < USB_40C) && (chip->usb_temp_r < USB_40C)) {
			delay = MAX_MONITOR_INTERVAL;
			total_count = 10;
		} else {
			delay = MIN_MONITOR_INTERVAL;
			total_count = 30;
		}

		oplus_update_usbtemp_current_status(chip);

		if ((chip->usbtemp_volt_l < USB_50C) && (chip->usbtemp_volt_r < USB_50C) && (vbus_volt < VBUS_VOLT_THRESHOLD))
			delay = VBUS_MONITOR_INTERVAL;

		/*condition1  :the temp is higher than 57*/
		if (chip->tbatt_temp/10 <= USB_50C &&(((chip->usb_temp_l >= USB_57C) && (chip->usb_temp_l < USB_100C))
			|| ((chip->usb_temp_r >= USB_57C) && (chip->usb_temp_r < USB_100C)))) {
			chg_err("in loop 1");
			for (i = 1; i < retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
							&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
				(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP,
							&chip->usb_temp_l, &chip->usb_temp_r);
				if (chip->usb_temp_r >= USB_57C && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= USB_57C && chip->usb_temp_l < USB_100C)
					count_l++;
				chg_err("countl : %d", count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				chip->dischg_flag = true;
				condition1 = true;
				chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if (chip->tbatt_temp/10 > USB_50C && (((chip->usb_temp_l >= chip->tbatt_temp/10 + 7) && (chip->usb_temp_l < USB_100C))
			|| ((chip->usb_temp_r >= chip->tbatt_temp/10 + 7) && (chip->usb_temp_r < USB_100C)))) {
			chg_err("in loop 1");
			for (i = 1; i <= retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
							&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
				(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP,
							&chip->usb_temp_l, &chip->usb_temp_r);
				if ((chip->usb_temp_r >= chip->tbatt_temp/10 + 7) && chip->usb_temp_r < USB_100C)
					count_r++;
				if ((chip->usb_temp_l >= chip->tbatt_temp/10 + 7) && chip->usb_temp_l < USB_100C)
					count_l++;
				chg_err("countl : %d", count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				chip->dischg_flag = true;
				condition1 = true;
				chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if(condition1 == true) {
			chg_err("jump_to_dischg");
			goto dischg;
		}

		/*condition2  :the temp uprising to fast*/
		if ((((chip->usb_temp_l - chip->tbatt_temp/10) > chip->usbtemp_batttemp_gap) && (chip->usb_temp_l < USB_100C))
				|| (((chip->usb_temp_r - chip->tbatt_temp/10) > chip->usbtemp_batttemp_gap) && (chip->usb_temp_r < USB_100C))) {
			if (count == 0) {
				last_usb_temp_r = chip->usb_temp_r;
				last_usb_temp_l = chip->usb_temp_l;
			} else {
				current_temp_r = chip->usb_temp_r;
				current_temp_l = chip->usb_temp_l;
			}
			if (((current_temp_l - last_usb_temp_l) >= 3) || (current_temp_r - last_usb_temp_r) >= 3) {
				for (i = 1; i <= retry_cnt; i++) {
					mdelay(RETRY_CNT_DELAY);
					(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
								&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
					(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP,
								&chip->usb_temp_l, &chip->usb_temp_r);
					if ((chip->usb_temp_r - last_usb_temp_r) >= 3 && chip->usb_temp_r < USB_100C)
						count_r++;
					if ((chip->usb_temp_l - last_usb_temp_l) >= 3 && chip->usb_temp_l < USB_100C)
						count_l++;
					chg_err("countl : %d,countr : %d", count_l, count_r);
				}
				current_temp_l = chip->usb_temp_l;
				current_temp_r = chip->usb_temp_r;
				if ((count_l >= retry_cnt &&  chip->usb_temp_l > USB_30C && chip->usb_temp_l < USB_100C) ||
				    (count_r >= retry_cnt &&  chip->usb_temp_r > USB_30C  && chip->usb_temp_r < USB_100C)) {
					chip->dischg_flag = true;
					chg_err("dischg enable3...,current_temp_l=%d,last_usb_temp_l=%d,current_temp_r=%d,last_usb_temp_r =%d\n",
							current_temp_l, last_usb_temp_l, current_temp_r, last_usb_temp_r);
					condition2 = true;
				}
				count_r = 1;
				count_l = 1;
			}
			count++;
			if (count > total_count)
				count = 0;
		} else {
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
	/*judge whether to go the action*/
	dischg:
		if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C)
				&& (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
			condition1 = false;
			condition2 = false;
			chip->dischg_flag = false;
		}
		if((condition1== true || condition2 == true) && chip->dischg_flag == true) {
			oplus_usbtemp_dischg_action(chip);
			condition1 = false;
			condition2 = false;
		}
		msleep(delay);
		log_count++;
		if (log_count == 40) {
			chg_err("==================usbtemp_volt_l[%d], usb_temp_l[%d], usbtemp_volt_r[%d], usb_temp_r[%d]\n",
					chip->usbtemp_volt_l, chip->usb_temp_l, chip->usbtemp_volt_r, chip->usb_temp_r);
			log_count = 0;
		}
	}

	return 0;
}

static void oplus_usbtemp_thread_init(void)
{
	oplus_usbtemp_kthread = kthread_run(oplus_usbtemp_monitor_main, 0, "usbtemp_kthread");
	if (IS_ERR(oplus_usbtemp_kthread)) {
		chg_err("failed to cread oplus_usbtemp_kthread\n");
	}
}

void oplus_wake_up_usbtemp_thread(void)
{
	struct oplus_virtual_charger *vc_dev;

	if (g_oplus_chip == NULL) {
		chg_err("g_oplus_chip is null\n");
		return;
	}
	vc_dev = container_of(g_oplus_chip, struct oplus_virtual_charger, chg_chip);

	if (oplus_usbtemp_check_is_support(vc_dev))
		wake_up_interruptible(&g_oplus_chip->oplus_usbtemp_wq);
}

static void oplus_set_usbtemp_wakelock(bool value)
{
	if(value) {
		__pm_stay_awake(usbtemp_wakelock);
	} else {
		__pm_relax(usbtemp_wakelock);
	}
}

static void oplus_usbtemp_recover_func(struct oplus_virtual_charger *vc_dev)
{
	bool dischg_enable;
	int count_time = 0;
	struct oplus_chg_chip *chip = &vc_dev->chg_chip;
	int rc;

	if (oplus_usbtemp_check_is_support(vc_dev)){
		rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS, &dischg_enable);
		if (rc < 0) {
			chg_err("get dischg status error, rc=%d\n", rc);
			dischg_enable = false;
		}
	} else {
		return;
	}

	if (dischg_enable) {
		oplus_set_usbtemp_wakelock(true);
		do {
			(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
						&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
			(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GET_USB_TEMP,
						&chip->usb_temp_l, &chip->usb_temp_r);
			msleep(2000);
			count_time++;
		} while (!(((chip->usb_temp_r < USB_55C || chip->usb_temp_r == USB_100C)
			&& (chip->usb_temp_l < USB_55C ||  chip->usb_temp_l == USB_100C)) || count_time == 30));
		oplus_set_usbtemp_wakelock(false);
		if (count_time == 30) {
			chg_err("[OPLUS_USBTEMP] temp still high");
		} else {
			chip->dischg_flag = false;
			chg_err("dischg disable...[%d]\n", chip->usbtemp_volt);
			oplus_clear_usb_status(USB_TEMP_HIGH);
			(void)oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, false);
			chg_err("[OPLUS_USBTEMP] usbtemp recover");
		}
	}
	return;
}

static void oplus_usbtemp_recover_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_charger *chip = container_of(dwork,
				struct oplus_virtual_charger, usbtemp_recover_work);

	oplus_usbtemp_recover_func(chip);
}

static bool oplus_ccdetect_is_support(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF ||
	    boot_mode == MSM_BOOT_MODE__WLAN ||
	    boot_mode == MSM_BOOT_MODE__FACTORY)
		return false;

	return true;
}

static int oplus_ccdetect_enable(struct oplus_virtual_charger *vc_dev, bool en)
{
	int rc;

	if (!oplus_ccdetect_is_support())
		return 0;

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE,
		en ? TYPEC_PORT_ROLE_DRP : TYPEC_PORT_ROLE_DISABLE);
	if (rc < 0)
		chg_err("%s ccdetect error, rc=%d\n", en ? "enable" : "disable", rc);

	return rc;
}

bool oplus_get_otg_switch_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct oplus_virtual_charger *vc;
	bool otg_switch;
	int rc;

	if (!chip) {
		chg_err("g_oplus_chip is null\n");
		return false;
	}
	vc = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS, &otg_switch);
	if (rc < 0) {
		chg_err("can't get otg switch status, rc=%d\n", rc);
		return false;
	}
	chip->otg_switch = otg_switch;

	return otg_switch;
}

int oplus_set_otg_switch_status(bool enable)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct oplus_virtual_charger *vc;
	int rc;

	if (!chip) {
		chg_err("g_oplus_chip is null\n");
		return -ENODEV;
	}
	vc = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc->buck_ic,
			       OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS, enable);
	if (rc < 0) {
		chg_err("can't %s otg switch status, rc=%d\n", enable ? "enable" : "disable", rc);
		return rc;
	}

	chip->otg_switch = enable;
	chg_info("otg_switch=%d, otg_online=%d\n", chip->otg_switch, chip->otg_online);

	return rc;
}

int oplus_get_otg_online_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct oplus_virtual_charger *vc;
	int online;
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return 0;
	}
	vc = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_ONLINE_STATUS, &online);
	if (rc < 0) {
		chg_err("can't get otg online status, rc=%d\n", rc);
		return 0;
	}
	chip->otg_online = online;

	return online;
}

int oplus_get_typec_cc_orientation(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct oplus_virtual_charger *vc;
	int cc_orientation;
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return 0;
	}
	vc = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION, &cc_orientation);
	if (rc < 0) {
		chg_err("can't get cc orientation status, rc=%d\n", rc);
		return 0;
	}

	return cc_orientation;
}

static void oplus_ccdetect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_charger *chip = container_of(dwork,
				struct oplus_virtual_charger, ccdetect_work);
	struct oplus_chg_chip *chg_chip = g_oplus_chip;
	int hw_detect;
	int rc;

	if (chg_chip == NULL) {
		chg_err("g_oplus_chip is NULL\n");
		return;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, &hw_detect);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		return;
	}

	chg_info("hw_detect=%d\n", hw_detect);
	if (hw_detect == 1) {
		oplus_ccdetect_enable(chip, true);
		if (chg_chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&chip->usbtemp_recover_work, 0);
		}
	} else {
		chg_chip->usbtemp_check = false;
		if(chg_chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&chip->usbtemp_recover_work, 0);
		}
		if (oplus_get_otg_switch_status() == false) {
			oplus_ccdetect_enable(chip, false);
		}
	}
	(void)oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_CC_DETECT_HAPPENED);
}

#define CCDETECT_DELAY_MS	50
static void oplus_vc_cc_detect_handler(struct oplus_chg_ic_dev *buck_ic, void *virq_data)
{
	struct oplus_virtual_charger *chip = virq_data;

	cancel_delayed_work(&chip->ccdetect_work);
	schedule_delayed_work(&chip->ccdetect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));
}

static void oplus_vc_cc_changed_handler(struct oplus_chg_ic_dev *buck_ic, void *virq_data)
{
	struct oplus_virtual_charger *chip = virq_data;

	schedule_delayed_work(&chip->typec_state_change_work, 0);
}

static void oplus_typec_state_change_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_charger *chip = container_of(dwork,
		struct oplus_virtual_charger, typec_state_change_work);
	struct oplus_chg_chip *chg_chip = g_oplus_chip;
	int hw_detect;
	int rc;

	if (chg_chip == NULL) {
		chg_err("g_oplus_chip is NULL\n");
		return;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, &hw_detect);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		return;
	}

	chg_info("hw_detect=%d\n", hw_detect);

	if(oplus_ccdetect_is_support()) {
		if (hw_detect == 0 && oplus_get_otg_switch_status() == false)
			oplus_ccdetect_enable(chip, false);
	}
}

static int oplus_get_charger_cycle(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int cycle;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_GET_CHARGER_CYCLE,
				&cycle);
	if (rc < 0)
		chg_err("error: get charger cycle, rc=%d\n", rc);
	else
		chg_info("charger_cycle = %d\n", cycle);

	return rc;
}

static int oplus_voocphy_enable(bool enable)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_VOOCPHY_ENABLE,
				enable);
	if (rc < 0)
		chg_err("error: %s voocphy, rc=%d\n", enable ? "enable" : "disable", rc);
	else
		chg_info("%s voocphy\n", enable ? "enable" : "disable");

	return rc;
}

static int oplus_voocphy_reset_again(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN);
	if (rc < 0)
		chg_err("error: voocphy reset again, rc=%d\n", rc);
	else
		chg_info("voocphy reset again\n");

	return rc;
}

static void oplus_vc_dump_regs(void)
{
	return;
}

static int oplus_vc_kick_wdt(void)
{
	return 0;
}

static int oplus_vc_set_fastchg_current_raw(int current_ma)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_FCC,
				current_ma);
	if (rc < 0)
		chg_err("error: fcc = %d, rc=%d\n", current_ma, rc);
	else
		chg_info("fcc = %d\n", current_ma);

	return rc;
}

static int oplus_vc_set_wls_boost_en(bool enable)
{
	return 0;
}

static int oplus_chg_set_input_current_with_no_aicl(int current_ma)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	if (chip->mmi_chg == 0 && current_ma != 0) {
		chg_info("mmi_chg, return\n");
		return rc;
	}

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				current_ma);
	if (rc < 0)
		chg_err("error: icl = %d, rc=%d\n", current_ma, rc);
	else
		chg_info("icl = %d\n", current_ma);

	return rc;
}

static void oplus_vc_set_aicl_point(int vol)
{
	/*do nothing*/
}

bool oplus_chg_vc_get_vbus_collapse_status(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	bool collapse_status;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
				&collapse_status);
	if (rc < 0)
		return false;

	return collapse_status;
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1350, 1500, 1750, 2000, 3000,
};

static int oplus_chg_set_input_current(int current_ma)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	if (chip->mmi_chg == 0) {
		chg_info("mmi_chg, return\n");
		return 0;
	}

	chg_info("usb input max current limit=%d setting %02x\n", current_ma, i);
	if (chip->batt_volt > 4100) {
		aicl_point = 4550;
	} else {
		aicl_point = 4500;
	}

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_BUCK_CURR_DROP);
	if (rc < 0) {
		chg_err("failed to set the current to drop, rc=%d\n", rc);
	}

	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_info("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		pr_debug("use 500 here\n");
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		pr_debug("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(90000, 91000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1350 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(130000, 131000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 2;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(90000, 91000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 3;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 3; /*We DO NOT use 1.2A here*/
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 2; /*We use 1.2A here*/
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 6; /* 1750 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 3;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 3; /*1.2*/
		goto aicl_pre_step;
	}

	i = 7; /* 2000 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 2;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i =  i - 2; /*1.5*/
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 8; /* 3000 */
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(90000, 91000);
	if (oplus_chg_vc_get_vbus_collapse_status() == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = oplus_get_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_debug("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
aicl_end:
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_debug("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
aicl_boost_back:
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ICL,
				usb_icl[i]);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_debug("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_boost_back\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
aicl_return:
	return rc;
}

static int oplus_vc_float_voltage_set(int vfloat_mv)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_FV,
				vfloat_mv);
	if (rc < 0)
		chg_err("error: fv = %d, rc=%d\n", vfloat_mv, rc);
	else
		chg_info("fv = %d\n", vfloat_mv);

	return rc;
}

static int oplus_vc_term_current_set(int term_current)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ITERM,
				term_current);
	if (rc < 0)
		chg_err("error: iterm = %d, rc=%d\n", term_current, rc);
	else
		chg_info("iterm = %d\n", term_current);

	return rc;
}

static int oplus_vc_charging_enable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
				false);
	if (rc < 0)
		chg_err("error: battery charge enable, rc=%d\n", rc);
	else
		chg_info("battery charge enable\n");

	return rc;
}

static int oplus_vc_charging_disable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
				true);
	if (rc < 0)
		chg_err("error: battery charge disable, rc=%d\n", rc);
	else
		chg_info("battery charge disable\n");

	return rc;
}

static int oplus_vc_get_charge_enable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	bool suspend;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND,
				&suspend);
	if (rc < 0) {
		chg_err("error: get battery charge status, rc=%d\n", rc);
		suspend = true;
	} else {
		chg_info("battery charge is %s\n", suspend ? "disable" : "enable");
	}

	return !suspend;
}

static int oplus_vc_usb_suspend_enable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
				true);
	if (rc < 0)
		chg_err("error: usb suspend, rc=%d\n", rc);
	else
		chg_info("usb suspend\n");

	return rc;
}

static int oplus_vc_usb_suspend_disable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
				false);
	if (rc < 0)
		chg_err("error: usb unsuspend, rc=%d\n", rc);
	else
		chg_info("usb unsuspend\n");

	return rc;
}

__maybe_unused static int oplus_vc_usb_get_input_current_limit(int *curr_ua)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_ICL,
				curr_ua);
	if (rc < 0) {
		chg_err("error: get icl, rc=%d\n", rc);
	} else {
		*curr_ua *= 1000;
		chg_info("get icl=%d\n", *curr_ua);
	}

	return rc;
}

static int oplus_chg_hw_init(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode != MSM_BOOT_MODE__RF && boot_mode != MSM_BOOT_MODE__WLAN) {
		oplus_vc_usb_suspend_disable();
	} else {
		oplus_vc_usb_suspend_enable();
	}

	oplus_chg_set_input_current_with_no_aicl(500);
	oplus_vc_charging_enable();

	return 0;
}

static int oplus_vc_set_rechg_vol(int rechg_vol)
{
	return 0;
}

static int oplus_vc_reset_charger(void)
{
	return 0;
}

static int oplus_vc_read_full(void)
{
	return 0;
}

static int oplus_vc_otg_enable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
				true);
	if (rc < 0)
		chg_err("error: otg enable, rc=%d\n", rc);
	else
		chg_info("otg enable\n");

	return rc;
}

static int oplus_vc_otg_disable(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
				false);
	if (rc < 0)
		chg_err("error: otg enable, rc=%d\n", rc);
	else
		chg_info("otg enable\n");

	return rc;
}

static int oplus_set_chging_term_disable(void)
{
	return 0;
}

static bool qcom_check_charger_resume(void)
{
	return true;
}

static int oplus_vc_get_chargerid_volt(void)
{
	return 0;
}

static void oplus_vc_set_chargerid_switch_val(int value)
{
}


static int oplus_vc_get_chargerid_switch_val(void)
{
	return -1;
}

static bool oplus_vc_need_to_check_ibatt(void)
{
	return false;
}

static int oplus_vc_get_chg_current_step(void)
{
	return 25;
}

static int opchg_vc_get_charger_type(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int charger_type;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
				&charger_type);
	if (rc < 0) {
		chg_err("error: get charger type, rc=%d\n", rc);
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		goto get_type_done;
	}

	switch (charger_type) {
		case OPLUS_CHG_USB_TYPE_UNKNOWN:
			charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		case OPLUS_CHG_USB_TYPE_SDP:
			charger_type = POWER_SUPPLY_TYPE_USB;
			break;
		case OPLUS_CHG_USB_TYPE_CDP:
			charger_type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		default:
			charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
	}

get_type_done:
	if (chip && chip->wireless_support && oplus_wpc_get_wireless_charge_start() == true)
		charger_type = POWER_SUPPLY_TYPE_WIRELESS;
	chg_info("charger type = %d\n", charger_type);
	return charger_type;
}

static int oplus_get_ibus_current(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int ibus_ma;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
				&ibus_ma);
	if (rc < 0) {
		chg_err("error: get ibus, rc=%d\n", rc);
		ibus_ma = 0;
	} else {
		chg_info("ibus = %d\n", ibus_ma);
	}

	return ibus_ma;
}

static bool oplus_chg_is_usb_present(void)
{
	int rc = 0;
	bool vbus_rising = false;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
				&vbus_rising);
	if (rc < 0) {
		chg_err("error: get input present, rc=%d\n", rc);
		return false;
	} else {
		chg_info("input present = %s\n", vbus_rising ? "true" : "false");
	}

	if (vbus_rising == false && oplus_vooc_get_fastchg_started() == true) {
		if (oplus_get_charger_voltage_now() > 2000) {
			chg_info("vbus low but fastchg started true and chg vol > 2V\n");
			vbus_rising = true;
		}
	}

#if 0 // voocphy TODO
	if (vbus_rising == false && (oplus_vooc_get_fastchg_started() == true && (chip->vbatt_num == 2))) {
			chg_info("vbus low but fastchg started true and SVOOC\n");
			vbus_rising = true;
	}
#endif
	if (vbus_rising == false && oplus_wpc_get_wireless_charge_start()) {
			chg_info("vbus low but wpc has started\n");
			vbus_rising = true;
	}

	return vbus_rising;
}

static int qpnp_get_battery_voltage(void)
{
	return 3800;//Not use anymore
}

static int oplus_vc_get_boot_reason(void)
{
	return 0;
}

static int oplus_chg_get_shutdown_soc(void)
{
	return 0;
}

static int oplus_chg_backup_soc(int backup_soc)
{
	return 0;
}

static int oplus_vc_get_aicl_level_ma(void)
{
	return 0;
}

static void oplus_vc_rerun_aicl(void)
{
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_chg_get_dyna_aicl_result(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int icl_ma;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return 0;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_ICL,
				&icl_ma);
	if (rc < 0) {
		chg_err("error: get icl, rc=%d\n", rc);
		icl_ma = 0;
	} else {
		chg_info("get icl = %d\n", icl_ma);
	}

	return icl_ma;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

static int oplus_chg_get_charger_subtype(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int charger_type;
	int charg_subtype = CHARGER_SUBTYPE_DEFAULT;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return CHARGER_SUBTYPE_DEFAULT;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
				&charger_type);
	if (rc < 0) {
		chg_err("error: get charger type, rc=%d\n", rc);
		return CHARGER_SUBTYPE_DEFAULT;
	}

	switch (charger_type) {
	case OPLUS_CHG_USB_TYPE_QC2:
	case OPLUS_CHG_USB_TYPE_QC3:
		charg_subtype = CHARGER_SUBTYPE_QC;
		break;
	case OPLUS_CHG_USB_TYPE_PD:
	case OPLUS_CHG_USB_TYPE_PD_DRP:
	case OPLUS_CHG_USB_TYPE_PD_PPS:
		charg_subtype = CHARGER_SUBTYPE_PD;
		break;
	case OPLUS_CHG_USB_TYPE_VOOC:
		charg_subtype = CHARGER_SUBTYPE_FASTCHG_VOOC;
		break;
	case OPLUS_CHG_USB_TYPE_SVOOC:
		charg_subtype = CHARGER_SUBTYPE_FASTCHG_SVOOC;
		break;
	default:
		charg_subtype = CHARGER_SUBTYPE_DEFAULT;
		break;
	}

	return charg_subtype;
}

static bool oplus_chg_get_pd_type(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int charger_type;
	bool is_pd_type = false;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
				&charger_type);
	if (rc < 0) {
		chg_err("error: get charger type, rc=%d\n", rc);
		return false;
	}

	switch (charger_type) {
	case OPLUS_CHG_USB_TYPE_PD:
	case OPLUS_CHG_USB_TYPE_PD_DRP:
	case OPLUS_CHG_USB_TYPE_PD_PPS:
		is_pd_type = true;
		break;
	default:
		is_pd_type = false;
		break;
	}

	return is_pd_type;
}

static int oplus_chg_set_pd_config(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

#define OPLUS_PD_5V_PDO	0x31912c
#define OPLUS_PD_9V_PDO 0x32d12c

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	if ((chip->limits.vbatt_pdqc_to_5v_thr) > 0 &&
	    (chip->charger_volt > 7500) &&
	    (chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr)) {
		rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
				OPLUS_PD_5V_PDO);
		if (rc)
			chg_err("set PDO 5V fail, rc=%d\n", rc);
		else
			chg_err("set PDO 5V OK\n");
	} else if (chip->batt_volt < 5000) { //alway to 9
		rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
				OPLUS_PD_9V_PDO);
		if (rc)
			chg_err("set PDO 9V fail, rc=%d\n", rc);
		else
			chg_err("set PDO 9V OK\n");
	}

	return rc;
}

static int oplus_chg_set_qc_config(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	if ((chip->limits.vbatt_pdqc_to_5v_thr) > 0 &&
	    (chip->charger_volt > 7500) &&
	    (chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr)) {
		rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
				OPLUS_CHG_QC_2_0, 5000);
		if (rc)
			chg_err("set QC 5V fail, rc=%d\n", rc);
		else
			chg_err("set QC 5V OK\n");
	} else if (chip->batt_volt < 5000) { //alway to 9
		rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
				OPLUS_CHG_QC_2_0, 9000);
		if (rc)
			chg_err("set QC 9V fail, rc=%d\n", rc);
		else
			chg_err("set QC 9V OK\n");
	}

	return rc;
}

static int oplus_chg_enable_qc_detect(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE,
				true);
	if (rc < 0)
		chg_err("error: enable qc detect, rc=%d\n", rc);
	else
		chg_info("enable qc detect\n");

	return rc;
}

static void oplus_chg_set_curr_level_to_voocphy(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int cool_down;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	cool_down = oplus_chg_get_cool_down_status();
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL,
				cool_down);
	if (rc < 0)
		chg_err("error: set current level, rc=%d\n", rc);
}

static void oplus_chg_set_match_temp_to_voocphy(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int match_temp;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	match_temp = oplus_chg_match_temp_for_chging();
	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP,
				match_temp);
	if (rc < 0)
		chg_err("error: set match temp, rc=%d\n", rc);
}

static int oplus_input_current_limit_ctrl_by_vooc_write(int current_ma)
{
	int rc = 0;
	int real_ibus;
	int temp_curr;
	struct oplus_virtual_charger *vc_dev;

	if (g_oplus_chip == NULL) {
		chg_err("g_oplus_chip is null\n");
		return -ENODEV;
	}
	vc_dev = container_of(g_oplus_chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_BUCK_CURR_DROP);
	if (rc < 0) {
		chg_err("failed to set the current to drop, rc=%d\n", rc);
	}

	real_ibus = oplus_get_ibus_current();
	chg_err(" get input_current = %d\n", real_ibus);

	if (current_ma > real_ibus) {
		for (temp_curr = real_ibus; temp_curr < current_ma; temp_curr += 500) {
			msleep(35);
			rc = oplus_chg_set_input_current_with_no_aicl(temp_curr);
			chg_err("[up] set input_current = %d\n", temp_curr);
		}
	} else {
		for (temp_curr = real_ibus; temp_curr > current_ma; temp_curr -= 500) {
			msleep(35);
			rc = oplus_chg_set_input_current_with_no_aicl(temp_curr);
			chg_err("[down] set input_current = %d\n", temp_curr);
		}
	}

	rc = oplus_chg_set_input_current_with_no_aicl(current_ma);
	return rc;
}

static void oplus_get_props_from_adsp_by_buffer(void)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_GAUGE_UPDATE);
	if (rc < 0)
		chg_err("error: gauge info update, rc=%d\n", rc);
	else
		chg_info("gauge info update\n");
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static unsigned long suspend_tm_sec = 0;
static int oplus_virtual_chg_pm_resume(struct device *dev)
{
	int rc = 0;
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;

	if (!g_oplus_chip)
		return 0;

	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time < 0) {
		sleep_time = 0;
	}

	oplus_chg_soc_update_when_resume(sleep_time);

	return 0;
}

static int oplus_virtual_chg_pm_suspend(struct device *dev)
{
	if (!g_oplus_chip)
		return 0;

	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}

	return 0;
}

static void oplus_set_wdt_enable(bool enable)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return;
	}
	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic, OPLUS_IC_FUNC_BUCK_WDT_ENABLE, enable);
	if (rc < 0)
		chg_err("error: %s wdt, rc=%d\n", enable ? "enable" : "disable", rc);
}

static const struct dev_pm_ops oplus_virtual_chg_pm_ops = {
	.resume		= oplus_virtual_chg_pm_resume,
	.suspend	= oplus_virtual_chg_pm_suspend,
};

struct oplus_chg_operations  oplus_virtual_chg_ops = {
	.get_charger_cycle = oplus_get_charger_cycle,
	.voocphy_enable = oplus_voocphy_enable,
	.voocphy_reset_again = oplus_voocphy_reset_again,
	.dump_registers = oplus_vc_dump_regs,
	.kick_wdt = oplus_vc_kick_wdt,
	.hardware_init = oplus_chg_hw_init,
	.charging_current_write_fast = oplus_vc_set_fastchg_current_raw,
	.set_wls_boost_en = oplus_vc_set_wls_boost_en,
	.set_aicl_point = oplus_vc_set_aicl_point,
	.input_current_write = oplus_chg_set_input_current,
	.float_voltage_write = oplus_vc_float_voltage_set,
	.term_current_set = oplus_vc_term_current_set,
	.charging_enable = oplus_vc_charging_enable,
	.charging_disable = oplus_vc_charging_disable,
	.get_charging_enable = oplus_vc_get_charge_enable,
	.charger_suspend = oplus_vc_usb_suspend_enable,
	.charger_unsuspend = oplus_vc_usb_suspend_disable,
	.set_rechg_vol = oplus_vc_set_rechg_vol,
	.reset_charger = oplus_vc_reset_charger,
	.read_full = oplus_vc_read_full,
	.otg_enable = oplus_vc_otg_enable,
	.otg_disable = oplus_vc_otg_disable,
	.set_charging_term_disable = oplus_set_chging_term_disable,
	.check_charger_resume = qcom_check_charger_resume,
	.get_chargerid_volt = oplus_vc_get_chargerid_volt,
	.set_chargerid_switch_val = oplus_vc_set_chargerid_switch_val,
	.get_chargerid_switch_val = oplus_vc_get_chargerid_switch_val,
	.need_to_check_ibatt = oplus_vc_need_to_check_ibatt,
	.get_chg_current_step = oplus_vc_get_chg_current_step,
	.get_charger_type = opchg_vc_get_charger_type,
	.get_charger_volt = oplus_get_charger_voltage_now,
	.get_charger_current = oplus_get_ibus_current,
	.check_chrdet_status = oplus_chg_is_usb_present,
	.get_instant_vbatt = qpnp_get_battery_voltage,
	.get_boot_mode = get_boot_mode,
	.get_boot_reason = oplus_vc_get_boot_reason,
	.get_rtc_soc = oplus_chg_get_shutdown_soc,
	.set_rtc_soc = oplus_chg_backup_soc,
	.get_aicl_ma = oplus_vc_get_aicl_level_ma,
	.rerun_aicl = oplus_vc_rerun_aicl,
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_chg_get_dyna_aicl_result,
#endif
	.get_charger_subtype = oplus_chg_get_charger_subtype,
	.oplus_chg_get_pd_type = oplus_chg_get_pd_type,
	.oplus_chg_pd_setup = oplus_chg_set_pd_config,
	.set_qc_config = oplus_chg_set_qc_config,
	.enable_qc_detect = oplus_chg_enable_qc_detect,
	.set_curr_level_to_voocphy = oplus_chg_set_curr_level_to_voocphy,
	.set_match_temp_to_voocphy = oplus_chg_set_match_temp_to_voocphy,
	.input_current_ctrl_by_vooc_write = oplus_input_current_limit_ctrl_by_vooc_write,
	.get_props_from_adsp_by_buffer = oplus_get_props_from_adsp_by_buffer,
	.set_typec_sinkonly = oplus_set_typec_sinkonly,
	.oplus_chg_wdt_enable = oplus_set_wdt_enable,
	.input_current_write_without_aicl = oplus_chg_set_input_current_with_no_aicl,
};

static int oplus_vb_virq_register(struct oplus_virtual_charger *vc_dev)
{
	int rc;

	rc = oplus_chg_ic_virq_register(vc_dev->buck_ic, OPLUS_IC_VIRQ_CC_DETECT,
		oplus_vc_cc_detect_handler, vc_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CC_DETECT error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(vc_dev->buck_ic, OPLUS_IC_VIRQ_CC_CHANGED,
		oplus_vc_cc_changed_handler, vc_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CC_CHANGED error, rc=%d", rc);

	return 0;
}

static void oplus_virtual_charger_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_charger *chip = container_of(dwork,
		struct oplus_virtual_charger, virtual_charger_init_work);
	struct device_node *node = chip->chg_chip.dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;

	chip->buck_ic = of_get_oplus_chg_ic(node, "oplus,virtual_buck_ic", 0);
	if (chip->buck_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->virtual_charger_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,virtual_buck_ic not found\n");
		}
		retry = 0;
		return;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->virtual_charger_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("virtual_buck_ic init timeout\n");
		}
	} else if (rc < 0) {
		chg_err("virtual_buck_ic init error, rc=%d\n", rc);
	}
	retry = 0;

	oplus_vb_virq_register(chip);
}

static int oplus_virtual_chg_probe(struct platform_device *pdev)
{
	struct oplus_chg_chip *oplus_chip;
	struct oplus_virtual_charger *vc_dev;
	struct device_node *node = pdev->dev.of_node;
	int rc;

	if (oplus_gauge_check_chip_is_null()) {
		chg_debug("gauge null, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}

	vc_dev = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_charger), GFP_KERNEL);
	if (!vc_dev) {
		chg_err("oplus_virtual_charger devm_kzalloc failed.\n");
		return -ENOMEM;
	}
	g_oplus_chip = oplus_chip = &vc_dev->chg_chip;
	oplus_chip->dev = &pdev->dev;
	rc = oplus_chg_parse_svooc_dt(oplus_chip);

	oplus_chip->chg_ops = &oplus_virtual_chg_ops;
	platform_set_drvdata(pdev, vc_dev);

	of_platform_populate(node, NULL, NULL, oplus_chip->dev);

	oplus_chg_parse_charger_dt(oplus_chip);
	oplus_chg_configfs_init(oplus_chip);
	oplus_chg_wake_update_work();

	rc = oplus_chg_init(oplus_chip);
	if (rc < 0)
		goto error;

	if (oplus_usbtemp_check_is_support(vc_dev)) {
		oplus_usbtemp_thread_init();
	}

	if (qpnp_is_power_off_charging() == false) {
		oplus_tbatt_power_off_task_init(oplus_chip);
	}

	INIT_DELAYED_WORK(&vc_dev->ccdetect_work, oplus_ccdetect_work);
	INIT_DELAYED_WORK(&vc_dev->usbtemp_recover_work, oplus_usbtemp_recover_work);
	INIT_DELAYED_WORK(&vc_dev->typec_state_change_work, oplus_typec_state_change_work);
	INIT_DELAYED_WORK(&vc_dev->virtual_charger_init_work, oplus_virtual_charger_init_work);
	schedule_delayed_work(&vc_dev->virtual_charger_init_work, 0);

	chg_err("probe success\n");
	return 0;

error:
	oplus_chg_configfs_exit();
	devm_kfree(&pdev->dev, vc_dev);
	return rc;
}

static int oplus_virtual_chg_remove(struct platform_device *pdev)
{
	struct oplus_virtual_charger *chip = platform_get_drvdata(pdev);

	if(chip == NULL && chip->buck_ic == NULL)
		return -ENODEV;

	if (chip->buck_ic->online)
		(void)oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_EXIT);
	oplus_chg_ic_virq_release(chip->buck_ic, OPLUS_IC_VIRQ_CC_DETECT, chip);
	oplus_chg_ic_virq_release(chip->buck_ic, OPLUS_IC_VIRQ_CC_CHANGED, chip);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void oplus_vc_enter_shipmode(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct oplus_virtual_charger *vc_dev = NULL;

	vc_dev = container_of(chip, struct oplus_virtual_charger, chg_chip);

	rc = oplus_chg_ic_func(vc_dev->buck_ic,
				OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
				true);
	if (rc < 0)
		chg_err("error: enable shipmode, rc=%d\n", rc);
	else
		chg_info("enable shipmode\n");
}

static void oplus_virtual_chg_shutdown(struct platform_device *pdev)
{
	if (g_oplus_chip && g_oplus_chip->enable_shipmode) {
		oplus_vc_enter_shipmode(g_oplus_chip);
		msleep(1000);
	}
}

static const struct of_device_id oplus_virtual_chg_match_table[] = {
	{ .compatible = "oplus,virtual_charger" },
	{},
};

static struct platform_driver oplus_virtual_chg_driver = {
	.driver = {
		.name = "oplus-virtual_charger",
		.of_match_table = oplus_virtual_chg_match_table,
		.pm	= &oplus_virtual_chg_pm_ops,
	},
	.probe = oplus_virtual_chg_probe,
	.remove = oplus_virtual_chg_remove,
	.shutdown = oplus_virtual_chg_shutdown,
};

static __init int oplus_virtual_chg_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&oplus_virtual_chg_driver);

	return ret;
}

static __exit void oplus_virtual_chg_driver_exit(void)
{
	platform_driver_unregister(&oplus_virtual_chg_driver);
}

oplus_chg_module_register(oplus_virtual_chg_driver);
