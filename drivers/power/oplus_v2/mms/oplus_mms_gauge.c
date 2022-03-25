#define pr_fmt(fmt) "[MMS_GAUGE]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#include "../oplus_chg_module.h"
#include "../hal/oplus_chg_ic.h"
#include "../oplus_gauge.h"
#include "../oplus_debug_info.h"
#include "oplus_mms.h"
#include "oplus_mms_gauge.h"

struct oplus_mms_gauge {
	struct device *dev;
	struct oplus_chg_ic_dev *gauge_ic;
	struct oplus_mms *gauge_topic;

	struct delayed_work mms_gauge_init_work;
	struct work_struct err_handler_work;

	int device_type;
	int device_type_for_vooc;
};

static struct oplus_mms_gauge *g_mms_gauge;

static int gauge_dbg_tbat = 0;
module_param(gauge_dbg_tbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_tbat, "debug battery temperature");

static int gauge_dbg_vbat = 0;
module_param(gauge_dbg_vbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_vbat, "debug battery voltage");

static int gauge_dbg_ibat = 0;
module_param(gauge_dbg_ibat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_ibat, "debug battery current");

int oplus_gauge_get_batt_mvolts(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 3800;

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		return gauge_dbg_vbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage error, rc=%d\n", rc);
		return 3800;
	}

	return vol_mv;
}

int oplus_gauge_get_batt_fc(void)
{
	int rc;
	int fc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FC, &fc);
	if (rc < 0) {
		chg_err("get battery fc error, rc=%d\n", rc);
		return 0;
	}

	return fc;
}

int oplus_gauge_get_batt_qm(void)
{
	int rc;
	int qm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QM, &qm);
	if (rc < 0) {
		chg_err("get battery qm error, rc=%d\n", rc);
		return 0;
	}

	return qm;
}

int oplus_gauge_get_batt_pd(void)
{
	int rc;
	int pd;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PD, &pd);
	if (rc < 0) {
		chg_err("get battery pd error, rc=%d\n", rc);
		return 0;
	}

	return pd;
}

int oplus_gauge_get_batt_rcu(void)
{
	int rc;
	int rcu;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCU, &rcu);
	if (rc < 0) {
		chg_err("get battery rcu error, rc=%d\n", rc);
		return 0;
	}

	return rcu;
}

int oplus_gauge_get_batt_rcf(void)
{
	int rc;
	int rcf;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCF, &rcf);
	if (rc < 0) {
		chg_err("get battery rcf error, rc=%d\n", rc);
		return 0;
	}

	return rcf;
}

int oplus_gauge_get_batt_fcu(void)
{
	int rc;
	int fcu;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCU, &fcu);
	if (rc < 0) {
		chg_err("get battery fcu error, rc=%d\n", rc);
		return 0;
	}

	return fcu;
}

int oplus_gauge_get_batt_fcf(void)
{
	int rc;
	int fcf;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCF, &fcf);
	if (rc < 0) {
		chg_err("get battery fcf error, rc=%d\n", rc);
		return 0;
	}

	return fcf;
}

int oplus_gauge_get_batt_sou(void)
{
	int rc;
	int sou;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOU, &sou);
	if (rc < 0) {
		chg_err("get battery sou error, rc=%d\n", rc);
		return 0;
	}

	return sou;
}

int oplus_gauge_get_batt_do0(void)
{
	int rc;
	int do0;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DO0, &do0);
	if (rc < 0) {
		chg_err("get battery do0 error, rc=%d\n", rc);
		return 0;
	}

	return do0;
}

int oplus_gauge_get_batt_doe(void)
{
	int rc;
	int doe;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DOE, &doe);
	if (rc < 0) {
		chg_err("get battery doe error, rc=%d\n", rc);
		return 0;
	}

	return doe;
}

int oplus_gauge_get_batt_trm(void)
{
	int rc;
	int trm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TRM, &trm);
	if (rc < 0) {
		chg_err("get battery trm error, rc=%d\n", rc);
		return 0;
	}

	return trm;
}

int oplus_gauge_get_batt_pc(void)
{
	int rc;
	int pc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PC, &pc);
	if (rc < 0) {
		chg_err("get battery pc error, rc=%d\n", rc);
		return 0;
	}

	return pc;
}

int oplus_gauge_get_batt_qs(void)
{
	int rc;
	int qs;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QS, &qs);
	if (rc < 0) {
		chg_err("get battery qs error, rc=%d\n", rc);
		return 0;
	}

	return qs;
}

int oplus_gauge_get_batt_mvolts_2cell_max(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

int oplus_gauge_get_batt_mvolts_2cell_min(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

int oplus_gauge_get_batt_temperature(void)
{
	int rc;
	int temp;

	if (!g_mms_gauge)
		return -400;
	if (gauge_dbg_tbat != 0) {
		chg_err("debug enabled, gauge_dbg_tbat[%d]\n", gauge_dbg_tbat);
		return gauge_dbg_tbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0) {
		chg_err("get battery temp error, rc=%d\n", rc);
		return -400;
	}
	if (get_eng_version() == HIGH_TEMP_AGING) {
		chg_info("[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here, "
			 "disable high tbat shutdown\n");
		if (temp > 690)
			temp = 690;
	}

	return temp;
}

int oplus_gauge_get_batt_soc(void)
{
	int rc;
	int soc;

	if (!g_mms_gauge)
		return 50;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		return 50;
	}

	return soc;
}

int oplus_gauge_get_batt_current(void)
{
	int rc;
	int curr_ma;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &curr_ma);
	if (rc < 0) {
		chg_err("get battery average current error, rc=%d\n", rc);
		return 0;
	}

	return curr_ma;
}

int oplus_gauge_get_remaining_capacity(void)
{
	int rc;
	int rm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		return 0;
	}

	return rm;
}

int oplus_gauge_get_device_type(void)
{
	if (!g_mms_gauge)
		return 0;
	return g_mms_gauge->device_type;
}

int oplus_gauge_get_device_type_for_vooc(void)
{
	if (!g_mms_gauge)
		return 0;
	return g_mms_gauge->device_type_for_vooc;
}

int oplus_gauge_get_batt_fcc(void)
{
	int rc;
	int fcc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
	if (rc < 0) {
		chg_err("get battery fcc error, rc=%d\n", rc);
		return 0;
	}

	return fcc;
}

int oplus_gauge_get_batt_cc(void)
{
	int rc;
	int cc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &cc);
	if (rc < 0) {
		chg_err("get battery cc error, rc=%d\n", rc);
		return 0;
	}

	return cc;
}

int oplus_gauge_get_batt_soh(void)
{
	int rc;
	int soh;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery soh error, rc=%d\n", rc);
		return 0;
	}

	return soh;
}

bool oplus_gauge_get_batt_hmac(void)
{
	int rc;
	bool pass;

	if (!g_mms_gauge)
		return false;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC, &pass);
	if (rc < 0) {
		chg_err("get battery hmac status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

bool oplus_gauge_get_batt_authenticate(void)
{
	int rc;
	bool pass;

	if (!g_mms_gauge)
		return false;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH, &pass);
	if (rc < 0) {
		chg_err("get battery authenticate status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

void oplus_gauge_set_batt_full(bool full)
{
	int rc;

	if (!g_mms_gauge)
		return;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL, full);
	if (rc < 0)
		chg_err("set battery full error, rc=%d\n", rc);
}

bool oplus_gauge_check_chip_is_null(void)
{
	if (!g_mms_gauge) {
		return true;
	} else {
		return false;
	}
}

int oplus_gauge_update_battery_dod0(void)
{
	int rc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0);
	if (rc < 0)
		chg_err("update battery dod0 error, rc=%d\n", rc);

	return 0;
}

int oplus_gauge_update_soc_smooth_parameter(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH);
	if (rc < 0)
		chg_err("update soc smooth parameter, rc=%d\n", rc);

	return rc;
}

int oplus_gauge_get_battery_cb_status(void)
{
	int rc;
	int cb_status;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_CB_STATUS, &cb_status);
	if (rc < 0) {
		chg_err("update soc smooth parameter, rc=%d\n", rc);
		return 0;
	}

	return cb_status;
}

int oplus_gauge_get_i2c_err(void)
{
	return 0; //nick.hu TODO
}

void oplus_gauge_clear_i2c_err(void)
{
	//nick.hu TODO
}

int oplus_gauge_get_passedchg(int *val)
{
	int rc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_PASSEDCHG, val);
	if (rc < 0) {
		chg_err("get passedchg error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int oplus_gauge_dump_register(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_REG_DUMP);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

int oplus_gauge_lock(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_LOCK, true);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

int oplus_gauge_unlock(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_LOCK, false);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

int oplus_gauge_get_batt_num(void)
{
	int rc;
	int num;

	if (!g_mms_gauge)
		return 1;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM, &num);
	if (rc < 0) {
		chg_err("can't get battery num, rc=%d\n", rc);
		return 1;
	}

	return num;
}

static int oplus_mms_gauge_virq_register(struct oplus_mms_gauge *chip);
static int oplus_mms_gauge_topic_init(struct oplus_mms_gauge *chip);

static void oplus_mms_gauge_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork,
		struct oplus_mms_gauge, mms_gauge_init_work);
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;

	chip->gauge_ic = of_get_oplus_chg_ic(node, "oplus,gauge_ic", 0);
	if (chip->gauge_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->mms_gauge_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,gauge_ic not found\n");
		}
		retry = 0;
		return;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->mms_gauge_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("gauge_ic init timeout\n");
		}
		retry = 0;
		return;
	} else if (rc < 0) {
		chg_err("gauge_ic init error, rc=%d\n", rc);
		retry = 0;
		return;
	}
	retry = 0;

	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
			       &chip->device_type);
	if (rc < 0) {
		chg_err("can't get device type, rc=%d\n");
		chip->device_type = 0;
	}
	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC,
			       &chip->device_type_for_vooc);
	if (rc < 0) {
		chg_err("can't get device type for vooc, rc=%d\n");
		chip->device_type_for_vooc = 0;
	}

	oplus_mms_gauge_virq_register(chip);
	g_mms_gauge = chip;

	(void)oplus_mms_gauge_topic_init(chip);
	chg_info("gauge topic ready\n");
}

static void oplus_mms_gauge_err_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(work, struct oplus_mms_gauge,
						err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&chip->gauge_ic->err_list_lock);
	if (!list_empty(&chip->gauge_ic->err_list))
		list_replace_init(&chip->gauge_ic->err_list, &msg_list);
	spin_unlock(&chip->gauge_ic->err_list_lock);

	list_for_each_entry_safe (msg, tmp, &msg_list, list) {
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_mms_gauge_err_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->err_handler_work);
}

static int oplus_mms_gauge_virq_register(struct oplus_mms_gauge *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->gauge_ic, OPLUS_IC_VIRQ_ERR,
		oplus_mms_gauge_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);

	return 0;
}

static int oplus_mms_gauge_update_soc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int soc;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		soc = 50;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		soc = 50;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		soc = 50;
	}

end:
	data->intval = soc;
	return 0;
}

static int oplus_mms_gauge_update_vol_max(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		vol = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		vol = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		vol = 0;
	}
end:
	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_vol_min(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		vol = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		vol = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		vol = 0;
	}
end:
	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int curr;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		curr = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		curr = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &curr);
	if (rc < 0) {
		chg_err("get battery current error, rc=%d\n", rc);
		curr = 0;
	}
end:
	data->intval = curr;
	return 0;
}

static int oplus_mms_gauge_update_temp(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int temp;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		temp = -40;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		temp = -40;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0) {
		chg_err("get battery temp error, rc=%d\n", rc);
		temp = -40;
	}
end:
	data->intval = temp;
	return 0;
}

static int oplus_mms_gauge_update_fcc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int fcc;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		fcc = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		fcc = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
	if (rc < 0) {
		chg_err("get battery fcc error, rc=%d\n", rc);
		fcc = 0;
	}
end:
	data->intval = fcc;
	return 0;
}

static int oplus_mms_gauge_update_rm(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rm;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		rm = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		rm = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		rm = 0;
	}
end:
	data->intval = rm;
	return 0;
}

static int oplus_mms_gauge_update_cc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int cc;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		cc = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		cc = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &cc);
	if (rc < 0) {
		chg_err("get battery cc error, rc=%d\n", rc);
		cc = 0;
	}
end:
	data->intval = cc;
	return 0;
}

static int oplus_mms_gauge_update_soh(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int soh;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		soh = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		soh = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery soh error, rc=%d\n", rc);
		soh = 0;
	}
end:
	data->intval = soh;
	return 0;
}

static void oplus_mms_gauge_update(struct oplus_mms *mms)
{
	struct oplus_mms_gauge *chip;
	struct mms_msg *msg;
	bool update = false;
	int i, rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}
	chip = oplus_mms_get_drvdata(mms);

	(void)oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE);

	for (i = 0; i < mms->desc->update_items_num; i++)
		update |= oplus_mms_item_update(mms, mms->desc->update_items[i], true);
	if (update) {
		msg = kzalloc(sizeof(struct mms_msg), GFP_KERNEL);
		if (msg == NULL) {
			chg_err("alloc msg buf error\n");
			return;
		}
		msg->prio = MSG_PRIO_MEDIUM;
		msg->type = MSG_TYPE_TIMER;
		rc = oplus_mms_publish_msg(mms, msg);
		if (rc < 0) {
			chg_err("publish msg error, rc=%d\n", rc);
			kfree(msg);
			return;
		}
	}
}

static struct mms_item oplus_mms_gauge_item[] = {
	{
		.desc = {
			.item_id = GAUGE_ITEM_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_soc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vol_max,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MIN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vol_min,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_curr,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_TEMP,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_temp,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_FCC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_fcc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RM,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_rm,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_cc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SOH,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_soh,
		}
	}
};

static const u32 oplus_mms_gauge_update_item[] = {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_VOL_MIN,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
	GAUGE_ITEM_FCC,
	GAUGE_ITEM_CC,
	GAUGE_ITEM_SOH,
	GAUGE_ITEM_RM,
};

static const struct oplus_mms_desc oplus_mms_gauge_desc = {
	.name = "gauge",
	.type = OPLUS_MMS_TYPE_GAUGE,
	.item_table = oplus_mms_gauge_item,
	.item_num = ARRAY_SIZE(oplus_mms_gauge_item),
	.update_items = oplus_mms_gauge_update_item,
	.update_items_num = ARRAY_SIZE(oplus_mms_gauge_update_item),
	.update_interval = 5000, /* ms */
	.update = oplus_mms_gauge_update,
};

static int oplus_mms_gauge_topic_init(struct oplus_mms_gauge *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	if (of_property_read_bool(mms_cfg.of_node,
				  "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node,
					  "oplus,topic-update-interval",
					  &mms_cfg.update_interval);
		if (rc < 0) {
			chg_err("can't read oplus,topic-update-interval, rc=%d\n",
				rc);
			mms_cfg.update_interval = 0;
		}
	}

	chip->gauge_topic = devm_oplus_mms_register(chip->dev, &oplus_mms_gauge_desc, &mms_cfg);
	if (IS_ERR(chip->gauge_topic)) {
		chg_err("Couldn't register gauge topic\n");
		rc = PTR_ERR(chip->gauge_topic);
		return rc;
	}

	return 0;
}

static int oplus_mms_gauge_probe(struct platform_device *pdev)
{
	struct oplus_mms_gauge *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_mms_gauge),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);

	INIT_DELAYED_WORK(&chip->mms_gauge_init_work, oplus_mms_gauge_init_work);
	INIT_WORK(&chip->err_handler_work, oplus_mms_gauge_err_handler_work);

	schedule_delayed_work(&chip->mms_gauge_init_work, 0);

	chg_info("probe success\n");
	return 0;
}

static int oplus_mms_gauge_remove(struct platform_device *pdev)
{
	struct oplus_mms_gauge *chip = platform_get_drvdata(pdev);

	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id oplus_mms_gauge_match[] = {
	{ .compatible = "oplus,mms_gauge" },
	{},
};

static struct platform_driver oplus_mms_gauge_driver = {
	.driver		= {
		.name = "oplus-mms_gauge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_mms_gauge_match),
	},
	.probe		= oplus_mms_gauge_probe,
	.remove		= oplus_mms_gauge_remove,
};

static __init int oplus_mms_gauge_init(void)
{
	return platform_driver_register(&oplus_mms_gauge_driver);
}

static __exit void oplus_mms_gauge_exit(void)
{
	platform_driver_unregister(&oplus_mms_gauge_driver);
}

oplus_chg_module_register(oplus_mms_gauge);
