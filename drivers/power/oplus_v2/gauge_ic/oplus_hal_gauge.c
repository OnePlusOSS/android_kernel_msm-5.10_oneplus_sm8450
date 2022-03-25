#define pr_fmt(fmt) "[HAL_GAUGE]([%s][%d]): " fmt, __func__, __LINE__

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

struct oplus_hal_gauge_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *gauge_ic;

	struct oplus_gauge_chip gauge_chip;

	struct delayed_work hal_gauge_init_work;
};

static struct oplus_hal_gauge_ic *g_gauge_ic;

static int oplus_chg_gauge_virq_register(struct oplus_hal_gauge_ic *chip);

static int oplus_chg_gauge_get_battery_mvolts(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_get_battery_fc(void)
{
	int rc;
	int fc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FC, &fc);
	if (rc < 0) {
		chg_err("get battery fc error, rc=%d\n", rc);
		return 0;
	}

	return fc;
}

static int oplus_chg_gauge_get_battery_qm(void)
{
	int rc;
	int qm;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QM, &qm);
	if (rc < 0) {
		chg_err("get battery qm error, rc=%d\n", rc);
		return 0;
	}

	return qm;
}

static int oplus_chg_gauge_get_battery_pd(void)
{
	int rc;
	int pd;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PD, &pd);
	if (rc < 0) {
		chg_err("get battery pd error, rc=%d\n", rc);
		return 0;
	}

	return pd;
}

static int oplus_chg_gauge_get_battery_rcu(void)
{
	int rc;
	int rcu;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCU, &rcu);
	if (rc < 0) {
		chg_err("get battery rcu error, rc=%d\n", rc);
		return 0;
	}

	return rcu;
}

static int oplus_chg_gauge_get_battery_rcf(void)
{
	int rc;
	int rcf;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCF, &rcf);
	if (rc < 0) {
		chg_err("get battery rcf error, rc=%d\n", rc);
		return 0;
	}

	return rcf;
}

static int oplus_chg_gauge_get_battery_fcu(void)
{
	int rc;
	int fcu;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCU, &fcu);
	if (rc < 0) {
		chg_err("get battery fcu error, rc=%d\n", rc);
		return 0;
	}

	return fcu;
}

static int oplus_chg_gauge_get_battery_fcf(void)
{
	int rc;
	int fcf;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCF, &fcf);
	if (rc < 0) {
		chg_err("get battery fcf error, rc=%d\n", rc);
		return 0;
	}

	return fcf;
}

static int oplus_chg_gauge_get_battery_sou(void)
{
	int rc;
	int sou;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOU, &sou);
	if (rc < 0) {
		chg_err("get battery sou error, rc=%d\n", rc);
		return 0;
	}

	return sou;
}

static int oplus_chg_gauge_get_battery_do0(void)
{
	int rc;
	int do0;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DO0, &do0);
	if (rc < 0) {
		chg_err("get battery do0 error, rc=%d\n", rc);
		return 0;
	}

	return do0;
}

static int oplus_chg_gauge_get_battery_doe(void)
{
	int rc;
	int doe;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DOE, &doe);
	if (rc < 0) {
		chg_err("get battery doe error, rc=%d\n", rc);
		return 0;
	}

	return doe;
}

static int oplus_chg_gauge_get_battery_trm(void)
{
	int rc;
	int trm;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TRM, &trm);
	if (rc < 0) {
		chg_err("get battery trm error, rc=%d\n", rc);
		return 0;
	}

	return trm;
}

static int oplus_chg_gauge_get_battery_pc(void)
{
	int rc;
	int pc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PC, &pc);
	if (rc < 0) {
		chg_err("get battery pc error, rc=%d\n", rc);
		return 0;
	}

	return pc;
}

static int oplus_chg_gauge_get_battery_qs(void)
{
	int rc;
	int qs;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QS, &qs);
	if (rc < 0) {
		chg_err("get battery qs error, rc=%d\n", rc);
		return 0;
	}

	return qs;
}

static int oplus_chg_gauge_get_battery_temperature(void)
{
	int rc;
	int temp;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return -400;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0) {
		chg_err("get battery temp error, rc=%d\n", rc);
		return -400;
	}

	return temp;
}

static int oplus_chg_gauge_get_batt_remaining_capacity(void)
{
	int rc;
	int rm;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		return 0;
	}

	return rm;
}

static int oplus_chg_gauge_get_battery_soc(void)
{
	int rc;
	int soc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 50;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		return 50;
	}

	return soc;
}

static int oplus_chg_gauge_get_average_current(void)
{
	int rc;
	int curr_ma;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &curr_ma);
	if (rc < 0) {
		chg_err("get battery average current error, rc=%d\n", rc);
		return 0;
	}

	return curr_ma;
}

static int oplus_chg_gauge_get_battery_fcc(void)
{
	int rc;
	int fcc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
	if (rc < 0) {
		chg_err("get battery fcc error, rc=%d\n", rc);
		return 0;
	}

	return fcc;
}

static int oplus_chg_gauge_get_prev_batt_fcc(void)
{
	int rc;
	int fcc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
	if (rc < 0) {
		chg_err("get battery fcc error, rc=%d\n", rc);
		return 0;
	}

	return fcc;
}

static int oplus_chg_gauge_get_battery_cc(void)
{
	int rc;
	int cc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &cc);
	if (rc < 0) {
		chg_err("get battery cc error, rc=%d\n", rc);
		return 0;
	}

	return cc;
}

static int oplus_chg_gauge_get_battery_soh(void)
{
	int rc;
	int soh;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery soh error, rc=%d\n", rc);
		return 0;
	}

	return soh;
}

static bool oplus_chg_gauge_get_battery_authenticate(void)
{
	int rc;
	bool pass;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return false;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH, &pass);
	if (rc < 0) {
		chg_err("get battery authenticate status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

static bool oplus_chg_gauge_get_battery_hmac(void)
{
	int rc;
	bool pass;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return false;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC, &pass);
	if (rc < 0) {
		chg_err("get battery hmac status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

static void oplus_chg_gauge_set_battery_full(bool full)
{
	int rc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL, full);
	if (rc < 0)
		chg_err("set battery full error, rc=%d\n", rc);
}

static int oplus_chg_gauge_get_prev_battery_mvolts(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_get_prev_battery_temperature(void)
{
	int rc;
	int temp;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return -400;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0) {
		chg_err("get battery temp error, rc=%d\n", rc);
		return -400;
	}

	return temp;
}

static int oplus_chg_gauge_get_prev_battery_soc(void)
{
	int rc;
	int soc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 50;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		return 50;
	}

	return soc;
}

static int oplus_chg_gauge_get_prev_average_current(void)
{
	int rc;
	int curr_ma;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &curr_ma);
	if (rc < 0) {
		chg_err("get battery average current error, rc=%d\n", rc);
		return 0;
	}

	return curr_ma;
}

static int oplus_chg_gauge_get_prev_batt_remaining_capacity(void)
{
	int rc;
	int rm;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		return 0;
	}

	return rm;
}

static int oplus_chg_gauge_get_battery_mvolts_2cell_max(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_get_battery_mvolts_2cell_min(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_get_prev_battery_mvolts_2cell_max(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_get_prev_battery_mvolts_2cell_min(void)
{
	int rc;
	int vol_mv;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_chg_gauge_update_battery_dod0(void)
{
	int rc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0);
	if (rc < 0)
		chg_err("update battery dod0 error, rc=%d\n", rc);

	return 0;
}

static int oplus_chg_gauge_update_soc_smooth_parameter(void)
{
	int rc;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH);
	if (rc < 0)
		chg_err("update soc smooth parameter, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_gauge_get_battery_cb_status(void)
{
	int rc;
	int cb_status;

	if (g_gauge_ic == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_gauge_ic->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_CB_STATUS, &cb_status);
	if (rc < 0) {
		chg_err("update soc smooth parameter, rc=%d\n", rc);
		return 0;
	}

	return cb_status;
}

static int oplus_chg_gauge_get_i2c_err(void)
{
	return 0;
}

static void oplus_chg_gauge_clear_i2c_err(void)
{
}

static int oplus_chg_gauge_get_passedchg(int *val)
{
	*val = 1;

	return 0;
}

static int oplus_chg_gauge_reg_dump(void)
{
	return 0;
}

static struct oplus_gauge_operations oplus_chg_gauge_gauge_ops = {
	.get_battery_mvolts = oplus_chg_gauge_get_battery_mvolts,
	.get_battery_fc = oplus_chg_gauge_get_battery_fc,
	.get_battery_qm = oplus_chg_gauge_get_battery_qm,
	.get_battery_pd = oplus_chg_gauge_get_battery_pd,
	.get_battery_rcu = oplus_chg_gauge_get_battery_rcu,
	.get_battery_rcf = oplus_chg_gauge_get_battery_rcf,
	.get_battery_fcu = oplus_chg_gauge_get_battery_fcu,
	.get_battery_fcf = oplus_chg_gauge_get_battery_fcf,
	.get_battery_sou = oplus_chg_gauge_get_battery_sou,
	.get_battery_do0 = oplus_chg_gauge_get_battery_do0,
	.get_battery_doe = oplus_chg_gauge_get_battery_doe,
	.get_battery_trm = oplus_chg_gauge_get_battery_trm,
	.get_battery_pc = oplus_chg_gauge_get_battery_pc,
	.get_battery_qs = oplus_chg_gauge_get_battery_qs,
	.get_battery_temperature = oplus_chg_gauge_get_battery_temperature,
	.get_batt_remaining_capacity = oplus_chg_gauge_get_batt_remaining_capacity,
	.get_battery_soc = oplus_chg_gauge_get_battery_soc,
	.get_average_current = oplus_chg_gauge_get_average_current,
	.get_battery_fcc = oplus_chg_gauge_get_battery_fcc,
	.get_prev_batt_fcc = oplus_chg_gauge_get_prev_batt_fcc,
	.get_battery_cc = oplus_chg_gauge_get_battery_cc,
	.get_battery_soh = oplus_chg_gauge_get_battery_soh,
	.get_battery_authenticate = oplus_chg_gauge_get_battery_authenticate,
	.get_battery_hmac = oplus_chg_gauge_get_battery_hmac,
	.set_battery_full = oplus_chg_gauge_set_battery_full,
	.get_prev_battery_mvolts = oplus_chg_gauge_get_prev_battery_mvolts,
	.get_prev_battery_temperature = oplus_chg_gauge_get_prev_battery_temperature,
	.get_prev_battery_soc = oplus_chg_gauge_get_prev_battery_soc,
	.get_prev_average_current = oplus_chg_gauge_get_prev_average_current,
	.get_prev_batt_remaining_capacity   = oplus_chg_gauge_get_prev_batt_remaining_capacity,
	.get_battery_mvolts_2cell_max = oplus_chg_gauge_get_battery_mvolts_2cell_max,
	.get_battery_mvolts_2cell_min = oplus_chg_gauge_get_battery_mvolts_2cell_min,
	.get_prev_battery_mvolts_2cell_max = oplus_chg_gauge_get_prev_battery_mvolts_2cell_max,
	.get_prev_battery_mvolts_2cell_min = oplus_chg_gauge_get_prev_battery_mvolts_2cell_min,
	.update_battery_dod0 = oplus_chg_gauge_update_battery_dod0,
	.update_soc_smooth_parameter = oplus_chg_gauge_update_soc_smooth_parameter,
	.get_battery_cb_status = oplus_chg_gauge_get_battery_cb_status,
	.get_gauge_i2c_err = oplus_chg_gauge_get_i2c_err,
	.clear_gauge_i2c_err = oplus_chg_gauge_clear_i2c_err,
	.get_passdchg = oplus_chg_gauge_get_passedchg,
	.dump_register = oplus_chg_gauge_reg_dump,
};

static void oplus_chg_gauge_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_hal_gauge_ic *chip = container_of(dwork,
		struct oplus_hal_gauge_ic, hal_gauge_init_work);
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	struct oplus_gauge_chip *gauge_chip = &chip->gauge_chip;
	int rc;

	chip->gauge_ic = of_get_oplus_chg_ic(node, "oplus,gauge_ic", 0);
	if (chip->gauge_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->hal_gauge_init_work,
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
			schedule_delayed_work(&chip->hal_gauge_init_work,
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
			       &gauge_chip->device_type);
	if (rc < 0) {
		chg_err("can't get device type, rc=%d\n");
		gauge_chip->device_type = 0;
	}
	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC,
			       &gauge_chip->device_type_for_vooc);
	if (rc < 0) {
		chg_err("can't get device type for vooc, rc=%d\n");
		gauge_chip->device_type_for_vooc = 0;
	}

	oplus_chg_gauge_virq_register(chip);
	oplus_gauge_init(gauge_chip);
}

static void oplus_chg_gauge_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
}

static int oplus_chg_gauge_virq_register(struct oplus_hal_gauge_ic *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->gauge_ic, OPLUS_IC_VIRQ_ERR,
		oplus_chg_gauge_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);

	return 0;
}

static int oplus_hal_gauge_probe(struct platform_device *pdev)
{
	struct oplus_hal_gauge_ic *chip;
	struct oplus_gauge_chip *gauge_chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_hal_gauge_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	gauge_chip = &chip->gauge_chip;
	gauge_chip->dev = chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);

	INIT_DELAYED_WORK(&chip->hal_gauge_init_work, oplus_chg_gauge_init_work);
	g_gauge_ic = chip;
	gauge_chip->gauge_ops = &oplus_chg_gauge_gauge_ops;

	schedule_delayed_work(&chip->hal_gauge_init_work, 0);

	chg_err("probe success\n");
	return 0;
}

static int oplus_hal_gauge_remove(struct platform_device *pdev)
{
	struct oplus_hal_gauge_ic *chip = platform_get_drvdata(pdev);

	g_gauge_ic = NULL;
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id oplus_hal_gauge_match[] = {
	{ .compatible = "oplus,hal_gauge" },
	{},
};

static struct platform_driver oplus_hal_gauge_driver = {
	.driver		= {
		.name = "oplus-hal_gauge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_hal_gauge_match),
	},
	.probe		= oplus_hal_gauge_probe,
	.remove		= oplus_hal_gauge_remove,
};

static __init int oplus_hal_gauge_init(void)
{
	return platform_driver_register(&oplus_hal_gauge_driver);
}

static __exit void oplus_hal_gauge_exit(void)
{
	platform_driver_unregister(&oplus_hal_gauge_driver);
}

oplus_chg_module_register(oplus_hal_gauge);
