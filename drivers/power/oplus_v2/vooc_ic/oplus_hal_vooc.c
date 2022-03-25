// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[HAL_VOOC]([%s][%d]): " fmt, __func__, __LINE__

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
#include "../oplus_vooc.h"
#include "../oplus_charger.h"
#include "../oplus_adapter.h"
#include "../oplus_debug_info.h"
#include "oplus_hal_vooc.h"

static const char * const oplus_chg_switch_mode_text[] = {
	[VOOC_SWITCH_MODE_NORMAL] = "normal",
	[VOOC_SWITCH_MODE_VOOC] = "vooc",
	[VOOC_SWITCH_MODE_HEADPHONE] = "headphone",
};

struct oplus_virtual_vooc_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *vooc_ic;

	struct oplus_vooc_chip vooc_chip;

	struct delayed_work delay_reset_mcu_work;
	struct delayed_work virtual_vooc_init_work;
};

static void oplus_vooc_fw_type_dt(struct oplus_vooc_chip *chip,
				  struct device_node *node);
static int oplus_chg_vooc_virq_register(struct oplus_virtual_vooc_ic *chip);
static int oplus_vooc_asic_fw_status(struct oplus_vooc_chip *vooc_chip);

static struct oplus_vooc_chip *g_vooc_chip = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
/* only for GKI compile */
bool __attribute__((weak)) qpnp_is_charger_reboot(void)
{
	return false;
}

bool __attribute__((weak)) qpnp_is_power_off_charging(void)
{
	return false;
}
#endif

static int __oplus_vooc_set_clock_active(struct oplus_virtual_vooc_ic *chip)
{
	struct oplus_chg_ic_dev *vooc_ic = chip->vooc_ic;
	int rc;

	if (vooc_ic == NULL) {
		chg_err("vooc_ic not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_SET_CLOCK_ACTIVE);
	if (rc < 0) {
		chg_err("set clock active error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int __oplus_vooc_set_clock_sleep(struct oplus_virtual_vooc_ic *chip)
{
	struct oplus_chg_ic_dev *vooc_ic = chip->vooc_ic;
	int rc;

	if (vooc_ic == NULL) {
		chg_err("vooc_ic not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_SET_CLOCK_SLEEP);
	if (rc < 0) {
		chg_err("set clock sleep error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int __oplus_vooc_set_reset_active(struct oplus_virtual_vooc_ic *chip)
{
	struct oplus_chg_ic_dev *vooc_ic = chip->vooc_ic;
	int rc;

	if (vooc_ic == NULL) {
		chg_err("vooc_ic not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_RESET_ACTIVE);
	if (rc < 0) {
		chg_err("set reset active error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int __oplus_vooc_set_reset_sleep(struct oplus_virtual_vooc_ic *chip)
{
	struct oplus_chg_ic_dev *vooc_ic = chip->vooc_ic;
	int rc;

	if (vooc_ic == NULL) {
		chg_err("vooc_ic not found\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_RESET_SLEEP);
	if (rc < 0) {
		chg_err("set reset sleep error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_vooc_fw_update(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 1;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_FW_UPGRADE);
	if (rc < 0) {
		chg_err("firmware upgrade error, rc=%d\n", rc);
		return 1;
	}

	return 0;
}

static int oplus_vooc_fw_check_then_recover(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return FW_ERROR_DATA_MODE;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_FW_CHECK_THEN_RECOVER);
	if (rc != FW_CHECK_MODE) {
		chg_err("fw_check_then_recover error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

__maybe_unused static int oplus_vooc_fw_check_then_recover_fix(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return FW_ERROR_DATA_MODE;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_FW_CHECK_THEN_RECOVER_FIX);
	if (rc != FW_CHECK_MODE) {
		chg_err("fw_check_then_recover_fix error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static void oplus_vooc_set_switch_mode(struct oplus_vooc_chip *vooc_chip, int mode)
{
	enum oplus_chg_vooc_switch_mode switch_mode;
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	switch (mode) {
	case VOOC_CHARGER_MODE:
		switch_mode = VOOC_SWITCH_MODE_VOOC;
		break;
	case HEADPHONE_MODE:
		switch_mode = VOOC_SWITCH_MODE_HEADPHONE;
		break;
	case NORMAL_CHARGER_MODE:
	default:
		switch_mode = VOOC_SWITCH_MODE_NORMAL;
		break;
	}

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_SET_SWITCH_MODE, switch_mode);
	if (rc < 0)
		chg_err("switch to %s mode error, rc=%d\n", oplus_chg_switch_mode_text[switch_mode], rc);
	vooc_chip->dpdm_switch_mode = mode;
}

static void oplus_vooc_eint_register(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_EINT_REGISTER);
	if (rc < 0)
		chg_err("eint register error, rc=%d\n", rc);
}

static void oplus_vooc_eint_unregister(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_EINT_UNREGISTER);
	if (rc < 0)
		chg_err("eint unregister error, rc=%d\n", rc);
}

static void oplus_vooc_set_data_active(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_SET_DATA_ACTIVE);
	if (rc < 0)
		chg_err("set data active error, rc=%d\n", rc);
}

static void oplus_vooc_set_data_sleep(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_SET_DATA_SLEEP);
	if (rc < 0)
		chg_err("set data sleep error, rc=%d\n", rc);
}

static void oplus_vooc_set_clock_active(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	rc = __oplus_vooc_set_clock_active(chip);
	if (rc < 0)
		chg_err("set clock active error, rc=%d\n", rc);
}

static void oplus_vooc_set_clock_sleep(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	rc = __oplus_vooc_set_clock_sleep(chip);
	if (rc < 0)
		chg_err("set clock sleep error, rc=%d\n", rc);
}

static int oplus_vooc_get_gpio_ap_data(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int val;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_GET_DATA_GPIO_VAL, &val);
	if (rc < 0) {
		chg_err("get data gpio val error, rc=%d\n", rc);
		return 0;
	}

	return val;
}

static int oplus_vooc_read_ap_data(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	bool bit;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_READ_DATA_BIT, &bit);
	if (rc < 0) {
		chg_err("get data bit error, rc=%d\n", rc);
		return 0;
	}

	return (int)bit;
}

static void oplus_vooc_reply_mcu_data(struct oplus_vooc_chip *vooc_chip, int ret_info, int device_type)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int data;
	int data_width;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	if (vooc_chip->vooc_multistep_adjust_current_support == false) {
		data_width = 3;
	} else {
		if (vooc_chip->vooc_reply_mcu_bits == 4) {
			data_width = 4;
		} else if (vooc_chip->vooc_reply_mcu_bits == 7) {
			data_width = 7;
		} else {
			data_width = 3;
		}
	}

	if (!vooc_chip->w_soc_temp_to_mcu)
		data = (ret_info << 1) | (device_type & BIT(0));
	else
		data = ret_info;
	vooc_chip->w_soc_temp_to_mcu = false;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_REPLY_DATA, data, data_width);
	if (rc < 0)
		chg_err("reply data error, rc=%d\n", rc);
}

static void oplus_vooc_reply_mcu_data_4bits(struct oplus_vooc_chip *vooc_chip, int ret_info, int device_type)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int data;
	int data_width;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	data_width = 4;
	data = (ret_info << 1) | (device_type & BIT(0));

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_REPLY_DATA, data, data_width);
	if (rc < 0)
		chg_err("reply data error, rc=%d\n", rc);
}

static void oplus_vooc_set_reset_active(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	rc = __oplus_vooc_set_reset_active(chip);
	if (rc < 0)
		chg_err("reset active error, rc=%d\n", rc);
}

static void oplus_vooc_set_reset_sleep(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	rc = __oplus_vooc_set_reset_sleep(chip);
	if (rc < 0)
		chg_err("reset sleep error, rc=%d\n", rc);
}

static void reset_fastchg_after_usbout(struct oplus_vooc_chip *vooc_chip)
{
	if (oplus_vooc_get_fastchg_started() == false) {
		chg_info("switch off fastchg\n");
		oplus_vooc_set_fastchg_type_unknow();
		oplus_vooc_set_switch_mode(vooc_chip, NORMAL_CHARGER_MODE);
		if (oplus_vooc_get_fastchg_dummy_started() == false) {
			oplus_vooc_set_reset_sleep(vooc_chip);
		}
	}
	oplus_vooc_set_fastchg_to_normal_false();
	oplus_vooc_set_fastchg_to_warm_false();
	oplus_vooc_set_fastchg_low_temp_full_false();
	oplus_vooc_set_fastchg_dummy_started_false();
}

static bool is_allow_fast_chg_real(struct oplus_vooc_chip *chip)
{
	int temp = 0;
	int cap = 0;
	int chg_type = 0;

	temp = oplus_chg_get_chg_temperature();
	cap = oplus_chg_get_ui_soc();
	chg_type = oplus_chg_get_chg_type();

	if (chg_type != POWER_SUPPLY_TYPE_USB_DCP) {
		return false;
	}
	if (temp < chip->vooc_low_temp) {
		return false;
	}
	if (temp >= chip->vooc_high_temp) {
		return false;
	}
	if (cap < chip->vooc_low_soc) {
		return false;
	}
	if (cap > chip->vooc_high_soc) {
		return false;
	}
	if (oplus_vooc_get_fastchg_to_normal() == true) {
		chg_debug(" oplus_vooc_get_fastchg_to_normal is true\n");
		return false;
	}

	if(chip->disable_real_fast_chg)
		return false;

	return true;
}

static bool is_allow_fast_chg_dummy(struct oplus_vooc_chip *chip)
{
	int chg_type = 0;
	bool allow_real = false;

	chg_type = oplus_chg_get_chg_type();
	if (chg_type != POWER_SUPPLY_TYPE_USB_DCP) {
		chip->reset_adapter = false;
		return false;
	}
	if (oplus_vooc_get_fastchg_to_normal() == true) {
		chip->reset_adapter = false;
		chg_info(" fast_switch_to_noraml is true\n");
		return false;
	}
	allow_real = is_allow_fast_chg_real(chip);
	if (oplus_vooc_get_fastchg_dummy_started() == true && (!allow_real)) {
		chip->reset_adapter = false;
		chg_info(" dummy_started true, allow_real false\n");
		return false;
	}


	if (oplus_vooc_get_fastchg_dummy_started() == true && allow_real && !chip->reset_adapter){
		chip->reset_adapter = true;
		oplus_chg_suspend_charger();
		oplus_chg_set_charger_type_unknown();
		chg_info(" dummy_started true, allow_real true, reset adapter\n");
		return false;
	}
	oplus_vooc_set_fastchg_allow(allow_real);
	return true;
}

static enum oplus_chg_vooc_switch_mode oplus_chg_vooc_get_switch_mode(struct oplus_virtual_vooc_ic *chip)
{
	int mode;
	int rc;

	rc = oplus_chg_ic_func(chip->vooc_ic, OPLUS_IC_FUNC_VOOC_GET_SWITCH_MODE, &mode);
	if (rc < 0) {
		chg_err("get switch mode error, rc=%d\n", rc);
		mode = VOOC_SWITCH_MODE_NORMAL;
	}

	return (enum oplus_chg_vooc_switch_mode)mode;
}

static void oplus_vooc_delay_reset_mcu(struct oplus_virtual_vooc_ic *chip)
{
	schedule_delayed_work(&chip->delay_reset_mcu_work,
		round_jiffies_relative(msecs_to_jiffies(1500)));
}

static void set_vooc_chargerid_switch_val(struct oplus_vooc_chip *vooc_chip, int value)
{
	int level = 0;

	if (value == 1)
		level = 1;
	else if (value == 0)
		level = 0;
	else
		return;
}

static void switch_fast_chg(struct oplus_vooc_chip *vooc_chip)
{
	bool allow_real = false;
	struct oplus_virtual_vooc_ic *chip;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	if (vooc_chip->dpdm_switch_mode == VOOC_CHARGER_MODE &&
	    oplus_chg_vooc_get_switch_mode(chip) == VOOC_SWITCH_MODE_VOOC) {
		if (!oplus_vooc_get_fastchg_started()) {
			allow_real = is_allow_fast_chg_real(vooc_chip);
			oplus_vooc_set_fastchg_allow(allow_real);
		}
		return;
	}

	if (is_allow_fast_chg_dummy(vooc_chip)) {
		if (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_UPDATE_FAIL) {
			oplus_vooc_delay_reset_mcu(chip);
			oplus_vooc_set_switch_mode(vooc_chip, VOOC_CHARGER_MODE);
		} else {
			if (!oplus_vooc_get_fastchg_allow() &&
			    oplus_vooc_get_fastchg_to_warm()) {
				chg_info(" fastchg_allow false, to_warm true, don't switch to vooc mode\n");
			} else {
				oplus_vooc_set_clock_sleep(vooc_chip);
				oplus_vooc_set_reset_active(vooc_chip);
				oplus_vooc_set_switch_mode(vooc_chip, VOOC_CHARGER_MODE);

				if(!opchg_get_mcu_update_state() &&
				   oplus_vooc_asic_fw_status(vooc_chip) == 0) {
					chg_err("check fw fail, go to update fw!\n");
					set_vooc_chargerid_switch_val(vooc_chip, 0);
					oplus_vooc_set_switch_mode(vooc_chip, NORMAL_CHARGER_MODE);
					oplus_chg_clear_chargerid_info();
					//oplus_vooc_fw_update_work_plug_in();
				}
			}
		}
	}
	chg_info(" end, allow_fast_chg:%d\n", oplus_vooc_get_fastchg_allow());
}

bool oplus_is_power_off_charging(struct oplus_vooc_chip *vooc_chip)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		return true;
	} else {
		return false;
	}
#else
	return qpnp_is_power_off_charging();
#endif
}

bool oplus_is_charger_reboot(struct oplus_vooc_chip *vooc_chip)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	int charger_type;

	charger_type = oplus_chg_get_chg_type();
	if (charger_type == 5) {
		chg_debug("dont need check fw_update\n");
		return true;
	} else {
		return false;
	}
#else
	return qpnp_is_charger_reboot();
#endif
}

static int oplus_vooc_get_reset_gpio_val(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int val;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_GET_RESET_GPIO_VAL, &val);
	if (rc < 0) {
		chg_err("get reset gpio val error, rc=%d\n", rc);
		return 0;
	}

	return val;
}

static int oplus_vooc_get_ap_clk_gpio_val(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	int val;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_GET_CLOCK_GPIO_VAL, &val);
	if (rc < 0) {
		chg_err("get clk gpio val error, rc=%d\n", rc);
		return 0;
	}

	return val;
}

static int oplus_vooc_get_switch_gpio_val(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	int val;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);

	switch (oplus_chg_vooc_get_switch_mode(chip)) {
	case VOOC_SWITCH_MODE_NORMAL:
		val = 0;
		break;
	case VOOC_SWITCH_MODE_VOOC:
		val = 1;
		break;
	case VOOC_SWITCH_MODE_HEADPHONE:
		val = 1;
		break;
	}

	return val;
}

static int oplus_vooc_get_fw_verion_from_ic(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	u32 version;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_GET_FW_VERSION, &version);
	if (rc < 0) {
		chg_err("get firmware version error, rc=%d\n", rc);
		return 0;
	}

	return (int)version;
}

static void oplus_vooc_update_temperature_soc(void)
{
}

__maybe_unused static int oplus_vooc_asic_fw_status(struct oplus_vooc_chip *vooc_chip)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_chg_ic_dev *vooc_ic;
	bool pass;
	int rc;

	if (vooc_chip == NULL) {
		chg_err("vooc_chip is NULL\n");
		return 0;
	}
	chip = container_of(vooc_chip, struct oplus_virtual_vooc_ic, vooc_chip);
	vooc_ic = chip->vooc_ic;

	rc = oplus_chg_ic_func(vooc_ic, OPLUS_IC_FUNC_VOOC_CHECK_FW_STATUS, &pass);
	if (rc < 0) {
		chg_err("get firmware status error, rc=%d\n", rc);
		return 0;
	}

	return (int)pass;
}

struct oplus_vooc_operations oplus_chg_vooc_ops = {
	.fw_update = oplus_vooc_fw_update,
	.fw_check_then_recover = oplus_vooc_fw_check_then_recover,
	//.fw_check_then_recover_fix = oplus_vooc_fw_check_then_recover_fix,
	.set_switch_mode = oplus_vooc_set_switch_mode,
	.eint_regist = oplus_vooc_eint_register,
	.eint_unregist = oplus_vooc_eint_unregister,
	.set_data_active = oplus_vooc_set_data_active,
	.set_data_sleep = oplus_vooc_set_data_sleep,
	.set_clock_active = oplus_vooc_set_clock_active,
	.set_clock_sleep = oplus_vooc_set_clock_sleep,
	.get_gpio_ap_data = oplus_vooc_get_gpio_ap_data,
	.read_ap_data = oplus_vooc_read_ap_data,
	.reply_mcu_data = oplus_vooc_reply_mcu_data,
	.reply_mcu_data_4bits = oplus_vooc_reply_mcu_data_4bits,
	.reset_fastchg_after_usbout = reset_fastchg_after_usbout,
	.switch_fast_chg = switch_fast_chg,
	.reset_mcu = oplus_vooc_set_reset_active,
	.set_mcu_sleep = oplus_vooc_set_reset_sleep,
	.set_vooc_chargerid_switch_val = set_vooc_chargerid_switch_val,
	.is_power_off_charging = oplus_is_power_off_charging,
	.get_reset_gpio_val = oplus_vooc_get_reset_gpio_val,
	.get_switch_gpio_val = oplus_vooc_get_switch_gpio_val,
	.get_ap_clk_gpio_val = oplus_vooc_get_ap_clk_gpio_val,
	.get_fw_version = oplus_vooc_get_fw_verion_from_ic,
	.get_clk_gpio_num = NULL,
	.get_data_gpio_num = NULL,
	.update_temperature_soc = oplus_vooc_update_temperature_soc,
	//.check_asic_fw_status = oplus_vooc_asic_fw_status,
};

int get_vooc_mcu_type(struct oplus_vooc_chip *chip)
{
	int mcu_hwid_type = OPLUS_VOOC_MCU_HWID_UNKNOW;

	if(chip == NULL){
		chg_err("chip is NULL\n");
		return OPLUS_VOOC_ASIC_HWID_RK826;
	}
	mcu_hwid_type = OPLUS_VOOC_ASIC_HWID_RK826;

	return mcu_hwid_type;
}

bool oplus_vooc_btb_temp_over(void)
{
	struct oplus_vooc_chip *chip = g_vooc_chip;

	if(chip == NULL){
		chg_err("chip is NULL\n");
		return false;
	}

	return chip->btb_temp_over;
}

static void delay_reset_mcu_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_vooc_ic *chip = container_of(dwork,
		struct oplus_virtual_vooc_ic, delay_reset_mcu_work);

	__oplus_vooc_set_clock_sleep(chip);
	__oplus_vooc_set_reset_active(chip);
}

static void oplus_chg_vooc_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_vooc_ic *chip = container_of(dwork,
		struct oplus_virtual_vooc_ic, virtual_vooc_init_work);
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	struct oplus_chg_ic_dev *real_vooc_ic = NULL;
	int rc;

	chip->vooc_ic = of_get_oplus_chg_ic(node, "oplus,vooc_ic", 0);
	if (chip->vooc_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->virtual_vooc_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,vooc_ic not found\n");
		}
		retry = 0;
		return;
	}

	rc = oplus_chg_ic_func(chip->vooc_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->virtual_vooc_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("vooc_ic init timeout\n");
		}
		retry = 0;
		return;
	} else if (rc < 0) {
		chg_err("vooc_ic init error, rc=%d\n", rc);
		retry = 0;
		return;
	}
	retry = 0;

	chip->vooc_chip.vops = &oplus_chg_vooc_ops;
	chip->vooc_chip.fw_mcu_version = 0;

	rc = oplus_chg_ic_func(chip->vooc_ic, OPLUS_IC_FUNC_VOOC_GET_IC_DEV, &real_vooc_ic);
	if (rc < 0) {
		chg_err("get real vooc ic error, rc=%d\n", rc);
		return;
	}
	if (real_vooc_ic == NULL) {
		chg_err("real vooc ic not found\n");
		return;
	}
	oplus_vooc_fw_type_dt(&chip->vooc_chip, real_vooc_ic->dev->of_node);
	oplus_vooc_fw_update_work_init(&chip->vooc_chip);
	oplus_vooc_init(&chip->vooc_chip);
	oplus_chg_vooc_virq_register(chip);
}

static void oplus_chg_vooc_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	//oplus_chg_asic_err();
}

static void oplus_chg_vooc_data_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	oplus_vooc_shedule_fastchg_work();
}

static void oplus_vooc_fw_type_dt(struct oplus_vooc_chip *chip,
				  struct device_node *node)
{
	int loop;
	int rc;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return;
	}

	chip->vooc_current_lvl_cnt = of_property_count_elems_of_size(node,
				"qcom,vooc_current_lvl", sizeof(*chip->vooc_current_lvl));
	if (chip->vooc_current_lvl_cnt > 0) {
		chg_err("vooc_current_lvl_cnt[%d]\n", chip->vooc_current_lvl_cnt);
		chip->vooc_current_lvl = devm_kcalloc(chip->dev, chip->vooc_current_lvl_cnt,
			sizeof(*chip->vooc_current_lvl), GFP_KERNEL);
		if (!chip->vooc_current_lvl){
			chg_err("devm_kcalloc vooc_current_lvl error\n");
			return;
		}
		rc = of_property_read_u32_array(node,
			"qcom,vooc_current_lvl", chip->vooc_current_lvl, chip->vooc_current_lvl_cnt);
		if (rc) {
			chg_err("qcom,cp_ffc_current_lvl error\n");
			return;
		}

		for(loop = 0; loop < chip->vooc_current_lvl_cnt; loop++) {
			chg_err("vooc_current_lvl[%d]\n", chip->vooc_current_lvl[loop]);
		}
	}
	chip->batt_type_4400mv = of_property_read_bool(node, "qcom,oplus_batt_4400mv");
	chip->support_vooc_by_normal_charger_path = of_property_read_bool(node,
		"qcom,support_vooc_by_normal_charger_path");
	chip->hiz_gnd_cable = of_property_read_bool(node, "qcom,supported_hiz_gnd_cable");

	chg_debug("oplus_vooc_fw_type_dt batt_type_4400 is %d,vooc_fw_type = 0x%x\n",
		chip->batt_type_4400mv, chip->vooc_fw_type);

	chip->vooc_fw_update_newmethod = of_property_read_bool(node,
		"qcom,vooc_fw_update_newmethod");
	chg_debug(" vooc_fw_upate:%d\n", chip->vooc_fw_update_newmethod);

	rc = of_property_read_u32(node, "qcom,vooc-low-temp",
		&chip->vooc_low_temp);
	if (rc) {
		chip->vooc_low_temp = 165;
	} else {
		chg_debug("qcom,vooc-low-temp is %d\n", chip->vooc_low_temp);
	}

	chip->vooc_batt_over_low_temp = chip->vooc_low_temp - 5;

	rc = of_property_read_u32(node, "qcom,vooc-little-cool-temp",
			&chip->vooc_little_cool_temp);
	if (rc) {
		chip->vooc_little_cool_temp = 160;
	} else {
		chg_debug("qcom,vooc-little-cool-temp is %d\n", chip->vooc_little_cool_temp);
	}
	chip->vooc_little_cool_temp_default = chip->vooc_little_cool_temp;

	rc = of_property_read_u32(node, "qcom,vooc-cool-temp",
			&chip->vooc_cool_temp);
	if (rc) {
		chip->vooc_cool_temp = 120;
	} else {
		chg_debug("qcom,vooc-cool-temp is %d\n", chip->vooc_cool_temp);
	}
	chip->vooc_cool_temp_default = chip->vooc_cool_temp;

	rc = of_property_read_u32(node, "qcom,vooc-little-cold-temp",
			&chip->vooc_little_cold_temp);
	if (rc) {
		chip->vooc_little_cold_temp = 50;
	} else {
		chg_debug("qcom,vooc-little-cold-temp is %d\n", chip->vooc_little_cold_temp);
	}
	chip->vooc_little_cold_temp_default = chip->vooc_little_cold_temp;


	rc = of_property_read_u32(node, "qcom,vooc-normal-low-temp",
			&chip->vooc_normal_low_temp);
	if (rc) {
		chip->vooc_normal_low_temp = 250;
	} else {
		chg_debug("qcom,vooc-normal-low-temp is %d\n", chip->vooc_normal_low_temp);
	}
	chip->vooc_normal_low_temp_default = chip->vooc_normal_low_temp;


	rc = of_property_read_u32(node, "qcom,vooc-high-temp", &chip->vooc_high_temp);
	if (rc) {
		chip->vooc_high_temp = 430;
	} else {
		chg_debug("qcom,vooc-high-temp is %d\n", chip->vooc_high_temp);
	}

	rc = of_property_read_u32(node, "qcom,vooc-low-soc", &chip->vooc_low_soc);
	if (rc) {
		chip->vooc_low_soc = 1;
	} else {
		chg_debug("qcom,vooc-low-soc is %d\n", chip->vooc_low_soc);
	}

	rc = of_property_read_u32(node, "qcom,vooc-high-soc", &chip->vooc_high_soc);
	if (rc) {
		chip->vooc_high_soc = 85;
	} else {
		chg_debug("qcom,vooc-high-soc is %d\n", chip->vooc_high_soc);
	}
	chip->vooc_multistep_adjust_current_support = of_property_read_bool(node,
		"qcom,vooc_multistep_adjust_current_support");
	chg_debug("qcom,vooc_multistep_adjust_current_supportis %d\n",
		chip->vooc_multistep_adjust_current_support);

	rc = of_property_read_u32(node, "qcom,vooc_reply_mcu_bits",
		&chip->vooc_reply_mcu_bits);
	if (rc) {
		chip->vooc_reply_mcu_bits = 4;
	} else {
		chg_debug("qcom,vooc_reply_mcu_bits is %d\n",
			chip->vooc_reply_mcu_bits);
	}

	rc = of_property_read_u32(node, "qcom,vooc_multistep_initial_batt_temp",
		&chip->vooc_multistep_initial_batt_temp);
	if (rc) {
		chip->vooc_multistep_initial_batt_temp = 305;
	} else {
		chg_debug("qcom,vooc_multistep_initial_batt_temp is %d\n",
			chip->vooc_multistep_initial_batt_temp);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy_normal_current",
		&chip->vooc_strategy_normal_current);
	if (rc) {
		chip->vooc_strategy_normal_current = 0x03;
	} else {
		chg_debug("qcom,vooc_strategy_normal_current is %d\n",
			chip->vooc_strategy_normal_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp1",
		&chip->vooc_strategy1_batt_low_temp1);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp1  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_low_temp1 is %d\n",
			chip->vooc_strategy1_batt_low_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp2",
		&chip->vooc_strategy1_batt_low_temp2);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp2 = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_low_temp2 is %d\n",
			chip->vooc_strategy1_batt_low_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp0",
		&chip->vooc_strategy1_batt_low_temp0);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp0 = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_low_temp0 is %d\n",
			chip->vooc_strategy1_batt_low_temp0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp0",
		&chip->vooc_strategy1_batt_high_temp0);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp0 = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_high_temp0 is %d\n",
			chip->vooc_strategy1_batt_high_temp0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp1",
		&chip->vooc_strategy1_batt_high_temp1);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp1 = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_high_temp1 is %d\n",
			chip->vooc_strategy1_batt_high_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp2",
		&chip->vooc_strategy1_batt_high_temp2);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp2 = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy1_batt_high_temp2 is %d\n",
			chip->vooc_strategy1_batt_high_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current0",
		&chip->vooc_strategy1_high_current0);
	if (rc) {
		chip->vooc_strategy1_high_current0  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_high_current0 is %d\n",
			chip->vooc_strategy1_high_current0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current1",
		&chip->vooc_strategy1_high_current1);
	if (rc) {
		chip->vooc_strategy1_high_current1  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_high_current1 is %d\n",
			chip->vooc_strategy1_high_current1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current2",
		&chip->vooc_strategy1_high_current2);
	if (rc) {
		chip->vooc_strategy1_high_current2  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_high_current2 is %d\n",
			chip->vooc_strategy1_high_current2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current2",
		&chip->vooc_strategy1_low_current2);
	if (rc) {
		chip->vooc_strategy1_low_current2  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_low_current2 is %d\n",
			chip->vooc_strategy1_low_current2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current1",
		&chip->vooc_strategy1_low_current1);
	if (rc) {
		chip->vooc_strategy1_low_current1  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_low_current1 is %d\n",
			chip->vooc_strategy1_low_current1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current0",
		&chip->vooc_strategy1_low_current0);
	if (rc) {
		chip->vooc_strategy1_low_current0  = chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy1_low_current0 is %d\n",
			chip->vooc_strategy1_low_current0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp1",
		&chip->vooc_strategy2_batt_up_temp1);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp1  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_temp1 is %d\n",
			chip->vooc_strategy2_batt_up_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_down_temp2",
		&chip->vooc_strategy2_batt_up_down_temp2);
	if (rc) {
		chip->vooc_strategy2_batt_up_down_temp2  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_down_temp2 is %d\n",
			chip->vooc_strategy2_batt_up_down_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp3",
		&chip->vooc_strategy2_batt_up_temp3);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp3  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_temp3 is %d\n",
			chip->vooc_strategy2_batt_up_temp3);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_down_temp4",
		&chip->vooc_strategy2_batt_up_down_temp4);
	if (rc) {
		chip->vooc_strategy2_batt_up_down_temp4  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_down_temp4 is %d\n",
			chip->vooc_strategy2_batt_up_down_temp4);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp5",
		&chip->vooc_strategy2_batt_up_temp5);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp5  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_temp5 is %d\n",
			chip->vooc_strategy2_batt_up_temp5);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp6",
		&chip->vooc_strategy2_batt_up_temp6);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp6  = chip->vooc_multistep_initial_batt_temp;
	} else {
		chg_debug("qcom,vooc_strategy2_batt_up_temp6 is %d\n",
			chip->vooc_strategy2_batt_up_temp6);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high0_current",
		&chip->vooc_strategy2_high0_current);
	if (rc) {
		chip->vooc_strategy2_high0_current	= chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy2_high0_current is %d\n",
			chip->vooc_strategy2_high0_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high1_current",
		&chip->vooc_strategy2_high1_current);
	if (rc) {
		chip->vooc_strategy2_high1_current	= chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy2_high1_current is %d\n",
			chip->vooc_strategy2_high1_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high2_current",
		&chip->vooc_strategy2_high2_current);
	if (rc) {
		chip->vooc_strategy2_high2_current	= chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy2_high2_current is %d\n",
			chip->vooc_strategy2_high2_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high3_current",
		&chip->vooc_strategy2_high3_current);
	if (rc) {
		chip->vooc_strategy2_high3_current	= chip->vooc_strategy_normal_current;
	} else {
		chg_debug("qcom,vooc_strategy2_high3_current is %d\n",
			chip->vooc_strategy2_high3_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_batt_over_high_temp",
		&chip->vooc_batt_over_high_temp);
	if (rc) {
		chip->vooc_batt_over_high_temp = -EINVAL;
	} else {
		chg_debug("qcom,vooc_batt_over_high_temp is %d\n",
			chip->vooc_batt_over_high_temp);
	}

	rc = of_property_read_u32(node, "qcom,vooc_over_high_or_low_current",
		&chip->vooc_over_high_or_low_current);
	if (rc) {
		chip->vooc_over_high_or_low_current = -EINVAL;
	} else {
		chg_debug("qcom,vooc_over_high_or_low_current is %d\n",
			chip->vooc_over_high_or_low_current);
	}
}

static int oplus_chg_vooc_virq_register(struct oplus_virtual_vooc_ic *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->vooc_ic, OPLUS_IC_VIRQ_ERR,
		oplus_chg_vooc_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->vooc_ic, OPLUS_IC_VIRQ_VOOC_DATA,
		oplus_chg_vooc_data_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_VOOC_DATA error, rc=%d", rc);

	return 0;
}

static int oplus_virtual_vooc_probe(struct platform_device *pdev)
{
	struct oplus_virtual_vooc_ic *chip;
	struct oplus_vooc_chip *vooc_chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_vooc_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	g_vooc_chip = vooc_chip = &chip->vooc_chip;
	vooc_chip->dev = chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);

	vooc_chip->pcb_version = 0;
	vooc_chip->vooc_fw_check = false;

	__oplus_vooc_set_clock_sleep(chip);
	INIT_DELAYED_WORK(&chip->delay_reset_mcu_work, delay_reset_mcu_work_func);
	INIT_DELAYED_WORK(&chip->virtual_vooc_init_work, oplus_chg_vooc_init_work);
	oplus_vooc_early_init(vooc_chip);

	schedule_delayed_work(&chip->virtual_vooc_init_work, 0);

	chg_err("probe success\n");
	return 0;
}

static int oplus_virtual_vooc_remove(struct platform_device *pdev)
{
	struct oplus_virtual_vooc_ic *chip = platform_get_drvdata(pdev);

	g_vooc_chip = NULL;
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id oplus_virtual_vooc_match[] = {
	{ .compatible = "oplus,hal_vooc" },
	{},
};

static struct platform_driver oplus_virtual_vooc_driver = {
	.driver		= {
		.name = "oplus-hal_vooc",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_vooc_match),
	},
	.probe		= oplus_virtual_vooc_probe,
	.remove		= oplus_virtual_vooc_remove,
};

static __init int oplus_virtual_vooc_init(void)
{
	return platform_driver_register(&oplus_virtual_vooc_driver);
}

static __exit void oplus_virtual_vooc_exit(void)
{
	platform_driver_unregister(&oplus_virtual_vooc_driver);
}

oplus_chg_module_register(oplus_virtual_vooc);
