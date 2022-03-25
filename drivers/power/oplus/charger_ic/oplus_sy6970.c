/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[sy6970]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
//#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <mt-plat/upmu_common.h>
#ifndef CONFIG_CHARGER_SY6970
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#endif /* CONFIG_CHARGER_SY6970 */
#include <mt-plat/mtk_boot.h>

//#include "../../mediatek/charger/mtk_charger_intf.h"
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include <charger_class.h>
#include <mtk_pd.h>
#define _BQ25890H_
#include "oplus_sy6970.h"
#include <linux/time.h>

extern void oplus_wake_up_usbtemp_thread(void);
extern void oplus_set_typec_sinkonly(void);
extern bool oplus_usbtemp_condition(void);
extern void oplus_get_usbtemp_burn_volt(struct oplus_chg_chip *chip);
extern struct charger_consumer *charger_manager_get_by_name(
		struct device *dev,	const char *name);
extern void set_charger_ic(int sel);
extern int mt6357_get_vbus_voltage(void);
extern bool mt6357_get_vbus_status(void);
extern int oplus_battery_meter_get_battery_voltage(void);
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern int set_rtc_spare_fg_value(int val);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

/*charger current limit*/
#define BQ_CHARGER_CURRENT_MAX_MA		3400
#define BQ_CHARGER_CURRENT_CDP_MA		1850
/*input current limit*/
#define BQ_INPUT_CURRENT_MAX_MA 		2000
#define BQ_INPUT_CURRENT_COLD_TEMP_MA		1000
#define BQ_INPUT_CURRENT_NORMAL_TEMP_MA 	2000
#define BQ_INPUT_CURRENT_WARM_TEMP_MA		1800
#define BQ_INPUT_CURRENT_WARM_TEMP_HVDCP_MA	1500
#define BQ_INPUT_CURRENT_HOT_TEMP_MA		1500
#define BQ_INPUT_CURRENT_HOT_TEMP_HVDCP_MA	1200

#define BQ_COLD_TEMPERATURE_DECIDEGC	0
#define BQ_WARM_TEMPERATURE_DECIDEGC	340
#define BQ_HOT_TEMPERATURE_DECIDEGC	    370

struct irq_gpio_pinctrl {
	int irq_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *irq_active;
	struct pinctrl_state *irq_sleep;
};

enum {
	PN_BQ25890H,
	PN_BQ25892,
	PN_BQ25895,
};

static int pn_data[] = {
	[PN_BQ25890H] = 0x03,
	[PN_BQ25892] = 0x00,
	[PN_BQ25895] = 0x07,
};

static char *pn_str[] = {
	[PN_BQ25890H] = "bq25890H",
	[PN_BQ25892] = "bq25892",
	[PN_BQ25895] = "bq25895",
};

enum hvdcp_type {
	HVDCP_5V,
	HVDCP_9V,
	HVDCP_12V,
	HVDCP_20V,
	HVDCP_CONTINOUS,
	HVDCP_DPF_DMF,
};

struct chg_para{
	int vlim;
	int ilim;

	int vreg;
	int ichg;
};

struct sy6970_platform_data {
	int iprechg;
	int iterm;

	int boostv;
	int boosti;

	struct chg_para usb;
};


struct sy6970 {
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work sy6970_aicr_setting_work;
	struct delayed_work sy6970_retry_adapter_detection;
	struct delayed_work sy6970_current_setting_work;
	struct delayed_work init_work;

	int part_no;
	int revision;

	const char *chg_dev_name;
	const char *eint_name;

	bool chg_det_enable;
	bool otg_enable;

	enum charger_type chg_type;
	enum power_supply_type oplus_chg_type;
	
	int status;
	int irq;

	struct mutex i2c_rw_lock;
	struct mutex chgdet_en_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;

	struct sy6970_platform_data *platform_data;
	struct charger_device *chg_dev;
	struct timespec ptime[2];

	struct power_supply *psy;
	struct power_supply *mtk_master_chg;
	struct charger_consumer *chg_consumer;
	bool disable_hight_vbus;
	bool pdqc_setup_5v;
	bool hvdcp_can_enabled;
	int pre_current_ma;
	int aicr;
	int chg_cur;
	int vbus_type;
	bool hvdcp_checked;
	int hw_aicl_point;
	bool retry_hvdcp_algo;
	bool nonstand_retry_bc;
	bool is_bq2589x;
	bool is_sy6970;
	struct power_supply_desc psy_desc;
	struct mutex attach_lock;
	struct irq_gpio_pinctrl irq_gpio_pinctrl;
	bool psy_online;
};

static bool disable_PE = 0;
static bool disable_QC = 0;
static bool disable_PD = 0;
static bool dumpreg_by_irq = 0;
static int  current_percent = 70;
module_param(disable_PE, bool, 0644);
module_param(disable_QC, bool, 0644);
module_param(disable_PD, bool, 0644);
module_param(current_percent, int, 0644);
module_param(dumpreg_by_irq, bool, 0644);
static struct sy6970 *g_bq;

static bool sy6970_is_usb(struct sy6970 *bq);
void oplus_wake_up_usbtemp_thread(void);

extern struct oplus_chg_chip *g_oplus_chip;

extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
void oplus_sy6970_set_mivr_by_battery_vol(void);
static void sy6970_dump_regs(struct sy6970 *bq);

static const struct charger_properties sy6970_chg_props = {
	.alias_name = "sy6970",
};

enum bq2589x_charging_status {
	BQ2589X_CHG_STATUS_NOT_CHARGING = 0,
	BQ2589X_CHG_STATUS_FAST_CHARGING,
	BQ2589X_CHG_STATUS_PRE_CHARGING,
	BQ2589X_CHG_STATUS_DONE,
	BQ2589X_CHG_STATUS_MAX,
};

static enum power_supply_usb_type bq2589x_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static int g_sy6970_read_reg(struct sy6970 *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int g_sy6970_write_reg(struct sy6970 *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int sy6970_read_byte(struct sy6970 *bq, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = g_sy6970_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int sy6970_write_byte(struct sy6970 *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = g_sy6970_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int sy6970_update_bits(struct sy6970 *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = g_sy6970_read_reg(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = g_sy6970_write_reg(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int sy6970_enable_otg(struct sy6970 *bq)
{
	u8 val = SY6970_OTG_ENABLE << SY6970_OTG_CONFIG_SHIFT;

	sy6970_update_bits(bq, SY6970_REG_03,
				   SY6970_OTG_CONFIG_MASK, val);

	if(g_oplus_chip->is_double_charger_support) {
		msleep(50);
		g_oplus_chip->sub_chg_ops->charger_suspend();
	}

	return 0;
}

static int sy6970_disable_otg(struct sy6970 *bq)
{
	u8 val = SY6970_OTG_DISABLE << SY6970_OTG_CONFIG_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_03,
				   SY6970_OTG_CONFIG_MASK, val);
}

static int sy6970_enable_hvdcp(struct sy6970 *bq)
{/*
	int ret;
	u8 val = SY6970_HVDCP_ENABLE << SY6970_HVDCPEN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
				SY6970_HVDCPEN_MASK, val);
	return ret;
*/
	return 0;
}
EXPORT_SYMBOL_GPL(sy6970_enable_hvdcp);

static int sy6970_disable_hvdcp(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_HVDCP_DISABLE << SY6970_HVDCPEN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
				SY6970_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(sy6970_disable_hvdcp);

static int sy6970_disable_maxc(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_MAXC_DISABLE << SY6970_MAXCEN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
				SY6970_MAXCEN_MASK, val);
	return ret;
}

static int sy6970_disable_batfet_rst(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_BATFET_RST_EN_DISABLE << SY6970_BATFET_RST_EN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_09, 
				SY6970_BATFET_RST_EN_MASK, val);
	return ret;
}

static int sy6970_disable_ico(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_ICO_DISABLE << SY6970_ICOEN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
				SY6970_ICOEN_MASK, val);
	return ret;
}
static int sy6970_enable_charger(struct sy6970 *bq)
{
	int ret;

	u8 val = SY6970_CHG_ENABLE << SY6970_CHG_CONFIG_SHIFT;

	dev_info(bq->dev, "%s\n", __func__);
	ret = sy6970_update_bits(bq, SY6970_REG_03, 
				SY6970_CHG_CONFIG_MASK, val);
	return ret;
}

static int sy6970_disable_charger(struct sy6970 *bq)
{
	int ret;

	u8 val = SY6970_CHG_DISABLE << SY6970_CHG_CONFIG_SHIFT;
	
	dev_info(bq->dev, "%s\n", __func__);
	ret = sy6970_update_bits(bq, SY6970_REG_03, 
				SY6970_CHG_CONFIG_MASK, val);
	return ret;
}

int sy6970_adc_start(struct sy6970 *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_02, &val);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & SY6970_CONV_RATE_MASK) >> SY6970_CONV_RATE_SHIFT) == SY6970_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = sy6970_update_bits(bq, SY6970_REG_02, SY6970_CONV_START_MASK,
					SY6970_CONV_START << SY6970_CONV_START_SHIFT);
	else
		ret = sy6970_update_bits(bq, SY6970_REG_02, SY6970_CONV_RATE_MASK,  
					SY6970_ADC_CONTINUE_ENABLE << SY6970_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(sy6970_adc_start);

int sy6970_adc_stop(struct sy6970 *bq)
{
	return sy6970_update_bits(bq, SY6970_REG_02, SY6970_CONV_RATE_MASK, 
				SY6970_ADC_CONTINUE_DISABLE << SY6970_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(sy6970_adc_stop);


int sy6970_adc_read_battery_volt(struct sy6970 *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = sy6970_read_byte(bq, SY6970_REG_0E, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = SY6970_BATV_BASE + ((val & SY6970_BATV_MASK) >> SY6970_BATV_SHIFT) * SY6970_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(sy6970_adc_read_battery_volt);


int sy6970_adc_read_sys_volt(struct sy6970 *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = sy6970_read_byte(bq, SY6970_REG_0F, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = SY6970_SYSV_BASE + ((val & SY6970_SYSV_MASK) >> SY6970_SYSV_SHIFT) * SY6970_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(sy6970_adc_read_sys_volt);

int sy6970_adc_read_vbus_volt(struct sy6970 *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = sy6970_read_byte(bq, SY6970_REG_11, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = SY6970_VBUSV_BASE + ((val & SY6970_VBUSV_MASK) >> SY6970_VBUSV_SHIFT) * SY6970_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(sy6970_adc_read_vbus_volt);

int sy6970_get_vbus_adc(struct charger_device *chg_dev, u32* vbus)
{
	int vol = 0;
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	vol = sy6970_adc_read_vbus_volt(bq);
	*vbus = vol;

	return 0;
}

int sy6970_adc_read_temperature(struct sy6970 *bq)
{
	uint8_t val;
	int temp;
	int ret;
	ret = sy6970_read_byte(bq, SY6970_REG_10, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return ret;
	} else{
		temp = SY6970_TSPCT_BASE + ((val & SY6970_TSPCT_MASK) >> SY6970_TSPCT_SHIFT) * SY6970_TSPCT_LSB ;
		return temp;
	}
}
EXPORT_SYMBOL_GPL(sy6970_adc_read_temperature);

int sy6970_adc_read_charge_current(struct sy6970 *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = sy6970_read_byte(bq, SY6970_REG_12, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(SY6970_ICHGR_BASE + ((val & SY6970_ICHGR_MASK) >> SY6970_ICHGR_SHIFT) * SY6970_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(sy6970_adc_read_charge_current);
int sy6970_set_chargecurrent(struct sy6970 *bq, int curr)
{
	u8 ichg;
	
	dev_info(bq->dev, "%s: ichg = %d\n", __func__, curr);
		
	if (curr < SY6970_ICHG_BASE)
		curr = SY6970_ICHG_BASE;

	ichg = (curr - SY6970_ICHG_BASE)/SY6970_ICHG_LSB;
	return sy6970_update_bits(bq, SY6970_REG_04, 
						SY6970_ICHG_MASK, ichg << SY6970_ICHG_SHIFT);

}

int sy6970_set_term_current(struct sy6970 *bq, int curr)
{
	u8 iterm;

	if (curr < SY6970_ITERM_BASE)
		curr = SY6970_ITERM_BASE;

	iterm = (curr - SY6970_ITERM_BASE) / SY6970_ITERM_LSB;

	return sy6970_update_bits(bq, SY6970_REG_05, 
						SY6970_ITERM_MASK, iterm << SY6970_ITERM_SHIFT);

}
EXPORT_SYMBOL_GPL(sy6970_set_term_current);

int sy6970_set_prechg_current(struct sy6970 *bq, int curr)
{
	u8 iprechg;

	if (curr < SY6970_IPRECHG_BASE)
		curr = SY6970_IPRECHG_BASE;

	iprechg = (curr - SY6970_IPRECHG_BASE) / SY6970_IPRECHG_LSB;

	return sy6970_update_bits(bq, SY6970_REG_05, 
						SY6970_IPRECHG_MASK, iprechg << SY6970_IPRECHG_SHIFT);

}
EXPORT_SYMBOL_GPL(sy6970_set_prechg_current);

int sy6970_set_chargevolt(struct sy6970 *bq, int volt)
{
	u8 val;
	
	dev_info(bq->dev, "%s: volt = %d\n", __func__, volt);

	if (volt < SY6970_VREG_BASE)
		volt = SY6970_VREG_BASE;

	val = (volt - SY6970_VREG_BASE)/SY6970_VREG_LSB;
	return sy6970_update_bits(bq, SY6970_REG_06, 
						SY6970_VREG_MASK, val << SY6970_VREG_SHIFT);
}

int sy6970_set_input_volt_limit(struct sy6970 *bq, int volt)
{
	u8 val;
	
	dev_info(bq->dev, "%s: volt = %d\n", __func__, volt);
	
	if (volt < SY6970_VINDPM_BASE)
		volt = SY6970_VINDPM_BASE;

	val = (volt - SY6970_VINDPM_BASE) / SY6970_VINDPM_LSB;
	
	sy6970_update_bits(bq, SY6970_REG_0D, 
						SY6970_FORCE_VINDPM_MASK, SY6970_FORCE_VINDPM_ENABLE << SY6970_FORCE_VINDPM_SHIFT);
						
	return sy6970_update_bits(bq, SY6970_REG_0D, 
						SY6970_VINDPM_MASK, val << SY6970_VINDPM_SHIFT);
}

int sy6970_set_input_current_limit(struct sy6970 *bq, int curr)
{
	u8 val;
	
	dev_info(bq->dev, "%s: curr = %d\n", __func__, curr);

	if (curr < SY6970_IINLIM_BASE)
		curr = SY6970_IINLIM_BASE;

	val = (curr - SY6970_IINLIM_BASE) / SY6970_IINLIM_LSB;

	return sy6970_update_bits(bq, SY6970_REG_00, SY6970_IINLIM_MASK, 
						val << SY6970_IINLIM_SHIFT);
}


int sy6970_set_watchdog_timer(struct sy6970 *bq, u8 timeout)
{
	u8 val;

	val = (timeout - SY6970_WDT_BASE) / SY6970_WDT_LSB;
	val <<= SY6970_WDT_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_07, 
						SY6970_WDT_MASK, val); 
}
EXPORT_SYMBOL_GPL(sy6970_set_watchdog_timer);

int sy6970_disable_watchdog_timer(struct sy6970 *bq)
{
	u8 val = SY6970_WDT_DISABLE << SY6970_WDT_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_07, 
						SY6970_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(sy6970_disable_watchdog_timer);

int sy6970_reset_watchdog_timer(struct sy6970 *bq)
{
	u8 val = SY6970_WDT_RESET << SY6970_WDT_RESET_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_03, 
						SY6970_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(sy6970_reset_watchdog_timer);


int sy6970_force_dpdm(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_FORCE_DPDM << SY6970_FORCE_DPDM_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
						SY6970_FORCE_DPDM_MASK, val);

	pr_info("Force DPDM %s\n", !ret ?  "successfully" : "failed");
	
	return ret;

}
EXPORT_SYMBOL_GPL(sy6970_force_dpdm);

int sy6970_reset_chip(struct sy6970 *bq)
{
	int ret;
	u8 val = SY6970_RESET << SY6970_RESET_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_14, 
						SY6970_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(sy6970_reset_chip);

int sy6970_enter_hiz_mode(struct sy6970 *bq)
{
	u8 val = SY6970_HIZ_ENABLE << SY6970_ENHIZ_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_00, 
						SY6970_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(sy6970_enter_hiz_mode);

int sy6970_exit_hiz_mode(struct sy6970 *bq)
{

	u8 val = SY6970_HIZ_DISABLE << SY6970_ENHIZ_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_00, 
						SY6970_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(sy6970_exit_hiz_mode);

int sy6970_disable_enlim(struct sy6970 *bq)
{
	u8 val = SY6970_ENILIM_DISABLE << SY6970_ENILIM_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_00, 
						SY6970_ENILIM_MASK, val);

}
EXPORT_SYMBOL_GPL(sy6970_disable_enlim);

int sy6970_get_hiz_mode(struct sy6970 *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & SY6970_ENHIZ_MASK) >> SY6970_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(sy6970_get_hiz_mode);

static int sy6970_enable_term(struct sy6970 *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SY6970_TERM_ENABLE << SY6970_EN_TERM_SHIFT;
	else
		val = SY6970_TERM_DISABLE << SY6970_EN_TERM_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_07, 
						SY6970_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(sy6970_enable_term);

int sy6970_set_boost_current(struct sy6970 *bq, int curr)
{
	u8 temp;

	if (curr < 750)
		temp = SY6970_BOOST_LIM_500MA;
	else if (curr < 1200)
		temp = SY6970_BOOST_LIM_750MA;
	else if (curr < 1400)
		temp = SY6970_BOOST_LIM_1200MA;
	else if (curr < 1650)
		temp = SY6970_BOOST_LIM_1400MA;
	else if (curr < 1870)
		temp = SY6970_BOOST_LIM_1650MA;
	else if (curr < 2150)
		temp = SY6970_BOOST_LIM_1875MA;
	else if (curr < 2450)
		temp = SY6970_BOOST_LIM_2150MA;
	else
		temp= SY6970_BOOST_LIM_2450MA;

	return sy6970_update_bits(bq, SY6970_REG_0A, 
				SY6970_BOOST_LIM_MASK, 
				temp << SY6970_BOOST_LIM_SHIFT);

}

static int sy6970_enable_auto_dpdm(struct sy6970* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SY6970_AUTO_DPDM_ENABLE << SY6970_AUTO_DPDM_EN_SHIFT;
	else
		val = SY6970_AUTO_DPDM_DISABLE << SY6970_AUTO_DPDM_EN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, 
						SY6970_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(sy6970_enable_auto_dpdm);

int sy6970_set_boost_voltage(struct sy6970 *bq, int volt)
{
	u8 val = 0;

	if (volt < SY6970_BOOSTV_BASE)
		volt = SY6970_BOOSTV_BASE;
	if (volt > SY6970_BOOSTV_BASE 
			+ (SY6970_BOOSTV_MASK >> SY6970_BOOSTV_SHIFT) 
			* SY6970_BOOSTV_LSB)
		volt = SY6970_BOOSTV_BASE 
			+ (SY6970_BOOSTV_MASK >> SY6970_BOOSTV_SHIFT) 
			* SY6970_BOOSTV_LSB;

	val = ((volt - SY6970_BOOSTV_BASE) / SY6970_BOOSTV_LSB) 
			<< SY6970_BOOSTV_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_0A, 
				SY6970_BOOSTV_MASK, val);


}
EXPORT_SYMBOL_GPL(sy6970_set_boost_voltage);

static int sy6970_enable_ico(struct sy6970* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SY6970_ICO_ENABLE << SY6970_ICOEN_SHIFT;
	else
		val = SY6970_ICO_DISABLE << SY6970_ICOEN_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_02, SY6970_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(sy6970_enable_ico);

static int sy6970_read_idpm_limit(struct sy6970 *bq, int *icl)
{
	uint8_t val;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_13, &val);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		*icl = SY6970_IDPM_LIM_BASE + ((val & SY6970_IDPM_LIM_MASK) >> SY6970_IDPM_LIM_SHIFT) * SY6970_IDPM_LIM_LSB ;
		return 0;
	}
}
EXPORT_SYMBOL_GPL(sy6970_read_idpm_limit);

static int sy6970_enable_safety_timer(struct sy6970 *bq)
{
	const u8 val = SY6970_CHG_TIMER_ENABLE << SY6970_EN_TIMER_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_07, SY6970_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(sy6970_enable_safety_timer);

static int sy6970_disable_safety_timer(struct sy6970 *bq)
{
	const u8 val = SY6970_CHG_TIMER_DISABLE << SY6970_EN_TIMER_SHIFT;

	return sy6970_update_bits(bq, SY6970_REG_07, SY6970_EN_TIMER_MASK,
				   val);

}
EXPORT_SYMBOL_GPL(sy6970_disable_safety_timer);

static int sy6970_switch_to_hvdcp(struct sy6970 *bq, enum hvdcp_type type)
{

	int ret;
	u8 val;
	u8 mask;

	switch (type) {
	case HVDCP_5V:
	case HVDCP_9V:
		/*
		val = (SY6970_DP_3P3V << SY6970_DPDAC_SHIFT)
			| (SY6970_DM_0P6V << SY6970_DMDAC_SHIFT);
		break;
		*/
		val = (SY6970_DP_0P6V << SY6970_DPDAC_SHIFT) 
			| (SY6970_DM_0V << SY6970_DMDAC_SHIFT);
		break;
	case HVDCP_12V:
		val = (SY6970_DP_0P6V << SY6970_DPDAC_SHIFT)
			| (SY6970_DM_0P6V << SY6970_DMDAC_SHIFT);
		break;
	case HVDCP_20V:
		val = (SY6970_DP_3P3V << SY6970_DPDAC_SHIFT)
			| (SY6970_DM_3P3V << SY6970_DMDAC_SHIFT);
		break;

	case HVDCP_CONTINOUS:
		val = (SY6970_DP_0P6V << SY6970_DPDAC_SHIFT)
			| (SY6970_DM_3P3V << SY6970_DMDAC_SHIFT);
		break;
	case HVDCP_DPF_DMF:
		val = (SY6970_DP_HIZ << SY6970_DPDAC_SHIFT)
			| (SY6970_DM_HIZ << SY6970_DMDAC_SHIFT);
		break;
	default:
		break;
	}

	mask = SY6970_DPDAC_MASK | SY6970_DMDAC_MASK;

	ret = sy6970_update_bits(bq, SY6970_REG_01, mask, val);

	return ret;
}

static int sy6970_check_charge_done(struct sy6970 *bq, bool *done)
{
	int ret;
	u8 val;

	ret = sy6970_read_byte(bq, SY6970_REG_0B, &val);
	if (!ret) {
		val = val & SY6970_CHRG_STAT_MASK;
		val = val >> SY6970_CHRG_STAT_SHIFT;
		*done = (val == SY6970_CHRG_STAT_CHGDONE);
	}

	return ret;

}

static struct sy6970_platform_data *sy6970_parse_dt(struct device_node *np,
						      struct sy6970 *bq)
{
	int ret;
	struct sy6970_platform_data *pdata;

	pdata = devm_kzalloc(bq->dev, sizeof(struct sy6970_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &bq->chg_dev_name) < 0) {
		bq->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &bq->eint_name) < 0) {
		bq->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	bq->chg_det_enable =
	    of_property_read_bool(np, "ti,sy6970,charge-detect-enable");

	ret = of_property_read_u32(np, "ti,sy6970,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = 4500;
		pr_err("Failed to read node of ti,sy6970,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,sy6970,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = 2000;
		pr_err("Failed to read node of ti,sy6970,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,sy6970,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = 4200;
		pr_err("Failed to read node of ti,sy6970,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "ti,sy6970,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = 2000;
		pr_err("Failed to read node of ti,sy6970,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "ti,sy6970,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 256;
		pr_err("Failed to read node of ti,sy6970,precharge-current\n");
	}

	ret = of_property_read_u32(np, "ti,sy6970,termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = 250;
		pr_err("Failed to read node of ti,sy6970,termination-current\n");
	}

	ret =
	    of_property_read_u32(np, "ti,sy6970,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = 5000;
		pr_err("Failed to read node of ti,sy6970,boost-voltage\n");
	}

	ret =
	    of_property_read_u32(np, "ti,sy6970,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = 1200;
		pr_err("Failed to read node of ti,sy6970,boost-current\n");
	}


	return pdata;
}

static int sy6970_get_charger_type(struct sy6970 *bq, enum charger_type *type)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;
	enum charger_type chg_type = CHARGER_UNKNOWN;

	ret = sy6970_read_byte(bq, SY6970_REG_0B, &reg_val);

	if (ret)
		return ret;

	vbus_stat = (reg_val & SY6970_VBUS_STAT_MASK);
	vbus_stat >>= SY6970_VBUS_STAT_SHIFT;
	bq->vbus_type = vbus_stat;
	pr_err("sy6970_get_charger_type:%d,reg0B = 0x%x\n",vbus_stat,reg_val);
	switch (vbus_stat) {

	case SY6970_VBUS_TYPE_NONE:
		if(bq->power_good && (bq->is_bq2589x == false) && (bq->is_sy6970 == false)) {
			chg_type = STANDARD_CHARGER;
			bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		} else {
			chg_type = CHARGER_UNKNOWN;
			bq->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		}
		break;
	case SY6970_VBUS_TYPE_SDP:
		chg_type = STANDARD_HOST;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
		break;
	case SY6970_VBUS_TYPE_CDP:
		chg_type = CHARGING_HOST;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case SY6970_VBUS_TYPE_DCP:
		chg_type = STANDARD_CHARGER;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SY6970_VBUS_TYPE_HVDCP:
		chg_type = STANDARD_CHARGER;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		if(!disable_QC){
			bq->hvdcp_can_enabled = true;
		}
		break;
	case SY6970_VBUS_TYPE_UNKNOWN:
		chg_type = NONSTANDARD_CHARGER;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SY6970_VBUS_TYPE_NON_STD:
		chg_type = NONSTANDARD_CHARGER;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		chg_type = NONSTANDARD_CHARGER;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	}

	*type = chg_type;

	return 0;
}

static int sy6970_inform_charger_type(struct sy6970 *bq)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!bq->mtk_master_chg) {
		bq->mtk_master_chg = power_supply_get_by_name("mtk-master-charger");
		if (!bq->mtk_master_chg) {
			pr_notice("%s Couldn't get bq->mtk_master_chg\n", __func__);
			return -ENODEV;
		}
	}

	if (bq->chg_type != CHARGER_UNKNOWN)
		propval.intval = 1;
	else
		propval.intval = 0;

	ret = power_supply_set_property(bq->mtk_master_chg, POWER_SUPPLY_PROP_ONLINE,
					&propval);

	if (ret < 0)
		pr_notice("inform power supply online failed:%d\n", ret);

	power_supply_changed(bq->mtk_master_chg);
	return ret;
}

static irqreturn_t sy6970_irq_handler(int irq, void *data)
{
	struct sy6970 *bq = (struct sy6970 *)data;
	int ret;
	u8 reg_val;
	bool prev_pg;
	enum charger_type prev_chg_type;
	struct oplus_chg_chip *chip = g_oplus_chip;

	ret = sy6970_read_byte(bq, SY6970_REG_0B, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	prev_pg = bq->power_good;

	bq->power_good = !!(reg_val & SY6970_PG_STAT_MASK);
	bq->chg_det_enable = bq->power_good;

	pr_notice("sy6970_irq_handler:(%d,%d)\n",prev_pg,bq->power_good);
	
	oplus_sy6970_set_mivr_by_battery_vol();
	
	if(dumpreg_by_irq)
		sy6970_dump_regs(bq);
	
	if (!prev_pg && bq->power_good) {
#ifdef CONFIG_TCPC_CLASS
		if (!bq->chg_det_enable)
			return IRQ_HANDLED;
#endif
		get_monotonic_boottime(&bq->ptime[0]);
		g_bq->psy_online = true;
		pr_notice("adapter/usb inserted\n");
	} else if (prev_pg && !bq->power_good) {
		bq->chg_cur = 0;
		bq->psy_online = false;
#ifdef CONFIG_TCPC_CLASS
		if (bq->chg_det_enable)
			return IRQ_HANDLED;
#endif
		bq->pre_current_ma = -1;
		bq->hvdcp_can_enabled = false;
		bq->hvdcp_checked = false;
		bq->chg_type = CHARGER_UNKNOWN;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		bq->nonstand_retry_bc = false;
		memset(&bq->ptime[0], 0, sizeof(struct timespec));
		memset(&bq->ptime[1], 0, sizeof(struct timespec));
		if (chip) {
			// chip->pd_chging = false;
			// chip->qc_chging = false;
			pr_notice("%s pd qc is false\n", __func__);			
		}

		sy6970_inform_charger_type(bq);
		if (bq->is_bq2589x){
			sy6970_disable_hvdcp(bq);
		}else{
			sy6970_enable_hvdcp(bq);
		}
		//Junbo.Guo@ODM_WT.BSP.CHG, 2020/05/06, Modify for fix OTG can not connect when QC plugout
		sy6970_switch_to_hvdcp(g_bq, HVDCP_DPF_DMF);
		Charger_Detect_Release();
		cancel_delayed_work_sync(&bq->sy6970_aicr_setting_work);
		cancel_delayed_work_sync(&bq->sy6970_retry_adapter_detection);
		cancel_delayed_work_sync(&bq->sy6970_current_setting_work);
		pr_notice("adapter/usb removed\n");
		return IRQ_HANDLED;
	} else if (!prev_pg && !bq->power_good) {
		pr_notice("prev_pg & now_pg is false\n");
		return IRQ_HANDLED;
	}

	if (g_bq->otg_enable) {
		pr_notice("%s otg_enable\n", __func__);
		g_bq->psy_online = false;
		return IRQ_HANDLED;
	}

	prev_chg_type = bq->chg_type;
	ret = sy6970_get_charger_type(bq, &bq->chg_type);
	oplus_wake_up_usbtemp_thread();
	sy6970_inform_charger_type(bq);
	if (!ret && prev_chg_type != bq->chg_type && bq->chg_det_enable) {
		if ((NONSTANDARD_CHARGER == bq->chg_type) && (!bq->nonstand_retry_bc)) {
			bq->nonstand_retry_bc = true;
			sy6970_force_dpdm(bq);
			return IRQ_HANDLED;
		} else if (STANDARD_CHARGER != bq->chg_type) {
			Charger_Detect_Release();
		}

		if (bq->chg_type == CHARGER_UNKNOWN) {
			if (bq->is_bq2589x){
				sy6970_disable_hvdcp(bq);
			}else{
				sy6970_enable_hvdcp(bq);
			}
			//Junbo.Guo@ODM_WT.BSP.CHG, 2020/05/06, Modify for fix OTG can not connect when QC plugout
			sy6970_switch_to_hvdcp(g_bq, HVDCP_DPF_DMF);
			memset(&bq->ptime[0], 0, sizeof(struct timespec));
			memset(&bq->ptime[1], 0, sizeof(struct timespec));
			cancel_delayed_work_sync(&bq->sy6970_aicr_setting_work);
			cancel_delayed_work_sync(&bq->sy6970_current_setting_work);
		}

		sy6970_inform_charger_type(bq);
		if (bq->is_bq2589x) {
			if (!bq->hvdcp_checked && bq->vbus_type == SY6970_VBUS_TYPE_DCP) {
				bq->hvdcp_checked = true;
				sy6970_enable_hvdcp(bq);
				sy6970_force_dpdm(bq);
			}
		}
	} else if (!ret && (prev_chg_type == bq->chg_type)
		&& (bq->vbus_type == SY6970_VBUS_TYPE_DCP)
		&& !bq->retry_hvdcp_algo && bq->chg_det_enable) {
		bq->retry_hvdcp_algo = true;
		schedule_delayed_work(&g_bq->sy6970_retry_adapter_detection, msecs_to_jiffies(3000));
	}

	return IRQ_HANDLED;
}


int irq_gpio_init_sy6970(struct sy6970 *bq)
{
	bq->irq_gpio_pinctrl.pinctrl = devm_pinctrl_get(bq->dev);
	bq->irq_gpio_pinctrl.irq_active =
		pinctrl_lookup_state(bq->irq_gpio_pinctrl.pinctrl,
			"irq_default");

	if (IS_ERR_OR_NULL(bq->irq_gpio_pinctrl.irq_active)) {
		chg_err("get irq_default fail\n");
		return -EINVAL;
	}
	bq->irq_gpio_pinctrl.irq_sleep =
			pinctrl_lookup_state(bq->irq_gpio_pinctrl.pinctrl,
				"irq_set_low");
	if (IS_ERR_OR_NULL(bq->irq_gpio_pinctrl.irq_sleep)) {
		chg_err("get irq_set_low fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(bq->irq_gpio_pinctrl.pinctrl,
		bq->irq_gpio_pinctrl.irq_active);
	return 0;
}

static int sy6970_register_interrupt(struct device_node *np,struct sy6970 *bq)
{
	int ret = 0;

	ret = irq_gpio_init_sy6970(bq);
	if (ret < 0) {
		pr_err("irq_gpio_init failed:%d\n", ret);
	}

	bq->irq = irq_of_parse_and_map(np, 0);
	pr_info("irq = %d\n", bq->irq);

	ret = devm_request_threaded_irq(bq->dev, bq->irq, NULL,
					sy6970_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					bq->eint_name, bq);
	if (ret < 0) {
		pr_err("request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(bq->irq);

	return 0;
}

static int sy6970_init_device(struct sy6970 *bq)
{
	int ret;

	sy6970_disable_watchdog_timer(bq);

	ret = sy6970_disable_enlim(bq);
	if (ret)
		pr_err("Failed to set daisable enlim, ret = %d\n", ret);

	ret = sy6970_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = sy6970_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = sy6970_set_boost_voltage(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = sy6970_set_boost_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	return 0;
}

bool sy6970_is_hvdcp(struct sy6970 *bq)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;

	ret = sy6970_read_byte(bq, SY6970_REG_0B, &reg_val);

	if (ret)
		return 0;

	vbus_stat = (reg_val & SY6970_VBUS_STAT_MASK);
	vbus_stat >>= SY6970_VBUS_STAT_SHIFT;

	if(vbus_stat == SY6970_VBUS_TYPE_HVDCP)
		return 1;

	return 0;
}

static bool sy6970_is_usb(struct sy6970 *bq)
{
	int ret = 0;
	u8 reg_val = 0;
	int vbus_stat = 0;

	ret = sy6970_read_byte(bq, SY6970_REG_0B, &reg_val);
	if (ret) {
		return false;
	}

	vbus_stat = (reg_val & SY6970_VBUS_STAT_MASK);
	vbus_stat >>= SY6970_VBUS_STAT_SHIFT;

	if (vbus_stat == SY6970_VBUS_TYPE_SDP) {
		return true;
	}

	return false;
}

static void determine_initial_status(struct sy6970 *bq)
{
	if(sy6970_is_hvdcp(bq)){
		sy6970_get_charger_type(bq, &bq->chg_type);
		sy6970_inform_charger_type(bq);
	}
}

static int sy6970_detect_device(struct sy6970 *bq)
{
	int ret;
	u8 data;

	ret = sy6970_read_byte(bq, SY6970_REG_14, &data);
	if (!ret) {
		bq->part_no = (data & SY6970_PN_MASK) >> SY6970_PN_SHIFT;
		bq->revision =
		    (data & SY6970_DEV_REV_MASK) >> SY6970_DEV_REV_SHIFT;
	}
	if (bq->part_no == 0b011) {
		bq->is_bq2589x = true;
		bq->is_sy6970 = false;
		pr_notice("/**********charger IC is BQ2589x***********/ \n");
	} else {
		bq->is_bq2589x = false;
		bq->is_sy6970 = true;
		pr_notice("/**********charger IC is SY6970***********/ \n");
	}
	return ret;
}


static int bq2589x_get_charging_status(struct sy6970 *bq, enum bq2589x_charging_status *chg_stat)
{
	int ret = 0;
	u8 val;
	
	ret = sy6970_read_byte(bq, SY6970_REG_0B, &val);
	if(ret < 0){
		*chg_stat = BQ2589X_CHG_STATUS_NOT_CHARGING;
		return ret;
	}
	val = (val & SY6970_CHRG_STAT_MASK) >> SY6970_CHRG_STAT_SHIFT;
	
	switch (val) {
		case SY6970_CHRG_STAT_IDLE:
			*chg_stat = BQ2589X_CHG_STATUS_NOT_CHARGING;
			break;
		case SY6970_CHRG_STAT_PRECHG:
			*chg_stat = BQ2589X_CHG_STATUS_PRE_CHARGING;
			break;
		case SY6970_CHRG_STAT_FASTCHG:
			*chg_stat = BQ2589X_CHG_STATUS_FAST_CHARGING;
			break;
		case SY6970_CHRG_STAT_CHGDONE:
			*chg_stat = BQ2589X_CHG_STATUS_DONE;
			break;
		default:
			*chg_stat = BQ2589X_CHG_STATUS_NOT_CHARGING;
			break;
	}
	return 0;
}

static void sy6970_dump_regs(struct sy6970 *bq)
{
	int addr;
	u8 val[25];
	int ret;
	char buf[400];
	char *s = buf;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = sy6970_read_byte(bq, addr, &val[addr]);
		msleep(1);
	}
	
	s+=sprintf(s,"sy6970_dump_regs:");
	for (addr = 0x0; addr <= 0x14; addr++){
		s+=sprintf(s,"[0x%.2x,0x%.2x]", addr, val[addr]);
	}
	s+=sprintf(s,"\n");
	
	dev_info(bq->dev,"%s",buf);
}

static ssize_t
sy6970_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct sy6970 *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sy6970 Reg");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = sy6970_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
sy6970_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sy6970 *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x14) {
		sy6970_write_byte(bq, (unsigned char) reg,
				   (unsigned char) val);
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, sy6970_show_registers,
		   sy6970_store_registers);

static struct attribute *sy6970_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sy6970_attr_group = {
	.attrs = sy6970_attributes,
};

static int sy6970_charging(struct charger_device *chg_dev, bool enable)
{

	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = sy6970_enable_charger(bq);
	else
		ret = sy6970_disable_charger(bq);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	ret = sy6970_read_byte(bq, SY6970_REG_03, &val);

	if (!ret)
		bq->charge_enabled = !!(val & SY6970_CHG_CONFIG_MASK);

	return ret;
}

static int sy6970_plug_in(struct charger_device *chg_dev)
{

	int ret;

	ret = sy6970_charging(chg_dev, true);

	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int sy6970_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = sy6970_charging(chg_dev, false);

	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int sy6970_dump_register(struct charger_device *chg_dev)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	sy6970_dump_regs(bq);

	return 0;
}

static int sy6970_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	*en = bq->charge_enabled;

	return 0;
}

static int sy6970_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	ret = sy6970_check_charge_done(bq, done);

	return ret;
}

static int sy6970_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge curr = %d\n", curr);

	return sy6970_set_chargecurrent(bq, curr / 1000);
}

static int _sy6970_get_ichg(struct sy6970 *bq, u32 *curr)
{
	u8 reg_val;
	int ichg;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_04, &reg_val);
	if (!ret) {
		ichg = (reg_val & SY6970_ICHG_MASK) >> SY6970_ICHG_SHIFT;
		ichg = ichg * SY6970_ICHG_LSB + SY6970_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int sy6970_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_04, &reg_val);
	if (!ret) {
		ichg = (reg_val & SY6970_ICHG_MASK) >> SY6970_ICHG_SHIFT;
		ichg = ichg * SY6970_ICHG_LSB + SY6970_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int sy6970_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * 1000;

	return 0;
}

static int sy6970_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge volt = %d\n", volt);

	return sy6970_set_chargevolt(bq, volt / 1000);
}

static int sy6970_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_06, &reg_val);
	if (!ret) {
		vchg = (reg_val & SY6970_VREG_MASK) >> SY6970_VREG_SHIFT;
		vchg = vchg * SY6970_VREG_LSB + SY6970_VREG_BASE;
		*volt = vchg * 1000;
	}

	return ret;
}

static int sy6970_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("vindpm volt = %d\n", volt);

	return sy6970_set_input_volt_limit(bq, volt / 1000);

}

static int sy6970_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	pr_err("indpm curr = %d\n", curr);

	return sy6970_set_input_current_limit(bq, curr / 1000);
}

static int sy6970_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = sy6970_read_byte(bq, SY6970_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & SY6970_IINLIM_MASK) >> SY6970_IINLIM_SHIFT;
		icl = icl * SY6970_IINLIM_LSB + SY6970_IINLIM_BASE;
		*curr = icl * 1000;
	}

	return ret;

}

static int sy6970_kick_wdt(struct charger_device *chg_dev)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	return sy6970_reset_watchdog_timer(bq);
}

static int sy6970_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = sy6970_enable_otg(bq);
	else
		ret = sy6970_disable_otg(bq);

	if(!ret)
		bq->otg_enable = en;

	pr_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int sy6970_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = sy6970_enable_safety_timer(bq);
	else
		ret = sy6970_disable_safety_timer(bq);

	return ret;
}

static int sy6970_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = sy6970_read_byte(bq, SY6970_REG_07, &reg_val);

	if (!ret)
		*en = !!(reg_val & SY6970_EN_TIMER_MASK);

	return ret;
}

static int sy6970_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("otg curr = %d\n", curr);

	ret = sy6970_set_boost_current(bq, curr / 1000);

	return ret;
}

static int sy6970_chgdet_en(struct sy6970 *bq, bool en)
{
	int ret;
	u8 val;
	struct oplus_chg_chip *chip = g_oplus_chip;

	pr_notice("sy6970_chgdet_en:%d\n",en);
	bq->chg_det_enable = en;

	if (en) {
		Charger_Detect_Init();
	} else {
		bq->pre_current_ma = -1;
		bq->hvdcp_can_enabled = false;
		bq->hvdcp_checked = false;
		bq->retry_hvdcp_algo = false;
		bq->nonstand_retry_bc = false;
		if (chip) {
			// chip->pd_chging = false;
			// chip->qc_chging = false;
			pr_notice("%s pd qc is false\n", __func__);
		}
		if (bq->is_bq2589x){
				sy6970_disable_hvdcp(bq);
			}else{
				sy6970_enable_hvdcp(bq);
			}
		//Junbo.Guo@ODM_WT.BSP.CHG, 2020/05/06, Modify for fix OTG can not connect when QC plugout
		sy6970_switch_to_hvdcp(g_bq, HVDCP_DPF_DMF);
		Charger_Detect_Release();
		memset(&bq->ptime[0], 0, sizeof(struct timespec));
		memset(&bq->ptime[1], 0, sizeof(struct timespec));
		cancel_delayed_work_sync(&bq->sy6970_aicr_setting_work);
		cancel_delayed_work_sync(&bq->sy6970_retry_adapter_detection);
		cancel_delayed_work_sync(&bq->sy6970_current_setting_work);
		bq->chg_type = CHARGER_UNKNOWN;
		bq->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		sy6970_inform_charger_type(bq);
	}

	val = en ? SY6970_AUTO_DPDM_ENABLE : SY6970_AUTO_DPDM_DISABLE;
	val <<= SY6970_AUTO_DPDM_EN_SHIFT;
	ret = sy6970_update_bits(bq, SY6970_REG_02,
						SY6970_AUTO_DPDM_EN_MASK, val);

	if (!ret && en) {
		if(false == sy6970_is_hvdcp(bq)){
			if (bq->is_bq2589x){
				sy6970_disable_hvdcp(bq);
			}else{
				sy6970_enable_hvdcp(bq);
			}		
			sy6970_force_dpdm(bq);
		}
	}

	return ret;
}

static int sy6970_enable_chgdet(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);

	pr_notice("sy6970_enable_chgdet:%d\n",en);

	ret = sy6970_chgdet_en(bq, en);

	return ret;
}

static int sy6970_enter_ship_mode(struct sy6970 *bq, bool en)
{
	int ret;
	u8 val;

	if (en)
		val = SY6970_BATFET_OFF;
	else
		val = SY6970_BATFET_ON;
	val <<= SY6970_BATFET_DIS_SHIFT;

	ret = sy6970_update_bits(bq, SY6970_REG_09, 
						SY6970_BATFET_DIS_MASK, val);
	return ret;

}

static int sy6970_enable_shipmode(bool en)
{
	int ret;
	
	ret = sy6970_enter_ship_mode(g_bq, en);
	
	return 0;
}

static int sy6970_set_hz_mode(bool en)
{
	int ret;

	if (en) {
		ret = sy6970_enter_hiz_mode(g_bq);
	} else {
		ret = sy6970_exit_hiz_mode(g_bq);
	}

	return ret;
}
#ifndef CONFIG_CHARGER_SY6970
static int sy6970_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	int ret = 0;
	struct mtk_charger *chg_mgr = NULL;

	chg_mgr = charger_dev_get_drvdata(chg_dev);
	if (!chg_mgr)
		return -EINVAL;
		
	chg_mgr->pe2.profile[0].vbat = 3400000;
	chg_mgr->pe2.profile[1].vbat = 3500000;
	chg_mgr->pe2.profile[2].vbat = 3600000;
	chg_mgr->pe2.profile[3].vbat = 3700000;
	chg_mgr->pe2.profile[4].vbat = 3800000;
	chg_mgr->pe2.profile[5].vbat = 3900000;
	chg_mgr->pe2.profile[6].vbat = 4000000;
	chg_mgr->pe2.profile[7].vbat = 4100000;
	chg_mgr->pe2.profile[8].vbat = 4200000;
	chg_mgr->pe2.profile[9].vbat = 4400000;

	chg_mgr->pe2.profile[0].vchr = 8000000;
	chg_mgr->pe2.profile[1].vchr = 8000000;
	chg_mgr->pe2.profile[2].vchr = 8000000;
	chg_mgr->pe2.profile[3].vchr = 8500000;
	chg_mgr->pe2.profile[4].vchr = 8500000;
	chg_mgr->pe2.profile[5].vchr = 8500000;
	chg_mgr->pe2.profile[6].vchr = 9000000;
	chg_mgr->pe2.profile[7].vchr = 9000000;
	chg_mgr->pe2.profile[8].vchr = 9000000;
	chg_mgr->pe2.profile[9].vchr = 9000000;

	return ret;
}

struct timespec ptime[13];
static int cptime[13][2];

static int dtime(int i)
{
	struct timespec time;

	time = timespec_sub(ptime[i], ptime[i-1]);
	return time.tv_nsec/1000000;
}

#define PEOFFTIME 40
#define PEONTIME 90

static int sy6970_set_pep20_current_pattern(struct charger_device *chg_dev,
	u32 chr_vol)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	int value;
	int i, j = 0;
	int flag;

	
	//bq25890_set_vindpm(0x13);
	sy6970_set_input_volt_limit(bq, 4500);
	//bq25890_set_ichg(8);
	sy6970_set_chargecurrent(bq, 512);
	//bq25890_set_ico_en_start(0);
	sy6970_enable_ico(bq, false);

	usleep_range(1000, 1200);
	value = (chr_vol - 5500000) / 500000;

	//bq25890_set_iinlim(0x0);
	sy6970_set_input_current_limit(bq, 0);
	
	msleep(70);

	get_monotonic_boottime(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
		flag = value & (1 << i);

		if (flag == 0) {
			//bq25890_set_iinlim(0xc);
			sy6970_set_input_current_limit(bq, 700);
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				pr_info(
					"charging_set_ta20_current_pattern fail1: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
			//bq25890_set_iinlim(0x0);
			sy6970_set_input_current_limit(bq, 0);
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				pr_info(
					"charging_set_ta20_current_pattern fail2: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;

		} else {
			//bq25890_set_iinlim(0xc);
			sy6970_set_input_current_limit(bq, 700);
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				pr_info(
					"charging_set_ta20_current_pattern fail3: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
			//bq25890_set_iinlim(0x0);
			sy6970_set_input_current_limit(bq, 0);
			
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				pr_info(
					"charging_set_ta20_current_pattern fail4: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
		}
	}

	//bq25890_set_iinlim(0xc);
	sy6970_set_input_current_limit(bq, 700);
	msleep(160);
	get_monotonic_boottime(&ptime[j]);
	cptime[j][0] = 160;
	cptime[j][1] = dtime(j);
	if (cptime[j][1] < 150 || cptime[j][1] > 240) {
		pr_info(
			"charging_set_ta20_current_pattern fail5: idx:%d target:%d actual:%d\n",
			i, PEOFFTIME, cptime[j][1]);
		return -EIO;
	}
	j++;

	//bq25890_set_iinlim(0x0);
	sy6970_set_input_current_limit(bq, 0);
	msleep(30);
	//bq25890_set_iinlim(0xc);
	sy6970_set_input_current_limit(bq, 700);

	pr_info(
	"[charging_set_ta20_current_pattern]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	chr_vol, value,
	cptime[1][0], cptime[2][0], cptime[3][0], cptime[4][0], cptime[5][0],
	cptime[6][0], cptime[7][0], cptime[8][0], cptime[9][0], cptime[10][0], cptime[11][0]);

	pr_info(
	"[charging_set_ta20_current_pattern2]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	chr_vol, value,
	cptime[1][1], cptime[2][1], cptime[3][1], cptime[4][1], cptime[5][1],
	cptime[6][1], cptime[7][1], cptime[8][1], cptime[9][1], cptime[10][1], cptime[11][1]);


	//bq25890_set_ico_en_start(1);
	sy6970_enable_ico(bq, true);
	
	//bq25890_set_iinlim(0x3f);
	sy6970_set_input_current_limit(bq, 3250);
	
	bq->pre_current_ma = -1;
	return 0;
}

static int sy6970_set_pep20_reset(struct charger_device *chg_dev)
{
	struct sy6970 *bq = dev_get_drvdata(&chg_dev->dev);
	
	//bq25890_set_vindpm(0x13);
	sy6970_set_input_volt_limit(bq, 4500);
	//bq25890_set_ichg(8);
	sy6970_set_chargecurrent(bq, 512);

	//bq25890_set_ico_en_start(0);
	sy6970_enable_ico(bq, false);
	
	//bq25890_set_iinlim(0x0);
	sy6970_set_input_current_limit(bq, 0);
	
	msleep(250);
	
	//bq25890_set_iinlim(0xc);
	sy6970_set_input_current_limit(bq, 700);
	
	//bq25890_set_ico_en_start(1);
	sy6970_enable_ico(bq, true);
	bq->pre_current_ma = -1;
	
	return 0;
}
#endif // !	CONFIG_CHARGER_SY6970
static struct charger_ops sy6970_chg_ops = {
	/* Normal charging */
	.plug_in = sy6970_plug_in,
	.plug_out = sy6970_plug_out,
	.dump_registers = sy6970_dump_register,
	.enable = sy6970_charging,
	.is_enabled = sy6970_is_charging_enable,
	.get_charging_current = sy6970_get_ichg,
	.set_charging_current = sy6970_set_ichg,
	.get_input_current = sy6970_get_icl,
	.set_input_current = sy6970_set_icl,
	.get_constant_voltage = sy6970_get_vchg,
	.set_constant_voltage = sy6970_set_vchg,
	.kick_wdt = sy6970_kick_wdt,
	.set_mivr = sy6970_set_ivl,
	.is_charging_done = sy6970_is_charging_done,
	.get_min_charging_current = sy6970_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = sy6970_set_safety_timer,
	.is_safety_timer_enabled = sy6970_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	.enable_chg_type_det = sy6970_enable_chgdet,
	/* OTG */
	.enable_otg = sy6970_set_otg,
	.set_boost_current_limit = sy6970_set_boost_ilmt,
	.enable_discharge = NULL,
#ifndef CONFIG_CHARGER_SY6970
	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = sy6970_set_pep20_efficiency_table,
	.send_ta20_current_pattern = sy6970_set_pep20_current_pattern,
	.reset_ta = sy6970_set_pep20_reset,
	.enable_cable_drop_comp = NULL,
#endif // !CONFIG_CHARGER_SY6970

	/* ADC */
	.get_tchg_adc = get_chgntc_adc_temp,
	.get_vbus_adc = sy6970_get_vbus_adc,
};

void oplus_sy6970_dump_registers(void)
{
	sy6970_dump_regs(g_bq);

	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->kick_wdt();
        g_oplus_chip->sub_chg_ops->dump_registers();
	}
}

int oplus_sy6970_kick_wdt(void)
{
	return sy6970_reset_watchdog_timer(g_bq);
}

int oplus_sy6970_set_ichg(int cur)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	u32 uA = cur*1000;
	u32 temp_uA;
	int ret = 0;

	if(cur <= 1000){   // <= 1A
		ret = sy6970_set_chargecurrent(g_bq, cur);
		if(chip->is_double_charger_support)
			chip->sub_chg_ops->charging_current_write_fast(0);
	} else if(chip->is_double_charger_support){
		temp_uA = uA  * current_percent / 100;
		ret = sy6970_set_chargecurrent(g_bq, temp_uA/1000);
		ret = _sy6970_get_ichg(g_bq, &temp_uA);
		uA-=temp_uA;
		chip->sub_chg_ops->charging_current_write_fast(uA/1000);
	} else {
		ret = sy6970_set_chargecurrent(g_bq, cur);
	}
	
	return ret;
}

void oplus_sy6970_set_mivr(int vbatt)
{
	if(g_bq->hw_aicl_point == 4400 && vbatt > 4250) {
		g_bq->hw_aicl_point = 4500;
	} else if(g_bq->hw_aicl_point == 4500 && vbatt < 4150) {
		g_bq->hw_aicl_point = 4400;
	}

	sy6970_set_input_volt_limit(g_bq, g_bq->hw_aicl_point);

	if(g_oplus_chip->is_double_charger_support) {
		g_oplus_chip->sub_chg_ops->set_aicl_point(vbatt);
	}
}

void oplus_sy6970_set_mivr_by_battery_vol(void)
{

	u32 mV =0;
	int vbatt = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (chip) {
		vbatt = chip->batt_volt;
	}
	
	if(vbatt > 4300){
		mV = vbatt + 400;
	}else if(vbatt > 4200){
		mV = vbatt + 300;
	}else{
		mV = vbatt + 200;
	}
	
    if(mV<4400)
        mV = 4400;

     sy6970_set_input_volt_limit(g_bq, mV);
}

static int usb_icl[] = {
	100, 500, 900, 1200, 1500, 1750, 2000, 3000,
};

int oplus_sy6970_set_aicr(int current_ma)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;

	if (g_bq->pre_current_ma == current_ma)
		return rc;
	else
		g_bq->pre_current_ma = current_ma;

	if(chip->is_double_charger_support)
		chip->sub_chg_ops->input_current_write(0);


	dev_info(g_bq->dev, "%s usb input max current limit=%d\n", __func__,current_ma);
	aicl_point_temp = aicl_point = 4500;
	//sy6970_set_input_volt_limit(g_bq, 4200);
	
	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(90);

	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		pr_debug( "use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(90);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1500 */
	aicl_point_temp = aicl_point + 50;
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(120);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 1; //We use 1.2A here
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 5; /* 1750 */
	aicl_point_temp = aicl_point + 50;
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(120);
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //1.2
		goto aicl_pre_step;
	}

	i = 6; /* 2000 */
	aicl_point_temp = aicl_point;
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(90);
	if (chg_vol < aicl_point_temp) {
		i =  i - 2;//1.5
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	msleep(90);
	//chg_vol = battery_get_vbus();
	chg_vol = mt6357_get_vbus_voltage();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	if(chip->is_double_charger_support){
		if(usb_icl[i]>1000){
			sy6970_set_input_current_limit(g_bq, usb_icl[i]*current_percent/100);
			chip->sub_chg_ops->input_current_write(usb_icl[i]*(100-current_percent)/100);
		}else{
			sy6970_set_input_current_limit(g_bq, usb_icl[i]);
		}
	}else{
		sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	}
	dev_info(g_bq->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n",__func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	return rc;
aicl_end:
	if(chip->is_double_charger_support){
		if(usb_icl[i]>1000){
			sy6970_set_input_current_limit(g_bq, usb_icl[i] *current_percent/100);
			chip->sub_chg_ops->input_current_write(usb_icl[i]*(100-current_percent)/100);
		}else{
			sy6970_set_input_current_limit(g_bq, usb_icl[i]);
		}
	}else{
		sy6970_set_input_current_limit(g_bq, usb_icl[i]);
	}
	dev_info(g_bq->dev, "%s:usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n",__func__, chg_vol, i, usb_icl[i], aicl_point_temp);
	return rc;
}

int oplus_sy6970_set_input_current_limit(int current_ma)
{
	struct timespec diff;
	unsigned int ms;

	get_monotonic_boottime(&g_bq->ptime[1]);
	diff = timespec_sub(g_bq->ptime[1], g_bq->ptime[0]);
	g_bq->aicr = current_ma;
	if (current_ma && diff.tv_sec < 10) {
		ms = (10 - diff.tv_sec)*1000;
		cancel_delayed_work(&g_bq->sy6970_aicr_setting_work);
		dev_info(g_bq->dev, "delayed work %d ms", ms);
		schedule_delayed_work(&g_bq->sy6970_aicr_setting_work, msecs_to_jiffies(ms));
	} else {
		cancel_delayed_work(&g_bq->sy6970_aicr_setting_work);
		schedule_delayed_work(&g_bq->sy6970_aicr_setting_work, 0);
	}

	return 0;
}

int oplus_sy6970_set_cv(int cur)
{
	return sy6970_set_chargevolt(g_bq, cur);
}

int oplus_sy6970_set_ieoc(int cur)
{
	return sy6970_set_term_current(g_bq, cur);
}

int oplus_sy6970_charging_enable(void)
{
	return sy6970_enable_charger(g_bq); 
}

int oplus_sy6970_charging_disable(void)
{
	struct mtk_charger *info = NULL;
	
	if(g_bq->chg_consumer != NULL)
		info = g_bq->chg_consumer->cm;

	if(!info){
		dev_info(g_bq->dev, "%s:error\n", __func__);
		return false;
	}

	//mtk_pdc_plugout(info);
	if(g_bq->hvdcp_can_enabled){
		if(!(g_bq->is_bq2589x)){
			  sy6970_disable_hvdcp(g_bq);
			  sy6970_force_dpdm(g_bq);
		  }else{
			sy6970_switch_to_hvdcp(g_bq, HVDCP_5V);
		  }
	dev_info(g_bq->dev, "%s: set qc to 5V", __func__);
	}

	sy6970_disable_watchdog_timer(g_bq);
	g_bq->pre_current_ma = -1;
	g_bq->hw_aicl_point =4400;
	sy6970_set_input_volt_limit(g_bq, g_bq->hw_aicl_point);
	return sy6970_disable_charger(g_bq);
}

int oplus_sy6970_hardware_init(void)
{
	int ret = 0;

	dev_info(g_bq->dev, "%s\n", __func__);

	g_bq->hw_aicl_point =4400;
	sy6970_set_input_volt_limit(g_bq, g_bq->hw_aicl_point);

	/* Enable WDT */
	ret = sy6970_set_watchdog_timer(g_bq, 80);
	if (ret < 0)
		dev_notice(g_bq->dev, "%s: en wdt fail\n", __func__);

	/* Enable charging */
	if (strcmp(g_bq->chg_dev_name, "primary_chg") == 0) {
		ret = sy6970_enable_charger(g_bq);
		if (ret < 0)
			dev_notice(g_bq->dev, "%s: en chg fail\n", __func__);
	}

	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		if(!sy6970_is_usb(g_bq)) {
			g_oplus_chip->sub_chg_ops->hardware_init();
		} else {
			g_oplus_chip->sub_chg_ops->charging_disable();
		}

	}

	return ret;

}

int oplus_sy6970_is_charging_enabled(void)
{
	
	return g_bq->charge_enabled;
}

int oplus_sy6970_is_charging_done(void)
{
	int ret = 0;
	bool done;

	sy6970_check_charge_done(g_bq, &done);

	return done;

}

int oplus_sy6970_enable_otg(void)
{
	int ret = 0;

	ret = sy6970_set_boost_current(g_bq, g_bq->platform_data->boosti);
	ret = sy6970_enable_otg(g_bq);

	if (ret < 0) {
		dev_notice(g_bq->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_bq->otg_enable = true;
	return ret;
}

int oplus_sy6970_disable_otg(void)
{
	int ret = 0;

	ret = sy6970_disable_otg(g_bq);

	if (ret < 0) {
		dev_notice(g_bq->dev, "%s disable otg fail(%d)\n", __func__, ret);
		return ret;
	}

	g_bq->otg_enable = false;
	return ret;

}

int oplus_sy6970_disable_te(void)
{
	return  sy6970_enable_term(g_bq, false);
}

int oplus_sy6970_get_chg_current_step(void)
{
	return SY6970_ICHG_LSB;
}

int oplus_sy6970_get_charger_type(void)
{
	return g_bq->oplus_chg_type;
}

int oplus_sy6970_charger_suspend(void)
{
	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->charger_suspend();
    }

	return 0;
}

int oplus_sy6970_charger_unsuspend(void)
{
	if((g_oplus_chip != NULL) && (g_oplus_chip->is_double_charger_support)) {
		g_oplus_chip->sub_chg_ops->charger_unsuspend();
	}

	return 0;
}

int oplus_sy6970_set_rechg_vol(int vol)
{
	return 0;
}

int oplus_sy6970_reset_charger(void)
{
	return 0;
}

bool oplus_sy6970_check_charger_resume(void)
{
	return true;
}

void oplus_sy6970_set_chargerid_switch_val(int value)
{
	return;
}

int oplus_sy6970_get_chargerid_switch_val(void)
{
	return 0;
}

int oplus_sy6970_get_chargerid_volt(void)
{
	return 0;
}

bool oplus_sy6970_check_chrdet_status(void)
{
	return g_bq->power_good;
}

int oplus_sy6970_get_charger_subtype(void)
{
	struct mtk_charger *info = NULL;

	if(g_bq->chg_consumer != NULL)
		info = g_bq->chg_consumer->cm;

	if(!info){
		dev_info(g_bq->dev, "%s:error\n", __func__);
		return false;
	}

	/*if (mtk_pdc_check_charger(info) && (!disable_PD)) {
		return CHARGER_SUBTYPE_PD;
	} else */
	if (g_bq->hvdcp_can_enabled){
		return CHARGER_SUBTYPE_QC;
	} else {
		return CHARGER_SUBTYPE_DEFAULT;
	}
}

bool oplus_sy6970_need_to_check_ibatt(void)
{
	return false;
}

int oplus_sy6970_get_dyna_aicl_result(void)
{
	int mA = 0;
	
	sy6970_read_idpm_limit(g_bq, &mA);
	return mA;
}

int oplus_sy6970_set_qc_config(void)
{  /*
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct mtk_charger *info = NULL;
		
	if(g_bq->chg_consumer != NULL)
		info = g_bq->chg_consumer->cm;

	if(!info){
		dev_info(g_bq->dev, "%s:error\n", __func__);
		return false;
	}
	
	if (!chip) {
		dev_info(g_bq->dev, "%s: error\n", __func__);
		return false;
	}

	if(disable_QC){
		dev_info(g_bq->dev, "%s:disable_QC\n", __func__);
		return false;
	}

	if(g_bq->disable_hight_vbus==1){
		dev_info(g_bq->dev, "%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if (chip->limits.vbatt_pdqc_to_5v_thr > 0 && chip->charger_volt > 7500
		&& chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr&&chip->ui_soc>=85&&chip->icharging > -1000) {
		if(!(g_bq->is_bq2589x)){
			sy6970_disable_hvdcp(g_bq);
			sy6970_force_dpdm(g_bq);
		}else{
			sy6970_switch_to_hvdcp(g_bq, HVDCP_5V);
		}
		g_bq->pdqc_setup_5v = 1;
		dev_info(g_bq->dev, "%s: set qc to 5V", __func__);
	} else { // 9v
			if(!(g_bq->is_bq2589x)){
				sy6970_enable_hvdcp(g_bq);
				sy6970_force_dpdm(g_bq);
			}else{
				sy6970_switch_to_hvdcp(g_bq, HVDCP_9V);
			}
	g_bq->pdqc_setup_5v = 0;
	dev_info(g_bq->dev, "%s:qc Force output 9V\n",__func__);
	}	
	return true;
	*/
	sy6970_switch_to_hvdcp(g_bq, HVDCP_5V);
	pr_err("%s:just use 5v\n",__func__);
	return 0;
}

int oplus_sy6970_enable_qc_detect(void)
{
	return 0;
}

bool oplus_sy6970_get_shortc_hw_gpio_status(void)
{
	return true;
}

int oplus_sy6970_chg_set_high_vbus(bool en)
{
	int subtype;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip) {
		dev_info(g_bq->dev, "%s: error\n", __func__);
		return false;
	}

	if(en){
		g_bq->disable_hight_vbus= 0;
		if(chip->charger_volt >7500){
			dev_info(g_bq->dev, "%s:charger_volt already 9v\n", __func__);
			return false;
		}

		if(g_bq->pdqc_setup_5v){
			dev_info(g_bq->dev, "%s:pdqc already setup5v no need 9v\n", __func__);
			return false;
		}

	}else{
		g_bq->disable_hight_vbus= 1;
		if(chip->charger_volt < 5400){
			dev_info(g_bq->dev, "%s:charger_volt already 5v\n", __func__);
			return false;
		}
	}
	
	subtype=oplus_sy6970_get_charger_subtype();
	if(subtype==CHARGER_SUBTYPE_QC){
	  if(en){
		  if(!(g_bq->is_bq2589x)){
			  sy6970_enable_hvdcp(g_bq);
			  sy6970_force_dpdm(g_bq);
		  	}else{
				sy6970_switch_to_hvdcp(g_bq, HVDCP_9V);
			}
		dev_info(g_bq->dev, "%s:QC Force output 9V\n",__func__);
	  	}else{
			if(!(g_bq->is_bq2589x)){
			  sy6970_disable_hvdcp(g_bq);
			  sy6970_force_dpdm(g_bq);
		  }else{
			sy6970_switch_to_hvdcp(g_bq, HVDCP_5V);
		  }
		dev_info(g_bq->dev, "%s: set qc to 5V", __func__);
	  }
	}else{
		dev_info(g_bq->dev, "%s:do nothing\n", __func__);
	}

	return false;
}

bool oplus_sy6970_get_pd_type(void)
{
	struct mtk_charger *info = NULL;

	if(g_bq->chg_consumer != NULL)
		info = g_bq->chg_consumer->cm;

	if(!info){
		dev_info(g_bq->dev, "%s:error\n", __func__);
		return false;
	}

	if(disable_PD){
		dev_info(g_bq->dev, "%s:disable_PD\n", __func__);
		return false;
	}
#ifndef CONFIG_CHARGER_SY6970
	info->enable_pe_4 = false;
	if(mtk_pdc_check_charger(info))
	{
		return true;
	}
#endif // !CONFIG_CHARGER_SY6970


	return false;

}
#define VBUS_VOL_5V	5000
#define VBUS_VOL_9V	9000
#define IBUS_VOL_2A	2000
#define IBUS_VOL_3A	3000
int oplus_sy6970_pd_setup (void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct mtk_charger *info = NULL;
	int vbus = 0, ibus = 0;
	if(g_bq->chg_consumer != NULL)
		info = g_bq->chg_consumer->cm;

	if(!info){
		dev_info(g_bq->dev, "%s:error\n", __func__);
		return false;
	}

	if (!chip) {
		dev_info(g_bq->dev, "%s: error\n", __func__);
		return false;
	}

	if(disable_PD){
		dev_info(g_bq->dev, "%s:disable_PD\n", __func__);
		return false;
	}

	if(g_bq->disable_hight_vbus==1){
		dev_info(g_bq->dev, "%s:disable_hight_vbus\n", __func__);
		return false;
	}

	if (chip->limits.vbatt_pdqc_to_5v_thr > 0 && chip->charger_volt > 7500
		&& chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr&&chip->ui_soc>=85&&chip->icharging > -1000) {
		dev_info(g_bq->dev, "%s: pd set qc to 5V", __func__);
		vbus = VBUS_VOL_5V;
		ibus = IBUS_VOL_3A;
		oplus_pdc_setup(&vbus, &ibus);
		g_bq->pdqc_setup_5v = 1;
	}else{
		dev_info(g_bq->dev, "%s:pd Force output 9V\n",__func__);
		vbus = VBUS_VOL_9V;
		ibus = IBUS_VOL_2A;
		oplus_pdc_setup(&vbus, &ibus);
		g_bq->pdqc_setup_5v = 0;
	}

	return true;
}


extern int oplus_battery_meter_get_battery_voltage(void);
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern int set_rtc_spare_fg_value(int val);
//extern void mt_power_off(void);
extern bool pmic_chrdet_status(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

static void oplus_mt_power_off(void)
{
	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (g_oplus_chip->ac_online != true) {
		if(!mt6357_get_vbus_status())
			kernel_power_off();
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: ac_online is true, return!\n", __func__);
	}
}

static void sy6970_init_work_handler(struct work_struct *work)
{

	if (mt6357_get_vbus_status()) {
		pr_notice("[%s] USB is inserted!\n", __func__);
		mutex_lock(&g_bq->chgdet_en_lock);
		g_bq->psy_online = true;
		sy6970_chgdet_en(g_bq, false);
		sy6970_chgdet_en(g_bq, true);
		determine_initial_status(g_bq);
		oplus_wake_up_usbtemp_thread();
		mutex_unlock(&g_bq->chgdet_en_lock);
	}

}

struct oplus_chg_operations  oplus_chg_sy6970_ops = {
	.dump_registers = oplus_sy6970_dump_registers,
	.kick_wdt = oplus_sy6970_kick_wdt,
	.hardware_init = oplus_sy6970_hardware_init,
	.charging_current_write_fast = oplus_sy6970_set_ichg,
	.set_aicl_point = oplus_sy6970_set_mivr,
	.input_current_write = oplus_sy6970_set_input_current_limit,
	.float_voltage_write = oplus_sy6970_set_cv,
	.term_current_set = oplus_sy6970_set_ieoc,
	.charging_enable = oplus_sy6970_charging_enable,
	.charging_disable = oplus_sy6970_charging_disable,
	.get_charging_enable = oplus_sy6970_is_charging_enabled,
	.charger_suspend = oplus_sy6970_charger_suspend,
	.charger_unsuspend = oplus_sy6970_charger_unsuspend,
	.set_rechg_vol = oplus_sy6970_set_rechg_vol,
	.reset_charger = oplus_sy6970_reset_charger,
	.read_full = oplus_sy6970_is_charging_done,
	.otg_enable = oplus_sy6970_enable_otg,
	.otg_disable = oplus_sy6970_disable_otg,
	.set_charging_term_disable = oplus_sy6970_disable_te,
	.check_charger_resume = oplus_sy6970_check_charger_resume,

	.get_charger_type = oplus_sy6970_get_charger_type,
	.get_charger_volt = mt6357_get_vbus_voltage,
	//.get_charger_volt = battery_get_vbus,
//	int (*get_charger_current)(void);
	.get_chargerid_volt = oplus_sy6970_get_chargerid_volt,
    .set_chargerid_switch_val = oplus_sy6970_set_chargerid_switch_val,
    .get_chargerid_switch_val = oplus_sy6970_get_chargerid_switch_val,
	.check_chrdet_status = (bool (*) (void)) oplus_sy6970_check_chrdet_status,

	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
	.set_power_off = oplus_mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
    .get_chg_current_step = oplus_sy6970_get_chg_current_step,
    .need_to_check_ibatt = oplus_sy6970_need_to_check_ibatt,
    .get_dyna_aicl_result = oplus_sy6970_get_dyna_aicl_result,
    .get_shortc_hw_gpio_status = oplus_sy6970_get_shortc_hw_gpio_status,
//	void (*check_is_iindpm_mode) (void);
    .oplus_chg_get_pd_type = oplus_sy6970_get_pd_type,
    .oplus_chg_pd_setup = oplus_sy6970_pd_setup,
	.get_charger_subtype = oplus_sy6970_get_charger_subtype,
	.set_qc_config = oplus_sy6970_set_qc_config,
	.enable_qc_detect = oplus_sy6970_enable_qc_detect,
#ifndef CONFIG_CHARGER_SY6970
	.oplus_chg_get_pe20_type = NULL,
	.oplus_chg_pe20_setup = NULL,
	.oplus_chg_reset_pe20 = NULL,
#endif // !CONFIG_CHARGER_SY6970

	.oplus_chg_set_high_vbus = oplus_sy6970_chg_set_high_vbus,
	.enable_shipmode = sy6970_enable_shipmode,
	.oplus_chg_set_hz_mode = sy6970_set_hz_mode,

	.get_usbtemp_volt = oplus_get_usbtemp_burn_volt,
	.set_typec_sinkonly = oplus_set_typec_sinkonly,
	.oplus_usbtemp_monitor_condition = oplus_usbtemp_condition,
};

static void retry_detection_work_callback(struct work_struct *work)
{
	sy6970_force_dpdm(g_bq);
	pr_notice("retry BC flow\n");
}

static void aicr_setting_work_callback(struct work_struct *work)
{
	oplus_sy6970_set_aicr(g_bq->aicr);
}

static void charging_current_setting_work(struct work_struct *work)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	u32 uA = g_bq->chg_cur*1000;
	u32 temp_uA;
	int ret = 0;

	if(g_bq->chg_cur > BQ_CHARGER_CURRENT_MAX_MA) {
		uA = BQ_CHARGER_CURRENT_MAX_MA * 1000;
	}
	temp_uA = uA  * current_percent / 100;
	ret = sy6970_set_chargecurrent(g_bq, temp_uA/1000);
	ret = _sy6970_get_ichg(g_bq, &temp_uA);
	uA-=temp_uA;
	chip->sub_chg_ops->charging_current_write_fast(uA/1000);
	if(ret) {
		pr_info("sy6970 set cur:%d %d failed\n", g_bq->chg_cur, temp_uA);
	}
	pr_err("[%s] g_bq->chg_cur = %d, current_percent = %d, temp_uA = %d, uA = %d\n",
			__func__, g_bq->chg_cur, current_percent, temp_uA, uA);
}

static struct of_device_id sy6970_charger_match_table[] = {
	{.compatible = "ti,sy6970",},
	{.compatible = "ti,bq25892",},
	{.compatible = "ti,bq25895",},
	{},
};

MODULE_DEVICE_TABLE(of, sy6970_charger_match_table);

static const struct i2c_device_id sy6970_i2c_device_id[] = {
	{ "sy6970", 0x03 },
	{ "bq25892", 0x00 },
	{ "bq25895", 0x07 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sy6970_i2c_device_id);

static int bq2589x_charger_get_online(struct sy6970 *bq,
				     bool *val)
{
	bool pwr_rdy = false;

	mutex_lock(&bq->attach_lock);
	pwr_rdy = bq->psy_online;
	mutex_unlock(&bq->attach_lock);

	dev_info(bq->dev, "%s: online = %d\n", __func__, pwr_rdy);
	*val = pwr_rdy;
	return 0;
}

static int bq2589x_charger_set_online(struct sy6970 *bq,
				     const union power_supply_propval *val)
{
	return sy6970_enable_chgdet(bq->chg_dev, val->intval);
}

static int bq2589x_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sy6970 *bq = power_supply_get_drvdata(psy);
	enum bq2589x_charging_status chg_stat = BQ2589X_CHG_STATUS_NOT_CHARGING;
	bool pwr_rdy = false, chg_en = false;
	int ret = 0;

	dev_dbg(bq->dev, "%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bq2589x_charger_get_online(bq, &pwr_rdy);
		val->intval = pwr_rdy;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq2589x_charger_get_online(bq, &pwr_rdy);
		ret = sy6970_is_charging_enable(bq->chg_dev, &chg_en);
		ret = bq2589x_get_charging_status(bq, &chg_stat);
		if (!pwr_rdy) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return ret;
		}
		switch (chg_stat) {
		case BQ2589X_CHG_STATUS_PRE_CHARGING:
			/* fallthrough */
		case BQ2589X_CHG_STATUS_FAST_CHARGING:
			if (chg_en)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case BQ2589X_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case BQ2589X_CHG_STATUS_NOT_CHARGING:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			ret = -ENODATA;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = bq->oplus_chg_type;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int bq2589x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct sy6970 *bq = power_supply_get_drvdata(psy);
	int ret;

	dev_dbg(bq->dev, "%s: prop = %d\n", __func__, psp);
	pr_err("%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bq2589x_charger_set_online(bq, val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int bq2589x_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property bq2589x_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc bq2589x_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= bq2589x_charger_properties,
	.num_properties		= ARRAY_SIZE(bq2589x_charger_properties),
	.get_property		= bq2589x_charger_get_property,
	.set_property		= bq2589x_charger_set_property,
	.property_is_writeable	= bq2589x_charger_property_is_writeable,
	.usb_types		= bq2589x_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(bq2589x_charger_usb_types),
};

static char *bq2589x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static int sy6970_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct sy6970 *bq;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	struct power_supply_config charger_cfg = {};
	int ret = 0;

	pr_info("sy6970 probe enter\n");
	bq = devm_kzalloc(&client->dev, sizeof(struct sy6970), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	g_bq = bq;


/*Sidong.Zhao@ODM_WT.BSP.CHG 2019/11/4,for detect hvdcp*/
	bq->chg_consumer =
		charger_manager_get_by_name(&client->dev, "sy6970");

	i2c_set_clientdata(client, bq);
	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->attach_lock);
	mutex_init(&bq->chgdet_en_lock);
	ret = sy6970_detect_device(bq);
	if (ret) {
		pr_err("No sy6970 device found!\n");
		ret = -ENODEV;
		goto err_nodev;
	}

	bq->platform_data = sy6970_parse_dt(node, bq);
	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		ret = -EINVAL;
		goto err_parse_dt;
	}

	ret = sy6970_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		goto err_init;
	}

	bq->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	bq->pre_current_ma = -1;

#ifndef CONFIG_TCPC_CLASS
	Charger_Detect_Init();
#endif
	if (bq->is_bq2589x)
		sy6970_disable_hvdcp(bq);
	else
		sy6970_enable_hvdcp(bq);
	sy6970_disable_maxc(bq);
	sy6970_disable_batfet_rst(bq);
	sy6970_disable_ico(bq);

	INIT_DELAYED_WORK(&bq->sy6970_aicr_setting_work, aicr_setting_work_callback);
	INIT_DELAYED_WORK(&bq->sy6970_retry_adapter_detection, retry_detection_work_callback);	
	INIT_DELAYED_WORK(&bq->sy6970_current_setting_work, charging_current_setting_work);
	INIT_DELAYED_WORK(&bq->init_work, sy6970_init_work_handler);
	
	bq->chg_dev = charger_device_register(bq->chg_dev_name,
					      &client->dev, bq,
					      &sy6970_chg_ops,
					      &sy6970_chg_props);
	if (IS_ERR_OR_NULL(bq->chg_dev)) {
		ret = PTR_ERR(bq->chg_dev);
		goto err_device_register;
	}

	sy6970_register_interrupt(node,bq);

	ret = sysfs_create_group(&bq->dev->kobj, &sy6970_attr_group);
	if (ret){
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_sysfs_create;
	}

	/* power supply register */
	memcpy(&bq->psy_desc,
		&bq2589x_charger_desc, sizeof(bq->psy_desc));
	bq->psy_desc.name = "sy6970";

	charger_cfg.drv_data = bq;
	charger_cfg.of_node = client->dev.of_node;
	charger_cfg.supplied_to = bq2589x_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(bq2589x_charger_supplied_to);
	bq->psy = devm_power_supply_register(&client->dev,
					&bq->psy_desc, &charger_cfg);
	if (IS_ERR_OR_NULL(bq->psy)) {
		dev_notice(&client->dev, "Fail to register power supply dev\n");
		ret = PTR_ERR(bq->psy);
		goto err_register_psy;
	}

	determine_initial_status(bq);

	if (strcmp(bq->chg_dev_name, "primary_chg") == 0) {
		schedule_delayed_work(&bq->init_work, msecs_to_jiffies(14000));
	}

//Junbo.Guo@ODM_WT.BSP.CHG, 2019/12/20, Modify for subcharger
	set_charger_ic(SY6970);
	pr_err("sy6970 probe successfully, Part Num:%d, Revision:%d\n!",
	       bq->part_no, bq->revision);

	return 0;

err_register_psy:
err_sysfs_create:
	charger_device_unregister(bq->chg_dev);
err_device_register:	
err_init:
err_parse_dt:	
//err_match:
err_nodev:
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->chgdet_en_lock);
	devm_kfree(bq->dev, bq);
	return ret;

}

static int sy6970_charger_remove(struct i2c_client *client)
{
	struct sy6970 *bq = i2c_get_clientdata(client);

	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->chgdet_en_lock);

	sysfs_remove_group(&bq->dev->kobj, &sy6970_attr_group);

	return 0;
}

static void sy6970_charger_shutdown(struct i2c_client *client)
{
	if((g_oplus_chip != NULL) && g_bq != NULL) {
		if((g_bq->hvdcp_can_enabled) && (g_oplus_chip->charger_exist)) {
			oplus_sy6970_charging_disable();
		}
	}
}

static struct i2c_driver sy6970_charger_driver = {
	.driver = {
		   .name = "sy6970-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = sy6970_charger_match_table,
		   },

	.probe = sy6970_charger_probe,
	.remove = sy6970_charger_remove,
	.shutdown = sy6970_charger_shutdown,
	.id_table = sy6970_i2c_device_id,

};

module_i2c_driver(sy6970_charger_driver);

MODULE_DESCRIPTION("TI SY6970 Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");


