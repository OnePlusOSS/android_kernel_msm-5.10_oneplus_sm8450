/*
 * SC8547 battery charging driver
*/

#define pr_fmt(fmt)	"[sc8547] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
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
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/proc_fs.h>

#include <trace/events/sched.h>
#include<linux/ktime.h>
#include "oplus_voocphy.h"
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "oplus_sc8547.h"

static struct oplus_voocphy_manager *oplus_voocphy_mg = NULL;
static struct mutex i2c_rw_lock;

static int sc8547_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data);

/************************************************************************/
static int __sc8547_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __sc8547_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}

	return 0;
}

static int sc8547_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_read_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int sc8547_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_write_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}


static int sc8547_update_bits(struct i2c_client *client, u8 reg,
                              u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_read_byte(client, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8547_write_byte(client, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static s32 sc8547_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;

	mutex_lock(&i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static s32 sc8547_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	mutex_lock(&i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		pr_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&i2c_rw_lock);
	return 0;
}

static int sc8547_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		pr_err("failed: chip is null\n");
		return -1;
	}

	//predata
	ret = sc8547_write_word(chip->client, SC8547_REG_31, val);
	if (ret < 0) {
		pr_err("failed: write predata\n");
		return -1;
	}
	pr_info("write predata 0x%0x\n", val);
	return ret;
}

static int sc8547_set_txbuff(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		pr_err("failed: chip is null\n");
		return -1;
	}

	//txbuff
	ret = sc8547_write_word(chip->client, SC8547_REG_2C, val);
	if (ret < 0) {
		pr_err("write txbuff\n");
		return -1;
	}

	return ret;
}

static int sc8547_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	s32 data;
	if (!chip) {
		pr_err("chip is null\n");
		return -1;
	}

	data = sc8547_read_word(chip->client, SC8547_REG_2E);

	if (data < 0) {
		pr_err("sc8547_read_word faile\n");
		return -1;
	}

	VOOCPHY_DATA16_SPLIT(data, chip->voocphy_rx_buff, chip->vooc_flag);
	pr_info("data: 0x%0x, vooc_flag: 0x%0x, vooc_rxdata: 0x%0x\n", data, chip->vooc_flag, chip->voocphy_rx_buff);

	return 0;
}

static void sc8547_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[10] = {0};
	int i = 0;
	u8 data = 0;
	int div_cp_ichg;
	int slave_cp_ichg;

	/*int_flag*/
	sc8547_read_byte(chip->client, SC8547_REG_0F, &data);
	chip->int_flag = data;

	/*parse data_block for improving time of interrupt*/
	i2c_smbus_read_i2c_block_data(chip->client, SC8547_REG_13, 10, data_block);
	for (i=0; i<10; i++) {
		pr_info("data_block[%d] = %u\n", i, data_block[i]);
	}
	chip->cp_ichg = ((data_block[0] << 8) | data_block[1])*1875 / 1000;
	chip->cp_vbus = ((data_block[2] << 8) | data_block[3])*375 / 100;
	chip->cp_vsys = ((data_block[6] << 8) | data_block[7])*125 / 100;
	chip->cp_vbat = ((data_block[8] << 8) | data_block[9])*125 / 100;
	pr_info("cp_ichg = %d cp_vbus = %d, cp_vsys = %d cp_vbat = %d int_flag = %d",
	        chip->cp_ichg, chip->cp_vbus, chip->cp_vsys, chip->cp_vbat, chip->int_flag);
}

static int sc8547_get_cp_ichg(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = {0};
	int i = 0;
	int cp_ichg = 0;
	u8 cp_enable = 0;

	sc8547_get_chg_enable(chip, &cp_enable);

	if(cp_enable == 0)
		return 0;
	/*parse data_block for improving time of interrupt*/
	i2c_smbus_read_i2c_block_data(chip->client, SC8547_REG_13, 2, data_block);

	cp_ichg = ((data_block[0] << 8) | data_block[1])*1875 / 1000;

	return cp_ichg;
}

int sc8547_get_cp_vbat(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = {0};
	int i = 0;

	/*parse data_block for improving time of interrupt*/
	i2c_smbus_read_i2c_block_data(chip->client, SC8547_REG_1B, 2, data_block);

	chip->cp_vbat = ((data_block[0] << 8) | data_block[1])*125 / 100;

	return chip->cp_vbat;
}

/*********************************************************************/
static int sc8547_reg_reset(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = SC8547_RESET_REG;
	else
		val = SC8547_NO_REG_RESET;

	val <<= SC8547_REG_RESET_SHIFT;

	ret = sc8547_update_bits(chip->client, SC8547_REG_07,
	                         SC8547_REG_RESET_MASK, val);

	return ret;
}


static int sc8547_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_07, data);
	if (ret < 0) {
		pr_err("SC8547_REG_07\n");
		return -1;
	}

	*data = *data >> 7;

	return ret;
}

static int sc8547_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_11, data);
	if (ret < 0) {
		pr_err("SC8547_REG_11\n");
		return -1;
	}

	*data = *data >> 7;

	return ret;
}

static u8 sc8547_get_int_value(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		pr_err("%s: chip null\n", __func__);
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_0F, &data);
	if (ret < 0) {
		pr_err(" read SC8547_REG_0F failed\n");
		return -1;
	}

	return data;
}

static int sc8547_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		return sc8547_write_byte(chip->client, SC8547_REG_07, 0x84);
	else
		return sc8547_write_byte(chip->client, SC8547_REG_07, 0x04);
}

static void sc8547_set_pd_svooc_config(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret = 0;
	u8 reg05_data = 0;
	if (!chip) {
		pr_err("Failed\n");
		return;
	}

	if (enable)
		sc8547_write_byte(chip->client, SC8547_REG_05, 0xA8);
	else
		sc8547_write_byte(chip->client, SC8547_REG_05, 0x28);

	ret = sc8547_read_byte(chip->client, SC8547_REG_05, &reg05_data);
	if (ret < 0) {
		pr_err("SC8547_REG_05\n");
		return;
	}
	pr_err("pd_svooc config SC8547_REG_05 = %d\n", reg05_data);
}

static bool sc8547_get_pd_svooc_config(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		pr_err("Failed\n");
		return false;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_05, &data);
	if (ret < 0) {
		pr_err("SC8547_REG_05\n");
		return false;
	}

	pr_err("SC8547_REG_05 = 0x%0x\n", data);

	data = data >> 7;
	if (data == 1)
		return true;
	else
		return false;
}

static int sc8547_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		return sc8547_write_byte(chip->client, SC8547_REG_11, 0x80);
	else
		return sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);
}

void sc8547_send_handshake(struct oplus_voocphy_manager *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x81);
}


static int sc8547_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	//turn off mos
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x04);

	//hwic config with plugout
	sc8547_write_byte(chip->client, SC8547_REG_00, 0x2E);
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x07);
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);
	sc8547_write_byte(chip->client, SC8547_REG_05, 0x28);
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);

	//clear tx data
	sc8547_write_byte(chip->client, SC8547_REG_2C, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_2D, 0x00);

	//disable vooc phy irq
	sc8547_write_byte(chip->client, SC8547_REG_30, 0x7f);

	//set D+ HiZ
	sc8547_write_byte(chip->client, SC8547_REG_21, 0xc0);

	//select big bang mode

	//disable vooc
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00);

	//set predata
	sc8547_write_word(chip->client, SC8547_REG_31, 0x0);
	//sc8547_set_predata(0x0);

	pr_info ("oplus_vooc_reset_voocphy done");

	return VOOCPHY_SUCCESS;
}


static int sc8547_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 value;

	//to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us
	//set predata
	sc8547_write_word(chip->client, SC8547_REG_31, 0x0);
	//sc8547_set_predata(0x0);
	sc8547_read_byte(chip->client, SC8547_REG_3A, &value);
	value = value | (3 << 5);
	sc8547_write_byte(chip->client, SC8547_REG_3A, value);

	//dpdm
	sc8547_write_byte(chip->client, SC8547_REG_21, 0x21);
	sc8547_write_byte(chip->client, SC8547_REG_22, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xD1);

	//clear tx data
	sc8547_write_byte(chip->client, SC8547_REG_2C, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_2D, 0x00);

	//vooc
	sc8547_write_byte(chip->client, SC8547_REG_30, 0x05);
	sc8547_send_handshake(chip);

	pr_info ("oplus_vooc_reactive_voocphy done");

	return VOOCPHY_SUCCESS;
}

static irqreturn_t sc8547_charger_interrupt(int irq, void *dev_id)
{
	struct oplus_voocphy_manager *chip = dev_id;

	if (!chip) {
		return IRQ_HANDLED;
	}

	return oplus_voocphy_interrupt_handler(chip);
}

static int sc8547_init_device(struct oplus_voocphy_manager *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x0);	//ADC_CTRL:disable
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x7);	//
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);	//VBUS_OVP:10 2:1 or 1:1V
	sc8547_write_byte(chip->client, SC8547_REG_00, 0x2E);	//VBAT_OVP:4.65V
	sc8547_write_byte(chip->client, SC8547_REG_05, 0x28);	//IBUS_OCP_UCP:3.6A
	sc8547_write_byte(chip->client, SC8547_REG_01, 0xbf);
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00);	//VOOC_CTRL:disable
	sc8547_write_byte(chip->client, SC8547_REG_3A, 0x60);

	return 0;
}

static int sc8547_init_vooc(struct oplus_voocphy_manager *chip)
{
	u8 value;
	pr_err(" >>>>start init vooc\n");

	sc8547_reg_reset(chip, true);
	sc8547_init_device(chip);

	//to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us
	//SET PREDATA
	sc8547_write_word(chip->client, SC8547_REG_31, 0x0);
	//sc8547_set_predata(0x0);
	sc8547_read_byte(chip->client, SC8547_REG_3A, &value);
	value = value | (1 << 6);
	sc8547_write_byte(chip->client, SC8547_REG_3A, value);
	//sc8547_read_byte(__sc, SC8547_REG_3A, &value);
	pr_err("read value %d\n", value);

	//dpdm
	sc8547_write_byte(chip->client, SC8547_REG_21, 0x21);
	sc8547_write_byte(chip->client, SC8547_REG_22, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xD1);

	//vooc
	sc8547_write_byte(chip->client, SC8547_REG_30, 0x05);

	return 0;
}

static int sc8547_irq_gpio_init(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	chip->irq_gpio = of_get_named_gpio(node,
	                                   "qcom,irq_gpio", 0);
	if (chip->irq_gpio < 0) {
		pr_err("chip->irq_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->irq_gpio)) {
			rc = gpio_request(chip->irq_gpio,
			                  "irq_gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n",
				         chip->irq_gpio);
			}
		}
		pr_err("chip->irq_gpio =%d\n", chip->irq_gpio);
	}
	//irq_num
	chip->irq = gpio_to_irq(chip->irq_gpio);
	pr_err("irq way1 chip->irq =%d\n",chip->irq);

	/* set voocphy pinctrl*/
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->charging_inter_active =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_active");
	if (IS_ERR_OR_NULL(chip->charging_inter_active)) {
		chg_err(": %d Failed to get the state pinctrl handle\n", __LINE__);
		return -EINVAL;
	}

	chip->charging_inter_sleep =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_sleep");
	if (IS_ERR_OR_NULL(chip->charging_inter_sleep)) {
		chg_err(": %d Failed to get the state pinctrl handle\n", __LINE__);
		return -EINVAL;
	}

	//irq active
	gpio_direction_input(chip->irq_gpio);
	pinctrl_select_state(chip->pinctrl, chip->charging_inter_active); /* no_PULL */

	rc = gpio_get_value(chip->irq_gpio);
	pr_err("irq chip->irq_gpio input =%d irq_gpio_stat = %d\n", chip->irq_gpio, rc);

	return 0;
}

static int sc8547_irq_register(struct oplus_voocphy_manager *chip)
{
	int ret;

	sc8547_irq_gpio_init(chip);
	pr_err("wenboyu sc8547 chip->irq = %d\n", chip->irq);
	if (chip->irq) {
		ret = request_threaded_irq(chip->irq, NULL,
		                           sc8547_charger_interrupt,
		                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		                           "sc8547_charger_irq", chip);
		if (ret < 0) {
			pr_debug("request irq for irq=%d failed, ret =%d\n",
			         chip->irq, ret);
			return ret;
		}
		enable_irq_wake(chip->irq);
	}
	pr_debug("request irq ok\n");

	return ret;
}

static int sc8547_svooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	//sc8547_write_byte(sc, SC8547_REG_00, 0x2E);
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01);	//VAC_OVP:12v
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);	//VBUS_OVP:10v
	sc8547_write_byte(chip->client, SC8547_REG_05, 0x28);	//IBUS_OCP_UCP:3.6A
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x03);	//WD:1000ms
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x80);	//ADC_CTRL:ADC_EN
	//sc8547_write_byte(chip->client, SC8547_REG_2B, 0x81);	//VOOC_CTRL,send handshake
	//oplus_vooc_send_handshake_seq();
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xd1);	//Loose_det=1
	sc8547_write_byte(chip->client, SC8547_REG_3A, 0x60);
	return 0;
}

static int sc8547_vooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	//sc8547_write_byte(sc, SC8547_REG_00, 0x2E);
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x07);	//VAC_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);	//VBUS_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_05, 0x2c);	//IBUS_OCP_UCP:
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x83);	//WD:5000ms
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x80);	//ADC_CTRL:
	//sc8547_write_byte(chip->client, SC8547_REG_2B, 0x81);	//VOOC_CTRL, send handshake
	//oplus_vooc_send_handshake_seq();
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xd1);	//Loose_det
	sc8547_write_byte(chip->client, SC8547_REG_3A, 0x60);
	return 0;
}

static int sc8547_5v2a_hw_setting(struct oplus_voocphy_manager *chip)
{
	//sc8547_write_byte(sc, SC8547_REG_00, 0x2E);
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x07);	//VAC_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);	//VBUS_OVP:
	//sc8547_write_byte(__sc, SC8547_REG_05, 0x2c);	//IBUS_OCP_UCP:
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x04);
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x00);	//WD:

	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);	//ADC_CTRL:
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00);	//VOOC_CTRL
	//sc8547_write_byte(__sc, SC8547_REG_31, 0x01);	//
	//sc8547_write_byte(__sc, SC8547_REG_32, 0x80);	//
	//sc8547_write_byte(__sc, SC8547_REG_33, 0xd1);	//Loose_det
	return 0;
}

static int sc8547_pdqc_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_00, 0x2E);	//VAC_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01);	//VAC_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x50);	//VBUS_OVP:
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x04);
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x00);	//WD:
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);	//ADC_CTRL:
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00);	//VOOC_CTRL
	return 0;
}

static int sc8547_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	if (!chip) {
		pr_err("chip is null exit\n");
		return -1;
	}
	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		sc8547_init_device(chip);
		pr_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_SVOOC:
		sc8547_svooc_hw_setting(chip);
		pr_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		sc8547_vooc_hw_setting(chip);
		pr_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		sc8547_5v2a_hw_setting(chip);
		pr_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		sc8547_pdqc_hw_setting(chip);
		pr_info("SETTING_REASON_PDQC\n");
		break;
	default:
		pr_err("do nothing\n");
		break;
	}
	return 0;
}

static ssize_t sc8547_show_registers(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8547");
	for (addr = 0x0; addr <= 0x3C; addr++) {
		if((addr < 0x24) || (addr > 0x2B && addr < 0x33)
		   || addr == 0x36 || addr == 0x3C) {
			ret = sc8547_read_byte(chip->client, addr, &val);
			if (ret == 0) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,
				               "Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}

	return idx;
}

static ssize_t sc8547_store_register(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x3C)
		sc8547_write_byte(chip->client, (unsigned char)reg, (unsigned char)val);

	return count;
}


static DEVICE_ATTR(registers, 0660, sc8547_show_registers, sc8547_store_register);

static void sc8547_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}


static struct of_device_id sc8547_charger_match_table[] = {
	{
		.compatible = "sc,sc8547-master",
	},
	{},
};

static int sc8547_gpio_init(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get chargerid_switch_gpio pinctrl fail\n");
		return -EINVAL;
	}

	chip->charger_gpio_sw_ctrl2_high =
	    pinctrl_lookup_state(chip->pinctrl,
	                         "switch1_act_switch2_act");
	if (IS_ERR_OR_NULL(chip->charger_gpio_sw_ctrl2_high)) {
		chg_err("get switch1_act_switch2_act fail\n");
		return -EINVAL;
	}

	chip->charger_gpio_sw_ctrl2_low =
	    pinctrl_lookup_state(chip->pinctrl,
	                         "switch1_sleep_switch2_sleep");
	if (IS_ERR_OR_NULL(chip->charger_gpio_sw_ctrl2_low)) {
		chg_err("get switch1_sleep_switch2_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->pinctrl,
	                     chip->charger_gpio_sw_ctrl2_low);

	printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip is ready!\n", __func__);
	return 0;
}

static int sc8547_parse_dt(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node * node = NULL;

	if (!chip) {
		pr_debug("chip null\n");
		return -1;
	}

	/* Parsing gpio switch gpio47*/
	node = chip->dev->of_node;
	chip->switch1_gpio = of_get_named_gpio(node,
	                                       "qcom,charging_switch1-gpio", 0);
	if (chip->switch1_gpio < 0) {
		pr_debug("chip->switch1_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->switch1_gpio)) {
			rc = gpio_request(chip->switch1_gpio,
			                  "charging-switch1-gpio");
			if (rc) {
				pr_debug("unable to request gpio [%d]\n",
				         chip->switch1_gpio);
			} else {
				rc = sc8547_gpio_init(chip);
				if (rc)
					chg_err("unable to init charging_sw_ctrl2-gpio:%d\n",
					        chip->switch1_gpio);
			}
		}
		pr_debug("chip->switch1_gpio =%d\n", chip->switch1_gpio);
	}

	return 0;
}

static void sc8547_set_switch_fast_charger(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		pr_err("sc8547_set_switch_fast_charger chip null\n");
		return;
	}

	pinctrl_select_state(chip->pinctrl, chip->charger_gpio_sw_ctrl2_high);
	gpio_direction_output(chip->switch1_gpio, 1);	/* out 1*/

	pr_err("switch switch2 %d to fast finshed\n", gpio_get_value(chip->switch1_gpio));

	return;
}

static void sc8547_set_switch_normal_charger(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		pr_err("sc8547_set_switch_normal_charger chip null\n");
		return;
	}

	pinctrl_select_state(chip->pinctrl, chip->charger_gpio_sw_ctrl2_low);
	if (chip->switch1_gpio > 0) {
		gpio_direction_output(chip->switch1_gpio, 0);	/* in 0*/
	}

	pr_err("switch switch2 %d to normal finshed\n", gpio_get_value(chip->switch1_gpio));

	return;
}

static void sc8547_set_switch_mode(struct oplus_voocphy_manager *chip, int mode)
{
	switch (mode) {
	case VOOC_CHARGER_MODE:
		sc8547_set_switch_fast_charger(chip);
		break;
	case NORMAL_CHARGER_MODE:
	default:
		sc8547_set_switch_normal_charger(chip);
		break;
	}

	return;
}

static struct oplus_voocphy_operations oplus_sc8547_ops = {
	.hw_setting		= sc8547_hw_setting,
	.init_vooc		= sc8547_init_vooc,
	.set_predata		= sc8547_set_predata,
	.set_txbuff		= sc8547_set_txbuff,
	.get_adapter_info	= sc8547_get_adapter_info,
	.update_data		= sc8547_update_data,
	.get_chg_enable		= sc8547_get_chg_enable,
	.set_chg_enable		= sc8547_set_chg_enable,
	.reset_voocphy		= sc8547_reset_voocphy,
	.reactive_voocphy	= sc8547_reactive_voocphy,
	.set_switch_mode	= sc8547_set_switch_mode,
	.send_handshake		= sc8547_send_handshake,
	.get_cp_vbat		= sc8547_get_cp_vbat,
	.get_int_value		= sc8547_get_int_value,
	.get_adc_enable		= sc8547_get_adc_enable,
	.set_adc_enable		= sc8547_set_adc_enable,
	.get_ichg			= sc8547_get_cp_ichg,
	.set_pd_svooc_config = sc8547_set_pd_svooc_config,
	.get_pd_svooc_config = sc8547_get_pd_svooc_config,
};

static int sc8547_charger_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
{
	struct oplus_voocphy_manager *chip;
	int ret;

	pr_err("sc8547_charger_probe enter!\n");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&i2c_rw_lock);

	i2c_set_clientdata(client, chip);

	sc8547_create_device_node(&(client->dev));

	ret = sc8547_parse_dt(chip);

	sc8547_reg_reset(chip, true);

	sc8547_init_device(chip);

	ret = sc8547_irq_register(chip);
	if (ret < 0)
		goto err_1;

	chip->ops = &oplus_sc8547_ops;

	oplus_voocphy_init(chip);

	oplus_voocphy_mg = chip;

	init_proc_voocphy_debug();

	pr_err("sc8547_parse_dt successfully!\n");

	return 0;

err_1:
	pr_err("sc8547 probe err_1\n");
	return ret;
}

static void sc8547_charger_shutdown(struct i2c_client *client)
{
	sc8547_write_byte(client, SC8547_REG_11, 0x00);
	sc8547_write_byte(client, SC8547_REG_21, 0x00);

	return;
}

static const struct i2c_device_id sc8547_charger_id[] = {
	{"sc8547-master", 0},
	{},
};

static struct i2c_driver sc8547_charger_driver = {
	.driver		= {
		.name	= "sc8547-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sc8547_charger_match_table,
	},
	.id_table	= sc8547_charger_id,

	.probe		= sc8547_charger_probe,
	.shutdown	= sc8547_charger_shutdown,
};

static int __init sc8547_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&sc8547_charger_driver) != 0) {
		chg_err(" failed to register sc8547 i2c driver.\n");
	} else {
		chg_debug(" Success to register sc8547 i2c driver.\n");
	}

	return ret;
}

subsys_initcall(sc8547_subsys_init);
//module_i2c_driver(sc8547_charger_driver);

MODULE_DESCRIPTION("SC SC8547 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aiden-yu@southchip.com");
