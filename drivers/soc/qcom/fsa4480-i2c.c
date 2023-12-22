// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */

#ifndef OPLUS_ARCH_EXTENDS
#define OPLUS_ARCH_EXTENDS
#endif /* OPLUS_ARCH_EXTENDS */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>
#include <linux/usb/ucsi_glink.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/qti-regmap-debugfs.h>

#ifdef OPLUS_ARCH_EXTENDS
#include <linux/of_gpio.h>
#include <linux/gpio.h>
/*add WAS4783 support*/
#include <oplus_audio_switch.h>
/*add for 3rd protocal stack notifier*/
#if IS_ENABLED(CONFIG_TCPC_CLASS)
#include <tcpci.h>
#include <tcpm.h>
#include <tcpci_typec.h>
#endif
#endif /* OPLUS_ARCH_EXTENDS */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#endif

#define FSA4480_I2C_NAME	"fsa4480-driver"

#ifdef OPLUS_ARCH_EXTENDS
/*add DIO4480/WAS4783 support*/
#define HL5280_DEVICE_REG_VALUE 0x49
#define DIO4480_DEVICE_REG_VALUE 0xF1
#define WAS4783_DEVICE_REG_VALUE 0x31
#define INVALID_DEVICE_REG_VALUE 0x00

#define FSA4480_DEVICE_ID       0x00
#endif /* OPLUS_ARCH_EXTENDS */
#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#ifdef OPLUS_ARCH_EXTENDS
#define FSA4480_SWITCH_STATUS0  0x06
#endif /* OPLUS_ARCH_EXTENDS */
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#ifdef OPLUS_ARCH_EXTENDS
#define FSA4480_FUN_EN          0x12
#define FSA4480_JACK_STATUS     0x17
#endif /* OPLUS_ARCH_EXTENDS */
#define FSA4480_RESET           0x1E

#ifdef OPLUS_ARCH_EXTENDS
/*
 * 0x1~0xff == 100us~25500us
 */
#define DEFAULT_SWITCH_DELAY		0x12
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
#undef dev_dbg
#define dev_dbg dev_info
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/*add DIO4480/WAS4783 support*/
enum switch_vendor {
    FSA4480 = 0,
    HL5280,
    DIO4480,
    WAS4783
};

/*add for 3rd protocal stack notifier*/
#if IS_ENABLED(CONFIG_TCPC_CLASS)
static int probe_retry = 0;
#endif

static int chipid_read_retry = 0;
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/*add WAS4783 support*/
#define GET_BIT(x, bit)                  ((x & (1 << bit)) >> bit)  /* Get bit x */
#define GET_BITS(_var, _index, _width)   (((_var) >> (_index)) & ((0x1 << (_width)) - 1))
#define CLEAR_BIT(x, bit)                (x &= ~(1 << bit))  /* Reset bit x */
#define SET_BIT(x, bit)                  (x |= (1 << bit))   /* Set bit x */

enum TYPEC_AUDIO_SWITCH_STATE {
	TYPEC_AUDIO_SWITCH_STATE_DPDM          = 0x0,
	TYPEC_AUDIO_SWITCH_STATE_FAST_CHG      = 0x1,
	TYPEC_AUDIO_SWITCH_STATE_AUDIO         = 0x1 << 1,
	TYPEC_AUDIO_SWITCH_STATE_UNKNOW        = 0x1 << 2,
	TYPEC_AUDIO_SWITCH_STATE_SUPPORT       = 0x1 << 4,
	TYPEC_AUDIO_SWITCH_STATE_NO_RAM,
	TYPEC_AUDIO_SWITCH_STATE_I2C_ERR       = 0x1 << 8,
	TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM = 0x1 << 9,
};

enum DNDP_STATE {
	DNL_OPEN_OR_DPR_OPEN      = 0x0, /* DN_L switch is open, or DN_R switch is open */
	DNL_DN_OR_DPR_DP          = 0x1, /* DN_L switch is connected to DN, or DN_R switch is connected to DP */
	DNL_L_OR_DPR_R            = 0x2, /* DN_L switch is connected to L, or DP_R switch is connected to R */
	DNL_DN2_OR_DPR_DP2        = 0x3, /* DN_L switch is connected to DN2, or DP_R switch is connected to DP2 */
};
/* SWITCH STATUS 0 */
#define DEFAULT_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L    (2)
#define DEFAULT_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L    (0)
#endif /* OPLUS_ARCH_EXTENDS */

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct notifier_block ucsi_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
	#ifdef OPLUS_ARCH_EXTENDS
	unsigned int hs_det_pin;
	#endif /* OPLUS_ARCH_EXTENDS */

	#ifdef OPLUS_ARCH_EXTENDS
	/*add DIO4480 support*/
	enum switch_vendor vendor;
	/*add for 3rd usb protocal support*/
	unsigned int usb_protocal;
	/*add WAS4783 support*/
	struct notifier_block chg_nb;
	struct mutex noti_lock;
	/*add for 3rd protocal stack notifer*/
	#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcpc_device *tcpc;
	#endif
	#endif
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_RESET,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	#ifdef OPLUS_ARCH_EXTENDS
	{FSA4480_SWITCH_CONTROL, 0x18},
	#endif /* OPLUS_ARCH_EXTENDS */
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
#ifdef OPLUS_ARCH_EXTENDS
	{FSA4480_DELAY_L_MIC, DEFAULT_SWITCH_DELAY},
#else
	{FSA4480_DELAY_L_MIC, 0x00},
#endif /* OPLUS_ARCH_EXTENDS */
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

#ifdef OPLUS_ARCH_EXTENDS
/*2021/07/31, add DIO4480 support*/
int fsa4480_get_chip_vendor(struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;


	return fsa_priv->vendor;
}
EXPORT_SYMBOL(fsa4480_get_chip_vendor);
#endif

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	u32 prev_control, prev_enable;

	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &prev_control);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &prev_enable);
	if (prev_control == switch_control && prev_enable == switch_enable) {
		dev_dbg(fsa_priv->dev, "%s: settings unchanged\n", __func__);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);

#ifdef OPLUS_ARCH_EXTENDS
/*add DIO4480 support*/
	if(fsa_priv->vendor == DIO4480) {
		regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
		usleep_range(1000, 1005);
	}
#endif

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
#ifdef OPLUS_ARCH_EXTENDS
	usleep_range(DEFAULT_SWITCH_DELAY*100, DEFAULT_SWITCH_DELAY*100+50);
#endif /* OPLUS_ARCH_EXTENDS */
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb, struct fsa4480_priv, ucsi_nb);
	struct device *dev;
#ifndef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
	enum typec_accessory acc = ((struct ucsi_glink_constat_info *)ptr)->acc;
#else
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcp_notify *noti = ptr;
	int old_state = TYPEC_UNATTACHED;
	int new_state = TYPEC_UNATTACHED;
#endif
	enum typec_accessory acc = TYPEC_ACCESSORY_NONE;
#endif /* OPLUS_ARCH_EXTENDS */

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

#ifdef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
	if (fsa_priv->usb_protocal != 1) {
		acc = ((struct ucsi_glink_constat_info *)ptr)->acc;
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifndef OPLUS_ARCH_EXTENDS
/*removed for test*/
	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
			__func__, acc, fsa_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);
#else
	if (fsa_priv->usb_protocal == 1) {
#if IS_ENABLED(CONFIG_TCPC_CLASS)
		dev_err(dev, "%s: USB change event received, new_state:%d, old_state:%d\n",
				__func__, noti->typec_state.new_state, noti->typec_state.old_state);
#endif
	} else {
		dev_err(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
				__func__, acc, fsa_priv->usbc_mode.counter,
				TYPEC_ACCESSORY_AUDIO);
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
	if (fsa_priv->usb_protocal == 1) {
#if IS_ENABLED(CONFIG_TCPC_CLASS)
		switch (evt) {
		case TCP_NOTIFY_TYPEC_STATE:
			old_state = noti->typec_state.old_state;
			new_state = noti->typec_state.new_state;
			if (old_state == TYPEC_UNATTACHED &&
			    new_state == TYPEC_ATTACHED_AUDIO) {
				dev_err(dev, "Audio plug in\n");
				/* enable AudioAccessory connection */
				acc = TYPEC_ACCESSORY_AUDIO;
			} else if (old_state == TYPEC_ATTACHED_AUDIO &&
				   new_state == TYPEC_UNATTACHED) {
				dev_err(dev, "Audio plug out\n");
				/* disable AudioAccessory connection */
				acc = TYPEC_ACCESSORY_NONE;
			}
			break;
		default:
			return 0;
		}
#endif
	}
#endif /* OPLUS_ARCH_EXTENDS */

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
	case TYPEC_ACCESSORY_NONE:
		if (atomic_read(&(fsa_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(fsa_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n",
			__func__);
		pm_stay_awake(fsa_priv->dev);
		queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);
		break;
	default:
		break;
	}

	return 0;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	int mode;
	struct device *dev;
	#ifdef OPLUS_ARCH_EXTENDS
	unsigned int switch_status = 0;
	unsigned int jack_status = 0;
	#endif /* OPLUS_ARCH_EXTENDS */

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);
	/* get latest mode again within locked context */
	mode = atomic_read(&(fsa_priv->usbc_mode));

	dev_dbg(dev, "%s: setting GPIOs active = %d\n",
		__func__, mode != TYPEC_ACCESSORY_NONE);

	#ifdef OPLUS_ARCH_EXTENDS
	dev_info(dev, "%s: USB mode %d\n", __func__, mode);
	#endif /* OPLUS_ARCH_EXTENDS */

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
		/* activate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);

		#ifdef OPLUS_ARCH_EXTENDS
		/*add DIO4480 support*/
		if (fsa_priv->vendor != DIO4480) {
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x45);
			usleep_range(4000, 4005);
			dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);

			regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &jack_status);
			dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_JACK_STATUS, jack_status);
			if (jack_status & 0x2) {
				//for 3 pole, mic switch to SBU2
				dev_info(dev, "%s: set mic to sbu2 for 3 pole.\n", __func__);
				fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
				usleep_range(4000, 4005);
			}
		}
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS0, switch_status);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS1, switch_status);
		#endif /* OPLUS_ARCH_EXTENDS */

		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
					     mode, NULL);

		#ifdef OPLUS_ARCH_EXTENDS
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			dev_info(dev, "%s: set hs_det_pin to low.\n", __func__);
			gpio_direction_output(fsa_priv->hs_det_pin, 0);
		}
		#endif /* OPLUS_ARCH_EXTENDS */

		break;
	case TYPEC_ACCESSORY_NONE:
		#ifdef OPLUS_ARCH_EXTENDS
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			dev_info(dev, "%s: set hs_det_pin to high.\n", __func__);
			gpio_direction_output(fsa_priv->hs_det_pin, 1);
		}
		#endif /* OPLUS_ARCH_EXTENDS */

		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);

		/* deactivate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

#ifdef OPLUS_ARCH_EXTENDS
static int typec_switch_to_fast_charger(struct fsa4480_priv *fsa_priv, unsigned long to_fast_charger)
{
	struct device *dev;
	int ret = 0;
	unsigned int reg_val = 0, dn_l_status = 0, dp_r_status = 0, width = 2;

	if (!fsa_priv) {
		pr_err("%s, fsa_priv is NULL", __func__);
		return -EINVAL;
	}

	dev = fsa_priv->dev;
	if (!dev) {
		pr_err("%s, fsa_priv->dev is NULL", __func__);
		return -EINVAL;
	}

	if (fsa_priv->vendor != WAS4783) {
		dev_err(dev, "%s, current chip 0x%02x, is not supported!", __func__, fsa_priv->vendor);
		return -EINVAL;
	}

	mutex_lock(&fsa_priv->noti_lock);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &reg_val);
	dn_l_status = GET_BITS(reg_val, DEFAULT_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L, width);
	dp_r_status = GET_BITS(reg_val, DEFAULT_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L, width);
	if (dn_l_status == 0x02 || dp_r_status == 0x02) {
		dev_info(dev,"%s, switching from headphone to charger is prohibited!\n", __func__);
		mutex_unlock(&fsa_priv->noti_lock);
		return -EPERM;
	}

	dev_info(dev, "%s: to_fast_charger = %ld\n", __func__, to_fast_charger);
	if (to_fast_charger) {
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x80);
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x98);
		usleep_range(10000, 10005);
		dev_info(dev, "%s, charger plugin. set to switch mode", __func__);
	} else {
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x18);
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x98);
		dev_info(dev, "%s, charger plugout. set to usb mode", __func__);
	}

	mutex_unlock(&fsa_priv->noti_lock);
	return ret;
}

/*add WAS4783 support*/
/* bit3~0:
* 0000 DPDM, 0001 fast charge, 0010 Headphone 0100 unknown
*
* bit7~4:
* 0000 not support 1to3 switch
* 0001 support 1to3 switch

* bit11~bit8:
* 0000 noram
* 0001 iic err
* 0010 invalid param */
static int typec_switch_get_status(struct fsa4480_priv *fsa_priv)
{
	struct device *dev = NULL;
	int rc = 0;
	unsigned int reg_status = 0, reg_val = 0, reg_l_shift = 0, reg_r_shift = 0;
	unsigned int width = 2, dn_l_status = 0, dp_r_status = 0;

	if (!fsa_priv) {
		pr_err("%s, fsa_priv is NULL", __func__);
		rc |= TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM;
		goto err_handler;
	}

	dev = fsa_priv->dev;
	if (!dev) {
		pr_err("%s, fsa_priv->dev is NULL", __func__);
		rc |= TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM;
		goto err_handler;
	}

	mutex_lock(&fsa_priv->noti_lock);
	if (fsa_priv->vendor == WAS4783) {
		rc |= TYPEC_AUDIO_SWITCH_STATE_SUPPORT;
	}

	reg_status = FSA4480_SWITCH_STATUS0;
	reg_l_shift = DEFAULT_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L;
	reg_r_shift = DEFAULT_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L;

	regmap_read(fsa_priv->regmap, reg_status, &reg_val);
	dev_info(dev, "%s, reg[0x%02x] = 0x%02x", __func__, reg_status, reg_val);
	dn_l_status = GET_BITS(reg_val, reg_l_shift, width);
	dp_r_status = GET_BITS(reg_val, reg_r_shift, width);

	dev_info(dev, "%s, dn_l_status = 0x%02x, dp_r_status = 0x%02x", __func__, dn_l_status, dp_r_status);
	switch (dn_l_status) {
	case DNL_OPEN_OR_DPR_OPEN:
		dev_info(dev, "%s, DN_L Switch Open/Not Connected", __func__);
		break;
	case DNL_DN_OR_DPR_DP:
		dev_info(dev, "%s, DN_L connected to DN1", __func__);
		break;
	case DNL_L_OR_DPR_R: // Headphone
		dev_info(dev, "%s, DN_L connected to L", __func__);
		break;
	case DNL_DN2_OR_DPR_DP2: // Fast Charger
		dev_info(dev, "%s, DN_L connected to DN2", __func__);
		break;
	default:
		break;
	}

	switch (dp_r_status) {
	case DNL_OPEN_OR_DPR_OPEN:
		dev_info(dev, "%s, DP_R Switch Open/Not Connected", __func__);
		break;
	case DNL_DN_OR_DPR_DP:
		dev_info(dev, "%s, DP_R connected to DP1", __func__);
		break;
	case DNL_L_OR_DPR_R: // Headphone
		dev_info(dev, "%s, DP_R connected to R", __func__);
		break;
	case DNL_DN2_OR_DPR_DP2: // Fast Charger
		dev_info(dev, "%s, DP_R connected to DP2", __func__);
		break;
	default:
		break;
	}

	if ((dn_l_status == DNL_DN_OR_DPR_DP) && (dp_r_status == DNL_DN_OR_DPR_DP)) { // Charger
		rc |= TYPEC_AUDIO_SWITCH_STATE_DPDM;
	} else if ((dn_l_status == DNL_DN2_OR_DPR_DP2) && (dp_r_status == DNL_DN2_OR_DPR_DP2)) { // Fast Charger
		rc |= TYPEC_AUDIO_SWITCH_STATE_FAST_CHG;
	} else if ((dn_l_status == DNL_L_OR_DPR_R) && (dp_r_status == DNL_L_OR_DPR_R)) { // Headphone
		rc |= TYPEC_AUDIO_SWITCH_STATE_AUDIO;
	} else { // Unknown
		rc |= TYPEC_AUDIO_SWITCH_STATE_UNKNOW;
		pr_err("%s, Typec audio switch state is unknow", __func__);
	}
	mutex_unlock(&fsa_priv->noti_lock);

err_handler:
	return rc;
}

static int typec_switch_chg_event_changed(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
				container_of(nb, struct fsa4480_priv, chg_nb);
	switch (event) {
	case TYPEC_AUDIO_SWITCH_STATE_DPDM:
	case TYPEC_AUDIO_SWITCH_STATE_FAST_CHG:
		typec_switch_to_fast_charger(fsa_priv, event);
		break;
	case TYPEC_AUDIO_SWITCH_STATE_AUDIO:
		return typec_switch_get_status(fsa_priv);
	default:
		break;
	}

	return NOTIFY_OK;
}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/*Add for dynamic check cross*/
int fsa4480_check_cross_conn(struct device_node *node)
{
	int ret = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client) {
		pr_err("%s: fsa4480 client is NULL\n", __func__);
		return 0;
	}

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv) {
		pr_err("%s: fsa_priv is NULL\n", __func__);
		return 0;
	}

	dev_dbg(fsa_priv->dev, "%s: registered vendor for %d\n",
		__func__, fsa_priv->vendor);

	switch (fsa_priv->vendor) {
	case FSA4480:
	case HL5280:
	case WAS4783:
	    ret = 0;
	    break;
	case DIO4480:
	    ret = 1;
	    break;
	default:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(fsa4480_check_cross_conn);
#endif /* OPLUS_ARCH_EXTENDS */

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);

	dev_dbg(fsa_priv->dev, "%s: registered notifier for %s\n",
		__func__, node->name);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	if (atomic_read(&(fsa_priv->usbc_mode)) == TYPEC_ACCESSORY_AUDIO) {
		dev_dbg(fsa_priv->dev, "%s: analog adapter already inserted\n",
			__func__);
		rc = fsa4480_usbc_analog_setup_switches(fsa_priv);
	}

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	return blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int switch_control = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;
#ifdef OPLUS_ARCH_EXTENDS
	unsigned int setting_reg_val = 0, control_reg_val = 0;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	char buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
#endif
#endif /* OPLUS_ARCH_EXTENDS */

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
	if (!fsa_priv->regmap)
		return -EINVAL;

	#ifdef OPLUS_ARCH_EXTENDS
	pr_info("%s - switch event: %d\n", __func__, event);
	#endif /* OPLUS_ARCH_EXTENDS */

	switch (event) {
	case FSA_MIC_GND_SWAP:
#ifdef OPLUS_ARCH_EXTENDS
		if (fsa_priv->usbc_mode.counter != TYPEC_ACCESSORY_AUDIO) {
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &setting_reg_val);
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &control_reg_val);
			pr_err("%s: error mode, reg[0x%x]=0x%x, reg[0x%x]=0x%x\n", __func__,
					FSA4480_SWITCH_SETTINGS, setting_reg_val, FSA4480_SWITCH_CONTROL, control_reg_val);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
			scnprintf(buf, sizeof(buf) - 1, "func@@%s$$typec_mode@@%d$$regs@@0x%x,0x%x", \
					__func__, fsa_priv->usbc_mode.counter, setting_reg_val, control_reg_val);
			upload_mm_fb_kevent_to_atlas_limit(OPLUS_AUDIO_EVENTID_HEADSET_DET, buf, MM_FB_KEY_RATELIMIT_5MIN);
#endif
			break;
		}
#endif /* OPLUS_ARCH_EXTENDS */

		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&switch_control);
		if ((switch_control & 0x07) == 0x07)
			switch_control = 0x0;
		else
			switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
		break;

	#ifdef OPLUS_ARCH_EXTENDS
	/*add DIO4480 support*/
	case FSA_CONNECT_LR:
		usleep_range(50, 55);
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9F);
		pr_info("%s - panzhao connect LR  \n", __func__);
		break;
	#endif /* OPLUS_ARCH_EXTENDS */

	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

#ifdef OPLUS_ARCH_EXTENDS
static int fsa4480_parse_dt(struct fsa4480_priv *fsa_priv,
	struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	const char *usb_protocal_name = "fsa4480,use-3rd-usb-protocal";
	int ret = 0;
	int rc = 0;

	if (dNode == NULL)
		return -ENODEV;

	if (!fsa_priv) {
		pr_err("%s: fsa_priv is NULL\n", __func__);
		return -ENOMEM;
	}

	fsa_priv->hs_det_pin = of_get_named_gpio(dNode,
	        "fsa4480,hs-det-gpio", 0);
	if (!gpio_is_valid(fsa_priv->hs_det_pin)) {
	    pr_info("%s: hs-det-gpio in dt node is missing\n", __func__);
	    return -ENODEV;
	}
	ret = gpio_request(fsa_priv->hs_det_pin, "fsa4480_hs_det");
	if (ret) {
		pr_err("%s: hs-det-gpio request fail\n", __func__);
		return ret;
	}

	gpio_direction_output(fsa_priv->hs_det_pin, 1);

	rc = of_property_read_u32(dNode, usb_protocal_name, &fsa_priv->usb_protocal);
	if (rc) {
		pr_info("%s: %s in dt node is missing\n", __func__, usb_protocal_name);
		fsa_priv->usb_protocal = 0;
	}

	return ret;
}
#endif /* OPLUS_ARCH_EXTENDS */

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	int rc = 0;
#ifdef OPLUS_ARCH_EXTENDS
/*add DIO4480 support*/
	unsigned int reg_value = 0;
#endif /* OPLUS_ARCH_EXTENDS */

	#ifdef OPLUS_ARCH_EXTENDS
	pr_err("%s: enter\n", __func__);
	#endif /* OPLUS_ARCH_EXTENDS */

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	fsa_priv->dev = &i2c->dev;

	#ifdef OPLUS_ARCH_EXTENDS
	fsa4480_parse_dt(fsa_priv, &i2c->dev);
	#endif /* OPLUS_ARCH_EXTENDS */

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	#ifdef OPLUS_ARCH_EXTENDS
	/*add DIO4480/WAS4783 support*/
	regmap_read(fsa_priv->regmap, FSA4480_DEVICE_ID, &reg_value);
	dev_info(fsa_priv->dev, "%s: device id reg value: 0x%x\n", __func__, reg_value);
	if (reg_value == HL5280_DEVICE_REG_VALUE) {
		dev_info(fsa_priv->dev, "%s: switch chip is HL5280\n", __func__);
		fsa_priv->vendor = HL5280;
	} else if (reg_value == DIO4480_DEVICE_REG_VALUE) {
		dev_info(fsa_priv->dev, "%s: switch chip is DIO4480\n", __func__);
		fsa_priv->vendor = DIO4480;
	} else if (reg_value == WAS4783_DEVICE_REG_VALUE) {
		dev_info(fsa_priv->dev, "%s: switch chip is WAS4783\n", __func__);
		fsa_priv->vendor = WAS4783;
	} else if (reg_value == INVALID_DEVICE_REG_VALUE && chipid_read_retry < 5) {
		dev_info(fsa_priv->dev, "%s: incorrect chip ID [0x%x]\n", __func__, reg_value);
		chipid_read_retry++;
		usleep_range(1*1000, 1*1005);
		rc = -EPROBE_DEFER;
		goto err_data;
	} else {
		dev_info(fsa_priv->dev, "%s: switch chip is FSA4480\n", __func__);
		fsa_priv->vendor = FSA4480;
	}

	if (fsa_priv->vendor != DIO4480) {
		fsa4480_update_reg_defaults(fsa_priv->regmap);
	} else {
		regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
		usleep_range(1*1000, 1*1005);
	}
	#endif /* OPLUS_ARCH_EXTENDS */

	fsa_priv->ucsi_nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->ucsi_nb.priority = 0;
#ifndef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
	rc = register_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
	if (rc) {
		dev_err(fsa_priv->dev, "%s: ucsi glink notifier registration failed: %d\n",
			__func__, rc);
		goto err_data;
	}
#else
	if (fsa_priv->usb_protocal != 1) {
		rc = register_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
		if (rc) {
			dev_err(fsa_priv->dev, "%s: ucsi glink notifier registration failed: %d\n",
				__func__, rc);
			goto err_data;
		}
	} else {
#if IS_ENABLED(CONFIG_TCPC_CLASS)
		dev_err(fsa_priv->dev, "%s: start register 3rd protocal stack notifier\n", __func__);
		fsa_priv->tcpc = tcpc_dev_get_by_name("type_c_port0");
		if (!fsa_priv->tcpc) {
			if (probe_retry > 30) {
				dev_err(fsa_priv->dev, "%s: get tcpc failed, jump tcp register\n", __func__);
				rc = 0;
				goto tcp_register_finish;
			} else {
				probe_retry++;
				dev_err(fsa_priv->dev, "%s: get tcpc failed, retry:%d \n", __func__, probe_retry);
				usleep_range(1*1000, 1*1005);
				rc = -EPROBE_DEFER;
				goto err_data;
			}
		}
		rc = register_tcp_dev_notifier(fsa_priv->tcpc, &fsa_priv->ucsi_nb, TCP_NOTIFY_TYPE_USB);
		if (rc) {
			dev_err(fsa_priv->dev, "%s: ucsi glink notifier registration failed: %d\n",
				__func__, rc);
			goto err_data;
		}
#endif
	}

#if IS_ENABLED(CONFIG_TCPC_CLASS)
tcp_register_finish:
#endif
#endif /* OPLUS_ARCH_EXTENDS */

	#ifdef OPLUS_ARCH_EXTENDS
	/* add WAS4783 support */
	mutex_init(&fsa_priv->noti_lock);

	if (fsa_priv->vendor == WAS4783) {
		fsa_priv->chg_nb.notifier_call = typec_switch_chg_event_changed;
		fsa_priv->chg_nb.priority = 0;
		rc = register_chg_glink_notifier(&fsa_priv->chg_nb);
		if (rc) {
			if (fsa_priv->usb_protocal != 1) {
				unregister_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
			} else {
				#if IS_ENABLED(CONFIG_TCPC_CLASS)
				if (fsa_priv->tcpc)
					unregister_tcp_dev_notifier(fsa_priv->tcpc, &fsa_priv->ucsi_nb, TCP_NOTIFY_TYPE_USB);
				#endif
			}
			mutex_destroy(&fsa_priv->noti_lock);
			dev_err(fsa_priv->dev, "%s: charge glink notifier registration failed: %d\n",
				__func__, rc);
			goto err_data;
		}
	}
	#endif /* OPLUS_ARCH_EXTENDS */

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	BLOCKING_INIT_NOTIFIER_HEAD(&fsa_priv->fsa4480_notifier);

	return 0;

err_data:
	#ifdef OPLUS_ARCH_EXTENDS
	/*removed for test*/
	pr_err("%s: finished since err\n", __func__);
	#endif /* OPLUS_ARCH_EXTENDS */

	#ifdef OPLUS_ARCH_EXTENDS
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	#endif /* OPLUS_ARCH_EXTENDS */
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
#ifdef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	int ret = 0;
#endif
#endif /* OPLUS_ARCH_EXTENDS */
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

#ifdef OPLUS_ARCH_EXTENDS
/*add for 3rd protocal stack notifier*/
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	if (fsa_priv->tcpc) {
		ret = unregister_tcp_dev_notifier(fsa_priv->tcpc, &fsa_priv->ucsi_nb, TCP_NOTIFY_TYPE_ALL);
		if (ret < 0) {
			pr_err("%s unregister tcpc notifier fail(%d)\n", __func__);
		}
	}
#endif
#endif /* OPLUS_ARCH_EXTENDS */
	unregister_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
	#ifdef OPLUS_ARCH_EXTENDS
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	/*add WAS4783 support*/
	if (fsa_priv->vendor == WAS4783) {
		unregister_chg_glink_notifier(&fsa_priv->chg_nb);
	}
	mutex_destroy(&fsa_priv->noti_lock);
	devm_kfree(&i2c->dev, fsa_priv);
	#endif /* OPLUS_ARCH_EXTENDS */
	dev_set_drvdata(&i2c->dev, NULL);

	return 0;
}

#ifdef OPLUS_ARCH_EXTENDS
static void fsa4480_shutdown(struct i2c_client *i2c) {
	struct fsa4480_priv *fsa_priv =
		(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv) {
		return;
	}

	pr_info("%s: recover all register while shutdown\n", __func__);

	if (fsa_priv->vendor == DIO4480) {
		regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
		return;
	}

	fsa4480_update_reg_defaults(fsa_priv->regmap);

	return;
}
#endif /* OPLUS_ARCH_EXTENDS */

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
#ifdef OPLUS_ARCH_EXTENDS
	.shutdown = fsa4480_shutdown,
#endif /* OPLUS_ARCH_EXTENDS */
};

static int __init fsa4480_init(void)
{
	int rc;

#ifdef OPLUS_ARCH_EXTENDS
/*removed for test*/
	pr_info("%s(): enter\n", __func__);
#endif /* OPLUS_ARCH_EXTENDS */
	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(fsa4480_init);

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
