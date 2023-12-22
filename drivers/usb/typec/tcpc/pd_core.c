/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * Power Delivery Core Driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/of.h>
#include <linux/slab.h>
#include "inc/tcpci.h"
#include "inc/pd_core.h"
#include "inc/pd_dpm_core.h"
#include "inc/tcpci_typec.h"
#include "inc/tcpci_event.h"
#include "inc/pd_policy_engine.h"
#ifdef CONFIG_DUAL_ROLE_USB_INTF
#include <linux/usb/class-dual-role.h>
#endif /* CONFIG_DUAL_ROLE_USB_INTF */

/* From DTS */

#ifdef CONFIG_USB_PD_REV30_BAT_INFO
static inline void pd_parse_pdata_bat_info(
	struct pd_port *pd_port, struct device_node *sub,
	struct pd_battery_info *bat_info)
{
	int ret = 0;
	u32 design_cap;
	uint32_t vid, pid;
	const char *mstring;

	struct pd_battery_capabilities *bat_cap = &bat_info->bat_cap;

#ifdef CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL
	struct pd_manufacturer_info *mfrs_info = &bat_info->mfrs_info;
#endif	/* CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL */

	ret = of_property_read_u32(sub, "bat,vid", (u32 *)&vid);
	if (ret < 0) {
		pr_err("%s get pd vid fail\n", __func__);
		vid = PD_IDH_VID(pd_port->id_vdos[0]);
	}

	ret = of_property_read_u32(sub, "bat,pid", (u32 *)&pid);
	if (ret < 0) {
		pr_err("%s get pd pid fail\n", __func__);
		pid = PD_PRODUCT_PID(pd_port->id_vdos[2]);
	}

#ifdef CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL
	mfrs_info->vid = vid;
	mfrs_info->pid = pid;

	ret = of_property_read_string(sub, "bat,mfrs", &mstring);
	if (ret < 0) {
		pr_err("%s get bat,mfrs fail\n", __func__);
		mstring = "no_bat_mfrs_string";
	}
	ret = snprintf(mfrs_info->mfrs_string,
		sizeof(mfrs_info->mfrs_string), "%s", mstring);
	if (ret < 0 || ret >= sizeof(mfrs_info->mfrs_string))
		pr_info("%s-%d snprintf fail\n", __func__, __LINE__);
#endif	/* CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL */

	ret = of_property_read_u32(sub, "bat,design_cap", &design_cap);
	if (ret < 0) {
		bat_cap->bat_design_cap = PD_BCDB_BAT_CAP_UNKNOWN;
		pr_err("%s get bat,dsn_cat fail\n", __func__);
	} else {
		bat_cap->bat_design_cap = (uint16_t)
			PD_BCDB_BAT_CAP_RAW(design_cap);
	}

	bat_cap->vid = vid;
	bat_cap->pid = pid;
	bat_cap->bat_last_full_cap = PD_BCDB_BAT_CAP_UNKNOWN;

	bat_info->bat_status = BSDO(
		BSDO_BAT_CAP_UNKNOWN, BSDO_BAT_INFO_IDLE);
}

static inline int pd_parse_pdata_bats(
	struct pd_port *pd_port, struct device_node *np)
{
	u32 val;
	int ret = 0, i;
	struct device_node *sub;
	char temp_string[26];

	ret = of_property_read_u32(np, "bat,nr", &val);
	if (ret < 0) {
		pr_err("%s get pd bat NR fail\n", __func__);
		pd_port->bat_nr = 0;
		return 0;
	}

	pd_port->bat_nr = val;
	pr_info("%s Battery NR = %d\n", __func__, pd_port->bat_nr);

	pd_port->fix_bat_info = devm_kzalloc(&pd_port->tcpc->dev,
		sizeof(struct pd_battery_info)*pd_port->bat_nr,
		GFP_KERNEL);

	if (!pd_port->fix_bat_info) {
		pr_err("%s get fix_bat_info memory fail\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < pd_port->bat_nr; i++) {
		snprintf(temp_string, 26, "bat-info%d", i);
		sub = of_find_node_by_name(np, temp_string);
		if (!sub) {
			pr_err("%s get sub bat node fail\n", __func__);
			return -ENODEV;
		}

		pd_parse_pdata_bat_info(
			pd_port, sub, &pd_port->fix_bat_info[i]);
	}

	for (i = 0; i < pd_port->bat_nr; i++) {
		pr_info("%s fix_bat_info[%d].mfrs_info.vid = 0x%x, .mfrs_info.pid = 0x%x, .mfrs_string = %s, .bat_design_cap = %d\n",
			__func__, i,
			pd_port->fix_bat_info[i].mfrs_info.vid,
			pd_port->fix_bat_info[i].mfrs_info.pid,
			pd_port->fix_bat_info[i].mfrs_info.mfrs_string,
			PD_BCDB_BAT_CAP_VAL(
			pd_port->fix_bat_info[i].bat_cap.bat_design_cap));
	}

	return 0;
}
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */

#ifdef CONFIG_USB_PD_REV30_COUNTRY_AUTHORITY
static inline int pd_parse_pdata_country(
	struct pd_port *pd_port,  struct device_node *sub,
	struct pd_country_authority *country_info)
{
	u32 val;
	int ret = 0, j;
	u32 *temp_u32;

	ret = of_property_read_u32(sub, "pd,country_code", &val);
	if (ret < 0) {
		pr_err("%s get country code fail\n", __func__);
		return -ENODEV;
	}

	country_info->code = (uint16_t) val;

	ret = of_property_read_u32(sub, "pd,country_len", &val);
	if (ret < 0) {
		pr_err("%s get country len fail\n", __func__);
		return -ENODEV;
	}

	country_info->len = (uint16_t) val;

	country_info->data = devm_kzalloc(
		&pd_port->tcpc->dev,
		sizeof(uint8_t)*country_info->len,
		GFP_KERNEL);

	if (!country_info->data) {
		pr_err("%s get country info data mem fail\n",
			__func__);
		return -ENOMEM;
	}

	temp_u32 = devm_kzalloc(&pd_port->tcpc->dev,
		sizeof(u32)*country_info->len, GFP_KERNEL);

	ret = of_property_read_u32_array(sub, "pd,country_data",
		temp_u32,
		country_info->len);
	if (ret < 0)
		pr_err("%s get country data fail\n", __func__);

	for (j = 0; j < country_info->len; j++)
		country_info->data[j] = (uint8_t) temp_u32[j];

	devm_kfree(&pd_port->tcpc->dev, temp_u32);

	return 0;
}

static inline int pd_parse_pdata_countries(
	struct pd_port *pd_port, struct device_node *np)
{
	int ret = 0, i, j;
	struct device_node *sub;
	char temp_string[26];

	ret = of_property_read_u32(np, "pd,country_nr",
			(u32 *)&pd_port->country_nr);
	if (ret < 0) {
		pr_err("%s get country nr fail\n", __func__);
		pd_port->country_nr = 0;
		return 0;
	}

	pr_info("%s Country NR = %d\n", __func__, pd_port->country_nr);

	pd_port->country_info = devm_kzalloc(&pd_port->tcpc->dev,
		sizeof(struct pd_country_authority)*pd_port->country_nr,
		GFP_KERNEL);

	if (!pd_port->country_info) {
		pr_err("%s get country info memory fail\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < pd_port->country_nr; i++) {
		snprintf(temp_string, 26, "country%d", i);
		sub = of_find_node_by_name(np, temp_string);
		if (!sub) {
			pr_err("%s get sub country node fail\n",
				__func__);
			return -ENODEV;
		}

		ret = pd_parse_pdata_country(pd_port,  sub,
			&pd_port->country_info[i]);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < pd_port->country_nr; i++) {
		pr_info("%s country_info[%d].code = 0x%x, .len = %d\n",
			__func__, i,
			pd_port->country_info[i].code,
			pd_port->country_info[i].len);
		for (j = 0; j < pd_port->country_info[i].len; j++) {
			pr_info("%s country_info[%d].data[%d] = 0x%x\n",
				__func__, i, j,
				pd_port->country_info[i].data[j]);
		}
	}
	return 0;
}
#endif	/* CONFIG_USB_PD_REV30_COUNTRY_AUTHORITY */

#ifdef CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL
static void pd_parse_log_src_cap_ext(struct pd_source_cap_ext *cap)
{
	pr_info("%s vid = 0x%x, pid = 0x%x, xid = 0x%x, fw_ver = 0x%x, hw_ver = 0x%0x\n",
		__func__,
		cap->vid, cap->pid, cap->xid,
		cap->fw_ver, cap->hw_ver);

	pr_info("%s voltage_regulation = %d, hold_time_ms = %d, compliance = 0x%x, touch_current = 0x%x, peak_current = %d %d %d\n",
		__func__,
		cap->voltage_regulation,
		cap->hold_time_ms,
		cap->compliance,
		cap->touch_current,
		cap->peak_current[0],
		cap->peak_current[1],
		cap->peak_current[2]);

	pr_info("%s touch_temp = %d, source_inputs = 0x%x, batteries = 0x%x, source_pdp = 0x%x\n",
		__func__,
		cap->touch_temp,
		cap->source_inputs,
		cap->batteries,
		cap->source_pdp);
}
#endif /* CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL	*/

static inline void pd_parse_pdata_src_cap_ext(
	struct pd_port *pd_port, struct device_node *np)
{
#ifdef CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL
	int ret = 0;

	ret = of_property_read_u32_array(np, "pd,source-cap-ext",
		(u32 *) &pd_port->src_cap_ext,
		sizeof(struct pd_source_cap_ext)/4);

	if (ret < 0)
		pr_err("%s get source-cap-ext fail\n", __func__);
	else
		pd_parse_log_src_cap_ext(&pd_port->src_cap_ext);

#ifdef CONFIG_USB_PD_REV30_BAT_INFO
	pd_port->src_cap_ext.batteries =
		PD_SCEDB_BATTERIES(0, pd_port->bat_nr);

	if (pd_port->src_cap_ext.batteries)
		pd_port->src_cap_ext.source_inputs |= PD_SCEDB_INPUT_INT;
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */
#endif	/* CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL */
}

static inline void pd_parse_pdata_mfrs(
	struct pd_port *pd_port, struct device_node *np)
{
	int ret = 0;
	uint32_t vid, pid;
#ifdef CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL
	const char *mstring;
#endif /* CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL */

#ifdef CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL
	struct pd_manufacturer_info *mfrs_info = &pd_port->mfrs_info;
#endif	/* CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL */

	ret = of_property_read_u32(np, "pd,vid", (u32 *)&vid);
	if (ret < 0) {
		pr_err("%s get pd vid fail\n", __func__);
		vid = PD_IDH_VID(pd_port->id_vdos[0]);
	}

	ret = of_property_read_u32(np, "pd,pid", (u32 *)&pid);
	if (ret < 0) {
		pr_err("%s get pd pid fail\n", __func__);
		pid = PD_PRODUCT_PID(pd_port->id_vdos[2]);
	}

	pr_info("%s VID = 0x%x, PID = 0x%x\n", __func__, vid, pid);

#ifdef CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL
	mfrs_info->vid = vid;
	mfrs_info->pid = pid;

	ret = of_property_read_string(np, "pd,mfrs", &mstring);
	if (ret < 0) {
		pr_err("%s get pd mfrs fail\n", __func__);
		mstring = "no_pd_mfrs_string";
	}
	ret = snprintf(mfrs_info->mfrs_string,
		sizeof(mfrs_info->mfrs_string), "%s", mstring);
	if (ret < 0 || ret >= sizeof(mfrs_info->mfrs_string))
		pr_info("%s-%d snprintf fail\n", __func__, __LINE__);

	pr_info("%s PD mfrs_string = %s\n",
		__func__, mfrs_info->mfrs_string);
#endif	/* CONFIG_USB_PD_REV30_MFRS_INFO_LOCAL */

#ifdef CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL
	pd_port->src_cap_ext.vid = vid;
	pd_port->src_cap_ext.pid = pid;
#endif	/* CONFIG_USB_PD_REV30_SRC_CAP_EXT_LOCAL */

	pd_port->id_vdos[0] &= ~PD_IDH_VID_MASK;
	pd_port->id_vdos[0] |= PD_IDH_VID(vid);

	pd_port->id_vdos[2] = VDO_PRODUCT(
		pid, PD_PRODUCT_BCD(pd_port->id_vdos[2]));

	pd_port->id_header = pd_port->id_vdos[0];
}

static int pd_parse_pdata(struct pd_port *pd_port)
{
	u32 val;
	struct device_node *np;
	int ret = 0, i;

	pr_info("%s\n", __func__);
	np = of_find_node_by_name(pd_port->tcpc->dev.of_node, "pd-data");

	if (np) {
		ret = of_property_read_u32(np, "pd,source-pdo-size",
				(u32 *)&pd_port->local_src_cap_default.nr);
		if (ret < 0)
			pr_err("%s get source pdo size fail\n", __func__);

		ret = of_property_read_u32_array(np, "pd,source-pdo-data",
			(u32 *)pd_port->local_src_cap_default.pdos,
			pd_port->local_src_cap_default.nr);
		if (ret < 0)
			pr_err("%s get source pdo data fail\n", __func__);

		pr_info("%s src pdo data =\n", __func__);
		for (i = 0; i < pd_port->local_src_cap_default.nr; i++) {
			pr_info("%s %d: 0x%08x\n", __func__, i,
				pd_port->local_src_cap_default.pdos[i]);
		}

		ret = of_property_read_u32(np, "pd,sink-pdo-size",
					(u32 *)&pd_port->local_snk_cap.nr);
		if (ret < 0)
			pr_err("%s get sink pdo size fail\n", __func__);

		ret = of_property_read_u32_array(np, "pd,sink-pdo-data",
			(u32 *)pd_port->local_snk_cap.pdos,
				pd_port->local_snk_cap.nr);
		if (ret < 0)
			pr_err("%s get sink pdo data fail\n", __func__);

		pr_info("%s snk pdo data =\n", __func__);
		for (i = 0; i < pd_port->local_snk_cap.nr; i++) {
			pr_info("%s %d: 0x%08x\n", __func__, i,
				pd_port->local_snk_cap.pdos[i]);

#ifdef CONFIG_USB_PD_REV30_PPS_SINK
			if (PDO_TYPE(pd_port->local_snk_cap.pdos[i]) !=
				PDO_TYPE_APDO)
				pd_port->local_snk_cap_nr_pd20++;
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
		}

#ifdef CONFIG_USB_PD_REV30_PPS_SINK
		pd_port->local_snk_cap_nr_pd30 = pd_port->local_snk_cap.nr;
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */

		ret = of_property_read_u32(np, "pd,id-vdo-size",
					(u32 *)&pd_port->id_vdo_nr);
		if (ret < 0)
			pr_err("%s get id vdo size fail\n", __func__);
		ret = of_property_read_u32_array(np, "pd,id-vdo-data",
			(u32 *)pd_port->id_vdos, pd_port->id_vdo_nr);
		if (ret < 0)
			pr_err("%s get id vdo data fail\n", __func__);

		pr_info("%s id vdos data =\n", __func__);
		for (i = 0; i < pd_port->id_vdo_nr; i++)
			pr_info("%s %d: 0x%08x\n", __func__, i,
			pd_port->id_vdos[i]);

		pd_port->id_header = pd_port->id_vdos[0];

		val = DPM_CHARGING_POLICY_MAX_POWER_LVIC;
		if (of_property_read_u32(np, "pd,charging_policy", &val) < 0)
			pr_info("%s get charging policy fail\n", __func__);

		pd_port->dpm_charging_policy = val;
		pd_port->dpm_charging_policy_default = val;
		pr_info("%s charging_policy = %d\n", __func__, val);

#ifdef CONFIG_USB_PD_REV30_BAT_INFO
		ret = pd_parse_pdata_bats(pd_port, np);
		if (ret < 0)
			return ret;
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */

#ifdef CONFIG_USB_PD_REV30_COUNTRY_AUTHORITY
		ret = pd_parse_pdata_countries(pd_port, np);
		if (ret < 0)
			return ret;
#endif	/* CONFIG_USB_PD_REV30_COUNTRY_AUTHORITY */

		pd_parse_pdata_src_cap_ext(pd_port, np);
		pd_parse_pdata_mfrs(pd_port, np);
	}

	return 0;
}

static const struct {
	const char *prop_name;
	uint32_t val;
} supported_dpm_caps[] = {
	{"local_dr_power", DPM_CAP_LOCAL_DR_POWER},
	{"local_dr_data", DPM_CAP_LOCAL_DR_DATA},
	{"local_ext_power", DPM_CAP_LOCAL_EXT_POWER},
	{"local_usb_comm", DPM_CAP_LOCAL_USB_COMM},
	{"local_usb_suspend", DPM_CAP_LOCAL_USB_SUSPEND},
	{"local_high_cap", DPM_CAP_LOCAL_HIGH_CAP},
	{"local_give_back", DPM_CAP_LOCAL_GIVE_BACK},
	{"local_no_suspend", DPM_CAP_LOCAL_NO_SUSPEND},
	{"local_vconn_supply", DPM_CAP_LOCAL_VCONN_SUPPLY},

	{"attemp_discover_cable_dfp", DPM_CAP_ATTEMP_DISCOVER_CABLE_DFP},
	{"attemp_enter_dp_mode", DPM_CAP_ATTEMP_ENTER_DP_MODE},
	{"attemp_discover_cable", DPM_CAP_ATTEMP_DISCOVER_CABLE},
	{"attemp_discover_id", DPM_CAP_ATTEMP_DISCOVER_ID},
	{"attemp_discover_svid", DPM_CAP_ATTEMP_DISCOVER_SVID},

	{"pr_reject_as_source", DPM_CAP_PR_SWAP_REJECT_AS_SRC},
	{"pr_reject_as_sink", DPM_CAP_PR_SWAP_REJECT_AS_SNK},
	{"pr_check_gp_source", DPM_CAP_PR_SWAP_CHECK_GP_SRC},
	{"pr_check_gp_sink", DPM_CAP_PR_SWAP_CHECK_GP_SNK},

	{"dr_reject_as_dfp", DPM_CAP_DR_SWAP_REJECT_AS_DFP},
	{"dr_reject_as_ufp", DPM_CAP_DR_SWAP_REJECT_AS_UFP},
};

static void pd_core_power_flags_init(struct pd_port *pd_port)
{
	uint32_t src_flag, snk_flag, val;
	struct device_node *np;
	int i;
	struct pd_port_power_caps *snk_cap = &pd_port->local_snk_cap;
	struct pd_port_power_caps *src_cap =
				&pd_port->local_src_cap_default;

	np = of_find_node_by_name(pd_port->tcpc->dev.of_node, "dpm_caps");

	for (i = 0; i < ARRAY_SIZE(supported_dpm_caps); i++) {
		if (of_property_read_bool(np,
			supported_dpm_caps[i].prop_name))
			pd_port->dpm_caps |=
				supported_dpm_caps[i].val;
			pr_info("dpm_caps: %s\n",
				supported_dpm_caps[i].prop_name);
	}

	if (of_property_read_u32(np, "pr_check", &val) == 0)
		pd_port->dpm_caps |= DPM_CAP_PR_CHECK_PROP(val);
	else
		pr_err("%s get pr_check data fail\n", __func__);

	if (of_property_read_u32(np, "dr_check", &val) == 0)
		pd_port->dpm_caps |= DPM_CAP_DR_CHECK_PROP(val);
	else
		pr_err("%s get dr_check data fail\n", __func__);

	pr_info("dpm_caps = 0x%08x\n", pd_port->dpm_caps);

	src_flag = 0;
	if (pd_port->dpm_caps & DPM_CAP_LOCAL_DR_POWER)
		src_flag |= PDO_FIXED_DUAL_ROLE;

	if (pd_port->dpm_caps & DPM_CAP_LOCAL_DR_DATA)
		src_flag |= PDO_FIXED_DATA_SWAP;

	if (pd_port->dpm_caps & DPM_CAP_LOCAL_EXT_POWER)
		src_flag |= PDO_FIXED_EXTERNAL;

	if (pd_port->dpm_caps & DPM_CAP_LOCAL_USB_COMM)
		src_flag |= PDO_FIXED_COMM_CAP;

	snk_flag = src_flag;

	if (pd_port->dpm_caps & DPM_CAP_LOCAL_USB_SUSPEND)
		src_flag |= PDO_FIXED_SUSPEND;

	if (pd_port->dpm_caps & DPM_CAP_LOCAL_HIGH_CAP)
		snk_flag |= PDO_FIXED_HIGH_CAP;

	snk_cap->pdos[0] |= snk_flag;
	src_cap->pdos[0] |= src_flag;
}

#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
static void fg_bat_absent_work(struct work_struct *work)
{
	struct pd_port *pd_port = container_of(work, struct pd_port,
					       fg_bat_work);
	struct tcpc_device *tcpc = pd_port->tcpc;
	int ret = 0;

	ret = tcpm_shutdown(tcpc);
	if (ret < 0)
		pr_notice("%s: tcpm shutdown fail\n", __func__);
}
#endif /* ONFIG_RECV_BAT_ABSENT_NOTIFY */

int pd_core_init(struct tcpc_device *tcpc)
{
	struct pd_port *pd_port = &tcpc->pd_port;
	int ret;

	mutex_init(&pd_port->pd_lock);

#ifdef CONFIG_USB_PD_BLOCK_TCPM
	mutex_init(&pd_port->tcpm_bk_lock);
	init_waitqueue_head(&pd_port->tcpm_bk_wait_que);
#endif	/* CONFIG_USB_PD_BLOCK_TCPM */

	pd_port->tcpc = tcpc;
	pd_port->pe_pd_state = PE_IDLE2;
	pd_port->cap_miss_match = 0; /* For src_cap miss match */

	ret = pd_parse_pdata(pd_port);
	if (ret < 0)
		return ret;

	pd_core_power_flags_init(pd_port);

	pd_dpm_core_init(pd_port);

#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
	INIT_WORK(&pd_port->fg_bat_work, fg_bat_absent_work);
#endif /* CONFIG_RECV_BAT_ABSENT_NOTIFY */

	PE_INFO("%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL(pd_core_init);

void pd_extract_rdo_power(uint32_t rdo, uint32_t pdo,
			uint32_t *op_curr, uint32_t *max_curr)
{
	uint32_t op_power, max_power, vmin;

	switch (pdo & PDO_TYPE_MASK) {
	case PDO_TYPE_FIXED:
	case PDO_TYPE_VARIABLE:
		*op_curr = RDO_FIXED_VAR_EXTRACT_OP_CURR(rdo);
		*max_curr = RDO_FIXED_VAR_EXTRACT_MAX_CURR(rdo);
		break;

	case PDO_TYPE_BATTERY: /* TODO: check it later !! */
		vmin = PDO_BATT_EXTRACT_MIN_VOLT(pdo);
		op_power = RDO_BATT_EXTRACT_OP_POWER(rdo);
		max_power = RDO_BATT_EXTRACT_MAX_POWER(rdo);

		*op_curr = op_power / vmin;
		*max_curr = max_power / vmin;
		break;

#ifdef CONFIG_USB_PD_REV30_PPS_SOURCE
	case PDO_TYPE_APDO:
		*op_curr = RDO_APDO_EXTRACT_OP_MA(rdo);
		*max_curr = RDO_APDO_EXTRACT_OP_MA(rdo);
		break;
#endif	/* CONFIG_USB_PD_REV30_PPS_SOURCE */

	default:
		*op_curr = *max_curr = 0;
		break;
	}
}

uint32_t pd_reset_pdo_power(struct tcpc_device *tcpc,
			uint32_t pdo, uint32_t imax)
{
	uint32_t ioper;

	switch (pdo & PDO_TYPE_MASK) {
	case PDO_TYPE_FIXED:
		ioper = PDO_FIXED_EXTRACT_CURR(pdo);
		if (ioper > imax)
			return PDO_FIXED_RESET_CURR(pdo, imax);
		break;

	case PDO_TYPE_VARIABLE:
		ioper = PDO_VAR_EXTRACT_CURR(pdo);
		if (ioper > imax)
			return PDO_VAR_RESET_CURR(pdo, imax);
		break;

	case PDO_TYPE_BATTERY:
		/* TODO: check it later !! */
		PD_ERR("No Support\n");
		break;

#ifdef CONFIG_USB_PD_REV30_PPS_SOURCE
	case PDO_TYPE_APDO:
		/* TODO: check it later !! */
		break;
#endif	/* CONFIG_USB_PD_REV30_PPS_SOURCE */
	}
	return pdo;
}

uint32_t pd_get_cable_curr_lvl(struct pd_port *pd_port)
{
	return PD_VDO_CABLE_CURR(
		pd_port->pe_data.cable_vdos[VDO_DISCOVER_ID_CABLE]);
}
EXPORT_SYMBOL(pd_get_cable_curr_lvl);

uint32_t pd_get_cable_current_limit(struct pd_port *pd_port)
{
	switch (pd_get_cable_curr_lvl(pd_port)) {
	case CABLE_CURR_1A5:
		return 1500;
	case CABLE_CURR_5A:
		return 5000;
	default:
	case CABLE_CURR_3A:
		return 3000;
	}
}

static inline bool pd_is_cable_communication_available(
	struct pd_port *pd_port)
{
	/*
	 * After pr_swap or fr_swap,
	 * the source (must be Vconn SRC) can communicate with Cable,
	 * the sink doesn't communicate with cable even if it's DFP.
	 *
	 * When an Explicit Contract is in place,
	 * Only the Vconn SRC can communicate with Cable.
	 */

#ifdef CONFIG_USB_PD_REV30_DISCOVER_CABLE_WITH_VCONN
	if (pd_check_rev30(pd_port) && (!pd_port->vconn_role))
		return false;
#endif	/* CONFIG_USB_PD_REV30_DISCOVER_CABLE_WITH_VCONN */

	return true;
}

bool pd_is_reset_cable(struct pd_port *pd_port)
{
	if (!dpm_reaction_check(pd_port, DPM_REACTION_CAP_RESET_CABLE))
		return false;

	return pd_is_cable_communication_available(pd_port);
}

bool pd_is_discover_cable(struct pd_port *pd_port)
{
	if (!dpm_reaction_check(pd_port, DPM_REACTION_CAP_DISCOVER_CABLE))
		return false;

	if (pd_port->pe_data.discover_id_counter >= PD_DISCOVER_ID_COUNT) {
		dpm_reaction_clear(pd_port,
			DPM_REACTION_DISCOVER_CABLE |
			DPM_REACTION_CAP_DISCOVER_CABLE);
		return false;
	}

	return pd_is_cable_communication_available(pd_port);
}

void pd_reset_svid_data(struct pd_port *pd_port)
{
	uint8_t i;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt; i++) {
		svid_data = &pd_port->svid_data[i];
		svid_data->exist = false;
		svid_data->remote_mode.mode_cnt = 0;
		svid_data->active_mode = 0;
	}
}

#define PE_RESET_MSG_ID(pd_port, sop)	{ \
	pd_port->pe_data.msg_id_tx[sop] = 0; \
	pd_port->pe_data.msg_id_rx[sop] = PD_MSG_ID_MAX; \
}

int pd_reset_protocol_layer(struct pd_port *pd_port, bool sop_only)
{
	struct pe_data *pe_data = &pd_port->pe_data;

	pd_port->state_machine = PE_STATE_MACHINE_NORMAL;

	pd_notify_pe_reset_protocol(pd_port);

#ifdef CONFIG_USB_PD_PE_SOURCE
	pe_data->cap_counter = 0;
#endif	/* CONFIG_USB_PD_PE_SOURCE */

	pe_data->explicit_contract = false;
	pe_data->local_selected_cap = 0;
	pe_data->remote_selected_cap = 0;
	pe_data->during_swap = 0;
	pd_port->cap_miss_match = 0;

#ifdef CONFIG_USB_PD_REV30_ALERT_REMOTE
	pe_data->remote_alert = 0;
#endif	/* CONFIG_USB_PD_REV30_ALERT_REMOTE */

#ifdef CONFIG_USB_PD_DFP_FLOW_DELAY_RESET
	if (pe_data->pd_prev_connected)
		dpm_reaction_set(pd_port, DPM_REACTION_DFP_FLOW_DELAY);
#endif	/* CONFIG_USB_PD_DFP_FLOW_DELAY_RESET */

#ifdef CONFIG_USB_PD_DFP_READY_DISCOVER_ID
	dpm_reaction_clear(pd_port, DPM_REACTION_RETURN_VCONN_SRC);
#endif	/* CONFIG_USB_PD_DFP_READY_DISCOVER_ID */

	PE_RESET_MSG_ID(pd_port, TCPC_TX_SOP);

	if (!sop_only) {
		pd_port->request_i = -1;
		pd_port->request_v = TCPC_VBUS_SINK_5V;

		PE_RESET_MSG_ID(pd_port, TCPC_TX_SOP_PRIME);
		PE_RESET_MSG_ID(pd_port, TCPC_TX_SOP_PRIME_PRIME);
	}

#ifdef CONFIG_USB_PD_IGNORE_PS_RDY_AFTER_PR_SWAP
	pd_port->msg_id_pr_swap_last = 0xff;
#endif	/* CONFIG_USB_PD_IGNORE_PS_RDY_AFTER_PR_SWAP */

	return 0;
}

int pd_set_rx_enable(struct pd_port *pd_port, uint8_t enable)
{
	pd_port->rx_cap = enable;
	return tcpci_set_rx_enable(pd_port->tcpc, enable);
}

int pd_enable_vbus_valid_detection(struct pd_port *pd_port, bool wait_valid)
{
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PE_DBG("WaitVBUS=%d\n", wait_valid);
	pd_notify_pe_wait_vbus_once(pd_port,
		wait_valid ? PD_WAIT_VBUS_VALID_ONCE :
					PD_WAIT_VBUS_INVALID_ONCE);
	return 0;
}

int pd_enable_vbus_safe0v_detection(struct pd_port *pd_port)
{
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PE_DBG("WaitVSafe0V\n");
	pd_notify_pe_wait_vbus_once(pd_port, PD_WAIT_VBUS_SAFE0V_ONCE);
	return 0;
}

int pd_enable_vbus_stable_detection(struct pd_port *pd_port)
{
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PE_DBG("WaitVStable\n");
	pd_notify_pe_wait_vbus_once(pd_port, PD_WAIT_VBUS_STABLE_ONCE);
	return 0;
}

static inline int pd_update_msg_header(struct pd_port *pd_port)
{
	return tcpci_set_msg_header(pd_port->tcpc,
		pd_port->power_role, pd_port->data_role);
}

int pd_set_data_role(struct pd_port *pd_port, uint8_t dr)
{
	int ret = 0;

	if (dr == pd_port->data_role)
		return ret;

	pd_port->data_role = dr;
	ret = pd_update_msg_header(pd_port);
	if (ret < 0)
		return ret;

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	/* dual role usb--> 0:ufp, 1:dfp */
	pd_port->tcpc->dual_role_mode = pd_port->data_role;
	/* dual role usb --> 0: Device, 1: Host */
	pd_port->tcpc->dual_role_dr = !(pd_port->data_role);
	dual_role_instance_changed(pd_port->tcpc->dr_usb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */

	tcpci_notify_role_swap(pd_port->tcpc, TCP_NOTIFY_DR_SWAP, dr);
	return ret;
}

int pd_set_power_role(struct pd_port *pd_port, uint8_t pr)
{
	int ret = 0;

	if (pr == pd_port->power_role)
		return ret;

	pd_port->power_role = pr;
	ret = pd_update_msg_header(pd_port);
	if (ret < 0)
		return ret;

	pd_notify_pe_pr_changed(pd_port);

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	/* 0:sink, 1: source */
	pd_port->tcpc->dual_role_pr = !(pd_port->power_role);
	dual_role_instance_changed(pd_port->tcpc->dr_usb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */

	tcpci_notify_role_swap(pd_port->tcpc, TCP_NOTIFY_PR_SWAP, pr);
	return ret;
}

static void pd_init_spec_revision(struct pd_port *pd_port)
{
#ifdef CONFIG_USB_PD_REV30_SYNC_SPEC_REV
	if (pd_port->tcpc->tcpc_flags & TCPC_FLAGS_PD_REV30) {
		pd_port->pd_revision[0] = PD_REV30;
		pd_port->pd_revision[1] = PD_REV30;
	} else {
		pd_port->pd_revision[0] = PD_REV20;
		pd_port->pd_revision[1] = PD_REV20;
	}
#endif	/* CONFIG_USB_PD_REV30_SYNC_SPEC_REV */
}

int pd_init_message_hdr(struct pd_port *pd_port, bool act_as_sink)
{
	if (act_as_sink) {
		pd_port->power_role = PD_ROLE_SINK;
		pd_port->data_role = PD_ROLE_UFP;
		pd_port->vconn_role = PD_ROLE_VCONN_OFF;
	} else {
		pd_port->power_role = PD_ROLE_SOURCE;
		pd_port->data_role = PD_ROLE_DFP;
		pd_port->vconn_role = PD_ROLE_VCONN_ON;
	}

	pd_init_spec_revision(pd_port);
	return pd_update_msg_header(pd_port);
}

int pd_set_vconn(struct pd_port *pd_port, uint8_t role)
{
	int ret = 0;
	bool enable = !!(role & PD_ROLE_VCONN_ON);
	bool en_role = role != PD_ROLE_VCONN_OFF;
	bool en_role_old = pd_port->vconn_role != PD_ROLE_VCONN_OFF;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PE_DBG("%s:%d\n", __func__, role);

#ifdef CONFIG_USB_PD_VCONN_SAFE5V_ONLY
	if (pd_port->pe_data.vconn_highv_prot) {
		PE_DBG("VC_OVER5V\n");
		return -EPERM;
	}
#endif	/* CONFIG_USB_PD_VCONN_SAFE5V_ONLY */

	if (role == pd_port->vconn_role)
		goto out;

	pd_port->vconn_role = role;
	ret = tcpci_set_vconn(tcpc, enable);
	if (ret < 0)
		return ret;

	if (en_role != en_role_old) {
		tcpci_notify_role_swap(tcpc, TCP_NOTIFY_VCONN_SWAP, en_role);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
		tcpc->dual_role_vconn = en_role;
		dual_role_instance_changed(tcpc->dr_usb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
	}
out:
#ifdef CONFIG_USB_PD_VCONN_STABLE_DELAY
	if (role == PD_ROLE_VCONN_DYNAMIC_ON)
		pd_restart_timer(pd_port, PD_TIMER_VCONN_STABLE);
#endif	/* CONFIG_USB_PD_VCONN_STABLE_DELAY */

	if (!enable)
		PE_RESET_MSG_ID(pd_port, TCPC_TX_SOP_PRIME);

	return ret;
}

static inline int pd_reset_modal_operation(struct pd_port *pd_port)
{
	uint8_t i;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt; i++) {
		svid_data = &pd_port->svid_data[i];

		if (svid_data->active_mode) {
			svid_data->active_mode = 0;
			tcpci_exit_mode(pd_port->tcpc, svid_data->svid);
		}
	}

	pd_port->pe_data.modal_operation = false;
	return 0;
}

int pd_reset_local_hw(struct pd_port *pd_port)
{
	uint8_t dr;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	pd_notify_pe_transit_to_default(pd_port);
	pd_unlock_msg_output(pd_port);

	pd_reset_pe_timer(pd_port);
	pd_set_rx_enable(pd_port, PD_RX_CAP_PE_HARDRESET);

	pd_port->pe_data.explicit_contract = false;
	pd_port->pe_data.pd_connected  = false;
	pd_port->pe_data.pe_ready = false;

#ifdef CONFIG_USB_PD_VCONN_SAFE5V_ONLY
	pd_port->pe_data.vconn_highv_prot = false;
	pd_port->pe_data.vconn_highv_prot_role = PD_ROLE_VCONN_OFF;
#endif	/* CONFIG_USB_PD_VCONN_SAFE5V_ONLY */

#ifdef CONFIG_USB_PD_RESET_CABLE
	dpm_reaction_clear(pd_port, DPM_REACTION_CAP_RESET_CABLE);
#endif	/* CONFIG_USB_PD_RESET_CABLE */

	pd_reset_modal_operation(pd_port);

	pd_set_vconn(pd_port, PD_ROLE_VCONN_OFF);

	if (pd_port->power_role == PD_ROLE_SINK)
		dr = PD_ROLE_UFP;
	else
		dr = PD_ROLE_DFP;

	pd_port->state_machine = PE_STATE_MACHINE_NORMAL;

	pd_set_data_role(pd_port, dr);
	pd_init_spec_revision(pd_port);
	pd_dpm_notify_pe_hardreset(pd_port);
	PE_DBG("reset_local_hw\n");

	return 0;
}

int pd_enable_bist_test_mode(struct pd_port *pd_port, bool en)
{
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PE_DBG("bist_test_mode=%d\n", en);
	return tcpci_set_bist_test_mode(tcpc, en);
}

/* ---- Handle PD Message ----*/

int pd_handle_soft_reset(struct pd_port *pd_port)
{
	PE_STATE_RECV_SOFT_RESET(pd_port);

	pd_reset_protocol_layer(pd_port, true);
	pd_notify_tcp_event_buf_reset(pd_port, TCP_DPM_RET_DROP_RECV_SRESET);
	return pd_send_sop_ctrl_msg(pd_port, PD_CTRL_ACCEPT);
}

void pd_handle_first_pd_command(struct pd_port *pd_port)
{
#ifdef CONFIG_USB_PD_REV30
	pd_sync_sop_spec_revision(pd_port);
#endif	/* CONFIG_USB_PD_REV30 */

	pd_port->pe_data.pd_connected = true;
	pd_port->pe_data.pd_prev_connected = true;
}

void pd_handle_hard_reset_recovery(struct pd_port *pd_port)
{
	/* Stop NoResponseTimer and reset HardResetCounter to zero */
	pd_port->pe_data.hard_reset_counter = 0;
	/* add for PISEN adapter */
	pd_port->pe_data.retry_cnt = 0;
	pd_disable_timer(pd_port, PD_TIMER_NO_RESPONSE);

#ifdef CONFIG_USB_PD_RENEGOTIATION_COUNTER
	pd_port->pe_data.renegotiation_count++;
#endif	/* CONFIG_USB_PD_RENEGOTIATION_COUNTER */

#ifdef CONFIG_USB_PD_RECV_HRESET_COUNTER
	pd_port->pe_data.recv_hard_reset_count = 0;
#endif	/* CONFIG_USB_PD_RECV_HRESET_COUNTER */

	pd_notify_pe_hard_reset_completed(pd_port);
}

/* ---- Send PD Message ----*/

int pd_send_message(struct pd_port *pd_port, uint8_t sop_type,
		uint8_t msg, bool ext, uint16_t count, const uint32_t *data)
{
	int ret;
	uint8_t msg_id;
	uint16_t msg_hdr;
	uint16_t msg_hdr_private;
	uint8_t pd_rev = pd_get_rev(pd_port, sop_type);
	uint8_t type = PD_TX_STATE_WAIT_CRC_PD;
	struct pe_data *pe_data = &pd_port->pe_data;
	struct tcpc_device *tcpc = pd_port->tcpc;

	if (tcpc->typec_attach_old == 0) {
		PE_DBG("[SendMsg] Unattached\n");
		return 0;
	}

	if (tcpc->pd_hard_reset_event_pending) {
		PE_DBG("[SendMsg] HardReset Pending");
		return 0;
	}

	if (sop_type == TCPC_TX_SOP)
		msg_hdr_private = PD_HEADER_ROLE(
			pd_port->power_role, pd_port->data_role);
	else
		msg_hdr_private = 0;

#ifdef CONFIG_USB_PD_REV30
	if (pd_rev >= PD_REV30)
		tcpc->pd_retry_count = PD30_RETRY_COUNT;
	else
		tcpc->pd_retry_count = PD_RETRY_COUNT;
#endif	/* CONFIG_USB_PD_REV30 */

	msg_id = pe_data->msg_id_tx[sop_type];
	msg_hdr = PD_HEADER_COMMON(
		msg, pd_rev, msg_id, count, ext, msg_hdr_private);

	/* ext-cmd 15 is reserved */
	if ((count > 0) && (msg == PD_DATA_VENDOR_DEF))
		type = PD_TX_STATE_WAIT_CRC_VDM;

	pe_data->msg_id_tx[sop_type] = (msg_id+1) % PD_MSG_ID_MAX;

	pd_notify_pe_transmit_msg(pd_port, type);
	ret = tcpci_transmit(pd_port->tcpc, sop_type, msg_hdr, data);
	if (ret < 0)
		PD_ERR("[SendMsg] Failed, %d\n", ret);

	return ret;
}

int pd_send_data_msg(struct pd_port *pd_port,
	uint8_t sop_type, uint8_t msg, uint8_t cnt, uint32_t *payload)
{
	return pd_send_message(pd_port,
		sop_type, msg, false, cnt, payload);
}

int pd_send_sop_ctrl_msg(
	struct pd_port *pd_port, uint8_t msg)
{
	return pd_send_message(
		pd_port, TCPC_TX_SOP, msg, false, 0, NULL);
}

int pd_send_sop_prime_ctrl_msg(struct pd_port *pd_port, uint8_t msg)
{
	return pd_send_message(
		pd_port, TCPC_TX_SOP_PRIME, msg, false, 0, NULL);
}

int pd_send_sop_data_msg(struct pd_port *pd_port,
	uint8_t msg, uint8_t cnt, const uint32_t *payload)
{
	return pd_send_message(pd_port,
		TCPC_TX_SOP, msg, false, cnt, payload);
}

int pd_reply_wait_reject_msg(struct pd_port *pd_port)
{
	PE_STATE_WAIT_TX_SUCCESS(pd_port);
	return pd_reply_wait_reject_msg_no_resp(pd_port);
}

int pd_reply_wait_reject_msg_no_resp(struct pd_port *pd_port)
{
	struct pd_event *pd_event = pd_get_curr_pd_event(pd_port);
	uint8_t msg = pd_event->msg_sec == PD_DPM_NAK_REJECT ?
		PD_CTRL_REJECT : PD_CTRL_WAIT;

	return pd_send_sop_ctrl_msg(pd_port, msg);
}

#ifdef CONFIG_USB_PD_REV30
int pd_send_ext_msg(struct pd_port *pd_port,
		uint8_t sop_type, uint8_t msg, bool request,
		uint8_t chunk_nr, uint8_t size, const uint8_t *data)
{
	uint8_t cnt;
	uint32_t payload[PD_DATA_OBJ_SIZE];

	uint16_t *ext_hdr = (uint16_t *)payload;

	cnt = ((size + PD_EXT_HEADER_PAYLOAD_INDEX - 1) / 4) + 1;
	payload[cnt-1] = 0;		/* Padding Byte should be 0 */

	*ext_hdr = PD_EXT_HEADER_CK(size, request, chunk_nr, true);
	memcpy(&ext_hdr[1], data, size);

	return pd_send_message(pd_port, sop_type, msg, true, cnt, payload);
}
#endif	/* CONFIG_USB_PD_REV30 */

#ifdef CONFIG_USB_PD_RESET_CABLE
int pd_send_cable_soft_reset(struct pd_port *pd_port)
{
	/* reset_protocol_layer */
	PE_RESET_MSG_ID(pd_port, TCPC_TX_SOP_PRIME);
	dpm_reaction_clear(pd_port, DPM_REACTION_CAP_RESET_CABLE);

	return pd_send_sop_prime_ctrl_msg(pd_port, PD_CTRL_SOFT_RESET);
}
#endif	/* CONFIG_USB_PD_RESET_CABLE */

int pd_send_soft_reset(struct pd_port *pd_port)
{
	PE_STATE_SEND_SOFT_RESET(pd_port);

	pd_reset_protocol_layer(pd_port, true);
	pd_notify_tcp_event_buf_reset(pd_port, TCP_DPM_RET_DROP_SENT_SRESET);
	return pd_send_sop_ctrl_msg(pd_port, PD_CTRL_SOFT_RESET);
}

int pd_send_hard_reset(struct pd_port *pd_port)
{
	struct tcpc_device *tcpc = pd_port->tcpc;

	PE_DBG("Send HARD Reset\n");
	__pm_wakeup_event(tcpc->attach_wake_lock, 6000);

	pd_port->pe_data.hard_reset_counter++;
	pd_notify_pe_send_hard_reset(pd_port);

	return tcpci_transmit(tcpc, TCPC_TX_HARD_RESET, 0, NULL);
}

int pd_send_bist_mode2(struct pd_port *pd_port)
{
	int ret = 0;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	pd_notify_tcp_event_buf_reset(pd_port, TCP_DPM_RET_DROP_SEND_BIST);

#ifdef CONFIG_USB_PD_TRANSMIT_BIST2
	TCPC_DBG("BIST_MODE_2\n");
	ret = tcpci_transmit(tcpc, TCPC_TX_BIST_MODE_2, 0, NULL);
#else
	ret = tcpci_set_bist_carrier_mode(tcpc, 1 << 2);
#endif

	return ret;
}

int pd_disable_bist_mode2(struct pd_port *pd_port)
{
#ifndef CONFIG_USB_PD_TRANSMIT_BIST2
	return tcpci_set_bist_carrier_mode(
		pd_port->tcpc, 0);
#else
	return 0;
#endif
}

/* ---- Send / Reply VDM Command ----*/

int pd_send_svdm_request(struct pd_port *pd_port,
		uint8_t sop_type, uint16_t svid, uint8_t vdm_cmd,
		uint8_t obj_pos, uint8_t cnt, uint32_t *data_obj,
		uint32_t timer_id)
{
#ifdef CONFIG_USB_PD_STOP_SEND_VDM_IF_RX_BUSY
	int rv;
	uint32_t alert_status;
#endif	/* CONFIG_USB_PD_STOP_SEND_VDM_IF_RX_BUSY */

	int ret;
	uint8_t ver = SVDM_REV10;
	uint32_t payload[PD_DATA_OBJ_SIZE];

	if (cnt > VDO_MAX_NR) {
		PD_BUG_ON(1);
		return -EINVAL;
	}

	if (pd_get_rev(pd_port, sop_type) >= PD_REV30)
		ver = SVDM_REV20;

	payload[0] = VDO_S(svid, ver, CMDT_INIT, vdm_cmd, obj_pos);
	memcpy(&payload[1], data_obj, sizeof(uint32_t) * cnt);

#ifdef CONFIG_USB_PD_STOP_SEND_VDM_IF_RX_BUSY
	rv = tcpci_get_alert_status(pd_port->tcpc, &alert_status);
	if (rv)
		return rv;

	if (alert_status & TCPC_REG_ALERT_RX_STATUS) {
		PE_DBG("RX Busy, stop send VDM\n");
		return 0;
	}
#endif	/* CONFIG_USB_PD_STOP_SEND_VDM_IF_RX_BUSY */

	ret = pd_send_data_msg(
			pd_port, sop_type, PD_DATA_VENDOR_DEF, 1+cnt, payload);

	if (ret == 0 && timer_id != 0)
		VDM_STATE_RESPONSE_CMD(pd_port, timer_id);

	return ret;
}

int pd_reply_svdm_request(struct pd_port *pd_port,
	uint8_t reply, uint8_t cnt, uint32_t *data_obj)
{
#ifdef CONFIG_USB_PD_STOP_REPLY_VDM_IF_RX_BUSY
	int rv;
	uint32_t alert_status;
#endif	/* CONFIG_USB_PD_STOP_REPLY_VDM_IF_RX_BUSY */
	uint8_t ver = SVDM_REV10;
	uint32_t payload[PD_DATA_OBJ_SIZE];
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	PD_BUG_ON(cnt > VDO_MAX_NR);

	if (pd_check_rev30(pd_port))
		ver = SVDM_REV20;

	payload[0] = VDO_REPLY(ver, reply, pd_get_msg_vdm_hdr(pd_port));

	if (cnt > 0 && cnt <= PD_DATA_OBJ_SIZE - 1) {
		PD_BUG_ON(data_obj == NULL);
		memcpy(&payload[1], data_obj, sizeof(uint32_t) * cnt);
	}

#ifdef CONFIG_USB_PD_STOP_REPLY_VDM_IF_RX_BUSY
	rv = tcpci_get_alert_status(tcpc, &alert_status);
	if (rv)
		return rv;

	if (alert_status & TCPC_REG_ALERT_RX_STATUS) {
		PE_DBG("RX Busy, stop reply VDM\n");
		return 0;
	}
#endif	/* CONFIG_USB_PD_STOP_REPLY_VDM_IF_RX_BUSY */

	if (reply != CMDT_RSP_ACK)
		PE_INFO("VDM_NAK_BUSY\n");
	else
		PE_INFO("VDM_ACK\n");

	VDM_STATE_REPLY_SVDM_REQUEST(pd_port);

	return pd_send_sop_data_msg(pd_port,
			PD_DATA_VENDOR_DEF, 1+cnt, payload);
}

#ifdef CONFIG_USB_PD_CUSTOM_VDM

int pd_send_custom_vdm(struct pd_port *pd_port, uint8_t sop_type)
{
	return pd_send_data_msg(pd_port,
		sop_type, PD_DATA_VENDOR_DEF,
		pd_port->uvdm_cnt, pd_port->uvdm_data);
}

int pd_reply_custom_vdm(struct pd_port *pd_port, uint8_t sop_type,
	uint8_t cnt, uint32_t *payload)
{
	VDM_STATE_REPLY_SVDM_REQUEST(pd_port);

	return pd_send_data_msg(pd_port,
		sop_type, PD_DATA_VENDOR_DEF, cnt, payload);
}

#endif	/* CONFIG_USB_PD_CUSTOM_VDM */

void pd_reset_pe_timer(struct pd_port *pd_port)
{
	tcpc_reset_pe_timer(pd_port->tcpc);

#ifdef CONFIG_USB_PD_REV30_PPS_SINK
	if (pd_port->request_apdo) {
		pd_port->request_apdo = false;
		pd_dpm_start_pps_request_thread(pd_port, false);
	}
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
}

void pd_lock_msg_output(struct pd_port *pd_port)
{
	if (pd_port->msg_output_lock)
		return;
	pd_port->msg_output_lock = true;

	pd_dbg_info_lock();
}

void pd_add_miss_msg(struct pd_port *pd_port,struct pd_event *pd_event,
		     uint8_t msg)
{
	struct pd_msg *pd_msg = pd_event->pd_msg;
	struct pd_msg * miss_msg = NULL;
	uint8_t sop_type = 0;
	struct pd_event evt = {
		.event_type = PD_EVT_CTRL_MSG,
		.msg = msg,
		.pd_msg = NULL,
	};

	if (pd_msg != NULL)
		sop_type = pd_msg->frame_type;
	pd_put_event(pd_port->tcpc, &evt, true);
	miss_msg = pd_alloc_msg(pd_port->tcpc);
	if (miss_msg == NULL)
		return;
	if (pd_msg != NULL)
		memcpy(miss_msg, pd_msg, sizeof(struct pd_msg));

	pd_put_pd_msg_event(pd_port->tcpc,miss_msg);
	pd_port->pe_data.msg_id_rx[sop_type]--;
}

void pd_unlock_msg_output(struct pd_port *pd_port)
{
	if (!pd_port->msg_output_lock)
		return;
	pd_port->msg_output_lock = false;

	pd_dbg_info_unlock();
}

int pd_update_connect_state(struct pd_port *pd_port, uint8_t state)
{
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (pd_port->pd_connect_state == state)
		return 0;

	pd_port->pd_connect_state = state;
	PE_INFO("pd_state=%d\n", state);
	return tcpci_notify_pd_state(tcpc, state);
}

#ifdef CONFIG_USB_PD_REV30

/*
 * Collision Avoidance : check tx ok
 */

#ifndef MIN
#define MIN(a, b)       ((a < b) ? (a) : (b))
#endif

void pd_set_sink_tx(struct pd_port *pd_port, uint8_t cc)
{
#ifdef CONFIG_USB_PD_REV30_COLLISION_AVOID
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (cc == PD30_SINK_TX_OK &&
		pd_port->pe_data.pd_traffic_control != PD_SINK_TX_OK) {
		PE_INFO("sink_tx_ok\n");
		tcpci_lock_typec(tcpc);
		tcpci_set_cc(tcpc, cc);
		tcpci_unlock_typec(tcpc);
		pd_port->pe_data.pd_traffic_control = PD_SINK_TX_OK;
		pd_disable_timer(pd_port, PD_TIMER_SINK_TX);
	} else if (cc == PD30_SINK_TX_NG &&
		pd_port->pe_data.pd_traffic_control == PD_SINK_TX_OK) {
		PE_INFO("sink_tx_ng\n");
		tcpci_lock_typec(tcpc);
		tcpci_set_cc(tcpc, cc);
		tcpci_unlock_typec(tcpc);
		pd_port->pe_data.pd_traffic_control = PD_SINK_TX_NG;
		pd_enable_timer(pd_port, PD_TIMER_SINK_TX);
	}
#endif	/* CONFIG_USB_PD_REV30_COLLISION_AVOID */
}

void pd_sync_sop_spec_revision(struct pd_port *pd_port)
{
#ifdef CONFIG_USB_PD_REV30_SYNC_SPEC_REV
	uint8_t rev = pd_get_msg_hdr_rev(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (!pd_port->pe_data.pd_connected) {
		pd_port->pd_revision[0] = MIN(PD_REV30, rev);
		pd_port->pd_revision[1] = MIN(pd_port->pd_revision[1], rev);

		PE_INFO("pd_rev=%d\n", pd_port->pd_revision[0]);
	}
#endif /* CONFIG_USB_PD_REV30_SYNC_SPEC_REV */
}

void pd_sync_sop_prime_spec_revision(struct pd_port *pd_port, uint8_t rev)
{
#ifdef CONFIG_USB_PD_REV30_SYNC_SPEC_REV
	struct pe_data *pe_data = &pd_port->pe_data;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (!pe_data->cable_rev_discovered) {
		pe_data->cable_rev_discovered = true;
		pd_port->pd_revision[1] = MIN(pd_port->pd_revision[1], rev);
		PE_INFO("cable_rev=%d\n", pd_port->pd_revision[1]);
	}
#endif /* CONFIG_USB_PD_REV30_SYNC_SPEC_REV */
}

bool pd_is_multi_chunk_msg(struct pd_port *pd_port)
{
	uint16_t size;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (pd_get_msg_hdr_ext(pd_port)) {
		size = pd_get_msg_data_size(pd_port);
		if (size > MAX_EXTENDED_MSG_CHUNK_LEN) {
			PE_INFO("multi_chunk_msg = TRUE (%d)\n", size);
			return true;
		}
	}

	return false;
}

struct pd_battery_info *pd_get_battery_info(
	struct pd_port *pd_port, enum pd_battery_reference ref)
{
#ifdef CONFIG_USB_PD_REV30_BAT_INFO
	if (ref < pd_get_fix_battery_nr(pd_port))
		return &pd_port->fix_bat_info[ref];
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */

	/* TODO: for swap battery */
	return NULL;
}

#endif	/* CONFIG_USB_PD_REV30 */
