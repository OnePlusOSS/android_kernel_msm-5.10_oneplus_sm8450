/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * PD Device Policy Manager for UVDM
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

#include "inc/tcpci.h"
#include "inc/pd_policy_engine.h"
#include "inc/pd_dpm_core.h"
#include "pd_dpm_prv.h"

#ifdef CONFIG_USB_PD_RICHTEK_UVDM

bool richtek_dfp_notify_pe_startup(
		struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	UVDM_INFO("%s\n", __func__);
	pd_port->richtek_init_done = false;
	return true;
}

int richtek_dfp_notify_pe_ready(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	if (pd_port->data_role != PD_ROLE_DFP)
		return 0;

	if (pd_port->richtek_init_done)
		return 0;

	pd_port->richtek_init_done = true;
	UVDM_INFO("%s\n", __func__);

#if 0
	pd_port->uvdm_cnt = 3;
	pd_port->uvdm_wait_resp = true;

	pd_port->uvdm_data[0] = PD_UVDM_HDR(USB_VID_RICHTEK, 0x4321);
	pd_port->uvdm_data[1] = 0x11223344;
	pd_port->uvdm_data[2] = 0x44332211;

	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_UVDM);
#endif

	return 1;
}

bool richtek_dfp_notify_uvdm(struct pd_port *pd_port,
				struct svdm_svid_data *svid_data, bool ack)
{
	uint32_t resp_cmd = 0;

	if (ack) {
		if (pd_port->uvdm_wait_resp)
			resp_cmd = PD_UVDM_HDR_CMD(pd_port->uvdm_data[0]);

		UVDM_INFO("dfp_notify: ACK (0x%x)\n", resp_cmd);
	} else
		UVDM_INFO("dfp_notify: NAK\n");

	return true;
}

bool richtek_ufp_notify_uvdm(struct pd_port *pd_port,
				struct svdm_svid_data *svid_data)
{
	uint8_t i;
	uint32_t reply_cmd[VDO_MAX_NR];
	uint16_t cmd = (uint16_t) PD_UVDM_HDR_CMD(pd_port->uvdm_data[0]);

	UVDM_INFO("ufp_notify: 0x%x\n", cmd);

	if (cmd >= 0x1000) {
		UVDM_INFO("uvdm_no_reply\n");
		VDM_STATE_DPM_INFORMED(pd_port);
		return true;
	}

	reply_cmd[0] = PD_UVDM_HDR(USB_VID_RICHTEK, cmd+1);

	for (i = 1; i < pd_port->uvdm_cnt; i++)
		reply_cmd[i] = ~pd_port->uvdm_data[i];

	pd_reply_custom_vdm(pd_port, TCPC_TX_SOP,
		pd_port->uvdm_cnt, reply_cmd);
	VDM_STATE_NORESP_CMD(pd_port);
	return true;
}

#endif	/* CONFIG_USB_PD_RICHTEK_UVDM */
