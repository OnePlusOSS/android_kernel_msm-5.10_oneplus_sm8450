/*
 * Universal Flash Storage Turbo Write
 *
 * Copyright (C) 2017-2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Yongmyung Lee <ymhungry.lee@samsung.com>
 *	Jinyoung Choi <j-young.choi@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#include "ufshcd.h"
#include "ufsfeature.h"
#include "ufstw.h"

static int ufstw_create_sysfs(struct ufsf_feature *ufsf, struct ufstw_lu *tw);

inline int ufstw_get_state(struct ufsf_feature *ufsf)
{
	return atomic_read(&ufsf->tw_state);
}

inline void ufstw_set_state(struct ufsf_feature *ufsf, int state)
{
	atomic_set(&ufsf->tw_state, state);
}

inline bool ufstw_check_query_operational(struct ufstw_lu *tw)
{
	enum UFSTW_STATE cur_state = ufstw_get_state(tw->ufsf);

	return (cur_state == TW_PRESENT || cur_state == TW_SUSPEND);
}

static int ufstw_is_not_present(struct ufsf_feature *ufsf)
{
	enum UFSTW_STATE cur_state = ufstw_get_state(ufsf);

	if (cur_state != TW_PRESENT) {
		INFO_MSG("tw_state != TW_PRESENT (%d)", cur_state);
		return -ENODEV;
	}
	return 0;
}

static int ufstw_read_lu_attr(struct ufstw_lu *tw, u8 idn, u32 *attr_val)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int ret = 0, sel, lun;
	u32 val;

	lun = (tw->lun == TW_LU_SHARED) ? 0 : tw->lun;
	sel = (idn == QUERY_ATTR_IDN_BKOPS_STATUS) ? 0 : UFSFEATURE_SELECTOR;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR, idn,
				      (u8)lun, sel, &val);
	if (ret) {
		ERR_MSG("read attr [0x%.2X](%s) failed. (%d)", idn,
			ATTR_IDN_NAME(idn), ret);
		goto out;
	}

	*attr_val = val;

	INFO_MSG("read attr LUN(%d) [0x%.2X](%s) success (%u)",
		 lun, idn, ATTR_IDN_NAME(idn), *attr_val);
out:
	return ret;
}

static int ufstw_write_lu_attr(struct ufstw_lu *tw, u8 idn, u32 *val)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int ret = 0, lun;

	lun = (tw->lun == TW_LU_SHARED) ? 0 : tw->lun;
	ret = ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR, idn,
				      (u8)lun, UFSFEATURE_SELECTOR, val);
	if (ret) {
		ERR_MSG("write attr [0x%.2X](%s) failed. (%d)", idn,
			ATTR_IDN_NAME(idn), ret);
		goto out;
	}

	INFO_MSG("write attr LUN(%d) [0x%.2X](%s) success (%u)",
		 lun, idn, ATTR_IDN_NAME(idn), *val);
out:
	return ret;
}

static int ufstw_set_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int ret = 0, lun;

	lun = (tw->lun == TW_LU_SHARED) ? 0 : tw->lun;
	ret = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG, idn,
				    (u8)lun, UFSFEATURE_SELECTOR, NULL);
	if (ret) {
		ERR_MSG("set flag [0x%.2X](%s) failed. (%d)", idn,
			FLAG_IDN_NAME(idn), ret);
		goto out;
	}

	if (flag_res) {
		*flag_res = true;

		INFO_MSG("set flag LUN(%d) [0x%.2X](%s) success. (%u)",
			 lun, idn, FLAG_IDN_NAME(idn), *flag_res);
	}
out:
	return ret;
}

static int ufstw_clear_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int ret = 0, lun;

	lun = (tw->lun == TW_LU_SHARED) ? 0 : tw->lun;
	ret = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_CLEAR_FLAG, idn,
				    (u8)lun, UFSFEATURE_SELECTOR, NULL);
	if (ret) {
		ERR_MSG("clear flag [0x%.2X](%s) failed. (%d)", idn,
			FLAG_IDN_NAME(idn), ret);
		goto out;
	}

	*flag_res = false;

	INFO_MSG("clear flag LUN(%d) [0x%.2X](%s) success. (%u)",
		 lun, idn, FLAG_IDN_NAME(idn), *flag_res);
out:
	return ret;
}

static int ufstw_read_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int ret = 0, lun;
	bool val;

	lun = (tw->lun == TW_LU_SHARED) ? 0 : tw->lun;
	ret = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_READ_FLAG, idn,
				    (u8)lun, UFSFEATURE_SELECTOR, &val);
	if (ret) {
		ERR_MSG("read flag [0x%.2X](%s) failed. (%d)", idn,
			FLAG_IDN_NAME(idn), ret);
		goto out;
	}

	*flag_res = val;

	INFO_MSG("read flag LUN(%d) [0x%.2X](%s) success. (%u)",
		 lun, idn, FLAG_IDN_NAME(idn), *flag_res);
out:
	return ret;
}

static inline bool ufstw_is_discard_lrbp(struct ufshcd_lrb *lrbp)
{
	return (lrbp->cmd->cmnd[0] == UNMAP);
}

static inline bool ufstw_is_write_lrbp(struct ufshcd_lrb *lrbp)
{
	return (lrbp->cmd->cmnd[0] == WRITE_10 || lrbp->cmd->cmnd[0] == WRITE_16);
}

static void ufstw_switch_disable_state(struct ufstw_lu *tw)
{
	int ret = 0;

	WARN_MSG("dTurboWriteBUfferLifeTImeEst (0x%.2X)", tw->lifetime_est);
	WARN_MSG("tw-mode will change to disable-mode");

	mutex_lock(&tw->sysfs_lock);
	ufstw_set_state(tw->ufsf, TW_FAILED);
	mutex_unlock(&tw->sysfs_lock);

	if (tw->tw_enable) {
		pm_runtime_get_sync(tw->ufsf->hba->dev);
		ret = ufstw_clear_lu_flag(tw, QUERY_FLAG_IDN_TW_EN,
					  &tw->tw_enable);
		pm_runtime_put_sync(tw->ufsf->hba->dev);
		if (ret)
			WARN_MSG("tw_enable flag clear failed");
	}
}

static int ufstw_check_lifetime_not_guarantee(struct ufstw_lu *tw)
{
	bool disable_flag = false;

	if (tw->lifetime_est & MASK_UFSTW_LIFETIME_NOT_GUARANTEE) {
		if (tw->lun == TW_LU_SHARED)
			WARN_MSG("lun-shared lifetime_est[31] (1)");
		else
			WARN_MSG("lun %d lifetime_est[31] (1)",
				    tw->lun);

		WARN_MSG("Device not guarantee the lifetime of TW Buffer");
#if defined(CONFIG_UFSTW_IGNORE_GUARANTEE_BIT)
		WARN_MSG("but we will ignore them for PoC");
#else
		disable_flag = true;
#endif
	}

	if (disable_flag ||
	    (tw->lifetime_est & ~MASK_UFSTW_LIFETIME_NOT_GUARANTEE) >=
	    UFSTW_MAX_LIFETIME_VALUE) {
		ufstw_switch_disable_state(tw);
		return -ENODEV;
	}

	return 0;
}

static void ufstw_update_resized_buffer(struct ufstw_lu *tw, u32 resize_status)
{
	u32 curr_size;
	int ret;

	if (resize_status == UFSTW_RESIZE_STATUS_SUCCESS) {
		TW_DEBUG(tw, "resize operation is completed");
		ufstw_set_lu_flag(tw,
				  QUERY_FLAG_IDN_TW_BUF_FULL_COUNT_INIT,
				  NULL);
	} else if (resize_status == UFSTW_RESIZE_STATUS_FAIL) {
		INFO_MSG("resize operation is failed");
	}

	/* Read resized buffer size */
	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_CURR_BUF_SIZE,
				 &curr_size);
	if (ret) {
		pm_runtime_put_sync(tw->ufsf->hba->dev);

		TW_DEBUG(tw, "query is failed after completion.");
		TW_DEBUG(tw, "complete resize check anyway.");

		tw->resize_op = UFSTW_RESIZE_NONE;
		return;
	}

	pm_runtime_put_sync(tw->ufsf->hba->dev);

	tw->resize_fully_decreased = false;
	tw->resize_fully_increased = false;

	/* Check resize result */
	if (tw->curr_buffer_size == curr_size) {
		if (tw->resize_op == UFSTW_RESIZE_INCREASE) {
			TW_DEBUG(tw, "buffer isn't increased");
			tw->resize_fully_increased = true;
		} else {
			TW_DEBUG(tw, "buffer isn't decreased");
			tw->resize_fully_decreased = true;
		}
	}

	TW_DEBUG(tw, "current_buffer_size: %u -> %u",
		 tw->curr_buffer_size, curr_size);

	tw->curr_buffer_size = curr_size;
	tw->resize_op = UFSTW_RESIZE_NONE;
}

static bool ufstw_is_resize_available(struct ufstw_lu *tw)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	u32 resize_not_avail, resize_status;
	int ret;

	/* Read resize related attributes */
	pm_runtime_get_sync(hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_NOT_AVAIL,
				 &resize_not_avail);
	if (ret || resize_not_avail) {
		pm_runtime_put_sync(hba->dev);
		TW_DEBUG(tw, "resize is not available");
		return false;
	}

	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_STATUS,
				 &resize_status);
	pm_runtime_put_sync(hba->dev);
	if (ret || resize_status == UFSTW_RESIZE_STATUS_IN_PROGRESS) {
		ERR_MSG("resize is in progress. resize_op %d", tw->resize_op);
		return false;
	}

	/* resize completion */
	if (resize_status != UFSTW_RESIZE_STATUS_IDLE)
		ufstw_update_resized_buffer(tw, resize_status);

	return true;
}

static inline u32 get_prov_max_buf_size(struct ufstw_lu *tw) {
	return TW_SHARED_BUF(tw->ufsf) ?
		tw->ufsf->tw_dev_info.tw_shared_buf_alloc_units :
		tw->lu_buf_size;
}

/*
 * Resize priority (decrease > increase)
 */
static bool ufstw_is_resize_needed(struct ufstw_lu *tw)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	u32 bkops_status;
	u32 full_count;
	int ret;

	pm_runtime_get_sync(hba->dev);

	/* Check bkops_status */
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_BKOPS_STATUS,
				 &bkops_status);
	if (ret)
		goto put_sync_out;

	TW_DEBUG(tw, "bkops_status = %u", bkops_status);

	/* High device fill ratio, buffer should be decreased. */
	if (bkops_status > BKOPS_STATUS_NON_CRITICAL) {
		/* Buffer can't be decreased. */
		if (tw->resize_fully_decreased) {
			TW_DEBUG(tw, "buffer is fully decreased, do nothing");
			goto put_sync_out;
		}

		/* Buffer can be decreased */
		TW_DEBUG(tw, "decrease buffer");
		tw->resize_op = UFSTW_RESIZE_DECREASE;
		goto put_sync_out;
	}

	/* Check full_count */
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_FULL_COUNT,
				 &full_count);
	if (ret)
		goto put_sync_out;

	TW_DEBUG(tw, "full_count = %u", full_count);

	/* full_count set, buffer should be increased. */
	if (full_count) {
		u32 prov_max_buf_size = get_prov_max_buf_size(tw);

		/* Buffer can't be increased. */
		if (tw->resize_fully_increased ||
		    tw->curr_buffer_size >= prov_max_buf_size) {
			TW_DEBUG(tw, "buffer is fully increased.");
			TW_DEBUG(tw, "curr_buffer_size %d prov_max_buf_size %d",
				 tw->curr_buffer_size, prov_max_buf_size);
			goto put_sync_out;
		}

		/* Buffer can be increased */
		TW_DEBUG(tw, "increase buffer");
		tw->resize_op = UFSTW_RESIZE_INCREASE;
		goto put_sync_out;
	}

	/* Buffer resizing is not needed */
	TW_DEBUG(tw, "resize is not needed");

put_sync_out:
	pm_runtime_put_sync(hba->dev);

	return tw->resize_op != UFSTW_RESIZE_NONE;
}

static void ufstw_interval_work_fn(struct work_struct *work)
{
	struct ufstw_lu *tw;
	int ret;

	tw = container_of(work, struct ufstw_lu, tw_interval_work);

	if (ufstw_is_not_present(tw->ufsf))
		return;

	TW_DEBUG(tw, "worker_start");

	pm_runtime_get_sync(tw->ufsf->hba->dev);

	/* Lifetime Est Check */
	if (tw->schedule_lifetime) {
		tw->schedule_lifetime = false;
		TW_DEBUG(tw, "lifetime Est Check");

		ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST,
					 &tw->lifetime_est);
		if (ret)
			goto put_sync_out;

		ret = ufstw_check_lifetime_not_guarantee(tw);
		if (ret)
			goto put_sync_out;
	}

	/* Resize Check */
	if (tw->schedule_resize) {
		tw->schedule_resize = false;
		TW_DEBUG(tw, "Resize Check");

		if (!tw->resize_auto)
			goto put_sync_out;

		/* Read resize related attributes */
		if (!ufstw_is_resize_available(tw))
			goto put_sync_out;

		/* Determine resizing */
		if (!ufstw_is_resize_needed(tw))
			goto put_sync_out;

		/* Request resize operation */
		ret = ufstw_write_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE,
					  &tw->resize_op);
		if (ret) {
			ERR_MSG("resize query is failed. set as UFSTW_RESIZE_NONE");
			tw->resize_op = UFSTW_RESIZE_NONE;
		}
	}

put_sync_out:
	pm_runtime_put_sync(tw->ufsf->hba->dev);

	TW_DEBUG(tw, "worker_end");
}

void ufstw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	struct ufstw_lu *tw;

	if (!lrbp || !ufsf_is_valid_lun(lrbp->lun))
		return;

	tw = ufsf->tw_lup[lrbp->lun];
	if (!tw || !tw->tw_enable)
		return;

	if (ufstw_is_discard_lrbp(lrbp)) {
		tw->resize_fully_increased = false;
		return;
	}

	if (!ufstw_is_write_lrbp(lrbp))
		return;

	/* Calculate cumulative write amount */
	spin_lock_bh(&tw->write_calc_lock);

	tw->write_lifetime += blk_rq_sectors(lrbp->cmd->request);
	TW_FTRACE(tw, "tw_lifetime: (%07u / %07u)",
		  tw->write_lifetime, tw->write_lifetime_interval);

	if (tw->write_lifetime > tw->write_lifetime_interval) {
		tw->write_lifetime = 0;
		tw->schedule_lifetime = true;
	}

	tw->write_resize += blk_rq_sectors(lrbp->cmd->request);
	TW_FTRACE(tw, "tw_resize: (%07u / %07u)",
		  tw->write_resize, tw->write_resize_interval);

	if (tw->write_resize > tw->write_resize_interval) {
		tw->write_resize = 0;
		tw->schedule_resize = true;
	}

	spin_unlock_bh(&tw->write_calc_lock);

	/* Schdeule worker */
	if (tw->schedule_lifetime || tw->schedule_resize) {
		TW_FTRACE(tw, "tw_interval_work scheduled");
		schedule_work(&tw->tw_interval_work);
	}
}

static inline void ufstw_init_lu_jobs(struct ufstw_lu *tw)
{
	INIT_WORK(&tw->tw_interval_work, ufstw_interval_work_fn);
}

static inline void ufstw_cancel_lu_jobs(struct ufstw_lu *tw)
{
	int ret;

	ret = cancel_work_sync(&tw->tw_interval_work);
	INFO_MSG("cancel_work_sync(tw_interval_work) ufstw_lu[%d] (%d)",
		 tw->lun, ret);
}

static inline int ufstw_version_check(struct ufstw_dev_info *tw_dev_info)
{
	INFO_MSG("Support TW Spec : Driver = %.4X, Device = %.4X",
		 UFSTW_VER, tw_dev_info->tw_ver);

	INFO_MSG("TW Driver Version : %.6X%s", UFSTW_DD_VER,
		 UFSTW_DD_VER_POST);

	if (tw_dev_info->tw_ver != UFSTW_VER)
		return -ENODEV;

	return 0;
}

#if defined(CONFIG_UFSTW_RESTORE_WB)
static int ufstw_wb_probe(struct ufs_hba *hba, u8* desc_buf)
{
	struct ufs_dev_info *dev_info = &hba->dev_info;
	u8 lun;
	u32 d_lu_wb_buf_alloc;

	/* Device support is already checked in ufstw_get_dev_info() */
	dev_info->d_ext_ufs_feature_sup =
		get_unaligned_be32(desc_buf +
				   DEVICE_DESC_PARAM_EXT_UFS_FEATURE_SUP);

	/*
	 * WB may be supported but not configured while provisioning.
	 * The spec says, in dedicated wb buffer mode,
	 * a max of 1 lun would have wb buffer configured.
	 * Now only shared buffer mode is supported.
	 */
	dev_info->b_wb_buffer_type =
		desc_buf[DEVICE_DESC_PARAM_WB_TYPE];

	dev_info->b_presrv_uspc_en =
		desc_buf[DEVICE_DESC_PARAM_WB_PRESRV_USRSPC_EN];

	if (dev_info->b_wb_buffer_type == WB_BUF_MODE_SHARED) {
		dev_info->d_wb_alloc_units =
		get_unaligned_be32(desc_buf +
				   DEVICE_DESC_PARAM_WB_SHARED_ALLOC_UNITS);
		if (!dev_info->d_wb_alloc_units)
			return -ENODEV;
	} else {
		for (lun = 0; lun < UFS_UPIU_MAX_WB_LUN_ID; lun++) {
			d_lu_wb_buf_alloc = 0;
			ufshcd_read_desc_param(hba,
					QUERY_DESC_IDN_UNIT,
					lun,
					UNIT_DESC_PARAM_WB_BUF_ALLOC_UNITS,
					(u8 *)&d_lu_wb_buf_alloc,
					sizeof(d_lu_wb_buf_alloc));
			if (d_lu_wb_buf_alloc) {
				dev_info->wb_dedicated_lu = lun;
				break;
			}
		}

		if (!d_lu_wb_buf_alloc)
			return -ENODEV;
	}

	hba->caps |= UFSHCD_CAP_WB_EN;
	return 0;
}

static inline u8 ufstw_wb_get_query_index(struct ufs_hba *hba)
{
	if (hba->dev_info.b_wb_buffer_type == WB_BUF_MODE_LU_DEDICATED)
		return hba->dev_info.wb_dedicated_lu;
	return 0;
}

static inline int ufstw_wb_enable(struct ufs_hba *hba, u8 index)
{
	return ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
				      QUERY_FLAG_IDN_WB_EN, index, NULL);
}

static inline int ufstw_wb_flush_during_h8_enable(struct ufs_hba *hba, u8 index)
{
	return ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
				QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
				index, NULL);
}

static inline int ufstw_wb_buf_flush_enable(struct ufs_hba *hba, u8 index)
{
	return ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
				      QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN,
				      index, NULL);
}

static void ufstw_restore_wb(struct ufs_hba *hba, u8 *desc_buf)
{
	int ret;
	u8 index;

	ret = ufstw_wb_probe(hba, desc_buf);
	if (ret) {
		ERR_MSG("WB is not supported %d", ret);
		return;
	}

	index = ufstw_wb_get_query_index(hba);
	ret = ufstw_wb_enable(hba, index);
	if (ret) {
		ERR_MSG("Enable WB is failed %d", ret);
		return;
	}

	hba->wb_enabled = true;
	INFO_MSG("Write Booster is Configured");

	ret = ufstw_wb_flush_during_h8_enable(hba, index);
	if (ret) {
		ERR_MSG("Enable WB flush during H8 is failed: %d", ret);
		return;
	}

	INFO_MSG("WB flush during H8 is enabled");

	if (hba->quirks & UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL) {
		INFO_MSG("Manual WB flush is set.");
		return;
	}

	ret = ufstw_wb_buf_flush_enable(hba, index);
	if (ret) {
		ERR_MSG("Enable WB flush is failed: %d", ret);
		return;
	}

	hba->wb_buf_flush_enabled = true;
	INFO_MSG("WB flush is enabled");
}
#endif

void ufstw_get_dev_info(struct ufsf_feature *ufsf, u8 *desc_buf)
{
	struct ufstw_dev_info *tw_dev_info = &ufsf->tw_dev_info;

	if (get_unaligned_be32(desc_buf + DEVICE_DESC_PARAM_EX_FEAT_SUP) &
		     UFS_FEATURE_SUPPORT_TW_BIT) {
		INFO_MSG("bUFSExFeaturesSupport: TW is set");
	} else {
		ERR_MSG("bUFSExFeaturesSupport: TW not support");
		ufstw_set_state(ufsf, TW_FAILED);
		return;
	}
	tw_dev_info->tw_buf_no_reduct =
		desc_buf[DEVICE_DESC_PARAM_TW_RETURN_TO_USER];
	tw_dev_info->tw_buf_type = desc_buf[DEVICE_DESC_PARAM_TW_BUF_TYPE];
	tw_dev_info->tw_shared_buf_alloc_units = get_unaligned_be32(
			desc_buf + DEVICE_DESC_PARAM_TW_SHARED_BUF_ALLOC_UNITS);
	tw_dev_info->tw_ver = get_unaligned_be16(
			desc_buf + DEVICE_DESC_PARAM_TW_VER);

	if (ufstw_version_check(tw_dev_info)) {
		ERR_MSG("TW Spec Version mismatch. TW disabled");
		ufstw_set_state(ufsf, TW_FAILED);
#if defined(CONFIG_UFSTW_RESTORE_WB)
		ufstw_restore_wb(ufsf->hba, desc_buf);
#endif
		return;
	}

	INFO_MSG("tw_dev [53] bTurboWriteBufferNoUserSpaceReductionEn (%u)",
		 tw_dev_info->tw_buf_no_reduct);
	INFO_MSG("tw_dev [54] bTurboWriteBufferType (%u)",
		 tw_dev_info->tw_buf_type);
	INFO_MSG("tw_dev [55] dNumSharedTUrboWriteBufferAllocUnits (%u)",
		 tw_dev_info->tw_shared_buf_alloc_units);

	if (TW_SHARED_BUF(ufsf) &&
	    tw_dev_info->tw_shared_buf_alloc_units == 0) {
		ERR_MSG("TW use shared buffer. But alloc unit is (0)");
		ufstw_set_state(ufsf, TW_FAILED);
		return;
	}
}

void ufstw_get_geo_info(struct ufsf_feature *ufsf, u8 *geo_buf)
{
	struct ufstw_dev_info *tw_dev_info = &ufsf->tw_dev_info;

	tw_dev_info->tw_number_lu = geo_buf[GEOMETRY_DESC_TW_NUMBER_LU];
	if (tw_dev_info->tw_number_lu == 0) {
		ERR_MSG("Turbo Write is not supported");
		ufstw_set_state(ufsf, TW_FAILED);
		return;
	}

	INFO_MSG("tw_geo [4F:52] dTurboWriteBufferMaxNAllocUnits (%u)",
		get_unaligned_be32(geo_buf + GEOMETRY_DESC_TW_MAX_SIZE));
	INFO_MSG("tw_geo [53] bDeviceMaxTurboWriteLUs (%u)",
		 tw_dev_info->tw_number_lu);
	INFO_MSG("tw_geo [54] bTurboWriteBufferCapAdjFac (%u)",
		 geo_buf[GEOMETRY_DESC_TW_CAP_ADJ_FAC]);
	INFO_MSG("tw_geo [55] bSupportedTWBufferUserSpaceReductionTypes (%u)",
		 geo_buf[GEOMETRY_DESC_TW_SUPPORT_USER_REDUCTION_TYPES]);
	INFO_MSG("tw_geo [56] bSupportedTurboWriteBufferTypes (%u)",
		 geo_buf[GEOMETRY_DESC_TW_SUPPORT_BUF_TYPE]);
}

static void ufstw_alloc_shared_lu(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;

	tw = kzalloc(sizeof(struct ufstw_lu), GFP_KERNEL);
	if (!tw) {
		ERR_MSG("ufstw_lu[shared] memory alloc failed");
		return;
	}

	tw->lun = TW_LU_SHARED;
	tw->ufsf = ufsf;
	ufsf->tw_lup[0] = tw;

	INFO_MSG("ufstw_lu[shared] is TurboWrite-Enabled");
}

static void ufstw_get_lu_info(struct ufsf_feature *ufsf, int lun, u8 *lu_buf)
{
	struct ufsf_lu_desc lu_desc;
	struct ufstw_lu *tw;

	lu_desc.tw_lu_buf_size = get_unaligned_be32(
			lu_buf + UNIT_DESC_TW_LU_WRITE_BUFFER_ALLOC_UNIT);

	ufsf->tw_lup[lun] = NULL;

	if (lu_desc.tw_lu_buf_size) {
		ufsf->tw_lup[lun] =
			kzalloc(sizeof(struct ufstw_lu), GFP_KERNEL);
		if (!ufsf->tw_lup[lun]) {
			ERR_MSG("ufstw_lu[%d] memory alloc faild", lun);
			return;
		}

		tw = ufsf->tw_lup[lun];
		tw->ufsf = ufsf;
		tw->lun = lun;
		tw->lu_buf_size = lu_desc.tw_lu_buf_size;
		INFO_MSG("ufstw_lu[%d] [29:2C] dLUNumTWBufferAllocUnits (%u)",
			 lun, lu_desc.tw_lu_buf_size);
		INFO_MSG("ufstw_lu[%d] is TurboWrite-Enabled.", lun);
	} else {
		INFO_MSG("ufstw_lu[%d] [29:2C] dLUNumTWBufferAllocUnits (%u)",
			 lun, lu_desc.tw_lu_buf_size);
		INFO_MSG("ufstw_lu[%d] is TurboWrite-disabled", lun);
	}
}

inline void ufstw_alloc_lu(struct ufsf_feature *ufsf, int lun, u8 *lu_buf)
{
	if (TW_SHARED_BUF(ufsf) && !ufsf->tw_lup[0])
		ufstw_alloc_shared_lu(ufsf);
	else
		ufstw_get_lu_info(ufsf, lun, lu_buf);
}

static inline void ufstw_print_lu_flag_attr(struct ufstw_lu *tw)
{
	char lun_str[20] = { 0 };

	if (tw->lun == TW_LU_SHARED)
		snprintf(lun_str, 7, "shared");
	else
		snprintf(lun_str, 2, "%d", tw->lun);

	INFO_MSG("tw_flag ufstw_lu[%s] IDN (0x%.2X) tw_enable (%d)",
		 lun_str, QUERY_FLAG_IDN_TW_EN, tw->tw_enable);
	INFO_MSG("tw_flag ufstw_lu[%s] IDN (0x%.2X) flush_enable (%d)",
		 lun_str, QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
		 tw->flush_enable);
	INFO_MSG("tw_flag ufstw_lu[%s] IDN (0x%.2X) flush_hibern (%d)",
		 lun_str, QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
		 tw->flush_during_hibern_enter);

	INFO_MSG("tw_attr ufstw_lu[%s] IDN (0x%.2X) flush_status (%u)",
		 lun_str, QUERY_ATTR_IDN_TW_FLUSH_STATUS, tw->flush_status);
	INFO_MSG("tw_attr ufstw_lu[%s] IDN (0x%.2X) buffer_size (%u)",
		 lun_str, QUERY_ATTR_IDN_TW_AVAIL_BUF_SIZE,
		 tw->available_buffer_size);
	INFO_MSG("tw_attr ufstw_lu[%s] IDN (0x%.2X) buffer_lifetime (0x%.2X)",
		 lun_str, QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST,
		 tw->lifetime_est);
	INFO_MSG("tw_attr ufstw_lu[%s] IDN (0x%.2X) current_buffer_size (0x%.2X)",
		 lun_str, QUERY_ATTR_IDN_TW_CURR_BUF_SIZE,
		 tw->curr_buffer_size);
}

static inline void ufstw_lu_update(struct ufstw_lu *tw)
{
	/* Flag */
	pm_runtime_get_sync(tw->ufsf->hba->dev);
	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_TW_EN, &tw->tw_enable))
		goto error_put;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
			       &tw->flush_enable))
		goto error_put;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
			       &tw->flush_during_hibern_enter))
		goto error_put;

	/* Attribute */
	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_FLUSH_STATUS,
			       &tw->flush_status))
		goto error_put;

	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_AVAIL_BUF_SIZE,
			       &tw->available_buffer_size))
		goto error_put;

	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST,
			       &tw->lifetime_est))
		goto error_put;

	ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_CURR_BUF_SIZE,
			       &tw->curr_buffer_size);
error_put:
	pm_runtime_put_sync(tw->ufsf->hba->dev);
}

static int ufstw_lu_init(struct ufsf_feature *ufsf, int lun)
{
	struct ufstw_lu *tw;
	int ret = 0;

	if (lun == TW_LU_SHARED)
		tw = ufsf->tw_lup[0];
	else
		tw = ufsf->tw_lup[lun];

	tw->ufsf = ufsf;
	spin_lock_init(&tw->write_calc_lock);

	tw->debug = false;

	ufstw_lu_update(tw);

	ret = ufstw_check_lifetime_not_guarantee(tw);
	if (ret)
		goto err_out;

	ufstw_print_lu_flag_attr(tw);

	tw->write_lifetime = 0;
	tw->write_lifetime_interval = UFSTW_LIFETIME_WRITE_SECT_INTERVAL_DEFAULT;
	tw->write_resize = 0;
	tw->write_resize_interval = UFSTW_RESIZE_WRITE_SECT_INTERVAL_DEFAULT;

	tw->schedule_lifetime = false;
	tw->schedule_resize = false;

	tw->resize_op = UFSTW_RESIZE_NONE;

	/* resize_auto */
	tw->resize_auto = false;
	tw->resize_fully_increased = false;
	tw->resize_fully_decreased = false;

	ufstw_init_lu_jobs(tw);

	pm_runtime_get_sync(ufsf->hba->dev);
	ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_BUF_FULL_COUNT_INIT, NULL);
#if defined(CONFIG_UFSTW_BOOT_ENABLED)
	ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_EN, &tw->tw_enable);
	ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
			  &tw->flush_during_hibern_enter);
#endif
	pm_runtime_put_sync(ufsf->hba->dev);

	ret = ufstw_create_sysfs(ufsf, tw);
	if (ret)
		ERR_MSG("create sysfs failed");
err_out:
	return ret;
}

static inline void ufstw_remove_sysfs(struct ufstw_lu *tw)
{
	int ret;

	ret = kobject_uevent(&tw->kobj, KOBJ_REMOVE);
	INFO_MSG("kobject removed (%d)", ret);
	kobject_del(&tw->kobj);
}

void ufstw_init(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun, ret = 0;
	unsigned int tw_enabled_lun = 0;

	INFO_MSG("init start.. tw_state (%d)", ufstw_get_state(ufsf));

	if (TW_SHARED_BUF(ufsf)) {
		if (!ufsf->tw_lup[0]) {
			ERR_MSG("tw_lup memory allocation failed");
			goto out;
		}
		BUG_ON(ufsf->tw_lup[0]->lun != TW_LU_SHARED);

		ret = ufstw_lu_init(ufsf, TW_LU_SHARED);
		if (ret)
			goto out_free_mem;

		INFO_MSG("ufstw_lu[shared] working");
		tw_enabled_lun++;
	} else {
		seq_scan_lu(lun) {
			if (!ufsf->tw_lup[lun])
				continue;

			ret = ufstw_lu_init(ufsf, lun);
			if (ret)
				goto out_kobj_del;

			INFO_MSG("ufstw_lu[%d] working", lun);
			tw_enabled_lun++;
		}

		if (tw_enabled_lun > ufsf->tw_dev_info.tw_number_lu) {
			ERR_MSG("lu count mismatched");
			goto out_kobj_del;
		}
	}

	if (tw_enabled_lun == 0) {
		ERR_MSG("tw_enabled_lun count zero");
		goto out_free_mem;
	}

	ufstw_set_state(ufsf, TW_PRESENT);

	return;

out_kobj_del:
	seq_scan_lu(lun) {
		if (!tw_enabled_lun)
			break;

		tw = ufsf->tw_lup[lun];
		if (!tw)
			continue;

		INFO_MSG("ufstw_lu[%d] %p disabled", lun, tw);
		ufstw_remove_sysfs(tw);
		tw_enabled_lun--;
	}

out_free_mem:
	seq_scan_lu(lun) {
		tw = ufsf->tw_lup[lun];
		if (!tw)
			continue;

		INFO_MSG("ufstw_lu[%d] %p removed", lun, tw);
		kfree(ufsf->tw_lup[lun]);
		ufsf->tw_lup[lun] = NULL;
	}

out:
	ERR_MSG("Turbo write initialization failed");

	ufstw_set_state(ufsf, TW_FAILED);
}

void ufstw_remove(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun;

	dump_stack();
	INFO_MSG("start release");

	ufstw_set_state(ufsf, TW_FAILED);

	if (TW_SHARED_BUF(ufsf)) {
		tw = ufsf->tw_lup[0];
		INFO_MSG("ufstw_lu[shared] %p", tw);
		ufsf->tw_lup[0] = NULL;
		ufstw_cancel_lu_jobs(tw);
		ufstw_remove_sysfs(tw);
		kfree(tw);
	} else {
		seq_scan_lu(lun) {
			tw = ufsf->tw_lup[lun];
			INFO_MSG("ufstw_lu[%d] %p", lun, tw);

			if (!tw)
				continue;
			ufsf->tw_lup[lun] = NULL;
			ufstw_cancel_lu_jobs(tw);
			ufstw_remove_sysfs(tw);
			kfree(tw);
		}
	}

	INFO_MSG("end release");
}

static void ufstw_reset_query_handling(struct ufstw_lu *tw)
{
	int ret;

	pm_runtime_get(tw->ufsf->hba->dev);
	if (tw->tw_enable) {
		ret = ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_EN,
					&tw->tw_enable);
		if (ret)
			tw->tw_enable = false;
	}

	if (tw->flush_enable) {
		ret = ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
					&tw->flush_enable);
		if (ret)
			tw->flush_enable = false;
	}

	if (tw->flush_during_hibern_enter) {
		ret = ufstw_set_lu_flag(tw,
					QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
					&tw->flush_during_hibern_enter);
		if (ret)
			tw->flush_during_hibern_enter = false;
	}

	pm_runtime_put_noidle(tw->ufsf->hba->dev);
}

static void ufstw_restore_flags(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun;

	if (TW_SHARED_BUF(ufsf)) {
		tw = ufsf->tw_lup[0];

		INFO_MSG("ufstw_lu[shared] restore");
		ufstw_reset_query_handling(tw);
	} else {
		seq_scan_lu(lun) {
			tw = ufsf->tw_lup[lun];
			if (!tw)
				continue;

			INFO_MSG("ufstw_lu[%d] restore", lun);
			ufstw_reset_query_handling(tw);
		}
	}
}

void ufstw_suspend(struct ufsf_feature *ufsf)
{
	INFO_MSG("ufstw suspend.");
	ufstw_set_state(ufsf, TW_SUSPEND);
}

void ufstw_resume(struct ufsf_feature *ufsf, bool is_link_off)
{
	INFO_MSG("ufstw resume start.");
	ufstw_set_state(ufsf, TW_PRESENT);

	if (is_link_off)
		ufstw_restore_flags(ufsf);

	INFO_MSG("ufstw resume finish.");
}

void ufstw_reset_host(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun;

	if (ufstw_is_not_present(ufsf))
		return;

	ufstw_set_state(ufsf, TW_RESET);
	if (TW_SHARED_BUF(ufsf)) {
		tw = ufsf->tw_lup[0];

		INFO_MSG("ufstw_lu[shared] cancel jobs");
		ufstw_cancel_lu_jobs(tw);
	} else {
		seq_scan_lu(lun) {
			tw = ufsf->tw_lup[lun];
			if (!tw)
				continue;

			INFO_MSG("ufstw_lu[%d] cancel jobs", lun);
			ufstw_cancel_lu_jobs(tw);
		}
	}
}

void ufstw_reset(struct ufsf_feature *ufsf)
{
	ufstw_set_state(ufsf, TW_PRESENT);

	ufstw_restore_flags(ufsf);
}

#define ufstw_sysfs_internal_show_func(_name)				\
static ssize_t ufstw_sysfs_show_##_name(struct ufstw_lu *tw, char *buf)	\
{									\
	INFO_MSG(""#_name": %u", tw->_name);				\
									\
	return snprintf(buf, PAGE_SIZE, ""#_name": %u\n", tw->_name);	\
}

#define ufstw_sysfs_internal_store_func(_name, _min, _max)		\
static ssize_t ufstw_sysfs_store_##_name(struct ufstw_lu *tw,		\
					 const char *buf, size_t count)	\
{									\
	unsigned long val;						\
									\
	if (kstrtoul(buf, 0, &val))					\
		return -EINVAL;						\
									\
	if (val < _min || val > _max) {					\
		INFO_MSG(""#_name": (min) %u ~ (max) %u", _min, _max);	\
		return -EINVAL;						\
	}								\
									\
	tw->_name = (unsigned int)val;					\
	INFO_MSG(""#_name" %u", tw->_name);				\
									\
	return count;							\
}

#define ufstw_sysfs_attr_show_func(_query, _name, _IDN, hex)		\
static ssize_t ufstw_sysfs_show_##_name(struct ufstw_lu *tw, char *buf)	\
{									\
	int ret;							\
									\
	if (!ufstw_check_query_operational(tw))				\
		return -ENODEV;						\
									\
	pm_runtime_get_sync(tw->ufsf->hba->dev);			\
	ret = ufstw_read_lu_##_query(tw, _IDN, &tw->_name);		\
	pm_runtime_put_sync(tw->ufsf->hba->dev);			\
	if (ret)							\
		return -ENODEV;						\
									\
	INFO_MSG("read "#_query" "#_name" %u (0x%X)",			\
		 tw->_name, tw->_name);					\
	if (hex)							\
		return snprintf(buf, PAGE_SIZE, "0x%.2X\n", tw->_name);	\
	return snprintf(buf, PAGE_SIZE, "%u\n", tw->_name);		\
}

#define ufstw_sysfs_attr_store_func(_name, _IDN)			\
static ssize_t ufstw_sysfs_store_##_name(struct ufstw_lu *tw,		\
					 const char *buf,		\
					 size_t count)			\
{									\
	unsigned long val;						\
	ssize_t ret =  count;						\
									\
	if (kstrtoul(buf, 0, &val))					\
		return -EINVAL;						\
									\
	if (!(val == 0  || val == 1))					\
		return -EINVAL;						\
									\
	INFO_MSG("val %lu", val);					\
	if (!ufstw_check_query_operational(tw))				\
		return -ENODEV;						\
									\
	pm_runtime_get_sync(tw->ufsf->hba->dev);			\
	if (val) {							\
		if (ufstw_set_lu_flag(tw, _IDN, &tw->_name))		\
			ret = -ENODEV;					\
	} else {							\
		if (ufstw_clear_lu_flag(tw, _IDN, &tw->_name))		\
			ret = -ENODEV;					\
	}								\
	pm_runtime_put_sync(tw->ufsf->hba->dev);			\
									\
	INFO_MSG(#_name " query success");				\
	return ret;							\
}

/* TW Internal sysfs functions */
ufstw_sysfs_internal_show_func(write_resize_interval);
ufstw_sysfs_internal_store_func(write_resize_interval,
				UFSTW_RESIZE_WRITE_SECT_INTERVAL_MIN,
				UFSTW_RESIZE_WRITE_SECT_INTERVAL_MAX);

static ssize_t ufstw_sysfs_show_resize_auto(struct ufstw_lu *tw, char *buf)
{
	INFO_MSG("resize_auto: %u (%s)", tw->resize_auto,
		 RESIZE_AUTO_TO_STR(tw->resize_auto));

	return snprintf(buf, PAGE_SIZE, "%u\n", tw->resize_auto);
}

static ssize_t ufstw_sysfs_store_resize_auto(struct ufstw_lu *tw,
					     const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (!(val == 0 || val == 1))
		return -EINVAL;

	tw->resize_auto = val ? true : false;

	INFO_MSG("resize_auto: %u (%s)", tw->resize_auto,
		 RESIZE_AUTO_TO_STR(tw->resize_auto));

	return count;
}

#if defined(CONFIG_UFSTW_DEBUG)
static ssize_t ufstw_sysfs_show_debug(struct ufstw_lu *tw, char *buf)
{
	INFO_MSG("debug %d", tw->debug);

	return snprintf(buf, PAGE_SIZE, "debug %d\n", tw->debug);
}

static ssize_t ufstw_sysfs_store_debug(struct ufstw_lu *tw,
				       const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (!(val == 0 || val == 1))
		return -EINVAL;

	tw->debug = val ? true : false;

	INFO_MSG("debug %d", tw->debug);

	return count;
}
#endif

/* Flag related sysfs functions */
ufstw_sysfs_attr_show_func(flag, tw_enable, QUERY_FLAG_IDN_TW_EN, 0);
ufstw_sysfs_attr_store_func(tw_enable, QUERY_FLAG_IDN_TW_EN);
ufstw_sysfs_attr_show_func(flag, flush_enable,
			   QUERY_FLAG_IDN_TW_BUF_FLUSH_EN, 0);
ufstw_sysfs_attr_store_func(flush_enable, QUERY_FLAG_IDN_TW_BUF_FLUSH_EN);
ufstw_sysfs_attr_show_func(flag, flush_during_hibern_enter,
			   QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN, 0);
ufstw_sysfs_attr_store_func(flush_during_hibern_enter,
			    QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN);

static ssize_t ufstw_sysfs_store_full_count_init(struct ufstw_lu *tw,
						 const char *buf, size_t count)
{
	unsigned long val;
	ssize_t ret;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (!(val == 1))
		return -EINVAL;

	if (tw->resize_auto) {
		INFO_MSG("NOT ALLOWED: resize_auto: 1 (ON)");
		return -ENODEV;
	}

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_TW_BUF_FULL_COUNT_INIT,
				NULL);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("full_count_init set flag success");

	return count;
}

/* Attribute related sysfs functions */
ufstw_sysfs_attr_show_func(attr, flush_status,
			   QUERY_ATTR_IDN_TW_FLUSH_STATUS, 0);
ufstw_sysfs_attr_show_func(attr, available_buffer_size,
			   QUERY_ATTR_IDN_TW_AVAIL_BUF_SIZE, 0);
ufstw_sysfs_attr_show_func(attr, lifetime_est,
			   QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST, 1);
ufstw_sysfs_attr_show_func(attr, curr_buffer_size,
			   QUERY_ATTR_IDN_TW_CURR_BUF_SIZE, 0);

static ssize_t ufstw_sysfs_show_max_no_flush_alloc_units(struct ufstw_lu *tw,
							 char *buf)
{
	u32 val;
	int ret;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw,
				 QUERY_ATTR_IDN_MAX_NO_FLUSH_TW_BUF_ALLOC_UNITS,
				 &val);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("read attr max_no_flush_alloc_units %u (0x%X)", val, val);
	return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t ufstw_sysfs_store_max_no_flush_alloc_units(struct ufstw_lu *tw,
							  const char *buf,
							  size_t count)
{
	u32 val;
	ssize_t ret;
	u32 prov_max_buf_size = get_prov_max_buf_size(tw);

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	if (val > prov_max_buf_size)
		return -EINVAL;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_write_lu_attr(tw,
				  QUERY_ATTR_IDN_MAX_NO_FLUSH_TW_BUF_ALLOC_UNITS,
				  &val);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("max_no_flush_alloc_units(%u) write query success", val);

	return count;
}

/* store 0 = decrease, 1 = increase */
static ssize_t ufstw_sysfs_store_resize(struct ufstw_lu *tw, const char *buf,
					size_t count)
{
	u32 val;
	ssize_t ret;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	if (!(val == 0 || val == 1))
		return -EINVAL;

	if (tw->resize_auto) {
		INFO_MSG("NOT ALLOWED: resize_auto: 1 (ON)");
		return -ENODEV;
	}

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_write_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE, &val);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("resize(%u) write query success", val);

	return count;
}

static ssize_t ufstw_sysfs_show_resize_status(struct ufstw_lu *tw, char *buf)
{
	u32 resize_status;
	int ret;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_STATUS,
				 &resize_status);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("resize_auto: %u (%s) resize_op : %d resize_status: %u (%s)",
		 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
		 tw->resize_op,
		 resize_status, RESIZE_STATUS_TO_STR(resize_status));

	return snprintf(buf, PAGE_SIZE, "%u\n", resize_status);
}

static ssize_t ufstw_sysfs_show_resized_alloc_units(struct ufstw_lu *tw,
						    char *buf)
{
	u32 resize_status, resize_not_avail, alloc_units;
	int ret;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	/* Resize available check */
	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_NOT_AVAIL,
				 &resize_not_avail);
	if (ret)
		goto put_sync_out;

	if (resize_not_avail) {
		INFO_MSG("resize_auto: %u (%s) resize_not_avail: %u",
			 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
			 resize_not_avail);
		goto put_sync_out;
	}

	/* Resize status check */
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_STATUS,
				 &resize_status);
	if (ret)
		goto put_sync_out;

	if (resize_status == UFSTW_RESIZE_STATUS_IN_PROGRESS) {
		INFO_MSG("resize_auto: %u (%s) resize_op: %d resize_status: %u (%s)",
			 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
			 tw->resize_op,
			 resize_status, RESIZE_STATUS_TO_STR(resize_status));
		goto put_sync_out;
	}

	/* Read resized buffer size */
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_RESIZED_TW_BUF_ALLOC_UNITS,
				 &alloc_units);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("resize_auto: %u (%s) resize_status: %u (%s) alloc_units %u (0x%X)",
		 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
		 resize_status, RESIZE_STATUS_TO_STR(resize_status),
		 alloc_units, alloc_units);

	return snprintf(buf, PAGE_SIZE, "%u\n", alloc_units);

put_sync_out:
	pm_runtime_put_sync(tw->ufsf->hba->dev);

	return -ENODEV;
}

static ssize_t ufstw_sysfs_show_full_count(struct ufstw_lu *tw, char *buf)
{
	u32 full_count;
	int ret;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_FULL_COUNT,
				 &full_count);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("resize_auto: %u (%s) full_count: %u",
		 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
		 full_count);

	return snprintf(buf, PAGE_SIZE, "%u\n", full_count);
}

static ssize_t ufstw_sysfs_show_resize_not_avail(struct ufstw_lu *tw, char *buf)
{
	u32 resize_not_avail;
	int ret;

	if (!ufstw_check_query_operational(tw))
		return -ENODEV;

	pm_runtime_get_sync(tw->ufsf->hba->dev);
	ret = ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_TW_BUF_RESIZE_NOT_AVAIL,
				 &resize_not_avail);
	pm_runtime_put_sync(tw->ufsf->hba->dev);
	if (ret)
		return -ENODEV;

	INFO_MSG("resize_auto: %u (%s) resize_not_avail: %u",
		 tw->resize_auto, RESIZE_AUTO_TO_STR(tw->resize_auto),
		 resize_not_avail);

	return snprintf(buf, PAGE_SIZE, "%u\n", resize_not_avail);
}

#define ufstw_sysfs_attr_ro(_name) __ATTR(_name, 0444,\
				      ufstw_sysfs_show_##_name, NULL)
#define ufstw_sysfs_attr_wo(_name) __ATTR(_name, 0200,\
					  NULL, ufstw_sysfs_store_##_name)
#define ufstw_sysfs_attr_rw(_name) __ATTR(_name, 0644,\
				      ufstw_sysfs_show_##_name, \
				      ufstw_sysfs_store_##_name)

static struct ufstw_sysfs_entry ufstw_sysfs_entries[] = {
	ufstw_sysfs_attr_rw(write_resize_interval),
	ufstw_sysfs_attr_rw(resize_auto),
#if defined(CONFIG_UFSTW_DEBUG)
	ufstw_sysfs_attr_rw(debug),
#endif
	/* Flag */
	ufstw_sysfs_attr_rw(tw_enable),
	ufstw_sysfs_attr_rw(flush_enable),
	ufstw_sysfs_attr_rw(flush_during_hibern_enter),
	ufstw_sysfs_attr_wo(full_count_init),
	/* Attribute */
	ufstw_sysfs_attr_ro(flush_status),
	ufstw_sysfs_attr_ro(available_buffer_size),
	ufstw_sysfs_attr_ro(lifetime_est),
	ufstw_sysfs_attr_ro(curr_buffer_size),
	ufstw_sysfs_attr_rw(max_no_flush_alloc_units),
	ufstw_sysfs_attr_wo(resize),
	ufstw_sysfs_attr_ro(resize_status),
	ufstw_sysfs_attr_ro(resized_alloc_units),
	ufstw_sysfs_attr_ro(full_count),
	ufstw_sysfs_attr_ro(resize_not_avail),
	__ATTR_NULL
};

static ssize_t ufstw_attr_show(struct kobject *kobj, struct attribute *attr,
			       char *page)
{
	struct ufstw_sysfs_entry *entry;
	struct ufstw_lu *tw;
	ssize_t error;

	entry = container_of(attr, struct ufstw_sysfs_entry, attr);
	if (!entry->show)
		return -EIO;

	tw = container_of(kobj, struct ufstw_lu, kobj);

	mutex_lock(&tw->sysfs_lock);
	error = entry->show(tw, page);
	mutex_unlock(&tw->sysfs_lock);
	return error;
}

static ssize_t ufstw_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *page, size_t length)
{
	struct ufstw_sysfs_entry *entry;
	struct ufstw_lu *tw;
	ssize_t error;

	entry = container_of(attr, struct ufstw_sysfs_entry, attr);
	if (!entry->store)
		return -EIO;

	tw = container_of(kobj, struct ufstw_lu, kobj);

	mutex_lock(&tw->sysfs_lock);
	error = entry->store(tw, page, length);
	mutex_unlock(&tw->sysfs_lock);
	return error;
}

static const struct sysfs_ops ufstw_sysfs_ops = {
	.show = ufstw_attr_show,
	.store = ufstw_attr_store,
};

static struct kobj_type ufstw_ktype = {
	.sysfs_ops = &ufstw_sysfs_ops,
	.release = NULL,
};

static int ufstw_create_sysfs(struct ufsf_feature *ufsf, struct ufstw_lu *tw)
{
	struct device *dev = ufsf->hba->dev;
	struct ufstw_sysfs_entry *entry;
	int err;
	char lun_str[20] = { 0 };

	tw->sysfs_entries = ufstw_sysfs_entries;

	kobject_init(&tw->kobj, &ufstw_ktype);
	mutex_init(&tw->sysfs_lock);

	if (tw->lun == TW_LU_SHARED) {
		snprintf(lun_str, 6, "ufstw");
		INFO_MSG("ufstw creates sysfs ufstw-shared");
	} else {
		snprintf(lun_str, 10, "ufstw_lu%d", tw->lun);
		INFO_MSG("ufstw creates sysfs ufstw_lu%d", tw->lun);
	}

	err = kobject_add(&tw->kobj, kobject_get(&dev->kobj), lun_str);
	if (!err) {
		for (entry = tw->sysfs_entries; entry->attr.name != NULL;
		     entry++) {
			if (tw->lun == TW_LU_SHARED)
				INFO_MSG("ufstw-shared sysfs attr creates: %s",
					 entry->attr.name);
			else
				INFO_MSG("ufstw_lu(%d) sysfs attr creates: %s",
					 tw->lun, entry->attr.name);

			err = sysfs_create_file(&tw->kobj, &entry->attr);
			if (err) {
				ERR_MSG("create entry(%s) failed",
					entry->attr.name);
				goto kobj_del;
			}
		}
		kobject_uevent(&tw->kobj, KOBJ_ADD);
	} else {
		ERR_MSG("kobject_add failed");
	}

	return err;
kobj_del:
	err = kobject_uevent(&tw->kobj, KOBJ_REMOVE);
	INFO_MSG("kobject removed (%d)", err);
	kobject_del(&tw->kobj);
	return -EINVAL;
}

MODULE_LICENSE("GPL v2");
