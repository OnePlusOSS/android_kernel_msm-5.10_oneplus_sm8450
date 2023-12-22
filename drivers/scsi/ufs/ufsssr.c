/*
 * Universal Flash Storage Secure Smart Report
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Soon Hwang <soon95.hwang@samsung.com>
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
#include "ufsssr.h"

static int ufsssr_issue_vendor_mode_retry(struct ufsf_feature *ufsf, u8 code);
static int ufsssr_create_sysfs(struct ufsssr_dev *ssr);

inline int ufsssr_get_state(struct ufsf_feature *ufsf)
{
	return atomic_read(&ufsf->ssr_state);
}

inline void ufsssr_set_state(struct ufsf_feature *ufsf, int state)
{
	atomic_set(&ufsf->ssr_state, state);
}

static inline int ufsssr_is_not_present(struct ufsssr_dev *ssr)
{
	enum UFSSSR_STATE cur_state = ufsssr_get_state(ssr->ufsf);

	if (cur_state != SSR_PRESENT) {
		INFO_MSG("ssr_state != SSR_PRESENT (%d)", cur_state);
		return -ENODEV;
	}
	return 0;
}

static struct scsi_device *ufsssr_get_sdev(struct ufsf_feature *ufsf,
					int target_lun)
{
	struct scsi_device *sdev = NULL;

	if (target_lun >= 0 && target_lun < UFS_UPIU_MAX_GENERAL_LUN)
		sdev = ufsf->sdev_ufs_lu[target_lun];

	if (!sdev)
		ERR_MSG("get scsi_device fail");
	else
		INFO_MSG("get scsi_device lun (%llu)", sdev->lun);

	return sdev;
}

static inline void ufsssr_set_vendor_mode_cmd(struct ufsf_feature *ufsf,
						  struct ufshcd_lrb *lrbp)
{
	unsigned char *cdb = lrbp->cmd->cmnd;
	u32 entry_2, entry_6, entry_12 = 0;

	entry_2 = ufsf->ssr_dev->input_signature;
	put_unaligned_be32(entry_2, cdb + 2);

	entry_6 = ufsf->ssr_dev->input_parameter;
	put_unaligned_be32(entry_6, cdb + 6);
	put_unaligned_be32(entry_12, cdb + 12);
}

#if defined(CONFIG_UFSSSR_POC)
static inline void ufsssr_set_passwd_cmd(struct ufsf_feature *ufsf,
					     struct ufshcd_lrb *lrbp)
{
	unsigned char *cdb = lrbp->cmd->cmnd;

	put_unaligned_be32(ufsf->ssr_dev->input_parameter, cdb + 2);
}
#endif

static inline void ufsssr_smart_cmd(struct ufsf_feature *ufsf,
						struct ufshcd_lrb *lrbp)
{
	unsigned char *cdb = lrbp->cmd->cmnd;

	cdb[4] = READ_DESC_OP;
	cdb[5] = SSR_DESC_IDN;
	put_unaligned_be32(ufsf->ssr_dev->transfer_bytes, cdb + 12);
}

/*
 * This function is for vendor cmd.
 * cdb[10] -> cdb[16]
 */
void ufsssr_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	unsigned char *cdb = lrbp->cmd->cmnd;
	u32 ssr_vendor_sig;

	if (!ufsf->ssr_dev)
		return;

	if (cdb[0] != VENDOR_CMD_OP)
		return;

	/* To Check if the VENDOR CMD is from UFS Feature Driver Path */
	ssr_vendor_sig = get_unaligned_be32(cdb + 2);
	if (ssr_vendor_sig != SSR_VENDOR_SIG)
		return;

	if (cdb[1] == SMART)
		ufsssr_smart_cmd(ufsf, lrbp);
	else if (cdb[1] == ENTER_VENDOR || cdb[1] == EXIT_VENDOR)
		ufsssr_set_vendor_mode_cmd(ufsf, lrbp);
#if defined(CONFIG_UFSSSR_POC)
	else if (cdb[1] == SET_PASSWD)
		ufsssr_set_passwd_cmd(ufsf, lrbp);
#endif

	lrbp->cmd->cmd_len = MAX_CDB_SIZE;
}

/*
 * scsi_execute() will copy cdb by 10-byte due to opcode.
 * so it will be changed in ufsf_ssr_prep_fn().
 */
static int ufsssr_issue_vendor_mode(struct ufsf_feature *ufsf, u8 code)
{
	struct scsi_sense_hdr sshdr;
	unsigned char cdb[10] = { 0 };
	struct scsi_device *sdev = NULL;
	int ret;

	sdev = ufsssr_get_sdev(ufsf, GET_DEFAULT_LU);
	if (!sdev)
		return -ENODEV;

	if (code != ENTER_VENDOR && code != EXIT_VENDOR) {
		ERR_MSG("vendor command code (%d)", code);
		return -EINVAL;
	}

	cdb[0] = VENDOR_CMD_OP;
	cdb[1] = code;

	/* Signature of SSR Vendor Command */
	put_unaligned_be32(SSR_VENDOR_SIG, cdb + 2);

	ret = scsi_execute(sdev, cdb, DMA_NONE, NULL, 0, NULL, &sshdr,
			   VENDOR_CMD_TIMEOUT, 0, 0, 0, NULL);
	INFO_MSG("vendor(%s) command %s",
		 code == ENTER_VENDOR ? "ENTER" : "EXIT",
		 ret ? "fail" : "success");
	if (ret) {
		ERR_MSG("code %x sense_key %x asc %x ascq %x",
			sshdr.response_code,
			sshdr.sense_key, sshdr.asc, sshdr.ascq);
		ERR_MSG("byte4 %x byte5 %x byte6 %x additional_len %x",
			sshdr.byte4, sshdr.byte5,
			sshdr.byte6, sshdr.additional_length);
	}

	return ret;
}

static int ufsssr_issue_vendor_mode_retry(struct ufsf_feature *ufsf, u8 code)
{
	int ret, retries;

	for (retries = 0; retries < 3; retries++) {
		ret = ufsssr_issue_vendor_mode(ufsf, code);
		if (ret)
			ERR_MSG("vendor_mode[%s] issue fail. retries %d",
				code == ENTER_VENDOR ? "ENTER" : "EXIT",
				retries + 1);
		else
			break;
	}
	return ret;
}

static int ufsssr_issue_smart(struct ufsf_feature *ufsf,
					void *buf, int transfer_bytes)
{
	struct scsi_device *sdev;
	struct scsi_sense_hdr sshdr;
	unsigned char cdb[10] = { 0 };
	int ret = 0, ret_issue = 0, retries;

	sdev = ufsssr_get_sdev(ufsf, GET_DEFAULT_LU);
	if (!sdev)
		return -ENODEV;

	ret = ufsssr_issue_vendor_mode_retry(ufsf, ENTER_VENDOR);
	if (ret) {
		ERR_MSG("enter vendor_mode fail. (%d)", ret);
		ret_issue = -EACCES;
		goto out;
	}

	cdb[0] = VENDOR_CMD_OP;
	cdb[1] = SMART;

	/* To Make Sure This Vendor CMD is from UFS Feature Driver */
	put_unaligned_be32(SSR_VENDOR_SIG, cdb + 2);

	for (retries = 0; retries < 3; retries++) {
		ret = scsi_execute(sdev, cdb, DMA_FROM_DEVICE, buf,
				   transfer_bytes, NULL, &sshdr,
				   VENDOR_CMD_TIMEOUT, 3, 0, 0, NULL);
		if (ret)
			INFO_MSG("SMART COMMAND for SSR fail ret %d retries %d",
				 ret, retries);
		else
			break;
	}

	INFO_MSG("SMART COMMAND for SSR %s", ret ? "fail" : "success");
	if (ret) {
		ERR_MSG("code %x sense_key %x asc %x ascq %x",
			sshdr.response_code,
			sshdr.sense_key, sshdr.asc, sshdr.ascq);
		ERR_MSG("byte4 %x byte5 %x byte6 %x additional_len %x",
			sshdr.byte4, sshdr.byte5,
			sshdr.byte6, sshdr.additional_length);

		ret_issue = ret;
	}
out:
	ret = ufsssr_issue_vendor_mode_retry(ufsf, EXIT_VENDOR);
	if (ret)
		ERR_MSG("exit vendor_mode fail. (%d)", ret);

	return ret_issue ? ret_issue : ret;
}

static inline void ufsssr_remove_sysfs(struct ufsssr_dev *ssr)
{
	int ret;

	ret = kobject_uevent(&ssr->kobj, KOBJ_REMOVE);
	INFO_MSG("kobject removed (%d)", ret);
	kobject_del(&ssr->kobj);
}

void ufsssr_get_dev_info(struct ufsf_feature *ufsf)
{
	struct ufsssr_dev *ssr;

	ufsf->ssr_dev = NULL;

	INFO_MSG("UFS SSR Version (%.4X)", UFSSSR_VER);
	INFO_MSG("SSR D/D version (%.6X%s)", UFSSSR_DD_VER, UFSSSR_DD_VER_POST);

	ufsf->ssr_dev = kzalloc(sizeof(struct ufsssr_dev), GFP_KERNEL);
	ssr = ufsf->ssr_dev;
	if (!ssr) {
		ERR_MSG("ssr_dev was not allocated. so disable ssr.");
		ufsssr_set_state(ufsf, SSR_FAILED);
		return;
	}
	ssr->ufsf = ufsf;
}

void ufsssr_get_geo_info(struct ufsf_feature *ufsf, u8 *geo_buf)
{
	struct ufsssr_dev *ssr = ufsf->ssr_dev;
	u8 dev_cap;

	dev_cap = geo_buf[GEOMETRY_DESC_PARAM_WB_BUFF_CAP_ADJ];

	if (dev_cap == QLC_DEVICE)
		ssr->transfer_bytes = QLC_SSR_DESC_SIZE;
	else
		ssr->transfer_bytes = TLC_SSR_DESC_SIZE;

	ssr->capacity = (get_unaligned_be64(geo_buf + GEOMETRY_DESC_PARAM_DEV_CAP)) >> 3;
	INFO_MSG(" transfer_bytes : %d , capacity : %u ", ssr->transfer_bytes, ssr->capacity);
}

void ufsssr_init(struct ufsf_feature *ufsf)
{
	struct ufsssr_dev *ssr = ufsf->ssr_dev;
	int ret;

	INFO_MSG("SSR_INIT_START");

	ssr->msg_buffer = kzalloc(ssr->transfer_bytes, GFP_KERNEL);
	if (!ssr->msg_buffer) {
		ERR_MSG("ssr msg_buffer allocation fail");
		goto memalloc_fail;
	}

	ssr->parsing = false;
	ssr->input_signature = 0;
	ssr->input_parameter = 0;
#if defined(CONFIG_UFSSSR_POC)
	ssr->input_signature = INPUT_SIG;
	ssr->input_parameter = INPUT_PARAM;
#endif

	ret = ufsssr_create_sysfs(ssr);
	if (ret) {
		ERR_MSG("Creating UFS SSR sysfs files. (%d)", ret);
		goto create_sysfs_fail;
	}

	INFO_MSG("UFS SSR create sysfs & procfs finished");

	ufsssr_set_state(ufsf, SSR_PRESENT);
	return;

create_sysfs_fail:
	kfree(ssr->msg_buffer);

memalloc_fail:
	kfree(ufsf->ssr_dev);
	ufsf->ssr_dev = NULL;
	ufsssr_set_state(ufsf, SSR_FAILED);
}

static int ufsssr_print_desc_to_sysfs(struct secure_smart_desc *desc, char *buf)
{
	int count = 0;

	count += snprintf(buf + count, 38, "%.8X %.2X%.2X%.4X %.8X %.8X\n", be32_to_cpu(desc->len), desc->desc_type, desc->ssr_ver, be16_to_cpu(desc->reserved_1), be32_to_cpu(desc->max_slc_erase_cycle), be32_to_cpu(desc->min_slc_erase_cycle));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->avg_slc_erase_cycle), be32_to_cpu(desc->max_mlc_erase_cycle), be32_to_cpu(desc->min_mlc_erase_cycle), be32_to_cpu(desc->avg_mlc_erase_cycle));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->read_reclaim_cnt), be32_to_cpu(desc->init_bad_blk), be32_to_cpu(desc->runtime_bad_blk), be32_to_cpu(desc->remain_bad_blk));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->reserved_2), be32_to_cpu(desc->req_recovery_lvl), be32_to_cpu(desc->written_data), be32_to_cpu(desc->open_cnt));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->fw_success_cnt), be32_to_cpu(desc->read_data), be32_to_cpu(desc->reserved_3), be32_to_cpu(desc->PON_init_cnt));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->SPOR_init_cnt), be32_to_cpu(desc->SRAM_ECC_cnt), be32_to_cpu(desc->read_fail_bad_blk), be32_to_cpu(desc->prog_fail_bad_blk));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->erase_fail_bad_blk), be32_to_cpu(desc->empty_field[0]), be32_to_cpu(desc->empty_field[1]), be32_to_cpu(desc->empty_field[2]));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->empty_field[3]), be32_to_cpu(desc->empty_field[4]), be32_to_cpu(desc->empty_field[5]), be32_to_cpu(desc->empty_field[6]));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->empty_field[7]), be32_to_cpu(desc->empty_field[8]), be32_to_cpu(desc->empty_field[9]), be32_to_cpu(desc->empty_field[10]));
	count += snprintf(buf + count, 38, "%.8X %.8X %.8X %.8X\n", be32_to_cpu(desc->empty_field[11]), be32_to_cpu(desc->empty_field[12]), be32_to_cpu(desc->empty_field[13]), be32_to_cpu(desc->empty_field[14]));
	count += snprintf(buf + count, 38, "%.8X %.4X%.4X\n", be32_to_cpu(desc->empty_field[15]), be16_to_cpu(desc->fw_exception_lv), be16_to_cpu(desc->fw_exception_subcode));
	return count;
}

static int ufsssr_print_desc_to_sysfs_parsing(struct secure_smart_desc *desc, char *buf)
{
	int count = 0;

	count += snprintf(buf + count, 33, "SSR Descriptor Length : %X\n", be32_to_cpu(desc->len));
	count += snprintf(buf + count, 16, "Desc type : %.2X\n", desc->desc_type);
	count += snprintf(buf + count, 19, "Desc Version : %.2X\n", desc->ssr_ver);
	count += snprintf(buf + count, 22, "SLC Erase Cycle INFO\n");
	count += snprintf(buf + count, 80, "MAX : %d(0x%.8X) Min :%d(0x%.8X) Avg : %d(0x%.8X)\n", be32_to_cpu(desc->max_slc_erase_cycle), be32_to_cpu(desc->max_slc_erase_cycle), be32_to_cpu(desc->min_slc_erase_cycle), be32_to_cpu(desc->min_slc_erase_cycle), be32_to_cpu(desc->avg_slc_erase_cycle), be32_to_cpu(desc->avg_slc_erase_cycle));
	count += snprintf(buf + count, 22, "MLC Erase Cycle INFO\n");
	count += snprintf(buf + count, 80, "MAX : %d(0x%.8X) Min : %d(0x%.8X) Avg : %d(0x%.8X)\n", be32_to_cpu(desc->max_mlc_erase_cycle), be32_to_cpu(desc->max_mlc_erase_cycle), be32_to_cpu(desc->min_mlc_erase_cycle), be32_to_cpu(desc->min_mlc_erase_cycle), be32_to_cpu(desc->avg_mlc_erase_cycle), be32_to_cpu(desc->avg_mlc_erase_cycle));
	count += snprintf(buf + count, 50, "Read Reclaim Count INFO : %d(0x%.8X)\n", be32_to_cpu(desc->read_reclaim_cnt), be32_to_cpu(desc->read_reclaim_cnt));
	count += snprintf(buf + count, 22, "Bad Block Count INFO\n");
	count += snprintf(buf + count, 80, "Initial : %d(0x%.8X) Runtime : %d(%.8X) Remain : %d(%.8X)\n", be32_to_cpu(desc->init_bad_blk), be32_to_cpu(desc->init_bad_blk), be32_to_cpu(desc->runtime_bad_blk),  be32_to_cpu(desc->runtime_bad_blk), be32_to_cpu(desc->remain_bad_blk), be32_to_cpu(desc->remain_bad_blk));
	count += snprintf(buf + count, 50, "Required Recovery Level : %d(0x%.8X)\n", be32_to_cpu(desc->req_recovery_lvl), be32_to_cpu(desc->req_recovery_lvl));
	count += snprintf(buf + count, 70, "Written Data (Unit : 10MB) : %d(0x%.8X)\n", be32_to_cpu(desc->written_data), be32_to_cpu(desc->written_data));
	count += snprintf(buf + count, 32, "Open Count : %d(0x%.8X)\n", be32_to_cpu(desc->open_cnt), be32_to_cpu(desc->open_cnt));
	count += snprintf(buf + count, 36, "FW Success Count : %d(0x%.8X)\n", be32_to_cpu(desc->fw_success_cnt), be32_to_cpu(desc->fw_success_cnt));
	count += snprintf(buf + count, 32, "Read Data : %d(0x%.8X)\n", be32_to_cpu(desc->read_data), be32_to_cpu(desc->read_data));
	count += snprintf(buf + count, 17, "Init Count INFO\n");
	count += snprintf(buf + count, 60, "PON : %d(0x%.8X) SPOR : %d(0x%.8X)\n", be32_to_cpu(desc->PON_init_cnt), be32_to_cpu(desc->PON_init_cnt), be32_to_cpu(desc->SPOR_init_cnt), be32_to_cpu(desc->SPOR_init_cnt));
	count += snprintf(buf + count, 50, "SRAM ECC Count : %d(0x%.8X)\n", be32_to_cpu(desc->SRAM_ECC_cnt), be32_to_cpu(desc->SRAM_ECC_cnt));
	count += snprintf(buf + count, 24, "Bad Block By Fail INFO\n");
	count += snprintf(buf + count, 80, "Read : %d(0x%.8X) Program : %d(0x%.8X) Erase : %d(0x%.8X)\n", be32_to_cpu(desc->read_fail_bad_blk), be32_to_cpu(desc->read_fail_bad_blk), be32_to_cpu(desc->prog_fail_bad_blk), be32_to_cpu(desc->prog_fail_bad_blk), be32_to_cpu(desc->erase_fail_bad_blk), be32_to_cpu(desc->erase_fail_bad_blk));
	count += snprintf(buf + count, 19, "FW Exception INFO\n");
	count += snprintf(buf + count, 80, "Level : %d(0x%.4X) Subcode : %d(0x%.4X)\n", be16_to_cpu(desc->fw_exception_lv), be16_to_cpu(desc->fw_exception_lv), be16_to_cpu(desc->fw_exception_subcode), be16_to_cpu(desc->fw_exception_subcode));
	return count;
}

static int ufsssr_print_to_sysfs(struct ufsf_feature *ufsf, char *buf)
{
	struct ufsssr_dev *ssr = ufsf->ssr_dev;
	struct secure_smart_desc *desc;

	int ret;
	void *desc_buf;
	void *p;
	int len;

	ssr = ufsf->ssr_dev;

	desc_buf = ssr->msg_buffer;

	ret = ufsssr_issue_smart(ufsf, desc_buf, ssr->transfer_bytes);
	if (ret) {
		ERR_MSG("issue SMART Command for SSR fail. (%d)", ret);
		return 0;
	}
	p = desc_buf;
	desc = (struct secure_smart_desc *)p;

	if (!ssr->parsing)
		len = ufsssr_print_desc_to_sysfs(desc, buf);
	else
		len = ufsssr_print_desc_to_sysfs_parsing(desc, buf);

	return len;
}

void ufsssr_remove(struct ufsf_feature *ufsf)
{
	struct ufsssr_dev *ssr = ufsf->ssr_dev;

	if (!ssr)
		return;

	INFO_MSG("start SSR release");
	ufsssr_set_state(ufsf, SSR_FAILED);

	ufsssr_remove_sysfs(ssr);

	kfree(ssr->msg_buffer);
	kfree(ssr);
	ufsf->ssr_dev = NULL;

	INFO_MSG("end SSR release");
}

/***********************************************************************
 * There are functions for SYSFS in below.
 **********************************************************************/

static int ufsssr_check_hex_input(const char *buf, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++) {
		if (!((buf[i] >= '0' && buf[i] <= '9') ||
		    (buf[i] >= 'A' && buf[i] <= 'F') ||
		    (buf[i] >= 'a' && buf[i] <= 'f')))
			return -EINVAL;
	}

	return 0;
}

#define ufsssr_sysfs_show_func(_name)				\
static ssize_t ufsssr_sysfs_show_##_name(struct ufsssr_dev *ssr,\
					     char *buf)			\
{									\
	INFO_MSG("read "#_name" %u (0x%X)", ssr->_name, ssr->_name);\
									\
	return snprintf(buf, PAGE_SIZE, "%u\n", ssr->_name);	\
}

#define ufsssr_sysfs_store_func(_name)				\
static ssize_t ufsssr_sysfs_store_##_name(struct ufsssr_dev *ssr,\
					      const char *buf,		\
					      size_t count)		\
{									\
	unsigned long val;						\
									\
	if (kstrtoul(buf, 0, &val))					\
		return -EINVAL;						\
									\
	if (!(val == 0  || val == 1))					\
		return -EINVAL;						\
									\
	ssr->_name = val;						\
									\
	INFO_MSG(#_name " success = %d", ssr->_name);		\
	return count;							\
}

#define ufsssr_sysfs_store_func_vendor(_name)			\
static ssize_t ufsssr_sysfs_store_##_name(struct ufsssr_dev *ssr,\
					      const char *buf,		\
					      size_t count)		\
{									\
	int ret, size;							\
									\
	size = strlen(buf);						\
									\
	if (size != count || size - 1 != VENDOR_INPUT_LEN) {		\
		ERR_MSG("buf size(%d) is not match to count(%ld)",	\
			size, count);					\
		ERR_MSG("(vendor input size is (%d))",			\
			VENDOR_INPUT_LEN);				\
		return -EINVAL;						\
	}								\
	ret = ufsssr_check_hex_input(buf, count - 1);		\
	if (ret) {							\
		ERR_MSG("input is not hex value. input (%s)", buf);	\
		return ret;						\
	}								\
	ret = kstrtouint(buf, 16, &ssr->_name);			\
	if (ret) {							\
		ERR_MSG("input is not valid. (%d)", ret);		\
		return ret;						\
	}								\
	INFO_MSG(#_name" (0x%08X)", ssr->_name);			\
	return count;							\
}

ufsssr_sysfs_show_func(parsing);
ufsssr_sysfs_store_func(parsing);
ufsssr_sysfs_store_func_vendor(input_signature);
ufsssr_sysfs_store_func_vendor(input_parameter);

static ssize_t ufsssr_sysfs_show_print_ssr(struct ufsssr_dev *ssr, char *buf)
{
	struct ufsf_feature *ufsf = ssr->ufsf;
	struct ufs_hba *hba = ufsf->hba;
	int ret;

	if (ufsssr_is_not_present(ssr))
		return -ENODEV;

	pm_runtime_get_sync(hba->dev);
	ret = ufsssr_print_to_sysfs(ufsf, buf);
	if (!ret) {
		ERR_MSG("Print SSR Descriptor Failed.");
		return -EINVAL;
	}
	pm_runtime_put_sync(hba->dev);

	return ret;
}

static ssize_t ufsssr_sysfs_show_fill_ratio(struct ufsssr_dev *ssr, char *buf)
{
	struct ufsf_feature *ufsf = ssr->ufsf;
	struct ufs_hba *hba = ufsf->hba;
	struct secure_smart_desc *desc;
	int ret;
	u64 written_LBA;
	u64 div_remainder;

	if (ufsssr_is_not_present(ssr))
		return -ENODEV;

	/* Fill_Ratio is only shown when UFS is in QLC Mode */
	if (ssr->transfer_bytes != QLC_SSR_DESC_SIZE)
		return -EINVAL;

	pm_runtime_get_sync(hba->dev);

	ret = ufsssr_issue_smart(ufsf, ssr->msg_buffer, ssr->transfer_bytes);
	if (ret) {
		ERR_MSG("issue SMART Command for SSR fail. (%d)", ret);
		pm_runtime_put_sync(hba->dev);
		return -EINVAL;
	}
	pm_runtime_put_sync(hba->dev);
	desc = (struct secure_smart_desc *)ssr->msg_buffer;
	/*
	 *   Casting Capacity to u32, written_bytes to u64 for do_div Function.
	 *  do_div Function is used for division in Kernel.
	 *  do_div Function's First Parameter is dividend(64-bit).
	 *  do_div Function's Second Parameter is divisor(32-bit).
	 *  Dividend is replaced to Quiotent(64-bit) and Function returns Remainder(32-bit).
	 */
	written_LBA = be32_to_cpu(desc->nHostFill) * 100;
	div_remainder = do_div(written_LBA, ssr->capacity);
	div_remainder *= 100;
	do_div(div_remainder, ssr->capacity);

	return snprintf(buf, PAGE_SIZE, "%llu.%.2llu\n", written_LBA, div_remainder);
}
#if defined(CONFIG_UFSSSR_POC)
/*
 * For POC.
 */
static ssize_t ufsssr_sysfs_store_debug_set_pwd(struct ufsssr_dev *ssr,
				     const char *buf, size_t count)
{
	struct ufsf_feature *ufsf = ssr->ufsf;
	struct scsi_sense_hdr sshdr;
	unsigned char cdb[10] = { 0 };
	struct scsi_device *sdev = NULL;
	int ret = 0;

	pm_runtime_get_sync(ufsf->hba->dev);

	sdev = ufsssr_get_sdev(ufsf, GET_DEFAULT_LU);
	if (!sdev) {
		ret = -ENODEV;
		goto out;
	}

	cdb[0] = VENDOR_CMD_OP;
	cdb[1] = SET_PASSWD;

	/* Signature of SSR Vendor Command */
	put_unaligned_be32(SSR_VENDOR_SIG, cdb + 2);

	ret = scsi_execute(sdev, cdb, DMA_NONE, NULL, 0, NULL, &sshdr,
			   VENDOR_CMD_TIMEOUT, 0, 0, 0, NULL);

	INFO_MSG("vendor(SET_PWD) command %s", ret ? "fail" : "successful");
	if (ret) {
		ERR_MSG("code %x sense_key %x asc %x ascq %x",
			sshdr.response_code,
			sshdr.sense_key, sshdr.asc, sshdr.ascq);
		ERR_MSG("byte4 %x byte5 %x byte6 %x additional_len %x",
			sshdr.byte4, sshdr.byte5,
			sshdr.byte6, sshdr.additional_length);
	}
out:
	pm_runtime_put_sync(ufsf->hba->dev);
	return ret ? ret : count;
}
#endif

/* SYSFS DEFINE */
#define define_sysfs_rw(_name) __ATTR(_name, 0644,		\
				      ufsssr_sysfs_show_##_name,	\
				      ufsssr_sysfs_store_##_name)
#define define_sysfs_ro(_name) __ATTR(_name, 0444,			\
				      ufsssr_sysfs_show_##_name, NULL)
#define define_sysfs_wo(_name) __ATTR(_name, 0220, NULL,		\
				      ufsssr_sysfs_store_##_name)

static struct ufsssr_sysfs_entry ufsssr_sysfs_entries[] = {
	define_sysfs_rw(parsing),

	define_sysfs_wo(input_signature),
	define_sysfs_wo(input_parameter),

	define_sysfs_ro(print_ssr),
	define_sysfs_ro(fill_ratio),
#if defined(CONFIG_UFSSSR_POC)
	/*
	 * This function will be removed in released version.
	 * It exists only for POC purpose.
	 */
	define_sysfs_wo(debug_set_pwd),
#endif
	__ATTR_NULL
};

static ssize_t ufsssr_attr_show(struct kobject *kobj,
				    struct attribute *attr, char *page)
{
	struct ufsssr_sysfs_entry *entry;
	struct ufsssr_dev *ssr;
	ssize_t error;

	entry = container_of(attr, struct ufsssr_sysfs_entry, attr);
	if (!entry->show)
		return -EIO;

	ssr = container_of(kobj, struct ufsssr_dev, kobj);
	if (ufsssr_is_not_present(ssr))
		return -ENODEV;

	mutex_lock(&ssr->sysfs_lock);
	error = entry->show(ssr, page);
	mutex_unlock(&ssr->sysfs_lock);

	return error;
}

static ssize_t ufsssr_attr_store(struct kobject *kobj,
				     struct attribute *attr, const char *page,
				     size_t length)
{
	struct ufsssr_sysfs_entry *entry;
	struct ufsssr_dev *ssr;
	ssize_t error;

	entry = container_of(attr, struct ufsssr_sysfs_entry, attr);
	if (!entry->store)
		return -EIO;

	ssr = container_of(kobj, struct ufsssr_dev, kobj);
	if (ufsssr_is_not_present(ssr))
		return -ENODEV;

	mutex_lock(&ssr->sysfs_lock);
	error = entry->store(ssr, page, length);
	mutex_unlock(&ssr->sysfs_lock);

	return error;
}

static const struct sysfs_ops ufsssr_sysfs_ops = {
	.show = ufsssr_attr_show,
	.store = ufsssr_attr_store,
};

static struct kobj_type ufsssr_ktype = {
	.sysfs_ops = &ufsssr_sysfs_ops,
	.release = NULL,
};

static int ufsssr_create_sysfs(struct ufsssr_dev *ssr)
{
	struct device *dev = ssr->ufsf->hba->dev;
	struct ufsssr_sysfs_entry *entry;
	int err;

	ssr->sysfs_entries = ufsssr_sysfs_entries;

	kobject_init(&ssr->kobj, &ufsssr_ktype);
	mutex_init(&ssr->sysfs_lock);

	INFO_MSG("ufsssr creates sysfs ufsssr %p dev->kobj %p",
		 &ssr->kobj, &dev->kobj);

	err = kobject_add(&ssr->kobj, kobject_get(&dev->kobj),
			  "ufsssr");
	if (!err) {
		for (entry = ssr->sysfs_entries; entry->attr.name != NULL;
		     entry++) {
			INFO_MSG("ufsssr sysfs attr creates: %s",
				 entry->attr.name);
			err = sysfs_create_file(&ssr->kobj, &entry->attr);
			if (err) {
				ERR_MSG("create entry(%s) failed",
					entry->attr.name);
				goto kobj_del;
			}
		}
		kobject_uevent(&ssr->kobj, KOBJ_ADD);
	} else {
		ERR_MSG("kobject_add failed");
	}

	return err;
kobj_del:
	err = kobject_uevent(&ssr->kobj, KOBJ_REMOVE);
	INFO_MSG("kobject removed (%d)", err);
	kobject_del(&ssr->kobj);
	return -EINVAL;
}
MODULE_LICENSE("GPL v2");
