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

#ifndef _UFSSSR_H_
#define _UFSSSR_H_

#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <linux/seq_file.h>
#include <asm/unaligned.h>
#include <scsi/scsi_common.h>
#include <linux/kernel.h>
#include <linux/random.h>
#include <linux/time.h>

/* for UFS D/D Version and UFS Version */
#define UFSSSR_VER		0x0101
#define UFSSSR_DD_VER		0x010001
#define UFSSSR_DD_VER_POST	""

/* for buffer */
#define TLC_SSR_DESC_SIZE	0x7C
#define QLC_SSR_DESC_SIZE	0x13B

/* for Classifying QLC / TLC */
#define QLC_DEVICE 0x04

/* for vendor cmd */
#define GET_DEFAULT_LU		0
#define VENDOR_CMD_TIMEOUT	(10 * HZ)
#define VENDOR_CMD_OP		0xC0
#define VENDOR_CDB_LENGTH	16
#define ENTER_VENDOR		0x00
#define EXIT_VENDOR		0x01
#define SET_PASSWD		0x03
#define SMART			0x40
#define READ_DESC_OP	0x1
#define SSR_DESC_IDN	0xA
#define VENDOR_INPUT_LEN	8
#define MAX_CDB_SIZE		16	/* for compatibility */

#define SSR_VENDOR_SIG	0xABCDDCBA

/* for POC*/
#if defined(CONFIG_UFSSSR_POC)
#define INPUT_SIG 0x9E4829C1
#define INPUT_PARAM 0X00000000
#endif

#define SSR_MSG(file, msg, args...)					\
	do {	if (file)						\
			seq_printf(file, msg "\n", ##args);		\
		else							\
			pr_err(msg "\n", ##args);			\
	} while (0)							\

enum UFSSSR_STATE {
	SSR_NEED_INIT = 0,
	SSR_PRESENT = 1,
	SSR_FAILED = -2,
};

struct secure_smart_desc {
	__be32 len;							/* 0..3 */
	__u8 desc_type;						/* 4 */
	__u8 ssr_ver;						/* 5 */
	__u16 reserved_1;					/* 6.7 */
	__be32 max_slc_erase_cycle;			/* 8..B */
	__be32 min_slc_erase_cycle;			/* C..F */
	__be32 avg_slc_erase_cycle;			/* 10..13 */
	__be32 max_mlc_erase_cycle;			/* 14..17 */
	__be32 min_mlc_erase_cycle;			/* 18..1B */
	__be32 avg_mlc_erase_cycle;			/* 1C..1F */
	__be32 read_reclaim_cnt;			/* 20..23 */
	__be32 init_bad_blk;				/* 24..27 */
	__be32 runtime_bad_blk;				/* 28..2B */
	__be32 remain_bad_blk;				/* 2C..2F */
	__be32 reserved_2;					/* 30..33 */
	__be32 req_recovery_lvl;			/* 34..37 */
	__be32 written_data;				/* 38..3B */
	__be32 open_cnt;					/* 3C..3F */
	__be32 fw_success_cnt;				/* 40..43 */
	__be32 read_data;					/* 44..47 */
	__be32 reserved_3;					/* 48..4B */
	__be32 PON_init_cnt;				/* 4C..4F */
	__be32 SPOR_init_cnt;				/* 50..53 */
	__be32 SRAM_ECC_cnt;				/* 54..57 */
	__be32 read_fail_bad_blk;			/* 58..5B */
	__be32 prog_fail_bad_blk;			/* 5C..5F */
	__be32 erase_fail_bad_blk;			/* 60..63 */
	__be32 empty_field[16];				/* 64..A3 */
	__be16 fw_exception_lv;			/* A4..A5 */
	__be16 fw_exception_subcode;		/* A6..A7 */
	__be32 nEfuseEcid0;					/* A8..AB */
	__be32 nEfuseEcid1;					/* AC..AF */
	__be32 reserved_4[3];				/* B0..BB */
	__be32 nDtt_EnterTotal_cnt;			/* BC..BF */
	__be32 nDtt_NormalEnter_cnt;		/* C0..C3 */
	__be32 nDtt_LightEnter_cnt;			/* C4..C7 */
	__be32 nDtt_HeavyEnter_cnt;			/* C8..CB */
	__be32 nDtt_CriticalEnter_cnt;		/* CC..CF */
	__be32 ctrl_revision_info;			/* D0..D3 */
	__be32 fw_exception_bitmap0;		/* D4..D7 */
	__be32 fw_exception_bitmap32;		/* D8..DB */
	__be32 fw_exception_bitmap64;		/* DC..DF */
	__be32 fw_exception_bitmap96;		/* E0..E3 */
	__be32 nHostFill;					/* E4..E7 */
	__be32 GC_fields[21];				/* E8..13B */
} __packed;

struct ufsssr_dev {
	struct ufsf_feature *ufsf;

	int transfer_bytes;	/* Length to Read through CDB */

	bool parsing;		/* default false */
	u32 input_parameter;	/* vendor specific */
	u32 input_signature;	/* vendor specific */

	void *msg_buffer;	/* Buffer for Reading SSR Descriptor */
	u32 capacity;		/* Device Raw Capcity */
	/* for sysfs & procfs */
	struct kobject kobj;
	struct mutex sysfs_lock;
	struct ufsssr_sysfs_entry *sysfs_entries;
};

struct ufsssr_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufsssr_dev *ssr, char *buf);
	ssize_t (*store)(struct ufsssr_dev *ssr, const char *buf, size_t count);
};

struct ufshcd_lrb;

int ufsssr_get_state(struct ufsf_feature *ufsf);
void ufsssr_set_state(struct ufsf_feature *ufsf, int state);
void ufsssr_get_dev_info(struct ufsf_feature *ufsf);
void ufsssr_get_geo_info(struct ufsf_feature *ufsf, u8 *geo_buf);
void ufsssr_init(struct ufsf_feature *ufsf);
void ufsssr_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);
void ufsssr_remove(struct ufsf_feature *ufsf);
#endif
