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

#ifndef _UFSTW_H_
#define _UFSTW_H_

#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/blktrace_api.h>
#include <linux/blkdev.h>
#include <scsi/scsi_cmnd.h>

#include "../../../block/blk.h"

#define UFSTW_VER					0x0203
#define UFSTW_DD_VER					0x020201
#define UFSTW_DD_VER_POST				""

#define UFSTW_LIFETIME_WRITE_SECT_INTERVAL_DEFAULT	2097152  /* 1GB */
#define UFSTW_RESIZE_WRITE_SECT_INTERVAL_DEFAULT	2097152  /* 1GB */
#define UFSTW_RESIZE_WRITE_SECT_INTERVAL_MIN		524288   /* 256KB */
#define UFSTW_RESIZE_WRITE_SECT_INTERVAL_MAX		20971520 /* 10GB */

#define UFSTW_MAX_LIFETIME_VALUE			0x0B
#define MASK_UFSTW_LIFETIME_NOT_GUARANTEE		0x80

#define UFSTW_RESIZE_STATUS_IDLE			0
#define UFSTW_RESIZE_STATUS_IN_PROGRESS			1
#define UFSTW_RESIZE_STATUS_SUCCESS			2
#define UFSTW_RESIZE_STATUS_FAIL			3

#define UFSTW_RESIZE_DECREASE				0x0
#define UFSTW_RESIZE_INCREASE				0x1
#define UFSTW_RESIZE_NONE				0xff

#define UFS_FEATURE_SUPPORT_TW_BIT			0x100

#define TW_LU_SHARED					-1

#define TW_SHARED_BUF(ufsf) (ufsf->tw_dev_info.tw_buf_type == TW_BUF_TYPE_SHARED)

#define FLAG_IDN_NAME(idn)						\
	(idn == QUERY_FLAG_IDN_TW_EN ? "tw_enable" :			\
	 idn == QUERY_FLAG_IDN_TW_BUF_FLUSH_EN ? "flush_enable" :	\
	 idn == QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN ? "flush_hibern" :\
	 idn == QUERY_FLAG_IDN_TW_BUF_FULL_COUNT_INIT ? "full_count_init" :\
	 "unknown")

#define ATTR_IDN_NAME(idn)						\
	(idn == QUERY_ATTR_IDN_TW_FLUSH_STATUS ? "flush_status" :	\
	 idn == QUERY_ATTR_IDN_TW_AVAIL_BUF_SIZE ? "avail_buffer_size" :\
	 idn == QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST ? "lifetime_est" :	\
	 idn == QUERY_ATTR_IDN_TW_CURR_BUF_SIZE ? "current_buf_size" :	\
	 idn == QUERY_ATTR_IDN_MAX_NO_FLUSH_TW_BUF_ALLOC_UNITS ?	\
	 "max_no_flush_alloc_units" :\
	 idn == QUERY_ATTR_IDN_TW_BUF_RESIZE ? "resize" :		\
	 idn == QUERY_ATTR_IDN_TW_BUF_RESIZE_STATUS ? "resize_status" :	\
	 idn == QUERY_ATTR_IDN_RESIZED_TW_BUF_ALLOC_UNITS ? "resized_alloc_units" :\
	 idn == QUERY_ATTR_IDN_TW_BUF_FULL_COUNT ? "full_count" :	\
	 idn == QUERY_ATTR_IDN_TW_BUF_RESIZE_NOT_AVAIL ? "resize_not_avail" :\
	 idn == QUERY_ATTR_IDN_BKOPS_STATUS ? "bkops_status" :\
	 "unknown")

#define RESIZE_AUTO_TO_STR(no) (no ? "ON" : "OFF")

#define RESIZE_STATUS_TO_STR(no)					\
	(no == UFSTW_RESIZE_STATUS_IDLE ? "idle" :			\
	 no == UFSTW_RESIZE_STATUS_IN_PROGRESS ? "in_progress" :	\
	 no == UFSTW_RESIZE_STATUS_SUCCESS ? "completed successfully" :	\
	 no == UFSTW_RESIZE_STATUS_FAIL ? "fail" : "unknown")

/*
 * UFSTW DEBUG
 */

#if defined(CONFIG_UFSTW_DEBUG)
#define TW_DEBUG(tw, msg, args...)			\
	do { if (tw && tw->debug)			\
		printk(KERN_ERR "%s:%d " msg "\n",	\
		       __func__, __LINE__, ##args);	\
	} while (0)
#else
#define TW_DEBUG(hpb, msg, args...)	do { } while (0)
#endif

#define TW_FTRACE(tw, msg, args...)			\
	trace_printk("%04d\t " msg "\n", __LINE__, ##args);

enum UFSTW_STATE {
	TW_NEED_INIT = 0,
	TW_PRESENT = 1,
	TW_SUSPEND = 2,
	TW_RESET = -3,
	TW_FAILED = -4,
};

enum {
	TW_BUF_TYPE_LU = 0,
	TW_BUF_TYPE_SHARED,
};

struct ufstw_dev_info {
	/* from Device Descriptor */
	u16 tw_ver;
	u32 tw_shared_buf_alloc_units;
	u8 tw_buf_no_reduct;
	u8 tw_buf_type;

	/* from Geometry Descriptor */
	u8 tw_number_lu;
};

struct ufstw_lu {
	struct ufsf_feature *ufsf;

	int lun;
	unsigned int lu_buf_size;

	bool tw_enable;
	bool flush_enable;
	bool flush_during_hibern_enter;

	unsigned int flush_status;
	unsigned int available_buffer_size;
	unsigned int curr_buffer_size;

	unsigned int lifetime_est;
	unsigned int resize_op;

	/* write calculation */
	spinlock_t write_calc_lock;
	u32 write_lifetime;
	u32 write_lifetime_interval;
	u32 write_resize;
	u32 write_resize_interval;

	struct work_struct tw_interval_work;
	bool schedule_lifetime;
	bool schedule_resize;

	/* resize_auto */
	bool resize_auto;
	bool resize_fully_increased;
	bool resize_fully_decreased;

	/* for sysfs */
	struct kobject kobj;
	struct mutex sysfs_lock;
	struct ufstw_sysfs_entry *sysfs_entries;

	/* for debug */
	bool force_resize;
	u32 force_resize_op;
	bool debug;
};

struct ufstw_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufstw_lu *tw, char *buf);
	ssize_t (*store)(struct ufstw_lu *tw, const char *buf, size_t count);
};

struct ufshcd_lrb;

int ufstw_get_state(struct ufsf_feature *ufsf);
void ufstw_set_state(struct ufsf_feature *ufsf, int state);
void ufstw_get_dev_info(struct ufsf_feature *ufsf, u8 *desc_buf);
void ufstw_get_geo_info(struct ufsf_feature *ufsf, u8 *geo_buf);
void ufstw_alloc_lu(struct ufsf_feature *ufsf, int lun, u8 *lu_buf);

void ufstw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);
void ufstw_init(struct ufsf_feature *ufsf);
void ufstw_remove(struct ufsf_feature *ufsf);
void ufstw_suspend(struct ufsf_feature *ufsf);
void ufstw_resume(struct ufsf_feature *ufsf, bool is_link_off);
void ufstw_reset_host(struct ufsf_feature *ufsf);
void ufstw_reset(struct ufsf_feature *ufsf);
#endif /* End of Header */
