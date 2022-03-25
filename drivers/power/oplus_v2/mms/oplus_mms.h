// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

#ifndef __OPLUS_MMS_H__
#define __OPLUS_MMS_H__

#include <linux/spinlock.h>
#include <linux/oplus_chg.h>

#define TOPIC_NAME_MAX 128

enum oplus_mms_type {
	OPLUS_MMS_TYPE_UNKNOWN,
	OPLUS_MMS_TYPE_ERROR,
	OPLUS_MMS_TYPE_GAUGE,
	OPLUS_MMS_TYPE_BATTERY,
	OPLUS_MMS_TYPE_USB,
	OPLUS_MMS_TYPE_WLS,
	OPLUS_MMS_TYPE_TEMP,
	OPLUS_MMS_TYPE_VOOC,
	OPLUS_MMS_TYPE_AIRVOOC,
};

enum mms_msg_type {
	MSG_TYPE_TIMER,
	MSG_TYPE_ITEM,
};

enum mms_msg_prio {
	MSG_PRIO_HIGH,
	MSG_PRIO_MEDIUM,
	MSG_PRIO_LOW,
};

struct oplus_mms;

union mms_msg_data {
	int intval;
	char *strval;
};

struct mms_item_desc {
	u32 item_id;
	bool str_data;
	bool up_thr_enable;
	bool down_thr_enable;
	bool dead_thr_enable;
	int update_thr_up;
	int update_thr_down;
	int dead_zone_thr;
	int (*update)(struct oplus_mms *, union mms_msg_data *);
};

struct mms_item {
	struct mms_item_desc desc;
	bool updated;
	rwlock_t lock;
	union mms_msg_data data;
	union mms_msg_data pre_data;
};

struct mms_msg {
	enum mms_msg_type type;
	enum mms_msg_prio prio;
	u32 item_id;
	struct list_head list;
};

struct mms_subscribe {
	char name[TOPIC_NAME_MAX];
	struct oplus_mms *mms;
	void *priv_data;
	struct list_head list;
	void (*callback)(struct mms_subscribe *, enum mms_msg_type, u32);
};

struct oplus_mms_config {
	struct device_node *of_node;
	struct fwnode_handle *fwnode;

	/* Driver private data */
	void *drv_data;

	int update_interval;
	/* Device specific sysfs attributes */
	const struct attribute_group **attr_grp;
};

struct oplus_mms_desc {
	const char *name;
	enum oplus_mms_type type;
	struct mms_item *item_table;
	int item_num;
	const u32 *update_items;
	int update_items_num;
	int update_interval;
	void (*update)(struct oplus_mms *);
};

struct oplus_mms {
	const struct oplus_mms_desc *desc;
	int static_update_interval;
	int normal_update_interval;
	struct list_head subscribe_list;
	struct list_head msg_list;
	spinlock_t subscribe_lock;
	struct mutex msg_lock;
	struct delayed_work update_work;
	struct delayed_work msg_work;

	struct device_node *of_node;
	void *drv_data;
	struct device dev;
	spinlock_t changed_lock;
	bool changed;
	bool initialized;
	bool removing;
	atomic_t use_cnt;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	u32 debug_item_id;
	struct mms_subscribe debug_subs;
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */
};

#define to_oplus_mms(device) container_of(device, struct oplus_mms, dev)

bool oplus_mms_item_is_str(struct oplus_mms *mms, u32 id);
int oplus_mms_get_item_data(struct oplus_mms *mms, u32 item_id,
			    union mms_msg_data *data, bool update);
bool oplus_mms_item_update(struct oplus_mms *mms, u32 item_id, bool check_update);
struct mms_msg *oplus_mms_alloc_msg(enum mms_msg_type type,
				    enum mms_msg_prio prio, u32 item_id);
int oplus_mms_publish_msg(struct oplus_mms *mms, struct mms_msg *msg);
int oplus_mms_subscribe(struct oplus_mms *mms, struct mms_subscribe *subs);
int oplus_mms_unsubscribe(struct mms_subscribe *subs);
int oplus_mms_set_publish_interval(struct oplus_mms *mms, int time_ms);
int oplus_mms_stop_publish(struct oplus_mms *mms);
int oplus_mms_restore_publish(struct oplus_mms *mms);
struct oplus_mms *oplus_mms_get_by_name(const char *name);
void oplus_mms_put(struct oplus_mms *mms);
struct oplus_mms *__must_check oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg);
struct oplus_mms *__must_check
oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg);
struct oplus_mms *__must_check
devm_oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg);
struct oplus_mms *__must_check
devm_oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg);
void oplus_mms_unregister(struct oplus_mms *mms);
void *oplus_mms_get_drvdata(struct oplus_mms *mms);

#endif /* __OPLUS_MMS_H__ */
