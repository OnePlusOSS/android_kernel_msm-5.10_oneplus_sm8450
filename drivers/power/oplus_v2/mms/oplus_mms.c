// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

/* OPLUS Micro Message Service */

#define pr_fmt(fmt) "[MMS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include "oplus_mms.h"

struct class *oplus_mms_class;

static struct device_type oplus_mms_dev_type;
static struct workqueue_struct	*mms_wq;

static struct mms_item *oplus_mms_get_item(struct oplus_mms *mms, u32 id)
{
	struct mms_item *item_table = mms->desc->item_table;
	int item_num = mms->desc->item_num;
	int i;

	for (i = 0; i < item_num; i++) {
		if (item_table[i].desc.item_id == id)
			return &item_table[i];
	}

	return NULL;
}

bool oplus_mms_item_is_str(struct oplus_mms *mms, u32 id)
{
	struct mms_item *item;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return false;
	}
	item = oplus_mms_get_item(mms, id);
	if (mms == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name);
		return false;
	}

	return item->desc.str_data;
}

int oplus_mms_get_item_data(struct oplus_mms *mms, u32 item_id,
			    union mms_msg_data *data, bool update)
{
	struct mms_item *item;

	if (mms == NULL || !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return -ENODEV;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}

	if (update)
		oplus_mms_item_update(mms, item_id, false);

	read_lock(&item->lock);
	memcpy(data, &item->data, sizeof(union mms_msg_data));
	read_unlock(&item->lock);

	return 0;
}

bool oplus_mms_item_update(struct oplus_mms *mms, u32 item_id, bool check_update)
{
	struct mms_item *item;
	union mms_msg_data data;

	if (mms == NULL || !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return false;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}
	if (item->desc.update == NULL) {
		chg_err("item update func is NULL");
		return false;
	}

	item->desc.update(mms, &data);
	write_lock(&item->lock);
	memcpy(&item->data, &data, sizeof(union mms_msg_data));
	if (!check_update) {
		item->updated = false;
		goto out;
	} else {
		item->updated = true;
	}

	if (item->desc.str_data) {
		/* Dynamic string data cannot be judged whether to update */
		/*
		if (!strcmp(item->data.strval, item->pre_data.strval))
			goto out;
		*/
		item->pre_data.strval = item->data.strval;
		item->updated = true;
		goto out;
	} else {
		if (item->data.intval == item->pre_data.intval) {
			item->updated = false;
			goto out;
		}
		if (item->desc.up_thr_enable) {
			if ((item->data.intval > item->pre_data.intval) &&
			    (item->pre_data.intval < item->desc.update_thr_up) &&
			    (item->data.intval >= item->desc.update_thr_up)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.update_thr_down) {
			if ((item->data.intval < item->pre_data.intval) &&
			    (item->pre_data.intval > item->desc.update_thr_down) &&
			    (item->data.intval < item->desc.update_thr_down)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.dead_thr_enable) {
			if (abs(item->data.intval - item->pre_data.intval) >=
			    item->desc.dead_zone_thr) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
	}

out:
	write_unlock(&item->lock);
	return item->updated;
}

struct mms_msg *oplus_mms_alloc_msg(enum mms_msg_type type,
				    enum mms_msg_prio prio, u32 item_id)
{
	struct mms_msg *msg;

	msg = kzalloc(sizeof(struct mms_msg), GFP_KERNEL);
	if (msg == NULL)
		return NULL;

	msg->type = type;
	msg->prio = prio;
	msg->item_id = item_id;

	return msg;
}

int oplus_mms_publish_msg(struct oplus_mms *mms, struct mms_msg *msg)
{
	struct mms_msg *temp_msg = NULL;
	struct list_head *temp_list = NULL;
	bool free_msg = false;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (msg == NULL) {
		chg_err("msg is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&mms->msg_lock);
	if (list_empty(&mms->msg_list)) {
		list_add_tail_rcu(&msg->list, &mms->msg_list);
	} else {
		list_for_each_entry_rcu(temp_msg, &mms->msg_list, list) {
			if ((temp_msg->prio == msg->prio) &&
			    (temp_msg->type == msg->type) &&
			    (temp_msg->item_id == msg->item_id)) {
				free_msg = true;
				break;
			}
			if (temp_msg->prio > msg->prio) {
				temp_list = &temp_msg->list;
				break;
			}
		}
		if (!free_msg) {
			list_add_tail_rcu(&msg->list, !temp_list ? &mms->msg_list : temp_list);
		}
	}
	synchronize_rcu();
	mutex_unlock(&mms->msg_lock);
	if (free_msg)
		kfree(msg);

	/* All messages are processed before the device is allowed to sleep */
	spin_lock(&mms->changed_lock);
	if (!mms->changed) {
		mms->changed = true;
		pm_stay_awake(&mms->dev);
	}
	spin_unlock(&mms->changed_lock);

	queue_delayed_work(mms_wq, &mms->msg_work, 0);

	return 0;
}

int oplus_mms_subscribe(struct oplus_mms *mms, struct mms_subscribe *subs)
{
	struct mms_subscribe *subs_temp;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (subs == NULL) {
		chg_err("subs is NULL\n");
		return -EINVAL;
	}

	spin_lock(&mms->subscribe_lock);
	list_for_each_entry_rcu(subs_temp, &mms->subscribe_list, list) {
		if (!strcmp(subs->name, subs_temp->name)) {
			spin_unlock(&mms->subscribe_lock);
			return 0;
		}
	}
	list_add_tail_rcu(&subs->list, &mms->subscribe_list);
	subs->mms = mms;
	spin_unlock(&mms->subscribe_lock);
	atomic_inc(&mms->use_cnt);

	return 0;
}

int oplus_mms_unsubscribe(struct mms_subscribe *subs)
{
	struct oplus_mms *mms;

	if (subs == NULL) {
		chg_err("subs is NULL");
		return -ENODEV;
	}
	mms = subs->mms;

	spin_lock(&mms->subscribe_lock);
	list_del_rcu(&subs->list);
	subs->priv_data = NULL;
	subs->mms = NULL;
	spin_unlock(&mms->subscribe_lock);
	atomic_dec(&mms->use_cnt);
	synchronize_rcu();

	return 0;
}

int oplus_mms_set_publish_interval(struct oplus_mms *mms, int time_ms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = time_ms;

	return 0;
}

int oplus_mms_stop_publish(struct oplus_mms *mms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = 0;
	cancel_delayed_work(&mms->update_work);

	return 0;
}

int oplus_mms_restore_publish(struct oplus_mms *mms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = mms->static_update_interval;
	queue_delayed_work(mms_wq, &mms->update_work,
			   msecs_to_jiffies(mms->normal_update_interval));

	return 0;
}

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
static ssize_t item_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mms->debug_item_id);
}

static ssize_t item_id_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	int val;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	mms->debug_item_id = val;

	return count;
}
static DEVICE_ATTR_RW(item_id);

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	union mms_msg_data data = { 0 };
	ssize_t rc;

	rc = oplus_mms_get_item_data(mms, mms->debug_item_id, &data, false);
	if (rc < 0)
		return rc;

	if (oplus_mms_item_is_str(mms, mms->debug_item_id))
		rc = sprintf(buf, "%s\n", data.strval);
	else
		rc = sprintf(buf, "%d\n", data.intval);

	return rc;
}
static DEVICE_ATTR_RO(data);

static struct device_attribute *oplus_mms_attributes[] = {
	&dev_attr_item_id,
	&dev_attr_data,
	NULL
};

static void debug_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id)
{
	struct oplus_mms *mms = subs->priv_data;
	char *env_buf;
	char *env[2] = { 0 };
	int length;
	int i;

	env_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!env_buf)
		return;
	length = snprintf(env_buf, PAGE_SIZE, "CHANGED_ITEM=");
	switch (type) {
		case MSG_TYPE_TIMER:
			for (i = 0; i < mms->desc->update_items_num; i++)
				length += snprintf(env_buf + length,
						   PAGE_SIZE - length, "%d,",
						   mms->desc->update_items[i]);
			env_buf[length - 1] = 0;
			break;
		case MSG_TYPE_ITEM:
			snprintf(env_buf + length, PAGE_SIZE - length, "%d", id);
			break;
	}
	env[0] = env_buf;
	kobject_uevent_env(&mms->dev.kobj, KOBJ_CHANGE, env);

	free_page((unsigned long)env_buf);
}
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

static void oplus_mms_update_work(struct work_struct *work)
{
	struct oplus_mms *mms = container_of(work, struct oplus_mms,
					update_work.work);

	mms->desc->update(mms);

	if (mms->normal_update_interval > 0)
		queue_delayed_work(mms_wq, &mms->update_work,
				   msecs_to_jiffies(mms->normal_update_interval));
}

static void oplus_mms_msg_work(struct work_struct *work)
{
	struct oplus_mms *mms = container_of(work, struct oplus_mms,
					msg_work.work);
	struct mms_subscribe *subs;
	struct mms_msg *msg;

	rcu_read_lock();
	if (list_empty(&mms->msg_list)) {
		rcu_read_unlock();
		spin_lock(&mms->changed_lock);
		if (likely(mms->changed)) {
			mms->changed = false;
			pm_relax(&mms->dev);
		}
		spin_unlock(&mms->changed_lock);
		return;
	}
	msg = list_entry_rcu(mms->msg_list.next, struct mms_msg, list);
	list_for_each_entry_rcu(subs, &mms->subscribe_list, list) {
		if (subs->callback)
			subs->callback(subs, msg->type, msg->item_id);
	}
	rcu_read_unlock();

	mutex_lock(&mms->msg_lock);
	list_del_rcu(&msg->list);
	mutex_unlock(&mms->msg_lock);
	synchronize_rcu();
	kfree(msg);

	queue_delayed_work(mms_wq, &mms->msg_work, 0);
}

static int oplus_mms_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct oplus_mms *mms = dev_get_drvdata(dev);

	return strcmp(mms->desc->name, name) == 0;
}

struct oplus_mms *oplus_mms_get_by_name(const char *name)
{
	struct oplus_mms *mms = NULL;
	struct device *dev = class_find_device(oplus_mms_class, NULL, name,
					oplus_mms_match_device_by_name);

	if (dev) {
		mms = dev_get_drvdata(dev);
		atomic_inc(&mms->use_cnt);
	}

	return mms;
}

void oplus_mms_put(struct oplus_mms *mms)
{
	might_sleep();

	atomic_dec(&mms->use_cnt);
	put_device(&mms->dev);
}

static void oplus_mms_dev_release(struct device *dev)
{
	struct oplus_mms *mms = to_oplus_mms(dev);
	dev_dbg(dev, "%s\n", __func__);
	kfree(mms);
}

static struct oplus_mms *__must_check
__oplus_mms_register(struct device *parent, const struct oplus_mms_desc *desc,
		     const struct oplus_mms_config *cfg, bool ws)
{
	struct device *dev;
	struct oplus_mms *mms;
	int rc;
	int i;
#ifdef CONFIG_OPLUS_CHG_IC_DEBUG
	struct device_attribute **attrs;
	struct device_attribute *attr;
#endif

	if (!parent)
		chg_info("Expected proper parent device for '%s'\n", desc->name);

	if (!desc || !desc->name || !desc->item_table || !desc->item_num)
		return ERR_PTR(-EINVAL);

	if (desc->update_interval && !desc->update)
		return ERR_PTR(-EINVAL);

	mms = kzalloc(sizeof(struct oplus_mms), GFP_KERNEL);
	if (!mms)
		return ERR_PTR(-ENOMEM);

	dev = &mms->dev;

	device_initialize(dev);

	dev->class = oplus_mms_class;
	dev->type = &oplus_mms_dev_type;
	dev->parent = parent;
	dev->release = oplus_mms_dev_release;
	dev_set_drvdata(dev, mms);
	mms->desc = desc;
	if (cfg) {
		dev->groups = cfg->attr_grp;
		mms->drv_data = cfg->drv_data;
		mms->of_node =
			cfg->fwnode ? to_of_node(cfg->fwnode) : cfg->of_node;
		if (cfg->update_interval)
			mms->static_update_interval = cfg->update_interval;
	}
	if (mms->static_update_interval == 0)
		mms->static_update_interval = desc->update_interval;
	mms->normal_update_interval = mms->static_update_interval;

	rc = dev_set_name(dev, "%s", desc->name);
	if (rc)
		goto dev_set_name_failed;

	spin_lock_init(&mms->subscribe_lock);
	mutex_init(&mms->msg_lock);
	spin_lock_init(&mms->changed_lock);
	for (i = 0; i < desc->item_num; i++)
		rwlock_init(&desc->item_table[i].lock);
	INIT_DELAYED_WORK(&mms->update_work, oplus_mms_update_work);
	INIT_DELAYED_WORK(&mms->msg_work, oplus_mms_msg_work);
	INIT_LIST_HEAD(&mms->subscribe_list);
	INIT_LIST_HEAD(&mms->msg_list);

	rc = device_add(dev);
	if (rc)
		goto device_add_failed;

	rc = device_init_wakeup(dev, ws);
	if (rc)
		goto wakeup_init_failed;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	attrs = oplus_mms_attributes;
	while ((attr = *attrs++)) {
		rc = device_create_file(&mms->dev, attr);
		if (rc) {
			chg_err("device create file fail, rc=%d\n", rc);
			goto device_create_file_err;
		}
	}
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

	atomic_inc(&mms->use_cnt);
	mms->initialized = true;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	snprintf(mms->debug_subs.name, TOPIC_NAME_MAX, "debug");
	mms->debug_subs.priv_data = mms;
	mms->debug_subs.callback = debug_subs_callback;
	INIT_LIST_HEAD(&mms->debug_subs.list);
	oplus_mms_subscribe(mms, &mms->debug_subs);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

	kobject_uevent(&dev->kobj, KOBJ_CHANGE);

	queue_delayed_work(mms_wq, &mms->update_work, 0);

	return mms;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
device_create_file_err:
	device_init_wakeup(&mms->dev, false);
	device_unregister(&mms->dev);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */
wakeup_init_failed:
device_add_failed:
dev_set_name_failed:
	put_device(dev);
	return ERR_PTR(rc);
}

struct oplus_mms *__must_check oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	return __oplus_mms_register(parent, desc, cfg, true);
}

struct oplus_mms *__must_check
oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	return __oplus_mms_register(parent, desc, cfg, false);
}

static void devm_oplus_mms_release(struct device *dev, void *res)
{
	struct oplus_mms **mms = res;

	oplus_mms_unregister(*mms);
}

struct oplus_mms *__must_check
devm_oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	struct oplus_mms **ptr, *mms;

	ptr = devres_alloc(devm_oplus_mms_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	mms = __oplus_mms_register(parent, desc, cfg, true);
	if (IS_ERR(mms)) {
		devres_free(ptr);
	} else {
		*ptr = mms;
		devres_add(parent, ptr);
	}
	return mms;
}

struct oplus_mms *__must_check
devm_oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	struct oplus_mms **ptr, *mms;

	ptr = devres_alloc(devm_oplus_mms_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	mms = __oplus_mms_register(parent, desc, cfg, false);
	if (IS_ERR(mms)) {
		devres_free(ptr);
	} else {
		*ptr = mms;
		devres_add(parent, ptr);
	}
	return mms;
}

void oplus_mms_unregister(struct oplus_mms *mms)
{
#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	struct device_attribute **attrs;
	struct device_attribute *attr;
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

	WARN_ON(atomic_dec_return(&mms->use_cnt));
	mms->removing = true;
	cancel_delayed_work_sync(&mms->update_work);
	cancel_delayed_work_sync(&mms->msg_work);
	sysfs_remove_link(&mms->dev.kobj, "powers");
#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	attrs = oplus_mms_attributes;
	while ((attr = *attrs++))
		device_remove_file(&mms->dev, attr);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */
	device_init_wakeup(&mms->dev, false);
	device_unregister(&mms->dev);
}


static int oplus_mms_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	return 0;
}

void *oplus_mms_get_drvdata(struct oplus_mms *mms)
{
	return mms->drv_data;
}

int oplus_mms_class_init(void)
{
	oplus_mms_class = class_create(THIS_MODULE, "oplus_mms");

	if (IS_ERR(oplus_mms_class))
		return PTR_ERR(oplus_mms_class);

	oplus_mms_class->dev_uevent = oplus_mms_uevent;

	mms_wq = alloc_workqueue("mms_wq", WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI, 0);
	if (!mms_wq) {
		pr_err("alloc mms_wq error\n");
		class_destroy(oplus_mms_class);
		return ENOMEM;
	}

	return 0;
}

void oplus_mms_class_exit(void)
{
	destroy_workqueue(mms_wq);
	class_destroy(oplus_mms_class);
}
