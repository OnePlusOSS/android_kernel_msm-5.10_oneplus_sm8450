// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm Peripheral Image Loader for Q6V5
 *
 * Copyright (C) 2016-2018 Linaro Ltd.
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, 2020-2021, The Linux Foundation. All rights reserved.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/remoteproc.h>
#include <linux/delay.h>
#include "qcom_common.h"
#include "qcom_q6v5.h"
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/qcom_scm.h>
#include <linux/regulator/consumer.h>
#include <linux/interconnect.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include "qcom_pil_info.h"
#include "remoteproc_internal.h"
#include <trace/events/rproc_qcom.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#define REMOTEPROC_ADSP "remoteproc-adsp"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FEEDBACK)
#include <soc/oplus/system/kernel_fb.h>
#define REMOTEPROC_SLPI "remoteproc-slpi"
#define REMOTEPROC_CDSP "remoteproc-cdsp"
#endif

#define Q6V5_PANIC_DELAY_MS	200

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
#define MAX_REASON_LEN 300
#define MAX_DEVICE_NAME 32
struct dev_crash_report_work {
	struct work_struct  work;
	struct device *crash_dev;
	char   device_name[MAX_DEVICE_NAME];
	char   crash_reason[MAX_REASON_LEN];
};

static struct workqueue_struct *crash_report_workqueue = NULL;
#endif

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
#define MAX_SSR_REASON_LEN	256U
#define MAX_SSR_DEFAULT_REASON_LEN 16
#define REMOTEPROC_MSS "remoteproc-mss"
extern bool SKIP_GENERATE_RAMDUMP;
extern void mdmreason_set(char * buf);
#endif

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
unsigned int getBKDRHash(char *str, unsigned int len)
{
	unsigned int seed = 131; /* 31 131 1313 13131 131313 etc.. */
	unsigned int hash = 0;
	unsigned int i    = 0;
	if (str == NULL) {
		return 0;
	}
	for(i = 0; i < len; str++, i++) {
		hash = (hash * seed) + (*str);
	}
	return hash;
}
EXPORT_SYMBOL(getBKDRHash);

static void __modem_send_uevent(struct device *dev, char *reason)
{
	int ret_val;
	char modem_event[] = "MODEM_EVENT=modem_failure";
	char modem_reason[MAX_REASON_LEN] = {0};
	char modem_hashreason[MAX_REASON_LEN] = {0};
	char *envp[4];
	unsigned int hashid = 0;

	envp[0] = (char *)&modem_event;
	if (reason && reason[0]) {
		snprintf(modem_reason, sizeof(modem_reason), "MODEM_REASON=%s", reason);
	} else {
	    snprintf(modem_reason, sizeof(modem_reason), "MODEM_REASON=unkown");
	}
	modem_reason[MAX_REASON_LEN - 1] = 0;
	envp[1] = (char *)&modem_reason;

	hashid = getBKDRHash(reason, strlen(reason));
	snprintf(modem_hashreason, sizeof(modem_hashreason), "MODEM_HASH_REASON=fid:%u;cause:%s", hashid, reason);
	modem_hashreason[MAX_REASON_LEN - 1] = 0;
	pr_info("__subsystem_send_uevent: modem_hashreason: %s\n", modem_hashreason);
	envp[2] = (char *)&modem_hashreason;

	envp[3] = 0;

	if (dev) {
		ret_val = kobject_uevent_env(&(dev->kobj), KOBJ_CHANGE, envp);
		if (!ret_val) {
			pr_info("modem crash:kobject_uevent_env success!\n");
		} else {
			pr_info("modem crash:kobject_uevent_env fail,error=%d!\n", ret_val);
		}
	}

	return;
}

void __adsp_send_uevent(struct device *dev, char *reason)
{
	int ret_val;
	char adsp_event[] = "ADSP_EVENT=adsp_crash";
	char adsp_reason[MAX_REASON_LEN] = {0};
	char *envp[3];

	envp[0] = (char *)&adsp_event;
	if (reason && reason[0]) {
		snprintf(adsp_reason, sizeof(adsp_reason), "ADSP_REASON=%s", reason);
	} else {
	    snprintf(adsp_reason, sizeof(adsp_reason), "ADSP_REASON=unkown");
	}
	adsp_reason[MAX_REASON_LEN - 1] = 0;
	envp[1] = (char *)&adsp_reason;
	envp[2] = 0;

	if (dev) {
		ret_val = kobject_uevent_env(&(dev->kobj), KOBJ_CHANGE, envp);
		if (!ret_val) {
			pr_info("adsp_crash:kobject_uevent_env success!\n");
		} else {
			pr_info("adsp_crash:kobject_uevent_env fail,error=%d!\n", ret_val);
		}
	}
}

static void subsystem_send_uevent(struct work_struct *wk)
{
	struct dev_crash_report_work  *crash_report_wk = container_of(wk, struct dev_crash_report_work, work);
	char *device_name = crash_report_wk->device_name;

	pr_info("[crash_log]: %s to send crash_uevnet\n", device_name);
	if (crash_report_wk && crash_report_wk->crash_dev) {
		if (strstr(device_name, REMOTEPROC_MSS)) {
			__modem_send_uevent(crash_report_wk->crash_dev, (char*)&crash_report_wk->crash_reason);
			pr_info("[crash_log]: %s send crash_uevnet\n", device_name);
		}
		else if (strstr(device_name, REMOTEPROC_ADSP)) {
			__adsp_send_uevent(crash_report_wk->crash_dev, (char*)&crash_report_wk->crash_reason);
		}
	}

	kfree(crash_report_wk);

	return;
}

void subsystem_schedule_crash_uevent_work(struct device *dev, const char *device_name, char *reason)
{
	struct dev_crash_report_work *crash_report_wk;

	if (!device_name) {
		return;
	}

	crash_report_wk = (struct dev_crash_report_work*)kzalloc(sizeof(struct dev_crash_report_work), GFP_ATOMIC);
	if (crash_report_wk == NULL) {
		printk("alloc dev_crash_report_work fail\n");
		return;
	}
	INIT_WORK(&(crash_report_wk->work), subsystem_send_uevent);
	crash_report_wk->crash_dev = dev;
	strlcpy((char*)&crash_report_wk->device_name, device_name, sizeof(crash_report_wk->device_name));
	if (reason) {
		strlcpy((char*)&crash_report_wk->crash_reason, reason, sizeof(crash_report_wk->crash_reason));
	}

	if (crash_report_workqueue) {
		printk("\n");
		pr_info("[crash_log]: queue_work crash_report_workqueue(%s)  \n", device_name);
		queue_work(crash_report_workqueue, &(crash_report_wk->work));
	} else {
		kfree(crash_report_wk);
	}

	return;
}
EXPORT_SYMBOL(subsystem_schedule_crash_uevent_work);
#endif


/**
 * qcom_q6v5_prepare() - reinitialize the qcom_q6v5 context before start
 * @q6v5:	reference to qcom_q6v5 context to be reinitialized
 *
 * Return: 0 on success, negative errno on failure
 */
int qcom_q6v5_prepare(struct qcom_q6v5 *q6v5)
{
	reinit_completion(&q6v5->start_done);
	reinit_completion(&q6v5->stop_done);

	q6v5->running = true;
	q6v5->handover_issued = false;

	enable_irq(q6v5->handover_irq);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_prepare);

/**
 * qcom_q6v5_unprepare() - unprepare the qcom_q6v5 context after stop
 * @q6v5:	reference to qcom_q6v5 context to be unprepared
 *
 * Return: 0 on success, 1 if handover hasn't yet been called
 */
int qcom_q6v5_unprepare(struct qcom_q6v5 *q6v5)
{
	disable_irq(q6v5->handover_irq);

	return !q6v5->handover_issued;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_unprepare);

void qcom_q6v5_register_ssr_subdev(struct qcom_q6v5 *q6v5, struct rproc_subdev *ssr_subdev)
{
	q6v5->ssr_subdev = ssr_subdev;
}
EXPORT_SYMBOL(qcom_q6v5_register_ssr_subdev);

static void qcom_q6v5_crash_handler_work(struct work_struct *work)
{
	struct qcom_q6v5 *q6v5 = container_of(work, struct qcom_q6v5, crash_handler);
	struct rproc *rproc = q6v5->rproc;
	struct rproc_subdev *subdev;
	int votes;
#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	size_t len = 0;
	char *msg = NULL;
	char reason[MAX_SSR_REASON_LEN] = {0};
	const char *defaultMsg = "Unknown reason!";
#endif /* OPLUS_FEATURE_MODEM_MINIDUMP */

	mutex_lock(&rproc->lock);

	rproc->state = RPROC_CRASHED;

	votes = atomic_xchg(&rproc->power, 0);
	/* if votes are zero, rproc has already been shutdown */
	if (votes == 0) {
		mutex_unlock(&rproc->lock);
		return;
	}

	list_for_each_entry_reverse(subdev, &rproc->subdevs, node) {
		if (subdev->stop)
			subdev->stop(subdev, true);
	}

	mutex_unlock(&rproc->lock);

	/*
	 * Temporary workaround until ramdump userspace application calls
	 * sync() and fclose() on attempting the dump.
	 */
	msleep(100);

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	if (strstr(q6v5->rproc->name, REMOTEPROC_MSS)) {
		dev_err(q6v5->dev, "remoteproc %s crashed\n", q6v5->rproc->name);
		msg = qcom_smem_get(QCOM_SMEM_HOST_ANY, q6v5->crash_reason, &len);
		if (msg != NULL) {
			memset(reason, 0, sizeof(reason));
			strlcpy(reason, msg, min(len, (size_t)MAX_SSR_REASON_LEN));
			dev_err(q6v5->dev, "remoteproc crashed reason: %s\n", reason);
		} else {
			dev_err(q6v5->dev, "Can not get crash reason from smem");
			memset(reason, 0, sizeof(reason));
			strlcpy(reason, defaultMsg, MAX_SSR_DEFAULT_REASON_LEN);
		}
		panic("Panicking, remoteproc %s crashed reason: %s\n", q6v5->rproc->name, reason);
	} else {
		panic("Panicking, remoteproc %s crashed\n", q6v5->rproc->name);
	}
#else /* OPLUS_FEATURE_MODEM_MINIDUMP */
	panic("Panicking, remoteproc %s crashed\n", q6v5->rproc->name);
#endif /* OPLUS_FEATURE_MODEM_MINIDUMP */

}

static irqreturn_t q6v5_wdog_interrupt(int irq, void *data)
{
	struct qcom_q6v5 *q6v5 = data;
	struct qcom_rproc_ssr *ssr;
	size_t len;
	char *msg;
	#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	char reason[MAX_SSR_REASON_LEN] = {0};
	const char *name =	q6v5->rproc->name;
	#endif

	/* Sometimes the stop triggers a watchdog rather than a stop-ack */
	if (!q6v5->running) {
		dev_info(q6v5->dev, "received wdog irq while q6 is offline\n");
		complete(&q6v5->stop_done);
		return IRQ_HANDLED;
	}

	msg = qcom_smem_get(QCOM_SMEM_HOST_ANY, q6v5->crash_reason, &len);
	if (!IS_ERR(msg) && len > 0 && msg[0])
		dev_err(q6v5->dev, "watchdog received: %s\n", msg);
	else
		dev_err(q6v5->dev, "watchdog without message\n");

	#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	if (strstr(q6v5->rproc->name, REMOTEPROC_ADSP)) {
		if (!IS_ERR(msg) && len > 0 && msg[0]) {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_ADSP_CRASH, \
				MM_FB_KEY_RATELIMIT_5MIN, "FieldData@@%s$$detailData@@audio$$module@@adsp", msg);
		} else {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_ADSP_CRASH, \
				MM_FB_KEY_RATELIMIT_5MIN, "FieldData@@watchdog without message$$detailData@@audio$$module@@adsp");
		}
	}
	#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FEEDBACK)
	if (strstr(name, REMOTEPROC_SLPI) || strstr(name, REMOTEPROC_CDSP)) {
		if (!IS_ERR(msg) && len > 0 && msg[0]) {
			strcat(reason, "$$module@@");
		} else {
			strcat(reason, "watchdog without message$$module@@");
		}
		strcat(reason, name);
		oplus_kevent_fb_str(FB_SENSOR, FB_SENSOR_ID_CRASH, reason);
	}
#endif

	#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	if (!IS_ERR(msg) && len > 0 && msg[0]) {
		strlcpy(reason, msg, min(len, (size_t)MAX_SSR_REASON_LEN));
		dev_err(q6v5->dev, "%s subsystem failure reason: %s.\n", name, reason);
		if (strstr(name, REMOTEPROC_MSS)) {
			mdmreason_set(reason);
			dev_err(q6v5->dev, "debug modem subsystem failure reason: %s.\n", reason);
			if (strstr(reason, "OPLUS_MODEM_NO_RAMDUMP_EXPECTED") || strstr(reason, "oplusmsg:go_to_error_fatal")) {
				dev_err(q6v5->dev, "%s will subsys reset", __func__);
				SKIP_GENERATE_RAMDUMP = true;
			}
			#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
			dev_err(q6v5->dev, "[crash_log]: %s to schedule crash work1!\n", name);
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, reason);
			#endif
		}
		if (strstr(name, REMOTEPROC_ADSP)) {
			dev_err(q6v5->dev, "[crash_log]: %s to schedule crash work2!\n", name);
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, reason);
		}
	}
	else {
		dev_err(q6v5->dev, "%s SFR: (unknown, empty string found).\n", name);
		if (strstr(name, REMOTEPROC_ADSP) || strstr(name, REMOTEPROC_MSS)) {
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, 0);
		}
	}
	#endif

	q6v5->running = false;
	trace_rproc_qcom_event(dev_name(q6v5->dev), "q6v5_wdog", msg);
	if (q6v5->rproc->recovery_disabled) {
		schedule_work(&q6v5->crash_handler);
	} else {
		if (q6v5->ssr_subdev) {
			qcom_notify_early_ssr_clients(q6v5->ssr_subdev);
			ssr = container_of(q6v5->ssr_subdev, struct qcom_rproc_ssr, subdev);
			ssr->is_notified = true;
		}

		rproc_report_crash(q6v5->rproc, RPROC_WATCHDOG);
	}

	return IRQ_HANDLED;
}

static irqreturn_t q6v5_fatal_interrupt(int irq, void *data)
{
	struct qcom_q6v5 *q6v5 = data;
	struct qcom_rproc_ssr *ssr;
	size_t len;
	char *msg;

	#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	char reason[MAX_SSR_REASON_LEN] = {0};
	const char *name =	q6v5->rproc->name;
	#endif

	if (!q6v5->running) {
		dev_info(q6v5->dev, "received fatal irq while q6 is offline\n");
		return IRQ_HANDLED;
	}

	msg = qcom_smem_get(QCOM_SMEM_HOST_ANY, q6v5->crash_reason, &len);
	if (!IS_ERR(msg) && len > 0 && msg[0])
		dev_err(q6v5->dev, "fatal error received: %s\n", msg);
	else
		dev_err(q6v5->dev, "fatal error without message\n");

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	if (strstr(q6v5->rproc->name, REMOTEPROC_ADSP)) {
		if (!IS_ERR(msg) && len > 0 && msg[0]) {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_ADSP_CRASH, \
				MM_FB_KEY_RATELIMIT_5MIN, "FieldData@@%s$$detailData@@audio$$module@@adsp", msg);
		} else {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_ADSP_CRASH, \
				MM_FB_KEY_RATELIMIT_5MIN, "FieldData@@fatal error without message$$detailData@@audio$$module@@adsp");
		}
	}
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FEEDBACK)
	if (strstr(name, REMOTEPROC_SLPI) || strstr(name, REMOTEPROC_CDSP)) {
		if (!IS_ERR(msg) && len > 0 && msg[0]) {
			strcat(reason, "$$module@@");
		} else {
			strcat(reason, "fatal error without message$$module@@");
		}
		strcat(reason, name);
		oplus_kevent_fb_str(FB_SENSOR, FB_SENSOR_ID_CRASH, reason);
	}
#endif

	#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	if (!IS_ERR(msg) && len > 0 && msg[0]) {
		strlcpy(reason, msg, min(len, (size_t)MAX_SSR_REASON_LEN));
		dev_err(q6v5->dev, "%s subsystem failure reason: %s.\n", name, reason);
		if (strstr(name, REMOTEPROC_MSS)) {
			mdmreason_set(reason);
			dev_err(q6v5->dev, "debug modem subsystem failure reason: %s.\n", reason);
			if (strstr(reason, "OPLUS_MODEM_NO_RAMDUMP_EXPECTED") || strstr(reason, "oplusmsg:go_to_error_fatal")) {
				dev_err(q6v5->dev, "%s will subsys reset", __func__);
				SKIP_GENERATE_RAMDUMP = true;
			}

			#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
			dev_err(q6v5->dev, "[crash_log]: %s to schedule crash work1!\n", name);
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, reason);
			#endif
		}
		if (strstr(name, REMOTEPROC_ADSP)) {
			dev_err(q6v5->dev, "[crash_log]: %s to schedule crash work2!\n", name);
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, reason);
		}
	}
	else {
		dev_err(q6v5->dev, "%s SFR: (unknown, empty string found).\n", name);
		if (strstr(name, REMOTEPROC_ADSP) || strstr(name, REMOTEPROC_MSS)) {
			subsystem_schedule_crash_uevent_work(q6v5->dev, name, 0);
		}
	}
	#endif

	q6v5->running = false;
	trace_rproc_qcom_event(dev_name(q6v5->dev), "q6v5_fatal", msg);
	if (q6v5->rproc->recovery_disabled) {
		schedule_work(&q6v5->crash_handler);
	} else {
		if (q6v5->ssr_subdev) {
			qcom_notify_early_ssr_clients(q6v5->ssr_subdev);
			ssr = container_of(q6v5->ssr_subdev, struct qcom_rproc_ssr, subdev);
			ssr->is_notified = true;
		}

		rproc_report_crash(q6v5->rproc, RPROC_FATAL_ERROR);
	}

	return IRQ_HANDLED;
}

static irqreturn_t q6v5_ready_interrupt(int irq, void *data)
{
	struct qcom_q6v5 *q6v5 = data;

	complete(&q6v5->start_done);

	return IRQ_HANDLED;
}

/**
 * qcom_q6v5_wait_for_start() - wait for remote processor start signal
 * @q6v5:	reference to qcom_q6v5 context
 * @timeout:	timeout to wait for the event, in jiffies
 *
 * qcom_q6v5_unprepare() should not be called when this function fails.
 *
 * Return: 0 on success, -ETIMEDOUT on timeout
 */
int qcom_q6v5_wait_for_start(struct qcom_q6v5 *q6v5, int timeout)
{
	int ret;

	ret = wait_for_completion_timeout(&q6v5->start_done, timeout);
	if (!ret)
		disable_irq(q6v5->handover_irq);

	return !ret ? -ETIMEDOUT : 0;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_wait_for_start);

static irqreturn_t q6v5_handover_interrupt(int irq, void *data)
{
	struct qcom_q6v5 *q6v5 = data;

	if (q6v5->handover)
		q6v5->handover(q6v5);

	q6v5->handover_issued = true;

	return IRQ_HANDLED;
}

static irqreturn_t q6v5_stop_interrupt(int irq, void *data)
{
	struct qcom_q6v5 *q6v5 = data;

	complete(&q6v5->stop_done);

	return IRQ_HANDLED;
}

/**
 * qcom_q6v5_request_stop() - request the remote processor to stop
 * @q6v5:	reference to qcom_q6v5 context
 * @sysmon:	reference to the remote's sysmon instance, or NULL
 *
 * Return: 0 on success, negative errno on failure
 */
int qcom_q6v5_request_stop(struct qcom_q6v5 *q6v5, struct qcom_sysmon *sysmon)
{
	int ret;

	q6v5->running = false;

	/* Don't perform SMP2P dance if sysmon already shut
	 * down the remote or if it isn't running
	 */
	if (q6v5->rproc->state != RPROC_RUNNING || qcom_sysmon_shutdown_acked(sysmon))
		return 0;

	qcom_smem_state_update_bits(q6v5->state,
				    BIT(q6v5->stop_bit), BIT(q6v5->stop_bit));

	ret = wait_for_completion_timeout(&q6v5->stop_done, 5 * HZ);

	qcom_smem_state_update_bits(q6v5->state, BIT(q6v5->stop_bit), 0);

	return ret == 0 ? -ETIMEDOUT : 0;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_request_stop);

/**
 * qcom_q6v5_panic() - panic handler to invoke a stop on the remote
 * @q6v5:	reference to qcom_q6v5 context
 *
 * Set the stop bit and sleep in order to allow the remote processor to flush
 * its caches etc for post mortem debugging.
 *
 * Return: 200ms
 */
unsigned long qcom_q6v5_panic(struct qcom_q6v5 *q6v5)
{
	qcom_smem_state_update_bits(q6v5->state,
				    BIT(q6v5->stop_bit), BIT(q6v5->stop_bit));

	return Q6V5_PANIC_DELAY_MS;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_panic);

/**
 * qcom_q6v5_init() - initializer of the q6v5 common struct
 * @q6v5:	handle to be initialized
 * @pdev:	platform_device reference for acquiring resources
 * @rproc:	associated remoteproc instance
 * @crash_reason: SMEM id for crash reason string, or 0 if none
 * @handover:	function to be called when proxy resources should be released
 *
 * Return: 0 on success, negative errno on failure
 */
int qcom_q6v5_init(struct qcom_q6v5 *q6v5, struct platform_device *pdev,
		   struct rproc *rproc, int crash_reason,
		   void (*handover)(struct qcom_q6v5 *q6v5))
{
	int ret;

	q6v5->rproc = rproc;
	q6v5->dev = &pdev->dev;
	q6v5->crash_reason = crash_reason;
	q6v5->handover = handover;
	q6v5->ssr_subdev = NULL;

	init_completion(&q6v5->start_done);
	init_completion(&q6v5->stop_done);

	q6v5->wdog_irq = platform_get_irq_byname(pdev, "wdog");
	if (q6v5->wdog_irq < 0)
		return q6v5->wdog_irq;

	ret = devm_request_threaded_irq(&pdev->dev, q6v5->wdog_irq,
					NULL, q6v5_wdog_interrupt,
					IRQF_ONESHOT,
					"q6v5 wdog", q6v5);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire wdog IRQ\n");
		return ret;
	}

	q6v5->fatal_irq = platform_get_irq_byname(pdev, "fatal");
	if (q6v5->fatal_irq < 0)
		return q6v5->fatal_irq;

	ret = devm_request_threaded_irq(&pdev->dev, q6v5->fatal_irq,
					NULL, q6v5_fatal_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"q6v5 fatal", q6v5);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire fatal IRQ\n");
		return ret;
	}

	q6v5->ready_irq = platform_get_irq_byname(pdev, "ready");
	if (q6v5->ready_irq < 0)
		return q6v5->ready_irq;

	ret = devm_request_threaded_irq(&pdev->dev, q6v5->ready_irq,
					NULL, q6v5_ready_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"q6v5 ready", q6v5);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire ready IRQ\n");
		return ret;
	}

	q6v5->handover_irq = platform_get_irq_byname(pdev, "handover");
	if (q6v5->handover_irq < 0)
		return q6v5->handover_irq;

	ret = devm_request_threaded_irq(&pdev->dev, q6v5->handover_irq,
					NULL, q6v5_handover_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"q6v5 handover", q6v5);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire handover IRQ\n");
		return ret;
	}
	disable_irq(q6v5->handover_irq);

	q6v5->stop_irq = platform_get_irq_byname(pdev, "stop-ack");
	if (q6v5->stop_irq < 0)
		return q6v5->stop_irq;

	ret = devm_request_threaded_irq(&pdev->dev, q6v5->stop_irq,
					NULL, q6v5_stop_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"q6v5 stop", q6v5);
	if (ret) {
		dev_err(&pdev->dev, "failed to acquire stop-ack IRQ\n");
		return ret;
	}

	q6v5->state = qcom_smem_state_get(&pdev->dev, "stop", &q6v5->stop_bit);
	if (IS_ERR(q6v5->state)) {
		dev_err(&pdev->dev, "failed to acquire stop state\n");
		return PTR_ERR(q6v5->state);
	}

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
	if (crash_report_workqueue == NULL) {
		crash_report_workqueue = create_singlethread_workqueue("crash_report_workqueue");
		if (crash_report_workqueue == NULL) {
			dev_err(&pdev->dev,"crash_report_workqueue alloc fail\n");
		}
	}
#endif
	INIT_WORK(&q6v5->crash_handler, qcom_q6v5_crash_handler_work);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_q6v5_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Peripheral Image Loader for Q6V5");
