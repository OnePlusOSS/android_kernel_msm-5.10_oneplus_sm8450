// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_debug_info.h"
#include "oplus_voocphy.h"

#define AP_ALLOW_FASTCHG	(1 << 6)
#define TARGET_VOL_OFFSET_THR	250
#define DELAY_TEMP_MONITOR_COUNTS		2
#define trace_oplus_tp_sched_change_ux(x, y)
#define CHARGER_UP_CPU_FREQ	2000000	//1.8 x 1000 x 1000

//int flag
#define VOUT_OVP_FLAG_MASK	BIT(7)
#define VBAT_OVP_FLAG_MASK	BIT(6)
#define IBAT_OCP_FLAG_MASK	BIT(5)
#define VBUS_OVP_FLAG_MASK	BIT(4)
#define IBUS_OCP_FLAG_MASK	BIT(3)
#define IBUS_UCP_FLAG_MASK	BIT(2)
#define ADAPTER_INSERT_FLAG_MASK	BIT(1)
#define VBAT_INSERT_FLAG_MASK	BIT(0)

//vooc flag
#define PULSE_FILTERED_STAT_MASK	BIT(7)
#define NINTH_CLK_ERR_FLAG_MASK		BIT(6)
#define TXSEQ_DONE_FLAG_MASK		BIT(5)
#define ERR_TRANS_DET_FLAG_MASK		BIT(4)
#define TXDATA_WR_FAIL_FLAG_MASK	BIT(3)
#define RX_START_FLAG_MASK			BIT(2)
#define RXDATA_DONE_FLAG_MASK		BIT(1)
#define TXDATA_DONE_FALG_MASK		BIT(0)

int voocphy_log_level = 1;
#define voocphy_info(fmt, ...)	\
do {						\
	if (voocphy_log_level <= 3)	\
		printk(KERN_INFO "%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define voocphy_dbg(fmt, ...)	\
do {					\
	if (voocphy_log_level <= 2)	\
		printk(KERN_DEBUG "%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define voocphy_err(fmt, ...)   \
do {                                    \
	if (voocphy_log_level <= 1)     \
		printk(KERN_ERR "%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);

#define IRQ_EVNET_NUM	8

struct voocphy_log_buf *g_voocphy_log_buf = NULL;
static struct oplus_voocphy_manager *g_voocphy_chip = NULL;
ktime_t calltime, rettime;
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
static struct pm_qos_request pm_qos_req;
#endif
static int disable_sub_cp_count = 0;
static int slave_trouble_count = 0;

extern int oplus_chg_get_battery_btb_temp_cal(void);
extern int oplus_chg_get_usb_btb_temp_cal(void);
extern void oplus_chg_adc_switch_ctrl(void);
extern bool oplus_chg_check_pd_svooc_adapater(void);

int __attribute__((weak)) oplus_chg_get_battery_btb_temp_cal(void)
{
	return 25;
}

int __attribute__((weak)) oplus_chg_get_usb_btb_temp_cal(void)
{
	return 25;
}

void __attribute__((weak)) oplus_chg_adc_switch_ctrl(void)
{
	return;
}

bool __attribute__((weak)) oplus_chg_check_pd_svooc_adapater(void)
{
	return false;
}

int base_cpufreq_for_chg = CHARGER_UP_CPU_FREQ;
int cpu_freq_stratehy = CPU_CHG_FREQ_STAT_AUTO;
EXPORT_SYMBOL(base_cpufreq_for_chg);
EXPORT_SYMBOL(cpu_freq_stratehy);

struct irqinfo {
	int mask;
	char except_info[30];
	int mark_except;
};

struct irqinfo int_flag[IRQ_EVNET_NUM] = {
	{VOUT_OVP_FLAG_MASK, "VOUT_OVP", 0},
	{VBAT_OVP_FLAG_MASK, "VBAT_OVP", 1},
	{IBAT_OCP_FLAG_MASK, "IBAT_OCP", 1},
	{VBUS_OVP_FLAG_MASK, "VBUS_OVP", 1},
	{IBUS_OCP_FLAG_MASK, "IBUS_OCP", 1},
	{IBUS_UCP_FLAG_MASK, "IBUS_UCP", 1},
	{ADAPTER_INSERT_FLAG_MASK, "ADAPTER_INSERT", 1},
	{VBAT_INSERT_FLAG_MASK,   "VBAT_INSERT", 1},
};


struct irqinfo vooc_flag[IRQ_EVNET_NUM] = {
	{PULSE_FILTERED_STAT_MASK	, "PULSE_FILTERED_STAT", 1},
	{NINTH_CLK_ERR_FLAG_MASK	, "NINTH_CLK_ERR_FLAG", 1},
	{TXSEQ_DONE_FLAG_MASK		, "TXSEQ_DONE_FLAG", 0},
	{ERR_TRANS_DET_FLAG_MASK	, "ERR_TRANS_DET_FLAG", 1},
	{TXDATA_WR_FAIL_FLAG_MASK	, "TXDATA_WR_FAIL_FLAG", 1},
	{RX_START_FLAG_MASK			, "RX_START_FLAG", 0},
	{RXDATA_DONE_FLAG_MASK		, "RXDATA_DONE_FLAG", 0},
	{TXDATA_DONE_FALG_MASK		, "TXDATA_DONE_FALG", 0},
};

unsigned char svooc_batt_sys_curve[BATT_SYS_ARRAY][7] = {0};
unsigned char vooc_batt_sys_curve[BATT_SYS_ARRAY][7] = {0};

/*svooc curv*/
/*0~50*/
struct batt_sys_curves svooc_curves_soc0_2_50[BATT_SYS_MAX] = {0};
/*50~75*/
struct batt_sys_curves svooc_curves_soc50_2_75[BATT_SYS_MAX] = {0};
/*75~85*/
struct batt_sys_curves svooc_curves_soc75_2_85[BATT_SYS_MAX] = {0};
/*85~90*/
struct batt_sys_curves svooc_curves_soc85_2_90[BATT_SYS_MAX] = {0};

/*vooc curv*/
/*0~50*/
struct batt_sys_curves vooc_curves_soc0_2_50[BATT_SYS_MAX] = {0};
/*50~75*/
struct batt_sys_curves vooc_curves_soc50_2_75[BATT_SYS_MAX] = {0};
/*75~85*/
struct batt_sys_curves vooc_curves_soc75_2_85[BATT_SYS_MAX] = {0};
/*85~90*/
struct batt_sys_curves vooc_curves_soc85_2_90[BATT_SYS_MAX] = {0};

extern	void oplus_chg_disable_charge(void);
extern bool oplus_voocphy_get_fastchg_to_warm(void);
extern void oplus_chg_clear_chargerid_info(void);
extern int oplus_chg_get_curr_time_ms(unsigned long *time_ms);
extern void oplus_vooc_switch_fast_chg(void);
bool oplus_voocphy_check_fastchg_real_allow(void);
extern int oplus_vooc_get_vooc_switch_val(void);
void oplus_vooc_switch_mode(int mode);
extern void oplus_chg_set_charger_type_unknown(void);
extern void oplus_chg_suspend_charger(void);
extern void oplus_chg_unsuspend_charger(void);

extern struct oplus_chg_chip *g_oplus_chip;
extern int oplus_chg_get_ui_soc(void);
extern int oplus_chg_get_chg_temperature(void);
extern int oplus_chg_get_charger_voltage(void);
int oplus_voocphy_ap_event_handle(unsigned long data);	//lkl need modify
int oplus_voocphy_vol_event_handle(unsigned long data);
int oplus_voocphy_safe_event_handle(unsigned long data);
int oplus_voocphy_ibus_check_event_handle(unsigned long data);
int oplus_voocphy_monitor_start(struct oplus_voocphy_manager *chip);
int oplus_voocphy_monitor_stop(struct oplus_voocphy_manager *chip);

int oplus_voocphy_temp_event_handle(unsigned long data);
int oplus_voocphy_discon_event_handle(unsigned long data);
int oplus_voocphy_btb_event_handle(unsigned long data);
void oplus_voocphy_reset_fastchg_after_usbout(void);
int oplus_voocphy_chg_out_check_event_handle(unsigned long data);
int oplus_voocphy_curr_event_handle(unsigned long data);
bool oplus_vooc_wake_voocphy_service_work(struct oplus_voocphy_manager *chip, int request);
extern bool oplus_vooc_wake_monitor_work(struct oplus_voocphy_manager *chip);
enum hrtimer_restart oplus_voocphy_monitor_base_timer(struct hrtimer* time);
void oplus_voocphy_set_status_and_notify_ap(struct oplus_voocphy_manager *chip, int fastchg_notify_status);
extern bool oplus_chg_stats(void);
int oplus_voocphy_fastchg_check_process_handle(unsigned long enable);
int oplus_voocphy_commu_process_handle(unsigned long enable);
extern void cpuboost_charge_event(int flag);

extern bool oplus_chg_get_flash_led_status(void);

static int oplus_voocphy_batt_fake_temp_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", g_voocphy_chip->batt_fake_temp);
	return 0;
}
static int oplus_voocphy_batt_fake_temp_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = single_open(file, oplus_voocphy_batt_fake_temp_show, NULL);
	return ret;
}

static void oplus_voocphy_reset_ibus_trouble_flag(void)
{
	slave_trouble_count = 0;
}

static ssize_t oplus_voocphy_batt_fake_temp_write(struct file *filp,
        const char __user *buff, size_t len, loff_t *data)
{
	int batt_fake_temp;
	char temp[16];

	if (len > sizeof(temp)) {
		return -EINVAL;
	}
	if (copy_from_user(temp, buff, len)) {
		voocphy_info("%s error.\n", __func__);
		return -EFAULT;
	}
	sscanf(temp, "%d", &batt_fake_temp);
	g_voocphy_chip->batt_fake_temp = batt_fake_temp;
	voocphy_info("batt_fake_temp is %d\n", batt_fake_temp);

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_voocphy_batt_fake_temp_fops = {
	.open = oplus_voocphy_batt_fake_temp_open,
	.write = oplus_voocphy_batt_fake_temp_write,
	.read = seq_read,
};
#else
static const struct proc_ops oplus_voocphy_batt_fake_temp_fops = {
	.proc_open = oplus_voocphy_batt_fake_temp_open,
	.proc_write = oplus_voocphy_batt_fake_temp_write,
	.proc_read = seq_read,
};
#endif

static int oplus_voocphy_batt_fake_soc_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", g_voocphy_chip->batt_fake_soc);
	return 0;
}

static int oplus_voocphy_batt_fake_soc_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = single_open(file, oplus_voocphy_batt_fake_soc_show, NULL);
	return ret;
}

static ssize_t oplus_voocphy_batt_fake_soc_write(struct file *filp,
        const char __user *buff, size_t len, loff_t *data)
{
	int batt_fake_soc;
	char temp[16];

	if (len > sizeof(temp)) {
		return -EINVAL;
	}
	if (copy_from_user(temp, buff, len)) {
		voocphy_info("voocphy_batt_fake_soc_write error.\n");
		return -EFAULT;
	}
	sscanf(temp, "%d", &batt_fake_soc);
	g_voocphy_chip->batt_fake_soc = batt_fake_soc;
	voocphy_info("batt_fake_soc is %d\n", batt_fake_soc);

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_voocphy_batt_fake_soc_fops = {
	.open = oplus_voocphy_batt_fake_soc_open,
	.write = oplus_voocphy_batt_fake_soc_write,
	.read = seq_read,
};
#else
static const struct proc_ops oplus_voocphy_batt_fake_soc_fops = {
	.proc_open = oplus_voocphy_batt_fake_soc_open,
	.proc_write = oplus_voocphy_batt_fake_soc_write,
	.proc_read = seq_read,
};
#endif

static int oplus_voocphy_loglevel_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", voocphy_log_level);
	return 0;
}
static int oplus_voocphy_loglevel_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = single_open(file, oplus_voocphy_loglevel_show, NULL);
	return ret;
}

static ssize_t oplus_voocphy_loglevel_write(struct file *filp,
        const char __user *buff, size_t len, loff_t *data)
{
	char temp[16];

	if (len > sizeof(temp)) {
		return -EINVAL;
	}
	if (copy_from_user(temp, buff, len)) {
		voocphy_info("voocphy_loglevel_write error.\n");
		return -EFAULT;
	}
	sscanf(temp, "%d", &voocphy_log_level);
	voocphy_info("voocphy_log_level is %d\n", voocphy_log_level);

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_voocphy_loglevel_fops = {
	.open = oplus_voocphy_loglevel_open,
	.write = oplus_voocphy_loglevel_write,
	.read = seq_read,
};
#else
static const struct proc_ops oplus_voocphy_loglevel_fops = {
	.proc_open = oplus_voocphy_loglevel_open,
	.proc_write = oplus_voocphy_loglevel_write,
	.proc_read = seq_read,
};
#endif

void init_proc_voocphy_debug(void)
{
	struct proc_dir_entry *p_batt_fake_temp = NULL;
	struct proc_dir_entry *p_batt_fake_soc = NULL;
	struct proc_dir_entry *p_batt_loglevel = NULL;

	p_batt_fake_temp = proc_create("voocphy_batt_fake_temp", 0664, NULL,
	                               &oplus_voocphy_batt_fake_temp_fops);
	if (!p_batt_fake_temp) {
		voocphy_info("proc_create voocphy_batt_fake_temp_fops fail!\n");
	}
	g_voocphy_chip->batt_fake_temp = 0;

	p_batt_fake_soc = proc_create("voocphy_batt_fake_soc", 0664, NULL,
	                              &oplus_voocphy_batt_fake_soc_fops);
	if (!p_batt_fake_soc) {
		voocphy_info("proc_create voocphy_batt_fake_soc_fops fail!\n");
	}
	g_voocphy_chip->batt_fake_soc = 0;


	p_batt_loglevel = proc_create("voocphy_loglevel", 0664, NULL,
	                              &oplus_voocphy_loglevel_fops);
	if (!p_batt_loglevel) {
		voocphy_info("proc_create voocphy_loglevel_fops fail!\n");
	}

}

int init_voocphy_log_buf(void)
{
	if(!g_voocphy_log_buf) {
		g_voocphy_log_buf = kzalloc(sizeof(struct voocphy_log_buf), GFP_KERNEL);
		chg_err("sizeof(t_voocphy_log_buf) [%d] sizeof(t_voocphy_info)[%d]", sizeof(struct voocphy_log_buf), sizeof(struct voocphy_info));
		if (g_voocphy_log_buf == NULL) {
			chg_err("voocphy_log_buf kzalloc error");
			return -1;
		}
	} else {
		memset(g_voocphy_log_buf->voocphy_info_buf, 0, VOOCPHY_LOG_BUF_LEN);
	}
	g_voocphy_log_buf->point = 0;
	return 0;
}

int write_info_to_voocphy_log_buf(struct voocphy_info *voocphy)
{
	int curr_point = 0;
	if (voocphy == NULL || g_voocphy_log_buf == NULL) {
		chg_err("voocphy || g_voocphy_log_buf is NULL");
		return -1;
	}
	if (g_voocphy_log_buf->point >= VOOCPHY_LOG_BUF_LEN) {
		chg_err("g_voocphy_log_buf->point[%d] >= VOOCPHY_LOG_BUF_LEN", g_voocphy_log_buf->point);
		return -1;
	}
	curr_point = g_voocphy_log_buf->point;
	memcpy(&g_voocphy_log_buf->voocphy_info_buf[curr_point], voocphy, sizeof(struct voocphy_info));
	curr_point = g_voocphy_log_buf->point++;

	voocphy->kernel_time = 0;
	voocphy->recv_adapter_msg = 0;
	voocphy->irq_took_time = 0;
	voocphy->send_msg = 0;
	return 0;
}

int print_voocphy_log_buf(void)
{
	int curr_point = 0;
	if (g_voocphy_log_buf == NULL) {
		//chg_err("voocphy || g_voocphy_log_buf is NULL");
		return -1;
	}
	if (g_voocphy_log_buf->point <= 0) {
		chg_err("g_voocphy_log_buf->point[%d]", g_voocphy_log_buf->point);
		return -1;
	}

	do {
		chg_err("g_voocphy_log_buf[%d, %d, %d, 0x%x, 0x%x, 0x%x]", curr_point,
		        g_voocphy_log_buf->voocphy_info_buf[curr_point].kernel_time,
		        g_voocphy_log_buf->voocphy_info_buf[curr_point].irq_took_time,
		        g_voocphy_log_buf->voocphy_info_buf[curr_point].recv_adapter_msg,
		        g_voocphy_log_buf->voocphy_info_buf[curr_point].send_msg,
		        g_voocphy_log_buf->voocphy_info_buf[curr_point].predata);
		curr_point++;
	} while(curr_point < g_voocphy_log_buf->point);
	init_voocphy_log_buf();

	return 0;
}

int oplus_voocphy_print_dbg_info(struct oplus_voocphy_manager *chip)
{
	int i = 0;

	//print irq event
	for (i = 0; i < IRQ_EVNET_NUM; i++) {
		if ((vooc_flag[i].mask & chip->vooc_flag) && vooc_flag[i].mark_except) {
			printk("vooc happened %s\n", vooc_flag[i].except_info);
			goto chg_exception;
		}
	}
	for (i = 0; i < IRQ_EVNET_NUM; i++) {
		if ((int_flag[i].mask & chip->int_flag) && int_flag[i].mark_except) {
			printk("cp int happened %s\n", int_flag[i].except_info);
			goto chg_exception;
		}
	}

	if (chip->fastchg_adapter_ask_cmd != VOOC_CMD_GET_BATT_VOL) {
		goto chg_exception;
	} else {
		if ((chip->irq_total_num % 50) == 0) {		//about 2s
			goto chg_exception;
		}
	}
	return 0;

chg_exception:
	printk(KERN_ERR "voocphydbg data[%d %d %d %d %d], status[%d, %d], init[%d, %d], set[%d, %d]"
	       "comm[rcv:%d, 0x%0x, 0x%0x, %d, 0x%0x, 0x%0x reply:0x%0x 0x%0x], "
	       "irqinfo[%d, %d, %d, %d, %d, %d, %d, %d] other[%d]\n",
	       chip->cp_vsys, chip->cp_ichg, chip->icharging, chip->cp_vbus,chip->gauge_vbatt, //data
	       chip->vooc_vbus_status, chip->vbus_vbatt,										//stutus
	       chip->batt_soc_plugin, chip->batt_temp_plugin,									//init
	       chip->current_expect, chip->current_max,										//setting
	       chip->vooc_move_head, chip->voocphy_rx_buff,									//comm rcv
	       chip->vooc_head, chip->adapter_type,
	       chip->adapter_mesg, chip->fastchg_adapter_ask_cmd,
	       chip->voocphy_tx_buff[0], chip->voocphy_tx_buff[1],								//comm reply
	       chip->vooc_flag, chip->int_flag,chip->irq_total_num,							//irqinfo
	       chip->irq_rcvok_num, chip->irq_rcverr_num, chip->irq_tx_timeout_num,
	       chip->irq_tx_timeout, chip->irq_hw_timeout_num,
	       base_cpufreq_for_chg);															//others

	return 0;
}

int oplus_voocphy_hardware_monitor_stop(void)
{
	int status = VOOCPHY_SUCCESS;

	/*disable batt adc*/

	/*disable temp adc*/

	/*disable dchg irq*/

	return status;
}

static void oplus_voocphy_monitor_timer_init (struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_err("oplus_voocphy_monitor_timer_init chip null\n");
		return;
	}

	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].expires = VOOC_AP_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].mornitor_hdler = oplus_voocphy_ap_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_AP].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].expires = VOOC_CURR_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].mornitor_hdler = oplus_voocphy_curr_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CURR].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].expires = VOOC_SOC_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].mornitor_hdler = NULL;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SOC].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].expires = VOOC_VOL_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].mornitor_hdler = oplus_voocphy_vol_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_VOL].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].expires = VOOC_TEMP_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].mornitor_hdler = oplus_voocphy_temp_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEMP].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].expires = VOOC_BTB_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].mornitor_hdler = oplus_voocphy_btb_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_BTB].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].expires = VOOC_SAFE_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].mornitor_hdler = oplus_voocphy_safe_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_SAFE].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].expires = VOOC_COMMU_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].data = true;
	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].mornitor_hdler = oplus_voocphy_commu_process_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_COMMU].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].expires = VOOC_DISCON_EVENT_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].mornitor_hdler = NULL;
	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_DISCON].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].expires = VOOC_FASTCHG_CHECK_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].data = true;
	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].mornitor_hdler = oplus_voocphy_fastchg_check_process_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_FASTCHG_CHECK].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].expires = VOOC_CHG_OUT_CHECK_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].mornitor_hdler = oplus_voocphy_chg_out_check_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_CHG_OUT_CHECK].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].expires = VOOC_TEST_CHECK_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].mornitor_hdler = NULL;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_TEST_CHECK].timeout = false;

	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].expires = VOOC_IBUS_CHECK_TIME / VOOC_BASE_TIME_STEP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].data = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].mornitor_hdler = oplus_voocphy_ibus_check_event_handle;
	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].status = VOOC_MONITOR_STOP;
	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].cnt = 0;
	chip->mornitor_evt[VOOC_THREAD_TIMER_IBUS_CHECK].timeout = false;

	//init hrtimer
	hrtimer_init(&chip->monitor_btimer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	chip->monitor_btimer.function = oplus_voocphy_monitor_base_timer;

	voocphy_dbg( "vooc: create timers successfully");
}

int oplus_voocphy_monitor_timer_start(struct oplus_voocphy_manager *chip,
                                      int timer_id, int duration_in_ms)
{
	if( (timer_id >= VOOC_THREAD_TIMER_INVALID) || (duration_in_ms < 0))
		return VOOCPHY_EFATAL;

	if (!chip) {
		voocphy_err( "oplus_voocphy_monitor_timer_start chip null\n");
		return VOOCPHY_EFATAL;
	}

	chip->mornitor_evt[timer_id].cnt = 0;
	chip->mornitor_evt[timer_id].timeout = false;
	chip->mornitor_evt[timer_id].status = VOOC_MONITOR_START;

	voocphy_dbg("timerid %ul\n", timer_id);

	return VOOCPHY_SUCCESS;
}

int oplus_voocphy_monitor_timer_stop(struct oplus_voocphy_manager *chip,
                                     int timer_id, int duration_in_ms)
{
	if( (timer_id >= VOOC_THREAD_TIMER_INVALID) || (duration_in_ms < 0))
		return VOOCPHY_EFATAL;

	if (!chip) {
		voocphy_err( "oplus_voocphy_monitor_timer_stop chip is null");
		return VOOCPHY_EFATAL;
	}

	chip->mornitor_evt[timer_id].cnt = 0;
	chip->mornitor_evt[timer_id].timeout = false;
	chip->mornitor_evt[timer_id].status = VOOC_MONITOR_STOP;
	voocphy_dbg("timerid %d\n", timer_id);

	return VOOCPHY_SUCCESS;
}

void oplus_voocphy_basetimer_monitor_start(struct oplus_voocphy_manager *chip)
{
	if (chip && !hrtimer_active(&chip->monitor_btimer)) {
		chip->moniotr_kt = ktime_set(0,5000000);// 0s	5000000ns	= 5ms
		hrtimer_start(&chip->monitor_btimer, chip->moniotr_kt, HRTIMER_MODE_REL);
		voocphy_info("finihed\n");
	}
}

void oplus_voocphy_basetimer_monitor_stop(struct oplus_voocphy_manager *chip)
{
	if (chip && hrtimer_active(&chip->monitor_btimer)) {
		hrtimer_cancel(&chip->monitor_btimer);
		voocphy_info("finihed");
	}
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_COMMU, VOOC_COMMU_EVENT_TIME);
}

int  oplus_voocphy_read_mesg_mask(unsigned char mask, unsigned char data, unsigned char *read_data)
{
	int status = VOOCPHY_SUCCESS;

	*read_data = (data & mask) >> VOOC_SHIFT_FROM_MASK(mask);

	return status;
}

int  oplus_voocphy_write_mesg_mask(unsigned char mask, unsigned char *pdata, unsigned char write_data)
{
	int status = VOOCPHY_SUCCESS;

	write_data <<= VOOC_SHIFT_FROM_MASK(mask);
	*pdata = (*pdata & ~mask) | (write_data & mask);

	return status;
}

static int oplus_voocphy_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (chip->ops && chip->ops->get_chg_enable) {
		rc = chip->ops->get_chg_enable(chip, data);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_get_chg_enable, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

int oplus_voocphy_get_adc_enable(u8 *data)
{
	int rc = 0;
	if (!g_voocphy_chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (g_voocphy_chip->ops && g_voocphy_chip->ops->get_adc_enable) {
		rc = g_voocphy_chip->ops->get_adc_enable(g_voocphy_chip, data);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_get_adc_enable, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

int oplus_voocphy_get_ichg(void)
{
	int cp_ichg = 0;
	if (!g_voocphy_chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (g_voocphy_chip->ops && g_voocphy_chip->ops->get_ichg) {
		cp_ichg = g_voocphy_chip->ops->get_ichg(g_voocphy_chip);
		return cp_ichg;
	}
	return 0;
}

int oplus_voocphy_get_slave_ichg(void)
{
	int slave_cp_ichg = 0;
	if (!g_voocphy_chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (g_voocphy_chip->slave_ops && g_voocphy_chip->slave_ops->get_ichg) {
		slave_cp_ichg = g_voocphy_chip->slave_ops->get_ichg(g_voocphy_chip);
		return slave_cp_ichg;
	}
	return 0;
}

int oplus_voocphy_slave_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int slave_cp_ichg = 0;
	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
	}

	if (chip->slave_ops && chip->slave_ops->get_chg_enable) {
		slave_cp_ichg = chip->slave_ops->get_chg_enable(chip, data);
		return slave_cp_ichg;
	}
	return 0;
}

static int oplus_voocphy_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (chip->ops && chip->ops->set_chg_enable) {
		rc = chip->ops->set_chg_enable(chip, enable);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_set_chg_enable fail, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

int oplus_voocphy_set_adc_enable(bool enable)
{
	int rc = 0;
	if (!g_voocphy_chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (g_voocphy_chip->ops && g_voocphy_chip->ops->set_adc_enable) {
		rc = g_voocphy_chip->ops->set_adc_enable(g_voocphy_chip, enable);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_set_adc_enable fail, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

int oplus_voocphy_slave_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
	}

	if (chip->slave_ops && chip->slave_ops->set_chg_enable) {
		rc = chip->slave_ops->set_chg_enable(chip, enable);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_set_chg_enable fail, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_voocphy_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
	}

	if (chip->ops && chip->ops->set_predata) {
		rc = chip->ops->set_predata(chip, val);
		if (rc < 0) {
			voocphy_info("oplus_voocphy_set_predata fail, rc=%d.\n", rc);
			return rc;
		}
	}

	return rc;
}

void oplus_voocphy_send_handshake(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return;
	}

	if (chip->ops && chip->ops->send_handshake) {
		chip->ops->send_handshake(chip);
	}

	return;
}

void oplus_voocphy_set_switch_mode(int mode)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}

	if (chip->ops && chip->ops->set_switch_mode) {
		chip->ops->set_switch_mode(chip, mode);
	}

	return;
}

void oplus_voocphy_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return;
	}

	if (chip->ops && chip->ops->reactive_voocphy) {
		chip->ops->reactive_voocphy(chip);
	}

	return;
}

int oplus_voocphy_reset_voocphy(void)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return VOOCPHY_EFAILED;
	}

	//stop hardware monitor	//lkl need modify
	status = oplus_voocphy_hardware_monitor_stop();
	if (status != VOOCPHY_SUCCESS)
		return VOOCPHY_EFAILED;

	if (chip->slave_ops && chip->slave_ops->reset_voocphy) {
		chip->slave_ops->reset_voocphy(chip);
	}

	if (chip->ops && chip->ops->reset_voocphy) {
		chip->ops->reset_voocphy(chip);
	}

	return VOOCPHY_SUCCESS;
}

static int oplus_voocphy_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (chip->ops && chip->ops->get_adapter_info) {
		rc = chip->ops->get_adapter_info(chip);
	}

	return rc;
}

int oplus_voocphy_get_adapter_type(void)
{
	if(!g_voocphy_chip)
		return ADAPTER_VOOC20;

	return g_voocphy_chip->adapter_type;
}

static void oplus_voocphy_update_data(struct oplus_voocphy_manager *chip)
{
	int div_cp_ichg;
	int slave_cp_ichg;

	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return ;
	}

	if (chip->ops && chip->ops->update_data) {
		chip->ops->update_data(chip);
	}

	if (chip->external_gauge_support)
		chip->gauge_vbatt = oplus_gauge_get_batt_mvolts();
	else
		chip->gauge_vbatt = chip->cp_vbat;

	if (chip->voocphy_dual_cp_support) {
		if(!chip->slave_ops) {
			voocphy_err("slave_ops is NULL!\n");
		} else {
			slave_cp_ichg = chip->slave_ops->get_ichg(chip);
			div_cp_ichg = (chip->cp_ichg > slave_cp_ichg)? (chip->cp_ichg - slave_cp_ichg) : (slave_cp_ichg - chip->cp_ichg);
			chip->cp_ichg = chip->cp_ichg + slave_cp_ichg;
			voocphy_err("total cp_ichg = %d div_ichg = %d\n", chip->cp_ichg, div_cp_ichg);
		}
	}

	return;
}

void oplus_voocphy_slave_update_data(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return ;
	}

	if (chip->slave_ops && chip->slave_ops->update_data) {
		chip->slave_ops->update_data(chip);
	}

	return;
}

static int oplus_voocphy_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (chip->ops && chip->ops->hw_setting) {
		rc = chip->ops->hw_setting(chip, reason);
	}

	return rc;
}

int oplus_voocphy_get_cp_vbat(void)
{
	int cp_vbat = 0;
	if (!g_voocphy_chip) {
		voocphy_err("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (g_voocphy_chip->ops && g_voocphy_chip->ops->get_cp_vbat) {
		cp_vbat = g_voocphy_chip->ops->get_cp_vbat(g_voocphy_chip);
	}

	return cp_vbat;
}

static u8 oplus_voocphy_get_int_value(struct oplus_voocphy_manager *chip)
{
	u8 value = 0;
	if (!chip) {
		voocphy_err("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (chip->ops && chip->ops->get_int_value) {
		value = chip->ops->get_int_value(chip);
	}

	return value;
}

int oplus_voocphy_slave_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (chip->slave_ops && chip->slave_ops->hw_setting) {
		rc = chip->slave_ops->hw_setting(chip, reason);
	}

	return rc;
}

bool oplus_voocphy_set_fastchg_state(struct oplus_voocphy_manager *chip,
                                     unsigned char fastchg_state)
{
	if (chip) {
		voocphy_info("fastchg_state %d\n", fastchg_state);
		chip->fastchg_stage = fastchg_state;
	}
	return true;
}

unsigned char oplus_voocphy_get_fastchg_state(void)
{
	if (!g_voocphy_chip) {
		voocphy_info("fastchg_state %d\n", g_voocphy_chip->fastchg_stage);
		return OPLUS_FASTCHG_STAGE_1;
	}

	return g_voocphy_chip->fastchg_stage;
}

int oplus_voocphy_reset_variables(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	int batt_temp = 0;

	if (!chip) {
		voocphy_info("oplus_voocphy_reset_variables chip null!\n");
		return VOOCPHY_EFATAL;
	}

	chip->voocphy_rx_buff = 0;
	chip->voocphy_tx_buff[0] = 0;
	chip->voocphy_tx_buff[1] = 0;
	if (chip->fastchg_reactive == false) {
		chip->adapter_check_ok = false;
		chip->adapter_model_detect_count = 3;
	}
	chip->adapter_ask_check_count = 3;
	chip->fastchg_adapter_ask_cmd = VOOC_CMD_UNKNOW;
	chip->vooc2_next_cmd = VOOC_CMD_ASK_FASTCHG_ORNOT;
	chip->adapter_check_cmmu_count = 0;
	chip->adapter_rand_h = 0;
	chip->adapter_rand_l = 0;
	chip->code_id_far = 0;
	chip->code_id_local = 0xFFFF;
	chip->code_id_temp_h = 0;
	chip->code_id_temp_l = 0;
	if (chip->fastchg_reactive == false) {
		chip->adapter_model_ver = 0;
	}
	chip->adapter_model_count = 0;
	chip->ask_batt_sys = 0;
	chip->current_expect = chip->current_default;
	chip->current_max = chip->current_default;
	chip->current_ap = chip->current_default;
	chip->current_batt_temp = chip->current_default;
	chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_NORMAL;
	if (chip->fastchg_reactive == false) {
		chip->adapter_rand_start = false;
		chip->adapter_type = ADAPTER_SVOOC;
	}
	chip->adapter_mesg = 0;
	chip->fastchg_allow = false;
	chip->ask_vol_again = false;
	chip->ap_need_change_current = 0;
	chip->adjust_curr = ADJ_CUR_STEP_DEFAULT;
	chip->adjust_fail_cnt = 0;
	chip->pre_adapter_ask_cmd = VOOC_CMD_UNKNOW;
	chip->ignore_first_frame = true;
	chip->vooc_vbus_status = VOOC_VBUS_UNKNOW;
	if (chip->fastchg_reactive == false) {
		chip->vooc_head = VOOC_DEFAULT_HEAD;
	}
	chip->ask_vooc3_detect = false;
	chip->force_2a = false;
	chip->force_3a = false;
	chip->force_2a_flag = false;
	chip->force_3a_flag = false;

	chip->btb_err_first = false;
	chip->fastchg_timeout_time = FASTCHG_TOTAL_TIME; //
	chip->fastchg_3c_timeout_time = FASTCHG_3000MA_TIMEOUT;
	chip->gauge_vbatt = 0;
	chip->vbus = 0;
	chip->fastchg_batt_temp_status = BAT_TEMP_NATURAL;
	chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_INIT;
	chip->fastchg_commu_stop = false;
	chip->fastchg_check_stop = false;
	chip->fastchg_monitor_stop = false;
	chip->current_pwd = chip->current_expect;
	chip->curr_pwd_count = 10;
	chip->usb_bad_connect = false;
	chip->vooc_move_head = false;
	//chip->fastchg_err_commu = false;
	chip->ask_current_first = true;
	chip->ask_bat_model_finished = false;
	chip->fastchg_need_reset = 0;
	oplus_voocphy_set_fastchg_state(chip, OPLUS_FASTCHG_STAGE_1);
	chip->fastchg_recover_cnt = 0;
	chip->fastchg_recovering = false;

	//get initial soc
	chip->batt_soc = oplus_chg_get_ui_soc();

	if (chip->batt_fake_soc) {
		chip->batt_soc = chip->batt_fake_soc;
	}

	//get initial temp
	batt_temp = oplus_chg_get_chg_temperature();
	if (chip->batt_fake_temp) {
		batt_temp = chip->batt_fake_temp;
	}

	if (chip->batt_soc >= VOOC_LOW_SOC
	    && chip->batt_soc <= VOOC_HIGH_SOC
	    && batt_temp >= VOOC_LOW_TEMP
	    && batt_temp <= VOOC_HIGH_TEMP) {
		chip->fastchg_real_allow = true;
	} else {
		chip->fastchg_real_allow = false;
	}
	voocphy_info( "!soc:%d, temp:%d, real_allow:%d", chip->batt_soc,
	              batt_temp, chip->fastchg_real_allow);

	//note exception info
	chip->vooc_flag = 0;
	chip->int_flag = 0;
	chip->irq_total_num = 0;
	chip->irq_rcvok_num = 0;
	chip->irq_rcverr_num = 0;
	chip->irq_tx_timeout_num = 0;
	chip->irq_tx_timeout = 0;
	chip->irq_hw_timeout_num = 0;

	//default vooc head as svooc
	status = oplus_voocphy_write_mesg_mask(VOOC_INVERT_HEAD_MASK,
	                                       &chip->voocphy_tx_buff[0], chip->vooc_head);

	return status;
}

void oplus_voocphy_get_soc_and_temp_with_enter_fastchg(struct oplus_voocphy_manager *chip)
{
	int sys_curve_temp_idx = BATT_SYS_CURVE_TEMP160_TO_TEMP430;
	int batt_temp = 0;

	if (!chip) {
		voocphy_info("%s, chip null\n", __func__);
	}

	batt_temp = chip->plug_in_batt_temp;
	chip->batt_sys_curv_found = false;

	//step1: find sys curv by soc
	if (chip->batt_soc <= 50) {
		chip->batt_soc_plugin = BATT_SOC_0_TO_50;
	} else if (chip->batt_soc <= 75) {
		chip->batt_soc_plugin = BATT_SOC_50_TO_75;
	} else if (chip->batt_soc <= 85) {
		chip->batt_soc_plugin = BATT_SOC_75_TO_85;
	} else if (chip->batt_soc <= 90) {
		chip->batt_soc_plugin = BATT_SOC_85_TO_90;
	} else {
		chip->batt_soc_plugin = BATT_SOC_90_TO_100;
	}

	//step2: find sys curv by temp range
	//update batt_temp_plugin status
	if (batt_temp < chip->vooc_little_cold_temp) { /*0-5C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COLD;
		chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COLD;
		sys_curve_temp_idx = BATT_SYS_CURVE_TEMP0_TO_TEMP50;
		chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_LITTLE_COLD;
	} else if (batt_temp < chip->vooc_cool_temp) { /*5-12C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_COOL;
		chip->fastchg_batt_temp_status = BAT_TEMP_COOL;
		sys_curve_temp_idx = BATT_SYS_CURVE_TEMP50_TO_TEMP120;
		chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_COOL;
	} else if (batt_temp < chip->vooc_little_cool_temp) { /*12-16C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COOL;
		chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COOL;
		sys_curve_temp_idx = BATT_SYS_CURVE_TEMP120_TO_TEMP160;
		chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_LITTLE_COOL;
	} else if (batt_temp < chip->vooc_normal_low_temp) { /*16-25C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
		chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
		sys_curve_temp_idx = BATT_SYS_CURVE_TEMP160_TO_TEMP430;
		chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_NORMAL;
	} else {/*25C-43C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_HIGH;
		chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_HIGH;
		sys_curve_temp_idx = BATT_SYS_CURVE_TEMP160_TO_TEMP430;
		chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_NORMAL;
	}

	chip->sys_curve_temp_idx = sys_curve_temp_idx;

	voocphy_info("current_expect:%d batt_soc_plugin:%d batt_temp_plugin:%d sys_curve_index:%d",
	             chip->current_expect, chip->batt_soc_plugin,
	             chip->batt_temp_plugin, sys_curve_temp_idx);
	return;
}

int oplus_voocphy_init_vooc(struct oplus_voocphy_manager *chip)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return -1;
	}

	oplus_voocphy_reset_variables(chip);

	if (chip->ops && chip->ops->init_vooc) {
		rc = chip->ops->init_vooc(chip);
	}

	chip->batt_soc = oplus_chg_get_ui_soc();
	if (chip->batt_fake_soc) {
		chip->batt_soc = chip->batt_fake_soc;
	}
	chip->plug_in_batt_temp = oplus_chg_get_chg_temperature();
	if (chip->batt_fake_temp) {
		chip->plug_in_batt_temp = chip->batt_fake_temp;
	}

	if (chip->batt_soc >= VOOC_LOW_SOC
	    && chip->batt_soc <= VOOC_HIGH_SOC
	    && chip->plug_in_batt_temp >= VOOC_LOW_TEMP
	    && chip->plug_in_batt_temp <= VOOC_HIGH_TEMP) {
		chip->fastchg_real_allow = true;
	} else {
		chip->fastchg_real_allow = false;
	}

	chip->ask_current_first = true;
	oplus_voocphy_get_soc_and_temp_with_enter_fastchg(chip);

	voocphy_info("batt_soc %d, plug_in_batt_temp = %d\n", chip->batt_soc, chip->plug_in_batt_temp);

	return rc;
}

int oplus_voocphy_slave_init_vooc(struct oplus_voocphy_manager *chip)
{
	int rc = 0;
	if (!chip) {
		voocphy_info("oplus_voocphy_manager chip null\n");
		return -1;
	}

	if (chip->slave_ops && chip->slave_ops->init_vooc) {
		rc = chip->slave_ops->init_vooc(chip);
	}

	return 0;
}

#define GET_VBAT_PREDATA_DEFAULT (0x64 << SC8547_DATA_H_SHIFT) | 0x02;
#define EXEC_TIME_THR	1500
static int oplus_voocphy_set_txbuff(struct oplus_voocphy_manager *chip)
{
	int rc = 0;
	u16 val;
	ktime_t delta;
	unsigned long long duration;

	if (!chip) {
		voocphy_err("chip is null\n");
		return -1;
	}

	if (chip->ops && chip->ops->set_txbuff) {
		//write txbuff //0x2c
		val = (chip->voocphy_tx_buff[0] << VOOCPHY_DATA_H_SHIFT) | chip->voocphy_tx_buff[1];
		rc = chip->ops->set_txbuff(chip, val);
		if (rc < 0) {
			voocphy_info("oplus_vooc_set_txbuff failed, rc=%d.\n", rc);
			return rc;
		}
	} else {
		voocphy_info("failed: chip->ops or chip->ops->set_txbuff null\n");
	}

	//end time
	rettime = ktime_get();
	delta = ktime_sub(rettime,calltime);
	duration = (unsigned long long) ktime_to_ns(delta) >> 10;
	if (duration >= EXEC_TIME_THR) {
		//note exception for systrace
		chip->irq_tx_timeout_num++;
		voocphy_dbg("dur time %lld usecs\n", duration);
	}

	voocphy_dbg("write txbuff 0x%0x usecs %lld\n", val, duration);
	//write predata
	if (chip->fastchg_adapter_ask_cmd == VOOC_CMD_GET_BATT_VOL) {
		rc = oplus_voocphy_set_predata(chip, val);
		if (rc < 0) {
			voocphy_info("oplus_vooc_set_predata fialed, rc=%d.\n", rc);
			return rc;
		}

		voocphy_dbg("write predata 0x%0x\n", val);

		//stop fastchg check
		chip->fastchg_err_commu = false;
		chip->fastchg_to_warm = false;
	}

	return 0;
}

int oplus_voocphy_handle_ask_fastchg_ornot_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return VOOCPHY_EFATAL;
	}

	switch (chip->adapter_type) {
	case ADAPTER_VOOC20 :
		if (chip->vooc2_next_cmd != VOOC_CMD_ASK_FASTCHG_ORNOT) {
			voocphy_info("copycat adapter");
			//reset voocphy
			status = oplus_voocphy_reset_voocphy();
		}

		//reset vooc2_next_cmd
		if (chip->fastchg_allow) {
			chip->vooc2_next_cmd = VOOC_CMD_IS_VUBS_OK;
		} else {
			chip->vooc2_next_cmd = VOOC_CMD_ASK_FASTCHG_ORNOT;
		}

		//set bit3 as fastchg_allow & set circuit R
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
		                                       &chip->voocphy_tx_buff[0], VOOC2_CIRCUIT_R_L_DEF
		                                       | chip->fastchg_allow);
		status = oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
		                                       &chip->voocphy_tx_buff[1], VOOC2_CIRCUIT_R_H_DEF);
		voocphy_info("VOOC20 handle ask_fastchg_ornot");
		break;
	case ADAPTER_VOOC30 :	//lkl need modify
		//set bit3 as fastchg_allow & set circuit R
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
		                                       &chip->voocphy_tx_buff[0], NO_VOOC3_CIRCUIT_R_L_DEF
		                                       | chip->fastchg_allow);
		status = oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
		                                       &chip->voocphy_tx_buff[1], NO_VOOC3_CIRCUIT_R_H_DEF);
		voocphy_info("VOOC30 handle ask_fastchg_ornot");
		break;
	case ADAPTER_SVOOC :
		//set bit3 as fastchg_allow & set circuit R
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
		                                       &chip->voocphy_tx_buff[0], NO_VOOC2_CIRCUIT_R_L_DEF
		                                       | chip->fastchg_allow);
		status = oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
		                                       &chip->voocphy_tx_buff[1], NO_VOOC2_CIRCUIT_R_H_DEF);
		voocphy_info("SVOOC handle ask_fastchg_ornot");
		break;
	default:
		voocphy_info("should not go to here");
		break;
	}

	//for vooc3.0  and svooc adapter checksum
	chip->adapter_rand_start = true;
	chip->adapter_rand_h = 0x78; //temp set 0x78

	return status;
}

int oplus_voocphy_calcucate_identification_code(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	unsigned char code_id_temp_l = 0, code_id_temp_h = 0;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	chip->code_id_local = ((chip->adapter_rand_h << 9)
	                       |(chip->adapter_rand_l << 1));

	//set present need send mesg
	code_id_temp_l = ((((chip->adapter_rand_l >> 6) & 0x1) << TX0_DET_BIT0_SHIFT)
	                  | (((chip->adapter_rand_l >> 4) & 0x1) << TX0_DET_BIT1_SHIFT)
	                  | (((chip->adapter_rand_h >> 6) & 0x1) << TX0_DET_BIT2_SHIFT)
	                  | ((chip->adapter_rand_l & 0x1) << TX0_DET_BIT3_SHIFT)
	                  | ((chip->adapter_rand_h & 0x1) << TX0_DET_BIT4_SHIFT));
	code_id_temp_h = ((((chip->adapter_rand_h >> 4) & 0x1) << TX1_DET_BIT0_SHIFT)
	                  | (((chip->adapter_rand_l >> 2) & 0x1) << TX1_DET_BIT1_SHIFT));
	status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
	                                       &chip->voocphy_tx_buff[0], code_id_temp_l);
	status |= oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
	                                        &chip->voocphy_tx_buff[1], code_id_temp_h);

	//save next time need send mesg
	chip->code_id_temp_l = ((((chip->adapter_rand_l >> 3) & 0x1))
	                        | (((chip->adapter_rand_h >> 5) & 0x1) << TX0_DET_BIT1_SHIFT)
	                        | (((chip->adapter_rand_l >> 5) & 0x1) << TX0_DET_BIT2_SHIFT)
	                        | (((chip->adapter_rand_h >> 1) & 0x1) << TX0_DET_BIT3_SHIFT)
	                        | (((chip->adapter_rand_h >> 2) & 0x1) << TX0_DET_BIT4_SHIFT));

	chip->code_id_temp_h = ((((chip->adapter_rand_l >> 1) & 0x1))
	                        | (((chip->adapter_rand_h >> 3) & 0x1) << TX1_DET_BIT1_SHIFT));

	voocphy_info("present_code_id_temp_l:%0x, present_code_id_temp_h: %0x,"
	             "next_code_id_temp_l:%0x, next_code_id_temp_h:%0x",
	             code_id_temp_l, code_id_temp_h, chip->code_id_temp_l, chip->code_id_temp_h);
	return status;
}


int oplus_voocphy_handle_identification_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	//choose response the identification cmd
	if (chip->adapter_ask_check_count == 0
	    ||chip->adapter_rand_start == false)
		return status;

	chip->adapter_ask_check_count--; //single vooc the idenficaition should not over 3 times
	chip->adapter_check_cmmu_count = ADAPTER_CHECK_COMMU_TIMES;
	chip->adapter_rand_l = 0x56;   //lizhijie add for tmep debug

	//next adapter ask cmd should be adapater_check_commu_process
	chip->fastchg_adapter_ask_cmd = VOOC_CMD_ADAPTER_CHECK_COMMU_PROCESS;

	//calculate the identification code that need send to adapter
	status = oplus_voocphy_calcucate_identification_code(chip);

	return status;
}

int oplus_voocphy_handle_adapter_check_commu_process_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->adapter_check_cmmu_count == 5) { //send before save rand information
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
		                                       &chip->voocphy_tx_buff[0], chip->code_id_temp_l);
		status = oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
		                                       &chip->voocphy_tx_buff[1], chip->code_id_temp_h);
	} else if (chip->adapter_check_cmmu_count == 4) {
		chip->code_id_far = chip->adapter_mesg << 12;
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);

	} else if (chip->adapter_check_cmmu_count == 3) {
		chip->code_id_far = chip->code_id_far
		                    | (chip->adapter_mesg << 8);
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);

	} else if (chip->adapter_check_cmmu_count== 2) {
		chip->code_id_far = chip->code_id_far
		                    | (chip->adapter_mesg << 4);
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);

	} else if (chip->adapter_check_cmmu_count == 1) {
		chip->code_id_far = chip->code_id_far |chip->adapter_mesg;
		chip->code_id_far &= 0xfefe;
		voocphy_info( "!!!!code_id_far: 0x%0x, code_id_local",
		              chip->code_id_far, chip->code_id_local);
		if (chip->code_id_far == chip->code_id_local) {
			chip->adapter_check_ok = true;
			voocphy_info( "adapter check ok");
		}
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);
		chip->fastchg_adapter_ask_cmd = VOOC_CMD_UNKNOW; //reset cmd type
	} else {
		voocphy_info( "should not go here");
	}

	return status;
}


int oplus_voocphy_handle_tell_model_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	chip->adapter_model_count = ADAPTER_MODEL_TIMES;

	//next adapter ask cmd should be adapater_tell_model_process
	chip->fastchg_adapter_ask_cmd = VOOC_CMD_TELL_MODEL_PROCESS;

	status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT4_MASK,
	                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);
	status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT6_MASK,
	                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);

	return status;
}

int oplus_voocphy_handle_tell_model_process_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->adapter_model_count == 2) {
		chip->adapter_model_ver = (chip->adapter_mesg << 4);
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);
	} else if (chip->adapter_model_count == 1) {
		status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                                       &chip->voocphy_tx_buff[0], BIT_ACTIVE);
		chip->adapter_model_ver = chip->adapter_model_ver
		                          | chip->adapter_mesg;
		//adapter id bit7 is need to clear
		chip->adapter_model_ver &= 0x7F;
		voocphy_info( "adapter_model_ver:0x%0x",
		              chip->adapter_model_ver );
		chip->fastchg_adapter_ask_cmd = VOOC_CMD_UNKNOW; //reset cmd type
	} else {
		voocphy_info( "should not go here");
	}

	return status;
}

int oplus_voocphy_handle_ask_bat_model_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	chip->ask_batt_sys = BATT_SYS_ARRAY + 1;

	//next adapter ask cmd should be ask_batt_sys_process
	chip->fastchg_adapter_ask_cmd = VOOC_CMD_ASK_BATT_SYS_PROCESS;

	status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT6_MASK,
	                                       &chip->voocphy_tx_buff[0], BATT_SYS_VOL_CUR_PAIR);

	return status;
}

int oplus_voocphy_handle_ask_bat_model_process_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	unsigned char temp_data_l = 0;
	unsigned char temp_data_h = 0;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->ask_batt_sys) {
		if (chip->adapter_type == ADAPTER_SVOOC) {  //svooc battery curve
			temp_data_l = (svooc_batt_sys_curve[chip->ask_batt_sys - 1][0] << 0)
			              | (svooc_batt_sys_curve[chip->ask_batt_sys - 1][1] << 1)
			              | (svooc_batt_sys_curve[chip->ask_batt_sys - 1][2] << 2)
			              | (svooc_batt_sys_curve[chip->ask_batt_sys - 1][3] << 3)
			              | (svooc_batt_sys_curve[chip->ask_batt_sys - 1][4] << 4);

			temp_data_h = (svooc_batt_sys_curve[chip->ask_batt_sys - 1][5] << 0)
			              | (svooc_batt_sys_curve[chip->ask_batt_sys - 1][6] << 1);
		} else if (chip->adapter_type == ADAPTER_VOOC30) {	//for cp
			temp_data_l = (vooc_batt_sys_curve[chip->ask_batt_sys - 1][0] << 0)
			              | (vooc_batt_sys_curve[chip->ask_batt_sys - 1][1] << 1)
			              | (vooc_batt_sys_curve[chip->ask_batt_sys - 1][2] << 2)
			              | (vooc_batt_sys_curve[chip->ask_batt_sys - 1][3] << 3)
			              | (vooc_batt_sys_curve[chip->ask_batt_sys - 1][4] << 4);

			temp_data_h = (vooc_batt_sys_curve[chip->ask_batt_sys - 1][5] << 0)
			              | (vooc_batt_sys_curve[chip->ask_batt_sys - 1][6] << 1);
		} else {
			voocphy_info( "error adapter type when write batt_sys_curve");
		}

		oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
		                              &chip->voocphy_tx_buff[0], temp_data_l);
		oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
		                              &chip->voocphy_tx_buff[1], temp_data_h);
		if (chip->ask_batt_sys == 1) {
			if (chip->fastchg_real_allow) {
				chip->fastchg_allow = true; //batt allow dchg
				chip->fastchg_start = true; //fastchg_start true
				chip->fastchg_to_warm = false;
				chip->fastchg_dummy_start = false;
				oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_PRESENT);
				voocphy_info( "fastchg_real_allow = true");
			} else {
				chip->fastchg_allow = false;
				chip->fastchg_start = false;
				chip->fastchg_to_warm = false;
				chip->fastchg_dummy_start = true;
				oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_DUMMY_START);
				voocphy_info( "fastchg_dummy_start and reset voocphy");
			}
			chip->ask_bat_model_finished = true;
			chip->fastchg_adapter_ask_cmd = VOOC_CMD_UNKNOW; //reset cmd
		}
	} else {
		voocphy_info( "should not go here");
	}

	return status;
}

int oplus_voocphy_vbus_vbatt_detect(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	int vbus_vbatt = 0;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	vbus_vbatt = chip->vbus = chip->cp_vbus;

	if (chip->adapter_type == ADAPTER_SVOOC)
		vbus_vbatt = chip->vbus - chip->gauge_vbatt * 2;
	else
		vbus_vbatt = chip->vbus -chip->gauge_vbatt;

	if (vbus_vbatt  < 100) {
		chip->vooc_vbus_status = VOOC_VBUS_LOW;
	} else if (vbus_vbatt > 200) {
		chip->vooc_vbus_status = VOOC_VBUS_HIGH;
	} else {
		chip->vooc_vbus_status = VOOC_VBUS_NORMAL;
	}
	chip->vbus_vbatt = vbus_vbatt;

	voocphy_info( "!!vbus=%d, vbatt=%d, vbus_vbatt=%d, vooc_vbus_status=%d",
	              chip->vbus, chip->gauge_vbatt, vbus_vbatt, chip->vooc_vbus_status);
	return status;
}

int oplus_voocphy_handle_is_vbus_ok_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->adapter_type == ADAPTER_VOOC20) {
		if (chip->vooc2_next_cmd != VOOC_CMD_IS_VUBS_OK) {
			status = oplus_voocphy_reset_voocphy();
		} else {
			oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_MASK,
			                              &chip->voocphy_tx_buff[0], BIT_ACTIVE);
		}
	}

	//detect vbus status
	status = oplus_voocphy_vbus_vbatt_detect(chip);
	if (chip->vooc_vbus_status == VOOC_VBUS_HIGH) {  //vbus high
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT4_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_ACTIVE);
	} else if (chip->vooc_vbus_status == VOOC_VBUS_NORMAL) {  //vbus ok
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT4_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_ACTIVE);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_ACTIVE);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_SLEEP);
	} else {   //vbus low
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_ACTIVE);
	}

	if (chip->vooc_vbus_status == VOOC_VBUS_NORMAL) { //vbus-vbatt = ok
		oplus_voocphy_set_chg_enable(chip, true);
	}

	if (chip->vooc_vbus_status == VOOC_VBUS_NORMAL) {
		switch (chip->adapter_type) {
		case ADAPTER_VOOC20 :
			voocphy_info("ADAPTER_VOOC20\n");
			chip->vooc2_next_cmd = VOOC_CMD_ASK_CURRENT_LEVEL;
			oplus_voocphy_monitor_start(chip);
			//oplus_vooc_hardware_monitor_start();
			break;
		case ADAPTER_VOOC30 :
		case ADAPTER_SVOOC :
			oplus_voocphy_monitor_start(chip);
			//oplus_vooc_hardware_monitor_start();
			break;
		default:
			voocphy_info( " should not go to here");
			break;
		}
	}

	return status;
}

void oplus_voocphy_choose_batt_sys_curve(struct oplus_voocphy_manager *chip)
{
	int sys_curve_temp_idx = 0;
	int idx = 0;
	int convert_ibus = 0;
	struct batt_sys_curves *batt_sys_curv_by_tmprange = NULL;
	struct batt_sys_curve *batt_sys_curve = NULL;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}

	if (BATT_SOC_90_TO_100 != chip->batt_soc_plugin) {
		if (chip->batt_sys_curv_found == false) {
			//step1: find sys curv by soc
			if (chip->adapter_type == ADAPTER_SVOOC) {
				if (chip->batt_soc_plugin == BATT_SOC_0_TO_50) {
					chip->batt_sys_curv_by_soc = svooc_curves_soc0_2_50;
				} else if (chip->batt_soc_plugin == BATT_SOC_50_TO_75) {
					chip->batt_sys_curv_by_soc = svooc_curves_soc50_2_75;
				} else if (chip->batt_soc_plugin == BATT_SOC_75_TO_85) {
					chip->batt_sys_curv_by_soc = svooc_curves_soc75_2_85;
				} else if (chip->batt_soc_plugin == BATT_SOC_85_TO_90) {
					chip->batt_sys_curv_by_soc = svooc_curves_soc85_2_90;
				} else {
					//do nothing
				}
			} else {
				if (chip->batt_soc_plugin == BATT_SOC_0_TO_50) {
					chip->batt_sys_curv_by_soc = vooc_curves_soc0_2_50;
				} else if (chip->batt_soc_plugin == BATT_SOC_50_TO_75) {
					chip->batt_sys_curv_by_soc = vooc_curves_soc50_2_75;
				} else if (chip->batt_soc_plugin == BATT_SOC_75_TO_85) {
					chip->batt_sys_curv_by_soc = vooc_curves_soc75_2_85;
				} else if (chip->batt_soc_plugin == BATT_SOC_85_TO_90) {
					chip->batt_sys_curv_by_soc = vooc_curves_soc85_2_90;
				} else {
					//do nothing
				}
			}

			//step2: find sys curv by temp range
			sys_curve_temp_idx = chip->sys_curve_temp_idx;

			//step3: note sys curv location by temp range, for example BATT_SYS_CURVE_TEMP160_TO_TEMP430
			chip->batt_sys_curv_by_tmprange = &(chip->batt_sys_curv_by_soc[sys_curve_temp_idx]);
			chip->cur_sys_curv_idx = 0;

			//step4: note init current_expect and current_max
			if (chip->external_gauge_support)
				chip->gauge_vbatt = oplus_gauge_get_batt_mvolts();
			else
				chip->gauge_vbatt = oplus_voocphy_get_cp_vbat();

			batt_sys_curv_by_tmprange = chip->batt_sys_curv_by_tmprange;
			for (idx = 0; idx <= batt_sys_curv_by_tmprange->sys_curv_num - 1; idx++) {	//big -> small
				batt_sys_curve = &(batt_sys_curv_by_tmprange->batt_sys_curve[idx]);
				convert_ibus = batt_sys_curve->target_ibus * 100;
				voocphy_info("!batt_sys_curve[%d] gauge_vbatt,exit,target_vbat,cp_ichg,target_ibus:[%d, %d, %d, %d, %d], \n", idx,
				             chip->gauge_vbatt, batt_sys_curve->exit, batt_sys_curve->target_vbat, chip->cp_ichg, convert_ibus);
				if (batt_sys_curve->exit == false) {
					if (chip->gauge_vbatt <= batt_sys_curve->target_vbat) {
						chip->cur_sys_curv_idx = idx;
						chip->batt_sys_curv_found = true;
						voocphy_info("! found batt_sys_curve first idx[%d]\n", idx);
						break;
					}
				} else {
					//exit fastchg
					voocphy_info("! not found batt_sys_curve first\n");
				}
			}
			if (chip->batt_sys_curv_found) {
				voocphy_info("! found batt_sys_curve idx idx[%d]\n", idx);
				chip->current_expect = chip->batt_sys_curv_by_tmprange->batt_sys_curve[idx].target_ibus;
				chip->current_max = chip->current_expect;
				chip->batt_sys_curv_by_tmprange->batt_sys_curve[idx].chg_time = 0;
			} else {
				chip->current_expect = chip->batt_sys_curv_by_tmprange->batt_sys_curve[0].target_ibus;
				chip->current_max = chip->current_expect;
				chip->batt_sys_curv_by_tmprange->batt_sys_curve[0].chg_time = 0;
			}
		}
	} else {
		//do nothing
	}

	voocphy_info("current_max:%d current_expect:%d batt_soc_plugin:%d batt_temp_plugin:%d sys_curve_index:%d ibus:%d ",
	             chip->current_max, chip->current_expect, chip->batt_soc_plugin,
	             chip->batt_temp_plugin, sys_curve_temp_idx, chip->cp_ichg);
}

int oplus_voocphy_handle_ask_current_level_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	//unsigned char temp_data_l = 0, temp_data_h = 0;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->ask_current_first) {
		oplus_voocphy_choose_batt_sys_curve(chip);
		chip->ask_current_first = false;
	}

	if (chip->adapter_type == ADAPTER_VOOC20) {
		if (chip->vooc2_next_cmd != VOOC_CMD_ASK_CURRENT_LEVEL) {
			status = oplus_voocphy_reset_voocphy();
		} else {
			 //if (chip->batt_soc <= VOOC_SOC_HIGH) { // soc_high is not
			 if (chip->voocphy_dual_cp_support) {	//10v3a design is not need to go here
				//set current 3.75A
				oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_MASK,
					&chip->voocphy_tx_buff[1], BIT_SLEEP); //bit 8
				oplus_voocphy_write_mesg_mask(TX1_DET_BIT1_MASK,
					&chip->voocphy_tx_buff[1], BIT_SLEEP); //bit 9
				oplus_voocphy_write_mesg_mask(TX0_DET_BIT7_MASK,
					&chip->voocphy_tx_buff[0], BIT_ACTIVE); //bit 7
			 } else {
				//do nothing, tx_buff keep default zero.
				//set current 3.0A
			}
			chip->vooc2_next_cmd = VOOC_CMD_GET_BATT_VOL; //vooc2_next_cmd;
		}
	} else { //svooc&vooc30
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_MASK,
		                              &chip->voocphy_tx_buff[0], (chip->current_expect >> 6) & 0x1);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT4_MASK,
		                              &chip->voocphy_tx_buff[0], (chip->current_expect >> 5) & 0x1);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT5_MASK,
		                              &chip->voocphy_tx_buff[0], (chip->current_expect >> 4) & 0x1);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT6_MASK,
		                              &chip->voocphy_tx_buff[0], (chip->current_expect >> 3) & 0x1);
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT7_MASK,
		                              &chip->voocphy_tx_buff[0], (chip->current_expect >> 2) & 0x1);
		oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_MASK,
		                              &chip->voocphy_tx_buff[1], (chip->current_expect >> 1) & 0x1);
		oplus_voocphy_write_mesg_mask(TX1_DET_BIT1_MASK,
		                              &chip->voocphy_tx_buff[1], (chip->current_expect >> 0) & 0x1);

		chip->ap_need_change_current = 0; //clear need change flag
		chip->current_pwd = chip->current_expect * 100 + 300;
		chip->curr_pwd_count = 10;
		if (chip->adjust_curr == ADJ_CUR_STEP_REPLY_VBATT) {	//step1
			chip->adjust_curr = ADJ_CUR_STEP_SET_CURR_DONE;		//step2
		}
		voocphy_info( "!! set expect_current:%d", chip->current_expect);
	}
	return status;
}

int oplus_voocphy_non_vooc20_handle_get_batt_vol_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	unsigned int vbatt = 0;
	unsigned char data_temp_l = 0, data_temp_h = 0;
	unsigned int vbatt1;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->btb_temp_over) {
		if (!chip->btb_err_first) { //1st->b'1100011:4394mv , 2nd->b'0001000:3484mv
			data_temp_l = 0x03; //bit 0 0011
			data_temp_h = 0x3; //bit 11
			chip->btb_err_first = true;
		} else {
			data_temp_l = 0x08; //bit 0 1000
			data_temp_h = 0x0; //bit 00
			chip->btb_err_first = false;
		}
	} else if (chip->ap_need_change_current && chip->adjust_fail_cnt <= 3) {
		data_temp_l = 0x1f; //bit 1 1111
		data_temp_h = 0x3; //bit 11
		chip->ap_need_change_current--;
		cpuboost_charge_event(CPU_CHG_FREQ_STAT_UP);
		#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
		pm_qos_update_request(&pm_qos_req, 400);
		#endif
		trace_oplus_tp_sched_change_ux(1, task_cpu(current));
		trace_oplus_tp_sched_change_ux(0, task_cpu(current));
		chip->adjust_curr = ADJ_CUR_STEP_REPLY_VBATT;
		voocphy_info( " ap_need_change_current adjust_fail_cnt[%d]\n", chip->adjust_fail_cnt);
	} else {
		/*calculate R*/
		if (chip->adapter_type == ADAPTER_SVOOC) {	//lkl need modify
			vbatt1 = (chip->cp_vbus/2) - (signed)(chip->cp_vsys - chip->gauge_vbatt)/4 - (chip->cp_ichg * 75)/1000;
		} else {
			vbatt1 = chip->gauge_vbatt;
		}

		if (vbatt1 < VBATT_BASE_FOR_ADAPTER) {
			vbatt = 0;
		} else {
			vbatt = (vbatt1 - VBATT_BASE_FOR_ADAPTER) / VBATT_DIV_FOR_ADAPTER;
		}

		data_temp_l =  ((vbatt >> 6) & 0x1) | (((vbatt >> 5) & 0x1) << 1)
		               | (((vbatt >> 4) & 0x1) << 2) | (((vbatt >> 3) & 0x1) << 3)
		               | (((vbatt >> 2) & 0x1) << 4);
		data_temp_h = ((vbatt >> 1) & 0x1) | (((vbatt >> 0) & 0x1) << 1);
	}

	voocphy_info( "gauge_vbatt=%d, vbatt=0x%0x, "
	              "data_temp_l=0x%0x, data_temp_h=0x%0x cp:%d %d %d %d",
	              chip->gauge_vbatt, vbatt,data_temp_l, data_temp_h, chip->cp_vsys,
	              chip->cp_ichg,chip->cp_vbus, vbatt1);

	oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
	                              &chip->voocphy_tx_buff[0], data_temp_l);
	oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
	                              &chip->voocphy_tx_buff[1], data_temp_h);
	return status;
}

int oplus_voocphy_vooc20_handle_get_batt_vol_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	unsigned int vbatt = 0;
	unsigned char data_temp_l = 0, data_temp_h = 0;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	if (chip->vooc2_next_cmd != VOOC_CMD_GET_BATT_VOL)
		status = oplus_voocphy_reset_voocphy();

	oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_MASK,
	                              &chip->voocphy_tx_buff[0], chip->fastchg_allow); //need refresh bit3

	if (chip->batt_soc > VOOC_SOC_HIGH &&  !chip->ask_vol_again) {
		data_temp_l = chip->fastchg_allow | 0x0e;
		data_temp_h = 0x01;
		chip->ask_vol_again = true;
	} else if (chip->gauge_vbatt > 3391) {

		vbatt = (chip->gauge_vbatt - 3392) / 16;		//+40mv
		if(vbatt > 61)	 {	// b'111110,vbatt >= 4384mv
			vbatt = 61;
		}
		if (chip->force_2a && (!chip->force_2a_flag)
		    && (vbatt < FORCE2A_VBATT_REPORT)) {
			vbatt = FORCE2A_VBATT_REPORT;		//adapter act it as 4332mv
			chip->force_2a_flag = true;
		} else if (chip->force_3a && (!chip->force_3a_flag)
		           && (vbatt < FORCE3A_VBATT_REPORT)) {
			vbatt = FORCE3A_VBATT_REPORT;		//adapter act it as 4316mv
			chip->force_3a_flag = true;
		}
		data_temp_l = (((vbatt >> 5) & 0x1) << 1) | (((vbatt >> 4) & 0x1) << 2)
		              | (((vbatt >> 3) & 0x1) << 3) | (((vbatt >> 2) & 0x1) << 4)
		              | chip->fastchg_allow;
		data_temp_h = ((vbatt & 0x1) << 1) | ((vbatt >> 1) & 0x1);

		//reply adapter when btb_temp_over
		if (chip->btb_temp_over) {
			if (!chip->btb_err_first) {
				data_temp_l = chip->fastchg_allow | 0x1e;
				data_temp_h = 0x01;
				chip->btb_err_first = true;
			} else {
				data_temp_l = chip->fastchg_allow | 0x10;
				data_temp_h = 0x02;
				chip->btb_err_first = false;
			}
		}
	} else if (chip->gauge_vbatt < 3000) {
		oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_MASK,
		                              &chip->voocphy_tx_buff[0], BIT_SLEEP);
		voocphy_info( "vbatt too low at vooc2 mode");
	}

	voocphy_info( "data_temp_l=0x%0x, data_temp_h=0x%0x",
	              data_temp_l, data_temp_h);
	oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
	                              &chip->voocphy_tx_buff[0], data_temp_l);
	oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
	                              &chip->voocphy_tx_buff[1], data_temp_h);

	chip->vooc2_next_cmd = VOOC_CMD_GET_BATT_VOL;

	return status;
}

int oplus_voocphy_handle_get_batt_vol_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	switch (chip->adapter_type) {
	case ADAPTER_VOOC30 :
	case ADAPTER_SVOOC :
		status = oplus_voocphy_non_vooc20_handle_get_batt_vol_cmd(chip);
		break;
	case ADAPTER_VOOC20 :
		status = oplus_voocphy_vooc20_handle_get_batt_vol_cmd(chip);
		break;
	default :
		voocphy_info( " should not go to here");
		break;
	}

	return status;
}

int oplus_voocphy_handle_null_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	//status = oplus_voocphy_reset_voocphy();
	voocphy_info("null cmd");

	return status;
}

int oplus_voocphy_handle_tell_usb_bad_cmd(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	//status = oplus_voocphy_reset_voocphy();
	oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_BAD_CONNECTED);
	chip->usb_bad_connect = true;
	chip->adapter_check_ok = 0;
	chip->adapter_model_ver = 0;
	chip->ask_batt_sys = 0;
	chip->adapter_check_cmmu_count = 0;

	voocphy_info( " usb bad connect");
	return status;
}

int oplus_voocphy_handle_ask_ap_status(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n");
		return VOOCPHY_EFATAL;
	}

	//read vbus&vbatt need suspend charger

	return status;
}


int oplus_voocphy_reply_adapter_mesg(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return VOOCPHY_EFATAL;
	}

	//memset set tx_buffer head & tx_buffer data
	status = oplus_voocphy_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
	                                       &chip->voocphy_tx_buff[0], 0);
	status |= oplus_voocphy_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
	                                        &chip->voocphy_tx_buff[1], 0);
	status = oplus_voocphy_write_mesg_mask(VOOC_INVERT_HEAD_MASK,
	                                       &chip->voocphy_tx_buff[0], chip->vooc_head);

	//handle special status
	if ((chip->fastchg_adapter_ask_cmd != VOOC_CMD_ADAPTER_CHECK_COMMU_PROCESS)
	    && (chip->fastchg_adapter_ask_cmd != VOOC_CMD_ASK_BATT_SYS_PROCESS)
	    && (chip->fastchg_adapter_ask_cmd != VOOC_CMD_TELL_MODEL_PROCESS)) {
		chip->fastchg_adapter_ask_cmd = chip->adapter_mesg;
	}

	voocphy_info("rx::0x%0x adapter_ask:0x%0x",chip->voocphy_rx_buff, chip->fastchg_adapter_ask_cmd);

	//handle the adaper request mesg
	switch (chip->fastchg_adapter_ask_cmd) {
	case VOOC_CMD_ASK_FASTCHG_ORNOT:
		status = oplus_voocphy_handle_ask_fastchg_ornot_cmd(chip);
		break;
	case VOOC_CMD_IDENTIFICATION:
		status = oplus_voocphy_handle_identification_cmd(chip);
		break;
	case VOOC_CMD_ADAPTER_CHECK_COMMU_PROCESS:
		status = oplus_voocphy_handle_adapter_check_commu_process_cmd(chip);
		break;
	case VOOC_CMD_TELL_MODEL:
		status = oplus_voocphy_handle_tell_model_cmd(chip);
		break;
	case VOOC_CMD_TELL_MODEL_PROCESS:
		status = oplus_voocphy_handle_tell_model_process_cmd(chip);
		break;
	case VOOC_CMD_ASK_BAT_MODEL:
		status = oplus_voocphy_handle_ask_bat_model_cmd(chip);
		break;
	case VOOC_CMD_ASK_BATT_SYS_PROCESS:
		status = oplus_voocphy_handle_ask_bat_model_process_cmd(chip);
		break;
	case VOOC_CMD_IS_VUBS_OK:
		status = oplus_voocphy_handle_is_vbus_ok_cmd(chip);
		break;
	case VOOC_CMD_ASK_CURRENT_LEVEL:
		status = oplus_voocphy_handle_ask_current_level_cmd(chip);
		break;
	case VOOC_CMD_GET_BATT_VOL:
		status = oplus_voocphy_handle_get_batt_vol_cmd(chip);
		break;
	case VOOC_CMD_NULL:
		status = oplus_voocphy_handle_null_cmd(chip);
		break;
	case VOOC_CMD_TELL_USB_BAD:
		status = oplus_voocphy_handle_tell_usb_bad_cmd(chip);
		break;
	case VOOC_CMD_ASK_AP_STATUS :
		status = oplus_voocphy_handle_ask_ap_status(chip);
		break;
	default:
		voocphy_info("cmd not support, default handle");
		break;
	}

	return status;
}

void oplus_voocphy_convert_tx_bit0_to_bit9(struct oplus_voocphy_manager *chip)
{
	unsigned int i = 0;
	unsigned int src_data = 0, des_data = 0, temp_data = 0;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}

	src_data = chip->voocphy_tx_buff[1] & TX1_DET_BIT0_TO_BIT1_MASK;
	src_data = (src_data << 8) | (chip->voocphy_tx_buff[0]);


	for (i=0; i<10; i++) {
		temp_data = (src_data >> i) & 0x1;
		des_data = des_data | (temp_data << (10 - i - 1));
	}

	chip->voocphy_tx_buff[1] = (des_data  >> 8) & TX1_DET_BIT0_TO_BIT1_MASK;
	chip->voocphy_tx_buff[0] = (des_data & 0xff);
};

void oplus_voocphy_convert_tx_bit0_to_bit9_for_move_model(struct oplus_voocphy_manager *chip)
{
	unsigned int i = 0;
	unsigned int src_data = 0, des_data = 0, temp_data = 0;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}

	src_data = chip->voocphy_tx_buff[1] & TX1_DET_BIT0_TO_BIT1_MASK;
	src_data = (src_data << 8) | (chip->voocphy_tx_buff[0]);


	for (i=0; i<10; i++) {
		temp_data = (src_data >> i) & 0x1;
		des_data = des_data | (temp_data << (10 - i - 1));
	}

	des_data = (des_data >> 1);
	chip->voocphy_tx_buff[1] = (des_data  >> 8) & TX1_DET_BIT0_TO_BIT1_MASK;
	chip->voocphy_tx_buff[0] = (des_data & 0xff);
};


int oplus_voocphy_send_mesg_to_adapter(struct oplus_voocphy_manager *chip)
{
	int ret;
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return VOOCPHY_EFATAL;
	}

	if (chip->adapter_check_cmmu_count) {
		chip->adapter_check_cmmu_count--;
	} else if (chip->adapter_model_count) {
		chip->adapter_model_count--;
	} else if (chip->ask_batt_sys) {
		chip->ask_batt_sys--;
		voocphy_info("ask_batt_sys %d\n", chip->ask_batt_sys);
	}

	voocphy_info("before convert tx_buff[0]=0x%0x, tx_buff[1]=0x%0x",
	             chip->voocphy_tx_buff[0], chip->voocphy_tx_buff[1]);

	//convert the tx_buff
	if (!chip->vooc_move_head) {
		oplus_voocphy_convert_tx_bit0_to_bit9(chip);
	} else {
		oplus_voocphy_convert_tx_bit0_to_bit9_for_move_model(chip);
	}
	chip->vooc_move_head = false;

	voocphy_info("tx_buff[0]=0x%0x, tx_buff[1]=0x%0x adapter_ask_cmd = 0x%0x adjust_curr = %d",
	             chip->voocphy_tx_buff[0], chip->voocphy_tx_buff[1], chip->fastchg_adapter_ask_cmd, chip->adjust_curr);

	//send tx_buff 10 bits to adapter
	ret = oplus_voocphy_set_txbuff(chip);
	//deal error when fastchg adjust current write txbuff
	if (VOOC_CMD_ASK_CURRENT_LEVEL == chip->fastchg_adapter_ask_cmd && ADJ_CUR_STEP_SET_CURR_DONE == chip->adjust_curr) {
		voocphy_info("adjust curr ADJ_CUR_STEP_SET_CURR_DONE\n");
	} else {
		if (VOOC_CMD_GET_BATT_VOL == chip->fastchg_adapter_ask_cmd && ADJ_CUR_STEP_DEFAULT == chip->adjust_curr) {
			cpuboost_charge_event(CPU_CHG_FREQ_STAT_AUTO);
			#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
			pm_qos_update_request(&pm_qos_req, PM_QOS_DEFAULT_VALUE);
			#endif
			//trace_oplus_tp_sched_change_ux(1, task_cpu(current));
			//trace_oplus_tp_sched_change_ux(0, task_cpu(current));
		}
	}

	if (ret < 0)
		status =  VOOCPHY_EFATAL;
	else
		status =  VOOCPHY_SUCCESS;

	return status;
}

int oplus_voocphy_get_mesg_from_adapter(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	unsigned char vooc_head = 0;
	unsigned char vooc_move_head = 0;

	if (!chip) {
		voocphy_info("%s, chip null\n", __func__);
		return VOOCPHY_EFATAL;
	}

	//handle the frame head & adapter_data
	vooc_head = (chip->voocphy_rx_buff & VOOC_HEAD_MASK) >> VOOC_HEAD_SHIFT;

	//handle the shift frame head
	vooc_move_head = (chip->voocphy_rx_buff & VOOC_MOVE_HEAD_MASK)
	                 >> VOOC_MOVE_HEAD_SHIFT;

	if (vooc_head == SVOOC_HEAD || vooc_move_head == SVOOC_HEAD) {
		if (vooc_head == SVOOC_HEAD) {
			voocphy_info("SVOOC_HEAD");
		} else {
			voocphy_info("SVOOC_MOVE_HEAD");
			chip->vooc_move_head = true;
		}
		chip->vooc_head = SVOOC_INVERT_HEAD;
		chip->adapter_type = ADAPTER_SVOOC; //detect svooc adapter
	} else if (vooc_head == VOOC3_HEAD || vooc_move_head == VOOC3_HEAD) {
		if (vooc_head == VOOC3_HEAD) {
			voocphy_info("VOOC30_HEAD");
		} else {
			voocphy_info("VOOC30_MOVE_HEAD");
			chip->vooc_move_head = true;
		}
		chip->vooc_head = VOOC3_INVERT_HEAD;
		chip->adapter_type = ADAPTER_VOOC30; //detect vooc30 adapter
	} else if (vooc_head == VOOC2_HEAD ||vooc_move_head == VOOC2_HEAD) {
		if (vooc_move_head == VOOC2_HEAD) {
			voocphy_info("VOOC20_MOVE_HEAD");
			chip->vooc_move_head = true;
		}

		if (chip->adapter_model_detect_count) {
			chip->adapter_model_detect_count--; //detect three times
		} else {	//adapter_model_detect_count
			chip->adapter_type = ADAPTER_VOOC30; //detect svooc adapter
#if	1	//def VOOC_ACCESS_CTRL_MOS
			//retry detect vooc30
			if (!chip->ask_vooc3_detect) {
				chip->ask_vooc3_detect = true;
				chip->adapter_model_detect_count =3;
				chip->fastchg_allow = false;
				chip->vooc_head = VOOC3_INVERT_HEAD; //set vooc3 head to detect
			} else {
				voocphy_info("VOOC20_HEAD");
				chip->adapter_type = ADAPTER_VOOC20; //old vooc2.0 adapter
				chip->vooc_head = VOOC2_INVERT_HEAD;
				chip->adapter_model_ver = POWER_SUPPLY_USB_SUBTYPE_VOOC;
				if (chip->fastchg_real_allow) {
					chip->fastchg_allow = true; //allow fastchg at old vooc2.0
					chip->fastchg_start = true; //fastchg_start true
					chip->fastchg_to_warm = false;
					chip->fastchg_dummy_start = false;
					oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_PRESENT);
					voocphy_info("fastchg_real_allow = true");
				} else {
					chip->fastchg_allow = false;
					chip->fastchg_start = false;
					chip->fastchg_to_warm = false;
					chip->fastchg_dummy_start = true;
					oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_DUMMY_START);
					voocphy_info("fastchg_dummy_start and reset voocphy");
				}
			}
#else
			voocphy_info("POWER_BANK_MODE");
			chip->vooc_head = VOOC2_INVERT_HEAD;
			chip->adapter_type = ADAPTER_VOOC20; //power_bank mode
			chip->fastchg_allow = false; //power_bank mode need set fastchg_allow false
#endif
		}
	} else {
		voocphy_info("unknow frame head, reset voocphy & fastchg rerun check");
		oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_ERR_COMMU);
		return VOOCPHY_EUNSUPPORTED;
	}

	if (!chip->vooc_move_head) {
		status = oplus_voocphy_read_mesg_mask(VOOC_MESG_READ_MASK,
		                                      chip->voocphy_rx_buff, &chip->adapter_mesg);
	} else {
		status = oplus_voocphy_read_mesg_mask(VOOC_MESG_MOVE_READ_MASK,
		                                      chip->voocphy_rx_buff, &chip->adapter_mesg);
	}

	if (chip->fastchg_reactive == true
	    && chip->adapter_mesg == VOOC_CMD_ASK_FASTCHG_ORNOT) {
		chip->fastchg_reactive = false;
	}

	if (chip->fastchg_reactive == true) {
		voocphy_info("(fastchg_dummy_start or err commu) && adapter_mesg != 0x04");
		status = VOOCPHY_EUNSUPPORTED;
	}

	voocphy_info("adapter_mesg=0x%0x", chip->adapter_mesg);
	return status;
}

int oplus_voocphy_adapter_commu_with_voocphy(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	if (!chip) {
		voocphy_info("%s, chip null\n", __func__);
		return -1;
	}

	//get data from adapter
	status = oplus_voocphy_get_mesg_from_adapter(chip);
	if (status != VOOCPHY_SUCCESS) {
		voocphy_info(" oplus_voocphy_get_mesg_from_adapter fail");
		goto EndError;
	}

	//calculate mesg for voocphy need reply to adapter
	status = oplus_voocphy_reply_adapter_mesg(chip);
	if (status != VOOCPHY_SUCCESS) {
		voocphy_info(" oplus_vooc_process_mesg fail");
		goto EndError;
	}

	//send data to adapter
	status = oplus_voocphy_send_mesg_to_adapter(chip);
	if (status != VOOCPHY_SUCCESS) {
		voocphy_info(" oplus_voocphy_send_mesg_to_adapter fail");
	}

EndError:
	return status;
}

bool oplus_voocphy_ap_allow_fastchg(struct oplus_voocphy_manager *chip)
{
	bool ap_allow_fastchg = false;

	if (!chip) {
		voocphy_info("%s, chip null\n", __func__);
		return false;
	}

	if (chip->fastchg_adapter_ask_cmd == VOOC_CMD_ASK_FASTCHG_ORNOT) {
		ap_allow_fastchg = (chip->voocphy_tx_buff[0] & AP_ALLOW_FASTCHG);
		if (ap_allow_fastchg) {
			voocphy_info("fastchg_stage OPLUS_FASTCHG_STAGE_2\n");
			oplus_voocphy_set_fastchg_state(chip, OPLUS_FASTCHG_STAGE_2);
		}
		return ap_allow_fastchg;
	}
	return false;
}

int oplus_voocphy_set_is_vbus_ok_predata (struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("%s, chip null\n", __func__);
		return VOOCPHY_EFATAL;
	}

	if (chip->adapter_type == ADAPTER_SVOOC) {
		if (chip->ask_bat_model_finished) {
			chip->ask_bat_model_finished = false;
			oplus_voocphy_set_predata(chip, OPLUS_IS_VUBS_OK_PREDATA_SVOOC);
		}
	} else if (chip->adapter_type == ADAPTER_VOOC30) {
		if (chip->ask_bat_model_finished) {
			chip->ask_bat_model_finished = false;
			oplus_voocphy_set_predata(chip, OPLUS_IS_VUBS_OK_PREDATA_VOOC30);
		}
	} else if (chip->adapter_type == ADAPTER_VOOC20) {
		oplus_voocphy_set_predata(chip, OPLUS_IS_VUBS_OK_PREDATA_VOOC20);
	} else {
		voocphy_info("adapter_type error !!!\n");
		return -1;
	}
	return 0;
}

static int oplus_voocphy_get_write_txbuff_error(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("chip null\n");
		return 0;
	}

	return chip->vooc_flag & (1 << 3);
}

irqreturn_t oplus_voocphy_interrupt_handler(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	chip->irq_total_num++;

	//for flash led
	if (chip->fastchg_need_reset) {
		voocphy_info("fastchg_need_reset%d\n");
		chip->fastchg_need_reset = 0;
		oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_USER_EXIT_FASTCHG);
		goto handle_done;
	}

	oplus_voocphy_get_adapter_info(chip);

	if (VOOC_CMD_ASK_CURRENT_LEVEL == chip->pre_adapter_ask_cmd && ADJ_CUR_STEP_SET_CURR_DONE == chip->adjust_curr) {
		if (oplus_voocphy_get_write_txbuff_error(chip)) {
			chip->ap_need_change_current = 5;
			chip->adjust_fail_cnt++;
			voocphy_info("adjust curr wtxbuff fail %d times\n", chip->adjust_fail_cnt);
		} else {
			cpuboost_charge_event(CPU_CHG_FREQ_STAT_AUTO);
			#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
			pm_qos_update_request(&pm_qos_req, PM_QOS_DEFAULT_VALUE);
			#endif
			//trace_oplus_tp_sched_change_ux(1, task_cpu(current));
			//trace_oplus_tp_sched_change_ux(0, task_cpu(current));
			voocphy_info("adjust cpu to default");
			chip->adjust_fail_cnt = 0;
			chip->adjust_curr = ADJ_CUR_STEP_DEFAULT;
		}
	}

	if (chip->vooc_flag == 0xF) {
		//do nothing
		voocphy_info("!RX_START_FLAG & TXDATA_WR_FAIL_FLAG occured do nothing");
	} else if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {	//rxdata recv done
		//feed soft monitor watchdog
		if (!chip->fastchg_commu_stop) {
			status = oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_COMMU, VOOC_COMMU_EVENT_TIME);
			if (status != VOOCPHY_SUCCESS) {
				voocphy_info("stop commu timer fail");
			}
			status = oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_COMMU,VOOC_COMMU_EVENT_TIME);
			if (status != VOOCPHY_SUCCESS) {
				voocphy_info("restart commu timer fail");
			}
		}

		chip->irq_rcvok_num++;
		//chip->voocphy_rx_buff = __sc->vooc_rxdata;
		oplus_voocphy_adapter_commu_with_voocphy(chip);

		if (oplus_voocphy_ap_allow_fastchg(chip)) {
			if (chip->fastchg_stage == OPLUS_FASTCHG_STAGE_2 && oplus_chg_get_flash_led_status()) {
				chip->fastchg_need_reset = 1;
				voocphy_info("OPLUS_FASTCHG_STAGE_2 and open torch exit fastchg\n");
				goto handle_done;
			} else {
				//oplus_chg_disable_charge();
				oplus_chg_suspend_charger();
				voocphy_info("allow fastchg adapter type %d\n", chip->adapter_type);
			}

			//handle timeout of adapter ask cmd 0x4
			oplus_voocphy_set_is_vbus_ok_predata(chip);

			if (chip->adapter_type == ADAPTER_SVOOC) {
				oplus_voocphy_hw_setting(chip, SETTING_REASON_SVOOC);
				if(chip->voocphy_dual_cp_support)
					oplus_voocphy_slave_hw_setting(chip, SETTING_REASON_SVOOC);
				if(oplus_chg_check_pd_svooc_adapater()) {
					voocphy_err("pd_svooc adapter, set config!\n");
					oplus_voocphy_set_pdsvooc_adapter_config(chip, true);
				}
			} else if (chip->adapter_type == ADAPTER_VOOC20 || chip->adapter_type == ADAPTER_VOOC30) {
				oplus_voocphy_hw_setting(chip, SETTING_REASON_VOOC);
				if(chip->voocphy_dual_cp_support)
					oplus_voocphy_slave_hw_setting(chip, SETTING_REASON_VOOC);
			} else {
				voocphy_info("SETTING_REASON_DEFAULT\n");
			}
		}
		chip->pre_adapter_ask_cmd = chip->fastchg_adapter_ask_cmd;
		oplus_voocphy_update_data(chip);
	}
	if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {
		chip->irq_hw_timeout_num++;
	}

	if (chip->vooc_flag & RXDATA_DONE_FLAG_MASK) {
		/*move to sc8547_update_chg_data for improving time of interrupt*/
	} else {
		chip->int_flag = oplus_voocphy_get_int_value(chip);
	}
	voocphy_info("pre_adapter_ask: 0x%0x, adapter_ask_cmd: 0x%0x, "
	             "int_flag: 0x%0x vooc_flag: 0x%0x\n",
	             chip->pre_adapter_ask_cmd, chip->fastchg_adapter_ask_cmd,
	             chip->int_flag, chip->vooc_flag);

	oplus_vooc_wake_voocphy_service_work(chip, VOOCPHY_REQUEST_UPDATE_DATA);

handle_done:
	return IRQ_HANDLED;
}

int oplus_voocphy_variables_init(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	chip->voocphy_rx_buff = 0;
	chip->voocphy_tx_buff[0] = 0;
	chip->voocphy_tx_buff[1] = 0;
	chip->adapter_check_ok = false;
	chip->adapter_ask_check_count = 3;
	chip->adapter_model_detect_count = 3;
	chip->fastchg_adapter_ask_cmd = VOOC_CMD_UNKNOW;
	chip->vooc2_next_cmd = VOOC_CMD_ASK_FASTCHG_ORNOT;
	chip->adapter_check_cmmu_count = 0;
	chip->adapter_rand_h = 0;
	chip->adapter_rand_l = 0;
	chip->code_id_far = 0;
	chip->code_id_local = 0xFFFF;
	chip->code_id_temp_h = 0;
	chip->code_id_temp_l = 0;
	chip->adapter_model_ver = 0;
	chip->adapter_model_count = 0;
	chip->ask_batt_sys = 0;
	chip->current_expect = chip->current_default;
	chip->current_max = chip->current_default;
	chip->current_ap = chip->current_default;
	chip->current_batt_temp = chip->current_default;
	chip->batt_temp_plugin = VOOCPHY_BATT_TEMP_NORMAL;
	chip->adapter_rand_start = false;
	chip->adapter_type = ADAPTER_SVOOC;
	chip->adapter_mesg = 0;
	chip->fastchg_allow = false;
	chip->ask_vol_again = false;
	chip->ap_need_change_current = 0;
	chip->adjust_curr = ADJ_CUR_STEP_DEFAULT;
	chip->adjust_fail_cnt = 0;
	chip->pre_adapter_ask_cmd = VOOC_CMD_UNKNOW;
	chip->ignore_first_frame = true;
	chip->vooc_vbus_status = VOOC_VBUS_UNKNOW;
	chip->vooc_head = VOOC_DEFAULT_HEAD;
	chip->ask_vooc3_detect = false;
	chip->force_2a = false;
	chip->force_3a = false;
	chip->force_2a_flag = false;
	chip->force_3a_flag = false;
	chip->btb_temp_over = false;
	chip->btb_err_first = false;
	chip->fastchg_notify_status = FAST_NOTIFY_UNKNOW;
	chip->fastchg_timeout_time = FASTCHG_TOTAL_TIME; //
	chip->fastchg_3c_timeout_time = FASTCHG_3000MA_TIMEOUT;
	chip->gauge_vbatt = 0;
	chip->vbus = 0;
	chip->batt_soc = 50;
	chip->fastchg_batt_temp_status = BAT_TEMP_NATURAL;
	chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_INIT;
	chip->fastchg_start = false;
	chip->fastchg_to_normal = false;
	chip->fastchg_to_warm = false;
	chip->fastchg_dummy_start = false;
	chip->fastchg_real_allow = false;
	chip->fastchg_commu_stop = false;
	chip->fastchg_check_stop = false;
	chip->fastchg_monitor_stop = false;
	chip->current_pwd = chip->current_expect;
	chip->curr_pwd_count = 10;
	chip->usb_bad_connect = false;
	chip->vooc_move_head = false;
	chip->fastchg_err_commu = false;
	chip->fastchg_reactive = false;
	chip->ask_bat_model_finished = false;
	oplus_voocphy_set_user_exit_fastchg(false);
	chip->fastchg_need_reset = 0;
	chip->fastchg_recover_cnt = 0;
	chip->fastchg_recovering = false;

	//note exception info
	chip->vooc_flag = 0;
	chip->int_flag = 0;
	chip->irq_total_num = 0;
	chip->irq_rcvok_num = 0;
	chip->irq_rcverr_num = 0;
	chip->irq_tx_timeout_num = 0;
	chip->irq_tx_timeout = 0;
	chip->irq_hw_timeout_num = 0;

	//default vooc head as svooc
	status = oplus_voocphy_write_mesg_mask(VOOC_INVERT_HEAD_MASK,
	                                       &chip->voocphy_tx_buff[0], chip->vooc_head);

	return status;
}

void oplus_voocphy_turn_off_fastchg(void)
{
	if (!g_voocphy_chip) {
		voocphy_err("g_voocphy_chipis null\n");
		return;
	}
	if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
		|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		oplus_chg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);

		g_voocphy_chip->fastchg_start= false;
		g_voocphy_chip->fastchg_to_normal = false;
		g_voocphy_chip->fastchg_to_warm = false;
		g_voocphy_chip->fastchg_ing = false;
		g_voocphy_chip->fastchg_dummy_start = false;
		g_voocphy_chip->fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;

		oplus_chg_clear_chargerid_info();
		oplus_chg_wake_update_work();
		oplus_chg_suspend_charger();
		chg_err("oplus_voocphy_turn_off_fastchg\n");
	}
	return;
}

void oplus_voocphy_handle_voocphy_status(struct work_struct *work)
{
	struct oplus_voocphy_manager *chip
	    = container_of(work, struct oplus_voocphy_manager,
	                   notify_fastchg_work.work);
	int intval = 0;

	if (!chip) {
		voocphy_err("chip null\n");
		return;
	}

	intval = chip->fastchg_notify_status;

	voocphy_info("vooc status: [%d]\n", intval);
	if (intval == FAST_NOTIFY_PRESENT) {
		chip->fastchg_start = true;		//lkl need to avoid guage data
		chip->fastchg_to_warm = false;
		chip->fastchg_dummy_start = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;
		chip->fast_chg_type = chip->adapter_model_ver;
		voocphy_info("fastchg start: [%d], adapter version: [0x%0x]\n",
		             chip->fastchg_start, chip->fast_chg_type);
		oplus_chg_wake_update_work();
	} else if (intval == FAST_NOTIFY_DUMMY_START) {
		chip->fastchg_start = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_dummy_start = true;
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;
		chip->fast_chg_type = chip->adapter_model_ver;
		voocphy_info("fastchg dummy start: [%d], adapter version: [0x%0x]\n",
		             chip->fastchg_dummy_start, chip->fast_chg_type);
		oplus_chg_wake_update_work();
	} else if ((intval & 0xFF) == FAST_NOTIFY_ONGOING) {
		chip->fastchg_ing = true;
		voocphy_info("voocphy fastchg_ing: [%d]\n", chip->fastchg_ing);
	} else if (intval == FAST_NOTIFY_FULL || intval == FAST_NOTIFY_BAD_CONNECTED) {
		voocphy_info("%s\r\n", intval == FAST_NOTIFY_FULL ? "FAST_NOTIFY_FULL" : "FAST_NOTIFY_BAD_CONNECTED");
		oplus_chg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);

		//note flags
		chip->fastchg_to_normal = true;
		usleep_range(350000, 350000);
		chip->fastchg_start = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_dummy_start = false;
		chip->fastchg_ing = false;
		voocphy_info("fastchg to normal: [%d]\n",
		             chip->fastchg_to_normal);

		//check fastchg out
		oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_CHG_OUT_CHECK, VOOC_CHG_OUT_CHECK_TIME);

		oplus_chg_wake_update_work();
	} else if (intval == FAST_NOTIFY_BATT_TEMP_OVER) {
		voocphy_info("FAST_NOTIFY_BATT_TEMP_OVER\r\n");
		oplus_chg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);

		chip->fastchg_to_warm = true;
		usleep_range(350000, 350000);
		chip->fastchg_start = false;
		chip->fastchg_dummy_start = true;
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;

		voocphy_info("fastchg to warm: [%d]\n",
		             chip->fastchg_to_warm);

		//check fastchg out and temp recover
		oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);

		oplus_chg_wake_update_work();
	} else if (intval == FAST_NOTIFY_ERR_COMMU) {
		chip->fastchg_start = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_dummy_start = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;
		chip->fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;
		oplus_chg_set_charger_type_unknown();
		oplus_chg_wake_update_work();
		voocphy_info("fastchg err commu: [%d]\n", intval);
	} else if (intval == FAST_NOTIFY_USER_EXIT_FASTCHG) {
		//switch usbswitch and ctrl2
		voocphy_info("FAST_NOTIFY_USER_EXIT_FASTCHG\r\n");
		oplus_chg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);

		oplus_voocphy_set_user_exit_fastchg(true);
		usleep_range(350000, 350000);
		chip->fastchg_start = false;
		chip->fastchg_dummy_start = true;
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;

		voocphy_info("user_exit_fastchg: [%d]\n",
		             chip->user_exit_fastchg);

		//check fastchg out and temp recover
		oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
		oplus_chg_set_charger_type_unknown();

		oplus_chg_wake_update_work();
	} else if (intval == FAST_NOTIFY_BTB_TEMP_OVER) {
		printk(KERN_ERR "!!![voocphy] FAST_NOTIFY_BTB_TEMP_OVER: [%d]\n", intval);
	} else {
		voocphy_info("non handle status: [%d]\n", intval);
	}
}

bool oplus_voocphy_wake_notify_fastchg_work(struct oplus_voocphy_manager *chip)
{
	return schedule_delayed_work(&chip->notify_fastchg_work, 0);
}

void oplus_voocphy_set_status_and_notify_ap(struct oplus_voocphy_manager *chip,
        int fastchg_notify_status)
{
	unsigned char dchg_enable_status = 0;

	if (!chip) {
		voocphy_err("chip null\n");
		return;
	}

	voocphy_info( "monitor fastchg_notify:%d", fastchg_notify_status);

	switch (fastchg_notify_status) {
	case FAST_NOTIFY_PRESENT:
	case FAST_NOTIFY_DUMMY_START:
		chip->fastchg_notify_status = fastchg_notify_status;
		if (fastchg_notify_status == FAST_NOTIFY_DUMMY_START) {
			//start fastchg_check timer
			voocphy_info( "VOOC_THREAD_TIMER_FASTCHG_CHECK start for FAST_NOTIFY_DUMMY_START:%d", fastchg_notify_status);
			oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);

			//reset vooc_phy
			oplus_voocphy_reset_voocphy();
			//oplus_vooc_wake_voocphy_service_work(VOOCPHY_REQUEST_RESET);

			//stop voocphy commu with adapter time
			oplus_voocphy_commu_process_handle(false);
		}
		break;
	case FAST_NOTIFY_ONGOING:
		chip->fastchg_notify_status = fastchg_notify_status;
		break;
	case FAST_NOTIFY_FULL:
	case FAST_NOTIFY_BATT_TEMP_OVER:
	case FAST_NOTIFY_BAD_CONNECTED:
	case FAST_NOTIFY_USER_EXIT_FASTCHG:
		oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_DISCON, VOOC_DISCON_EVENT_TIME);
		chip->fastchg_notify_status = fastchg_notify_status;

		//stop monitor thread
		oplus_voocphy_monitor_stop(chip);

		if (fastchg_notify_status == FAST_NOTIFY_BATT_TEMP_OVER) {
			//start fastchg_check timer
			chip->fastchg_to_warm = true;
			voocphy_info( "VOOC_THREAD_TIMER_FASTCHG_CHECK start for FAST_NOTIFY_BATT_TEMP_OVER:%d", fastchg_notify_status);
			//oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
		}

		if (fastchg_notify_status == FAST_NOTIFY_USER_EXIT_FASTCHG) {
			oplus_voocphy_set_user_exit_fastchg(true);
			chip->fastchg_dummy_start = true;
		}

		//reset vooc_phy
		oplus_voocphy_reset_voocphy();

		//stop voocphy commu with adapter time
		oplus_voocphy_commu_process_handle(false);		//lkl need modify
		break;
	case FAST_NOTIFY_ERR_COMMU:
		chip->fastchg_notify_status = fastchg_notify_status;

		/*if dchg enable, then reset_voocphy will create pluginout, so wo need not run
		5s fastchg check, if not we need run the 5s thread to recovery the vooc*/
		//PmSchgVooc_GetDirectChgEnable(PMIC_INDEX_3, &dchg_enable_status);
		oplus_voocphy_get_chg_enable(chip, &dchg_enable_status);
		if (chip->fastchg_err_commu == false && ((chip->adapter_type == ADAPTER_SVOOC && dchg_enable_status == 0)
		        ||(chip->adapter_type == ADAPTER_VOOC20 && chip->fastchg_allow))) {
			voocphy_info("!VOOC_THREAD_TIMER_FASTCHG_CHECK abnormal status, should run fastchg check");
			oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
			chip->fastchg_err_commu = true;
			oplus_voocphy_reset_fastchg_after_usbout();
			chip->fastchg_start = false;
		} else {
			chip->fastchg_start = false;
			oplus_voocphy_reset_fastchg_after_usbout();
		}

		//reset vooc_phy
		oplus_voocphy_reset_voocphy();

		//stop voocphy commu with adapter time
		oplus_voocphy_commu_process_handle(false);

		//stop monitor thread
		oplus_voocphy_monitor_stop(chip);

		oplus_chg_set_charger_type_unknown();
		break;
	case FAST_NOTIFY_BTB_TEMP_OVER:
		chip->fastchg_notify_status = fastchg_notify_status;
		voocphy_err("FAST_NOTIFY_BTB_TEMP_OVER");
		break;
	default:
		/*handle other error status*/
		chip->fastchg_start = false;
		oplus_voocphy_reset_fastchg_after_usbout();

		//reset vooc_phy
		oplus_voocphy_reset_voocphy();

		//stop voocphy commu with adapter time
		oplus_voocphy_commu_process_handle(false);

		//stop monitor thread
		oplus_voocphy_monitor_stop(chip);
		break;
	}

	oplus_voocphy_wake_notify_fastchg_work(chip);
}

bool oplus_voocphy_user_exit_fastchg(struct oplus_voocphy_manager *chip)
{
	if (chip) {
		voocphy_dbg("user 1 %s fastchg\n", chip->user_exit_fastchg == true ? "not allow" : "allow");
		return chip->user_exit_fastchg;
	} else {
		voocphy_dbg("user 2 %s fastchg\n", "not allow");
		return false;
	}
}

bool oplus_voocphy_set_user_exit_fastchg(unsigned char exit)
{
	if (g_voocphy_chip) {
		voocphy_info("exit %d\n", exit);
		g_voocphy_chip->user_exit_fastchg = exit;
	}
	return true;
}

bool oplus_voocphy_stage_check(void)
{
	unsigned long cur_chg_time = 0;

	if (oplus_voocphy_get_fastchg_state() == OPLUS_FASTCHG_STAGE_2) {
		voocphy_info("torch return for OPLUS_FASTCHG_STAGE_2\n");
		return false;
	}

	if (oplus_voocphy_get_fastchg_state() == OPLUS_FASTCHG_STAGE_1) {
		if (oplus_voocphy_user_exit_fastchg(g_voocphy_chip)) {
			voocphy_info("torch return for user_exit_fastchg \n");
			return false;
		} else if (g_voocphy_chip->fastchg_recovering) {
			oplus_chg_get_curr_time_ms(&cur_chg_time);
			cur_chg_time = cur_chg_time/1000;
			if (cur_chg_time - g_voocphy_chip->fastchg_recover_cnt >= 30) {
				/*fastchg_recovering need to check num of calling*/
				voocphy_info("torch return for fastchg_recovering1  \n");
				g_voocphy_chip->fastchg_recovering = false;
				g_voocphy_chip->fastchg_recover_cnt = 0;
				goto stage_check;
			} else {
				voocphy_info("torch return for fastchg_recovering2  \n");
				return false;
			}
		} else {
			voocphy_dbg("do nothing for [%d %d]\n", g_voocphy_chip->fastchg_recovering, g_voocphy_chip->fastchg_recover_cnt);
			goto stage_check;
		}
	}
stage_check:
	return true;
}

void oplus_voocphy_notify(int event)
{
	bool plugIn = false;
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	unsigned long cur_chg_time = 0;
	voocphy_info(" start %s flash led %d, %d\n", event ? "open" : "close", chip->user_exit_fastchg, chip->fastchg_recovering);
	if (1 == event) {
		if (oplus_voocphy_get_fastchg_state() == OPLUS_FASTCHG_STAGE_2) {
			oplus_voocphy_set_status_and_notify_ap(chip, FAST_NOTIFY_USER_EXIT_FASTCHG);
		} else if (chip->fastchg_recovering == false) {
			//plugIn = oplus_chg_stats();
			oplus_chg_wake_update_work();
			voocphy_info("is normal adapter plugIn = %d\n", plugIn);
		} else {
			chip->fastchg_recovering = false;
			oplus_voocphy_set_user_exit_fastchg(true) ;
		}
	} else if (0 == event/* && chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP*/) {		//recover
		oplus_voocphy_set_user_exit_fastchg(false) ;
		oplus_voocphy_set_fastchg_state(chip, OPLUS_FASTCHG_STAGE_1);
		chip->fastchg_recover_cnt = 0;
		chip->fastchg_recovering = true;
		//plugIn = oplus_chg_stats();
		//oplus_voocphy_monitor_timer_start(VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
		oplus_chg_get_curr_time_ms(&cur_chg_time);
		chip->fastchg_recover_cnt = cur_chg_time/1000;

		//voocphy_info("VOOC_THREAD_TIMER_FASTCHG_CHECK plugIn %d charger_type %d %d switch %d %d\n",
		//	plugIn, chip->charger_type, chip->chg_ops->get_charger_type(),
		//	chip->chg_ops->get_chargerid_switch_val(), oplus_vooc_get_vooc_switch_val());
		//if (plugIn && chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		//	oplus_chg_wake_update_work();
		//}
	}
	voocphy_info(" end %s flash led %d\n", event ? "open" : "close", chip->user_exit_fastchg);
}

unsigned char oplus_voocphy_set_fastchg_current(struct oplus_voocphy_manager *chip)
{
	unsigned char current_expect_temp = chip->current_expect;

	voocphy_info( "max_curr[%d] ap_curr[%d] temp_curr[%d]",
	              chip->current_max, chip->current_ap, chip->current_batt_temp);

	chip->current_expect = chip->current_max > chip->current_ap ? chip->current_ap : chip->current_max;

	chip->current_expect = chip->current_expect > chip->current_batt_temp ? chip->current_batt_temp : chip->current_expect;

	if (current_expect_temp != chip->current_expect) {
		voocphy_info( "current_expect[%d --> %d]",
		              current_expect_temp, chip->current_expect);
		return 5;
	}
	voocphy_err("set current expect = %d\n", chip->current_expect);

	return 0;
}

enum hrtimer_restart oplus_voocphy_monitor_base_timer(struct hrtimer* time)
{
	int i = 0;
	struct oplus_voocphy_manager *chip = g_voocphy_chip;

	//cont for all evnents
	for (i = 0; i < MONITOR_EVENT_NUM; i++) {
		if (chip->mornitor_evt[i].status == VOOC_MONITOR_START) {
			chip->mornitor_evt[i].cnt++;
			if (chip->mornitor_evt[i].cnt == chip->mornitor_evt[i].expires) {
				chip->mornitor_evt[i].timeout = true;
				chip->mornitor_evt[i].cnt = 0;
				chip->mornitor_evt[i].status = VOOC_MONITOR_STOP;
				//notify monitor event
				oplus_vooc_wake_monitor_work(chip);
			}
		}
	}
	hrtimer_forward(&chip->monitor_btimer,chip->monitor_btimer.base->get_time(),chip->moniotr_kt);

	return HRTIMER_RESTART;
}

void oplus_voocphy_monitor_process_events(struct work_struct *work)
{
	int evt_i = 0;
	unsigned long data = 0;
	struct oplus_voocphy_manager *chip
	    = container_of(work, struct oplus_voocphy_manager,
	                   monitor_work.work);

	//event loop
	for (evt_i = 0; evt_i < MONITOR_EVENT_NUM; evt_i++) {
		if (chip->mornitor_evt[evt_i].timeout) {
			chip->mornitor_evt[evt_i].timeout = false;		//avoid repeating run monitor event
			data = chip->mornitor_evt[evt_i].data;
			if (chip->mornitor_evt[evt_i].mornitor_hdler)
				chip->mornitor_evt[evt_i].mornitor_hdler(data);
		}
	}

	return;
}

int oplus_voocphy_get_ap_current(void)
{
	struct oplus_chg_chip *g_charger_chip = oplus_chg_get_chg_struct();
	unsigned char ap_request_current = 0;

	if (!g_charger_chip) {
		voocphy_info("g_charger_chip is null\n");
		return 0;
	}

	voocphy_info("cool_down: %d\n", g_charger_chip->cool_down);
	ap_request_current = g_voocphy_chip->svooc_cool_down_current_limit[g_charger_chip->cool_down];

	if(g_voocphy_chip->voocphy_dual_cp_support) {
		if(g_voocphy_chip->adapter_type == ADAPTER_SVOOC) {

			ap_request_current /= 2;
		} else if(g_voocphy_chip->adapter_type == ADAPTER_VOOC20
		          || g_voocphy_chip->adapter_type == ADAPTER_VOOC30) {
			if (ap_request_current > 59) {
				ap_request_current = 59;
			}
		}
	} else {
		if (ap_request_current == 0 || ap_request_current > 30) {
			ap_request_current = 30;
		}
	}

	voocphy_info("ap_current = %u\n", ap_request_current);
	return ap_request_current;
}

int oplus_voocphy_ap_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	static bool ignore_first_monitor = true;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_ap_event_handle ignore");
		return status;
	}
	if ((chip->fastchg_notify_status & 0XFF) == FAST_NOTIFY_PRESENT) {
		ignore_first_monitor = true;
	}

	oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_ONGOING);

	chip->current_ap = oplus_voocphy_get_ap_current();
	voocphy_info( "ignore_first_monitor %d %d\n", ignore_first_monitor, chip->current_ap);
	if (!chip->btb_temp_over) {   //btb temp is normal
		if ((!chip->ap_need_change_current)
		    && (ignore_first_monitor == false)) {
			chip->ap_need_change_current
			    = oplus_voocphy_set_fastchg_current(chip);
			voocphy_info( "ap_need_change_current %d\n", chip->ap_need_change_current);
		} else {
			ignore_first_monitor = false;
		}

		status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_AP, VOOC_AP_EVENT_TIME);
	}

	return status;
}

#define VOOC_TEMP_OVER_COUNTS	2
#define VOOC_TEMP_RANGE_THD					10

void oplus_voocphy_reset_temp_range(struct oplus_voocphy_manager *chip)
{
	if (chip != NULL) {
		chip->vooc_little_cold_temp = chip->vooc_little_cold_temp_default;
		chip->vooc_cool_temp = chip->vooc_cool_temp_default;
		chip->vooc_little_cool_temp = chip->vooc_little_cool_temp_default;
		chip->vooc_normal_low_temp = chip->vooc_normal_low_temp_default;
	}
}

static int oplus_voocphy_set_current_temp_normal_range(struct oplus_voocphy_manager *chip, int vbat_temp_cur)
{
	int ret = 0;

	chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_HIGH;

	switch (chip->fastchg_batt_temp_status) {
	case BAT_TEMP_NORMAL_HIGH:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp0) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH0;
			ret = chip->vooc_strategy1_high_current0;
		} else if (vbat_temp_cur >= chip->vooc_normal_low_temp) {
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_HIGH;
			ret = chip->vooc_strategy_normal_current;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
			ret = chip->vooc_strategy_normal_current;
			oplus_voocphy_reset_temp_range(chip);
			chip->vooc_normal_low_temp += VOOC_TEMP_RANGE_THD;
		}
		break;
	case BAT_TEMP_HIGH0:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp1) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH1;
			ret = chip->vooc_strategy1_high_current1;
		} else if (vbat_temp_cur < chip->vooc_strategy1_batt_low_temp0) {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW0;
			ret = chip->vooc_strategy1_low_current0;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH0;
			ret = chip->vooc_strategy1_high_current0;
		}
		break;
	case BAT_TEMP_HIGH1:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp2) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH2;
			ret = chip->vooc_strategy1_high_current2;
		} else if (vbat_temp_cur < chip->vooc_strategy1_batt_low_temp1) {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW1;
			ret = chip->vooc_strategy1_low_current1;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH1;
			ret = chip->vooc_strategy1_high_current1;
		}
		break;
	case BAT_TEMP_HIGH2:
		if (chip->vooc_batt_over_high_temp != -EINVAL
		    && vbat_temp_cur > chip->vooc_batt_over_high_temp) {
			chip->vooc_strategy_change_count++;
			if (chip->vooc_strategy_change_count >= VOOC_TEMP_OVER_COUNTS) {
				chip->vooc_strategy_change_count = 0;
				chip->fastchg_batt_temp_status = BAT_TEMP_EXIT;
				voocphy_info( "[vooc_monitor] temp_over:%d", vbat_temp_cur);
				oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BATT_TEMP_OVER);
			}
		} else if (vbat_temp_cur < chip->vooc_strategy1_batt_low_temp2) {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW2;
			ret = chip->vooc_strategy1_low_current2;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH2;
			ret = chip->vooc_strategy1_high_current2;;
		}
		break;
	case BAT_TEMP_LOW0:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp0) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH0;
			ret = chip->vooc_strategy1_high_current0;
		} else if (vbat_temp_cur < chip->vooc_normal_low_temp) {/*T<25C*/
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
			ret = chip->vooc_strategy_normal_current;
			oplus_voocphy_reset_temp_range(chip);
			chip->vooc_normal_low_temp += VOOC_TEMP_RANGE_THD;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW0;
			ret = chip->vooc_strategy1_low_current0;
		}
		break;
	case BAT_TEMP_LOW1:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp1) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH1;
			ret = chip->vooc_strategy1_high_current1;
		} else if (vbat_temp_cur < chip->vooc_strategy1_batt_low_temp0) {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW0;
			ret = chip->vooc_strategy1_low_current0;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW1;
			ret = chip->vooc_strategy1_low_current1;
		}
		break;
	case BAT_TEMP_LOW2:
		if (vbat_temp_cur > chip->vooc_strategy1_batt_high_temp2) {
			chip->fastchg_batt_temp_status = BAT_TEMP_HIGH2;
			ret = chip->vooc_strategy1_high_current2;
		} else if (vbat_temp_cur < chip->vooc_strategy1_batt_low_temp1) {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW1;
			ret = chip->vooc_strategy1_low_current1;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_LOW2;
			ret = chip->vooc_strategy1_low_current2;
		}
		break;
	default:
		break;
	}
	voocphy_info("the ret: %d, the temp =%d, status = %d\r\n", ret, vbat_temp_cur, chip->fastchg_batt_temp_status);
	return ret;
}

static int oplus_voocphy_set_current_temp_low_normal_range(struct oplus_voocphy_manager *chip, int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->vooc_normal_low_temp
	    && vbat_temp_cur >= chip->vooc_little_cool_temp) { /*16C<=T<25C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
		chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
		ret = chip->vooc_strategy_normal_current;
	} else {
		if (vbat_temp_cur >= chip->vooc_normal_low_temp) {
			chip->vooc_normal_low_temp -= VOOC_TEMP_RANGE_THD;
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_HIGH;
			ret = chip->vooc_strategy_normal_current;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_HIGH;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COOL;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COOL;
			ret = chip->vooc_strategy_normal_current;
			oplus_voocphy_reset_temp_range(chip);
			chip->vooc_little_cool_temp += VOOC_TEMP_RANGE_THD;
		}
	}

	return ret;
}

static int oplus_voocphy_set_current_temp_little_cool_range(struct oplus_voocphy_manager *chip, int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->vooc_little_cool_temp
	    && vbat_temp_cur >= chip->vooc_cool_temp) {/*12C<=T<16C*/
		chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COOL;
		ret = chip->vooc_strategy_normal_current;
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COOL;
	} else {
		if (vbat_temp_cur >= chip->vooc_little_cool_temp) {
			chip->vooc_little_cool_temp -= VOOC_TEMP_RANGE_THD;
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
			ret = chip->vooc_strategy_normal_current;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_COOL;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_COOL;
			ret = chip->vooc_strategy_normal_current;
			oplus_voocphy_reset_temp_range(chip);
			chip->vooc_cool_temp += VOOC_TEMP_RANGE_THD;
		}
	}

	return ret;
}

static int oplus_voocphy_set_current_temp_cool_range(struct oplus_voocphy_manager *chip, int vbat_temp_cur)
{
	int ret = 0;
	if (chip->vooc_batt_over_high_temp != -EINVAL
	    && vbat_temp_cur < chip->vooc_batt_over_low_temp) {
		chip->vooc_strategy_change_count++;
		if (chip->vooc_strategy_change_count >= VOOC_TEMP_OVER_COUNTS) {
			chip->vooc_strategy_change_count = 0;
			chip->fastchg_batt_temp_status = BAT_TEMP_EXIT;
			voocphy_info( "[vooc_monitor] temp_over:%d", vbat_temp_cur);
			oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BATT_TEMP_OVER);
		}
	} else if (vbat_temp_cur < chip->vooc_cool_temp
	           && vbat_temp_cur >= chip->vooc_little_cold_temp) {/*5C <=T<12C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_COOL;
		chip->fastchg_batt_temp_status = BAT_TEMP_COOL;
		ret = chip->vooc_strategy_normal_current;
	} else {
		if (vbat_temp_cur >= chip->vooc_cool_temp) {
			chip->vooc_cool_temp -= VOOC_TEMP_RANGE_THD;
			chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COOL;
			ret = chip->vooc_strategy_normal_current;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COOL;
		} else {
			chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COLD;
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COLD;
			ret = chip->vooc_strategy_normal_current;
			oplus_voocphy_reset_temp_range(chip);
			chip->vooc_little_cold_temp += VOOC_TEMP_RANGE_THD;
		}
	}

	return ret;
}

static int oplus_voocphy_set_current_temp_little_cold_range(struct oplus_voocphy_manager *chip, int vbat_temp_cur)
{
	int ret = 0;
	if (chip->vooc_batt_over_high_temp != -EINVAL
	    && vbat_temp_cur < chip->vooc_batt_over_low_temp) {
		chip->vooc_strategy_change_count++;
		if (chip->vooc_strategy_change_count >= VOOC_TEMP_OVER_COUNTS) {
			chip->vooc_strategy_change_count = 0;
			chip->fastchg_batt_temp_status = BAT_TEMP_EXIT;
			voocphy_info( "[vooc_monitor] temp_over:%d", vbat_temp_cur);
			oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BATT_TEMP_OVER);
		}
	} else if (vbat_temp_cur < chip->vooc_little_cold_temp) { /*0C<=T<5C*/
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COLD;
		chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COLD;
		ret = chip->vooc_strategy_normal_current;
	} else {
		chip->fastchg_batt_temp_status = BAT_TEMP_COOL;
		ret = chip->vooc_strategy_normal_current;
		chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_COOL;
		oplus_voocphy_reset_temp_range(chip);
		chip->vooc_little_cold_temp -= VOOC_TEMP_RANGE_THD;
	}

	return ret;
}

static int oplus_voocphy_set_current_by_batt_temp(struct oplus_voocphy_manager *chip,
					int vbat_temp_cur)
{
	int ret = 0;

	if (chip->vooc_temp_cur_range == FASTCHG_TEMP_RANGE_INIT) {
		if (vbat_temp_cur < chip->vooc_little_cold_temp) { /*0-5C*/
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COLD;
			chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COLD;
		} else if (vbat_temp_cur < chip->vooc_cool_temp) { /*5-12C*/
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_COOL;
			chip->fastchg_batt_temp_status = BAT_TEMP_COOL;
		} else if (vbat_temp_cur < chip->vooc_little_cool_temp) { /*12-16C*/
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_LITTLE_COOL;
			chip->fastchg_batt_temp_status = BAT_TEMP_LITTLE_COOL;
		} else if (vbat_temp_cur < chip->vooc_normal_low_temp) { /*16-25C*/
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_LOW;
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_LOW;
		} else {/*25C-43C*/
			chip->vooc_temp_cur_range = FASTCHG_TEMP_RANGE_NORMAL_HIGH;
			chip->fastchg_batt_temp_status = BAT_TEMP_NORMAL_HIGH;
		}
	}

	switch (chip->vooc_temp_cur_range) {
	case FASTCHG_TEMP_RANGE_NORMAL_HIGH:
		ret = oplus_voocphy_set_current_temp_normal_range(chip, vbat_temp_cur);
		break;
	case FASTCHG_TEMP_RANGE_NORMAL_LOW:
		ret = oplus_voocphy_set_current_temp_low_normal_range(chip, vbat_temp_cur);
		break;
	case FASTCHG_TEMP_RANGE_LITTLE_COOL:
		ret = oplus_voocphy_set_current_temp_little_cool_range(chip, vbat_temp_cur);
		break;
	case FASTCHG_TEMP_RANGE_COOL:
		ret = oplus_voocphy_set_current_temp_cool_range(chip, vbat_temp_cur);
		break;
	case FASTCHG_TEMP_RANGE_LITTLE_COLD:
		ret = oplus_voocphy_set_current_temp_little_cold_range(chip, vbat_temp_cur);
		break;
	default:
		break;
	}

	voocphy_info("the ret: %d, the temp =%d, temp_status = %d, temp_range = %d\r\n",
	             ret, vbat_temp_cur, chip->fastchg_batt_temp_status, chip->vooc_temp_cur_range);
	return ret;
}

bool oplus_voocphy_btb_and_usb_temp_detect(void)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	bool detect_over = false;
	int status = VOOCPHY_SUCCESS;
	int btb_temp =0, usb_temp = 0;
	static unsigned char temp_over_count = 0;

	if ((chip->fastchg_notify_status & 0XFF)== FAST_NOTIFY_PRESENT) {
		temp_over_count = 0;
	}
	voocphy_err("use adc ctrl!\n");
	oplus_chg_adc_switch_ctrl();

	btb_temp = oplus_chg_get_battery_btb_temp_cal();
	usb_temp = oplus_chg_get_usb_btb_temp_cal();

	if (status != VOOCPHY_SUCCESS) {
		voocphy_info( "get btb and usb temp error");
		return detect_over;
	}

	voocphy_err( "btb_temp: %d, usb_temp: %d", btb_temp, usb_temp);
	if (btb_temp >= 80 ||usb_temp >= 80) {
		temp_over_count++;
		if (temp_over_count > 9) {
			detect_over = true;
			voocphy_info( "btb_and_usb temp over");
		}
	} else {
		temp_over_count = 0;
	}

	return detect_over;
}

bool oplus_voocphy_get_btb_temp_over(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->btb_temp_over;
	}
}
int oplus_voocphy_btb_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;

	bool btb_detect = false;
	static bool btb_err_report = false;

	voocphy_info( "oplus_voocphy_btb_event_handle");
	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_btb_event_handle ignore");
		return status;
	}

	btb_detect = oplus_voocphy_btb_and_usb_temp_detect();
	if ((chip->fastchg_notify_status & 0XFF) == FAST_NOTIFY_PRESENT) {
		btb_err_report = false;
	}

	if(!chip->btb_temp_over) {
		if (btb_detect) {
			oplus_voocphy_slave_set_chg_enable(chip, false);
			oplus_voocphy_set_chg_enable(chip, false);
			chip->btb_temp_over = true;
		}
		status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_BTB, VOOC_BTB_EVENT_TIME);
	} else {
		if (!btb_err_report) {
			btb_err_report = true;
			oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BTB_TEMP_OVER);
		}
		status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_BTB, VOOC_BTB_OVER_EVENT_TIME);
	}

	return status;
}

int oplus_voocphy_temp_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	int batt_temp = 0;
	static bool allow_temp_monitor = false;
	static unsigned char delay_temp_count = 0;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_temp_event_handle ignore");
		return status;
	}

	batt_temp = oplus_chg_get_chg_temperature();
	if (chip->batt_fake_temp) {
		batt_temp = chip->batt_fake_temp;
	}
	if ((chip->fastchg_notify_status & 0XFF) == FAST_NOTIFY_PRESENT) {
		delay_temp_count = 0;
		allow_temp_monitor = false;
	}

	if (delay_temp_count < DELAY_TEMP_MONITOR_COUNTS) {
		delay_temp_count++;
		allow_temp_monitor = false;
	} else {
		allow_temp_monitor = true;
		delay_temp_count = DELAY_TEMP_MONITOR_COUNTS;
	}

	voocphy_info( "!batt_temp = %d %d %d %d\n", batt_temp, allow_temp_monitor,
	              chip->vooc_temp_cur_range, chip->fastchg_batt_temp_status);

	if ((!chip->btb_temp_over) && (allow_temp_monitor == true)) {
		chip->current_batt_temp = oplus_voocphy_set_current_by_batt_temp(chip, batt_temp);

		if (chip->voocphy_dual_cp_support) {
			if(chip->adapter_type == ADAPTER_SVOOC) {
				chip->current_batt_temp /= 2;
			} else if (chip->adapter_type == ADAPTER_VOOC20
			           || chip->adapter_type == ADAPTER_VOOC30) {
				if (chip->current_batt_temp > 59) {
					chip->current_batt_temp = 59;
				}
			}
		} else {
			if (chip->current_batt_temp > 30) {
				chip->current_batt_temp = 30;
			}
		}

		if (!chip->ap_need_change_current) {
			chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
		}
	}

	if (batt_temp >= 440 || batt_temp <= 0) {
		oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BATT_TEMP_OVER);
	}


	status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_TEMP, VOOC_TEMP_EVENT_TIME);

	return status;
}

void oplus_voocphy_request_fastchg_curv(struct oplus_voocphy_manager *chip)
{
	static int cc_cnt = 0;
	int idx = 0;
	int convert_ibus = 0;
	struct batt_sys_curves *batt_sys_curv_by_tmprange = chip->batt_sys_curv_by_tmprange;
	struct batt_sys_curve *batt_sys_curve = NULL;
	struct batt_sys_curve *batt_sys_curve_next = NULL;

	if (!chip || !batt_sys_curv_by_tmprange) {
		voocphy_info("chip or batt_sys_curv_by_tmprange is NULL pointer!!!\n");
		return;
	}

	if (!chip->batt_sys_curv_found) {	/*first enter and find index*/
		//use default chg
		idx = chip->cur_sys_curv_idx;
		voocphy_info("!fastchg curv1 [%d %d %d]\n", chip->current_max, (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_ibus, (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_vbat);
	} else {	/*adjust current*/
		idx = chip->cur_sys_curv_idx;
		if (idx >= BATT_SYS_ROW_MAX) {
			voocphy_info("idx out of bound of array!!!!\n");
			return;
		}
		batt_sys_curve = &(batt_sys_curv_by_tmprange->batt_sys_curve[idx]);
		if(!batt_sys_curve) {
			voocphy_info("batt_sys_curve is a NULL pointer!!!!\n");
			return;
		}
		batt_sys_curve->chg_time++;
		if (batt_sys_curve->exit == false) {
			voocphy_info("!fastchg curv2 [%d %d %d %d %d %d %d %d %d]\n", cc_cnt, idx, batt_sys_curve->exit,
			             batt_sys_curve->target_time, batt_sys_curve->chg_time,
			             chip->gauge_vbatt, chip->current_max,
			             (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_ibus,
			             (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_vbat);
			convert_ibus = batt_sys_curve->target_ibus * 100;
			if ((chip->gauge_vbatt > batt_sys_curve->target_vbat			//switch by vol_thr
			     && chip->cp_ichg > convert_ibus - TARGET_VOL_OFFSET_THR
			     && chip->cp_ichg < convert_ibus + TARGET_VOL_OFFSET_THR)
			    || (batt_sys_curve->target_time > 0 && batt_sys_curve->chg_time >= batt_sys_curve->target_time)		//switch by chg time
			   ) {
				cc_cnt++;
				if (cc_cnt > 3) {
					if (batt_sys_curve->exit == false) {
						if (batt_sys_curve->chg_time >= batt_sys_curve->target_time) {
							voocphy_info("switch fastchg curv by chgtime[%d, %d]\n", batt_sys_curve->chg_time, batt_sys_curve->target_time);
						}
						batt_sys_curve->chg_time = 0;
						voocphy_info("!bf switch fastchg curv [%d %d %d]\n", chip->current_max, (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_ibus, (&(batt_sys_curv_by_tmprange->batt_sys_curve[idx]))->target_vbat);
						batt_sys_curve_next = &(batt_sys_curv_by_tmprange->batt_sys_curve[idx+1]);
						if(!batt_sys_curve_next) {
							voocphy_info("batt_sys_curve is a NULL pointer!!!!\n");
							return;
						}
						chip->cur_sys_curv_idx += 1;
						cc_cnt = 0;
						chip->current_max = chip->current_max > batt_sys_curve_next->target_ibus ? batt_sys_curve_next->target_ibus : chip->current_max;
						voocphy_info("!af switch fastchg curv [%d %d %d]\n", chip->current_max, batt_sys_curve_next->target_ibus, batt_sys_curve_next->target_vbat);
					}
				}
			} else {
				cc_cnt = 0;
			}
		} else {
			//exit fastchg
			voocphy_info("! exit fastchg\n");
		}
	}

	voocphy_info( "curv info [%d %d %d %d]\n", chip->gauge_vbatt, chip->cp_ichg, chip->current_max, cc_cnt);
}

int oplus_voocphy_vol_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	static unsigned char fast_full_count = 0;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_vol_event_handle ignore");
		return status;
	}

	voocphy_info( "[oplus_voocphy_vol_event_handle] vbatt: %d",chip->gauge_vbatt);
	if ((chip->fastchg_notify_status & 0xFF) == FAST_NOTIFY_PRESENT)
		fast_full_count = 0;

	if (!chip->btb_temp_over) {
		oplus_voocphy_request_fastchg_curv(chip);
		//set above calculate max current
		if (!chip->ap_need_change_current)
			chip->ap_need_change_current
			    = oplus_voocphy_set_fastchg_current(chip);

		//notify full at some condition
		if ((chip->batt_temp_plugin == VOOCPHY_BATT_TEMP_LITTLE_COLD			//0-5 chg to 4430mV
		     && chip->gauge_vbatt > TEMP0_2_TEMP45_FULL_VOLTAGE)
		    || (chip->batt_temp_plugin == VOOCPHY_BATT_TEMP_COOL				//5-12 chg to 4430mV
		        && chip->gauge_vbatt > TEMP45_2_TEMP115_FULL_VOLTAGE)
		    || (chip->gauge_vbatt > BAT_FULL_1TIME_THD)) {			//4510mV
			voocphy_info( "vbatt 1time fastchg full: %d",chip->gauge_vbatt);
			oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_FULL);
		} else if (chip->gauge_vbatt > BAT_FULL_NTIME_THD) {			//4480mV
			fast_full_count++;
			if (fast_full_count > 5) {
				voocphy_info( "vbatt ntime fastchg full: %d",chip->gauge_vbatt);
				oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_FULL);
			}
		}

		status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_VOL, VOOC_VOL_EVENT_TIME);
	}

	return status;
}

int oplus_voocphy_safe_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_safe_event_handle ignore");
		return status;
	}

	if (chip->fastchg_timeout_time)
		chip->fastchg_timeout_time--;
	if (chip->fastchg_3c_timeout_time)
		chip->fastchg_3c_timeout_time--;

	if (chip->fastchg_timeout_time == 0
	    && (!chip->btb_temp_over)) {
		oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_FULL);
	}

	if (chip->usb_bad_connect) {
		oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BAD_CONNECTED);
		//reset vooc_phy
		oplus_voocphy_reset_voocphy();
	}
	status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_SAFE, VOOC_SAFE_EVENT_TIME);

	return status;
}

int oplus_voocphy_ibus_check_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	static int ibus_trouble_count = 0;
	int status = VOOCPHY_SUCCESS;
	int main_cp_ibus = 0;
	int slave_cp_ibus = 0;
	u8 slave_cp_status = 0;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_ibus_check_event_handle ignore");
		return status;
	}

	oplus_voocphy_slave_get_chg_enable(chip, &slave_cp_status);
	if (slave_cp_status == 1) {
		main_cp_ibus = oplus_voocphy_get_ichg();
		slave_cp_ibus = oplus_voocphy_get_slave_ichg();

		voocphy_err("main_cp_ibus = -%d slave_cp_ibus = -%d", main_cp_ibus, slave_cp_ibus);
		if (chip->adapter_type == ADAPTER_SVOOC) {
			voocphy_err("adapter type = %d", chip->adapter_type);
			if (main_cp_ibus > 3200 || slave_cp_ibus > 3200) {
				ibus_trouble_count = ibus_trouble_count + 1;
			} else {
				voocphy_err("Discontinuity satisfies the condition, ibus_trouble_count = 0!\n");
				ibus_trouble_count = 0;
			}

			if (ibus_trouble_count >= 3) {
				voocphy_err("ibus_trouble_count >= 3, limit current to 3A!\n");
				if (chip->current_max > 30) {
					chip->current_max = 30;
					if (!chip->ap_need_change_current)
						chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
				}
			}
		} else if (chip->adapter_type == ADAPTER_VOOC30
					|| chip->adapter_type == ADAPTER_VOOC20) {
			voocphy_err("adapter type = %d", chip->adapter_type);
			if (main_cp_ibus > 3500 || slave_cp_ibus > 3500) {
				ibus_trouble_count = ibus_trouble_count + 1;
			} else {
				voocphy_err("Discontinuity satisfies the condition, ibus_trouble_count = 0!\n");
				ibus_trouble_count = 0;
			}

			if (ibus_trouble_count >= 3) {
				voocphy_err("ibus_trouble_count >= 3, limit current to 3A!\n");
				if (chip->current_max > 30) {
					chip->current_max = 30;
					if (!chip->ap_need_change_current)
						chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
				}
			}
		}
	} else if (slave_cp_status == 0) {
		main_cp_ibus = oplus_voocphy_get_ichg();
		if (chip->adapter_type == ADAPTER_SVOOC) {
			voocphy_err("adapter type = %d", chip->adapter_type);
			if (main_cp_ibus > 3200) {
				ibus_trouble_count = ibus_trouble_count + 1;
			} else {
				voocphy_err("Discontinuity satisfies the condition, ibus_trouble_count = 0!\n");
				ibus_trouble_count = 0;
			}
		if (ibus_trouble_count >= 3) {
				voocphy_err("ibus_trouble_count >= 3, limit current to 3A!\n");
				if (chip->current_max > 30) {
					chip->current_max = 30;
					if (!chip->ap_need_change_current)
						chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
				}
			}
		} else if (chip->adapter_type == ADAPTER_VOOC30
					|| chip->adapter_type == ADAPTER_VOOC20) {
			voocphy_err("adapter type = %d", chip->adapter_type);
			if (main_cp_ibus > 3500) {
				ibus_trouble_count = ibus_trouble_count + 1;
			} else {
				voocphy_err("Discontinuity satisfies the condition, ibus_trouble_count = 0!\n");
				ibus_trouble_count = 0;
			}
			if (ibus_trouble_count >= 3) {
				voocphy_err("ibus_trouble_count >= 3, limit current to 3A!\n");
				if (chip->current_max > 30) {
					chip->current_max = 30;
					if (!chip->ap_need_change_current)
						chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
				}
			}
		}
	}

	status = oplus_voocphy_monitor_timer_start(g_voocphy_chip, VOOC_THREAD_TIMER_IBUS_CHECK, VOOC_IBUS_CHECK_TIME);

	return status;
}


int oplus_voocphy_discon_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;

	chip->fastchg_start = false; //fastchg_start need be reset false;
	if (chip->fastchg_notify_status == FAST_NOTIFY_FULL || chip->fastchg_notify_status == FAST_NOTIFY_BAD_CONNECTED) {
		chip->fastchg_to_normal = true;
	} else if (chip->fastchg_notify_status == FAST_NOTIFY_BATT_TEMP_OVER) {
		chip->fastchg_to_warm = true;
	} else if (chip->fastchg_notify_status == FAST_NOTIFY_USER_EXIT_FASTCHG) {
	}

	voocphy_info ("!!![oplus_voocphy_discon_event_handle] [%d %d %d]",
	              chip->fastchg_to_normal, chip->fastchg_to_warm, chip->user_exit_fastchg);


	//code below moved to function oplus_voocphy_handle_voocphy_status
	//timer of check charger out time should been started
	//status = oplus_voocphy_monitor_timer_start(
	//		VOOC_THREAD_TIMER_CHG_OUT_CHECK, VOOC_CHG_OUT_CHECK_TIME);
	return status;
}

void oplus_voocphy_reset_fastchg_after_usbout(void)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;

	if (!chip) {
		voocphy_err ("g_voocphy_chip is null\n");
		return;
	}

	if (chip->fastchg_start == false) {
		chip->fastchg_notify_status = FAST_NOTIFY_UNKNOW;
		voocphy_info ("fastchg_start");
	}
	chip->fastchg_to_normal = false;
	chip->fastchg_to_warm = false;
	chip->fastchg_dummy_start = false;
	/*avoid displaying vooc symbles when btb_temp_over and plug out*/
	if (chip->btb_temp_over) {
		voocphy_info("btb_temp_over plug out\n");
		chip->btb_temp_over = false;
	} else {
		chip->fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;
	}
	chip->fastchg_reactive = false;
	oplus_voocphy_set_user_exit_fastchg(false);
	oplus_voocphy_set_fastchg_state(chip, OPLUS_FASTCHG_STAGE_1);

	oplus_voocphy_reset_variables(chip);
	oplus_voocphy_basetimer_monitor_stop(chip);
	oplus_chg_set_chargerid_switch_val(0);
	oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
	oplus_chg_clear_chargerid_info();
	oplus_voocphy_reset_ibus_trouble_flag();
	cpuboost_charge_event(CPU_CHG_FREQ_STAT_AUTO);
	#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
	pm_qos_update_request(&pm_qos_req, PM_QOS_DEFAULT_VALUE);
	#endif

	voocphy_info("reset fastchg after usbout");
}

static int oplus_voocphy_get_ichg_devation(void)
{
	int cp_ibus_devation = 0;
	int main_cp_ibus = 0;
	int slave_cp_ibus = 0;

	if (!g_voocphy_chip)
		return 0;

	main_cp_ibus = oplus_voocphy_get_ichg();
	slave_cp_ibus = oplus_voocphy_get_slave_ichg();

	if (main_cp_ibus >= slave_cp_ibus) {
		cp_ibus_devation = main_cp_ibus - slave_cp_ibus;
	} else {
		cp_ibus_devation = slave_cp_ibus - main_cp_ibus;
	}

	voocphy_err("cp ibus devation = %d\n", cp_ibus_devation);
	return cp_ibus_devation;
}

static bool oplus_voocphy_check_slave_cp_status(void)
{
	int count = 0;
	int i;
	u8 slave_cp_status = 0;

	if (!g_voocphy_chip)
		return false;

	if (g_voocphy_chip->slave_ops && g_voocphy_chip->slave_ops->get_cp_status) {
		for (i=0; i<3; i++) {
			oplus_voocphy_slave_get_chg_enable(g_voocphy_chip, &slave_cp_status);
			if (oplus_voocphy_get_slave_ichg() < 500
					|| g_voocphy_chip->slave_ops->get_cp_status(g_voocphy_chip) == 0
					|| oplus_voocphy_get_ichg_devation() > 800) {
				count = count + 1;
			} else {
				count = 0;
			}
			msleep(10);
		}

		if (count >= 3) {
			voocphy_err("count >= 3, return false, slave cp is in trouble!\n");
			return false;
		} else {
			voocphy_err("count < 3, return true, slave cp is not in trouble!\n");
			return true;
		}
	} else {
		voocphy_err("slave cp don't exist, return false!\n");
		return false;
	}
}

static void oplus_voocphy_check_chg_out_work_func(void)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int chg_vol = 0;

	if (!chip) {
		voocphy_err("g_voocphy_chip is null\n");
		return;
	}

	chg_vol = oplus_chg_get_charger_voltage();
	if (chg_vol >= 0 && chg_vol < 2000) {
		if (chip->fastchg_start == false ) {
			chip->fastchg_notify_status = FAST_NOTIFY_UNKNOW;
		}
		chip->fastchg_to_normal = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_dummy_start = false;
		chip->fastchg_reactive = false;
		oplus_voocphy_set_user_exit_fastchg(false);

		oplus_chg_clear_chargerid_info();
		oplus_voocphy_basetimer_monitor_stop(g_voocphy_chip);

		voocphy_info("chg_vol = %d %d\n", chg_vol, chip->fastchg_start);
		oplus_chg_wake_update_work();
	}
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_CHG_OUT_CHECK, VOOC_CHG_OUT_CHECK_TIME);
	voocphy_info("notify chg work chg_vol = %d\n", chg_vol);
}

int oplus_voocphy_chg_out_check_event_handle(unsigned long data)
{
	int status = VOOCPHY_SUCCESS;

	oplus_voocphy_check_chg_out_work_func();

	return status;
}

bool oplus_voocphy_check_fastchg_real_allow(void)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int batt_soc = 0;
	int batt_temp = 0;
	bool ret = false;

	batt_soc = oplus_chg_get_ui_soc();			// get batt soc
	if (chip->batt_fake_soc) {
		batt_soc = chip->batt_fake_soc;
	}
	//batt_soc /= 100;
	batt_temp = oplus_chg_get_chg_temperature(); //get batt temp
	if (chip->batt_fake_temp) {
		batt_temp = chip->batt_fake_temp;
	}

	if (batt_soc >= VOOC_LOW_SOC && batt_soc <= VOOC_HIGH_SOC
	    && batt_temp >= VOOC_LOW_TEMP && batt_temp <= VOOC_HIGH_TEMP)
		ret = true;
	else
		ret = false;

	voocphy_info("ret:%d, temp:%d, soc:%d", ret, batt_temp, batt_soc);

	return ret;
}
bool oplus_voocphy_get_real_fastchg_allow(void)
{
	if(!g_voocphy_chip) {
		return false;
	} else {
		g_voocphy_chip->fastchg_real_allow = oplus_voocphy_check_fastchg_real_allow();
		return g_voocphy_chip->fastchg_real_allow;
	}
}

int oplus_voocphy_fastchg_check_process_handle(unsigned long enable)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	bool bPlugIn = false;

	bPlugIn = oplus_chg_stats();
	chip->fastchg_real_allow = oplus_voocphy_check_fastchg_real_allow();

	voocphy_info( "fastchg_to_warm[%d %d] fastchg_dummy_start[%d] fastchg_err_commu[%d] fastchg_check_stop[%d] enable[%d] bPlugIn[%d] fastchg_real_allow[%d]\n",
	              chip->fastchg_to_warm, oplus_voocphy_get_fastchg_to_warm(), chip->fastchg_dummy_start, chip->fastchg_err_commu,
	              chip->fastchg_check_stop, enable, bPlugIn, chip->fastchg_real_allow);
	if ((chip->fastchg_to_warm || oplus_voocphy_get_fastchg_to_warm()
	     || chip->fastchg_dummy_start
	     || chip->fastchg_err_commu)
	    && (!chip->fastchg_check_stop)
	    && enable && bPlugIn) {
		if (chip->fastchg_real_allow == true) {
			if (chip->fastchg_to_warm == false) {
				chip->fastchg_reactive = true;
			}
			oplus_voocphy_reset_variables(chip);
			oplus_voocphy_reactive_voocphy(chip); //re_active voocphy
		} else {
			voocphy_info( "[vooc_monitor] VOOC_THREAD_TIMER_FASTCHG_CHECK start for self fastchg check time start");
			status = oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
		}
	} else {
		chip->fastchg_check_stop = true;
		status = oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_FASTCHG_CHECK, VOOC_FASTCHG_CHECK_TIME);
		voocphy_info( "[vooc_monitor] VOOC_THREAD_TIMER_FASTCHG_CHECK stop fastchg check time stop");
	}

	return status;
}

int oplus_voocphy_commu_process_handle(unsigned long enable)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	unsigned long long duration = 1500;

	//status = BattMngr_WaitLockAcquire(
	//		&(chip->voocphy_commu_wait_lock), NULL);
	if (status != VOOCPHY_SUCCESS) {
		voocphy_info("get commu wait lock fail");
		return status;
	}

	//note exception for systrace
	//trace_oplus_tp_sched_change_ux(duration, task_cpu(current));
	//trace_oplus_tp_sched_change_ux(0, task_cpu(current));
	voocphy_info("dur time %lld usecs\n", duration);

	if (enable && chip->fastchg_commu_stop == false) {
		voocphy_info("[vooc_monitor] commu timeout");
		chip->fastchg_start = false;
		oplus_voocphy_reset_fastchg_after_usbout();
		status = oplus_voocphy_monitor_stop(chip);
		status |= oplus_voocphy_reset_voocphy();
		oplus_chg_set_charger_type_unknown();
	} else {
		chip->fastchg_commu_stop = true;
		status = oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_COMMU, VOOC_COMMU_EVENT_TIME);
		voocphy_info("[vooc_monitor] commu time stop [%d]", chip->fastchg_notify_status);
	}

	return status;
}

#define OPEN_SUB_CP_THRE_CURR 2000
#define CLOSE_SUB_CP_THRE_CURR 1500
#define TARGET_CURR_OFFSET_SUB_CP_THR 100
int oplus_voocphy_curr_event_handle(unsigned long data)
{
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	int status = VOOCPHY_SUCCESS;
	int vbat_temp_cur = 0;
	static unsigned char curr_over_count = 0;
	static unsigned char chg_curr_over_count = 0;
	static unsigned char term_curr_over_count = 0;
	static unsigned char curr_curve_pwd_count = 10;
	u8 slave_cp_enable;
	bool pd_svooc_status;

	if (chip->fastchg_monitor_stop == true) {
		voocphy_info( "oplus_voocphy_curr_event_handle ignore");
		return status;
	}

	if ((chip->fastchg_notify_status & 0XFF) == FAST_NOTIFY_PRESENT) {
		curr_over_count = 0;
		chg_curr_over_count = 0;
		term_curr_over_count = 0;
		curr_curve_pwd_count = 10;
	}

	vbat_temp_cur = chip->icharging;

	if(chip->voocphy_dual_cp_support) {
		oplus_voocphy_slave_get_chg_enable(chip, &slave_cp_enable);
		voocphy_err("slave_status = %d\n", slave_cp_enable);
		if (chip->cp_ichg > chip->slave_cp_enable_thr
		    && slave_cp_enable == 0
		    && slave_trouble_count <= 1
		    && (chip->adapter_type == ADAPTER_SVOOC
		        || chip->adapter_type == ADAPTER_VOOC20
		        ||chip->adapter_type == ADAPTER_VOOC30)) {
			if (chip->adapter_type == ADAPTER_SVOOC && slave_trouble_count == 1) {
				oplus_voocphy_slave_hw_setting(chip, SETTING_REASON_SVOOC);
			} else if ((chip->adapter_type == ADAPTER_VOOC20
		        ||chip->adapter_type == ADAPTER_VOOC30) && slave_trouble_count == 1) {
				oplus_voocphy_slave_hw_setting(chip, SETTING_REASON_VOOC);
			}
			voocphy_err("chip->cp_vbus = %d chip->cp_vbat = %d!!!!!\n", chip->cp_vbus, chip->cp_vbat);
			oplus_voocphy_slave_set_chg_enable(chip, true);
			msleep(200);
			if (oplus_voocphy_check_slave_cp_status()) {
				voocphy_err("slave cp is normal, then set current!\n");
			}else {
				slave_trouble_count = slave_trouble_count + 1;
				voocphy_err("slave cp is abnormal, disable it!\n");
				oplus_voocphy_slave_set_chg_enable(chip, false);
				oplus_voocphy_reset_slave_cp(chip);
			}
		}

		if (slave_trouble_count == 1) {
			if (chip->current_max > 30) {
				chip->current_max = 30;
				if (!chip->ap_need_change_current)
					chip->ap_need_change_current = oplus_voocphy_set_fastchg_current(chip);
			}
		}

		if (chip->cp_ichg < chip->slave_cp_disable_thr_high
		    && slave_cp_enable == 1
		    && (chip->adapter_type == ADAPTER_SVOOC
		        || chip->adapter_type == ADAPTER_VOOC20
		        ||chip->adapter_type == ADAPTER_VOOC30)) {
			if(disable_sub_cp_count == 3) {
				voocphy_err("Ibus < 1.5A 3 times, disable sub cp!\n");
				oplus_voocphy_slave_set_chg_enable(chip, false);
				disable_sub_cp_count = 0;
			} else {
				disable_sub_cp_count = disable_sub_cp_count + 1;
				voocphy_err("Ibus < 1.5A count = %d\n", disable_sub_cp_count);
			}
		} else {
			disable_sub_cp_count = 0;
			voocphy_err("Discontinuity satisfies the condition, count = 0!\n");
		}
	}

	voocphy_info( "[oplus_voocphy_curr_event_handle] current: %d, vbatt: %d btb: %d current_pwd[%d %d]",
	              vbat_temp_cur, chip->gauge_vbatt, chip->btb_temp_over,
	              chip->curr_pwd_count, chip->current_pwd);

	pd_svooc_status = oplus_voocphy_get_pdsvooc_adapter_config(chip);
	if (oplus_chg_check_pd_svooc_adapater()) {
		voocphy_err("pd_svooc_status = %d, chip->cp_ichg = %d\n", pd_svooc_status, chip->cp_ichg);
		if (pd_svooc_status == true && chip->cp_ichg > 1000) {
			voocphy_err("IBUS > 1A set 0x5 to 0x28!\n");
			oplus_voocphy_set_pdsvooc_adapter_config(chip, false);
		}
	}

	if (!chip->btb_temp_over) {//non btb temp over
		if (vbat_temp_cur < -2000) { 	//BATT OUPUT 2000MA
			curr_over_count++;
			if (curr_over_count > 3) {
				voocphy_info("vcurr low than -2000mA\n");
				oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_ABSENT);
			}
		} else {
			curr_over_count = 0;
		}

		if (vbat_temp_cur > VOOC_OVER_CURRENT_VALUE) {
			chg_curr_over_count++;
			if (chg_curr_over_count > 7) {
				oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_BAD_CONNECTED);
			}
		} else {
			chg_curr_over_count = 0;
		}

		if (((vbat_temp_cur < LOW_CURRENT_TERM_LO) && (chip->gauge_vbatt > LOW_CURRENT_TERM_VBAT_LO))
		    || ((vbat_temp_cur < LOW_CURRENT_TERM_HI) && (chip->gauge_vbatt > LOW_CURRENT_TERM_VBAT_HI))) {
			term_curr_over_count++;
			if (term_curr_over_count > 5) {
				voocphy_info( "curr ntime fastchg full %d, ",chip->gauge_vbatt, vbat_temp_cur);
				oplus_voocphy_set_status_and_notify_ap(g_voocphy_chip, FAST_NOTIFY_FULL);
			}
		} else {
			term_curr_over_count = 0;
		}

		if (vbat_temp_cur/2 > chip->current_pwd && chip->adapter_type == ADAPTER_SVOOC) {
			if (chip->curr_pwd_count) {
				chip->curr_pwd_count--;
			} else {
				voocphy_info("FAST_NOTIFY_ADAPTER_COPYCAT for over current");
				//oplus_voocphy_set_status_and_notify_ap(FAST_NOTIFY_ADAPTER_COPYCAT);
			}
		}
		if ((chip->gauge_vbatt > BATT_PWD_VOL_THD1)
		    && (vbat_temp_cur > BATT_PWD_CURR_THD1)) {
			curr_curve_pwd_count--;
			if (curr_curve_pwd_count == 0) {
				voocphy_info("FAST_NOTIFY_ADAPTER_COPYCAT for over vbatt && ibat[%d %d]\n", BATT_PWD_VOL_THD1, BATT_PWD_CURR_THD1);
				//oplus_voocphy_set_status_and_notify_ap(FAST_NOTIFY_ADAPTER_COPYCAT);
			}
		} else {
			curr_curve_pwd_count = 10;
		}

		status = oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_CURR, VOOC_CURR_EVENT_TIME);

	}

	return status;
}

int oplus_voocphy_monitor_start(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;
	if (!chip) {
		voocphy_info( "chip is null");
		return VOOCPHY_EFATAL;
	}

	chip->fastchg_monitor_stop = false;
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_SAFE, VOOC_SAFE_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_BTB, VOOC_BTB_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_VOL, VOOC_VOL_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_TEMP, VOOC_TEMP_EVENT_TIME);
	//oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_SOC, VOOC_SOC_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_CURR, VOOC_CURR_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_AP, VOOC_AP_EVENT_TIME);
	oplus_voocphy_monitor_timer_start(chip, VOOC_THREAD_TIMER_IBUS_CHECK, VOOC_IBUS_CHECK_TIME);

	voocphy_info( "vooc_monitor hrstart success");
	return status;
}

int oplus_voocphy_monitor_stop(struct oplus_voocphy_manager *chip)
{
	int status = VOOCPHY_SUCCESS;

	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_AP, VOOC_AP_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_CURR, VOOC_CURR_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_SOC, VOOC_SOC_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_VOL, VOOC_VOL_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_TEMP, VOOC_TEMP_EVENT_TIME);
	//oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_BTB, VOOC_BTB_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_SAFE, VOOC_SAFE_EVENT_TIME);
	oplus_voocphy_monitor_timer_stop(chip, VOOC_THREAD_TIMER_IBUS_CHECK, VOOC_IBUS_CHECK_TIME);

	chip->fastchg_monitor_stop = true;
	voocphy_info( "vooc_monitor stop hrtimer success");
	return status;
}

void voocphy_service(struct work_struct *work)
{
	struct oplus_voocphy_manager *chip
	    = container_of(work, struct oplus_voocphy_manager,
	                   voocphy_service_work.work);

	if (!chip) {
		printk(KERN_ERR "[%s] chip null\n", __func__);
		return ;
	}

	chip->icharging = -oplus_gauge_get_batt_current();
	oplus_voocphy_print_dbg_info(chip);
}

bool oplus_vooc_wake_voocphy_service_work(struct oplus_voocphy_manager *chip, int request)
{
	chip->voocphy_request= request;
	schedule_delayed_work(&chip->voocphy_service_work, 0);
	voocphy_dbg("request %d \n", request);
	return true;
}

bool oplus_vooc_wake_monitor_work(struct oplus_voocphy_manager *chip)
{
	schedule_delayed_work(&chip->monitor_work, 0);
	return true;
}

bool oplus_voocphy_chip_is_null(void)
{
	if (!g_voocphy_chip)
		return true;
	else
		return false;
}

void oplus_voocphy_init(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}

	oplus_voocphy_parse_batt_curves(chip);

	oplus_voocphy_variables_init(chip);

	oplus_voocphy_monitor_timer_init(chip);

	INIT_DELAYED_WORK(&(chip->voocphy_service_work), voocphy_service);
	INIT_DELAYED_WORK(&(chip->notify_fastchg_work), oplus_voocphy_handle_voocphy_status);
	INIT_DELAYED_WORK(&(chip->monitor_work), oplus_voocphy_monitor_process_events);

	g_voocphy_chip = chip;
	#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
	pm_qos_add_request(&pm_qos_req, PM_QOS_CPU_DMA_LATENCY, 1000);
	#endif
}

void oplus_voocphy_slave_init(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		voocphy_info("%s chip null\n", __func__);
		return;
	}
	g_voocphy_chip->slave_client = chip->slave_client;
	g_voocphy_chip->slave_dev = chip->slave_dev;
	g_voocphy_chip->slave_ops = chip->slave_ops;
}

bool oplus_voocphy_get_fastchg_ing(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->fastchg_ing;
	}
}

bool oplus_voocphy_get_external_gauge_support(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->external_gauge_support;
	}
}

bool oplus_voocphy_get_fastchg_start(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->fastchg_start;
	}
}

bool oplus_voocphy_get_fastchg_to_normal(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->fastchg_to_normal;
	}
}

bool oplus_voocphy_get_fastchg_to_warm(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->fastchg_to_warm;
	}
}

bool oplus_voocphy_get_fastchg_dummy_start(void)
{
	if (!g_voocphy_chip) {
		return false;
	} else {
		return g_voocphy_chip->fastchg_dummy_start;
	}
}

int oplus_voocphy_get_fast_chg_type(void)
{
	if (!g_voocphy_chip) {
		return FASTCHG_CHARGER_TYPE_UNKOWN;
	} else {
		return g_voocphy_chip->fast_chg_type;
	}
}

int oplus_voocphy_get_switch_gpio_val(void)
{
	if (!g_voocphy_chip) {
		return 0;
	} else {
		return gpio_get_value(g_voocphy_chip->switch1_gpio);
	}
}

void oplus_voocphy_switch_fast_chg(void)
{
	if (!g_voocphy_chip) {
		return;
	}

	if (oplus_vooc_get_vooc_switch_val() == 1) {
		return;
	}

	if (oplus_voocphy_get_fastchg_to_normal()
	    || (!oplus_voocphy_get_real_fastchg_allow()
	        && oplus_voocphy_get_fastchg_dummy_start())
	    || oplus_voocphy_user_exit_fastchg(g_voocphy_chip)) {
		voocphy_info("not allow to fastchg\n");
		cpuboost_charge_event(CPU_CHG_FREQ_STAT_AUTO);
		#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
		pm_qos_update_request(&pm_qos_req, PM_QOS_DEFAULT_VALUE);
		#endif
	} else {
		if (oplus_voocphy_get_real_fastchg_allow() == false
		    && oplus_voocphy_get_fastchg_to_warm() == true) {
			chg_err(" fastchg_allow false, to_warm true, don't switch to vooc mode\n");
		} else {
			oplus_voocphy_basetimer_monitor_start(g_voocphy_chip);
			cpuboost_charge_event(CPU_CHG_FREQ_STAT_UP);
			#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
			pm_qos_update_request(&pm_qos_req, 400);
			#endif
			oplus_voocphy_init_vooc(g_voocphy_chip);
			oplus_voocphy_reset_ibus_trouble_flag();
			oplus_voocphy_slave_init_vooc(g_voocphy_chip);
			oplus_voocphy_set_switch_mode(VOOC_CHARGER_MODE);
			oplus_voocphy_send_handshake(g_voocphy_chip);
		}
	}
}

static const char * const strategy_soc[] = {
	[BATT_SOC_0_TO_50]	= "strategy_soc_0_to_50",
	[BATT_SOC_50_TO_75]	= "strategy_soc_50_to_75",
	[BATT_SOC_75_TO_85]	= "strategy_soc_75_to_85",
	[BATT_SOC_85_TO_90]	= "strategy_soc_85_to_90",
};

static const char * const strategy_temp[] = {
	[BATT_SYS_CURVE_TEMP0_TO_TEMP50]	= "strategy_temp_0_to_50",
	[BATT_SYS_CURVE_TEMP50_TO_TEMP120]	= "strategy_temp_50_to_120",
	[BATT_SYS_CURVE_TEMP120_TO_TEMP160]	= "strategy_temp_120_to_160",
	[BATT_SYS_CURVE_TEMP160_TO_TEMP430]	= "strategy_temp_160_to_430",
};

static int oplus_voocphy_parse_svooc_batt_curves(struct oplus_voocphy_manager *chip)
{
	struct device_node *node, *svooc_node, *soc_node;
	int rc = 0, i, j, length;

	node = chip->dev->of_node;

	svooc_node = of_get_child_by_name(node, "svooc_charge_strategy");
	if (!svooc_node) {
		voocphy_info("Can not find svooc_charge_strategy node\n");
		return -EINVAL;
	}

	for (i=0; i<BATT_SOC_90_TO_100; i++) {
		soc_node = of_get_child_by_name(svooc_node, strategy_soc[i]);
		if (!soc_node) {
			voocphy_info("Can not find %s node\n", strategy_soc[i]);
			return -EINVAL;
		}

		for (j=0; j<BATT_SYS_CURVE_MAX; j++) {
			rc = of_property_count_elems_of_size(soc_node, strategy_temp[j], sizeof(u32));
			if (rc < 0) {
				voocphy_info("Count %s failed, rc=%d\n", strategy_temp[j], rc);
				return rc;
			}

			length = rc;

			switch(i) {
			case BATT_SOC_0_TO_50:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)svooc_curves_soc0_2_50[j].batt_sys_curve,
				                                length);
				svooc_curves_soc0_2_50[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_50_TO_75:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)svooc_curves_soc50_2_75[j].batt_sys_curve,
				                                length);
				svooc_curves_soc50_2_75[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_75_TO_85:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)svooc_curves_soc75_2_85[j].batt_sys_curve,
				                                length);
				svooc_curves_soc75_2_85[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_85_TO_90:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)svooc_curves_soc85_2_90[j].batt_sys_curve,
				                                length);
				svooc_curves_soc85_2_90[j].sys_curv_num = length/5;
				break;
			default:
				break;
			}
		}
	}

	return rc;
}

static int oplus_voocphy_parse_vooc_batt_curves(struct oplus_voocphy_manager *chip)
{
	struct device_node *node, *vooc_node, *soc_node;
	int rc = 0, i, j, length;

	node = chip->dev->of_node;

	vooc_node = of_get_child_by_name(node, "vooc_charge_strategy");
	if (!vooc_node) {
		voocphy_info("Can not find vooc_charge_strategy node\n");
		return -EINVAL;
	}

	for (i=0; i<BATT_SOC_90_TO_100; i++) {
		soc_node = of_get_child_by_name(vooc_node, strategy_soc[i]);
		if (!soc_node) {
			voocphy_info("Can not find %s node\n", strategy_soc[i]);
			return -EINVAL;
		}

		for (j=0; j<BATT_SYS_CURVE_MAX; j++) {
			rc = of_property_count_elems_of_size(soc_node, strategy_temp[j], sizeof(u32));
			if (rc < 0) {
				voocphy_info("Count %s failed, rc=%d\n", strategy_temp[j], rc);
				return rc;
			}

			length = rc;

			switch(i) {
			case BATT_SOC_0_TO_50:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)vooc_curves_soc0_2_50[j].batt_sys_curve,
				                                length);
				vooc_curves_soc0_2_50[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_50_TO_75:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)vooc_curves_soc50_2_75[j].batt_sys_curve,
				                                length);
				vooc_curves_soc50_2_75[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_75_TO_85:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)vooc_curves_soc75_2_85[j].batt_sys_curve,
				                                length);
				vooc_curves_soc75_2_85[j].sys_curv_num = length/5;
				break;
			case BATT_SOC_85_TO_90:
				rc = of_property_read_u32_array(soc_node, strategy_temp[j],
				                                (u32 *)vooc_curves_soc85_2_90[j].batt_sys_curve,
				                                length);
				vooc_curves_soc85_2_90[j].sys_curv_num = length/5;
				break;
			default:
				break;
			}
		}
	}

	return 0;
}

int oplus_voocphy_parse_batt_curves(struct oplus_voocphy_manager *chip)
{
	struct device_node *node;
	int rc = 0, i, j;
	int length = BATT_SYS_ARRAY*7;
	u32 data[BATT_SYS_ARRAY*7] = {0};

	node = chip->dev->of_node;

	chip->voocphy_dual_cp_support = of_property_read_bool(node,
	                                "qcom,voocphy_dual_cp_support");
	voocphy_info("voocphy_dual_cp_support = %d\n", chip->voocphy_dual_cp_support);

	rc = of_property_read_u32(node, "qcom,slave_cp_enable_thr",
	                          &chip->slave_cp_enable_thr);
	if (rc) {
		chip->slave_cp_enable_thr = 1900;
	}
	voocphy_info("slave_cp_enable_thr is %d\n", chip->slave_cp_enable_thr);

	rc = of_property_read_u32(node, "qcom,slave_cp_disable_thr_high",
	                          &chip->slave_cp_disable_thr_high);
	if (rc) {
		chip->slave_cp_disable_thr_high = 1600;
	}
	voocphy_info("slave_cp_disable_thr_high is %d\n", chip->slave_cp_disable_thr_high);

	rc = of_property_read_u32(node, "qcom,slave_cp_disable_thr_low",
	                          &chip->slave_cp_disable_thr_low);
	if (rc) {
		chip->slave_cp_disable_thr_low = 1400;
	}
	voocphy_info("slave_cp_disable_thr_low is %d\n", chip->slave_cp_disable_thr_low);

	rc = of_property_read_u32(node, "qcom,voocphy_current_default",
	                          &chip->current_default);
	if (rc) {
		chip->current_default = 30;
	} else {
		voocphy_info("qcom,voocphy_current_default is %d\n",
		             chip->current_default);
	}

	rc = of_property_read_u32(node, "qcom,vooc_little_cool_temp",
	                          &chip->vooc_little_cool_temp);
	if (rc) {
		chip->vooc_little_cool_temp = 160;
	} else {
		voocphy_info("qcom,vooc_little_cool_temp is %d\n", chip->vooc_little_cool_temp);
	}
	chip->vooc_little_cool_temp_default = chip->vooc_little_cool_temp;

	rc = of_property_read_u32(node, "qcom,vooc-cool-temp",
	                          &chip->vooc_cool_temp);
	if (rc) {
		chip->vooc_cool_temp = 120;
	} else {
		voocphy_info("qcom,vooc_cool_temp is %d\n", chip->vooc_cool_temp);
	}
	chip->vooc_cool_temp_default = chip->vooc_cool_temp;

	rc = of_property_read_u32(node, "qcom,vooc_little_cold_temp",
	                          &chip->vooc_little_cold_temp);
	if (rc) {
		chip->vooc_little_cold_temp = 50;
	} else {
		voocphy_info("qcom,vooc_little_cold_temp is %d\n", chip->vooc_little_cold_temp);
	}
	chip->vooc_little_cold_temp_default = chip->vooc_little_cold_temp;


	rc = of_property_read_u32(node, "qcom,vooc_normal_low_temp",
	                          &chip->vooc_normal_low_temp);
	if (rc) {
		chip->vooc_normal_low_temp = 250;
	} else {
		voocphy_info("qcom,vooc_normal_low_temp is %d\n", chip->vooc_normal_low_temp);
	}
	chip->vooc_normal_low_temp_default = chip->vooc_normal_low_temp;

	rc = of_property_read_u32(node, "qcom,vooc_multistep_initial_batt_temp",
	                          &chip->vooc_multistep_initial_batt_temp);
	if (rc) {
		chip->vooc_multistep_initial_batt_temp = 305;
	} else {
		voocphy_info("qcom,vooc_multistep_initial_batt_temp is %d\n",
		             chip->vooc_multistep_initial_batt_temp);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy_normal_current",
	                          &chip->vooc_strategy_normal_current);
	if (rc) {
		chip->vooc_strategy_normal_current = 0x03;
	} else {
		voocphy_info("qcom,vooc_strategy_normal_current is %d\n",
		             chip->vooc_strategy_normal_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp1",
	                          &chip->vooc_strategy1_batt_low_temp1);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp1  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_low_temp1 is %d\n",
		             chip->vooc_strategy1_batt_low_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp2",
	                          &chip->vooc_strategy1_batt_low_temp2);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp2 = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_low_temp2 is %d\n",
		             chip->vooc_strategy1_batt_low_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_low_temp0",
	                          &chip->vooc_strategy1_batt_low_temp0);
	if (rc) {
		chip->vooc_strategy1_batt_low_temp0 = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_low_temp0 is %d\n",
		             chip->vooc_strategy1_batt_low_temp0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp0",
	                          &chip->vooc_strategy1_batt_high_temp0);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp0 = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_high_temp0 is %d\n",
		             chip->vooc_strategy1_batt_high_temp0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp1",
	                          &chip->vooc_strategy1_batt_high_temp1);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp1 = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_high_temp1 is %d\n",
		             chip->vooc_strategy1_batt_high_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_batt_high_temp2",
	                          &chip->vooc_strategy1_batt_high_temp2);
	if (rc) {
		chip->vooc_strategy1_batt_high_temp2 = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy1_batt_high_temp2 is %d\n",
		             chip->vooc_strategy1_batt_high_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current0",
	                          &chip->vooc_strategy1_high_current0);
	if (rc) {
		chip->vooc_strategy1_high_current0  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_high_current0 is %d\n",
		             chip->vooc_strategy1_high_current0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current1",
	                          &chip->vooc_strategy1_high_current1);
	if (rc) {
		chip->vooc_strategy1_high_current1  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_high_current1 is %d\n",
		             chip->vooc_strategy1_high_current1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_high_current2",
	                          &chip->vooc_strategy1_high_current2);
	if (rc) {
		chip->vooc_strategy1_high_current2  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_high_current2 is %d\n",
		             chip->vooc_strategy1_high_current2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current2",
	                          &chip->vooc_strategy1_low_current2);
	if (rc) {
		chip->vooc_strategy1_low_current2  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_low_current2 is %d\n",
		             chip->vooc_strategy1_low_current2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current1",
	                          &chip->vooc_strategy1_low_current1);
	if (rc) {
		chip->vooc_strategy1_low_current1  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_low_current1 is %d\n",
		             chip->vooc_strategy1_low_current1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy1_low_current0",
	                          &chip->vooc_strategy1_low_current0);
	if (rc) {
		chip->vooc_strategy1_low_current0  = chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy1_low_current0 is %d\n",
		             chip->vooc_strategy1_low_current0);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp1",
	                          &chip->vooc_strategy2_batt_up_temp1);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp1  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_temp1 is %d\n",
		             chip->vooc_strategy2_batt_up_temp1);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_down_temp2",
	                          &chip->vooc_strategy2_batt_up_down_temp2);
	if (rc) {
		chip->vooc_strategy2_batt_up_down_temp2  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_down_temp2 is %d\n",
		             chip->vooc_strategy2_batt_up_down_temp2);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp3",
	                          &chip->vooc_strategy2_batt_up_temp3);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp3  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_temp3 is %d\n",
		             chip->vooc_strategy2_batt_up_temp3);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_down_temp4",
	                          &chip->vooc_strategy2_batt_up_down_temp4);
	if (rc) {
		chip->vooc_strategy2_batt_up_down_temp4  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_down_temp4 is %d\n",
		             chip->vooc_strategy2_batt_up_down_temp4);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp5",
	                          &chip->vooc_strategy2_batt_up_temp5);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp5  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_temp5 is %d\n",
		             chip->vooc_strategy2_batt_up_temp5);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_batt_up_temp6",
	                          &chip->vooc_strategy2_batt_up_temp6);
	if (rc) {
		chip->vooc_strategy2_batt_up_temp6  = chip->vooc_multistep_initial_batt_temp;
	} else {
		voocphy_info("qcom,vooc_strategy2_batt_up_temp6 is %d\n",
		             chip->vooc_strategy2_batt_up_temp6);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high0_current",
	                          &chip->vooc_strategy2_high0_current);
	if (rc) {
		chip->vooc_strategy2_high0_current	= chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy2_high0_current is %d\n",
		             chip->vooc_strategy2_high0_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high1_current",
	                          &chip->vooc_strategy2_high1_current);
	if (rc) {
		chip->vooc_strategy2_high1_current	= chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy2_high1_current is %d\n",
		             chip->vooc_strategy2_high1_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high2_current",
	                          &chip->vooc_strategy2_high2_current);
	if (rc) {
		chip->vooc_strategy2_high2_current	= chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy2_high2_current is %d\n",
		             chip->vooc_strategy2_high2_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_strategy2_high3_current",
	                          &chip->vooc_strategy2_high3_current);
	if (rc) {
		chip->vooc_strategy2_high3_current	= chip->vooc_strategy_normal_current;
	} else {
		voocphy_info("qcom,vooc_strategy2_high3_current is %d\n",
		             chip->vooc_strategy2_high3_current);
	}

	rc = of_property_read_u32(node, "qcom,vooc_batt_over_low_temp",
	                          &chip->vooc_batt_over_low_temp);
	if (rc) {
		chip->vooc_batt_over_low_temp = -EINVAL;
	} else {
		voocphy_info("qcom,vooc_batt_over_low_temp is %d\n",
		             chip->vooc_batt_over_low_temp);
	}

	rc = of_property_read_u32(node, "qcom,vooc_batt_over_high_temp",
	                          &chip->vooc_batt_over_high_temp);
	if (rc) {
		chip->vooc_batt_over_high_temp = -EINVAL;
	} else {
		voocphy_info("qcom,vooc_batt_over_high_temp is %d\n",
		             chip->vooc_batt_over_high_temp);
	}

	rc = of_property_read_u32_array(node, "svooc_batt_sys_curve",
	                                data, length);
	if (rc < 0) {
		voocphy_info("Couldn't read svooc_batt_sys_curve rc = %d\n", rc);
		return rc;
	}

	for(i=0; i<BATT_SYS_ARRAY; i++) {
		for(j=0; j<7; j++) {
			svooc_batt_sys_curve[i][j] = data[i*7 + j];
		}
	}

	rc = of_property_read_u32_array(node, "vooc_batt_sys_curve",
	                                data, length);
	if (rc < 0) {
		voocphy_info("Couldn't read vooc_batt_sys_curve rc = %d\n", rc);
		return rc;
	}

	for(i=0; i<BATT_SYS_ARRAY; i++) {
		for(j=0; j<7; j++) {
			vooc_batt_sys_curve[i][j] = data[i*7 + j];
		}
	}

	chip->external_gauge_support = of_property_read_bool(node,
	                               "qcom,external_gauge_support");
	voocphy_info("external_gauge_support = %d\n", chip->external_gauge_support);

	rc = of_property_count_elems_of_size(node, "qcom,svooc_cool_down_current_limit", sizeof(u32));
	if (rc < 0) {
		voocphy_info("Count svooc_cool_down_current_limit failed, rc=%d\n", rc);
		return rc;
	}

	length = rc;

	rc = of_property_read_u32_array(node, "qcom,svooc_cool_down_current_limit",
	                                chip->svooc_cool_down_current_limit,
	                                length);
	if (rc < 0) {
		voocphy_info("parse svooc_cool_down_current_limit failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_count_elems_of_size(node, "qcom,vooc_cool_down_current_limit", sizeof(u32));
	if (rc < 0) {
		voocphy_info("Count vooc_cool_down_current_limit failed, rc=%d\n", rc);
		return rc;
	}

	length = rc;

	rc = of_property_read_u32_array(node, "qcom,vooc_cool_down_current_limit",
	                                chip->vooc_cool_down_current_limit,
	                                length);
	if (rc < 0) {
		voocphy_info("parse vooc_cool_down_current_limit failed, rc=%d\n", rc);
		return rc;
	}

	oplus_voocphy_parse_svooc_batt_curves(chip);

	oplus_voocphy_parse_vooc_batt_curves(chip);

	return 0;
}

void oplus_voocphy_set_pdqc_config(void)
{
	if (!g_voocphy_chip)
		return;

	oplus_voocphy_hw_setting(g_voocphy_chip, SETTING_REASON_PDQC);
	if(g_voocphy_chip->voocphy_dual_cp_support)
		oplus_voocphy_slave_hw_setting(g_voocphy_chip, SETTING_REASON_PDQC);
}

void oplus_voocphy_set_pdsvooc_adapter_config(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip)
		return;

	if (chip->ops && chip->ops->set_pd_svooc_config) {
		chip->ops->set_pd_svooc_config(chip, enable);
	} else {
		return;
	}
}

bool oplus_voocphy_get_pdsvooc_adapter_config(struct oplus_voocphy_manager *chip)
{
	if (!chip)
		return false;

	if (chip->ops && chip->ops->get_pd_svooc_config) {
		return chip->ops->get_pd_svooc_config(chip);
	} else {
		return false;
	}
}

void oplus_voocphy_reset_slave_cp(struct oplus_voocphy_manager *chip)
{
	if (!chip)
		return;

	if (chip->slave_ops && chip->slave_ops->reset_voocphy) {
		chip->slave_ops->reset_voocphy(chip);
	} else {
		return;
	}
}

bool oplus_voocphy_get_dual_cp_support(void)
{
	if (!g_voocphy_chip)
		return false;


	return g_voocphy_chip->voocphy_dual_cp_support;
}

void oplus_voocphy_set_chip(struct oplus_voocphy_manager *chip)
{
	g_voocphy_chip = chip;
}

void oplus_adsp_voocphy_reset_status_when_crash_recover(void)
{
	if (!g_voocphy_chip)
		return ;

	if (g_voocphy_chip->fast_chg_type != FASTCHG_CHARGER_TYPE_UNKOWN) {
		g_voocphy_chip->fastchg_dummy_start = true;
	}
	g_voocphy_chip->fastchg_ing = false;
	g_voocphy_chip->fastchg_start = false;
	g_voocphy_chip->fastchg_to_normal = false;
	g_voocphy_chip->fastchg_to_warm = false;
}

void oplus_adsp_voocphy_clear_status(void)
{
	if (!g_voocphy_chip)
		return;

	g_voocphy_chip->fastchg_dummy_start = false;
	g_voocphy_chip->fastchg_ing = false;
	g_voocphy_chip->fastchg_start = false;
	g_voocphy_chip->fastchg_to_normal = false;
	g_voocphy_chip->fastchg_to_warm = false;
	g_voocphy_chip->fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;
}

void oplus_adsp_voocphy_turn_on(void)
{
	int rc  = 0;
	int try_count = 10;
	struct oplus_voocphy_manager *chip = g_voocphy_chip;
	
	if (!chip) {
		return;
	}

	voocphy_info("oplus_adsp_voocphy_turn_on\n");

	if (chip->ops->adsp_voocphy_enable) {
		do {
			rc = chip->ops->adsp_voocphy_enable(true);
			voocphy_info("%s: try_count = %d, rc =%d\n", __func__, try_count, rc);
		} while ((rc) && (--try_count > 0));
	}
}

void oplus_adsp_voocphy_turn_off(void)
{
	int rc  = 0;
	int try_count = 10;
	struct oplus_voocphy_manager *chip = g_voocphy_chip;

	if (!chip) {
		return;
	}

	voocphy_info("oplus_adsp_voocphy_turn_off\n");

	if (chip->fastchg_start == true)
		chip->fastchg_dummy_start = false;
	chip->fastchg_ing = false;
	chip->fastchg_start = false;
	chip->fastchg_to_normal = false;
	chip->fastchg_to_warm = false;
	chip->fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;

	if (chip->ops->adsp_voocphy_enable) {
		do {
			rc = chip->ops->adsp_voocphy_enable(false);
			voocphy_info("%s: try_count = %d, rc =%d\n", __func__, try_count, rc);
		} while ((rc) && (--try_count > 0));
		if ((try_count > 0) && (!rc)) {
			if (g_oplus_chip && g_oplus_chip->stop_chg == 0) {
				oplus_chg_suspend_charger();
			} else {
				oplus_chg_unsuspend_charger();
			}
		}
	}
}

void oplus_adsp_voocphy_reset(void)
{
	int rc  = 0;
	int try_count = 10;
	struct oplus_voocphy_manager *chip = g_voocphy_chip;

	if (!chip) {
		return;
	}

	voocphy_info("oplus_adsp_voocphy_reset_again\n");

	if (chip->ops->adsp_voocphy_reset_again) {
		do {
			rc = chip->ops->adsp_voocphy_reset_again();
			voocphy_info("%s: try_count = %d, rc =%d\n", __func__, try_count, rc);
		} while ((rc) && (--try_count > 0));
	}
}

int oplus_adsp_voocphy_get_rx_data(void)
{
	if (!g_voocphy_chip)
		return false;


	return g_voocphy_chip->adsp_voocphy_rx_data;
}





