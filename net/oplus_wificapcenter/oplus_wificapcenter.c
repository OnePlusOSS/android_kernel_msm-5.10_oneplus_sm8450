/******************************************************************************
** Copyright (C), 2019-2029, OPLUS Mobile Comm Corp., Ltd
** All rights reserved.
** File: - opluswificap center.c
** Description: wificapcenter (wcc)
**
** Version: 1.0
** Date : 2020/07/05
** Author: XuFenghua@CONNECTIVITY.WIFI.BASIC.CAPCENTER.190453
** TAG: OPLUS_FEATURE_WIFI_CAPCENTER
** ------------------------------- Revision History: ----------------------------
** <author>                                <data>        <version>       <desc>
** ------------------------------------------------------------------------------
 *******************************************************************************/

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <net/route.h>
#include <net/ip.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/version.h>
#include <net/tcp.h>
#include <linux/random.h>
#include <net/sock.h>
#include <net/dst.h>
#include <linux/file.h>
#include <net/tcp_states.h>
#include <linux/netlink.h>
#include <net/sch_generic.h>
#include <net/pkt_sched.h>
#include <net/netfilter/nf_queue.h>
#include <linux/netfilter/xt_state.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_owner.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <net/genetlink.h>

#include <net/oplus/oplus_wfd_wlan.h>

#define LOG_TAG "[oplus_wificapcenter] %s line:%d "
#define debug(fmt, args...) printk(LOG_TAG fmt, __FUNCTION__, __LINE__, ##args)

/*2021/4/15 Xufenghua for GKI2.0 modify ALM ID:1177828, MSG id modify from 0 to MAX*/
/*If response event is closely related with request, response can be same with req to reduce msg*/
enum{
        OPLUS_WIFI_CAP_CENTER_UNDEFINE,
        OPLUS_COMMON_MSG_BASE,
        OPLUS_WIFI_CAP_CENTER_NOTIFY_PID,

        OPLUS_SYNC_MSG_BASE,
        OPLUS_SAMPLE_SYNC_GET,
        OPLUS_SAMPLE_SYNC_GET_NO_RESP,

        OPLUS_SYNC_REMOVE_HE_IE_FROM_PROBE_REQUEST,
        OPLUS_SYNC_DBS_CAPACITY_GET,
        OPLUS_SYNC_PHY_CAPACITY_GET,
        OPLUS_SYNC_SUPPORTED_CHANNELS_GET,
        OPLUS_SYNC_AVOID_CHANNELS_GET,

        OPLUS_ASYNC_MSG_BASE,
        OPLUS_SAMPLE_ASYNC_GET,

        __OPLUS_WIFI_CAP_CENTER_MSG_MAX,
};
#define OPLUS_WIFI_CAP_CENTER_MSG_MAX (__OPLUS_WIFI_CAP_CENTER_MSG_MAX - 1)

enum {
        OPLUS_WIFI_CAP_CENTER_UNSPEC,
        OPLUS_WIFI_CAP_CENTER_SYNC_CMD,
        OPLUS_WIFI_CAP_CENTER_ASYNC_CMD,
        __OPLUS_WIFI_CAP_CENTER_CMD_MAX
};
#define OPLUS_WIFI_CAP_CENTER_CMD_MAX (__OPLUS_WIFI_CAP_CENTER_CMD_MAX - 1)
/*End for GKI2.0 modify*/

static DEFINE_MUTEX(oplus_wcc_sync_nl_mutex);
static DEFINE_MUTEX(oplus_wcc_async_nl_mutex);
static struct ctl_table_header *oplus_table_hrd;
static rwlock_t oplus_sync_nl_lock;
static rwlock_t oplus_async_nl_lock;
static u32 oplus_wcc_debug = 1;
/*user space pid*/
static u32 oplus_sync_nl_pid = 0;
static u32 oplus_async_nl_pid = 0;
/*
2021/4/15 Xufenghua for GKI2.0 modify ALM ID:1177828,
make oplus_sync_nl_sock & oplus_async_nl_sock have different usage and different init place,
check oplus_wcc_genl_init() for detail
*/
/*kernel sock*/
static struct sock *oplus_sync_nl_sock = NULL;
static struct sock *oplus_async_nl_sock = NULL;

static struct timer_list oplus_timer;
static int async_msg_type = 0;

/*2021/4/15 Xufenghua for GKI2.0 modify ALM ID:1177828, MSG id modify from 0 to MAX*/
#define OPLUS_WCC_FAMILY_VERSION        1
#define OPLUS_WCC_SYNC_FAMILY "oplus_wcc_sync"
#define OPLUS_WCC_ASYNC_FAMILY "oplus_wcc_async"
#define NLA_DATA(na)            ((char *)((char*)(na) + NLA_HDRLEN))
#define GENL_ID_GENERATE        0

static int oplus_wcc_sync_genl_rcv(struct sk_buff *skb, struct genl_info *info);
static int oplus_wcc_async_genl_rcv(struct sk_buff *skb, struct genl_info *info);
static inline int genl_msg_prepare_usr_msg(struct sock *oplus_sock, size_t size, struct sk_buff **skbp);
static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len);

/*check msg_type in range of sync & async, 1 stands in range, 0 not in range*/
static int check_msg_in_range(struct sock *nl_sock, int msg_type)
{
        debug("nl_sock: %p, msg_type:%d", nl_sock, msg_type);
        if (msg_type >= OPLUS_COMMON_MSG_BASE && msg_type < OPLUS_SYNC_MSG_BASE) {
                return 1;
        }

        /*not in common part*/
        if (nl_sock == oplus_sync_nl_sock) {
                if (msg_type >= OPLUS_SYNC_MSG_BASE && msg_type< OPLUS_ASYNC_MSG_BASE) {
                        return 1;
                } else {
                        return 0;
                }
        } else if (nl_sock == oplus_async_nl_sock) {
                if (msg_type >= OPLUS_ASYNC_MSG_BASE && msg_type < __OPLUS_WIFI_CAP_CENTER_MSG_MAX) {
                        return 1;
                } else {
                        return 0;
                }
        } else {
                return 0;
        }
}

/* send to user space */
static int oplus_wcc_send_to_user(struct sock *oplus_sock,
        u32 oplus_pid, int msg_type, char *payload, int payload_len)
{
        int ret = 0;
        struct sk_buff *skbuff;
        /*struct nlmsghdr *nlh = NULL;*/
        size_t size;
        void *head;
        /* 2021/11/18 fix coverity check error "UNINIT" CID 158846, init default value */
        pid_t pid = oplus_sync_nl_pid;

        if (!check_msg_in_range(oplus_sock, msg_type)) {
                debug("msg_type:%d not in range\n", msg_type);
                return -1;
        }

        if (oplus_sock == oplus_sync_nl_sock) {
                pid = oplus_sync_nl_pid;
        } else if (oplus_sock == oplus_async_nl_sock) {
                pid = oplus_async_nl_pid;
        }

        /*allocate new buffer cache */
        /*skbuff = alloc_skb(NLMSG_SPACE(payload_len), GFP_ATOMIC);*/
        size = nla_total_size(payload_len);
        ret = genl_msg_prepare_usr_msg(oplus_sock, size, &skbuff);
        if (ret) {
                return ret;
        }

        ret = genl_msg_mk_usr_msg(skbuff, msg_type, payload, payload_len);
        if (oplus_wcc_debug) {
                debug("skb_len:%u, msg_type:%d, payload_len:%d\n", skbuff->len, msg_type, payload_len);
        }
        if (ret) {
                kfree_skb(skbuff);
                return ret;
        }

        head = genlmsg_data(nlmsg_data(nlmsg_hdr(skbuff)));
        genlmsg_end(skbuff, head);

        /* send data */
        ret = genlmsg_unicast(&init_net, skbuff, pid);
        if (ret < 0) {
                printk(KERN_ERR "oplus_wificapcenter: oplus_wcc_send_to_user, can not unicast skbuff, ret = %d\n", ret);
                return -1;
        }

        if (oplus_wcc_debug) {
                debug("skb_len=%u", skbuff->len);
        }
        return 0;
}

static void oplus_wcc_sample_resp(struct sock *oplus_sock, u32 oplus_pid, int msg_type)
{
        int payload[4];
        payload[0] = 5;
        payload[1] = 6;
        payload[2] = 7;
        payload[3] = 8;

        oplus_wcc_send_to_user(oplus_sock, oplus_pid, msg_type, (char *)payload, sizeof(payload));
        if (oplus_wcc_debug) {
                debug("msg_type = %d, sample_resp =%d%d%d%d\n", msg_type, payload[0], payload[1], payload[2], payload[3]);
        }

        return;
}

static int oplus_wcc_sample_sync_get(struct nlattr *nla)
{
        u32 *data = (u32 *)NLA_DATA(nla);

        debug("sample_sync_get: %u%u%u%u\n", data[0], data[1], data[2], data[3]);
        oplus_wcc_sample_resp(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SAMPLE_SYNC_GET);
        return 0;
}

static int oplus_wcc_sample_sync_get_no_resp(struct nlattr *nla)
{
        u32 *data = (u32 *)NLA_DATA(nla);

        debug("sample_sync_get_no_resp: %u%u%u%u\n", data[0], data[1], data[2], data[3]);
        return 0;
}

/*#ifdef OPLUS_FEATURE_WIFI_OPLUSWFD*/
/*XiaZijian@CONNECTIVITY.WIFI.P2P.26106,20200703*/
#define OPLUS_WFD_FREQS_NUM_MAX 100
/*the first one for length*/
static int s_oplus_wfd_freqs[OPLUS_WFD_FREQS_NUM_MAX + 1];
enum oplus_wfd_band
{
	oplus_wfd_band_2g = 0,
	oplus_wfd_band_5g,
	oplus_wfd_band_6g,
	oplus_wfd_band_max
};

static void remove_he_ie_from_probe_request_stub(int remove) {
	debug("remove_he_ie_from_probe_request_stub");
}

static int get_dbs_capacity_stub(void) {
	debug("get_dbs_capacity_stub");
	return 0;
}

static int get_phy_capacity_stub(int band)
{
	debug("get_phy_capacity_stub");
	return 0;
}

static void get_supported_channels_stub(int band, int *len, int *freqs, int max_num)
{
	debug("get_supported_channels_stub");
	*len = 0;
}
static void get_avoid_channels_stub(int *len, int *freqs, int max_num)
{
	debug("get_avoid_channels_stub");
	*len = 0;
}

struct oplus_wfd_wlan_ops_t oplus_wfd_wlan_ops = {
	.remove_he_ie_from_probe_request = remove_he_ie_from_probe_request_stub,
	.get_dbs_capacity = get_dbs_capacity_stub,
	.get_phy_capacity = get_phy_capacity_stub,
	.get_supported_channels = get_supported_channels_stub,
	.get_avoid_channels = get_avoid_channels_stub
};

void register_oplus_wfd_wlan_ops(struct oplus_wfd_wlan_ops_t *ops) {
	if (ops == NULL)
		return;
	if (ops->remove_he_ie_from_probe_request)
		oplus_wfd_wlan_ops.remove_he_ie_from_probe_request = ops->remove_he_ie_from_probe_request;
	if (ops->get_dbs_capacity)
		oplus_wfd_wlan_ops.get_dbs_capacity = ops->get_dbs_capacity;
	if (ops->get_phy_capacity)
		oplus_wfd_wlan_ops.get_phy_capacity = ops->get_phy_capacity;
	if (ops->get_supported_channels)
		oplus_wfd_wlan_ops.get_supported_channels = ops->get_supported_channels;
	if (ops->get_avoid_channels)
		oplus_wfd_wlan_ops.get_avoid_channels = ops->get_avoid_channels;
}

static void oplus_wcc_remove_He_IE_from_probe_request(struct nlattr *nla)
{
	if (nla_len(nla) > 0) {
		u32 *data = (u32 *)NLA_DATA(nla);
		debug("remove he from probe rquest: %d", *data);
		oplus_wfd_wlan_ops.remove_he_ie_from_probe_request(*data);
	}
}

static void oplus_wcc_get_dbs_capacity(struct nlattr *nla)
{
	s32 cap = oplus_wfd_wlan_ops.get_dbs_capacity();
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_DBS_CAPACITY_GET, (char*)&cap, sizeof(cap));
}
static void oplus_wcc_get_phy_capacity(struct nlattr *nla)
{
	u32 band = 0;
	u32 cap = 0;
	if (nla_len(nla) > 0) {
		u32* data = (u32 *)NLA_DATA(nla);
		band = *data;
		if (band < oplus_wfd_band_max) {
			cap = oplus_wfd_wlan_ops.get_phy_capacity(band);
			debug("oplus_wcc_get_phy_capacity, cap= %d", cap);
		}
	}
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_PHY_CAPACITY_GET, (char*)&cap, sizeof(cap));
}

static void oplus_wcc_get_supported_channels(struct nlattr *nla)
{
	u32 band = 0;
	u32 len = 0;

	memset(s_oplus_wfd_freqs, 0, sizeof(s_oplus_wfd_freqs));
	if (nla_len(nla) > 0) {
		u32* data = (u32 *)NLA_DATA(nla);
		band = *data;
		if (band < oplus_wfd_band_max) {
			oplus_wfd_wlan_ops.get_supported_channels(band, &len, s_oplus_wfd_freqs + 1, OPLUS_WFD_FREQS_NUM_MAX);
			s_oplus_wfd_freqs[0] = len;
			debug("get supported channels, num = %d", len);
		}
	}
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_SUPPORTED_CHANNELS_GET, (char*)s_oplus_wfd_freqs, (len + 1)*sizeof(u32));
}

static void oplus_wcc_get_avoid_channels(struct nlattr *nla)
{
	u32 len = 0;

	memset(s_oplus_wfd_freqs, 0, sizeof(s_oplus_wfd_freqs));

	oplus_wfd_wlan_ops.get_avoid_channels(&len, s_oplus_wfd_freqs + 1, OPLUS_WFD_FREQS_NUM_MAX);
	s_oplus_wfd_freqs[0] = len;
	debug("get avoid channels, num = %d", len);

	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_AVOID_CHANNELS_GET, (char*)s_oplus_wfd_freqs, (len + 1)*sizeof(u32));
}
EXPORT_SYMBOL_GPL(register_oplus_wfd_wlan_ops);
/*#endif OPLUS_FEATURE_WIFI_OPLUSWFD*/

static int oplus_wcc_sample_async_get(struct nlattr *nla)
{
        u32 *data = (u32 *)NLA_DATA(nla);

        async_msg_type = OPLUS_SAMPLE_ASYNC_GET;
        oplus_timer.expires = jiffies + HZ;/* timer expires in ~1s*/
        add_timer(&oplus_timer);

        debug("sample_async_set: %u%u%u%u\n", data[0], data[1], data[2], data[3]);

        return 0;
}

static void oplus_wcc_timer_function(struct timer_list *t)
{
        if (async_msg_type == OPLUS_SAMPLE_ASYNC_GET) {
                oplus_wcc_sample_resp(oplus_async_nl_sock, oplus_async_nl_pid, OPLUS_SAMPLE_ASYNC_GET);
                async_msg_type = 0;
        }
}

static void oplus_wcc_timer_init(void)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
	init_timer(&oplus_timer);
	oplus_timer.function = (void*)oplus_wcc_timer_function;
#else
	timer_setup(&oplus_timer, (void*)oplus_wcc_timer_function, 0);
#endif
}

static void oplus_wcc_timer_fini(void)
{
	del_timer(&oplus_timer);
}

/*2021/4/15 Xufenghua add for GKI2.0 modify ALM ID:1177828*/
static int oplus_wcc_sync_genl_rcv_msg(struct sk_buff *skb, struct genl_info *info)
{
        int ret = 0;
        struct nlmsghdr *nlhdr;
        struct genlmsghdr *genlhdr;
        struct nlattr *nla;
        u32 portid;

        nlhdr = nlmsg_hdr(skb);
        genlhdr = nlmsg_data(nlhdr);
        nla = genlmsg_data(genlhdr);

        portid = nlhdr->nlmsg_pid; /*NETLINK_CB(skb).portid;*/
        debug("the nla->nla_type = %u, len = %u, port id = %d\n", nla->nla_type, nla->nla_len, portid);
        if (nla->nla_type == OPLUS_WIFI_CAP_CENTER_NOTIFY_PID) {
                oplus_sync_nl_pid = portid;
                debug("oplus_sync_nl_pid port id=%d\n", oplus_sync_nl_pid);
        }
        /* only recv msg from target pid*/
        if (portid != oplus_sync_nl_pid) {
                return -1;
        }
        if (!check_msg_in_range(oplus_sync_nl_sock, nla->nla_type)) {
                debug("msg_type:%d not in range\n", nla->nla_type);
                return -1;
        }

        switch (nla->nla_type) {
        case OPLUS_SAMPLE_SYNC_GET:
                oplus_wcc_sample_sync_get(nla);
                break;
        case OPLUS_SAMPLE_SYNC_GET_NO_RESP:
                oplus_wcc_sample_sync_get_no_resp(nla);
                break;
        case OPLUS_SYNC_REMOVE_HE_IE_FROM_PROBE_REQUEST:
                oplus_wcc_remove_He_IE_from_probe_request(nla);
                break;
        case OPLUS_SYNC_DBS_CAPACITY_GET:
                oplus_wcc_get_dbs_capacity(nla);
                break;
        case OPLUS_SYNC_PHY_CAPACITY_GET:
                oplus_wcc_get_phy_capacity(nla);
                break;
        case OPLUS_SYNC_SUPPORTED_CHANNELS_GET:
                oplus_wcc_get_supported_channels(nla);
                break;
        case OPLUS_SYNC_AVOID_CHANNELS_GET:
                oplus_wcc_get_avoid_channels(nla);
                break;
        default:
                return -EINVAL;
        }

        return 0;
}

static int oplus_wcc_async_genl_rcv_msg(struct sk_buff *skb, struct genl_info *info)
{
        int ret = 0;
        struct nlmsghdr *nlhdr;
        struct genlmsghdr *genlhdr;
        struct nlattr *nla;
        u32 portid;

        nlhdr = nlmsg_hdr(skb);
        genlhdr = nlmsg_data(nlhdr);
        nla = genlmsg_data(genlhdr);
        portid = nlhdr->nlmsg_pid; /*NETLINK_CB(skb).portid;*/
        debug("the nla->nla_type = %u, len = %u, port id = %d\n", nla->nla_type, nla->nla_len, portid);
	if (nla->nla_type == OPLUS_WIFI_CAP_CENTER_NOTIFY_PID) {
	        oplus_async_nl_pid = portid;
	        debug("oplus_async_nl_pid pid=%d\n", oplus_async_nl_pid);
	}
	/* only recv msg from target pid*/
	if (portid != oplus_async_nl_pid) {
		return -1;
	}
	if (!check_msg_in_range(oplus_async_nl_sock, nla->nla_type)) {
	        debug("msg_type:%d not in range\n", nla->nla_type);
	        return -1;
	}

        switch (nla->nla_type) {
        case OPLUS_SAMPLE_ASYNC_GET:
                oplus_wcc_sample_async_get(nla);
                break;
        default:
                return -EINVAL;
        }

	return 0;
}

static int oplus_wcc_sync_genl_rcv(struct sk_buff *skb, struct genl_info *info)
{
        int ret = 0;
	mutex_lock(&oplus_wcc_sync_nl_mutex);
	ret = oplus_wcc_sync_genl_rcv_msg(skb, info);
	mutex_unlock(&oplus_wcc_sync_nl_mutex);
	return ret;
}

static int oplus_wcc_async_genl_rcv(struct sk_buff *skb, struct genl_info *info)
{
        int ret = 0;
	mutex_lock(&oplus_wcc_async_nl_mutex);
	ret = oplus_wcc_async_genl_rcv_msg(skb, info);
	mutex_unlock(&oplus_wcc_async_nl_mutex);
	return ret;
}

static const struct genl_ops oplus_wcc_genl_sync_ops[] = {
	{
		.cmd = OPLUS_WIFI_CAP_CENTER_SYNC_CMD,
		.flags = 0,
		.doit = oplus_wcc_sync_genl_rcv,
		.dumpit = NULL,
	},
};

static const struct genl_ops oplus_wcc_genl_async_ops[] = {
	{
		.cmd = OPLUS_WIFI_CAP_CENTER_ASYNC_CMD,
		.flags = 0,
		.doit = oplus_wcc_async_genl_rcv,
		.dumpit = NULL,
	},
};

static struct genl_family oplus_wcc_genl_sync_family = {
	.id = 0,
	.hdrsize = 0,
	.name = OPLUS_WCC_SYNC_FAMILY,
	.version = OPLUS_WCC_FAMILY_VERSION,
	.maxattr = OPLUS_WIFI_CAP_CENTER_MSG_MAX,
	.ops = oplus_wcc_genl_sync_ops,
	.n_ops = ARRAY_SIZE(oplus_wcc_genl_sync_ops),
};

static struct genl_family oplus_wcc_genl_async_family = {
	.id = 0,
	.hdrsize = 0,
	.name = OPLUS_WCC_ASYNC_FAMILY,
	.version = OPLUS_WCC_FAMILY_VERSION,
	.maxattr = OPLUS_WIFI_CAP_CENTER_MSG_MAX,
	.ops = oplus_wcc_genl_async_ops,
	.n_ops = ARRAY_SIZE(oplus_wcc_genl_async_ops),
};

static inline int genl_msg_prepare_usr_msg(struct sock *oplus_sock, size_t size, struct sk_buff **skbp)
{
        /* 2021/11/18 fix coverity check error "UNINIT" CID 163555 162070 159525, init default value */
        struct sk_buff *skb;
        u8 cmd = OPLUS_WIFI_CAP_CENTER_SYNC_CMD;
        pid_t pid = oplus_async_nl_pid;
        struct genl_family *family = &oplus_wcc_genl_sync_family;
        if (oplus_sock == oplus_sync_nl_sock) {
                cmd = OPLUS_WIFI_CAP_CENTER_SYNC_CMD;
                pid = oplus_sync_nl_pid;
                family = &oplus_wcc_genl_sync_family;
        } else if (oplus_sock == oplus_async_nl_sock) {
                cmd = OPLUS_WIFI_CAP_CENTER_ASYNC_CMD;
                pid = oplus_async_nl_pid;
                family = &oplus_wcc_genl_async_family;
        }

        /* create a new netlink msg */
        skb = genlmsg_new(size, GFP_ATOMIC);
        if (skb == NULL) {
                return -ENOMEM;
        }
        /* Add a new netlink message to an skb */
        genlmsg_put(skb, pid, 0, family, 0, cmd);
        debug("skb_len:%u, pid:%u, cmd:%u, id:%u\n", skb->len, (unsigned int)pid, cmd, (*family).id);
        *skbp = skb;
        return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
        int ret;
        /* add a netlink attribute to a socket buffer */
        if ((ret = nla_put(skb, type, len, data)) != 0) {
                return ret;
        }
        return 0;
}

static int oplus_wcc_genl_init(void)
{
        int ret;
        ret = genl_register_family(&oplus_wcc_genl_sync_family);
        if (ret) {
                return ret;
        } else {
                debug("genl_register_family complete, sync id = %d\n", oplus_wcc_genl_sync_family.id);
        }
        /*2021/4/16 oplus_sync_nl_sock & oplus_async_nl_sock is used as the netlink sock handler,
        and will be used to check parameters in many places.
        When use generic netlink, it doesn't have this handlder, but it's not so easy to modify all the original using places,
        so we just manual init them to 1 and 2, then the using places needn't change.
        This will make the variable name a litte trick, but make GKI modification more easy and safe.
        */
        oplus_sync_nl_sock = (struct sock *)1;

        ret = genl_register_family(&oplus_wcc_genl_async_family);
        if (ret) {
                return ret;
        } else {
                debug("genl_register_family complete, async id = %d\n", oplus_wcc_genl_async_family.id);
        }
        oplus_async_nl_sock = (struct sock *)2;

        return ret;
}

static void oplus_wcc_genl_exit(void)
{
        genl_unregister_family(&oplus_wcc_genl_sync_family);
        genl_unregister_family(&oplus_wcc_genl_async_family);

        oplus_sync_nl_sock = NULL;
        oplus_async_nl_sock = NULL;
}
/*Add End for GKI2.0 modify*/

static void oplus_wcc_netlink_exit(void)
{
	netlink_kernel_release(oplus_sync_nl_sock);
	oplus_sync_nl_sock = NULL;

	netlink_kernel_release(oplus_async_nl_sock);
	oplus_async_nl_sock = NULL;
}

static struct ctl_table oplus_wcc_sysctl_table[] = {
	{
		.procname	= "oplus_wcc_debug",
		.data		= &oplus_wcc_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int oplus_wcc_sysctl_init(void)
{
	/* 2021/8/5 register_net_sysctl & unregister_net_sysctl_table is not in GKI2.0 whitelist default */
#if 0
	oplus_table_hrd = register_net_sysctl
	        (&init_net, "net/oplus_wcc", oplus_wcc_sysctl_table);
	return oplus_table_hrd == NULL ? -ENOMEM : 0;
#endif
	return 0;
}

static void oplus_wcc_sysctl_fini(void)
{
	/* 2021/8/5 register_net_sysctl & unregister_net_sysctl_table is not in GKI2.0 whitelist default */
#if 0
	if(oplus_table_hrd) {
		unregister_net_sysctl_table(oplus_table_hrd);
		oplus_table_hrd = NULL;
	}
#endif
	oplus_table_hrd = NULL;
}

static int __init oplus_wcc_init(void)
{
        int ret = 0;
        rwlock_init(&oplus_sync_nl_lock);
        rwlock_init(&oplus_async_nl_lock);

        ret = oplus_wcc_genl_init();

        if (ret < 0) {
                debug("oplus_wcc_init module failed to init netlink.\n");
        } else {
                debug("oplus_wcc_init module init netlink successfully.\n");
        }

        ret = oplus_wcc_sysctl_init();
        if (ret < 0) {
                debug("oplus_wcc_init module failed to init sysctl.\n");
        }
        else {
                debug("oplus_wcc_init module init sysctl successfully.\n");
        }

        oplus_wcc_timer_init();

        return ret;
}

static void __exit oplus_wcc_fini(void)
{
	oplus_wcc_sysctl_fini();
	oplus_wcc_genl_exit();

	oplus_wcc_timer_fini();
}

module_init(oplus_wcc_init);
module_exit(oplus_wcc_fini);

MODULE_LICENSE("GPL");
