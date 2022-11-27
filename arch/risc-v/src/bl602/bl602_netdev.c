/****************************************************************************
 * arch/risc-v/src/bl602/bl602_netdev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/signal.h>

#include <sys/types.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>

#ifdef CONFIG_NET_PKT
#include <nuttx/net/pkt.h>
#endif

#include "wifi_manager/include/bitset.h"
#include "wifi_manager/wifi_mgmr.h"
#include "wifi_manager/wifi_mgmr_api.h"
#include "wifi_manager/bl_wifi.h"
#include "wifi_manager/include/wifi_mgmr_ext.h"
#include "bl602_os_hal.h"
#include "bl602_netdev.h"
#include "bl602_efuse.h"

#ifdef CONFIG_BL602_WIRELESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

/* FIXME According to some network IO throughput tests, using HPWORK will
 * get higher throughput.
 */

#define ETHWORK HPWORK

/* BL602_NET_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_BL602_NET_MULTI_INTERFACE
#define BL602_NET_NINTERFACES 1
#else
#define BL602_NET_NINTERFACES 2
#endif

#define WIFI_MTU_SIZE 1516

#define BL602_NET_TXBUFF_NUM  12
#define BL602_NET_TXBUFF_SIZE (WIFI_MTU_SIZE + PRESERVE_80211_HEADER_LEN)

#define BL602_TXDESC_THRESHOLD 3

#if BL602_NET_TXBUFF_SIZE & 0x3 != 0
#error "BL602_NET_TXBUFF_SIZE must be aligned to 4 bytes"
#endif

#if !(BL602_TXDESC_THRESHOLD > 0)
#error "BL602_TXDESC_THRESHOLD invalid."
#endif

#define TX_BUFF_BIT_SIZE BL602_NET_TXBUFF_NUM

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->net_dev.d_buf)

#define WIFI_MGMR wifiMgmr

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The bl602_net_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct bl602_net_driver_s
{
  struct work_s availwork;

  /* wifi manager */

  struct wlan_netif *wlan;

  /* there is impossble to concurrency access these fields, so
   * we use bit-field to save some little space :)
   */

  unsigned int current_mode : 2;    /* current mode */
  unsigned int push_cnt : 4;        /* max 16 */
  unsigned int prev_connectd : 1;   /* mark of prev connection status */

  uint16_t channel; /* FIXME freq number. eg. 3, 0 is not set */

  char bssid[18]; /* AP mac address */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s net_dev; /* Interface understood by the network */
};

struct scan_parse_param_s
{
  struct bl602_net_driver_s *priv;

  int                flags;
  struct iw_scan_req scan_req;
};

BITSET_DEFINE(tx_buf_ind_s, TX_BUFF_BIT_SIZE);

typedef uint8_t (*tx_buff_t)[BL602_NET_TXBUFF_SIZE];

/* When a data packet is received, the NuttX protocol stack may use this
 * RX buffer to construct a response data packet. However, the WiFi Firmware
 * does not support using the RX buffer as the TX buffer directly, so it is
 * necessary to allocate a TX buffer to make a copy.
 * If there is no TX buffer, the response packet will be discarded.
 * Therefore, when a data packet is received, it will check whether there is
 * an available TX buffer. If not, it will hang the RX packet in this linked
 * list, and wait until TX buffer is available before submitting it to the
 * NuttX protocol stack.
 */

struct rx_pending_item_s
{
  struct list_node node;
  struct bl602_net_driver_s *priv; /* Which interface should to deliver */
  uint8_t *data;
  int      len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver state structure */

struct bl602_net_driver_s g_bl602_net[BL602_NET_NINTERFACES];

static struct tx_buf_ind_s g_tx_buf_indicator =
  BITSET_T_INITIALIZER((1 << BL602_NET_TXBUFF_NUM) - 1);

static uint8_t locate_data(".wifi_ram.txbuff")
g_tx_buff[BL602_NET_TXBUFF_NUM][BL602_NET_TXBUFF_SIZE];

static mutex_t g_wifi_scan_lock = NXMUTEX_INITIALIZER;
static sem_t g_wifi_connect_sem = SEM_INITIALIZER(0);

/* Rx Pending List */

static struct list_node g_rx_pending;

/* Firmware default config */

static wifi_conf_t g_conf =
{
  .country_code = CONFIG_BL602_WIRELESS_CONTRY_CODE,
};

/* Global state */

static struct
{
  uint32_t scan_result_status : 2; /* WiFi scan result status */
  uint32_t scan_result_len : 6;
  uint32_t retry_cnt : 4; /* MAX 16 retries */
  uint32_t sta_connected: 1;
  uint32_t ap_stared: 1;
} g_state;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

extern int  bl_output(struct bl_hw *bl_hw, char *p, int tot_len, int is_sta);
extern void bl_free_rx_buffer(void *p);
extern void bl_irq_handler(void);
extern void wifi_main_init(void);
extern void ipc_emb_notify(void);
extern void wifi_mgmr_tsk_init(void);
extern int bl602_ef_ctrl_read_mac_address(uint8_t mac[6]);
extern void wifi_main(int argc, char *argv[]);
extern void wifi_mgmr_start_background(wifi_conf_t *conf);
extern int bl_pm_init(void);
extern struct net_device bl606a0_sta;

/* Common TX logic */

static int bl602_net_transmit(struct bl602_net_driver_s *priv);
static int bl602_net_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void bl602_net_reply(struct bl602_net_driver_s *priv);
static void bl602_net_receive(struct bl602_net_driver_s *priv);

/* NuttX callback functions */

static int bl602_net_ifup(struct net_driver_s *dev);
static int bl602_net_ifdown(struct net_driver_s *dev);

static void bl602_net_txavail_work(void *arg);
static int  bl602_net_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int bl602_net_addmac(struct net_driver_s *dev,
                            const uint8_t *mac);
# ifdef CONFIG_NET_MCASTGROUP
static int bl602_net_rmmac(struct net_driver_s *dev,
                           const uint8_t *mac);
# endif
# ifdef CONFIG_NET_ICMPv6
static void bl602_net_ipv6multicast(struct bl602_net_driver_s *priv);
# endif
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int
bl602_net_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_net_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bl602_net_transmit(struct bl602_net_driver_s *priv)
{
  int ret = OK;
  ninfo("tx pkt len:%d\n", priv->net_dev.d_len);

  DEBUGASSERT(priv->net_dev.d_len <= WIFI_MTU_SIZE);
  DEBUGASSERT(priv->push_cnt < BL602_TXDESC_THRESHOLD);

  if (!IFF_IS_RUNNING(priv->net_dev.d_flags))
    {
      nwarn("drop packet due to iface no carrier\n");
      bl602_netdev_free_txbuf(priv->net_dev.d_buf -
                              PRESERVE_80211_HEADER_LEN);
      priv->net_dev.d_buf = NULL;

      return -EPERM;
    }

  DEBUGASSERT(priv->wlan != NULL);

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->net_dev);

  bl_output(bl606a0_sta.bl_hw,
            (char *)priv->net_dev.d_buf,
            priv->net_dev.d_len,
            0 == priv->wlan->mode);

  priv->push_cnt++;

  if (priv->push_cnt == BL602_TXDESC_THRESHOLD)
    {
      /* notify to tx now */

      bl_irq_handler();
      priv->push_cnt = 0;
    }

  priv->net_dev.d_buf = NULL;

  return ret;
}

/****************************************************************************
 * Name: bl602_net_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bl602_net_txpoll(struct net_driver_s *dev)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;

  if (priv->net_dev.d_len > 0)
    {
      DEBUGASSERT(priv->net_dev.d_buf);

      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->net_dev.d_flags))
#endif
        {
          arp_out(&priv->net_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->net_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Check if the network is sending this packet to the IP address of
       * this device.  If so, just loop the packet back into the network but
       * don't attempt to put it on the wire.
       */

      if (!devif_loopback(&priv->net_dev))
        {
          /* Send the packet */

          bl602_net_transmit(priv);

          /* Check if there is room in the device to hold another packet.
           * If not, return a non-zero value to terminate the poll.
           */

          priv->net_dev.d_buf = bl602_netdev_alloc_txbuf();
          if (priv->net_dev.d_buf)
            {
              priv->net_dev.d_buf += PRESERVE_80211_HEADER_LEN;
              priv->net_dev.d_len = 0;
            }

          return priv->net_dev.d_buf == NULL;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: bl602_net_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return return with an outgoing packet.  This function checks for
 *   that case and performs the transmission if necessary.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void bl602_net_reply(struct bl602_net_driver_s *priv)
{
  uint8_t *tx_p = NULL;

  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (priv->net_dev.d_len > 0)
    {
      /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      /* Check for an outgoing IPv4 packet */

      if (IFF_IS_IPv4(priv->net_dev.d_flags))
#endif
        {
          arp_out(&priv->net_dev);
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      /* Otherwise, it must be an outgoing IPv6 packet */

      else
#endif
        {
          neighbor_out(&priv->net_dev);
        }
#endif

      /* alloc tx buffer and copy to it */

      tx_p = bl602_netdev_alloc_txbuf();
      if (tx_p)
        {
          tx_p += PRESERVE_80211_HEADER_LEN;
          DEBUGASSERT(priv->net_dev.d_len <=
                 BL602_NET_TXBUFF_SIZE - PRESERVE_80211_HEADER_LEN);

          /* we copy it, and release rx buffer */

          memcpy(tx_p, priv->net_dev.d_buf, priv->net_dev.d_len);

          bl_free_rx_buffer(priv->net_dev.d_buf);

          priv->net_dev.d_buf = tx_p;

          bl602_net_transmit(priv);

#if BL602_TXDESC_THRESHOLD > 1
          /* notify to tx now */

          bl_irq_handler();
          priv->push_cnt = 0;
#endif

          return;
        }
      else
        {
          /* NOTIC: The release path of tx buffer cannot acquire net lock */

          nerr("can not replay due to no tx buffer!\n");
          PANIC();
        }
    }

  /* we not have tx buffer, so we lost this packet */

  bl_free_rx_buffer(priv->net_dev.d_buf);
  priv->net_dev.d_buf = NULL;
}

/****************************************************************************
 * Name: bl602_net_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void bl602_net_receive(struct bl602_net_driver_s *priv)
{
#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->net_dev);
#endif

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 packet */

  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->net_dev);

      /* Handle ARP on input, then dispatch IPv4 packet to the network
       * layer.
       */

      arp_ipin(&priv->net_dev);
      ipv4_input(&priv->net_dev);

      /* Check for a reply to the IPv4 packet */

      bl602_net_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 packet */

  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->net_dev);

      /* Dispatch IPv6 packet to the network layer */

      ipv6_input(&priv->net_dev);

      /* Check for a reply to the IPv6 packet */

      bl602_net_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      /* Dispatch ARP packet to the network layer */

      arp_arpin(&priv->net_dev);
      NETDEV_RXARP(&priv->net_dev);

      if (priv->net_dev.d_len > 0)
        {
          uint8_t *tx_p = bl602_netdev_alloc_txbuf();
          if (tx_p != NULL)
            {
              DEBUGASSERT(priv->net_dev.d_len <=
                     BL602_NET_TXBUFF_SIZE - PRESERVE_80211_HEADER_LEN);

              tx_p += PRESERVE_80211_HEADER_LEN;

              /* we copy it, and release rx buffer */

              memcpy(tx_p, priv->net_dev.d_buf, priv->net_dev.d_len);

              bl_free_rx_buffer(priv->net_dev.d_buf);

              priv->net_dev.d_buf = tx_p;
              bl602_net_transmit(priv);

#if BL602_TXDESC_THRESHOLD > 1
              /* notify to tx now */

              bl_irq_handler();
              priv->push_cnt = 0;
#endif
              return;
            }
          else
            {
              nerr("can not replay due to no tx buffer!\n");
              PANIC();
            }
        }

      /* we not have tx buffer, so we lost this packet */

      bl_free_rx_buffer(priv->net_dev.d_buf);
      priv->net_dev.d_buf = NULL;
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->net_dev);
      bl_free_rx_buffer(priv->net_dev.d_buf);
      priv->net_dev.d_buf = NULL;
    }
}

static int bl602_launch_pending_rx(void)
{
  struct rx_pending_item_s *item;
  irqstate_t                irqstate;
  int                       tx_buf_empty;

  while (1)
    {
      net_lock();

      irqstate     = enter_critical_section();
      tx_buf_empty = BIT_EMPTY(TX_BUFF_BIT_SIZE, &g_tx_buf_indicator);
      leave_critical_section(irqstate);

      if (tx_buf_empty)
        {
          /* we dont have tx buffer, so we cant go ahead, abort.. */

          nwarn("tx buf empty!\n");

          net_unlock();
          return -ENOMEM;
        }

      irqstate = enter_critical_section();
      item =
        list_remove_head_type(&g_rx_pending, struct rx_pending_item_s, node);
      leave_critical_section(irqstate);

      if (item == NULL)
        {
          /* we have free tx buffer, but we don't have pending rx data to
           * input, we start poll the normal tx routine.
           */

          net_unlock();

          return OK;
        }

      ninfo("input stack rx data :%p %d\n", item->data, item->len);

      /* now we have avaliable tx buffer and pending rx data, launch it */

      DEBUGASSERT(item->priv != NULL);
      DEBUGASSERT(item->priv->net_dev.d_buf == NULL);
      DEBUGASSERT(item->data != NULL && item->len > 0);

      item->priv->net_dev.d_buf = item->data;
      item->priv->net_dev.d_len = item->len;
      bl602_net_receive(item->priv);

      DEBUGASSERT(item->priv->net_dev.d_buf == NULL);
      net_unlock();

      kmm_free(item);
    }
}

/****************************************************************************
 * Name: bl602_net_notify
 *
 * Description:
 *   BL602 WiFi notify handler, similar interrupt function
 *
 * Input Parameters:
 *   event: notify type, tx done or received new data
 *   data: The data of the event, may be NULL
 *   len: data length
 *   opaque: customer data
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int bl602_net_notify(uint32_t event, uint8_t *data, int len, void *opaque)
{
  DEBUGASSERT(opaque != NULL);

  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)opaque;

  DEBUGASSERT(priv == &g_bl602_net[0] || priv == &g_bl602_net[1]);

  int ret;

  if (event & BL602_NET_EVT_TX_DONE)
    {
      /* if we have tx buffer, we put pending input packet first */

      ret = bl602_launch_pending_rx();
      if (ret != OK)
        {
          /* There is no tx buffer, we needn't to poll.. */

          return -ENOMEM;
        }

      if (work_available(&priv->availwork))
        {
          /* Schedule to serialize the poll on the worker thread. */

          work_queue(
            ETHWORK, &priv->availwork, bl602_net_txavail_work, priv, 0);
        }
    }

  if (event & BL602_NET_EVT_RX)
    {
      int tx_buf_empty = 0;
      DEBUGASSERT(data != NULL && len > 0);

      if (!IFF_IS_UP(priv->net_dev.d_flags))
        {
          wlinfo("Drop rx packet due to iface not up\n");
          bl_free_rx_buffer(data);

          return OK;
        }

      irqstate_t irqstate = enter_critical_section();
      tx_buf_empty        = BIT_EMPTY(TX_BUFF_BIT_SIZE, &g_tx_buf_indicator);
      leave_critical_section(irqstate);

      if (tx_buf_empty)
        {
          /* pending this packet to list */

          struct rx_pending_item_s *item =
            kmm_malloc(sizeof(struct rx_pending_item_s));
          if (item == NULL)
            {
              wlwarn("failed to alloc rx pending item, drop rx packet!\n");
              bl_free_rx_buffer(data);

              return -ENOMEM;
            }

          list_initialize(&item->node);

          item->data = data;
          item->len  = len;
          item->priv = priv;

          wlinfo("pending rx data :%p %d\n", item->data, item->len);

          irqstate = enter_critical_section();
          list_add_tail(&g_rx_pending, &item->node);
          leave_critical_section(irqstate);
        }
      else
        {
          /* Thanks god, we have tx buffer now, put this packet to stack */

          wlinfo("input stack direct:%p %d\n", data, len);

          net_lock();
          DEBUGASSERT(priv->net_dev.d_buf == NULL);

          priv->net_dev.d_buf = data;
          priv->net_dev.d_len = len;
          bl602_net_receive(priv);

          DEBUGASSERT(priv->net_dev.d_buf == NULL);
          net_unlock();
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_net_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Wireless interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bl602_net_ifup(struct net_driver_s *dev)
{
#ifdef CONFIG_NET_ICMPv6
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;
#endif

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %ld.%ld.%ld.%ld\n",
        dev->d_ipaddr & 0xff,
        (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff,
        dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0],
        dev->d_ipv6addr[1],
        dev->d_ipv6addr[2],
        dev->d_ipv6addr[3],
        dev->d_ipv6addr[4],
        dev->d_ipv6addr[5],
        dev->d_ipv6addr[6],
        dev->d_ipv6addr[7]);
#endif

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  bl602_net_ipv6multicast(priv);
#endif

  return OK;
}

static int bl602_net_soft_reset(void)
{
  int idx;

  wifi_mgmr_sta_disconnect();
  nxsig_sleep(1);
  wifi_mgmr_api_ap_stop();
  nxsig_sleep(1);
  wifi_mgmr_api_idle();
  wifi_mgmr_reset();

  for (idx = 0; idx < BL602_NET_NINTERFACES; idx++)
    {
      g_bl602_net[idx].current_mode = IW_MODE_AUTO;
    }

  return 0;
}

/****************************************************************************
 * Name: bl602_net_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bl602_net_ifdown(struct net_driver_s *dev)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;
  irqstate_t flags;

  net_lock();

  flags = enter_critical_section();

  leave_critical_section(flags);

  if (priv == &g_bl602_net[0])
    {
      bl602_net_soft_reset();
    }

  net_unlock();
  return OK;
}

/****************************************************************************
 * Name: bl602_net_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs on a work queue thread.
 *
 ****************************************************************************/

static void bl602_net_txavail_work(void *arg)
{
  struct bl602_net_driver_s *priv = (struct bl602_net_driver_s *)arg;

  net_lock();
  DEBUGASSERT(priv->net_dev.d_buf == NULL);
  DEBUGASSERT(priv->push_cnt == 0);

  /* Ignore the notification if the interface is not yet up */

  if (!IFF_IS_UP(priv->net_dev.d_flags))
    {
      net_unlock();
      return;
    }

  priv->net_dev.d_buf = bl602_netdev_alloc_txbuf();
  if (priv->net_dev.d_buf)
    {
      priv->net_dev.d_buf += PRESERVE_80211_HEADER_LEN;
      priv->net_dev.d_len = 0;
    }

  /* If so, then poll the network for new XMIT data */

  if (priv->net_dev.d_buf)
    {
      devif_poll(&priv->net_dev, bl602_net_txpoll);

      if (priv->push_cnt != 0)
        {
          /* notify to tx now */

          bl_irq_handler();
          priv->push_cnt = 0;
        }

      if (priv->net_dev.d_buf != NULL)
        {
          bl602_netdev_free_txbuf(priv->net_dev.d_buf -
                                  PRESERVE_80211_HEADER_LEN);
          priv->net_dev.d_buf = NULL;
        }
      else
        {
          work_queue(
            ETHWORK, &priv->availwork, bl602_net_txavail_work, priv, 0);
        }
    }

  DEBUGASSERT(priv->net_dev.d_buf == NULL);
  net_unlock();
}

/****************************************************************************
 * Name: bl602_net_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bl602_net_txavail(struct net_driver_s *dev)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;

  if (work_available(&priv->availwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->availwork, bl602_net_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_net_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int bl602_net_addmac(struct net_driver_s *dev,
                            const uint8_t *mac)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_net_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int bl602_net_rmmac(struct net_driver_s *dev,
                           const uint8_t *mac)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_net_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void bl602_net_ipv6multicast(struct bl602_net_driver_s *priv)
{
  struct net_driver_s *dev;
  uint16_t                 tmp16;
  uint8_t                  mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Wireless MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Wireless MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->net_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]);

  bl602_net_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes MAC address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  bl602_net_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers MAC address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  bl602_net_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

static void scan_complete_indicate(void *data, void *param)
{
  int                        i;
  struct scan_parse_param_s *para;
  wifi_mgmr_scan_item_t     *scan;

  para = (struct scan_parse_param_s *)data;
  DEBUGASSERT(para != NULL);
  DEBUGASSERT(para->priv != NULL);
  g_state.scan_result_len = 0;

  for (i = 0;
       i < sizeof(WIFI_MGMR.scan_items) / sizeof(WIFI_MGMR.scan_items[0]);
       i++)
    {
      scan = &WIFI_MGMR.scan_items[i];

      if (wifi_mgmr_scan_item_is_timeout(&WIFI_MGMR,
                                         &(WIFI_MGMR.scan_items[i])))
        {
          scan->is_used = 0;
        }
      else if (scan->is_used)
        {
          if (para->priv->channel != 0 &&
              scan->channel != para->priv->channel)
            {
              scan->is_used = 0;
            }
          else if (para->flags & IW_SCAN_THIS_ESSID)
            {
              if (strncmp(scan->ssid,
                          (char *)para->scan_req.essid,
                          sizeof(scan->ssid)) == 0)
                {
                  scan->is_used = 1;
                  g_state.scan_result_len++;
                }
              else
                {
                  scan->is_used = 0;
                }
            }
          else
            {
              g_state.scan_result_len++;
            }
        }
    }

  kmm_free(data);
}

static int rssi_compare(const void *arg1, const void *arg2)
{
  wifi_mgmr_scan_item_t *item1 = &WIFI_MGMR.scan_items[*(uint8_t *)arg1];
  wifi_mgmr_scan_item_t *item2 = &WIFI_MGMR.scan_items[*(uint8_t *)arg2];

  return item1->rssi - item2->rssi;
}

static int format_scan_result_to_wapi(struct iwreq *req, int result_cnt)
{
  int      i = 0;
  int      j = 0;
  int      event_buff_len = 0;
  uint8_t *curr_pos       = NULL;

  uint8_t *rssi_list = NULL; /* for sort */

  if (result_cnt == 0)
    {
      return OK;
    }

  /* More compact arrangement */

  event_buff_len = result_cnt *
    (offsetof(struct iw_event, u) * 4 + sizeof(struct sockaddr) +
    sizeof(struct iw_freq) + sizeof(struct iw_quality) +
    sizeof(struct iw_point) + IW_ESSID_MAX_SIZE);

  if (req->u.data.length == 0 || req->u.data.length < event_buff_len)
    {
      return -E2BIG;
    }

  req->u.data.length = event_buff_len;

  /* alloc rssi list */

  rssi_list = kmm_malloc(result_cnt * sizeof(uint8_t));
  if (rssi_list == NULL)
    {
      return -ENOMEM;
    }

  /* record all valid scan result items */

  for (i = 0, j = 0;
       i < sizeof(WIFI_MGMR.scan_items) / sizeof(WIFI_MGMR.scan_items[0]);
       i++)
    {
      wifi_mgmr_scan_item_t *scan = &WIFI_MGMR.scan_items[i];

      if (scan->is_used == 0)
        {
          continue;
        }

      rssi_list[j++] = i;
    }

  DEBUGASSERT(j == result_cnt);

  /* sort the valid list according the rssi */

  qsort(rssi_list, result_cnt, sizeof(uint8_t), rssi_compare);

  /* construct iw event buffer */

  curr_pos = req->u.data.pointer;
  DEBUGASSERT(curr_pos != NULL);
  DEBUGASSERT(((uintptr_t)curr_pos & 0x3) == 0);

  for (i = 0; i < result_cnt; i++)
    {
      int                    idx  = rssi_list[i];
      wifi_mgmr_scan_item_t *scan = &WIFI_MGMR.scan_items[idx];

      struct iw_event *iwe;

      iwe = (struct iw_event *)curr_pos;
      DEBUGASSERT(((uintptr_t)iwe & 0x3) == 0);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct sockaddr);
      iwe->cmd = SIOCGIWAP;
      memcpy(iwe->u.ap_addr.sa_data, scan->bssid, sizeof(struct ether_addr));

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      DEBUGASSERT(((uintptr_t)iwe & 0x3) == 0);
      iwe->len      = offsetof(struct iw_event, u) + sizeof(struct iw_freq);
      iwe->cmd      = SIOCGIWFREQ;
      iwe->u.freq.e = 0;
      iwe->u.freq.m = scan->channel;

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      DEBUGASSERT(((uintptr_t)iwe & 0x3) == 0);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct iw_quality);
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.level   = scan->rssi;
      iwe->u.qual.updated = IW_QUAL_DBM;

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      DEBUGASSERT(((uintptr_t)iwe & 0x3) == 0);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct iw_point) +
                 IW_ESSID_MAX_SIZE;
      iwe->cmd            = SIOCGIWESSID;
      iwe->u.essid.length = strlen(scan->ssid);
      iwe->u.essid.flags  = 1;

      /* refer:wapi wireless.c:272 */

      iwe->u.essid.pointer = (void *)(uintptr_t)sizeof(struct iw_point);

      memcpy((uint8_t *)iwe + offsetof(struct iw_event, u) +
               sizeof(struct iw_point),
             scan->ssid,
             IW_ESSID_MAX_SIZE);

      curr_pos = (uint8_t *)(uintptr_t)iwe + iwe->len;
    }

  kmm_free(rssi_list);

  return OK;
}

static wifi_mgmr_t *
bl602_netdev_get_wifi_mgmr(struct bl602_net_driver_s *priv)
{
  if (priv->current_mode == IW_MODE_INFRA)
    {
      return container_of(priv->wlan, wifi_mgmr_t, wlan_sta);
    }
  else if (priv->current_mode == IW_MODE_MASTER)
    {
      return container_of(priv->wlan, wifi_mgmr_t, wlan_ap);
    }
  else
    {
      return NULL;
    }
}

#ifdef CONFIG_NETDEV_IOCTL
static int bl602_ioctl_wifi_start(struct bl602_net_driver_s *priv,
                                  uintptr_t                      arg)
{
  UNUSED(arg);

  /* preform connect ap */

  wifi_mgmr_t *mgmr = bl602_netdev_get_wifi_mgmr(priv);
  if (mgmr == NULL)
    {
      return -EPERM;
    }

  if (priv->current_mode == IW_MODE_INFRA)
    {
      int state;

      if (g_state.sta_connected == 1)
        {
          return OK;
        }

      priv->prev_connectd = 0;
      g_state.retry_cnt = 0;

      wifi_mgmr_sta_autoconnect_enable();
      if (wifi_mgmr_sta_connect(NULL, mgmr->wifi_mgmr_stat_info.ssid,
                                mgmr->wifi_mgmr_stat_info.passphr,
                                NULL,
                                (uint8_t *)priv->bssid,
                                0,
                                priv->channel) == -1)
        {
          return -EPIPE;
        }

      nxsem_wait(&g_wifi_connect_sem);

      /* check connect state */

      wifi_mgmr_state_get_internal(&state);
      if (state != WIFI_STATE_CONNECTED_IP_GOT)
        {
          return -EPIPE;
        }
    }
  else if (priv->current_mode == IW_MODE_MASTER)
    {
      int channel;

      if (g_state.ap_stared == 1)
        {
          return OK;
        }

      wifi_mgmr_channel_get(&channel);
      wlinfo("AP channel:%d\n", channel);

      if (wifi_mgmr_api_ap_start(mgmr->wifi_mgmr_stat_info.ssid,
                                 mgmr->wifi_mgmr_stat_info.passphr,
                                 channel ? channel : 1,
                                 0) < 0)
        {
          return -EPIPE;
        }
    }
  else
    {
      return -ENOSYS;
    }

  return OK;
}

static int bl602_ioctl_wifi_stop(struct bl602_net_driver_s *priv,
                                 uintptr_t                      arg)
{
  UNUSED(arg);

  /* perform disconnect */

  if (priv->current_mode == IW_MODE_INFRA)
    {
      if (g_state.sta_connected == 0)
        {
          return OK;
        }

      wifi_mgmr_sta_disconnect();
      nxsig_sleep(1);
      wifi_mgmr_api_idle();
    }
  else if (priv->current_mode == IW_MODE_MASTER)
    {
      if (g_state.ap_stared == 0)
        {
          return OK;
        }

      wifi_mgmr_api_ap_stop();
      nxsig_sleep(1);
      wifi_mgmr_api_idle();
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_net_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int
bl602_net_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
  struct bl602_net_driver_s *priv =
    (struct bl602_net_driver_s *)dev->d_private;
  int ret = -ENOSYS;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
    case SIOCSIWSCAN:
      do
        {
          struct iwreq              *req  = (struct iwreq *)arg;
          struct scan_parse_param_s *para = NULL;

          para = kmm_malloc(sizeof(struct scan_parse_param_s));
          if (para == NULL)
            {
              return -ENOMEM;
            }

          para->priv = priv;
          if (req->u.data.flags)
            {
              if (req->u.data.pointer == NULL)
                {
                  kmm_free(para);
                  return -EINVAL;
                }

              para->flags = req->u.data.flags;
              para->scan_req = *(struct iw_scan_req *)req->u.data.pointer;
            }
          else
            {
              para->flags = 0;
            }

          if (nxmutex_trylock(&g_wifi_scan_lock) == 0)
            {
              if (priv->channel != 0)
                {
                  const char *ssid = para->flags & IW_SCAN_THIS_ESSID ?
                    (char *)para->scan_req.essid : NULL;
                  wifi_mgmr_scan_adv(para, scan_complete_indicate,
                      &priv->channel, 1, ssid);
                }
              else
                {
                  const char *ssid = para->flags & IW_SCAN_THIS_ESSID ?
                    (char *)para->scan_req.essid : NULL;
                  wifi_mgmr_scan_adv(para, scan_complete_indicate,
                      NULL, 0, ssid);
                }

              return OK;
            }
          else
            {
              kmm_free(para);
              return -EBUSY;
            }
        }
      while (0);
      break;

    case SIOCGIWSCAN:
      do
        {
          struct iwreq *req = (struct iwreq *)arg;

          nxmutex_lock(&g_wifi_scan_lock);

          if (g_state.scan_result_status != 0)
            {
              wlwarn("scan failed\n");
              nxmutex_unlock(&g_wifi_scan_lock);
              return -EIO;
            }

          if (g_state.scan_result_len == 0)
            {
              req->u.data.length = 0;
              nxmutex_unlock(&g_wifi_scan_lock);
              return OK;
            }

          ret = format_scan_result_to_wapi(req, g_state.scan_result_len);
          nxmutex_unlock(&g_wifi_scan_lock);
          return ret;
        }
      while (0);
      break;

    case SIOCSIFHWADDR: /* Set device MAC address */
      return -ENOSYS;
      break;

    case SIOCSIWAUTH:
      /* FIXME not support set auth param now, return OK to support
       * wapi reconnect command
       */

      return OK;
      break;

    case SIOCSIWENCODEEXT: /* Set psk */
      do
        {
          struct iwreq         *req        = (struct iwreq *)arg;
          struct iw_encode_ext *ext        = req->u.encoding.pointer;
          char                 *passphrase = kmm_malloc(ext->key_len + 1);
          if (passphrase == NULL)
            {
              return -ENOMEM;
            }

          strlcpy(passphrase, (char *)ext->key, ext->key_len + 1);

          wifi_mgmr_sta_passphr_set(passphrase);
          kmm_free(passphrase);
          return OK;
        }
      while (0);
      break;

    case SIOCSIWFREQ: /* Set channel/frequency (Hz) */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;

          if (req->u.freq.e != 0)
            {
              return -EINVAL;
            }

          priv->channel = req->u.freq.m;

          wlinfo("set channel to %d\n", priv->channel);

          return OK;
        }
      while (0);
      break;

    case SIOCGIWFREQ: /* Get channel/frequency (Hz) */
      wlwarn("WARNING: SIOCGIWFREQ not implemented\n");
      ret = -ENOSYS;
      break;

    case SIOCSIWMODE: /* Set operation mode */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;
#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
          int interface_idx = priv - g_bl602_net;

          DEBUGASSERT(interface_idx >= 0 &&
              interface_idx < BL602_NET_NINTERFACES);
#endif

          if (req->u.mode == priv->current_mode)
            {
              wlinfo("mode not change\n");
              return OK;
            }

          if (req->u.mode == IW_MODE_INFRA)
            {
              /* station */

#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
              if (interface_idx != 0)
                {
                  wlwarn("The interface does not support this mode.\n");
                  return -ENOSYS;
                }
#endif

              priv->wlan = wifi_mgmr_sta_enable(priv);

              memcpy(priv->wlan->mac,
                     priv->net_dev.d_mac.ether.ether_addr_octet,
                     6);
              wlinfo("now in station mode\n");
              priv->current_mode = IW_MODE_INFRA;
              return OK;
            }
          else if (req->u.mode == IW_MODE_MASTER)
            {
              /* AP Mode */

#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
              if (interface_idx != 1)
                {
                  wlwarn("The interface does not support this mode.\n");
                  return -ENOSYS;
                }
#endif

              priv->wlan = wifi_mgmr_ap_enable(priv);
              memcpy(priv->wlan->mac,
                     priv->net_dev.d_mac.ether.ether_addr_octet,
                     6);
              wlinfo("now in ap state\n");
              priv->current_mode = IW_MODE_MASTER;
              return OK;
            }
          else
            {
              wlwarn("WARNING: Unsupport mode:%ld\n", req->u.mode);
              return -ENOSYS;
            }
        }
      while (0);
      break;

    case SIOCGIWMODE: /* Get operation mode */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;
          req->u.mode       = priv->current_mode;
          return OK;
        }
      while (0);
      break;

    case SIOCSIWAP: /* Set access point MAC addresses */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;
          if (priv->current_mode == IW_MODE_INFRA)
            {
              /* Set bssid */

              memcpy(
                priv->bssid, req->u.ap_addr.sa_data, sizeof(priv->bssid));
              wlinfo("ap bssid:%s\n", priv->bssid);
            }
          else if (priv->current_mode == IW_MODE_MASTER)
            {
              bl_wifi_ap_mac_addr_set((uint8_t *)req->u.ap_addr.sa_data);
            }
          else
            {
              return -ENOSYS;
            }

          return OK;
        }
      while (0);
      break;

    case SIOCGIWAP: /* Get access point MAC addresses */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;
          if (priv->current_mode == IW_MODE_INFRA)
            {
              /* Get bssid */

              memcpy(req->u.ap_addr.sa_data,
                     priv->bssid,
                     sizeof(req->u.ap_addr.sa_data));
            }
          else if (priv->current_mode == IW_MODE_MASTER)
            {
              bl_wifi_ap_mac_addr_get((uint8_t *)req->u.ap_addr.sa_data);
            }

          return OK;
        }
      while (0);
      break;

    case SIOCSIWESSID: /* Set ESSID (network name) */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;

          wifi_mgmr_sta_ssid_set(req->u.essid.pointer);

          wlinfo("essid: %s\n", (char *)req->u.essid.pointer);

          if (req->u.essid.flags == 0)
            {
              return bl602_ioctl_wifi_stop(priv, arg);
            }
          else if (req->u.essid.flags == 1)
            {
              return bl602_ioctl_wifi_start(priv, arg);
            }
          else
            {
              wlinfo("unknow essid action: %d\n", req->u.essid.flags);
              return -ENOSYS;
            }
        }
      while (0);
      break;

    case SIOCGIWESSID: /* Get ESSID */
      do
        {
          struct iwreq *req  = (struct iwreq *)arg;
          wifi_mgmr_t * mgmr = bl602_netdev_get_wifi_mgmr(priv);
          int           length;

          DEBUGASSERT(req->u.essid.length > 0);
          DEBUGASSERT(req->u.essid.pointer != NULL);

          if (mgmr == NULL)
            {
              *(uint8_t *)req->u.essid.pointer = 0;

              return OK;
            }

          length = strlen(mgmr->wifi_mgmr_stat_info.ssid) + 1;
          length =
            length > req->u.essid.length ? req->u.essid.length : length;

          memcpy(
            req->u.essid.pointer, mgmr->wifi_mgmr_stat_info.ssid, length);

          return OK;
        }
      while (0);
      break;

    case SIOCSIWRATE: /* Set default bit rate (bps) */
      wlwarn("WARNING: SIOCSIWRATE not implemented\n");
      ret = -ENOSYS;
      break;

    case SIOCGIWRATE: /* Get default bit rate (bps) */
      wlwarn("WARNING: SIOCGIWRATE not implemented\n");
      ret = -ENOSYS;
      break;

    case SIOCSIWTXPOW: /* Set transmit power (dBm) */
      wlwarn("WARNING: SIOCSIWTXPOW not implemented\n");
      ret = -ENOSYS;
      break;

    case SIOCGIWTXPOW: /* Get transmit power (dBm) */
      wlwarn("WARNING: SIOCGIWTXPOW not implemented\n");
      ret = -ENOSYS;
      break;

    case SIOCGIWENCODEEXT: /* Get encoding token mode */
      do
        {
          struct iwreq         *req = (struct iwreq *)arg;
          struct iw_encode_ext *ext;
          wifi_mgmr_t          *mgmr = bl602_netdev_get_wifi_mgmr(priv);
          int                   length;

          DEBUGASSERT(mgmr != NULL);

          ext      = (struct iw_encode_ext *)req->u.encoding.pointer;
          length   = req->u.encoding.length - sizeof(struct iw_encode_ext);
          ext->alg = IW_ENCODE_ALG_NONE;
          ext->key_len = strlen(mgmr->wifi_mgmr_stat_info.passphr);
          if (ext->key_len > length)
            {
              return -E2BIG;
            }

          memcpy(ext->key, mgmr->wifi_mgmr_stat_info.passphr, ext->key_len);

          return OK;
        }
      while (0);
      break;

    case SIOCSIWCOUNTRY: /* Set country code */
      do
        {
          struct iwreq *req = (struct iwreq *)arg;

          ret = wifi_mgmr_set_country_code(req->u.data.pointer);
          if (ret != 0)
            {
              return -EINVAL;
            }

          return OK;
        }
      while (0);
      break;

    default:
      wlerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
      return -ENOTTY; /* Special return value for this case */
    }

  return OK;
}
#endif

static int wifi_manager_process(int argc, char *argv[])
{
  bl_pm_init();
  wifi_main_init();
  ipc_emb_notify();

  wifi_mgmr_drv_init(&g_conf);
  wifi_mgmr_tsk_init();

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_netdev_alloc_txbuf
 *
 * Description:
 *   Allocate wifi transmit buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Non-zero: Buffer address, otherwise error.
 *
 ****************************************************************************/

uint8_t *bl602_netdev_alloc_txbuf(void)
{
  irqstate_t irq = enter_critical_section();
  int        idx = BIT_FFS(TX_BUFF_BIT_SIZE, &g_tx_buf_indicator);
  if (idx == 0)
    {
      leave_critical_section(irq);
      wlwarn("tx buff alloc failed!\n");
      return NULL;
    }

  wlinfo("tx buff alloc:%d\n", idx - 1);
  BIT_CLR(TX_BUFF_BIT_SIZE, idx - 1, &g_tx_buf_indicator);
  leave_critical_section(irq);

  return g_tx_buff[idx - 1];
}

/****************************************************************************
 * Name: bl602_netdev_free_txbuf
 *
 * Description:
 *   Free wifi transmit buffer
 *
 * Input Parameters:
 *   buf: The address of the buffer to be released.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_netdev_free_txbuf(uint8_t *buf)
{
  irqstate_t irq;
  int        idx;

  idx = (tx_buff_t)buf - g_tx_buff;

  DEBUGASSERT(idx < BL602_NET_TXBUFF_NUM && idx >= 0);
  DEBUGASSERT(g_tx_buff[idx] == buf);

  wlinfo("tx buff free %d\n", idx);

  irq = enter_critical_section();
  DEBUGASSERT(!BIT_ISSET(TX_BUFF_BIT_SIZE, idx, &g_tx_buf_indicator));
  BIT_SET(TX_BUFF_BIT_SIZE, idx, &g_tx_buf_indicator);
  leave_critical_section(irq);
}

/****************************************************************************
 * Name: bl602_net_event
 *
 * Description:
 *   BL602 WiFi event handler, called by BL602 wifi manager private library
 *
 * Input Parameters:
 *   event: event number
 *   val: the value of the event
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

void bl602_net_event(int evt, int val)
{
  net_lock();

  switch (evt)
    {
    case CODE_WIFI_ON_CONNECTED:
      do
        {
          struct bl602_net_driver_s *priv = &g_bl602_net[0];
          priv->prev_connectd = 1;
          g_state.sta_connected = 1;

          netdev_carrier_on(&priv->net_dev);

          wifi_mgmr_sta_autoconnect_disable();
          nxsem_post(&g_wifi_connect_sem);
        }
      while (0);
      break;

    case CODE_WIFI_ON_DISCONNECT:
      do
        {
          /* struct bl602_net_driver_s *priv = &g_bl602_net[0]; */

          if (g_state.sta_connected == 1)
            {
              /* netdev_carrier_off(&priv->net_dev); */

              g_state.sta_connected = 0;
            }
        }
      while (0);
      break;

    case CODE_WIFI_CMD_RECONNECT:
      do
        {
          struct bl602_net_driver_s *priv = &g_bl602_net[0];

          wlinfo("retry connect : %d\n", g_state.retry_cnt);
          if (!priv->prev_connectd)
            {
              if (g_state.retry_cnt++ > 3)
                {
                  wifi_mgmr_sta_autoconnect_disable();
                  wifi_mgmr_api_idle();

                  nxsem_post(&g_wifi_connect_sem);
                }
            }
        }
      while (0);
      break;

    case CODE_WIFI_ON_SCAN_DONE:
      do
        {
          g_state.scan_result_status = val;
          nxmutex_unlock(&g_wifi_scan_lock);
        }
      while (0);

    case CODE_WIFI_ON_AP_STARTED:
      do
        {
#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
          struct bl602_net_driver_s *priv = &g_bl602_net[1];
          netdev_carrier_on(&priv->net_dev);
#endif
          g_state.ap_stared = 1;
        }
      while (0);
      break;

    case CODE_WIFI_ON_AP_STOPPED:
      do
        {
#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
          struct bl602_net_driver_s *priv = &g_bl602_net[1];
          netdev_carrier_off(&priv->net_dev);
#endif
          g_state.ap_stared = 0;
        }
      while (0);
      break;

    default:
      wlwarn("unhandled msg:%d\n", evt);
      break;
    }

  net_unlock();
}

/****************************************************************************
 * Name: bl602_net_initialize
 *
 * Description:
 *   Initialize the Wireless controller and driver
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

int bl602_net_initialize(void)
{
  struct bl602_net_driver_s *priv;
  int                        idx;
  uint8_t                    mac[6];

  list_initialize(&g_rx_pending);

  /* Start wifi process */

  wifi_manager_process(0, NULL);

  /* Read the MAC address from the hardware into
   * priv->net_dev.d_mac.ether.ether_addr_octet
   * Applies only if the Wireless MAC has its own internal address.
   */

  bl602_efuse_read_mac_address(mac);
  wlinfo(":::MAC:%x %x %x %x %x %x\n",
         mac[0],
         mac[1],
         mac[2],
         mac[3],
         mac[4],
         mac[5]);

  for (idx = 0; idx < BL602_NET_NINTERFACES; idx++)
    {
      /* Get the interface structure associated with this interface number. */

      priv = &g_bl602_net[idx];

      /* Initialize the driver structure */

      memset(priv, 0, sizeof(struct bl602_net_driver_s));
      priv->net_dev.d_ifup =
        bl602_net_ifup; /* I/F up (new IP address) callback */

      priv->net_dev.d_ifdown  = bl602_net_ifdown;  /* I/F down callback */
      priv->net_dev.d_txavail = bl602_net_txavail; /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
      priv->net_dev.d_addmac = bl602_net_addmac; /* Add multicast MAC address */
      priv->net_dev.d_rmmac  = bl602_net_rmmac;  /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
      priv->net_dev.d_ioctl = bl602_net_ioctl; /* Handle network IOCTL commands */
#endif
      priv->net_dev.d_private = priv; /* Used to recover private state from dev */
      priv->net_dev.d_pktsize =
        BL602_NET_TXBUFF_SIZE - PRESERVE_80211_HEADER_LEN;

#ifdef CONFIG_BL602_NET_MULTI_INTERFACE
      /* Set AP's MAC address equals STA's MAC */

      memcpy(priv->net_dev.d_mac.ether.ether_addr_octet, mac, 6);

      if (idx == 0)
        {
          bl_wifi_sta_mac_addr_set(
              priv->net_dev.d_mac.ether.ether_addr_octet);
        }
      else
        {
          bl_wifi_ap_mac_addr_set(
              priv->net_dev.d_mac.ether.ether_addr_octet);
        }
#else
      DEBUGASSERT(idx == 0);

      memcpy(priv->net_dev.d_mac.ether.ether_addr_octet, mac, 6);
      bl_wifi_sta_mac_addr_set(priv->net_dev.d_mac.ether.ether_addr_octet);
      bl_wifi_ap_mac_addr_set(priv->net_dev.d_mac.ether.ether_addr_octet);
#endif

      /* Enable scan hidden SSID */

      wifi_mgmr_scan_filter_hidden_ssid(0);

      priv->current_mode = IW_MODE_AUTO;

      /* Register the device with the OS so that socket IOCTLs can be
       * performed
       */

      return netdev_register(&priv->net_dev, NET_LL_IEEE80211);
    }

  return OK;
}

#endif /* CONFIG_NET_BL602_NETDEV */
