/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_espnow_pktradio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/pktradio.h>
#include <nuttx/wdog.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_espnow_pktradio.h"

#ifdef CONFIG_ESPRESSIF_ESPNOW_PKTRADIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TX timeout = 1 minute */

#define ESPRESSIF_ESPNOW_TXTOUT (60 * CLK_TCK)

/* We need to have the work queue */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#else
#  if defined(CONFIG_SCHED_LPWORK)
#    define LPBKWORK LPWORK
#  elif defined(CONFIG_SCHED_HPWORK)
#    define LPBKWORK HPWORK
#  else
#    error Neither CONFIG_SCHED_LPWORK nor CONFIG_SCHED_HPWORK defined
#  endif
#endif

#ifndef CONFIG_WIRELESS_PKTRADIO
#  error CONFIG_WIRELESS_PKTRADIO=y is required.
#endif

#ifndef CONFIG_NET_6LOWPAN
#  error CONFIG_NET_6LOWPAN=y is required.
#endif

#if CONFIG_PKTRADIO_ADDRLEN < 2
#error Unsupported configuration, for espnow pktradio \
       CONFIG_PKTRADIO_ADDRLEN needs to be at least 2.
#endif

#if CONFIG_IOB_BUFSIZE < CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_FRAMELEN
#  error Unsupported configuration: reduce \
         CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_FRAMELEN or increase \
         CONFIG_IOB_BUFSIZE.
#endif

/****************************************************************************
 * MAC HDR: 3 byte MAGIC, 2 byte PANID, 1 byte FCB, 2 byte SRC addr,
 *          2 byte DST addr, 4 byte RND (random, optional).
 *
 * FCB (Frame Control Byte):
 *    BIT0-1: frame type: 00 -> advertising frame
 *                        01 -> data frame
 *                        10 -> control frame
 *    BIT2-3: future expansion
 *    BIT4  : ciphered (when ciphered the 4 byte RND is added)
 *    BIT5-7: future expansion
 *
 ****************************************************************************/

#define MAC_MAGIC        "6lo"
#define MAC_PANIDOFFSET  sizeof(MAC_MAGIC) - 1
#define MAC_PANIDLEN     2
#define MAC_FCBOFFSET    MAC_PANIDOFFSET + MAC_PANIDLEN
#define MAC_FCBLEN       1
#define MAC_SRCOFFSET    MAC_FCBOFFSET + MAC_FCBLEN
#define MAC_ADDRLEN      2
#define MAC_DSTOFFSET    MAC_SRCOFFSET + MAC_ADDRLEN
#define MAC_UHDRLEN      MAC_DSTOFFSET + MAC_ADDRLEN  /* Unciphered */
#define MAC_RNDLEN       4
#define MAC_CHDRLEN      MAC_UHDRLEN + MAC_RNDLEN     /* Ciphered */

#define MAC_TYPEMASK     0x03
#define MAC_ADVFCB       0x00
#define MAC_DATAFCB      0x01
#define MAC_CTRLFCB      0x02

#define MAC_ENCMASK      0x10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * The espnow_driver_s encapsulates all state information for a single
 * hardware interface
 *
 ****************************************************************************/

struct espnow_driver_s
{
  bool bifup; /* true:ifup false:ifdown */

  uint8_t panid[MAC_PANIDLEN];      /* network id */

  struct sixlowpan_reassbuf_s iobuffer; /* buffer for packet reassembly */

  FAR struct iob_s *rxhead; /* Head of IOBs queued for rx */
  FAR struct iob_s *rxtail; /* Tail of IOBs queued for rx */
  FAR struct iob_s *txhead; /* Head of IOBs queued for tx */
  FAR struct iob_s *txtail; /* Tail of IOBs queued for tx */

  bool txblocked;           /* Indicates a blocked/failed tx */

  struct wdog_s txtimeout;  /* TX timeout timer */

  struct work_s rxwork;     /* Defer rx work to the work queue */
  struct work_s txwork;     /* Defer tx work to the work queue */
  struct work_s pollwork;   /* Defer pollwork to the work queue */
  struct work_s toutwork;   /* Defer driver reset to the work queue */

  spinlock_t lock;          /* Device specific lock */

  struct radio_driver_s radio;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct espnow_driver_s g_espnow;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static void espnow_addr2ip(FAR struct net_driver_s *dev);
static void espnow_ip2addr(FAR struct net_driver_s *dev);
static inline void espnow_netmask(FAR struct net_driver_s *dev);

/* Queue functions */

static void espnow_txtailadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob);
static void espnow_txheadadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob);
static FAR struct iob_s *espnow_txheadget(FAR struct espnow_driver_s *priv);
static void espnow_rxtailadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob);
static FAR struct iob_s *espnow_rxheadget(FAR struct espnow_driver_s *priv);

/* ESPNOW low level routines */

static int espnow_error_to_errno(esp_err_t err);
static int espnow_send(FAR struct iob_s *iob);
static void espnow_recv_cb(const esp_now_recv_info_t * esp_now_info,
                           const uint8_t *data, int data_len);

/* Translate pktmeta to MAC header and vice versa */

static void espnow_add_header(struct espnow_driver_s *espnow,
                              FAR struct pktradio_metadata_s *pktmeta,
                              FAR struct iob_s *iob);
static void espnow_extract_pktmeta(FAR struct iob_s *iob,
                                   FAR struct pktradio_metadata_s *pktmeta);

/* TX and RX work */

static void espnow_rxwork(FAR void *arg);
static void espnow_txwork(FAR void *arg);

/* TX polling logic */

static void espnow_txavail_work(FAR void *arg);
static int  espnow_txpoll_callback(FAR struct net_driver_s *dev);

/* Watchdog timer expirations */

static void espnow_txtimeout_work(void *arg);
static void espnow_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  espnow_ifup(FAR struct net_driver_s *dev);
static int  espnow_ifdown(FAR struct net_driver_s *dev);
static int  espnow_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int  espnow_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac);
static int  espnow_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  espnow_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int espnow_get_mhrlen(FAR struct radio_driver_s *netdev,
              FAR const void *meta);
static int espnow_req_data(FAR struct radio_driver_s *netdev,
              FAR const void *meta, FAR struct iob_s *framelist);
static int espnow_properties(FAR struct radio_driver_s *netdev,
              FAR struct radiodev_properties_s *properties);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: espnow_addr2ip
 *
 * Description:
 *   Create a MAC-based IP address from the radio address.
 *
 *  128  112  96   80    64   48   32   16
 * ---- ---- ---- ---- ---- ---- ---- ----
 * fe80 0000 0000 0000 0000 00ff fe00 xxxx 2-byte address
 *
 ****************************************************************************/

static void espnow_addr2ip(FAR struct net_driver_s *dev)
{
  FAR struct espnow_driver_s *priv =
    (FAR struct espnow_driver_s *)dev->d_private;
  uint8_t mac_address[MAC_ADDRLEN];

  /* Get the radio address */

  memcpy(mac_address, dev->d_mac.radio.nv_addr, sizeof(mac_address));

  /* Set the IP address based on the 2 byte radio address */

  dev->d_ipv6addr[0] = HTONS(0xfe80);
  dev->d_ipv6addr[1] = 0;
  dev->d_ipv6addr[2] = 0;
  dev->d_ipv6addr[3] = 0;
  dev->d_ipv6addr[4] = 0;
  dev->d_ipv6addr[5] = HTONS(0x00ff);
  dev->d_ipv6addr[6] = HTONS(0xfe00);
  dev->d_ipv6addr[7] = ((uint16_t)mac_address[0] << 8) + mac_address[1];
}

/****************************************************************************
 * Name: espnow_ip2addr
 *
 * Description:
 *   Create a pktradio address from the IP address assigned to the node.
 *
 *  128  112  96   80    64   48   32   16
 * ---- ---- ---- ---- ---- ---- ---- ----
 * AAAA 0000 0000 0000 0000 00ff fe00 aabb -> radio address 0xbbaa
 *
 ****************************************************************************/

static void espnow_ip2addr(FAR struct net_driver_s *dev)
{
  FAR struct espnow_driver_s *priv =
    (FAR struct espnow_driver_s *)dev->d_private;
  uint8_t mac_address[2];

  /* Make sure the IP address is correct */

  dev->d_ipv6addr[4] = 0;
  dev->d_ipv6addr[5] = HTONS(0x00ff);
  dev->d_ipv6addr[6] = HTONS(0xfe00);

  mac_address[0] = (uint8_t)(dev->d_ipv6addr[7] >> 8);
  mac_address[1] = (uint8_t)(dev->d_ipv6addr[7] & 0xff);

  /* Set the radio MAC address as the saddr */

  dev->d_mac.radio.nv_addrlen = MAC_ADDRLEN;
  memcpy(dev->d_mac.radio.nv_addr, mac_address, sizeof(mac_address));
}

/****************************************************************************
 * Name: espnow_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address for pktradio address.
 *
 *  128  112   96   80   64   48   32   16
 * ---- ---- ---- ---- ---- ---- ---- ----
 * ffff ffff ffff ffff ffff ffff ffff 0000 2-byte address
 *
 ****************************************************************************/

static inline void espnow_netmask(FAR struct net_driver_s *dev)
{
  dev->d_ipv6netmask[0] = 0xffff;
  dev->d_ipv6netmask[1] = 0xffff;
  dev->d_ipv6netmask[2] = 0xffff;
  dev->d_ipv6netmask[3] = 0xffff;
  dev->d_ipv6netmask[4] = 0xffff;
  dev->d_ipv6netmask[5] = 0xffff;
  dev->d_ipv6netmask[6] = 0xffff;
  dev->d_ipv6netmask[7] = 0;
}

/****************************************************************************
 * Name: espnow_add_header
 *
 * Description
 *   Create and add the mac header to iob for TX
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   pktmeta - pktmeta
 *   iob     - iob to add the header to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_add_header(struct espnow_driver_s *priv,
                              FAR struct pktradio_metadata_s *pktmeta,
                              FAR struct iob_s *iob)
{
  uint8_t fcb = MAC_DATAFCB;

  memcpy(&iob->io_data[0], MAC_MAGIC, sizeof(MAC_MAGIC) - 1);
  memcpy(&iob->io_data[MAC_PANIDOFFSET], priv->panid, MAC_PANIDLEN);
  memcpy(&iob->io_data[MAC_FCBOFFSET], &fcb, sizeof(fcb));
  memcpy(&iob->io_data[MAC_SRCOFFSET], pktmeta->pm_src.pa_addr, MAC_ADDRLEN);
  memcpy(&iob->io_data[MAC_DSTOFFSET], pktmeta->pm_dest.pa_addr,
         MAC_ADDRLEN);
}

/****************************************************************************
 * Name: espnow_extract_pktmeta
 *
 * Description
 *   Extract pktmeta from received iob
 *
 * Input Parameters:
 *   iob     - iob to extract pktmeta from
 *   pktmeta - pktmeta to update
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_extract_pktmeta(FAR struct iob_s *iob,
                                   FAR struct pktradio_metadata_s *pktmeta)
{
  pktmeta->pm_src.pa_addrlen = MAC_ADDRLEN;
  pktmeta->pm_dest.pa_addrlen = MAC_ADDRLEN;

  memcpy(pktmeta->pm_src.pa_addr, &iob->io_data[MAC_SRCOFFSET], MAC_ADDRLEN);
  memcpy(pktmeta->pm_dest.pa_addr, &iob->io_data[MAC_DSTOFFSET],
         MAC_ADDRLEN);

  iob->io_offset = MAC_UHDRLEN;
}

/****************************************************************************
 * Name: espnow_txtailadd
 *
 * Description
 *   Add a iob to txtail
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   iob  - iob to add
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_txtailadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob)
{
  irqstate_t flags;

  iob->io_flink = NULL;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->txhead == NULL)
    {
      /* Add iob to empty list */

      priv->txhead = iob;
    }
  else
    {
      /* Update tx tail io_flink */

      priv->txtail->io_flink = iob;
    }

  /* Add iob to list */

  priv->txtail = iob;

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: espnow_txheadadd
 *
 * Description
 *   Add a iob to txhead
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   iob  - iob to add
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static void espnow_txheadadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->txhead == NULL)
    {
      /* add iob to empty list */

      priv->txhead = iob;
      priv->txtail = iob;
    }
  else
    {
      /* update iob->io_flink and txhead */

      iob->io_flink = priv->txhead;
      priv->txhead = iob;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: espnow_txheadget
 *
 * Description
 *   Remove the iob from txhead
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   iob pointer
 *
 ****************************************************************************/

static FAR struct iob_s *espnow_txheadget(FAR struct espnow_driver_s *priv)
{
  FAR struct iob_s *iob;
  irqstate_t flags;

  if (priv->txhead == NULL)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->lock);

  /* Remove the IOB from the tx list */

  iob = priv->txhead;
  priv->txhead = iob->io_flink;
  iob->io_flink = NULL;

  /* Did the tx list become empty? */

  if (priv->txhead == NULL)
    {
          priv->txtail = NULL;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return iob;
}

/****************************************************************************
 * Name: espnow_rxtailadd
 *
 * Description
 *   Add a iob to rxtail
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   iob  - iob to add
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_rxtailadd(FAR struct espnow_driver_s *priv,
                             FAR struct iob_s *iob)
{
  irqstate_t flags;

  iob->io_flink = NULL;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->rxhead == NULL)
    {
      /* Add iob to rx list */

      priv->rxhead = iob;
    }
  else
    {
      /* Update rx tail io_flink */

      priv->rxtail->io_flink = iob;
    }

  /* Add iob to rxtail */

  priv->rxtail = iob;

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: espnow_rxheadget
 *
 * Description
 *   Remove the iob from rxhead
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   iob
 *
 ****************************************************************************/

static FAR struct iob_s *espnow_rxheadget(FAR struct espnow_driver_s *priv)
{
  FAR struct iob_s *iob;
  irqstate_t flags;

  if (priv->rxhead == NULL)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->lock);

  /* Remove the IOB from the rx queue */

  iob = priv->rxhead;
  priv->rxhead = iob->io_flink;
  iob->io_flink = NULL;

  /* Did the framelist become empty? */

  if (priv->rxhead == NULL)
    {
          priv->rxtail = NULL;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return iob;
}

/****************************************************************************
 * Name: espnow_error_to_nuttx
 *
 * Description:
 *   Translate espnow_error to nuttx
 *
 * Input Parameters:
 *   err - espnow error
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int espnow_error_to_errno(esp_err_t err)
{
  int ret;

  switch (err)
    {
      case ESP_OK:
        ret = OK;
        break;
      case ESP_ERR_ESPNOW_NOT_INIT:
      case ESP_ERR_ESPNOW_INTERNAL:
        ret = -EIO;
        break;
      case ESP_ERR_ESPNOW_ARG:
        ret = -EINVAL;
        break;
      case ESP_ERR_ESPNOW_NO_MEM:
        ret = -ENOMEM;
        break;
      case ESP_ERR_ESPNOW_NOT_FOUND:
      case ESP_ERR_ESPNOW_IF:
        ret = -ENOSYS;
        break;
      default:
        ret = ERROR;
        break;
    }

  if (ret != OK)
    {
      wlerr("ERROR: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: espnow_send
 *
 * Description:
 *   Broadcast a frame over espnow
 *
 * Input Parameters:
 *   iob - iob to send
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int espnow_send(FAR struct iob_s *iob)
{
  const uint8_t broadcast[] =
    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

  esp_err_t err;

  err = esp_now_send(broadcast, iob->io_data, iob->io_len);
  return espnow_error_to_errno(err);
}

/****************************************************************************
 * Name: espnow_recv_cb
 *
 * Description:
 *   Callback when data is received over espnow, allocate a new iob, copy the
 *   received data and add the iob to the rx queue.
 *
 * Input Parameters:
 *   esp_now_info - info of the broadcasting node (unused),
 *   data         - received data
 *   len          - received data size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_recv_cb(const esp_now_recv_info_t * esp_now_info,
                           const uint8_t *data, int data_len)
{
  FAR struct espnow_driver_s *priv = &g_espnow;
  FAR struct iob_s *iob;
  irqstate_t flags;

  /* Drop received data if interface not up */

  if (!priv->bifup)
    {
      return;
    }

  /* Drop received data if header is too small */

  if (data_len < MAC_UHDRLEN)
    {
      return;
    }

  /* Drop received data if header does not start with magic */

  if (memcmp(data, MAC_MAGIC, sizeof(MAC_MAGIC) - 1))
    {
      return;
    }

  /* Drop received data if header PANID is not ours
   * todo: add support for multiple PANID's
   */

  if (memcmp(&data[MAC_PANIDOFFSET], priv->panid, MAC_PANIDLEN))
    {
      return;
    }

  /* Header data seems ok, continue with processing */

  iob = iob_tryalloc(false);

  /* Ignore the packet if allocation fails or packet too large */

  if ((iob == NULL) || (data_len > CONFIG_IOB_BUFSIZE))
    {
      return;
    }

  /* Copy the data to the iob */

  memcpy(iob->io_data, data, data_len);
  iob->io_len = data_len;

  /* Add the iob to the rx queue */

  espnow_rxtailadd(priv, iob);

  /* Schedule work to queue */

  if (work_available(&priv->rxwork))
    {
      work_queue(LPBKWORK, &priv->rxwork, espnow_rxwork, priv, 0);
    }
}

/****************************************************************************
 * Name: espnow_send_cb
 *
 * Description:
 *   Callback after data is send over espnow, just cancel the watchdog.
 *
 * Input Parameters:
 *   tx_info - Sending information for ESPNOW data
 *   status  - The send result
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status)
{
  FAR struct espnow_driver_s *priv = &g_espnow;

  wd_cancel(&priv->txtimeout);

  /* If TX has been blocked reschedule espnow_txwork */

  if (priv->txblocked)
    {
      priv->txblocked = false;

      /* Schedule the work on the worker thread. */

      if (work_available(&priv->txwork))
        {
          work_queue(LPBKWORK, &priv->txwork, espnow_txwork, priv, 0);
        }
    }
}

/****************************************************************************
 * Name: espnow_transmit
 *
 * Description:
 *   Try to send all TX packets. If this fails break the loop and schedule
 *   return to the transmit.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_transmit(FAR struct espnow_driver_s *priv)
{
  FAR struct iob_s *iob;
  int ret;

  while ((iob = espnow_txheadget(priv)))
    {
      ninfo("Sending IOB %p [%d]\n", iob, iob->io_len);

      ret = espnow_send(iob);

      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->radio.r_dev);

      if (ret < 0)
        {
          nerr("ERROR: espnow_send returned %d\n", ret);
          NETDEV_TXERRORS(&priv->radio.r_dev);
          NETDEV_ERRORS(&priv->radio.r_dev);
        }

      /* Reschedule on failure */

      if (ret == -ENOMEM)
        {
          espnow_txheadadd(priv, iob);
          wd_start(&priv->txtimeout, ESPRESSIF_ESPNOW_TXTOUT,
                   espnow_txtimeout_expiry, (uint32_t)priv);
          priv->txblocked = true;
          break;
        }
      else
        {
          iob_free(iob);
        }
    }
}

/****************************************************************************
 * Name: espnow_txwork
 *
 * Description:
 *   Call espnow_transmit to send available packets to espnow.
 *
 * Input Parameters:
 * priv - Reference to the driver state structure
 *
 *
 ****************************************************************************/

static void espnow_txwork(void *arg)
{
  FAR struct espnow_driver_s *priv = (FAR struct espnow_driver_s *)arg;

  espnow_transmit(priv);
}

/****************************************************************************
 * Name: espnow_rxwork
 *
 * Description:
 *   Call espnow_transmit to send available packets to espnow, process
 *   received packets from espnow and call espnow_transmit again
 *   to send responses.
 *
 * Input Parameters:
 * priv - Reference to the driver state structure
 *
 *
 ****************************************************************************/

static void espnow_rxwork(void *arg)
{
  FAR struct espnow_driver_s *priv = (FAR struct espnow_driver_s *)arg;
  struct pktradio_metadata_s pktmeta;
  FAR struct iob_s *iob;
  int ret;

  /* Send available packets */

  /* Loop while there are frames to be processed, send them to sixlowpan */

  net_lock();

  while ((iob = espnow_rxheadget(priv)))
    {
      /* Make sure the our single packet buffer is attached */

      priv->radio.r_dev.d_buf = priv->iobuffer.rb_buf;

      /* Return the next frame to the network */

      ninfo("Send frame %p to the network:  Offset=%u Length=%u\n",
            iob, iob->io_offset, iob->io_len);

      memset(&pktmeta, 0, sizeof(struct pktradio_metadata_s));

      espnow_extract_pktmeta(iob, &pktmeta);

      ret = sixlowpan_input(&priv->radio, iob, (FAR void *)&pktmeta);

      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->radio.r_dev);

      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_input returned %d\n", ret);
          NETDEV_RXERRORS(&priv->espnow_radio.r_dev);
          NETDEV_ERRORS(&priv->espnow_radio.r_dev);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: espnow_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work (reset) from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static void espnow_txtimeout_work(void *arg)
{
  struct espnow_driver_s *priv = (struct espnow_driver_s *)arg;

  net_lock();

  espnow_ifdown(&priv->radio.r_dev);
  espnow_ifup(&priv->radio.r_dev);

  net_unlock();
}

/****************************************************************************
 * Function: espnow_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer callback handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void espnow_txtimeout_expiry(wdparm_t arg)
{
  struct espnow_driver_s *priv = (struct espnow_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread. */

  if (work_available(&priv->toutwork))
    {
      work_queue(LPBKWORK, &priv->toutwork, espnow_txtimeout_work, priv, 0);
    }
}

/****************************************************************************
 * Name: espnow_txpoll_callback
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

static int espnow_txpoll_callback(FAR struct net_driver_s *dev)
{
  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: espnow_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int espnow_ifup(FAR struct net_driver_s *dev)
{
  FAR struct espnow_driver_s *priv =
    (FAR struct espnow_driver_s *)dev->d_private;
  FAR struct iob_s *iob;
  esp_now_peer_info_t broadcast =
    {
      .peer_addr =
        {
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff
        },

      .channel = 0,
      .encrypt = false,
    };

  /* Free all RX iobs */

  while ((iob = espnow_rxheadget(priv)))
    {
      iob_free(iob);
    }

  if (esp_now_init() != ESP_OK)
    {
      return -ENOSYS;
    }

  if (esp_now_add_peer(&broadcast) != ESP_OK)
    {
      return -ENOSYS;
    }

  if (esp_now_register_recv_cb(espnow_recv_cb) != ESP_OK)
    {
      return -ENOSYS;
    }

  if (esp_now_register_send_cb(espnow_send_cb) != ESP_OK)
    {
      return -ENOSYS;
    }

  espnow_ip2addr(dev);

  ninfo("Bringing up: IPv6 %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
  ninfo("Pan id: %02x:%02x, Pan address: %02x:%02x\n",
        priv->panid[0], priv->panid[1],
        dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1]);

  priv->txblocked = false;
  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Name: espnow_ifdown
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
 *
 ****************************************************************************/

static int espnow_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct espnow_driver_s *priv =
    (FAR struct espnow_driver_s *)dev->d_private;
  FAR struct iob_s *iob;
  const uint8_t broadcast[] =
    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

  ninfo("IP up: %u\n", priv->bifup);

  /* Free all TX iobs */

  while ((iob = espnow_txheadget(priv)))
    {
      iob_free(iob);
    }

  (void)esp_now_del_peer(broadcast);

  (void)esp_now_unregister_recv_cb();

  (void)esp_now_unregister_send_cb();

  (void)esp_now_deinit();

  /* Mark the device "down" */

  priv->bifup = false;

  return OK;
}

/****************************************************************************
 * Name: espnow_txavail_work
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
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void espnow_txavail_work(FAR void *arg)
{
  FAR struct espnow_driver_s *priv = (FAR struct espnow_driver_s *)arg;

  ninfo("TX available work. IP up: %u\n", priv->bifup);

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* If so, then poll the network for new XMIT data */

#ifdef CONFIG_NET_6LOWPAN
      /* Make sure the our single packet buffer is attached */

      priv->radio.r_dev.d_buf = priv->iobuffer.rb_buf;
#endif

      /* Then perform the poll */

      devif_poll(&priv->radio.r_dev, espnow_txpoll_callback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: espnow_txavail
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
 *   Called in normal user mode
 *
 ****************************************************************************/

static int espnow_txavail(FAR struct net_driver_s *dev)
{
  FAR struct espnow_driver_s *priv = (FAR struct espnow_driver_s *)
                                     dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending actions and we will have to ignore the Tx availability
   * action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPBKWORK, &priv->pollwork, espnow_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: espnow_addmac
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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int espnow_addmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac)
{
  /* There is no multicast support in the espnow_pktradio driver */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: espnow_rmmac
 *
 * Description:
 *   NuttX Callback:
 *   Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int espnow_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* There is no multicast support in the espnow_pktradio driver */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: espnow_ioctl
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
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int espnow_ioctl(FAR struct net_driver_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct pktradio_ifreq_s *cmddata;
  FAR struct espnow_driver_s *priv;
  int ret = -ENOTTY;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL && arg != 0ul);
  priv    = (FAR struct espnow_driver_s *)dev->d_private;
  cmddata = (FAR struct pktradio_ifreq_s *)((uintptr_t)arg);

  switch (cmd)
    {
      /* SIOCPKTRADIOGGPROPS
       *   Description:   Get the radio properties
       *   Input:         Pointer to read-write instance of struct
       *                  pktradio_ifreq_s
       *   Output:        Properties returned in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGGPROPS:
        {
          FAR struct radio_driver_s *radio =
            (FAR struct radio_driver_s *)dev;
          FAR struct radiodev_properties_s *props =
            (FAR struct radiodev_properties_s *)&cmddata->pifr_props;

          ret = espnow_properties(radio, props);
        }
        break;

      /* SIOCPKTRADIOSNODE
       *   Description:   Set the radio node address
       *   Input:         Pointer to read-only instance of struct
       *                  pktradio_ifreq_s
       *   Output:        None
       */

      case SIOCPKTRADIOSNODE:
        {
          FAR const struct pktradio_addr_s *newaddr =
            (FAR const struct pktradio_addr_s *)&cmddata->pifr_hwaddr;

          if (newaddr->pa_addrlen != MAC_ADDRLEN)
            {
              ret = -EINVAL;
            }
          else
           {
              FAR struct netdev_varaddr_s *devaddr = &dev->d_mac.radio;

              devaddr->nv_addrlen = MAC_ADDRLEN;
              devaddr->nv_addr[0] = newaddr->pa_addr[0];
              devaddr->nv_addr[1] = newaddr->pa_addr[1];

              /* Update the ip address */

              espnow_addr2ip(dev);

              ret = OK;
            }
        }
        break;

      /* SIOCPKTRADIOGNODE
       *   Description:   Get the radio node address
       *   Input:         Pointer to read-write instance of
       *                  struct pktradio_ifreq_s
       *   Output:        Node address return in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGNODE:
        {
          FAR struct pktradio_addr_s *retaddr =
            (FAR struct pktradio_addr_s *)&cmddata->pifr_hwaddr;
          FAR const struct netdev_varaddr_s *devaddr = &dev->d_mac.radio;

          retaddr->pa_addrlen = devaddr->nv_addrlen;
          retaddr->pa_addr[0] = devaddr->nv_addr[0];
          retaddr->pa_addr[1] = devaddr->nv_addr[1];

          ret = OK;
        }
        break;

      default:
        wlwarn("WARNING: Unrecognized IOCTL command: %02x\n", cmd);
        break;
    }

  UNUSED(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: espnow_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int espnow_get_mhrlen(FAR struct radio_driver_s *netdev,
                             FAR const void *meta)
{
  return MAC_UHDRLEN;
}

/****************************************************************************
 * Name: espnow_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int espnow_req_data(FAR struct radio_driver_s *netdev,
                           FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct espnow_driver_s *priv;
  FAR struct iob_s *iob;
  struct pktradio_metadata_s *pktmeta = (struct pktradio_metadata_s *)meta;
  uint8_t src[MAC_ADDRLEN];
  uint8_t dst[MAC_ADDRLEN];

  DEBUGASSERT(netdev != NULL && netdev->r_dev.d_private != NULL);
  priv = (FAR struct espnow_driver_s *)netdev->r_dev.d_private;

  DEBUGASSERT(meta != NULL && framelist != NULL);

  /* Add the incoming list of framelist to queue of framelist to tx */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      memcpy(src, pktmeta->pm_src.pa_addr, MAC_ADDRLEN);
      memcpy(dst, pktmeta->pm_dest.pa_addr, MAC_ADDRLEN);
      ninfo("Queuing frame %p size %d: %02x:%02x --> %02x:%02x\n", iob,
            iob->io_len, src[0], src[1], dst[0], dst[1]);

      /* Add the MAC header */

      espnow_add_header(priv, pktmeta, iob);

      /* Add the IOB to the tail of the tx list */

      espnow_txtailadd(priv, iob);
    }

  /* Schedule the txwork on the worker thread. */

  if (work_available(&priv->txwork))
    {
      work_queue(LPBKWORK, &priv->txwork, espnow_txwork, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: espnow_properties
 *
 * Description:
 *   Different packet radios may have different properties.  If there are
 *   multiple packet radios, then those properties have to be queried at
 *   run time.  This information is provided to the 6LoWPAN network via the
 *   following structure.
 *
 * Input Parameters:
 *   netdev     - The network device to be queried
 *   properties - Location where radio properties will be returned.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int espnow_properties(FAR struct radio_driver_s *netdev,
                             FAR struct radiodev_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct radiodev_properties_s));

  /* General */

  /* Length of an address */

  properties->sp_addrlen  = MAC_ADDRLEN;

  /* Fixed frame length */

  properties->sp_framelen = CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_FRAMELEN;

  /* Multicast address */

  properties->sp_mcast.nv_addrlen = MAC_ADDRLEN;
  memset(properties->sp_mcast.nv_addr, 0xee, MAC_ADDRLEN);

  /* Broadcast address */

  properties->sp_bcast.nv_addrlen = MAC_ADDRLEN;
  memset(properties->sp_mcast.nv_addr, 0xff, MAC_ADDRLEN);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pktradio_espnow
 *
 * Description:
 *   Initialize and register the 6LoWPAN over espnow MAC network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pktradio_espnow(void)
{
  FAR struct espnow_driver_s *priv;
  FAR struct radio_driver_s *radio;
  FAR struct net_driver_s *dev;
  uint8_t mac_address[MAC_ADDRLEN];

  ninfo("Initializing\n");

  /* Get the interface structure associated with this interface number. */

  priv = &g_espnow;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct espnow_driver_s));

  radio               = &priv->radio;
  dev                 = &radio->r_dev;
  dev->d_ifup         = espnow_ifup;          /* I/F up callback */
  dev->d_ifdown       = espnow_ifdown;        /* I/F down callback */
  dev->d_txavail      = espnow_txavail;       /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  dev->d_addmac       = espnow_addmac;        /* Add multicast MAC address */
  dev->d_rmmac        = espnow_rmmac;         /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = espnow_ioctl;         /* Handle network IOCTL commands */
#endif
  dev->d_private      = priv;                 /* Link private state to dev */

  /* Set the PANID */

  priv->panid[0] = CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_PANID >> 8;
  priv->panid[1] = CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_PANID & 0xff;

  /* Set the PANADDRESS */

  mac_address[0] = CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_PANADDR >> 8;
  mac_address[1] = CONFIG_ESPRESSIF_ESPNOW_PKTRADIO_PANADDR & 0xff;

  dev->d_mac.radio.nv_addrlen = MAC_ADDRLEN;
  memcpy(dev->d_mac.radio.nv_addr, mac_address, sizeof(mac_address));

  /* Set the MAC-based IP address */

  espnow_addr2ip(dev);
  espnow_netmask(dev);

  /* Initialize the Network frame-related callbacks */

  radio->r_get_mhrlen = espnow_get_mhrlen;    /* Get MAC header length */
  radio->r_req_data   = espnow_req_data;      /* Enqueue frame for transmission */
  radio->r_properties = espnow_properties;    /* Returns radio properties */

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached.
   * We must do this before registering the device since,
   * once the device is registered, a packet may
   * be attempted to be forwarded and require the buffer.
   */

  priv->radio.r_dev.d_buf = priv->iobuffer.rb_buf;
#endif

  /* Register the packet radio device with the OS so that socket IOCTLs can
   * be performed.
   */

  netdev_register(&priv->radio.r_dev, NET_LL_PKTRADIO);

  return OK;
}

#endif /* CONFIG_ESPRESSIF_ESPNOW_PKTRADIO */
