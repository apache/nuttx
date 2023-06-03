/****************************************************************************
 * wireless/bluetooth/bt_netdev.c
 * Network stack interface
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

#include <arpa/inet.h>

#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <netpacket/bluetooth.h>

#include "bt_hcicore.h"

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
#include "bt_l2cap.h"
#include "bt_conn.h"
#endif

#include "bt_ioctl.h"

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_BLUETOOTH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* Frame size */

#if BLUETOOTH_MAX_FRAMELEN > CONFIG_IOB_BUFSIZE
#  error CONFIG_IOB_BUFSIZE to small for max Bluetooth frame
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is our private version of the MAC callback structure */

struct btnet_callback_s
{
  /* This holds the information visible to the MAC layer */

  FAR struct btnet_driver_s *bc_priv;  /* Our priv data */
};

/* The btnet_driver_s encapsulates all state information for a single
 * Bluetooth device interface.
 */

struct btnet_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct radio_driver_s bd_dev;      /* Interface understood by the network */
                                     /* Cast compatible with struct btnet_driver_s */

  /* For internal use by this driver */

  bool bd_bifup;                     /* true:ifup false:ifdown */
  struct work_s bd_pollwork;         /* Defer poll work to the work queue */

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  struct bt_conn_cb_s bd_hcicb;      /* HCI connection status callbacks */
  struct bt_l2cap_chan_s bd_l2capcb; /* L2CAP status callbacks */
#else
  struct bt_hci_cb_s bd_hcicb;       /* HCI RAW packet callbacks */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions ********************************************************/

static int  btnet_advertise(FAR struct net_driver_s *netdev);
static inline void btnet_netmask(FAR struct net_driver_s *netdev);

/* Bluetooth callback functions *********************************************/

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/* L2CAP callbacks */

static void btnet_l2cap_connected(FAR struct bt_conn_s *conn,
              FAR void *context, uint16_t cid);
static void btnet_l2cap_disconnected(FAR struct bt_conn_s *conn,
              FAR void *context, uint16_t cid);
static void btnet_l2cap_encrypt_change(FAR struct bt_conn_s *conn,
              FAR void *context, uint16_t cid);
static void btnet_l2cap_receive(FAR struct bt_conn_s *conn,
              FAR struct bt_buf_s *buf, FAR void *context, uint16_t cid);

/* HCI callbacks */

static void btnet_hci_connected(FAR struct bt_conn_s *conn,
              FAR void *context);
static void btnet_hci_disconnected(FAR struct bt_conn_s *conn,
              FAR void *context);
#else
static void btnet_hci_received(FAR struct bt_buf_s *buf, FAR void *context);
#endif

/* Network interface support ************************************************/

/* Common TX logic */

static int  btnet_txpoll_callback(FAR struct net_driver_s *netdev);

/* NuttX callback functions */

static int  btnet_ifup(FAR struct net_driver_s *netdev);
static int  btnet_ifdown(FAR struct net_driver_s *netdev);

static void btnet_txavail_work(FAR void *arg);
static int  btnet_txavail(FAR struct net_driver_s *netdev);

#ifdef CONFIG_NET_MCASTGROUP
static int  btnet_addmac(FAR struct net_driver_s *netdev,
              FAR const uint8_t *mac);
static int  btnet_rmmac(FAR struct net_driver_s *netdev,
              FAR const uint8_t *mac);
#endif
static int  btnet_get_mhrlen(FAR struct radio_driver_s *netdev,
              FAR const void *meta);
static int  btnet_req_data(FAR struct radio_driver_s *netdev,
              FAR const void *meta, FAR struct iob_s *framelist);
static int  btnet_properties(FAR struct radio_driver_s *netdev,
              FAR struct radiodev_properties_s *properties);

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
static int  btnet_req_l2cap_data(FAR struct btnet_driver_s *priv,
                                 FAR struct bluetooth_frame_meta_s *meta,
                                 FAR struct iob_s *framelist);
#endif

static int  btnet_req_hci_data(FAR struct btnet_driver_s *priv,
                               FAR struct bluetooth_frame_meta_s *meta,
                               FAR struct iob_s *framelist);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static struct sixlowpan_reassbuf_s g_iobuffer;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btnet_advertise
 *
 * Description:
 *   Advertise the MAC and IPv6 address for this node.
 *
 *   Creates a MAC-based IP address from the 6-byte address address assigned
 *   to the device.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0200 xxxx xxxx xxxx
 *
 ****************************************************************************/

static int btnet_advertise(FAR struct net_driver_s *netdev)
{
  FAR uint8_t *addr;

  DEBUGASSERT(netdev != NULL && netdev->d_private != NULL);

  /* Get the 6-byte local address from the device.
   *
   * REVISIT: The use of the g_btdev global restricts the implementation to
   * a single Bluetooth device.
   */

  addr = g_btdev.bdaddr.val;

  /* Set the MAC address using 6-byte local address from the device. */

  BLUETOOTH_ADDRCOPY(netdev->d_mac.radio.nv_addr, addr);
  netdev->d_mac.radio.nv_addrlen = BLUETOOTH_ADDRSIZE;

#ifdef CONFIG_NET_IPv6
  /* Set the IP address based on the 6-byte address */

  netdev->d_ipv6addr[0] = HTONS(0xfe80);
  netdev->d_ipv6addr[1] = 0;
  netdev->d_ipv6addr[2] = 0;
  netdev->d_ipv6addr[3] = 0;
  netdev->d_ipv6addr[4] = HTONS(0x0200);
  netdev->d_ipv6addr[5] = (uint16_t)addr[0] << 8 | (uint16_t)addr[1];
  netdev->d_ipv6addr[6] = (uint16_t)addr[2] << 8 | (uint16_t)addr[3];
  netdev->d_ipv6addr[7] = (uint16_t)addr[4] << 8 | (uint16_t)addr[5];
#endif

  return OK;
}

/****************************************************************************
 * Name: btnet_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address which is based on the 6-byte
 *   Bluetooth address.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx
 *
 ****************************************************************************/

static inline void btnet_netmask(FAR struct net_driver_s *netdev)
{
#ifdef CONFIG_NET_IPv6
  netdev->d_ipv6netmask[0]  = 0xffff;
  netdev->d_ipv6netmask[1]  = 0xffff;
  netdev->d_ipv6netmask[2]  = 0xffff;
  netdev->d_ipv6netmask[3]  = 0xffff;
  netdev->d_ipv6netmask[4]  = 0;
  netdev->d_ipv6netmask[5]  = 0;
  netdev->d_ipv6netmask[6]  = 0;
  netdev->d_ipv6netmask[7]  = 0;
#endif
}

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/****************************************************************************
 * Name: btnet_hci_connect/disconnect/encrypt_change
 *
 * Description:
 *   There are callbacks that are involved by the core HCI layer when a
 *   change is detected in the connection status or encryption.
 *
 * Input Parameters:
 *   conn - The connection whose
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   No assumption should be made about the thread of execution that these
 *   are called from
 *
 ****************************************************************************/

static void btnet_l2cap_connected(FAR struct bt_conn_s *conn,
                                  FAR void *context, uint16_t cid)
{
  wlinfo("Connected\n");
}

static void btnet_l2cap_disconnected(FAR struct bt_conn_s *conn,
                                     FAR void *context, uint16_t cid)
{
  wlinfo("Disconnected\n");
}

static void btnet_l2cap_encrypt_change(FAR struct bt_conn_s *conn,
                                       FAR void *context, uint16_t cid)
{
  wlinfo("Encryption change\n");
}

/****************************************************************************
 * Name: btnet_l2cap_receive
 *
 * Description:
 *   Handle received frames forward by the Bluetooth L2CAP layer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  On success, the meta data and its contained iob will be
 *   freed.   The meta data will be intact if this function returns a
 *   failure.
 *
 ****************************************************************************/

static void btnet_l2cap_receive(FAR struct bt_conn_s *conn,
                                FAR struct bt_buf_s *buf,
                                FAR void *context, uint16_t cid)
{
  FAR struct btnet_driver_s *priv;
  FAR struct iob_s *frame;
  struct bluetooth_frame_meta_s meta;
  int ret = -ENODEV;

  wlinfo("Received frame\n");

  DEBUGASSERT(conn != NULL && buf != NULL && buf->frame != NULL &&
              context != NULL && cid < UINT8_MAX);

  /* Detach the IOB frame from the buffer structure */

  frame      = buf->frame;
  buf->frame = NULL;

  /* Ignore the frame if the network is not up */

  priv = (FAR struct btnet_driver_s *)context;
  if (!priv->bd_bifup)
    {
      wlwarn("WARNING: Dropped... Network is down\n");
      goto drop;
    }

  /* Make sure that the size/offset data matches the buffer structure data.
   * REVISIT:  Wouldn't it be better to just have one copy rather than having
   * to synchronize?
   */

  frame->io_len    = buf->len;
  frame->io_pktlen = buf->len;
  frame->io_offset = (unsigned int)
    ((uintptr_t)buf->data - (uintptr_t)frame->io_data);

  DEBUGASSERT(frame->io_len <= CONFIG_IOB_BUFSIZE);
  DEBUGASSERT(frame->io_offset < CONFIG_IOB_BUFSIZE);

  /* Construct the frame meta data.
   * REVISIT: Where do we get the channel number?
   */

  BLUETOOTH_ADDRCOPY(meta.bm_raddr.val, conn->src.val);
  meta.bm_channel = cid;

  /* Transfer the frame to the network logic */

  net_lock();

#ifdef CONFIG_NET_BLUETOOTH
  /* Invoke the PF_BLUETOOTH tap first.  If the frame matches
   * with a connected PF_BLUETOOTH socket, it will take the
   * frame and return success.
   */

  ret = bluetooth_input(&priv->bd_dev, frame, (FAR void *)&meta);
#endif
#ifdef CONFIG_NET_6LOWPAN
  if (ret < 0)
    {
      /* If the frame is not a 6LoWPAN frame, then thefirst byte at the
       * io_offset should be a valid IPHC header.
       */

      if ((frame->io_data[frame->io_offset] &
           SIXLOWPAN_DISPATCH_NALP_MASK) == SIXLOWPAN_DISPATCH_NALP)
        {
          wlwarn("WARNING: Dropped... Not a 6LoWPAN frame: %02x\n",
                 frame->io_data[frame->io_offset]);
          ret = -EINVAL;
        }
      else
        {
          /* Make sure the our single packet buffer is attached */

          priv->bd_dev.r_dev.d_buf = g_iobuffer.rb_buf;

          /* And give the packet to 6LoWPAN */

          ret = sixlowpan_input(&priv->bd_dev, frame, (FAR void *)&meta);
        }
    }
#endif

drop:

  /* Handle errors */

  if (ret < 0)
    {
      iob_free(frame);

      /* Increment statistics */

      NETDEV_RXDROPPED(&priv->bd_dev.r_dev);
    }
  else
    {
      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->bd_dev.r_dev);
      NETDEV_RXIPV6(&priv->bd_dev.r_dev);
    }

  /* Release our reference on the buffer */

  bt_buf_release(buf);
  net_unlock();
}

/****************************************************************************
 * Name: btnet_hci_connect/disconnect
 *
 * Description:
 *   There are callbacks that are involved by the core HCI layer when a
 *   change is detected in the connection status.
 *
 * Input Parameters:
 *   conn - The connection whose
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   No assumption should be made about the thread of execution that these
 *   are called from
 *
 ****************************************************************************/

static void btnet_hci_connected(FAR struct bt_conn_s *conn,
                                FAR void *context)
{
  wlinfo("Connected\n");
}

static void btnet_hci_disconnected(FAR struct bt_conn_s *conn,
                                   FAR void *context)
{
  wlinfo("Disconnected\n");
}
#else

/****************************************************************************
 * Name: btnet_hci_received
 *
 * Description:
 *   This callback is called from the RX queue handling when an HCI
 *   packet is received from controller.
 *
 * Input Parameters:
 *   buf - The packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   No assumption should be made about the thread of execution that these
 *   are called from
 *
 ****************************************************************************/

static void btnet_hci_received(FAR struct bt_buf_s *buf, FAR void *context)
{
  FAR struct btnet_driver_s *priv;
  FAR struct iob_s *frame;
  struct bluetooth_frame_meta_s meta;
  int ret = -ENODEV;

  wlinfo("Received frame\n");

  DEBUGASSERT(buf != NULL && buf->frame != NULL);

  /* Detach the IOB frame from the buffer structure */

  frame      = buf->frame;
  buf->frame = NULL;

  net_lock();

  /* Ignore the frame if the network is not up */

  priv = (FAR struct btnet_driver_s *)context;
  if (!priv->bd_bifup)
    {
      wlwarn("WARNING: Dropped... Network is down\n");
      goto drop;
    }

  /* Make sure that the size/offset data matches the buffer structure data. */

  DEBUGASSERT(frame->io_offset == BLUETOOTH_H4_HDRLEN);

  /* Rearrange IOB to consider H4 header */

  frame->io_len    = buf->len + frame->io_offset;
  frame->io_pktlen = buf->len + frame->io_offset;
  frame->io_offset = 0;

  DEBUGASSERT(frame->io_len <= CONFIG_IOB_BUFSIZE);

  /* Write H4 header */

  switch (buf->type)
    {
      case BT_EVT:
        frame->io_data[0] = HCI_EVENT_PKT;
        break;
      case BT_ACL_IN:
        frame->io_data[0] = HCI_ACLDATA_PKT;
        break;
      default:
        wlerr("Bad HCI type: %i\n", buf->type);
        goto drop;
        break;
    }

  /* Construct the frame meta data.
   */

  meta.bm_channel = HCI_CHANNEL_RAW;
  meta.bm_proto = BTPROTO_HCI;

  /* Transfer the frame to the network logic */

#ifdef CONFIG_NET_BLUETOOTH
  /* Invoke the PF_BLUETOOTH tap first.  If the frame matches
   * with a connected PF_BLUETOOTH socket, it will take the
   * frame and return success.
   */

  ret = bluetooth_input(&priv->bd_dev, frame, (FAR void *)&meta);
#endif

drop:

  /* Handle errors */

  if (ret < 0)
    {
      iob_free(frame);

      /* Increment statistics */

      NETDEV_RXDROPPED(&priv->bd_dev.r_dev);
    }
  else
    {
      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->bd_dev.r_dev);
      NETDEV_RXIPV6(&priv->bd_dev.r_dev);
    }

  /* Release our reference on the buffer */

  net_unlock();
}

#endif

/****************************************************************************
 * Name: btnet_txpoll_callback
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
 *   netdev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int btnet_txpoll_callback(FAR struct net_driver_s *netdev)
{
  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: btnet_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Bluetooth interface when an IP address
 *   is provided
 *
 * Input Parameters:
 *   netdev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int btnet_ifup(FAR struct net_driver_s *netdev)
{
  FAR struct btnet_driver_s *priv =
    (FAR struct btnet_driver_s *)netdev->d_private;
  int ret;

  /* Set the IP address based on the addressing assigned to the node */

  ret = btnet_advertise(netdev);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_IPv6
      wlinfo("Bringing up: IP   %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
             netdev->d_ipv6addr[0], netdev->d_ipv6addr[1],
             netdev->d_ipv6addr[2], netdev->d_ipv6addr[3],
             netdev->d_ipv6addr[4], netdev->d_ipv6addr[5],
             netdev->d_ipv6addr[6], netdev->d_ipv6addr[7]);
      wlinfo("             ADDR %02x:%02x:%02x:%02x:%02x:%02x\n",
             netdev->d_mac.radio.nv_addr[0], netdev->d_mac.radio.nv_addr[1],
             netdev->d_mac.radio.nv_addr[2], netdev->d_mac.radio.nv_addr[3],
             netdev->d_mac.radio.nv_addr[4], netdev->d_mac.radio.nv_addr[5]);

#else
      wlinfo("Bringing up: %02x:%02x:%02x:%02x:%02x:%02x\n",
             netdev->d_mac.radio.nv_addr[0], netdev->d_mac.radio.nv_addr[1],
             netdev->d_mac.radio.nv_addr[2], netdev->d_mac.radio.nv_addr[3],
             netdev->d_mac.radio.nv_addr[4], netdev->d_mac.radio.nv_addr[5]);
#endif

      /* The interface is now up */

      priv->bd_bifup = true;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: btnet_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   netdev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int btnet_ifdown(FAR struct net_driver_s *netdev)
{
  FAR struct btnet_driver_s *priv =
    (FAR struct btnet_driver_s *)netdev->d_private;
  irqstate_t flags;

  /* Disable interruption */

  flags = spin_lock_irqsave(NULL);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the btnet_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->bd_bifup = false;
  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: btnet_txavail_work
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

static void btnet_txavail_work(FAR void *arg)
{
  FAR struct btnet_driver_s *priv = (FAR struct btnet_driver_s *)arg;

  wlinfo("ifup=%u\n", priv->bd_bifup);

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bd_bifup)
    {
#ifdef CONFIG_NET_6LOWPAN
      /* Make sure the our single packet buffer is attached */

      priv->bd_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

      /* Then poll the network for new XMIT data */

      devif_poll(&priv->bd_dev.r_dev, btnet_txpoll_callback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: btnet_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   netdev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int btnet_txavail(FAR struct net_driver_s *netdev)
{
  FAR struct btnet_driver_s *priv =
    (FAR struct btnet_driver_s *)netdev->d_private;

  wlinfo("Available=%u\n", work_available(&priv->bd_pollwork));

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->bd_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPWORK, &priv->bd_pollwork, btnet_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: btnet_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   netdev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int btnet_addmac(FAR struct net_driver_s *netdev,
                        FAR const uint8_t *mac)
{
  /* Add the MAC address to the hardware multicast routing table.
   *  Not used with Bluetooth.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: btnet_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   netdev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int btnet_rmmac(FAR struct net_driver_s *netdev,
                       FAR const uint8_t *mac)
{
  /* Remove the MAC address from the hardware multicast routing table
   *  Not used with Bluetooth.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: btnet_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input Parameters:
 *   netdev  - The networkd device that will mediate the MAC interface
 *   meta    - Obfuscated meta-data structure needed to create the radio
 *             MAC header
 *
 * Returned Value:
 *   A non-negative MAC header length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int btnet_get_mhrlen(FAR struct radio_driver_s *netdev,
                            FAR const void *meta)
{
  const struct bluetooth_frame_meta_s *btmeta = meta;

  if (btmeta->bm_proto == BTPROTO_HCI)
    {
      /* the net device only requires the H4 header, the rest is already
       * part of the packet
       */

      return BLUETOOTH_H4_HDRLEN;
    }
  else if (btmeta->bm_proto == BTPROTO_L2CAP)
    {
      /* Report the complete header size, since H4 + ACL + L2CAP header
       * will not be part of the packet
       */

      /* TODO: correct? */

      return BLUETOOTH_MAX_HDRLEN;
    }

  return BLUETOOTH_MAX_HDRLEN;
}

/****************************************************************************
 * Name: btnet_req_data
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

static int btnet_req_data(FAR struct radio_driver_s *netdev,
                          FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct btnet_driver_s *priv;
  FAR struct bluetooth_frame_meta_s *btmeta;

  priv   = (FAR struct btnet_driver_s *)netdev;
  btmeta = (FAR struct bluetooth_frame_meta_s *)meta;

  wlinfo("Received framelist\n");
  DEBUGASSERT(priv != NULL && meta != NULL && framelist != NULL);

  if (btmeta->bm_proto == BTPROTO_HCI)
    {
      return btnet_req_hci_data(priv, btmeta, framelist);
    }
#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  else if (btmeta->bm_proto == BTPROTO_L2CAP)
    {
      return btnet_req_l2cap_data(priv, btmeta, framelist);
    }
#endif
  else
    {
      return -EOPNOTSUPP;
    }

  return OK;
}

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/****************************************************************************
 * Name: btnet_req_l2cap_data
 *
 * Description:
 *   Requests the transfer of a list of L2CAP frames to the MAC.
 *
 * Input Parameters:
 *   priv      - Bluetooth network device
 *   btmeta    - Bluetooth frame metadata
 *   framelist - Head of a list of L2CAP frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int  btnet_req_l2cap_data(FAR struct btnet_driver_s *priv,
                                 FAR struct bluetooth_frame_meta_s *btmeta,
                                 FAR struct iob_s *framelist)
{
  FAR struct bt_conn_s *conn;
  bt_addr_le_t peer;
  FAR struct iob_s *iob;
  FAR struct bt_buf_s *buf;
  UNUSED(priv);

  /* Create a connection structure for this peer if one does not already
   * exist.
   *
   * Assumptions to REVISIT:
   *
   *   1. Role is Master (see bt_conn_create_le())
   *   2. Address type is BT_ADDR_LE_PUBLIC (vs. BT_ADDR_LE_RANDOM)
   */

  BLUETOOTH_ADDRCOPY(peer.val, btmeta->bm_raddr.val);
  peer.type = BT_ADDR_LE_PUBLIC;

  conn = bt_conn_create_le(&peer);
  if (conn == NULL)
    {
      /* bt_conn_create_le() can fail if (1) the connection exists, but is
       * in a bad state or (2) CONFIG_BLUETOOTH_MAX_CONN has been exceeded.
       * Assume the latter.
       */

      return -ENOMEM;
    }

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->bd_dev.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      DEBUGASSERT(iob->io_offset == BLUETOOTH_MAX_HDRLEN &&
                  iob->io_len >= BLUETOOTH_MAX_HDRLEN);

      /* Allocate a buffer to contain the IOB */

      buf = bt_buf_alloc(BT_ACL_OUT, iob, BLUETOOTH_MAX_HDRLEN);
      if (buf == NULL)
        {
          wlerr("ERROR:  Failed to allocate buffer container\n");
          return -ENOMEM;
        }

      /* Transfer the frame to the Bluetooth stack. */

      bt_l2cap_send(conn, (uint16_t)btmeta->bm_channel, buf);
      NETDEV_TXDONE(&priv->bd_dev.r_dev);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: btnet_req_hci_data
 *
 * Description:
 *   Requests the transfer of a list of HCI frames to the MAC.
 *
 * Input Parameters:
 *   priv      - Bluetooth network device
 *   btmeta    - Bluetooth frame metadata
 *   framelist - Head of a list of HCI frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int  btnet_req_hci_data(FAR struct btnet_driver_s *priv,
                               FAR struct bluetooth_frame_meta_s *meta,
                               FAR struct iob_s *framelist)
{
  FAR struct iob_s *iob;
  FAR struct bt_buf_s *buf;

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->bd_dev.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      DEBUGASSERT(iob->io_offset == BLUETOOTH_H4_HDRLEN &&
                  iob->io_len >= BLUETOOTH_H4_HDRLEN);

      /* Allocate a buffer to contain the IOB */

      switch (iob->io_data[iob->io_offset])
        {
          case HCI_ACLDATA_PKT:
            iob->io_offset += 1;
            buf = bt_buf_alloc(BT_ACL_OUT, iob, 0);
            break;
          case HCI_COMMAND_PKT:
            iob->io_offset += 1;
            buf = bt_buf_alloc(BT_CMD, iob, 0);
            break;
          case HCI_EVENT_PKT:
            iob->io_offset += 1;
            buf = bt_buf_alloc(BT_EVT, iob, 0);
            break;
          default:
            return -EOPNOTSUPP;
            break;
        }

      if (buf == NULL)
        {
          wlerr("ERROR:  Failed to allocate buffer container\n");
          return -ENOMEM;
        }

      bt_send(g_btdev.btdev, buf);
      bt_buf_release(buf);

      /* Transfer the frame to the Bluetooth stack. */

      NETDEV_TXDONE(&priv->bd_dev.r_dev);
    }

  return OK;
}

/****************************************************************************
 * Name: btnet_properties
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

static int btnet_properties(FAR struct radio_driver_s *netdev,
                            FAR struct radiodev_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct radiodev_properties_s));

  /* General */

  properties->sp_addrlen  = BLUETOOTH_ADDRSIZE;     /* Length of an address */
  properties->sp_framelen = BLUETOOTH_MAX_FRAMELEN; /* Fixed frame length */

  /* Multicast, multicast, and star hub node addresses not supported */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_netdev_register
 *
 * Description:
 *   Register a network driver to access the Bluetooth layer using a 6LoWPAN
 *   IPv6 or AF_BLUETOOTH socket.
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int bt_netdev_register(FAR struct bt_driver_s *btdev)
{
  FAR struct btnet_driver_s *priv;
  FAR struct radio_driver_s *radio;
  FAR struct net_driver_s  *netdev;

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  FAR struct bt_conn_cb_s *hcicb;
  FAR struct bt_l2cap_chan_s *l2capcb;
#else
  FAR struct bt_hci_cb_s *hcicb;
#endif
  int ret;

  /* Get the interface structure associated with this interface number. */

  btdev->bt_net = priv = (FAR struct btnet_driver_s *)
    kmm_zalloc(sizeof(struct btnet_driver_s));

  if (priv == NULL)
    {
      nerr("ERROR: Failed to allocate the device structure\n");
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  radio               = &priv->bd_dev;
  netdev              = &radio->r_dev;
  netdev->d_ifup      = btnet_ifup;        /* I/F up (new IP address) callback */
  netdev->d_ifdown    = btnet_ifdown;      /* I/F down callback */
  netdev->d_txavail   = btnet_txavail;     /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  netdev->d_addmac    = btnet_addmac;      /* Add multicast MAC address */
  netdev->d_rmmac     = btnet_rmmac;       /* Remove multicast MAC address */
#endif
#if defined(CONFIG_NETDEV_IOCTL) && defined(CONFIG_WIRELESS_BLUETOOTH_HOST)
  netdev->d_ioctl     = btnet_ioctl;       /* Handle network IOCTL commands */
#endif
  netdev->d_private   = priv;              /* Used to recover private state from netdev */

  /* Connection status change callbacks */

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  hcicb               = &priv->bd_hcicb;
  hcicb->context      = priv;
  hcicb->connected    = btnet_hci_connected;
  hcicb->disconnected = btnet_hci_disconnected;

  bt_conn_cb_register(hcicb);

  /* L2CAP status change callbacks */

  l2capcb                 = &priv->bd_l2capcb;
  l2capcb->context        = priv;
  l2capcb->connected      = btnet_l2cap_connected;
  l2capcb->disconnected   = btnet_l2cap_disconnected;
  l2capcb->encrypt_change = btnet_l2cap_encrypt_change;
  l2capcb->receive        = btnet_l2cap_receive;

  bt_l2cap_chan_default(l2capcb);
#else
  hcicb                = &priv->bd_hcicb;
  hcicb->context       = priv;
  hcicb->received      = btnet_hci_received;

  bt_hci_cb_register(hcicb);
#endif

  /* Set the network mask. */

  btnet_netmask(netdev);

  /* Initialize the Network frame-related callbacks */

  radio->r_get_mhrlen = btnet_get_mhrlen;  /* Get MAC header length */
  radio->r_req_data   = btnet_req_data;    /* Enqueue frame for transmission */
  radio->r_properties = btnet_properties;  /* Return radio properties */

  btdev->receive      = bt_receive;

  /* Associate the driver in with the Bluetooth stack.
   *
   * REVISIT:  We will eventually need to remember which Bluetooth device
   * we a serving.  Not a problem now because only a single BLE device is
   * supported.
   */

  ret = bt_driver_register(btdev);
  if (ret < 0)
    {
      nerr("ERROR: bt_driver_register() failed: %d\n", ret);
      goto errout;
    }

  /* Initialize the Bluetooth stack.
   *
   * REVISIT:  This function should be called only once after all BLE
   * drivers are registered.  Not a problem now because only a single
   * BLE device is supported.
   */

  ret = bt_initialize();
  if (ret < 0)
    {
      nerr("ERROR:  bt_initialize() failed: %d\n", ret);
      goto errout;
    }

  /* Put the interface in the down state. */

  btnet_ifdown(netdev);

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached.
   * We must do this before registering the device since, once the device is
   * registered, a packet may be attempted to be forwarded and require the
   * buffer.
   */

  priv->bd_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  bt_add_services();
#endif

  /* Register the network device with the OS so that socket IOCTLs can be
   * performed
   */

  ret = netdev_register(&priv->bd_dev.r_dev, NET_LL_BLUETOOTH);
  if (ret >= 0)
    {
      return OK;
    }

  nerr("ERROR: netdev_register() failed: %d\n", ret);

errout:

  btnet_ifdown(netdev);
  bt_driver_unregister(btdev);

  /* Free memory and return the error */

  kmm_free(btdev->bt_net);
  btdev->bt_net = NULL;
  return ret;
}

/****************************************************************************
 * Name: bt_netdev_unregister
 *
 * Description:
 *   Unregister a network a driver registered by bt_netdev_register.
 *
 * Input Parameters:
 *   btdev - An instance of the low-level driver interface structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int bt_netdev_unregister(FAR struct bt_driver_s *btdev)
{
  int ret;
  FAR struct btnet_driver_s *priv;

  if (!btdev)
    {
      return -EINVAL;
    }

  priv = (FAR struct btnet_driver_s *)btdev->bt_net;
  if (!priv)
    {
      nerr("ERROR: bt_driver_s is probably not registered\n");
      return -EINVAL;
    }

  btnet_ifdown(&priv->bd_dev.r_dev);

  ret = netdev_unregister(&priv->bd_dev.r_dev);
  if (ret < 0)
    {
      nerr("ERROR: netdev_unregister bfailed: %d\n", ret);
    }

  bt_deinitialize();

  bt_driver_unregister(btdev);

  kmm_free(btdev->bt_net);
  btdev->bt_net = NULL;

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
