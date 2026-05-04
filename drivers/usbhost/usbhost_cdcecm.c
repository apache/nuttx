/****************************************************************************
 * drivers/usbhost/usbhost_cdcecm.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <fcntl.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default max segment size if not specified by device */

#define CDCECM_DEFAULT_MAXSEG   1514

/* ECM control requests */

#define ECM_SET_PACKET_FILTER_REQUEST 0x43

/* ECM packet filter bits */

#define ECM_PACKET_TYPE_PROMISCUOUS    (1 << 0)
#define ECM_PACKET_TYPE_ALL_MULTICAST  (1 << 1)
#define ECM_PACKET_TYPE_DIRECTED       (1 << 2)
#define ECM_PACKET_TYPE_BROADCAST      (1 << 3)
#define ECM_PACKET_TYPE_MULTICAST      (1 << 4)

/* Default packet filter: unicast + broadcast */

#define ECM_DEFAULT_PACKET_FILTER      (ECM_PACKET_TYPE_DIRECTED | \
                                        ECM_PACKET_TYPE_BROADCAST)

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_USBHOST_ASYNCH
#  warning Asynchronous transfer support is required (CONFIG_USBHOST_ASYNCH)
#endif

#ifdef CONFIG_USBHOST_CDCECM_NTDELAY
#  define USBHOST_CDCECM_NTDELAY MSEC2TICK(CONFIG_USBHOST_CDCECM_NTDELAY)
#else
#  define USBHOST_CDCECM_NTDELAY MSEC2TICK(200)
#endif

#ifndef CONFIG_USBHOST_CDCECM_RXBUFSIZE
  #define CONFIG_USBHOST_CDCECM_RXBUFSIZE 2048
#endif

#ifndef CONFIG_USBHOST_CDCECM_TXBUFSIZE
  #define CONFIG_USBHOST_CDCECM_TXBUFSIZE 2048
#endif

/* Used in usbhost_cfgdesc() */

#define USBHOST_CTRLIFFOUND 0x01
#define USBHOST_DATAIFFOUND 0x02
#define USBHOST_INTRIFFOUND 0x04
#define USBHOST_BINFOUND    0x08
#define USBHOST_BOUTFOUND   0x10
#define USBHOST_ECMFOUND    0x20
#define USBHOST_ALLFOUND    0x3f

#define USBHOST_MAX_CREFS   0x7fff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usb_csifdesc_s
{
  uint8_t len;
  uint8_t type;
  uint8_t subtype;
};

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct usbhost_cdcecm_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;

  /* The remainder of the fields are provided to the class driver */

  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint16_t                ctrlif;       /* Control interface number */
  uint16_t                dataif;       /* Data interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  mutex_t                 lock;         /* Used to maintain mutual exclusive access */
  struct work_s           ntwork;       /* Notification work */
  struct work_s           bulk_rxwork;
  struct work_s           destroywork;
  int16_t                 nnbytes;      /* Number of bytes received in notification */
  int16_t                 bulkinbytes;
  uint16_t                maxintsize;   /* Maximum size of interrupt IN packet */
  FAR uint8_t            *ctrlreq;      /* Allocated ctrl request structure */
  FAR uint8_t            *notification; /* Allocated RX buffer for async notifications */
  FAR uint8_t            *bulkinbuf;    /* Allocated RX buffer for bulk IN */
  FAR uint8_t            *bulkoutbuf;   /* Allocated TX buffer for bulk OUT */
  usbhost_ep_t            intin;        /* Interrupt endpoint */
  usbhost_ep_t            bulkin;       /* Bulk IN endpoint */
  usbhost_ep_t            bulkout;      /* Bulk OUT endpoint */
  uint16_t                bulkmxpacket; /* Max packet size for Bulk OUT endpoint */

  /* Device info from ECM descriptor */

  uint16_t                maxsegment;    /* Max Ethernet frame size */
  uint8_t                 macstridx;     /* MAC address string index */
  uint8_t                 macaddr[6];    /* Device MAC address */

  /* Network device lowerhalf members */

  netpkt_queue_t          rx_queue;     /* RX packet queue for lowerhalf */
  bool                    bifup;        /* true:ifup false:ifdown */
  struct netdev_lowerhalf_s dev;        /* lower half network interface */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Memory allocation services */

static inline FAR struct usbhost_cdcecm_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_cdcecm_s *usbclass);

/* Worker thread actions */

static void usbhost_notification_work(FAR void *arg);
static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes);
static void usbhost_bulkin_work(FAR void *arg);

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static int usbhost_cfgdesc(FAR struct usbhost_cdcecm_s *priv,
                           FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_devinit(FAR struct usbhost_cdcecm_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);

/* Buffer memory management */

static int usbhost_alloc_buffers(FAR struct usbhost_cdcecm_s *priv);
static void usbhost_free_buffers(FAR struct usbhost_cdcecm_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s
              *usbhost_create(FAR struct usbhost_hubport_s *hport,
                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Lowerhalf network device operations */

static int cdcecm_net_ifup(FAR struct netdev_lowerhalf_s *dev);
static int cdcecm_net_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int cdcecm_net_transmit(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt);
static FAR netpkt_t *cdcecm_net_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int cdcecm_net_ioctl(FAR struct netdev_lowerhalf_s *dev,
                            int cmd, unsigned long arg);
#endif
static void cdcecm_net_reclaim(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

static const struct usbhost_id_s g_cdcecm_id[] =
{
  {
    USB_CLASS_CDC,          /* base - CDC class */
    CDC_SUBCLASS_NONE,      /* subclass */
    CDC_PROTO_NONE,         /* proto    */
    0,                      /* vid      */
    0                       /* pid      */
  },
  {
    USB_CLASS_CDC,          /* base - CDC class */
    CDC_SUBCLASS_ECM,       /* subclass - Ethernet Control Model */
    CDC_PROTO_NONE,         /* proto - No class specific protocol */
    0,                      /* vid - Any vendor */
    0                       /* pid - Any product */
  }
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_cdcecm =
{
  NULL,                           /* flink */
  usbhost_create,                 /* create */
  sizeof(g_cdcecm_id) /
    sizeof(g_cdcecm_id[0]),       /* nids */
  g_cdcecm_id                     /* id[] */
};

/* Netdev operations */

static const struct netdev_ops_s g_cdcecm_netdev_ops =
{
    .ifup     = cdcecm_net_ifup,
    .ifdown   = cdcecm_net_ifdown,
    .transmit = cdcecm_net_transmit,
    .receive  = cdcecm_net_receive,
#ifdef CONFIG_NETDEV_IOCTL
    .ioctl    = cdcecm_net_ioctl,
#endif
    .reclaim  = cdcecm_net_reclaim,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(USBHOST_CDCECM_PACKET_PRINT)
#  define cdcecm_print_packet(m,b,n) uinfodumpbuffer(m,b,n)
#else
#  define cdcecm_print_packet(m,b,n) {}
#endif

static int usbhost_ctrl_cmd(FAR struct usbhost_cdcecm_s *priv,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t index, uint8_t *payload, uint16_t len)
{
  FAR struct usbhost_hubport_s *hport;
  struct usb_ctrlreq_s *ctrlreq;
  int ret;

  hport = priv->usbclass.hport;

  ctrlreq       = (struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = type;
  ctrlreq->req  = req;

  usbhost_putle16(ctrlreq->value, value);
  usbhost_putle16(ctrlreq->index, index);
  usbhost_putle16(ctrlreq->len,   len);

  if (type & USB_REQ_DIR_IN)
    {
      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, payload);
    }
  else
    {
      ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, payload);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_cdcecm_s *usbhost_allocclass(void)
{
  FAR struct usbhost_cdcecm_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_cdcecm_s *)kmm_malloc(
                                         sizeof(struct usbhost_cdcecm_s));
  uinfo("Allocated: %p\n", priv);
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   usbclass - A reference to the class instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_cdcecm_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (perhaps calling sched_kmm_free() in case we are
   * executing from an interrupt handler.
   */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

static void usbhost_bulkin_callback(FAR void *arg, ssize_t nbytes)
{
  struct usbhost_cdcecm_s *priv = (struct usbhost_cdcecm_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  /* We can't lock the mutex from an interrupt context */

  if (priv->disconnected)
    {
      return;
    }

  priv->bulkinbytes = (int16_t)nbytes;

  if (nbytes < 0)
    {
      if (nbytes != -EAGAIN)
        {
          uerr("ERROR: Transfer failed: %d\n", nbytes);
        }

      delay = MSEC2TICK(30);
    }

  if (work_available(&priv->bulk_rxwork))
    {
      work_queue(LPWORK, &priv->bulk_rxwork,
                 usbhost_bulkin_work, priv, delay);
    }
}

static void usbhost_bulkin_work(FAR void *arg)
{
  struct usbhost_cdcecm_s *priv;
  struct usbhost_hubport_s *hport;
  FAR netpkt_t *pkt = NULL;

  priv = (struct usbhost_cdcecm_s *)arg;
  DEBUGASSERT(priv);

  nxmutex_lock(&priv->lock);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  if (priv->disconnected || !priv->bifup)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  /* Process received data into netpkt and queue for upper half */

  if (priv->bulkinbytes > 0)
    {
      uinfo("received packet: %d len\n", priv->bulkinbytes);
      cdcecm_print_packet("ECM RX Packet:",
                          priv->bulkinbuf, priv->bulkinbytes);

      /* ECM sends raw Ethernet frames - no additional framing */

      pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
      if (pkt != NULL)
        {
          int ret = netpkt_copyin(&priv->dev, pkt,
                                  priv->bulkinbuf,
                                  priv->bulkinbytes, 0);
          if (ret < 0)
            {
              netpkt_free(&priv->dev, pkt, NETPKT_RX);
              pkt = NULL;
            }
          else
            {
              ret = netpkt_tryadd_queue(pkt, &priv->rx_queue);
              if (ret != 0)
                {
                  netpkt_free(&priv->dev, pkt, NETPKT_RX);
                  pkt = NULL;
                }
            }
        }
    }
  else if (priv->bulkinbytes < 0 && priv->bulkinbytes != -EAGAIN)
    {
      uerr("Bulk IN error: %d\n", priv->bulkinbytes);
    }

  /* Restart async RX */

  DRVR_ASYNCH(hport->drvr, priv->bulkin,
              (uint8_t *)priv->bulkinbuf, CONFIG_USBHOST_CDCECM_RXBUFSIZE,
              usbhost_bulkin_callback, priv);

  nxmutex_unlock(&priv->lock);

  /* Notify upper half after unlocking to avoid deadlock */

  if (pkt != NULL)
    {
      netdev_lower_rxready(&priv->dev);
    }
}

/****************************************************************************
 * Name: usbhost_notification_work
 *
 * Description:
 *   Handle receipt of an asynchronous notification from the CDC device
 *
 * Input Parameters:
 *   arg - The argument provided with the asynchronous I/O was setup
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_notification_work(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct cdc_notification_s *inmsg;
  int ret;

  priv = (FAR struct usbhost_cdcecm_s *)arg;
  DEBUGASSERT(priv);

  nxmutex_lock(&priv->lock);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  /* Are we still connected? */

  if (!priv->disconnected && priv->intin)
    {
      /* Yes.. Was an interrupt IN message received correctly? */

      if (priv->nnbytes >= 0)
        {
          /* Yes.. decode the notification */

          inmsg = (FAR struct cdc_notification_s *)priv->notification;

          if ((inmsg->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
            {
              switch (inmsg->notification)
              {
                case ECM_NETWORK_CONNECTION:

                  /* Link state changed. This is supposed to be sent only
                   * on-change, but I found some devices send it repeatedly.
                   * So only update on-change to avoid a spam of prints
                   */

                  if (inmsg->value[0] &&
                      !IFF_IS_RUNNING(priv->dev.netdev.d_flags))
                  {
                    uinfo("Network connected\n");
                    netdev_lower_carrier_on(&priv->dev);
                  }
                  else if (!inmsg->value[0] &&
                           IFF_IS_RUNNING(priv->dev.netdev.d_flags))
                  {
                    uinfo("Network disconnected\n");
                    netdev_lower_carrier_off(&priv->dev);
                  }
                  break;

                case ECM_SPEED_CHANGE:

                  /* Connection speed changed - informational only.
                   * Disabled print since some adapters send this very
                   * frequently even if there is no change
                   */

                  /* uinfo("Speed change notification\n"); */

                  break;

                default:
                  uinfo("Unknown notification: 0x%02x\n",
                        inmsg->notification);
                  break;
              }
            }
        }

      /* Setup to receive the next notification */

      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        priv->maxintsize,
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: usbhost_notification_callback
 *
 * Description:
 *   Handle receipt of Response Available from the CDC/ECM device
 *
 * Input Parameters:
 *   arg - The argument provided with the asynchronous I/O was setup
 *   nbytes - The number of bytes actually transferred (or a negated errno
 *     value;
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  /* We can't lock the mutex from an interrupt context */

  /* Are we still connected? */

  if (!priv->disconnected)
    {
      /* Check for a failure.  On higher end host controllers, the
       * asynchronous transfer will pend until data is available (OHCI and
       * EHCI).  On lower end host controllers (like STM32 and EFM32), the
       * transfer will fail immediately when the device NAKs the first
       * attempted interrupt IN transfer (with nbytes == -EAGAIN).  In that
       * case (or in the case of other errors), we must fall back to
       * polling.
       */

      DEBUGASSERT(nbytes >= INT16_MIN && nbytes <= INT16_MAX);
      priv->nnbytes = (int16_t)nbytes;

      if (nbytes < 0)
        {
          /* This debug output is good to know, but really a nuisance for
           * those configurations where we have to fall back to polling.
           * FIX:  Don't output the message is the result is -EAGAIN.
           */

#if defined(CONFIG_DEBUG_USB) && !defined(CONFIG_DEBUG_INFO)
          if (nbytes != -EAGAIN)
#endif
            {
              uerr("ERROR: Transfer failed: %d\n", nbytes);
            }

          /* We don't know the nature of the failure, but we need to do all
           * that we can do to avoid a CPU hog error loop.
           *
           * Use the low-priority work queue and delay polling for the next
           * event.  We want to use as little CPU bandwidth as possible in
           * this case.
           */

          delay = USBHOST_CDCECM_NTDELAY;
        }

      /* Make sure that the work structure available.  There is a remote
       * chance that this may collide with a device disconnection event.
       */

      if (work_available(&priv->ntwork))
        {
          work_queue(LPWORK, &priv->ntwork,
                     usbhost_notification_work,
                     priv, delay);
        }
    }
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_driver_s *drvr;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  DEBUGASSERT(hport->drvr);
  drvr = hport->drvr;

  uinfo("crefs: %d\n", priv->crefs);

  /* Unregister the network device */

  netdev_lower_unregister(&priv->dev);

  /* Cancel any pending work */

  work_cancel(LPWORK, &priv->ntwork);

  work_cancel(LPWORK, &priv->bulk_rxwork);

  /* Free the endpoints */

  if (priv->intin)
    {
      DRVR_EPFREE(hport->drvr, priv->intin);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkin);
    }

  if (priv->bulkout)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkout);
    }

  /* Free any transfer buffers */

  usbhost_free_buffers(priv);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(drvr, hport);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_cfgdesc(FAR struct usbhost_cdcecm_s *priv,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  FAR struct usbhost_epdesc_s iindesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
   */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        case USB_DESC_TYPE_CSINTERFACE:
          {
            FAR struct usb_csifdesc_s *csdesc = (FAR struct usb_csifdesc_s *)
                                                desc;

            if (csdesc->subtype == CDC_DSUBTYPE_ECM &&
                    desc->len >= sizeof(struct cdc_ecm_funcdesc_s))
            {
              /* ECM functional descriptor */

              FAR struct cdc_ecm_funcdesc_s *ecmdesc =
                (FAR struct cdc_ecm_funcdesc_s *)desc;
              priv->macstridx = ecmdesc->mac;
              priv->maxsegment = usbhost_getle16(ecmdesc->maxseg);

              uinfo("ECM: mac_idx=%d maxseg=%d\n",
                    priv->macstridx, priv->maxsegment);
              found |= USBHOST_ECMFOUND;
            }
          }
          break;

        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)
                                              configdesc;

            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Is this the control interface? */

            if (ifdesc->classid  == CDC_CLASS_COMM &&
                ifdesc->subclass == CDC_SUBCLASS_ECM &&
                ifdesc->protocol == CDC_PROTO_NONE)
              {
                priv->ctrlif  = ifdesc->ifno;
                found        |= USBHOST_CTRLIFFOUND;
              }

            /* Is this the data interface? */

            else if (ifdesc->classid  == USB_CLASS_CDC_DATA &&
                     ifdesc->subclass == CDC_SUBCLASS_NONE &&
                     ifdesc->protocol == CDC_DATA_PROTO_NONE)
              {
                priv->dataif  = ifdesc->ifno;
                found        |= USBHOST_DATAIFFOUND;
              }
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)
                                              configdesc;

            uinfo("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for interrupt endpoint */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                if (USB_ISEPIN(epdesc->addr))
                  {
                    found |= USBHOST_INTRIFFOUND;
                    iindesc.hport        = hport;
                    iindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    iindesc.in           = true;
                    iindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    iindesc.interval     = epdesc->interval;
                    iindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          iindesc.addr, iindesc.mxpacketsize);

                    priv->maxintsize = iindesc.mxpacketsize;
                  }
              }

            /* Check for a bulk endpoint. */

            else if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                     USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.hport        = hport;
                    boutdesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);

                    priv->bulkmxpacket = boutdesc.mxpacketsize;
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */

                    bindesc.hport        = hport;
                    bindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if (found != USBHOST_ALLFOUND)
    {
      uerr("ERROR: Found CTRLIF:%s DATAIF: %s INTR: %s "
           "BIN:%s BOUT:%s ECM: %s\n",
           (found & USBHOST_CTRLIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_DATAIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_INTRIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_ECMFOUND) != 0 ? "YES" : "NO");

      return -EINVAL;
    }

  /* Set default max segment size if not provided */

  if (priv->maxsegment == 0)
  {
    priv->maxsegment = CDCECM_DEFAULT_MAXSEG;
  }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(hport->drvr, &boutdesc, &priv->bulkout);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &bindesc, &priv->bulkin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &iindesc, &priv->intin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Interrupt IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      (void)DRVR_EPFREE(hport->drvr, priv->bulkin);
      return ret;
    }

  uinfo("Endpoints allocated\n");
  return OK;
}

static int usbhost_setinterface(FAR struct usbhost_cdcecm_s *priv,
                                uint16_t iface, uint16_t setting)
{
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD |
                          USB_REQ_RECIPIENT_INTERFACE,
                          USB_REQ_SETINTERFACE, setting, iface, NULL, 0);
}

/****************************************************************************
 * Name: usbhost_parse_mac_string
 *
 * Description:
 *   Parse MAC address from USB string descriptor.
 *   The string is a Unicode representation of the MAC address as hex digits
 *   (e.g., "001122334455" -> 0x00, 0x11, 0x22, 0x33, 0x44, 0x55).
 *
 ****************************************************************************/

static int usbhost_parse_mac_string(FAR const uint8_t * strdesc,
                                    FAR uint8_t * macaddr)
{
  FAR const uint8_t * unicode;
  int len;
  int i;
  uint8_t byte;
  uint8_t nibble;

  /* String descriptor format:
   * byte 0: length
   * byte 1: descriptor type (3)
   * bytes 2+: Unicode string (16-bit chars)
   */

  if (strdesc[0] < 26 || strdesc[1] != USB_DESC_TYPE_STRING)
    {
      uerr("Invalid MAC string descriptor\n");
      return -EINVAL;
    }

  len = (strdesc[0] - 2) / 2;  /* Number of Unicode characters */
  if (len < 12)
    {
      uerr("MAC string too short: %d chars\n", len);
      return -EINVAL;
    }

  unicode = &strdesc[2];

  for (i = 0; i < 6; i++)
    {
      /* Parse high nibble */

      nibble = unicode[i * 4];  /* Unicode chars are 2 bytes each */
      if (nibble >= '0' && nibble <= '9')
        {
          byte = (nibble - '0') << 4;
        }
      else if (nibble >= 'A' && nibble <= 'F')
        {
          byte = (nibble - 'A' + 10) << 4;
        }
      else if (nibble >= 'a' && nibble <= 'f')
        {
          byte = (nibble - 'a' + 10) << 4;
        }
      else
        {
          return -EINVAL;
        }

      /* Parse low nibble */

      nibble = unicode[i * 4 + 2];
      if (nibble >= '0' && nibble <= '9')
        {
          byte |= (nibble - '0');
        }
      else if (nibble >= 'A' && nibble <= 'F')
        {
          byte |= (nibble - 'A' + 10);
        }
      else if (nibble >= 'a' && nibble <= 'f')
        {
          byte |= (nibble - 'a' + 10);
        }
      else
        {
          return -EINVAL;
        }

      macaddr[i] = byte;
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_get_mac_address
 *
 * Description:
 *   Retrieve MAC address from the device using the string descriptor index
 *   from the ECM functional descriptor.
 *
 ****************************************************************************/

static int usbhost_get_mac_address(FAR struct usbhost_cdcecm_s * priv)
{
  int ret;
  uint8_t *strbuf;

  if (priv->macstridx == 0)
    {
      uwarn("No MAC address string index provided\n");
      return -ENOENT;
    }

  ret = DRVR_IOALLOC(priv->usbclass.hport->drvr, &strbuf, 64);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of notification buf failed: %d (%d bytes)\n",
           ret, priv->maxintsize);
      return ret;
    }

  ret = usbhost_ctrl_cmd(priv,
                         USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD |
                         USB_REQ_RECIPIENT_DEVICE,
                         USB_REQ_GETDESCRIPTOR,
                         (USB_DESC_TYPE_STRING << 8) | priv->macstridx,
                         0x0409, strbuf, 64);
  if (ret < 0)
    {
      uerr("Failed to get MAC string descriptor: %d\n", ret);
      goto errout;
    }

  /* Parse MAC address from Unicode string */

  ret = usbhost_parse_mac_string(strbuf, priv->macaddr);
  if (ret < 0)
    {
      uerr("Failed to parse MAC address: %d\n", ret);
      goto errout;
    }

  uinfo("Device MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        priv->macaddr[0], priv->macaddr[1], priv->macaddr[2],
        priv->macaddr[3], priv->macaddr[4], priv->macaddr[5]);

  ret = OK;

errout:
  (void)DRVR_IOFREE(priv->usbclass.hport->drvr, strbuf);
  return ret;
}

/****************************************************************************
 * Name: usbhost_set_packet_filter
 *
 * Description:
 *   Set the ECM packet filter on the device.
 *
 ****************************************************************************/

static int usbhost_set_packet_filter(FAR struct usbhost_cdcecm_s * priv,
                                     uint16_t filter)
{
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                          USB_REQ_RECIPIENT_INTERFACE,
                          ECM_SET_PACKET_FILTER_REQUEST,
                          filter, priv->ctrlif, NULL, 0);
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  int ret = OK;

  hport = priv->usbclass.hport;

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device */

  /* Set aside transfer buffers for exclusive use by the class driver */

  ret = usbhost_alloc_buffers(priv);
  if (ret)
    {
      uerr("ERROR: failed to allocate buffers\n");
      return ret;
    }

  /* Get MAC address from device */

  ret = usbhost_get_mac_address(priv);
  if (ret < 0)
    {
      /* Generate a random MAC if we couldn't get one */

      uwarn("Using random MAC address\n");
      priv->macaddr[0] = 0x02;  /* Locally administered */
      priv->macaddr[1] = rand() & 0xff;
      priv->macaddr[2] = rand() & 0xff;
      priv->macaddr[3] = rand() & 0xff;
      priv->macaddr[4] = rand() & 0xff;
      priv->macaddr[5] = rand() & 0xff;
    }

  /* Setup and register the lowerhalf network device */

  FAR struct netdev_lowerhalf_s *dev = &priv->dev;

  memset(dev, 0, sizeof(*dev));
  memcpy(dev->netdev.d_mac.ether.ether_addr_octet,
         priv->macaddr, sizeof(priv->macaddr));

  dev->ops = &g_cdcecm_netdev_ops;
  dev->quota[NETPKT_TX] = 1;
  dev->quota[NETPKT_RX] = 1;
  dev->rxtype           = NETDEV_RX_WORK;
  dev->priority         = LPWORK;

  netdev_lower_register(dev, NET_LL_ETHERNET);

  if (priv->intin)
    {
      /* Begin monitoring of message available events */

      uinfo("Start notification monitoring\n");
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        priv->maxintsize,
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed on intin: %d\n", ret);
        }
    }

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the character
   * driver has been registered.
   */

  if (ret >= 0)
    {
      nxmutex_lock(&priv->lock);
      DEBUGASSERT(priv->crefs >= 2);

      /* Handle a corner case where (1) open() has been called so the
       * reference count is > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 2 && priv->disconnected)
        {
          /* We don't have to give the semaphore because it will be
           * destroyed when usb_destroy is called.
           */

          ret = -ENODEV;
        }
      else
        {
          /* Ready for normal operation */

          uinfo("Successfully initialized\n");
          priv->crefs--;
        }

      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 |
                                   (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_alloc_buffers
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int usbhost_alloc_buffers(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->ctrlreq == NULL);
  hport = priv->usbclass.hport;

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of ctrlreq failed: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(maxlen >= sizeof(struct usb_ctrlreq_s));

  /* Allocate buffer for interrupt IN notifications */

  ret = DRVR_IOALLOC(hport->drvr, &priv->notification, priv->maxintsize);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of notification buf failed: %d (%d bytes)\n",
           ret, priv->maxintsize);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->bulkinbuf,
                     CONFIG_USBHOST_CDCECM_RXBUFSIZE);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of net rx buf failed: %d (%d bytes)\n",
           ret, CONFIG_USBHOST_CDCECM_RXBUFSIZE);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->bulkoutbuf,
                     CONFIG_USBHOST_CDCECM_TXBUFSIZE);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_IOALLOC of net tx buf failed: %d (%d bytes)\n",
           ret, CONFIG_USBHOST_CDCECM_TXBUFSIZE);
      goto errout;
    }

  return OK;

errout:
  usbhost_free_buffers(priv);
  return ret;
}

/****************************************************************************
 * Name: usbhost_free_buffers
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_free_buffers(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  if (priv->ctrlreq)
    {
      (void)DRVR_FREE(hport->drvr, priv->ctrlreq);
    }

  if (priv->notification)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->notification);
    }

  if (priv->bulkinbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->bulkinbuf);
    }

  if (priv->bulkoutbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->bulkoutbuf);
    }

  priv->ctrlreq      = NULL;
  priv->notification = NULL;
  priv->bulkinbuf    = NULL;
  priv->bulkoutbuf   = NULL;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub hat manages the new class instance.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hport input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s
                  *usbhost_create(FAR struct usbhost_hubport_s *hport,
                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_cdcecm_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_cdcecm_s));

      /* Initialize class method function pointers */

      priv->usbclass.hport        = hport;
      priv->usbclass.connect      = usbhost_connect;
      priv->usbclass.disconnected = usbhost_disconnected;

      /* The initial reference count is 1... One reference is held by
       * the driver.
       */

      priv->crefs = 1;

      /* Initialize mutex (this works in the interrupt context) */

      nxmutex_init(&priv->lock);

      /* Return the instance of the USB class driver */

      return &priv->usbclass;
    }

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.
 *   It is the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)
                                       usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret < 0)
        {
          uerr("ERROR: usbhost_devinit() failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)
                                       usbclass;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = enter_critical_section();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  uinfo("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uinfo("Queuing destruction: worker %p->%p\n",
                priv->destroywork.worker, usbhost_destroy);
          DEBUGASSERT(priv->destroywork.worker == NULL);
          work_queue(LPWORK, &priv->destroywork,
                           usbhost_destroy, priv, 0);
        }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcecm_net_transmit
 *
 * Description:
 *   Lowerhalf transmit callback. Copies packet data to USB bulk OUT buffer
 *   and performs a blocking USB transfer.
 *
 * Input Parameters:
 *   dev - Reference to the lowerhalf driver structure
 *   pkt - The packet to send (driver takes ownership and must free)
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int cdcecm_net_transmit(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt)
{
  FAR struct usbhost_cdcecm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  unsigned int datalen;
  ssize_t ret;

  priv = container_of(dev, struct usbhost_cdcecm_s, dev);
  hport = priv->usbclass.hport;

  datalen = netpkt_getdatalen(dev, pkt);
  uinfo("transmit packet: %u bytes\n", datalen);

  if (datalen > CONFIG_USBHOST_CDCECM_TXBUFSIZE)
    {
      netpkt_free(dev, pkt, NETPKT_TX);
      return -EMSGSIZE;
    }

  /* Copy packet data to USB bulk out buffer.
   * ECM sends raw Ethernet frames - no additional framing needed.
   */

  ret = netpkt_copyout(dev, priv->bulkoutbuf, pkt, datalen, 0);
  netpkt_free(dev, pkt, NETPKT_TX);

  if (ret < 0)
    {
      return ret;
    }

  cdcecm_print_packet("ECM TX Packet:", priv->bulkoutbuf, datalen);

  ret = DRVR_TRANSFER(hport->drvr, priv->bulkout,
                      priv->bulkoutbuf, datalen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_TRANSFER failed: %zd\n", ret);
      return ret;
    }

  netdev_lower_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: cdcecm_net_receive
 *
 * Description:
 *   Lowerhalf receive callback. Dequeues a received packet from the RX
 *   queue. Called by upper half after netdev_lower_rxready() notification.
 *
 * Input Parameters:
 *   dev - Reference to the lowerhalf driver structure
 *
 * Returned Value:
 *   A netpkt containing the received packet, or NULL if no more packets.
 *
 ****************************************************************************/

static FAR netpkt_t *cdcecm_net_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv;

  priv = container_of(dev, struct usbhost_cdcecm_s, dev);
  return netpkt_remove_queue(&priv->rx_queue);
}

/****************************************************************************
 * Name: cdcecm_net_ioctl
 *
 * Description:
 *   Lowerhalf ioctl callback.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int cdcecm_net_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: cdcecm_net_reclaim
 *
 * Description:
 *   Lowerhalf reclaim callback. TX is synchronous via DRVR_TRANSFER
 *   so there are no queued TX packets to reclaim.
 *
 ****************************************************************************/

static void cdcecm_net_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
}

/****************************************************************************
 * Name: cdcecm_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the ECM interface
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

static int cdcecm_net_ifup(struct netdev_lowerhalf_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv;

  priv = container_of(dev, struct usbhost_cdcecm_s, dev);
  nxmutex_lock(&priv->lock);

  struct usbhost_hubport_s *hport = priv->usbclass.hport;
  int ret;

  /* Activate the data interface by setting alternate setting 1 */

  ret = usbhost_setinterface(priv, priv->dataif, 1);
  if (ret < 0)
    {
      uerr("Failed to activate data interface: %d\n", ret);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Set the packet filter */

  ret = usbhost_set_packet_filter(priv, ECM_DEFAULT_PACKET_FILTER);
  if (ret < 0)
    {
      uwarn("Failed to set packet filter: %d (continuing)\n", ret);

      /* Continue anyway - some devices don't support this */
    }

  /* Start RX asynch on bulk in */

  if (priv->bulkin)
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->bulkin,
                        priv->bulkinbuf, CONFIG_USBHOST_CDCECM_RXBUFSIZE,
                        usbhost_bulkin_callback, priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed on bulkin: %d\n", ret);
        }
    }

  priv->bifup = true;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: cdcecm_ifdown
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

static int cdcecm_net_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct usbhost_cdcecm_s *priv;

  priv = container_of(dev, struct usbhost_cdcecm_s, dev);
  nxmutex_lock(&priv->lock);

  /* Deactivate the data interface */

  usbhost_setinterface(priv, priv->dataif, 0);

  priv->bifup = false;

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_cdcecm_initialize
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_cdcecm_initialize(void)
{
  /* Perform any one-time initialization of the class implementation */

  /* Advertise our availability to support CDC ECM devices */

  return usbhost_registerclass(&g_cdcecm);
}
