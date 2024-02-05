/****************************************************************************
 * drivers/usbhost/usbhost_bthci.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include <nuttx/net/bluetooth.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/bthci[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/bthci%c"
#define DEV_NAMELEN         12

/* Used in usbhci_cfgdesc() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_IINFOUND    0x08
#define USBHOST_ALLFOUND    0x0F

#define USBHOST_MAX_CREFS   0x7fff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;
  struct bt_driver_s      btdev;

  /* The remainder of the fields are provide to the class driver */

  char                    devchar;      /* Character identifying the /dev/bthci[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint8_t                 ifno;         /* Interface number */
  mutex_t                 lock;         /* Used to maintain mutual exclusive access */
  struct work_s           work;         /* For interacting with the worker thread */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t            bulkin;       /* Bulk IN endpoint */
  usbhost_ep_t            bulkout;      /* Bulk OUT endpoint */
  usbhost_ep_t            intin;        /* Interrupt endpoint */
  FAR uint8_t            *ctrlreq;      /* Allocated ctrl request structure */
  FAR uint8_t            *evbuffer;     /* Allocated event buffer */
  size_t                  evbuflen;     /* Size of the allocated event buffer */
  struct work_s           acwork;       /* For asynchronous event work */
  struct work_s           evwork;       /* For asynchronous event work */
  int16_t                 acbytes;      /* The number of bytes actually transferred */
  int16_t                 evbytes;      /* The number of bytes actually transferred */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhci_connect() */

static inline int usbhci_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc,
                                  int desclen);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);

/* Transfer descriptor memory management */

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv);
static inline void usbhost_tfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *
  usbhost_create(FAR struct usbhost_hubport_s *hport,
                 FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhci_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Driver methods --
 * depend upon the type of NuttX driver interface exported
 */

static int usbhost_ctrl_cmd(FAR struct usbhost_state_s *priv,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t iface, uint8_t *payload, uint16_t len);
static void usbhost_event_work(FAR void *arg);
static void usbhost_event_callback(FAR void *arg, ssize_t nbytes);
static void usbhost_acl_callback(FAR void *arg, ssize_t nbytes);

static ssize_t usbhost_cmd_tx(FAR struct usbhost_state_s *priv,
                      FAR const void *buffer, size_t buflen);
static ssize_t usbhost_acl_tx(FAR struct usbhost_state_s *priv,
                      FAR const void *buffer, size_t buflen);

static int usbhost_bthci_send(FAR struct bt_driver_s *dev,
                enum bt_buf_type_e type,
                FAR void *data, size_t len);
static int usbhost_bthci_open(FAR struct bt_driver_s *dev);
static void usbhost_bthci_close(FAR struct bt_driver_s *dev);
static int usbhost_bthci_ioctl(FAR struct bt_driver_s *dev,
                 int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

static const struct usbhost_id_s g_id[] =
{
  {
    USB_CLASS_WIRELESS_CONTROLLER,  /* base     */
    0x01,                           /* subclass */
    0x01,                           /* proto    */
    0,                              /* vid      */
    0                               /* pid      */
  },
};

/* This is the USB host bthci class's registry entry */

static struct usbhost_registry_s g_bthci =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  1,                      /* nids     */
  &g_id[0]                /* id[]     */
};

/* This is a bitmap that is used to allocate device names /dev/bthcia-z. */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());

  priv = (FAR struct usbhost_state_s *)
    kmm_malloc(sizeof(struct usbhost_state_s));

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

static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (perhaps calling sched_kmm_free() in case we are
   * executing from an interrupt handler.
   */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  if (priv->devchar >= 'a')
    {
      int devno = priv->devchar - 'a';

      if (devno >= 0 && devno < 26)
        {
          irqstate_t flags = enter_critical_section();
          g_devinuse &= ~(1 << devno);
          leave_critical_section(flags);
        }
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
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
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_driver_s *drvr;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  DEBUGASSERT(hport->drvr);
  drvr = hport->drvr;

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the endpoints */

  if (priv->bulkin)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkin);
    }

  if (priv->bulkout)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkout);
    }

  /* Cancel any pending asynchronous I/O */

  if (priv->intin)
    {
      DRVR_EPFREE(hport->drvr, priv->intin);
    }

  /* Free any transfer buffers */

  usbhost_tfree(priv);

  /* Destroy the semaphores */

  nxmutex_destroy(&priv->lock);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(drvr, hport);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhci_cfgdesc
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

static inline int usbhci_cfgdesc(FAR struct usbhost_state_s *priv,
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

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc =
              (FAR struct usb_ifdesc_s *)configdesc;

            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            if (ifdesc->ifno == 0)
              {
                priv->ifno = ifdesc->ifno;
                found |= USBHOST_IFFOUND;
              }
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                if (USB_ISEPIN(epdesc->addr))
                  {
                    iindesc.hport        = hport;
                    iindesc.addr         = epdesc->addr &
                                               USB_EP_ADDR_NUMBER_MASK;
                    iindesc.in           = true;
                    iindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    iindesc.interval     = epdesc->interval;
                    iindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);

                    if (iindesc.addr == 1)
                      {
                        found |= USBHOST_IINFOUND;
                      }
                  }
              }

            /* Check for a bulk endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
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

                    /* Save the bulk OUT endpoint information */

                    boutdesc.hport        = hport;
                    boutdesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    if (boutdesc.addr == 2)
                      {
                        found |= USBHOST_BOUTFOUND;
                      }
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

                    /* Save the bulk IN endpoint information */

                    bindesc.hport        = hport;
                    bindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    if (bindesc.addr == 2)
                      {
                        found |= USBHOST_BINFOUND;
                      }
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
      uerr("ERROR: Found IF:%s IIN:%s BIN:%s BOUT:%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_IINFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
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
      DRVR_EPFREE(hport->drvr, priv->bulkout);
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

/****************************************************************************
 * Name: usbhost_ctrl_cmd
 *
 * Description:
 *   Do a USB control transfer.
 *
 * Input Parameters:
 *   priv    - A reference to the USB host class instance.
 *   type    - Transfer flags.
 *   req     - control transfer type.
 *   value   - Value for control transfer.
 *   index   - Index for control transfer.
 *   payload - Data buffer.
 *   len     - Length of data buffer.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int usbhost_ctrl_cmd(FAR struct usbhost_state_s *priv,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t indx, uint8_t *payload, uint16_t len)
{
  FAR struct usbhost_hubport_s *hport;
  struct usb_ctrlreq_s *ctrlreq;
  int ret;

  hport = priv->usbclass.hport;

  ctrlreq       = (struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = type;
  ctrlreq->req  = req;

  usbhost_putle16(ctrlreq->value, value);
  usbhost_putle16(ctrlreq->index, indx);
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
 * Name: usbhost_event_callback
 *
 * Description:
 *   Handle event packet reception
 *
 * Input Parameters:
 *   arg    - The argument provided when the asynchronous I/O was setup
 *   nbytes - The number of bytes actually transferred (or a negated errno
 *            value).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_event_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_state_s *priv;

  priv = (FAR struct usbhost_state_s *)arg;
  DEBUGASSERT(priv);

  if (priv->intin)
    {
      priv->evbytes = (int16_t)nbytes;

      if (priv->evbytes >= 0 && work_available(&priv->evwork))
        {
          work_queue(LPWORK, &priv->evwork,
                     usbhost_event_work,
                     priv, 0);
        }
    }
}

/****************************************************************************
 * Name: usbhost_event_work
 *
 * Description:
 *   Receive bluetooth events.
 *
 * Input Parameters:
 *   arg - A reference to the USB host class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_event_work(FAR void *arg)
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhost_hubport_s *hport;
  int ret;

  priv = (FAR struct usbhost_state_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  if (priv->evbytes > 0)
    {
      bt_netdev_receive(&priv->btdev, BT_EVT, priv->evbuffer,
                        priv->evbytes);
      priv->evbytes = 0;
    }

  if (priv->intin)
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->evbuffer,
                        priv->evbuflen, usbhost_event_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: usbhost_cmd_tx
 *
 * Description:
 *   Send a bluetooth HCI command.
 *
 * Input Parameters:
 *   priv   - A reference to the USB host class instance.
 *   buffer - The buffer to be sent by the driver.
 *   len    - The length of the buffer.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static ssize_t usbhost_cmd_tx(FAR struct usbhost_state_s *priv,
                             FAR const void *buffer, size_t buflen)
{
  int ret;

  nxmutex_lock(&priv->lock);

  ret = usbhost_ctrl_cmd(priv,
                         USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS |
                         USB_REQ_RECIPIENT_DEVICE,
                         0, 0, 0, (uint8_t *)buffer, buflen);

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: usbhost_acl_work
 *
 * Description:
 *   Receive bluetooth ACL packets.
 *
 * Input Parameters:
 *   arg - A reference to the USB host class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_acl_work(FAR void *arg)
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhost_hubport_s *hport;
  int ret;

  priv = (FAR struct usbhost_state_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  if (priv->acbytes > 0)
    {
      bt_netdev_receive(&priv->btdev, BT_ACL_IN, priv->tbuffer,
                        priv->acbytes);
      priv->acbytes = 0;
    }

  if (priv->bulkin)
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->bulkin,
                        (FAR uint8_t *)priv->tbuffer,
                        priv->tbuflen, usbhost_acl_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: usbhost_acl_callback
 *
 * Description:
 *   Handle ACL packet reception
 *
 * Input Parameters:
 *   arg    - The argument provided when the asynchronous I/O was setup
 *   nbytes - The number of bytes actually transferred (or a negated errno
 *            value).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_acl_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_state_s *priv;

  priv = (FAR struct usbhost_state_s *)arg;
  DEBUGASSERT(priv);

  if (priv->bulkin)
    {
      priv->acbytes = (int16_t)nbytes;

      if (priv->acbytes >= 0 && work_available(&priv->acwork))
        {
          work_queue(LPWORK, &priv->acwork, usbhost_acl_work, priv, 0);
        }
    }
}

/****************************************************************************
 * Name: usbhost_acl_tx
 *
 * Description:
 *   Send a bluetooth ACL packet.
 *
 * Input Parameters:
 *   priv   - A reference to the USB host class instance.
 *   buffer - The buffer to be sent by the driver.
 *   len    - The length of the buffer.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static ssize_t usbhost_acl_tx(FAR struct usbhost_state_s *priv,
                             FAR const void *buffer, size_t buflen)
{
  ssize_t nwritten = 0;
  FAR struct usbhost_hubport_s *hport;
  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  nxmutex_lock(&priv->lock);

  nwritten = DRVR_TRANSFER(hport->drvr, priv->bulkout,
                               (uint8_t *)buffer, buflen);

  if (nwritten < 0)
    {
      uerr("ERROR: DRVR_TRANSFER for ACL failed: %d\n", (int)nwritten);
    }
  else
    {
      nwritten = OK;
    }

  nxmutex_unlock(&priv->lock);

  return nwritten;
}

/****************************************************************************
 * Name: usbhost_bthci_send
 *
 * Description:
 *   Send the packet in the provided buffer.
 *
 * Input Parameters:
 *   btdev - An instance of the BT low-level driver's interface structure.
 *   type  - The type of packet in the buffer.
 *   data  - The buffer to be sent by the driver.
 *   len   - The length of the buffer.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int usbhost_bthci_send(FAR struct bt_driver_s *dev,
                enum bt_buf_type_e type,
                FAR void *data, size_t len)
{
  int ret;
  FAR struct usbhost_state_s *priv;
  priv = container_of(dev, struct usbhost_state_s, btdev);

  if (type == BT_CMD)
    {
      ret = usbhost_cmd_tx(priv, data, len);
    }
  else if (type == BT_ACL_OUT)
    {
      ret = usbhost_acl_tx(priv, data, len);
    }
  else
    {
      ret = -EINVAL;
      uerr("ERROR: unexpected packet type %x\n", type);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_bthci_open
 *
 * Description:
 *   Initialize the bluetooth hardware.
 *
 * Input Parameters:
 *   dev   - An instance of the BT low-level driver's interface structure.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int usbhost_bthci_open(FAR struct bt_driver_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: usbhost_bthci_close
 *
 * Description:
 *   Close the bluetooth hardware.
 *
 * Input Parameters:
 *   dev   - An instance of the BT low-level driver's interface structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_bthci_close(FAR struct bt_driver_s *dev)
{
}

/****************************************************************************
 * Name: usbhost_bthci_ioctl
 *
 * Description:
 *   Handle IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - An instance of the BT low-level driver's interface structure.
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int usbhost_bthci_ioctl(FAR struct bt_driver_s *dev,
                 int cmd, unsigned long arg)
{
  return -ENOTTY;
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

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  int ret = OK;

  if (priv->bulkin && work_available(&priv->acwork))
    {
      /* Begin monitoring of acl packets */

      work_queue(LPWORK, &priv->acwork, usbhost_acl_work, priv, 0);
    }

  if (priv->intin && work_available(&priv->evwork))
    {
      /* Begin monitoring of message available events */

      work_queue(LPWORK, &priv->evwork, usbhost_event_work, priv, 0);
    }

  if (ret >= 0 && priv->devchar == 'a')
    {
      /* Register the driver with the network stack. */

      ret = bt_netdev_register(&priv->btdev);
      if (ret < 0)
        {
          uerr("ERROR: bt_netdev_register failed: %d\n", ret);
        }
    }

  /* Check if we successfully initialized. */

  if (ret >= 0)
    {
      /* Ready for normal operation */

      uinfo("Successfully initialized\n");
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
  /* Little endian means LSB first in byte stream */

  dest[0] = val & 0xff;
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_talloc
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

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->tbuffer == NULL);
  hport = priv->usbclass.hport;

  /* Allocate buffer for events. */

  if (priv->intin)
    {
      ret = DRVR_ALLOC(hport->drvr, &priv->evbuffer, &priv->evbuflen);
      if (ret < 0)
        {
          uerr("ERROR: DRVR_IOALLOC of evbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of ctrlreq failed: %d\n", ret);

      usbhost_tfree(priv);
      return ret;
    }

  DEBUGASSERT(maxlen >= sizeof(struct usb_ctrlreq_s));

  ret = DRVR_ALLOC(hport->drvr, &priv->tbuffer, &priv->tbuflen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of buffer failed: %d\n", ret);
      usbhost_tfree(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline void usbhost_tfree(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  if (priv->evbuffer)
    {
      DRVR_FREE(hport->drvr, priv->evbuffer);
      priv->evbuffer = NULL;
      priv->evbuflen = 0;
    }

  if (priv->ctrlreq)
    {
      DRVR_FREE(hport->drvr, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  if (priv->tbuffer)
    {
      DRVR_FREE(hport->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.  The create() method is a callback into the class
 *   implementation.  It is used to (1) create a new instance of the USB
 *   host class state and to (2) bind a USB host driver "session" to the
 *   class instance.  Use of this create() method will support environments
 *   where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub port that manages the new class instance.
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

static FAR struct usbhost_class_s *
usbhost_create(FAR struct usbhost_hubport_s *hport,
               FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv == NULL)
    {
      return NULL;
    }

  /* Initialize the allocated storage class instance */

  memset(priv, 0, sizeof(struct usbhost_state_s));

  /* Assign a device number to this class instance */

  if (usbhost_allocdevno(priv) == OK)
    {
      /* Initialize class method function pointers */

      priv->usbclass.hport        = hport;
      priv->usbclass.connect      = usbhci_connect;
      priv->usbclass.disconnected = usbhost_disconnected;

      priv->btdev.open            = usbhost_bthci_open;
      priv->btdev.send            = usbhost_bthci_send;
      priv->btdev.close           = usbhost_bthci_close;
      priv->btdev.ioctl           = usbhost_bthci_ioctl;

      /* Initialize semaphores
       * (this works okay in the interrupt context)
       */

      nxmutex_init(&priv->lock);

      /* Return the instance of the USB class driver */

      return &priv->usbclass;
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  usbhost_freeclass(priv);

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhci_connect
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

static int usbhci_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhci_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhci_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      ret = usbhost_talloc(priv);
      if (ret < 0)
        {
          uerr("ERROR: Failed to allocate transfer buffer\n");
          return ret;
        }

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
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = enter_critical_section();

  priv->disconnected = true;

  if (priv->devchar == 'a')
    {
      bt_netdev_unregister(&priv->btdev);
    }

  /* Cancel any ongoing transfers */

  work_cancel(LPWORK, &priv->acwork);
  work_cancel(LPWORK, &priv->evwork);

  /* Destroy the class instance.  If we are executing from an interrupt
   * handler, then defer the destruction to the worker thread.
   * Otherwise, destroy the instance now.
   */

  if (up_interrupt_context())
    {
      /* Destroy the instance on the worker thread. */

      DEBUGASSERT(priv->work.worker == NULL);
      work_queue(LPWORK, &priv->work, usbhost_destroy, priv, 0);
    }
  else
    {
      /* Do the work now */

      usbhost_destroy(priv);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_bthci_initialize
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

int usbhost_bthci_initialize(void)
{
  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_bthci);
}
