/****************************************************************************
 * drivers/usbdev/usbmsc.c
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

/* Mass storage class device.  Bulk-only with SCSI subclass. */

/* References:
 *   "Universal Serial Bus Mass Storage Class, Specification Overview,"
 *   Revision 1.2,  USB Implementer's Forum, June 23, 2003.
 *
 *   "Universal Serial Bus Mass Storage Class, Bulk-Only Transport,"
 *   Revision 1.0, USB Implementer's Forum, September 31, 1999.
 *
 *   "SCSI Primary Commands - 3 (SPC-3),"  American National Standard
 *   for Information Technology, May 4, 2005
 *
 *   "SCSI Primary Commands - 4 (SPC-4),"  American National Standard
 *   for Information Technology, July 19, 2008
 *
 *   "SCSI Block Commands -2 (SBC-2)," American National Standard
 *   for Information Technology, November 13, 2004
 *
 *   "SCSI Multimedia Commands - 3 (MMC-3),"  American National Standard
 *   for Information Technology, November 12, 2001
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbmsc.h"

#ifdef CONFIG_USBMSC_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The internal version of the class driver */

struct usbmsc_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct usbmsc_dev_s    *dev;
};

/* This is what is allocated */

struct usbmsc_alloc_s
{
  struct usbmsc_dev_s    dev;
  struct usbmsc_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Class Driver Support *****************************************************/

static void   usbmsc_ep0incomplete(FAR struct usbdev_ep_s *ep,
                FAR struct usbdev_req_s *req);
static struct usbdev_req_s *usbmsc_allocreq(FAR struct usbdev_ep_s *ep,
                uint16_t len);
static void   usbmsc_freereq(FAR struct usbdev_ep_s *ep,
                FAR struct usbdev_req_s *req);

/* Class Driver Operations (most at interrupt level) ************************/

static int    usbmsc_bind(FAR struct usbdevclass_driver_s *driver,
                FAR struct usbdev_s *dev);
static void   usbmsc_unbind(FAR struct usbdevclass_driver_s *driver,
                FAR struct usbdev_s *dev);
static int    usbmsc_setup(FAR struct usbdevclass_driver_s *driver,
                FAR struct usbdev_s *dev,
                FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                size_t outlen);
static void   usbmsc_disconnect(FAR struct usbdevclass_driver_s *driver,
                FAR struct usbdev_s *dev);

/* Initialization/Uninitialization ******************************************/

static void   usbmsc_lununinitialize(struct usbmsc_lun_s *lun);
#if !defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBMSC_COMPOSITE)
static int    usbmsc_exportluns(FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver operations ********************************************************/

static struct usbdevclass_driverops_s g_driverops =
{
  usbmsc_bind,       /* bind */
  usbmsc_unbind,     /* unbind */
  usbmsc_setup,      /* setup */
  usbmsc_disconnect, /* disconnect */
  NULL,              /* suspend */
  NULL               /* resume */
};

/* Used to hand-off the state structure when the SCSI worker thread is
 * started.
 */

FAR struct usbmsc_dev_s *g_usbmsc_handoff;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbmsc_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: usbmsc_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *usbmsc_allocreq(FAR struct usbdev_ep_s *ep,
                                            uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);
      if (!req->buf)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }

  return req;
}

/****************************************************************************
 * Name: usbmsc_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbmsc_freereq(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      if (req->buf != NULL)
        {
          EP_FREEBUFFER(ep, req->buf);
        }

      EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: usbmsc_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbmsc_bind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct usbmsc_dev_s *priv =
    ((FAR struct usbmsc_driver_s *)driver)->dev;
  FAR struct usbmsc_req_s *reqcontainer;
  irqstate_t flags;
  int ret = OK;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  dev->ep0->priv = priv;
#endif

  /* The configured EP0 size should match the reported EP0 size.  We could
   * easily adapt to the reported EP0 size, but then we could not use the
   * const, canned descriptors.
   */

  DEBUGASSERT(CONFIG_USBMSC_EP0MAXPACKET == dev->ep0->maxpacket);

  /* Preallocate control request */

  priv->ctrlreq = usbmsc_allocreq(dev->ep0, USBMSC_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = usbmsc_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbmsc_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, USBMSC_MKEPBULKIN(&priv->devinfo),
                               true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, USBMSC_MKEPBULKOUT(&priv->devinfo),
                                false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests */

  for (i = 0; i < CONFIG_USBMSC_NRDREQS; i++)
    {
      reqcontainer      = &priv->rdreqs[i];
      reqcontainer->req = usbmsc_allocreq(priv->epbulkout,
                                          CONFIG_USBMSC_BULKOUTREQLEN);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDALLOCREQ),
                   (uint16_t)-ret);
          ret = -ENOMEM;
          goto errout;
        }

      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = usbmsc_rdcomplete;
    }

  /* Pre-allocate write request containers and put in a free list */

  for (i = 0; i < CONFIG_USBMSC_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = usbmsc_allocreq(priv->epbulkin,
                                          CONFIG_USBMSC_BULKINREQLEN);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRALLOCREQ),
                   (uint16_t)-ret);
          ret = -ENOMEM;
          goto errout;
        }

      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = usbmsc_wrcomplete;

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)reqcontainer, &priv->wrreqlist);
      leave_critical_section(flags);
    }

  /* Report if we are selfpowered (unless we are part of a composite
   * device).
   */

#ifndef CONFIG_USBMSC_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function (unless we are
   * part of a composite device)
   */

  DEV_CONNECT(dev);
#endif
  return OK;

errout:
  usbmsc_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbmsc_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbmsc_req_s *reqcontainer;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNBINDINVALIDARGS), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct usbmsc_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EP0NOTBOUND1), 0);
      return;
    }
#endif

  /* The worker thread should have already been stopped by the
   * driver un-initialize logic.
   */

  DEBUGASSERT(priv->thstate == USBMSC_STATE_TERMINATED ||
              priv->thstate == USBMSC_STATE_NOTSTARTED);

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling usbmsc_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      usbmsc_resetconfig(priv);
      up_mdelay(50);

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbmsc_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      for (i = 0; i < CONFIG_USBMSC_NRDREQS; i++)
        {
          reqcontainer = &priv->rdreqs[i];
          if (reqcontainer->req)
            {
              usbmsc_freereq(priv->epbulkout, reqcontainer->req);
              reqcontainer->req = NULL;
            }
        }

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      /* Free write requests that are not in use (which should be all
       * of them
       */

      flags = enter_critical_section();
      while (!sq_empty(&priv->wrreqlist))
        {
          reqcontainer = (struct usbmsc_req_s *)
            sq_remfirst(&priv->wrreqlist);

          if (reqcontainer->req != NULL)
            {
              usbmsc_freereq(priv->epbulkin, reqcontainer->req);
            }
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: usbmsc_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbmsc_setup(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout, size_t outlen)
{
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SETUPINVALIDARGS), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct usbmsc_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EP0NOTBOUND2), 0);
      return -ENODEV;
    }
#endif

  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            /* The value field specifies the descriptor type in the MS byte
             * and the descriptor index in the LS byte (order is little
             * endian)
             */

            switch (ctrl->value[1])
              {
                /* If the mass storage device is used in as part of a
                 * composite device, then the device descriptor is is
                 * provided by logic in the composite device implementation.
                 */

#ifndef CONFIG_USBMSC_COMPOSITE
              case USB_DESC_TYPE_DEVICE:
                {
                  ret = USB_SIZEOF_DEVDESC;
                  memcpy(ctrlreq->buf, usbmsc_getdevdesc(), ret);
                }
                break;
#endif

                /* If the mass storage device is used in as part of a
                 * composite device, then the device qualifier descriptor is
                 * provided by logic in the composite device implementation.
                 */

#if !defined(CONFIG_USBMSC_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, usbmsc_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif

                /* If the mass storage device is used in as part of a
                 * composite device, then the configuration descriptor is
                 * provided by logic in the composite device implementation.
                 */

#ifndef CONFIG_USBMSC_COMPOSITE
              case USB_DESC_TYPE_CONFIG:
                {
#ifdef CONFIG_USBDEV_DUALSPEED
                  ret = usbmsc_mkcfgdesc(ctrlreq->buf, &priv->devinfo,
                                         dev->speed, ctrl->value[1]);
#else
                  ret = usbmsc_mkcfgdesc(ctrlreq->buf, &priv->devinfo);
#endif
                }
                break;
#endif

                /* If the mass storage device is used in as part of a
                 * composite device, then the language string descriptor is
                 * provided by logic in the composite device implementation.
                 */

#ifndef CONFIG_USBMSC_COMPOSITE
              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret = usbmsc_mkstrdesc(ctrl->value[0],
                                         (FAR struct usb_strdesc_s *)
                                         ctrlreq->buf);
                }
                break;
#endif

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_GETUNKNOWNDESC),
                           value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                /* Signal the worker thread to instantiate the new
                 * configuration.
                 */

                priv->theventset |= USBMSC_EVENT_CFGCHANGE;
                priv->thvalue     = value;
                usbmsc_scsi_signal(priv);

                /* Return here... the response will be provided later by the
                 * worker thread.
                 */

                return OK;
              }
          }
          break;

          /* If the mass storage device is used in as part of a composite
           * device, then the overall composite class configuration is
           * managed by logic in the composite device implementation.
           */

#ifndef CONFIG_USBMSC_COMPOSITE
        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                ctrlreq->buf[0] = priv->config;
                ret = 1;
              }
          }
          break;
#endif

        case USB_REQ_SETINTERFACE:
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
              {
                if (priv->config == USBMSC_CONFIGID &&
                    index == USBMSC_INTERFACEID &&
                    value == USBMSC_ALTINTERFACEID)
                  {
                    /* Signal to instantiate the interface change */

                    priv->theventset |= USBMSC_EVENT_IFCHANGE;
                    usbmsc_scsi_signal(priv);

                    /* Return here... the response will be provided later by
                     * the worker thread.
                     */

                    return OK;
                  }
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == USBMSC_CONFIGIDNONE)
              {
                if (index != USBMSC_INTERFACEID)
                  {
                    ret = -EDOM;
                  }
                else
                  {
                    ctrlreq->buf[0] = USBMSC_ALTINTERFACEID;
                    ret = 1;
                  }
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNSUPPORTEDSTDREQ),
                   ctrl->req);
          break;
        }
    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * Bulk-Only Mass Storage Class Requests
       **********************************************************************/

      /* Verify that we are configured */

      if (!priv->config)
        {
           usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_NOTCONFIGURED), 0);
           return ret;
        }

      switch (ctrl->req)
        {
        case USBMSC_REQ_MSRESET: /* Reset mass storage device and interface */
          {
            if (ctrl->type == USBMSC_TYPE_SETUPOUT && value == 0 && len == 0)
              {
                /* Only one interface is supported */

                if (index != USBMSC_INTERFACEID)
                  {
                    usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_MSRESETNDX),
                             index);
                    ret = -EDOM;
                  }
                else
                  {
                    /* Signal to stop the current operation and reinitialize
                     * state.
                     */

                     priv->theventset |= USBMSC_EVENT_RESET;
                     usbmsc_scsi_signal(priv);

                    /* Return here... the response will be provided later by
                     * the worker thread.
                     */

                    return OK;
                  }
              }
          }
          break;

        case USBMSC_REQ_GETMAXLUN: /* Return number LUNs supported */
          {
            if (ctrl->type == USBMSC_TYPE_SETUPIN && value == 0 && len == 1)
              {
                /* Only one interface is supported */

                if (index != USBMSC_INTERFACEID)
                  {
                    usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_GETMAXLUNNDX),
                             index);
                    ret = -EDOM;
                  }
                else
                  {
                    ctrlreq->buf[0] = priv->nluns - 1;
                    ret = 1;
                  }
              }
          }
          break;

        default:
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BADREQUEST), ctrl->req);
          break;
        }
    }
  else
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall EP0.
   */

  if (ret >= 0)
    {
      /* Configure the response */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_USBMSC_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
#else
      ret = composite_ep0submit(driver, dev, ctrlreq, ctrl);
#endif
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EPRESPQ), (uint16_t)-ret);
#if 0 /* Not necessary */
          ctrlreq->result = OK;
          usbmsc_ep0incomplete(dev->ep0, ctrlreq);
#endif
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbmsc_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  struct usbmsc_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_DISCONNECTINVALIDARGS), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct usbmsc_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EP0NOTBOUND3), 0);
      return;
    }
#endif

  /* Reset the configuration */

  flags = enter_critical_section();
  usbmsc_resetconfig(priv);

  /* Signal the worker thread */

  priv->theventset |= USBMSC_EVENT_DISCONNECT;
  usbmsc_scsi_signal(priv);
  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: usbmsc_lununinitialize
 ****************************************************************************/

static void usbmsc_lununinitialize(struct usbmsc_lun_s *lun)
{
  /* Has a block driver has been bound to the LUN? */

  if (lun->inode)
    {
      /* Close the block driver */

      close_blockdriver(lun->inode);
    }

  memset(lun, 0, sizeof(struct usbmsc_lun_s));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queuing read and write requests.
 *
 ****************************************************************************/

int usbmsc_setconfig(FAR struct usbmsc_dev_s *priv, uint8_t config)
{
  FAR struct usbmsc_req_s *privreq;
  FAR struct usbdev_req_s *req;
  struct usb_epdesc_s epdesc;
  bool hispeed = false;
  int i;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SETCONFIGINVALIDARGS), 0);
      return -EIO;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_ALREADYCONFIGURED), 0);
      return OK;
    }

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (priv->usbdev->speed == USB_SPEED_HIGH);
#endif

  /* Discard the previous configuration data */

  usbmsc_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == USBMSC_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CONFIGNONE), 0);
      return OK;
    }

  /* We only accept one configuration */

  if (config != USBMSC_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN bulk endpoint */

  usbmsc_copy_epdesc(USBMSC_EPBULKIN, &epdesc, &priv->devinfo,
                     hispeed);
  ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

  usbmsc_copy_epdesc(USBMSC_EPBULKOUT, &epdesc, &priv->devinfo,
                     hispeed);
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  for (i = 0; i < CONFIG_USBMSC_NRDREQS; i++)
    {
      privreq       = &priv->rdreqs[i];
      req           = privreq->req;
      req->len      = CONFIG_USBMSC_BULKOUTREQLEN;
      req->priv     = privreq;
      req->callback = usbmsc_rdcomplete;

      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDSUBMIT),
                   (uint16_t)-ret);
          goto errout;
        }
    }

  priv->config = config;
  return OK;

errout:
  usbmsc_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

void usbmsc_resetconfig(FAR struct usbmsc_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != USBMSC_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = USBMSC_CONFIGIDNONE;

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbmsc_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

void usbmsc_wrcomplete(FAR struct usbdev_ep_s *ep,
                       FAR struct usbdev_req_s *req)
{
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbmsc_req_s *privreq;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRCOMPLETEINVALIDARGS), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv    = (FAR struct usbmsc_dev_s *)ep->priv;
  privreq = (FAR struct usbmsc_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)privreq, &priv->wrreqlist);
  leave_critical_section(flags);

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
    case OK: /* Normal completion */
      usbtrace(TRACE_CLASSWRCOMPLETE, req->xfrd);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRSHUTDOWN), 0);
      break;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };

  /* Inform the worker thread that a write request has been returned */

  priv->theventset |= USBMSC_EVENT_WRCOMPLETE;
  usbmsc_scsi_signal(priv);
}

/****************************************************************************
 * Name: usbmsc_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

void usbmsc_rdcomplete(FAR struct usbdev_ep_s *ep,
                       FAR struct usbdev_req_s *req)
{
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbmsc_req_s *privreq;
  irqstate_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDCOMPLETEINVALIDARGS), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv    = (FAR struct usbmsc_dev_s *)ep->priv;
  privreq = (FAR struct usbmsc_req_s *)req->priv;

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
    case 0: /* Normal completion */
      {
        usbtrace(TRACE_CLASSRDCOMPLETE, req->xfrd);

        /* Add the filled read request from the rdreqlist */

        flags = enter_critical_section();
        sq_addlast((FAR sq_entry_t *)privreq, &priv->rdreqlist);
        leave_critical_section(flags);

        /* Signal the worker thread that there is received data to be
         * processed.
         */

        priv->theventset |= USBMSC_EVENT_RDCOMPLETE;
        usbmsc_scsi_signal(priv);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDSHUTDOWN), 0);

        /* Drop the read request... it will be cleaned up later */
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDUNEXPECTED),
                 (uint16_t)-req->result);

        /* Return the read request to the bulk out endpoint for re-filling */

        req           = privreq->req;
        req->priv     = privreq;
        req->callback = usbmsc_rdcomplete;

        ret = EP_SUBMIT(priv->epbulkout, req);
        if (ret != OK)
          {
            usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_RDCOMPLETERDSUBMIT),
                     (uint16_t)-ret);
          }
      }
      break;
    }
}

/****************************************************************************
 * Name: usbmsc_deferredresponse
 *
 * Description:
 *   Some EP0 setup request cannot be responded to immediately because they
 *   require some asynchronous action from the SCSI worker thread.  This
 *   function is provided for the SCSI thread to make that deferred response.
 *   The specific requests that require this deferred response are:
 *
 *   1. USB_REQ_SETCONFIGURATION,
 *   2. USB_REQ_SETINTERFACE, or
 *   3. USBMSC_REQ_MSRESET
 *
 *   In all cases, the success response is a zero-length packet; the failure
 *   response is an EP0 stall.
 *
 * Input Parameters:
 *   priv  - Private state structure for this USB storage instance
 *   stall - true is the action failed and a stall is required
 *
 ****************************************************************************/

void usbmsc_deferredresponse(FAR struct usbmsc_dev_s *priv, bool failed)
{
#ifndef CONFIG_USBMSC_COMPOSITE
  FAR struct usbdev_s *dev;
  FAR struct usbdev_req_s *ctrlreq;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->usbdev || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_DEFERREDRESPINVALIDARGS), 0);
      return;
    }
#endif

  dev     = priv->usbdev;
  ctrlreq = priv->ctrlreq;

  /* If no error occurs, respond to the deferred setup command with a null
   * packet.
   */

  if (!failed)
    {
      ctrlreq->len   = 0;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret            = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_DEFERREDRESPSUBMIT),
                   (uint16_t)-ret);
#if 0 /* Not necessary */
          ctrlreq->result = OK;
          usbmsc_ep0incomplete(dev->ep0, ctrlreq);
#endif
        }
    }
  else
    {
      /* On a failure, the USB driver will stall. */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_DEFERREDRESPSTALLED), 0);
      EP_STALL(dev->ep0);
    }
#endif
}

/****************************************************************************
 * Name: usbmsc_sync_wait
 *
 * Description:
 *   Wait for the worker thread to obtain the USB MSC state data
 *
 ****************************************************************************/

static int usbmsc_sync_wait(FAR struct usbmsc_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->thsynch);
}

/****************************************************************************
 * Name: usbmsc_configure
 *
 * Description:
 *   One-time initialization of the USB storage driver.  The initialization
 *   sequence is as follows:
 *
 *   1. Call usbmsc_configure to perform one-time initialization specifying
 *      the number of luns.
 *   2. Call usbmsc_bindlun to configure each supported LUN
 *   3. Call usbmsc_exportluns when all LUNs are configured
 *
 * Input Parameters:
 *   nluns  - the number of LUNs that will be registered
 *   handle - Location to return a handle that is used in other API calls.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

int usbmsc_configure(unsigned int nluns, void **handle)
{
  FAR struct usbmsc_alloc_s  *alloc;
  FAR struct usbmsc_dev_s    *priv;
  FAR struct usbmsc_driver_s *drvr;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (nluns > 15)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_TOOMANYLUNS), 0);
      return -EDOM;
    }
#endif

  /* Allocate the structures needed */

  alloc = (FAR struct usbmsc_alloc_s *)
    kmm_malloc(sizeof(struct usbmsc_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Initialize the USB storage driver structure */

  priv = &alloc->dev;
  memset(priv, 0, sizeof(struct usbmsc_dev_s));

  /* Initialize semaphores & mutex */

  nxsem_init(&priv->thsynch, 0, 0);
  nxmutex_init(&priv->thlock);
  nxsem_init(&priv->thwaitsem, 0, 0);

  sq_init(&priv->wrreqlist);
  priv->nluns = nluns;

  /* Allocate the LUN table */

  priv->luntab = (FAR struct usbmsc_lun_s *)
    kmm_malloc(priv->nluns*sizeof(struct usbmsc_lun_s));

  if (!priv->luntab)
    {
      ret = -ENOMEM;
      goto errout;
    }

  memset(priv->luntab, 0, priv->nluns * sizeof(struct usbmsc_lun_s));

  /* Initialize the USB class driver structure */

  drvr             = &alloc->drvr;
#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed = USB_SPEED_HIGH;
#else
  drvr->drvr.speed = USB_SPEED_FULL;
#endif
  drvr->drvr.ops   = &g_driverops;
  drvr->dev        = priv;

  /* Initialize the device information if we are not part of a composite.
   * If we are part of a composite, the device information will be
   * initialized through coordinated actions of
   * usbmsc_get_composite_devdesc() and board-specific logic.
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  /* minor - not used */

  /* Interfaces (ifnobase == 0) */

  priv->devinfo.ninterfaces = USBMSC_NINTERFACES; /* Number of interfaces
                                                   * in the configuration */

  /* Strings (strbase == 0) */

  priv->devinfo.nstrings    = USBMSC_NSTRIDS;     /* Number of Strings */

  /* Endpoints */

  priv->devinfo.nendpoints = USBMSC_NENDPOINTS;
  priv->devinfo.epno[USBMSC_EP_BULKIN_IDX]  = CONFIG_USBMSC_EPBULKIN;
  priv->devinfo.epno[USBMSC_EP_BULKOUT_IDX] = CONFIG_USBMSC_EPBULKOUT;
#endif

  /* Return the handle and success */

  *handle = (FAR void *)alloc;
  return OK;

errout:
  usbmsc_uninitialize(alloc);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_bindlun
 *
 * Description:
 *   Bind the block driver specified by drvrpath to a USB storage LUN.
 *
 * Input Parameters:
 *   handle      - The handle returned by a previous call to
 *                 usbmsc_configure().
 *   drvrpath    - the full path to the block driver
 *   startsector - A sector offset into the block driver to the start of the
 *                 partition on drvrpath (0 if no partitions)
 *   nsectors    - The number of sectors in the partition (if 0, all sectors
 *                 to the end of the media will be exported).
 *   lunno       - the LUN to bind to
 *
 * Returned Value:
 *  0 on success; a negated errno on failure.
 *
 ****************************************************************************/

int usbmsc_bindlun(FAR void *handle, FAR const char *drvrpath,
                   unsigned int lunno, off_t startsector, size_t nsectors,
                   bool readonly)
{
  FAR struct usbmsc_alloc_s *alloc = (FAR struct usbmsc_alloc_s *)handle;
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbmsc_lun_s *lun;
  FAR struct inode *inode;
  struct geometry geo;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!alloc || !drvrpath || startsector < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BINLUNINVALIDARGS1), 0);
      return -EINVAL;
    }
#endif

  priv = &alloc->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->luntab)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INTERNALCONFUSION1), 0);
      return -EIO;
    }

  if (lunno > priv->nluns)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BINDLUNINVALIDARGS2), 0);
      return -EINVAL;
    }
#endif

  lun = &priv->luntab[lunno];

#ifdef CONFIG_DEBUG_FEATURES
  if (lun->inode != NULL)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_LUNALREADYBOUND), 0);
      return -EBUSY;
    }
#endif

  /* Open the block driver */

  ret = open_blockdriver(drvrpath, 0, &inode);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BLKDRVEOPEN), 0);
      return ret;
    }

  /* Get the drive geometry */

  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry ||
      inode->u.i_bops->geometry(inode, &geo) != OK || !geo.geo_available)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_NOGEOMETRY), 0);
      return -ENODEV;
    }

  /* Verify that the partition parameters are valid */

  if (startsector >= geo.geo_nsectors)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BINDLUNINVALIDARGS3), 0);
      return -EDOM;
    }
  else if (nsectors == 0)
    {
      nsectors = geo.geo_nsectors - startsector;
    }
  else if (startsector + nsectors >= geo.geo_nsectors)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_BINDLUNINVALIDARGS4), 0);
      return -EDOM;
    }

  /* Initialize the LUN structure */

  memset(lun, 0, sizeof(struct usbmsc_lun_s));

  /* Allocate an I/O buffer big enough to hold one hardware sector.  SCSI
   * commands are processed one at a time so all LUNs may share a single I/O
   * buffer.  The I/O buffer will be allocated so that is it as large as the
   * largest block device sector size
   */

  if (!priv->iobuffer)
    {
#ifdef CONFIG_USBMSC_WRMULTIPLE
      priv->iobuffer = (FAR uint8_t *)kmm_malloc(geo.geo_sectorsize *
                                                 CONFIG_USBMSC_NWRREQS);
#else
      priv->iobuffer = (FAR uint8_t *)kmm_malloc(geo.geo_sectorsize);
#endif
      if (!priv->iobuffer)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_ALLOCIOBUFFER),
                   geo.geo_sectorsize);
          return -ENOMEM;
        }

#ifdef CONFIG_USBMSC_WRMULTIPLE
      priv->iosize = geo.geo_sectorsize * CONFIG_USBMSC_NWRREQS;
#else
      priv->iosize = geo.geo_sectorsize;
#endif
    }
  else if (priv->iosize < geo.geo_sectorsize)
    {
      FAR void *tmp;

      tmp = (FAR void *)kmm_realloc(priv->iobuffer, geo.geo_sectorsize);
      if (!tmp)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_REALLOCIOBUFFER),
                   geo.geo_sectorsize);
          return -ENOMEM;
        }

      priv->iobuffer = (FAR uint8_t *)tmp;
      priv->iosize   = geo.geo_sectorsize;
    }

  lun->inode       = inode;
  lun->startsector = startsector;
  lun->nsectors    = nsectors;
  lun->sectorsize  = geo.geo_sectorsize;
  lun->readonly    = readonly;

  /* If the driver does not support the write method, then this is read-
   * only.
   */

  if (!inode->u.i_bops->write)
    {
      lun->readonly = true;
    }

  return OK;
}

/****************************************************************************
 * Name: usbmsc_unbindlun
 *
 * Description:
 *   Un-bind the block driver for the specified LUN
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure().
 *   lun    - the LUN to unbind from
 *
 * Returned Value:
 *  0 on success; a negated errno on failure.
 *
 ****************************************************************************/

int usbmsc_unbindlun(FAR void *handle, unsigned int lunno)
{
  FAR struct usbmsc_alloc_s *alloc = (FAR struct usbmsc_alloc_s *)handle;
  FAR struct usbmsc_dev_s *priv;
  FAR struct usbmsc_lun_s *lun;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNBINDLUNINVALIDARGS1), 0);
      return -EINVAL;
    }
#endif

  priv = &alloc->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->luntab)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INTERNALCONFUSION2), 0);
      return -EIO;
    }

  if (lunno > priv->nluns)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNBINDLUNINVALIDARGS2), 0);
      return -EINVAL;
    }
#endif

  lun = &priv->luntab[lunno];
  ret = nxmutex_lock(&priv->thlock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_DEBUG_FEATURES
  if (lun->inode == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_LUNNOTBOUND), 0);
      ret = -EBUSY;
    }
  else
#endif
    {
      /* Close the block driver */

      usbmsc_lununinitialize(lun);
      ret = OK;
    }

  nxmutex_unlock(&priv->thlock);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_exportluns
 *
 * Description:
 *   After all of the LUNs have been bound, this function may be called
 *   in order to export those LUNs in the USB storage device.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure().
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#if !defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBMSC_COMPOSITE)
static
#endif
int usbmsc_exportluns(FAR void *handle)
{
  FAR struct usbmsc_alloc_s *alloc = (FAR struct usbmsc_alloc_s *)handle;
  FAR struct usbmsc_dev_s *priv;
#ifndef CONFIG_USBMSC_COMPOSITE
  FAR struct usbmsc_driver_s *drvr;
#endif
  irqstate_t flags;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_EXPORTLUNSINVALIDARGS), 0);
      return -ENXIO;
    }
#endif

  priv = &alloc->dev;
#ifndef CONFIG_USBMSC_COMPOSITE
  drvr = &alloc->drvr;
#endif

  /* Start the worker thread
   *
   * REVISIT:  g_usbmsc_handoff is a global and, hence, really requires
   * some protection against re-entrant usage.
   */

  ret = nxmutex_lock(&priv->thlock);
  if (ret < 0)
    {
      return ret;
    }

  priv->thstate = USBMSC_STATE_NOTSTARTED;
  priv->theventset = USBMSC_EVENT_NOEVENTS;

  g_usbmsc_handoff = priv;

  uinfo("Starting SCSI worker thread\n");
  ret = kthread_create("scsid", CONFIG_USBMSC_SCSI_PRIO,
                       CONFIG_USBMSC_SCSI_STACKSIZE,
                       usbmsc_scsi_main, NULL);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_THREADCREATE), (uint16_t)ret);
      goto errout_with_lock;
    }

  priv->thpid = (pid_t)ret;

  /* Wait for the worker thread to run and initialize */

  uinfo("Waiting for the SCSI worker thread\n");
  ret = usbmsc_sync_wait(priv);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  DEBUGASSERT(g_usbmsc_handoff == NULL);

  /* Register the USB storage class driver (unless we are part of a composite
   * device).
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  ret = usbdev_register(&drvr->drvr);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      goto errout_with_lock;
    }
#endif

  /* Signal to start the thread */

  uinfo("Signalling for the SCSI worker thread\n");
  flags = enter_critical_section();
  priv->theventset |= USBMSC_EVENT_READY;
  usbmsc_scsi_signal(priv);
  leave_critical_section(flags);

errout_with_lock:
  nxmutex_unlock(&priv->thlock);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_classobject
 *
 * Description:
 *   Register USB mass storage device and return the class object.
 *
 * Input Parameters:
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
int usbmsc_classobject(FAR void *handle,
                       FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct usbmsc_alloc_s *alloc = (FAR struct usbmsc_alloc_s *)handle;
  int ret;

  DEBUGASSERT(handle != NULL && classdev != NULL);

  /* Save the device description */

  memcpy(&alloc->dev.devinfo, devinfo, sizeof(struct usbdev_devinfo_s));

  /* Export the LUNs as with the "standalone" USB mass storage driver, but
   * don't register the class instance with the USB device infrastructure.
   */

  ret = usbmsc_exportluns(handle);
  if (ret == OK)
    {
      /* On success, return an (typed) instance of the class instance */

      *classdev = &alloc->drvr.drvr;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: usbmsc_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbmsc_uninitialize(FAR void *handle)
{
  FAR struct usbmsc_alloc_s *alloc = (FAR struct usbmsc_alloc_s *)handle;
  FAR struct usbmsc_dev_s *priv;
  irqstate_t flags;
  int ret;
  int i;

#ifdef CONFIG_DEBUG_FEATURES
  if (!handle)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_UNINITIALIZEINVALIDARGS), 0);
      return;
    }
#endif

  priv = &alloc->dev;

#ifdef CONFIG_USBMSC_COMPOSITE
  /* Check for pass 2 uninitialization.  We did most of the work on the
   * first pass uninitialization.
   */

  if (priv->thpid == 0)
    {
      /* In this second and final pass, all that remains to be done is to
       * free the memory resources.
       */

      kmm_free(priv);
      return;
    }
#endif

  /* If the thread hasn't already exitted, tell it to exit now */

  if (priv->thstate != USBMSC_STATE_NOTSTARTED)
    {
      /* Get exclusive access to SCSI state data */

      do
        {
          ret = nxmutex_lock(&priv->thlock);

          /* nxmutex_lock() will fail with ECANCELED, only
           * if this thread is canceled.  At this point, we
           * have no option but to continue with the teardown.
           */

          DEBUGASSERT(ret == OK || ret == -ECANCELED);
        }
      while (ret < 0);

      /* The thread was started.. Is it still running? */

      if (priv->thstate != USBMSC_STATE_TERMINATED)
        {
          /* Yes.. Ask the thread to stop */

          flags = enter_critical_section();
          priv->theventset |= USBMSC_EVENT_TERMINATEREQUEST;
          usbmsc_scsi_signal(priv);
          leave_critical_section(flags);
        }

      nxmutex_unlock(&priv->thlock);

      /* Wait for the thread to exit */

      while ((priv->theventset & USBMSC_EVENT_TERMINATEREQUEST) != 0)
        {
          ret = usbmsc_sync_wait(priv);
          if (ret < 0)
            {
              /* Just break out and continue if the thread has been
               * canceled.
               */

              break;
            }
        }
    }

  priv->thpid = 0;

  /* Unregister the driver (unless we are a part of a composite device) */

#ifndef CONFIG_USBMSC_COMPOSITE
  usbdev_unregister(&alloc->drvr.drvr);
#endif

  /* Uninitialize and release the LUNs */

  for (i = 0; i < priv->nluns; ++i)
    {
      usbmsc_lununinitialize(&priv->luntab[i]);
    }

  kmm_free(priv->luntab);

  /* Release the I/O buffer */

  if (priv->iobuffer)
    {
      kmm_free(priv->iobuffer);
    }

  /* Uninitialize and release the driver structure */

  nxsem_destroy(&priv->thsynch);
  nxmutex_destroy(&priv->thlock);
  nxsem_destroy(&priv->thwaitsem);

#ifndef CONFIG_USBMSC_COMPOSITE
  /* For the case of the composite driver, there is a two pass
   * uninitialization sequence.  We cannot yet free the driver structure.
   * We will do that on the second pass (and we will know that it is the
   * second pass because of priv->thpid == 0)
   */

  kmm_free(priv);
#endif
}

/****************************************************************************
 * Name: usbmsc_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBMSC_COMPOSITE)
void usbmsc_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the CDC/ACM class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc          = usbmsc_mkcfgdesc;
  dev->mkstrdesc           = usbmsc_mkstrdesc;

  dev->nconfigs            = USBMSC_NCONFIGS;        /* Number of configurations supported */
  dev->configid            = USBMSC_CONFIGID;        /* The only supported configuration ID */
  dev->cfgdescsize         = SIZEOF_USBMSC_CFGDESC;  /* The size of the config descriptor */

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = USBMSC_NINTERFACES;     /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = USBMSC_NSTRIDS;         /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = USBMSC_NENDPOINTS;
}
#endif
