/****************************************************************************
 * drivers/usbdev/composite.c
 *
 *   Copyright (C) 2012, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "composite.h"

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The internal version of the class driver */

struct composite_driver_s
{
  struct usbdevclass_driver_s  drvr;
  FAR struct composite_dev_s  *dev;
};

/* This is what is allocated */

struct composite_alloc_s
{
  struct composite_dev_s       dev;
  struct composite_driver_s    drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* USB helps ****************************************************************/

static void    composite_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static int     composite_classsetup(FAR struct composite_dev_s *priv,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static struct usbdev_req_s *composite_allocreq(FAR struct usbdev_ep_s *ep,
                 uint16_t len);
static void    composite_freereq(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     composite_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    composite_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     composite_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    composite_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    composite_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    composite_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  composite_bind,       /* bind */
  composite_unbind,     /* unbind */
  composite_setup,      /* setup */
  composite_disconnect, /* disconnect */
  composite_suspend,    /* suspend */
  composite_resume,     /* resume */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char g_compvendorstr[]  = CONFIG_COMPOSITE_VENDORSTR;
const char g_compproductstr[] = CONFIG_COMPOSITE_PRODUCTSTR;
const char g_compserialstr[]  = CONFIG_COMPOSITE_SERIALSTR;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_ep0incomplete
 *
 * Description:
 *   Handle completion of the composite driver's EP0 control operations
 *
 ****************************************************************************/

static void composite_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                    FAR struct usbdev_req_s *req)
{
  /* Just check the result of the transfer */

  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: composite_classsetup
 *
 * Description:
 *   Forward a setup command to the appropriate component device
 *
 ****************************************************************************/

static int composite_classsetup(FAR struct composite_dev_s *priv,
                                FAR struct usbdev_s *dev,
                                FAR const struct usb_ctrlreq_s *ctrl,
                                FAR uint8_t *dataout, size_t outlen)
{
  uint16_t index;
  uint8_t  interface;
  int ret = -EOPNOTSUPP;
  int i;

  index     = GETUINT16(ctrl->index);
  interface = (uint8_t)(index & 0xff);

  for (i = 0; i < priv->ndevices; i++)
    {
      if (interface >= priv->device[i].compdesc.devinfo.ifnobase &&
          interface < (priv->device[i].compdesc.devinfo.ifnobase +
                       priv->device[i].compdesc.devinfo.ninterfaces))
        {
          ret = CLASS_SETUP(priv->device[i].dev, dev, ctrl, dataout, outlen);
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: composite_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *composite_allocreq(FAR struct usbdev_ep_s *ep,
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
 * Name: composite_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void composite_freereq(FAR struct usbdev_ep_s *ep,
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
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: composite_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int composite_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct composite_dev_s *priv =
    ((FAR struct composite_driver_s *)driver)->dev;

  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events.
   */

  dev->ep0->priv = priv;

  /* Preallocate one control request */

  priv->ctrlreq = composite_allocreq(dev->ep0, priv->cfgdescsize);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  /* Initialize the pre-allocated control request */

  priv->ctrlreq->callback = composite_ep0incomplete;

  /* Then bind each of the constituent class drivers */

  for (i = 0; i < priv->ndevices; i++)
    {
      ret = CLASS_BIND(priv->device[i].dev, dev);
      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Report if we are selfpowered */

#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function */

  DEV_CONNECT(dev);
  return OK;

errout:
  composite_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: composite_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void composite_unbind(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev)
{
  FAR struct composite_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct composite_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      int i;

      /* Unbind the constituent class drivers */

      flags = enter_critical_section();
      for (i = 0; i < priv->ndevices; i++)
        {
          CLASS_UNBIND(priv->device[i].dev, dev);
        }

      /* Free the pre-allocated control request */

      priv->config = COMPOSITE_CONFIGIDNONE;
      if (priv->ctrlreq != NULL)
        {
          composite_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: composite_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int composite_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen)
{
  FAR struct composite_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  bool dispatched = false;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_SETUPINVALIDARGS), 0);
      return -EIO;
     }
#endif

  /* Extract a reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct composite_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EP0NOTBOUND2), 0);
      return -ENODEV;
    }
#endif

  ctrlreq   = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value     = GETUINT16(ctrl->value);
  index     = GETUINT16(ctrl->index);
  len       = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);
  UNUSED(index);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            /* The value field specifies the descriptor type in the MS byte and the
             * descriptor index in the LS byte (order is little endian)
             */

            switch (ctrl->value[1])
              {
              case USB_DESC_TYPE_DEVICE:
                {
                  ret = USB_SIZEOF_DEVDESC;
                  memcpy(ctrlreq->buf, composite_getdevdesc(), ret);
                }
                break;

#ifdef CONFIG_USBDEV_DUALSPEED
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, composite_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif

              case USB_DESC_TYPE_CONFIG:
                {
#ifdef CONFIG_USBDEV_DUALSPEED
                    ret = composite_mkcfgdesc(priv, ctrlreq->buf, dev->speed,
                                            ctrl->value[1]);
#else
                    ret = composite_mkcfgdesc(priv, ctrlreq->buf);
#endif
                }
                break;

              case USB_DESC_TYPE_STRING:
                {
                  /* value == string index. Zero is the language ID. */

                  uint8_t strid = ctrl->value[0];
                  FAR struct usb_strdesc_s *buf = (FAR struct usb_strdesc_s *)ctrlreq->buf;

                  if (strid <= COMPOSITE_NSTRIDS)
                    {
                      ret = composite_mkstrdesc(strid, buf);
                    }
                  else
                    {
                      int i;

                      for (i = 0; i < priv->ndevices; i++)
                        {
                          if (strid >= priv->device[i].compdesc.devinfo.strbase &&
                              strid <  priv->device[i].compdesc.devinfo.strbase +
                                       priv->device[i].compdesc.devinfo.nstrings)
                            {
                              ret = priv->device[i].compdesc.mkstrdesc(strid - priv->device[i].compdesc.devinfo.strbase, buf);
                              break;
                            }
                        }
                    }
                }
                break;

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_GETUNKNOWNDESC), value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                int i;

                /* Save the configuration and inform the constituent classes */

                for (i = 0; i < priv->ndevices; i++)
                  {
                    ret = CLASS_SETUP(priv->device[i].dev, dev, ctrl, dataout, outlen);
                  }

                dispatched = true;
                priv->config = value;
              }
          }
          break;

        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                ctrlreq->buf[0] = priv->config;
                ret = 1;
              }
          }
          break;

        case USB_REQ_SETINTERFACE:
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
                priv->config == COMPOSITE_CONFIGID)
              {
                ret = composite_classsetup(priv, dev, ctrl, dataout, outlen);
                dispatched = true;
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == COMPOSITE_CONFIGIDNONE)
              {
                ret = composite_classsetup(priv, dev, ctrl, dataout, outlen);
                dispatched = true;
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
          break;
        }
    }
  else
    {
      uint8_t recipient;

      /**********************************************************************
       * Non-Standard Class Requests
       **********************************************************************/

       /* Class implementations should handle there own interface and endpoint
        * requests.
        */

       recipient = ctrl->type & USB_REQ_RECIPIENT_MASK;
       if (recipient == USB_REQ_RECIPIENT_INTERFACE || recipient == USB_REQ_RECIPIENT_ENDPOINT)
         {
           ret = composite_classsetup(priv, dev, ctrl, dataout, outlen);
           dispatched = true;
         }
    }

  /* Respond to the setup command if (1) data was returned, and (2) the
   * request was NOT successfully dispatched to the component class driver.
   * On an error return value (ret < 0), the USB driver will stall EP0.
   */

  if (ret >= 0 && !dispatched)
    {
      /* Setup the request */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* And submit the request to the USB controller driver */

      ret = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          composite_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: composite_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void composite_disconnect(FAR struct usbdevclass_driver_s *driver,
                                 FAR struct usbdev_s *dev)
{
  FAR struct composite_dev_s *priv;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct composite_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Reset the configuration and inform the constituent class drivers of
   * the disconnection.
   */

  flags = enter_critical_section();
  priv->config = COMPOSITE_CONFIGIDNONE;

  for (i = 0; i < priv->ndevices; i++)
    {
      CLASS_DISCONNECT(priv->device[i].dev, dev);
    }

  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated.
   */

  DEV_CONNECT(dev);
}

/****************************************************************************
 * Name: composite_suspend
 *
 * Description:
 *   Invoked on a USB suspend event.
 *
 ****************************************************************************/

static void composite_suspend(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  FAR struct composite_dev_s *priv;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSSUSPEND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct composite_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Forward the suspend event to the constituent devices */

  flags = enter_critical_section();

  for (i = 0; i < priv->ndevices; i++)
    {
      CLASS_SUSPEND(priv->device[i].dev, priv->usbdev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: composite_resume
 *
 * Description:
 *   Invoked on a USB resume event.
 *
 ****************************************************************************/

static void composite_resume(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev)
{
  FAR struct composite_dev_s *priv = NULL;
  irqstate_t flags;
  int i;

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct composite_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Forward the resume event to the constituent devices */

  flags = enter_critical_section();

  for (i = 0; i < priv->ndevices; i++)
    {
      CLASS_RESUME(priv->device[i].dev, priv->usbdev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: composite_initialize
 *
 * Description:
 *   Register USB composite device as configured.  This function will call
 *   board-specific implementations in order to obtain the class objects for
 *   each of the members of the composite.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.  This handle may be used
 *   later with composite_uninitialize() in order to removed the composite
 *   device.  This handle is the (untyped) internal representation of the
 *   the class driver instance.
 *
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR void *composite_initialize(uint8_t ndevices,
                               FAR struct composite_devdesc_s *pdevices)
{
  FAR struct composite_alloc_s *alloc;
  FAR struct composite_dev_s *priv;
  FAR struct composite_driver_s *drvr;
  int ret;
  int i;

  DEBUGASSERT(pdevices != NULL && ndevices <= NUM_DEVICES_TO_HANDLE);

  /* Allocate the structures needed */

  alloc = (FAR struct composite_alloc_s *)
    kmm_malloc(sizeof(struct composite_alloc_s));

  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_ALLOCDEVSTRUCT), 0);
      return NULL;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB composite driver structure */

  memset(priv, 0, sizeof(struct composite_dev_s));

  priv->cfgdescsize = USB_SIZEOF_CFGDESC;
  priv->ninterfaces = 0;

  /* Get the constituent class driver objects */

  for (i = 0; i < ndevices; i++)
    {
      memcpy(&priv->device[i].compdesc, &pdevices[i],
             sizeof(struct composite_devdesc_s));

      ret =
        priv->device[i].compdesc.classobject(priv->device[i].compdesc.minor,
                                             &priv->device[i].compdesc.devinfo,
                                             &priv->device[i].dev);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_CLASSOBJECT),
                   (uint16_t)-ret);
          goto errout_with_alloc;
        }

      priv->cfgdescsize += priv->device[i].compdesc.cfgdescsize;
      priv->ninterfaces += priv->device[i].compdesc.devinfo.ninterfaces;
    }

  priv->ndevices = ndevices;

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  /* Register the USB composite class driver */

  ret = usbdev_register(&drvr->drvr);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBCOMPOSITE_TRACEERR_DEVREGISTER),
                             (uint16_t)-ret);
      goto errout_with_alloc;
    }

  return (FAR void *)alloc;

errout_with_alloc:
  kmm_free(alloc);
  return NULL;
}

/****************************************************************************
 * Name: composite_uninitialize
 *
 * Description:
 *   Un-initialize the USB composite driver.  The handle is the USB composite
 *   class' device object as was returned by composite_initialize().  This
 *   function will call  board-specific implementations in order to free the
 *   class objects for each of the members of the composite.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to composite_initialize().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void composite_uninitialize(FAR void *handle)
{
  FAR struct composite_alloc_s *alloc = (FAR struct composite_alloc_s *)handle;
  FAR struct composite_dev_s *priv;
  int i;

  DEBUGASSERT(alloc != NULL);

  /* First phase uninitialization each of the member classes */

  priv = &alloc->dev;

  for (i = 0; i < priv->ndevices; i++)
    {
      priv->device[i].compdesc.uninitialize(priv->device[i].dev);
    }

  /* Then unregister and destroy the composite class */

  usbdev_unregister(&alloc->drvr.drvr);

  /* Free any resources used by the composite driver */
  /* None */

  /* Second phase uninitialization:  Clean up all memory resources */

  for (i = 0; i < priv->ndevices; i++)
    {
      priv->device[i].compdesc.uninitialize(priv->device[i].dev);
    }

  /* Then free the composite driver state structure itself */

  kmm_free(priv);
}

/****************************************************************************
 * Name: composite_ep0submit
 *
 * Description:
 *   Members of the composite cannot send on EP0 directly because EP0 is
 *   is "owned" by the composite device.  Instead, when configured as members
 *   of a composite device, those classes should call this method so that
 *   the composite device can send on EP0 onbehalf of the class.
 *
 ****************************************************************************/

int composite_ep0submit(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR struct usbdev_req_s *ctrlreq)
{
  /* This function is not really necessary in the current design.  However,
   * keeping this will provide us a little flexibility in the future if
   * it becomes necessary to manage the completion callbacks.
   */

   return EP_SUBMIT(dev->ep0, ctrlreq);
}

#endif /* CONFIG_USBDEV_COMPOSITE */
