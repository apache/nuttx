/****************************************************************************
 * drivers/usbdev/dfu.c
 *
 *   Copyright (C) 2011-2018 Gregory Nutt. All rights reserved.
 *   Authors: Petteri Aimonen <jpa@git.mail.kapsi.fi>
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

/* This is a driver for the USB Device Firmware Upgrade protocol v1.1.
 * Currently it supports the app-side ("Run-Time") part of the protocol:
 * a sequence of DFU_DETACH and USB reset commands, which will reboot into
 * a separate USB DFU bootloader.
 *
 * The bootloader is provided by board-specific logic, or STM32's
 * built-in ROM bootloader can be used.
 *
 * https://www.usb.org/sites/default/files/DFU_1.1.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/dfu.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define DFU_MAX_TIMEOUT     255
#define DFU_MAX_TRANSFER    2048
#define DFU_VERSION         0x011A

#define USB_REQ_DFU_DETACH      0
#define USB_REQ_DFU_GETSTATUS   3

#ifdef CONFIG_DFU_MSFT_OS_DESCRIPTORS
#  define DFU_MAX_DESCRIPTOR_LEN 256
#else
#  define DFU_MAX_DESCRIPTOR_LEN sizeof(struct dfu_cfgdesc_s)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Response to DFU_GETSTATUS */

struct dfu_getstatus_response_s
{
  uint8_t status;           /* Status of latest command */
  uint8_t poll_timeout[3];  /* Time until next GETSTATUS request */
  uint8_t state;            /* Current state of the device */
  uint8_t string_idx;       /* Optional string description */
};

/* DFU functional descriptor */

struct dfu_funcdesc_s
{
  uint8_t   len;                /* Descriptor length */
  uint8_t   type;               /* 0x21 = DFU FUNCTIONAL */
  uint8_t   attributes;         /* Bit mask of supported features */
  uint8_t   detach_timeout[2];  /* Maximum time in milliseconds between DFU_DETACH and USB reset */
  uint8_t   transfer_size[2];   /* Maximum number of bytes in control writes */
  uint8_t   dfu_version[2];     /* Version of DFU specification supported */
};

/* USB configuration descriptor */

struct dfu_cfgdesc_s
{
  struct usb_ifdesc_s   ifdesc;    /* DFU interface descriptor */
  struct dfu_funcdesc_s funcdesc;  /* DFU functional descriptor */
};

struct dfu_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbdev_devinfo_s  devinfo;
  FAR struct usbdev_req_s *ctrlreq;  /* Pointer to preallocated control request */
  struct work_s work_item;           /* Work queue entry for activating bootloader */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* usbclass callbacks */

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen);
static int  usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);
static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev);
static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB driver operations */

const static struct usbdevclass_driverops_s g_dfu_driverops =
{
  &usbclass_bind,
  &usbclass_unbind,
  &usbclass_setup,
  &usbclass_disconnect,
  NULL,
  NULL
};

static const struct dfu_cfgdesc_s g_dfu_cfgdesc =
{
  {
    .len            = USB_SIZEOF_IFDESC,
    .type           = USB_DESC_TYPE_INTERFACE,
    .ifno           = 0,
    .alt            = 0,
    .neps           = 0,
    .classid        = 0xFE,
    .subclass       = 0x01,
    .protocol       = 0x01, /* DFU runtime protocol */
    .iif            = 0
  },
  {
    .len            = sizeof(struct dfu_funcdesc_s),
    .type           = 0x21,
    .attributes     = 0x0B,
    .detach_timeout = { LSBYTE(DFU_MAX_TIMEOUT), MSBYTE(DFU_MAX_TIMEOUT) },
    .transfer_size  = { LSBYTE(DFU_MAX_TRANSFER), MSBYTE(DFU_MAX_TRANSFER) },
    .dfu_version    = { LSBYTE(DFU_VERSION), MSBYTE(DFU_VERSION) }
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(FAR struct usbdev_ep_s *ep,
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
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static FAR struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                                  uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);

      if (req->buf == NULL)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }

  return req;
}

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
}

static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
{
  FAR struct dfu_cfgdesc_s *dest = (FAR struct dfu_cfgdesc_s *)buf;

  *dest = g_dfu_cfgdesc;
  dest->ifdesc.ifno += devinfo->ifnobase;
  dest->ifdesc.iif = devinfo->strbase;

  return sizeof(g_dfu_cfgdesc);
}

static int convert_to_utf16(FAR uint8_t *dest, FAR const char *src)
{
  int bytes = 0;

  while (*src)
    {
      *dest++ = *src++;
      *dest++ = 0x00;
      bytes += 2;
    }

  return bytes;
}

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR const char *str;

  if (id == 0)
    {
      str = CONFIG_DFU_INTERFACE_NAME;
    }
  else
    {
      return -EINVAL;
    }

  strdesc->len  = 2 + convert_to_utf16(strdesc->data, str);
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

#ifdef CONFIG_DFU_MSFT_OS_DESCRIPTORS

static int dfu_make_msft_extprop_desc(FAR uint8_t *buf)
{
  FAR const char *propname = "DeviceInterfaceGUIDs";
  FAR const char *propval = CONFIG_DFU_INTERFACE_GUID;
  FAR struct usb_msft_os_extprop_hdr_s *hdr =
    (FAR struct usb_msft_os_extprop_hdr_s *)buf;
  FAR uint8_t *payload = buf + sizeof(struct usb_msft_os_extprop_hdr_s);
  int namelen;
  int valuelen;
  int proplen;
  int totallen;

  namelen  = strlen(propname) * 2 + 2;
  valuelen = strlen(propval) * 2 + 4;
  proplen  = 14 + namelen + valuelen;
  totallen = sizeof(struct usb_msft_os_extprop_hdr_s) + proplen;

  memset(buf, 0, totallen);
  hdr->len[0]     = LSBYTE(totallen);
  hdr->len[1]     = MSBYTE(totallen);
  hdr->version[1] = 0x01;
  hdr->index[0]   = MSFTOSDESC_INDEX_EXTPROP;
  hdr->count[0]   = 1;

  *payload++ = LSBYTE(proplen); /* dwSize */
  *payload++ = MSBYTE(proplen);
  *payload++ = 0;
  *payload++ = 0;
  *payload++ = 7; /* dwPropertyDataType = REG_MULTI_SZ */
  *payload++ = 0;
  *payload++ = 0;
  *payload++ = 0;
  *payload++ = LSBYTE(namelen); /* wPropertyNameLength */
  *payload++ = MSBYTE(namelen);
  payload   += convert_to_utf16(payload, propname); /* bPropertyName */
  *payload++ = 0; /* Null terminator */
  *payload++ = 0;
  *payload++ = LSBYTE(valuelen); /* dwPropertyDataLength */
  *payload++ = MSBYTE(valuelen);
  *payload++ = 0;
  *payload++ = 0;
  payload   += convert_to_utf16(payload, propval);
  *payload++ = 0; /* Null terminator for string */
  *payload++ = 0;
  *payload++ = 0; /* Null terminator for array */
  *payload++ = 0;

  return totallen;
}
#endif

static void dfu_workqueue_callback(void *arg)
{
  usbdev_dfu_activate_bootloader();
}

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen)
{
  FAR struct dfu_driver_s *priv = (FAR struct dfu_driver_s *)driver;
  FAR struct usbdev_req_s *ctrlreq = priv->ctrlreq;
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  uinfo("type=%02x req=%02x value=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      if (ctrl->req == USB_REQ_GETDESCRIPTOR)
        {
          if (ctrl->value[1] == USB_DESC_TYPE_CONFIG)
            {
              ret = usbclass_mkcfgdesc(ctrlreq->buf, &priv->devinfo);
            }
          else if (ctrl->value[1] == USB_DESC_TYPE_STRING)
            {
              ret = usbclass_mkstrdesc(ctrl->value[0],
                                       (FAR struct usb_strdesc_s *)ctrlreq->buf);
            }
        }
      else if (ctrl->req == USB_REQ_SETCONFIGURATION)
        {
          return 0; /* Composite driver will send the reply */
        }
      else if (ctrl->req == USB_REQ_SETINTERFACE)
        {
          /* Only one alternate setting (0) is supported */

          if (value == 0)
            {
              ret = 0;
            }
        }
      else if (ctrl->req == USB_REQ_GETINTERFACE)
        {
          *(FAR uint8_t *) ctrlreq->buf = 0;
          ret = 1;
        }
    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      if (ctrl->req == USB_REQ_DFU_DETACH)
        {
          /* Execute the bootloader activation through work queue, so that
           * we can send the USB reply packet first.
           */

          work_queue(HPWORK, &priv->work_item, dfu_workqueue_callback, NULL, 1);
          ret = 0;
        }
      else if (ctrl->req == USB_REQ_DFU_GETSTATUS)
        {
          /* Respond with APP_IDLE status */

          FAR struct dfu_getstatus_response_s *response =
            (FAR struct dfu_getstatus_response_s *)ctrlreq->buf;

          memset(response, 0, sizeof(struct dfu_getstatus_response_s));
          ret = sizeof(struct dfu_getstatus_response_s);
        }
    }
#ifdef CONFIG_DFU_MSFT_OS_DESCRIPTORS
  else if (ctrl->req == USB_REQ_GETMSFTOSDESCRIPTOR)
    {
      ret = dfu_make_msft_extprop_desc(ctrlreq->buf);
    }
#endif

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = (len < ret) ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret            = composite_ep0submit(driver, dev, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
        }
    }

  return ret;
}

static int  usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct dfu_driver_s *priv = (FAR struct dfu_driver_s *)driver;

  priv->ctrlreq = usbclass_allocreq(dev->ep0, DFU_MAX_DESCRIPTOR_LEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      return -ENOMEM;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  return OK;
}

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct dfu_driver_s *priv = (FAR struct dfu_driver_s *)driver;

  if (priv->ctrlreq != NULL)
    {
      usbclass_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }
}

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
}

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Allocate memory for the RNDIS driver class object
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct dfu_driver_s *alloc;

  alloc = kmm_zalloc(sizeof(struct dfu_driver_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  *classdev = &alloc->drvr;

  alloc->drvr.speed = USB_SPEED_FULL;
  alloc->drvr.ops = &g_dfu_driverops;

  return OK;
}

/****************************************************************************
 * Name: usbclass_uninitialize
 *
 * Description:
 *   Free allocated memory
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  kmm_free(classdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void usbdev_dfu_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbclass_classobject;
  dev->uninitialize        = usbclass_uninitialize;
  dev->nconfigs            = 1;
  dev->configid            = 0;
  dev->cfgdescsize         = sizeof(g_dfu_cfgdesc);
  dev->devinfo.ninterfaces = 1;
  dev->devinfo.nstrings    = 1;
  dev->devinfo.nendpoints  = 0;

#ifdef CONFIG_DFU_MSFT_OS_DESCRIPTORS
  memcpy(dev->msft_compatible_id, "WINUSB", 6);
#endif
}
