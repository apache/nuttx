/****************************************************************************
 * drivers/usbdev/composite_desc.c
 *
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev_trace.h>

#include "composite.h"

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device Descriptor */

static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
#ifndef CONFIG_COMPOSITE_IAD
  USB_CLASS_MISC,                               /* classid */
  2,                                            /* subclass */
  1,                                            /* protocol */
#else
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
#endif
  CONFIG_COMPOSITE_EP0MAXPACKET,                /* maxpacketsize */
  {
    LSBYTE(CONFIG_COMPOSITE_VENDORID),          /* vendor */
    MSBYTE(CONFIG_COMPOSITE_VENDORID)
  },
  {
    LSBYTE(CONFIG_COMPOSITE_PRODUCTID),         /* product */
    MSBYTE(CONFIG_COMPOSITE_PRODUCTID)
  },
  {
    LSBYTE(CONFIG_COMPOSITE_VERSIONNO),         /* device */
    MSBYTE(CONFIG_COMPOSITE_VERSIONNO)
  },
  COMPOSITE_MANUFACTURERSTRID,                  /* imfgr */
  COMPOSITE_PRODUCTSTRID,                       /* iproduct */
  COMPOSITE_SERIALSTRID,                        /* serno */
  COMPOSITE_NCONFIGS                            /* nconfigs */
};

#ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
     LSBYTE(0x0200),
     MSBYTE(0x0200)
  },
  USB_CLASS_VENDOR_SPEC,                        /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_COMPOSITE_EP0MAXPACKET,                /* mxpacketsize */
  COMPOSITE_NCONFIGS,                           /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int composite_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(COMPOSITE_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(COMPOSITE_STR_LANGUAGE);
        return 4;
      }

    case COMPOSITE_MANUFACTURERSTRID:
      str = g_compvendorstr;
      break;

    case COMPOSITE_PRODUCTSTRID:
      str = g_compproductstr;
      break;

    case COMPOSITE_SERIALSTRID:
      str = g_compserialstr;
      break;

    case COMPOSITE_CONFIGSTRID:
      str = CONFIG_COMPOSITE_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
}

/****************************************************************************
 * Name: composite_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

FAR const struct usb_devdesc_s *composite_getdevdesc(void)
{
  return &g_devdesc;
}

/****************************************************************************
 * Name: composite_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t composite_mkcfgdesc(FAR struct composite_dev_s *priv, FAR uint8_t *buf,
                            uint8_t speed, uint8_t type)
#else
int16_t composite_mkcfgdesc(FAR struct composite_dev_s *priv, FAR uint8_t *buf)
#endif
{
  FAR struct usb_cfgdesc_s *cfgdesc = (FAR struct usb_cfgdesc_s *)buf;
  int16_t len;
  int16_t total;
  int i;

  /* Configuration descriptor for the composite device */
  /* Fill in the values directly into the buf */

  cfgdesc->len         = USB_SIZEOF_CFGDESC;               /* Descriptor length */
  cfgdesc->type        = USB_DESC_TYPE_CONFIG;             /* Descriptor type */
  cfgdesc->totallen[0] = LSBYTE(priv->cfgdescsize);        /* Lower Byte of Total length */
  cfgdesc->totallen[1] = MSBYTE(priv->cfgdescsize);        /* High Byte of Total length */
  cfgdesc->ninterfaces = priv->ninterfaces;                /* Number of interfaces */
  cfgdesc->cfgvalue    = COMPOSITE_CONFIGID;               /* Configuration value */
  cfgdesc->icfg        = COMPOSITE_CONFIGSTRID;            /* Configuration */
  cfgdesc->attr        = USB_CONFIG_ATTR_ONE |             /* Attributes */
                         COMPOSITE_SELFPOWERED |
                         COMPOSITE_REMOTEWAKEUP;
  cfgdesc->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2; /* Max power (mA/2) */

  /* increment the size and buf to point right behind the information filled in */

  total = USB_SIZEOF_CFGDESC;
  buf += USB_SIZEOF_CFGDESC;

  /* Copy all contained interface descriptors into the buffer too */

  for (i = 0; i < priv->ndevices; i++)
    {
#ifdef CONFIG_USBDEV_DUALSPEED
      len = priv->device[i].compdesc.mkconfdesc(buf,
                                                &priv->device[i].compdesc.devinfo,
                                                speed, type);
      total += len;
      buf += len;
#else
      len = priv->device[i].compdesc.mkconfdesc(buf,
                                                &priv->device[i].compdesc.devinfo);
      total += len;
      buf += len;
#endif
    }

  return total;
}

/****************************************************************************
 * Name: composite_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
FAR const struct usb_qualdesc_s *composite_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif

#endif /* CONFIG_USBDEV_COMPOSITE */
