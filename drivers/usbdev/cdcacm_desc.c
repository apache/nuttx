/****************************************************************************
 * drivers/usbdev/cdcacm_desc.c
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB descriptor templates these will be copied and modified **************/
/* Device Descriptor.  If the USB serial device is configured as part of
 * composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_CDC,                                /* class */
  CDC_SUBCLASS_NONE,                            /* subclass */
  CDC_PROTO_NONE,                               /* protocol */
  CONFIG_CDCACM_EP0MAXPACKET,                   /* maxpacketsize */
  {
    LSBYTE(CONFIG_CDCACM_VENDORID),             /* vendor */
    MSBYTE(CONFIG_CDCACM_VENDORID)
  },
  {
    LSBYTE(CONFIG_CDCACM_PRODUCTID),            /* product */
    MSBYTE(CONFIG_CDCACM_PRODUCTID)
  },
  {
    LSBYTE(CDCACM_VERSIONNO),                   /* device */
    MSBYTE(CDCACM_VERSIONNO)
  },
  CDCACM_MANUFACTURERSTRID,                     /* imfgr */
  CDCACM_PRODUCTSTRID,                          /* iproduct */
  CDCACM_SERIALSTRID,                           /* serno */
  CDCACM_NCONFIGS                               /* nconfigs */
};
#endif

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
     LSBYTE(0x0200),
     MSBYTE(0x0200)
  },
  USB_CLASS_VENDOR_SPEC,                        /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_CDCACM_EP0MAXPACKET,                   /* mxpacketsize */
  CDCACM_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int cdcacm_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
#if !defined(CONFIG_CDCACM_COMPOSITE) || defined(CONFIG_CDCACM_NOTIFSTR) || \
     defined(CONFIG_CDCACM_DATAIFSTR)

  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_CDCACM_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(CDCACM_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(CDCACM_STR_LANGUAGE);
        return 4;
      }

    case CDCACM_MANUFACTURERSTRID:
      str = CONFIG_CDCACM_VENDORSTR;
      break;

    case CDCACM_PRODUCTSTRID:
      str = CONFIG_CDCACM_PRODUCTSTR;
      break;

    case CDCACM_SERIALSTRID:
      str = CONFIG_CDCACM_SERIALSTR;
      break;

    case CDCACM_CONFIGSTRID:
      str = CONFIG_CDCACM_CONFIGSTR;
      break;
#endif

#ifdef CONFIG_CDCACM_NOTIFSTR
    case CDCACM_NOTIFSTRID:
      str = CONFIG_CDCACM_NOTIFSTR;
      break;
#endif

#ifdef CONFIG_CDCACM_DATAIFSTR
    case CDCACM_DATAIFSTRID:
      str = CONFIG_CDCACM_DATAIFSTR;
      break;
#endif

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   if (len > (CDCACM_MAXSTRLEN / 2))
     {
       len = (CDCACM_MAXSTRLEN / 2);
     }

   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
#else
   return -EINVAL;
#endif
}

/****************************************************************************
 * Name: cdcacm_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
FAR const struct usb_devdesc_s *cdcacm_getdevdesc(void)
{
  return &g_devdesc;
}
#endif

/****************************************************************************
 * Name: cdcacm_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in (sizeof(struct usb_epdesc_s)).
 *
 ****************************************************************************/

int cdcacm_copy_epdesc(enum cdcacm_epdesc_e epid,
                       FAR struct usb_epdesc_s *epdesc,
                       FAR struct usbdev_devinfo_s *devinfo,
                       bool hispeed)
{
#ifndef CONFIG_USBDEV_DUALSPEED
    UNUSED(hispeed);
#endif

    switch (epid)
    {
    case CDCACM_EPINTIN:  /* Interrupt IN endpoint */
        {
          epdesc->len  = USB_SIZEOF_EPDESC;            /* Descriptor length */
          epdesc->type = USB_DESC_TYPE_ENDPOINT;       /* Descriptor type */
          epdesc->addr = CDCACM_MKEPINTIN(devinfo);    /* Endpoint address */
          epdesc->attr = CDCACM_EPINTIN_ATTR;          /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
          if (hispeed)
            {
              /* Maximum packet size (high speed) */

              epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPINTIN_HSSIZE);
              epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPINTIN_HSSIZE);
            }
          else
#endif
            {
              /* Maximum packet size (full speed) */

              epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPINTIN_FSSIZE);
              epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPINTIN_FSSIZE);
            }

          epdesc->interval = 10;                       /* Interval */
      }
      break;

    case CDCACM_EPBULKOUT:  /* Bulk OUT endpoint */
      {
        epdesc->len = USB_SIZEOF_EPDESC;             /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;       /* Descriptor type */
        epdesc->addr = CDCACM_MKEPBULKOUT(devinfo);  /* Endpoint address */
        epdesc->attr = CDCACM_EPOUTBULK_ATTR;        /* Endpoint attributes */
#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPBULKOUT_HSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPBULKOUT_HSSIZE);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPBULKOUT_FSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPBULKOUT_FSSIZE);
          }

        epdesc->interval = 1;                        /* Interval */
      }
      break;

    case CDCACM_EPBULKIN:  /* Bulk IN endpoint */
      {
        epdesc->len  = USB_SIZEOF_EPDESC;            /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;       /* Descriptor type */
        epdesc->addr = CDCACM_MKEPBULKIN(devinfo);   /* Endpoint address */
        epdesc->attr = CDCACM_EPINBULK_ATTR;         /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPBULKIN_HSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPBULKIN_HSSIZE);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_CDCACM_EPBULKIN_FSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_CDCACM_EPBULKIN_FSSIZE);
          }

        epdesc->interval = 1;                        /* Interval */
      }
      break;

    default:
      return 0;
    }

  return sizeof(struct usb_epdesc_s);
}

/****************************************************************************
 * Name: cdcacm_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type)
#else
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  int length = 0;
  bool hispeed = false;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  /* Fill in all descriptors directly to the buf */

  /* Configuration Descriptor.  If the serial device is used in as part
   * or a composite device, then the configuration descriptor is
   * provided by the composite device logic.
   */

#if !defined(CONFIG_CDCACM_COMPOSITE)
  if (buf != NULL)
    {
      /* Configuration descriptor.  If the USB serial device is configured as part of
       * composite device, then the configuration descriptor will be provided by the
       * composite device logic.
       */

      FAR struct usb_cfgdesc_s *dest = (FAR struct usb_cfgdesc_s *)buf;

      /* Let's calculate the size... */

#ifdef CONFIG_USBDEV_DUALSPEED
      int16_t size = cdcacm_mkcfgdesc(NULL, NULL, speed, type);
#else
      int16_t size = cdcacm_mkcfgdesc(NULL, NULL);
#endif

      dest->len         = USB_SIZEOF_CFGDESC;                /* Descriptor length */
      dest->type        = USB_DESC_TYPE_CONFIG;              /* Descriptor type */
      dest->totallen[0] = LSBYTE(size);                      /* LS Total length */
      dest->totallen[1] = MSBYTE(size);                      /* MS Total length */
      dest->ninterfaces = CDCACM_NINTERFACES;                /* Number of interfaces */
      dest->cfgvalue    = CDCACM_CONFIGID;                   /* Configuration value */
      dest->icfg        = CDCACM_CONFIGSTRID;                /* Configuration */
      dest->attr        = USB_CONFIG_ATTR_ONE |              /* Attributes */
                          CDCACM_SELFPOWERED |
                          CDCACM_REMOTEWAKEUP;
      dest->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2;  /* Max power (mA/2) */

      buf += sizeof(struct usb_cfgdesc_s);
    }

  length += sizeof(struct usb_cfgdesc_s);

  /* If the serial device is part of a composite device, then it should
   * begin with an interface association descriptor (IAD) because the
   * CDC/ACM device consists of more than one interface. The IAD associates
   * the two CDC/ACM interfaces with the same CDC/ACM device.
   */

#elif defined(CONFIG_COMPOSITE_IAD)
  /* Interface association descriptor */

  if (buf != NULL)
    {
      FAR struct usb_iaddesc_s *dest = (FAR struct usb_iaddesc_s *)buf;

      dest->len       = USB_SIZEOF_IADDESC;                  /* Descriptor length */
      dest->type      = USB_DESC_TYPE_INTERFACEASSOCIATION;  /* Descriptor type */
      dest->firstif   = devinfo->ifnobase;                   /* Number of first interface of the function */
      dest->nifs      = devinfo->ninterfaces;                /* Number of interfaces associated with the function */
      dest->classid   = USB_CLASS_CDC;                       /* Class code */
      dest->subclass  = CDC_SUBCLASS_ACM;                    /* Sub-class code */
      dest->protocol  = CDC_PROTO_NONE;                      /* Protocol code */
      dest->ifunction = 0;                                   /* Index to string identifying the function */

      buf += sizeof(struct usb_iaddesc_s);
    }

  length += sizeof(struct usb_iaddesc_s);
#endif

  /* Notification interface */

  if (buf != NULL)
    {
      FAR struct usb_ifdesc_s *dest = (FAR struct usb_ifdesc_s *)buf;

      dest->len      = USB_SIZEOF_IFDESC;                    /* Descriptor length */
      dest->type     = USB_DESC_TYPE_INTERFACE;              /* Descriptor type */
      dest->ifno     = devinfo->ifnobase;                    /* Interface number */
      dest->alt      = CDCACM_NOTALTIFID;                    /* Alternate setting */
      dest->neps     = 1;                                    /* Number of endpoints */
      dest->classid  = USB_CLASS_CDC;                        /* Interface class */
      dest->subclass = CDC_SUBCLASS_ACM;                     /* Interface sub-class */
      dest->protocol = CDC_PROTO_ATM;                        /* Interface protocol */
#ifdef CONFIG_CDCACM_NOTIFSTR
      dest->iif      = devinfo->strbase + CDCACM_NOTIFSTRID; /* iInterface */
#else
      dest->iif      = 0;                                    /* iInterface */
#endif

      buf += sizeof(struct usb_ifdesc_s);
    }

  length += sizeof(struct usb_ifdesc_s);

  /* Header functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_hdr_funcdesc_s *dest = (FAR struct cdc_hdr_funcdesc_s *)buf;

      dest->size    = SIZEOF_HDR_FUNCDESC;                   /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;             /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_HDR;                      /* Descriptor sub-type */
      dest->cdc[0]  = LSBYTE(CDC_VERSIONNO);                 /* CDC release number in BCD */
      dest->cdc[1]  = MSBYTE(CDC_VERSIONNO);

      buf += sizeof(struct cdc_hdr_funcdesc_s);
    }

  length += sizeof(struct cdc_hdr_funcdesc_s);

  /* ACM functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_acm_funcdesc_s *dest = (FAR struct cdc_acm_funcdesc_s *)buf;

      dest->size    = SIZEOF_ACM_FUNCDESC;                   /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;             /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_ACM;                      /* Descriptor sub-type */
      dest->caps    = 0x06;                                  /* Bit encoded capabilities */

      buf += sizeof(struct cdc_acm_funcdesc_s);
    }

  length += sizeof(struct cdc_acm_funcdesc_s);

  /* This codeblock is just for future use - currently we didn't need it */

#ifdef OPTIONAL_UNION_FUNCTIONAL_DESCRIPTOR
  /* Union functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_union_funcdesc_s *dest = (FAR struct cdc_union_funcdesc_s *)buf;

      dest->size     = SIZEOF_UNION_FUNCDESC(1);              /* Descriptor length */
      dest->type     = USB_DESC_TYPE_CSINTERFACE;             /* Descriptor type */
      dest->subtype  = CDC_DSUBTYPE_UNION;                    /* Descriptor sub-type */
      dest->master   = devinfo->ifnobase;                     /* Master interface number */
      dest->slave[0] = devinfo->ifnobase + 1;                 /* Slave[0] interface number */

      buf += sizeof(struct cdc_union_funcdesc_s);
    }

  length += sizeof(struct cdc_union_funcdesc_s);
#endif

  /* Call Management functional descriptor */

  if (buf != NULL)
    {
      FAR struct cdc_callmgmt_funcdesc_s *dest = (FAR struct cdc_callmgmt_funcdesc_s *)buf;

      dest->size    = SIZEOF_CALLMGMT_FUNCDESC;               /* Descriptor length */
      dest->type    = USB_DESC_TYPE_CSINTERFACE;              /* Descriptor type */
      dest->subtype = CDC_DSUBTYPE_CALLMGMT;                  /* Descriptor sub-type */
      dest->caps    = 3;                                      /* Bit encoded capabilities */
      dest->ifno    = devinfo->ifnobase + 1;                  /* Interface number of Data Class interface */

      buf += sizeof(struct cdc_callmgmt_funcdesc_s);
    }

  length += sizeof(struct cdc_callmgmt_funcdesc_s);

  /* Interrupt IN endpoint descriptor */

  if (buf != NULL)
    {
      cdcacm_copy_epdesc(CDCACM_EPINTIN, (struct usb_epdesc_s *)buf, devinfo, hispeed);

      buf += USB_SIZEOF_EPDESC;
    }

  length += USB_SIZEOF_EPDESC;

  /* Data interface descriptor */

  if (buf != NULL)
    {
      FAR struct usb_ifdesc_s *dest = (FAR struct usb_ifdesc_s *)buf;

      dest->len      = USB_SIZEOF_IFDESC;                     /* Descriptor length */
      dest->type     = USB_DESC_TYPE_INTERFACE;               /* Descriptor type */
      dest->ifno     = devinfo->ifnobase + 1;                 /* Interface number */
      dest->alt      = CDCACM_DATAALTIFID;                    /* Alternate setting */
      dest->neps     = 2;                                     /* Number of endpoints */
      dest->classid  = USB_CLASS_CDC_DATA;                    /* Interface class */
      dest->subclass = CDC_DATA_SUBCLASS_NONE;                /* Interface sub-class */
      dest->protocol = CDC_DATA_PROTO_NONE;                   /* Interface protocol */
#ifdef CONFIG_CDCACM_DATAIFSTR
      dest->iif      = devinfo->strbase + CDCACM_DATAIFSTRID; /* iInterface */
#else
      dest->iif      = 0;                                     /* iInterface */
#endif

      buf += sizeof(struct usb_ifdesc_s);
    }

  length += sizeof(struct usb_ifdesc_s);

  /* Bulk OUT endpoint descriptor */

  if (buf != NULL)
    {
      cdcacm_copy_epdesc(CDCACM_EPBULKOUT, (struct usb_epdesc_s *)buf, devinfo, hispeed);
      buf += USB_SIZEOF_EPDESC;
    }

  length += USB_SIZEOF_EPDESC;

  /* Bulk IN endpoint descriptor */

  if (buf != NULL)
    {
      cdcacm_copy_epdesc(CDCACM_EPBULKIN, (struct usb_epdesc_s *)buf, devinfo, hispeed);
      buf += USB_SIZEOF_EPDESC;
    }

  length += USB_SIZEOF_EPDESC;
  return length;
}

/****************************************************************************
 * Name: cdcacm_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *cdcacm_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif
