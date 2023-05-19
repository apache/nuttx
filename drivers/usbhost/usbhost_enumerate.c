/****************************************************************************
 * drivers/usbhost/usbhost_enumerate.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hub.h>
#include <nuttx/usb/usbhost_devaddr.h>

#include "usbhost_composite.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);

static inline int usbhost_devdesc(const struct usb_devdesc_s *devdesc,
              FAR struct usbhost_id_s *id);
static inline int usbhost_configdesc(const uint8_t *configdesc, int desclen,
                                     uint8_t start_ifnum,
                                     uint8_t *ret_ifnum,
                                     struct usbhost_id_s *id);
static inline int usbhost_classbind(FAR struct usbhost_hubport_s *hport,
              FAR const uint8_t *configdesc, int desclen,
              FAR struct usbhost_id_s *id,
              FAR struct usbhost_class_s **devclass);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
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
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_devdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 ****************************************************************************/

static inline int usbhost_devdesc(FAR const struct usb_devdesc_s *devdesc,
                                  FAR struct usbhost_id_s *id)
{
  /* Clear the ID info */

  memset(id, 0, sizeof(struct usbhost_id_s));

  /* Pick off the class ID info */

  id->base     = devdesc->classid;
  id->subclass = devdesc->subclass;
  id->proto    = devdesc->protocol;

  /* Pick off the VID and PID as well (for vendor specific devices) */

  id->vid = usbhost_getle16(devdesc->vendor);
  id->pid = usbhost_getle16(devdesc->product);

  uinfo("class:%d subclass:%d protocol:%d vid:%04x pid:%04x\n",
        id->base, id->subclass, id->proto, id->vid, id->pid);
  return OK;
}

/****************************************************************************
 * Name: usbhost_configdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 ****************************************************************************/

static inline int usbhost_configdesc(const uint8_t *configdesc, int cfglen,
                                     uint8_t start_ifnum,
                                     uint8_t *ret_ifnum,
                                     struct usbhost_id_s *id)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_ifdesc_s *ifdesc;
  int remaining;

  DEBUGASSERT(configdesc != NULL && cfglen >= USB_SIZEOF_CFGDESC);

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (struct usb_cfgdesc_s *)configdesc;
  uinfo("cfg len:%d total len:%d\n", cfgdesc->len, cfglen);

  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining   = cfglen - cfgdesc->len;

  /* Loop while there are more descriptors to examine */

  memset(id, 0, sizeof(FAR struct usb_desc_s));
  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? Is it an interface descriptor? */

      ifdesc = (struct usb_ifdesc_s *)configdesc;
      if (ifdesc->type == USB_DESC_TYPE_INTERFACE &&
          ifdesc->ifno >= start_ifnum)
        {
          /* Yes, extract the class information from the interface
           * descriptor.
           * Typically these values are zero meaning that the "real" ID
           * information resides in the device descriptor.
           */

          DEBUGASSERT(remaining >= sizeof(struct usb_ifdesc_s));
          id->base     = ifdesc->classid;
          id->subclass = ifdesc->subclass;
          id->proto    = ifdesc->protocol;
          uinfo("class:%d subclass:%d protocol:%d\n",
                id->base, id->subclass, id->proto);
          *ret_ifnum = ifdesc->ifno;
          return OK;
        }

      /* Increment the address of the next descriptor */

      configdesc += ifdesc->len;
      remaining  -= ifdesc->len;
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: usbhost_classbind
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Try to
 *   bind this configuration descriptor with a supported class.
 *
 ****************************************************************************/

static inline int usbhost_classbind(FAR struct usbhost_hubport_s *hport,
                                    const uint8_t *configdesc, int desclen,
                                    struct usbhost_id_s *id,
                                    FAR struct usbhost_class_s **usbclass)
{
  FAR struct usbhost_class_s *devclass;
  FAR const struct usbhost_registry_s *reg;
  int ret = -EINVAL;

  /* Is there is a class implementation registered to support this device. */

  reg = usbhost_findclass(id);
  uinfo("usbhost_findclass: %p\n", reg);
  if (reg != NULL)
    {
      /* Yes.. there is a class for this device.  Get an instance of
       * its interface.
       */

      ret = -ENOMEM;
      devclass = CLASS_CREATE(reg, hport, id);
      uinfo("CLASS_CREATE: %p\n", devclass);
      if (devclass != NULL)
        {
          /* Then bind the newly instantiated class instance */

          ret = CLASS_CONNECT(devclass, configdesc, desclen);
          if (ret < 0)
            {
              /* On failures, call the class disconnect method which
               * should then free the allocated devclass instance.
               */

              uerr("ERROR: CLASS_CONNECT failed: %d\n", ret);
              CLASS_DISCONNECTED(devclass);
            }
          else
            {
              *usbclass = devclass;
            }
        }
    }

  uinfo("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   hport - The hub port that manages the new class.
 *   devclass - If the class driver for the device is successful located
 *      and bound to the hub port, the allocated class instance is returned
 *      into this caller-provided memory location.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

int usbhost_enumerate(FAR struct usbhost_hubport_s *hport,
                      FAR struct usbhost_class_s **devclass)
{
  FAR struct usb_ctrlreq_s *ctrlreq = NULL;
  struct usbhost_id_s id;
  size_t maxlen;
  unsigned int cfglen;
  uint8_t maxpacketsize;
  uint8_t descsize;
  uint8_t funcaddr = 0;
  FAR uint8_t *buffer = NULL;
  int ret;

  DEBUGASSERT(hport != NULL && hport->drvr != NULL);

  /* Allocate descriptor buffers for use in this function.  We will need two:
   * One for the request and one for the data buffer.
   */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC failed: %d\n", ret);
      return ret;
    }

  ret = DRVR_ALLOC(hport->drvr, &buffer, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC failed: %d\n", ret);
      goto errout;
    }

  /* Pick an appropriate packet size for this device
   *
   * USB 2.0, Paragraph 5.5.3 "Control Transfer Packet Size Constraints"
   *
   *  "An endpoint for control transfers specifies the maximum data
   *   payload size that the endpoint can accept from or transmit to
   *   the bus. The allowable maximum control transfer data payload
   *   sizes for full-speed devices is 8, 16, 32, or 64 bytes; for
   *   high-speed devices, it is 64 bytes and for low-speed devices,
   *   it is 8 bytes. This maximum applies to the data payloads of the
   *   Data packets following a Setup..."
   */

  if (hport->speed == USB_SPEED_HIGH)
    {
      /* For high-speed, we must use 64 bytes */

      maxpacketsize = 64;
      descsize      = USB_SIZEOF_DEVDESC;
    }
  else
    {
      /* Eight will work for both low- and full-speed */

      maxpacketsize = 8;
      descsize      = 8;
    }

  /* Configure EP0 with the initial maximum packet size */

  DRVR_EP0CONFIGURE(hport->drvr, hport->ep0, 0, hport->speed,
                    maxpacketsize);

  /* Read first bytes of the device descriptor */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_DEVICE << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, descsize);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, buffer);
  if (ret < 0)
    {
      uerr("ERROR: Failed to get device descriptor, length=%d: %d\n",
           descsize, ret);
      goto errout;
    }

  /* Extract the correct max packetsize from the device descriptor */

  maxpacketsize = ((struct usb_devdesc_s *)buffer)->mxpacketsize;
  uinfo("maxpacksetsize: %d\n", maxpacketsize);

  /* And reconfigure EP0 with the correct maximum packet size */

  DRVR_EP0CONFIGURE(hport->drvr, hport->ep0, 0, hport->speed,
                    maxpacketsize);

  /* Now read the full device descriptor (if we have not already done so) */

  if (descsize < USB_SIZEOF_DEVDESC)
    {
      ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
      ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
      usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_DEVICE << 8));
      usbhost_putle16(ctrlreq->index, 0);
      usbhost_putle16(ctrlreq->len, USB_SIZEOF_DEVDESC);

      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, buffer);
      if (ret < 0)
        {
          uerr("ERROR: Failed to get device descriptor, length=%d: %d\n",
               USB_SIZEOF_DEVDESC, ret);
          goto errout;
        }
    }

  /* Get class identification information from the device descriptor.  Most
   * devices set this to USB_CLASS_PER_INTERFACE (zero) and provide the
   * identification information in the interface descriptor(s).  That allows
   * a device to support multiple, different classes.
   */

  usbhost_devdesc((struct usb_devdesc_s *)buffer, &id);

  /* Assign a function address to the device connected to this port */

  funcaddr = usbhost_devaddr_create(hport);
  if (funcaddr < 0)
    {
      uerr("ERROR: usbhost_devaddr_create failed: %d\n", ret);
      goto errout;
    }

  /* Set the USB device address */

  ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_SETADDRESS;
  usbhost_putle16(ctrlreq->value, (uint16_t)funcaddr);
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, 0);

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to set address: %d\n", ret);
      goto errout;
    }

  nxsig_usleep(2 * 1000);

  /* Assign the function address to the port */

  DEBUGASSERT(hport->funcaddr == 0 && funcaddr != 0);
  hport->funcaddr = funcaddr;

  /* And reconfigure EP0 with the correct address */

  DRVR_EP0CONFIGURE(hport->drvr, hport->ep0, hport->funcaddr,
                    hport->speed, maxpacketsize);

  /* Get the configuration descriptor (only), index == 0.  Should not be
   * hard-coded! More logic is needed in order to handle devices with
   * multiple configurations.
   */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_CONFIG << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, USB_SIZEOF_CFGDESC);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, buffer);
  if (ret < 0)
    {
      uerr("ERROR: Failed to get configuration descriptor, length=%d: %d\n",
           USB_SIZEOF_CFGDESC, ret);
      goto errout;
    }

  /* Extract the full size of the configuration data */

  cfglen = (unsigned int)
           usbhost_getle16(((struct usb_cfgdesc_s *)buffer)->totallen);
  uinfo("sizeof config data: %d\n", cfglen);

  if (cfglen > maxlen)
    {
      uerr("ERROR: Configuration doesn't fit in buffer, "
           "length=%d, maxlen=%zu\n",
           cfglen, maxlen);
      ret = -E2BIG;
      goto errout;
    }

  /* Get all of the configuration descriptor data, index == 0 (Should not be
   * hard-coded!)
   */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_CONFIG << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, cfglen);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, buffer);
  if (ret < 0)
    {
      uerr("ERROR: Failed to get configuration descriptor, length=%d: %d\n",
           cfglen, ret);
      goto errout;
    }

  /* Select device configuration 1 (Should not be hard-coded!) */

  ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_SETCONFIGURATION;
  usbhost_putle16(ctrlreq->value, 1);
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, 0);

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to set configuration: %d\n", ret);
      goto errout;
    }

  /* Some devices may require some delay before initialization */

  nxsig_usleep(100 * 1000);

  /* Was the class identification information provided in the device
   * descriptor? Or do we need to find it in the interface descriptor(s)?
   */

  if (id.base == USB_CLASS_PER_INTERFACE)
    {
      /* Get the class identification information for this device from the
       * interface descriptor(s).  Hmmm.. More logic is need to handle the
       * case of multiple interface descriptors.
       */

      uint8_t ninterfaces = ((struct usb_cfgdesc_s *)buffer)->ninterfaces;
      uint8_t ifnum = 0;

      for (ifnum = 0; ifnum < ninterfaces; ifnum++)
        {
          uinfo("Parsing interface: %d\n", ifnum);
          ret = usbhost_configdesc(buffer, cfglen, ifnum, &ifnum, &id);
          if (ret < 0)
            {
              uerr("ERROR: usbhost_configdesc failed: %d\n", ret);
              goto errout;
            }

          ret = usbhost_classbind(hport, buffer, cfglen, &id, devclass);
          if (ret < 0)
            {
              uerr("ERROR: usbhost_classbind failed %d\n", ret);
            }

          ret = OK;
        }
    }
  else
    {
#ifdef CONFIG_USBHOST_COMPOSITE
      /* Check if the device attached to the downstream port if a USB
       * composite device and, if so, create the composite device wrapper
       * and bind it to the HCD.
       *
       * usbhost_composite() will return a negated errno value is on any
       * failure.  The value -ENOENT, in particular means that the attached
       * device is not a composite device.  Other values would indicate other
       * various, unexpected failures.  We make no real distinction here.
       */

      ret = usbhost_composite(hport, buffer, cfglen, &id, devclass);
      if (ret >= 0)
        {
          uinfo("usbhost_composite has bound the composite device\n");
        }

      /* Apparently this is not a composite device */

      else
#endif
        {
          /* Parse the configuration descriptor and bind to the class
           * instance for the device.  This needs to be the last thing
           * done because the class driver will begin configuring the
           * device.
           */

          ret = usbhost_classbind(hport, buffer, cfglen, &id, devclass);
          if (ret < 0)
            {
              uerr("ERROR: usbhost_classbind failed %d\n", ret);
            }
        }
    }

errout:
  if (ret < 0)
    {
      /* Release the device function address on any failure */

      usbhost_devaddr_destroy(hport, funcaddr);
      hport->funcaddr = 0;
    }

  /* Release temporary buffers in any event */

  if (buffer != NULL)
    {
      DRVR_FREE(hport->drvr, buffer);
    }

  if (ctrlreq)
    {
      DRVR_FREE(hport->drvr, (FAR uint8_t *)ctrlreq);
    }

  return ret;
}
