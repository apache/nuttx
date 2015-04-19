/*******************************************************************************
 * drivers/usbhost/usbhost_enumerate.c
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hub.h>

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);

static void usbhost_callback(FAR struct usbhost_transfer_s *xfer);

static inline int usbhost_devdesc(const struct usb_devdesc_s *devdesc,
                                  FAR struct usbhost_id_s *id);
static inline int usbhost_configdesc(const uint8_t *configdesc, int desclen,
                                     FAR struct usbhost_id_s *id);
static inline int usbhost_classbind(FAR struct usbhost_class_s *devclass,
                                    const uint8_t *configdesc, int desclen,
                                    FAR struct usbhost_id_s *id);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

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
 *******************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_callback
 *
 * Description:
 *
 *
 *******************************************************************************/

static void usbhost_callback(FAR struct usbhost_transfer_s *xfer)
{
  sem_post(&xfer->done);
}

/*******************************************************************************
 * Name: usbhost_devdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 *******************************************************************************/

static inline int usbhost_devdesc(FAR const struct usb_devdesc_s *devdesc,
                                  FAR struct usbhost_id_s *id)
{
  /* Clear the ID info */

  memset(id, 0, sizeof(struct usbhost_id_s));

  /* Pick off the class ID info */

  id->base     = devdesc->classid;
  id->subclass = devdesc->subclass;
  id->proto    = devdesc->protocol;

  /* Pick off the VID and PID as well (for vendor specfic devices) */

  id->vid = usbhost_getle16(devdesc->vendor);
  id->pid = usbhost_getle16(devdesc->product);

  uvdbg("class:%d subclass:%04x protocol:%04x vid:%d pid:%d\n",
        id->base, id->subclass, id->proto, id->vid, id->pid);
  return OK;
}

/*******************************************************************************
 * Name: usbhost_configdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 *******************************************************************************/

static inline int usbhost_configdesc(const uint8_t *configdesc, int cfglen,
                                     struct usbhost_id_s *id)
{
  struct usb_cfgdesc_s *cfgdesc;
  struct usb_ifdesc_s *ifdesc;
  int remaining;

  DEBUGASSERT(configdesc != NULL && cfglen >= USB_SIZEOF_CFGDESC);

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (struct usb_cfgdesc_s *)configdesc;
  uvdbg("cfg len:%d total len:%d\n", cfgdesc->len, cfglen);

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
      if (ifdesc->type == USB_DESC_TYPE_INTERFACE)
        {
          /* Yes, extract the class information from the interface descriptor.
           * Typically these values are zero meaning that the "real" ID
           * information resides in the device descriptor.
           */

          DEBUGASSERT(remaining >= sizeof(struct usb_ifdesc_s));
          id->base     = ifdesc->classid;
          id->subclass = ifdesc->subclass;
          id->proto    = ifdesc->protocol;
          uvdbg("class:%d subclass:%d protocol:%d\n",
                id->base, id->subclass, id->proto);
          return OK;
        }

     /* Increment the address of the next descriptor */

      configdesc += ifdesc->len;
      remaining  -= ifdesc->len;
    }

  return -ENOENT;
}

/*******************************************************************************
 * Name: usbhost_classbind
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Try to
 *   bind this configuration descriptor with a supported class.
 *
 *******************************************************************************/

static inline int usbhost_classbind(FAR struct usbhost_class_s *devclass,
                                    const uint8_t *configdesc, int desclen,
                                    FAR struct usbhost_id_s *id)
{
  FAR const struct usbhost_registry_s *reg;
  int ret = -EINVAL;

  /* Is there is a class implementation registered to support this device. */

  reg = usbhost_findclass(id);
  uvdbg("usbhost_findclass: %p\n", reg);

  if (reg != NULL)
    {
      /* Yes.. there is a class for this device.  Get an instance of
       * its interface.
       */

      ret = CLASS_CREATE(reg, devclass, id);
      uvdbg("CLASS_CREATE: %p\n", devclass->priv);

      if (devclass->priv != NULL)
        {
          /* Then bind the newly instantiated class instance */

          ret = CLASS_CONNECT(devclass, configdesc, desclen);
          if (ret != OK)
            {
              /* On failures, call the class disconnect method which
               * should then free the allocated devclass instance.
               */

              udbg("CLASS_CONNECT failed: %d\n", ret);
              CLASS_DISCONNECTED(devclass);
            }
        }
    }

  uvdbg("Returning: %d\n", ret);
  return ret;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
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
 *   devclass - USB class information common across all classes. Whoever calls
 *     enumerate should fill address, speed, driver and parent class
 *     pointer. Enumeration will fill the control endpoint ep0,
 *     transaction translator (if applicable) and private data
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

int usbhost_enumerate(FAR struct usbhost_class_s *devclass)
{
  FAR struct usb_ctrlreq_s *ctrlreq = NULL;
  struct usbhost_devinfo_s devinfo;
  struct usbhost_id_s id;
  size_t maxlen;
  unsigned int cfglen;
  uint8_t maxpacketsize;
  uint8_t descsize;
  FAR uint8_t *buffer = NULL;
  int  ret;

  DEBUGASSERT(devclass != NULL && devclass->drvr != NULL);

  /* Allocate descriptor buffers for use in this function.  We will need two:
   * One for the request and one for the data buffer.
   */

  ret = DRVR_ALLOC(devclass->drvr, (FAR uint8_t **)&ctrlreq, &maxlen);
  if (ret != OK)
    {
      udbg("DRVR_ALLOC failed: %d\n", ret);
      return ret;
    }

  ret = DRVR_ALLOC(devclass->drvr, &buffer, &maxlen);
  if (ret != OK)
    {
      udbg("DRVR_ALLOC failed: %d\n", ret);
      goto errout;
    }

  /* Get information about the connected device */

  ret = DRVR_GETDEVINFO(devclass->drvr, &devinfo);
  if (ret != OK)
    {
      udbg("DRVR_GETDEVINFO failed: %d\n", ret);
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

  if (devinfo.speed == DEVINFO_SPEED_HIGH)
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

  DRVR_EP0CONFIGURE(devclass->drvr, devclass->ep0, 0, maxpacketsize);

  /* Read first bytes of the device descriptor */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_DEVICE << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, descsize);

  ret = usbhost_ctrlxfer(devclass, ctrlreq, buffer);
  if (ret != OK)
    {
      udbg("ERROR: Failed to get device descriptor, length=%d: %d\n",
           descsize, ret);
      goto errout;
    }

  /* Extract the correct max packetsize from the device descriptor */

  maxpacketsize = ((struct usb_devdesc_s *)buffer)->mxpacketsize;
  uvdbg("maxpacksetsize: %d\n", maxpacketsize);

  /* And reconfigure EP0 with the correct maximum packet size */

  DRVR_EP0CONFIGURE(devclass->drvr, devclass->ep0, 0, maxpacketsize);

  /* Set the USB device address */

  ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_SETADDRESS;
  usbhost_putle16(ctrlreq->value, ((uint16_t)devclass->addr << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, 0);

  ret = usbhost_ctrlxfer(devclass, ctrlreq, NULL);
  if (ret != OK)
    {
      udbg("ERROR: Failed to set address: %d\n");
      goto errout;
    }

  usleep(2*1000);

   /* And reconfigure EP0 with the correct address */

  DRVR_EP0CONFIGURE(devclass->drvr, devclass->ep0, devclass->addr,
                    maxpacketsize);

  /* Now read the full device descriptor (if we have not already done so) */

  if (descsize < USB_SIZEOF_DEVDESC)
    {
      ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
      ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
      usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_DEVICE << 8));
      usbhost_putle16(ctrlreq->index, 0);
      usbhost_putle16(ctrlreq->len, USB_SIZEOF_DEVDESC);

      ret = usbhost_ctrlxfer(devclass, ctrlreq, buffer);
      if (ret != OK)
        {
          udbg("ERROR: Failed to get device descriptor, length=%d: %d\n",
               USB_SIZEOF_DEVDESC, ret);
          goto errout;
        }
    }

  /* Get class identification information from the device descriptor.  Most
   * devices set this to USB_CLASS_PER_INTERFACE (zero) and provide the
   * identification information in the interface descriptor(s).  That allows
   * a device to support multiple, different classes.
   */

  (void)usbhost_devdesc((struct usb_devdesc_s *)buffer, &id);

 /* Get the configuration descriptor (only), index == 0.  Should not be
  * hard-coded! More logic is needed in order to handle devices with
  * multiple configurations.
  */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_CONFIG << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, USB_SIZEOF_CFGDESC);

  ret = usbhost_ctrlxfer(devclass, ctrlreq, buffer);
  if (ret != OK)
   {
      udbg("ERROR: Failed to get configuration descriptor, length=%d: %d\n",
           USB_SIZEOF_CFGDESC, ret);
      goto errout;
    }

  /* Extract the full size of the configuration data */

  cfglen = (unsigned int)usbhost_getle16(((struct usb_cfgdesc_s *)buffer)->totallen);
  uvdbg("sizeof config data: %d\n", cfglen);

  /* Get all of the configuration descriptor data, index == 0 (Should not be
   * hard-coded!)
   */

  ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_GETDESCRIPTOR;
  usbhost_putle16(ctrlreq->value, (USB_DESC_TYPE_CONFIG << 8));
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, cfglen);

  ret = usbhost_ctrlxfer(devclass, ctrlreq, buffer);
  if (ret != OK)
    {
      udbg("ERROR: Failed to get configuration descriptor, length=%d: %d\n",
           cfglen, ret);
      goto errout;
    }

  /* Select device configuration 1 (Should not be hard-coded!) */

  ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE;
  ctrlreq->req  = USB_REQ_SETCONFIGURATION;
  usbhost_putle16(ctrlreq->value, 1);
  usbhost_putle16(ctrlreq->index, 0);
  usbhost_putle16(ctrlreq->len, 0);

  ret = usbhost_ctrlxfer(devclass, ctrlreq, NULL);
  if (ret != OK)
    {
      udbg("ERROR: Failed to set configuration: %d\n", ret);
      goto errout;
    }

  /* Was the class identification information provided in the device
   * descriptor? Or do we need to find it in the interface descriptor(s)?
   */

  if (id.base == USB_CLASS_PER_INTERFACE)
    {
      /* Get the class identification information for this device from the
       * interface descriptor(s).  Hmmm.. More logic is need to handle the
       * case of multiple interface descriptors.
       */

      ret = usbhost_configdesc(buffer, cfglen, &id);
      if (ret != OK)
        {
          udbg("ERROR: Failed to read class identification: %d\n", ret);
          goto errout;
        }
    }

  /* Some devices may require some delay before initialization */

  usleep(100*1000);

  /* Parse the configuration descriptor and bind to the class instance for the
   * device.  This needs to be the last thing done because the class driver
   * will begin configuring the device.
   */

  ret = usbhost_classbind(devclass, buffer, cfglen, &id);
  if (ret != OK)
    {
      udbg("ERROR: Failed to bind class: %d\n", ret);
    }

errout:
  if (buffer != NULL)
    {
      DRVR_FREE(devclass->drvr, buffer);
    }

  if (ctrlreq)
    {
      DRVR_FREE(devclass->drvr, (FAR uint8_t *)ctrlreq);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_ctrlxfer
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   devclass - A reference to the class instance.
 *   ctrlreq - Describes the control request transfer
 *   buffer - Data accompanying the control transfer
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int usbhost_ctrlxfer(FAR struct usbhost_class_s *devclass,
                     FAR struct usb_ctrlreq_s *ctrlreq,
                     FAR uint8_t *buffer)
{
  struct usbhost_transfer_s xfer;
  struct timespec timeout;
  uint16_t len;
  int ret;

  len           = usbhost_getle16(ctrlreq->len);
  xfer.buffer   = buffer;
  xfer.buflen   = len;
  xfer.len      = len;
  xfer.status   = -EIO;
  xfer.devclass = devclass;
  xfer.ep       = devclass->ep0;
  xfer.callback = usbhost_callback;

  sem_init(&xfer.done, 0, 0);

  if (ROOTHUB(devclass))
    {
      ret = DRVR_RHCTRL(devclass->drvr, &xfer, ctrlreq);
    }
  else
    {
      if ((ctrlreq->type & USB_REQ_DIR_IN) != 0)
        {
          ret = DRVR_CTRLIN(devclass->drvr, &xfer, ctrlreq);
        }
      else
        {
          ret = DRVR_CTRLOUT(devclass->drvr, &xfer, ctrlreq);
        }
    }

  if (ret != OK)
    {
      goto out;
    }

  timeout.tv_sec  = 5;
  timeout.tv_nsec = 1000*1000;

  ret = sem_timedwait(&xfer.done, &timeout);
  if (ret == OK)
    {
      ret = xfer.status;
    }

out:
  sem_destroy(&xfer.done);

  return ret;
}
