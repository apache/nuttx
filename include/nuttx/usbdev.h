/************************************************************************************
 * include/nuttx/usbdev.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef _INCLUDE_NUTTX_USBDEV_H
#define _INCLUDE_NUTTX_USBDEV_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Endpoint helpers *****************************************************************/

/* Configure endpoint, making it usable.  The class driver may deallocate or re-use
 * the 'desc' structure after returning
 */

#define EP_CONFIGURE(ep,desc) ep->ops->configure(ep,desc)

/* The endpoint will no longer be used */

#define EP_DISABLE(ep) ep->ops->disable(ep)

/* Allocate/free I/O requests */

#define EP_ALLOCREQ(ep,nytes) ep->ops->allocreq(ep)
#define EP_FREEREQ(ep,buff)   ep->ops->freereq(ep,req)

/* Allocate/free an I/O buffer */

#ifdef CONFIG_ARCH_USBDEV_DMA
#  define EP_ALLOCBUFFER(ep,nytes) ep->ops->alloc(ep,nbytes)
#  define EP_FREEBUFFER(ep,buff)   ep->ops->free(ep,buf)
#else
#  define EP_ALLOCBUFFER(ep,nytes) malloc(nbytes)
#  define EP_FREEBUFFER(ep,buff)   free(buf)
#endif

/* Submit an I/O request to the endpoint */

#define EP_SUBMIT(ep,req) ep->ops->submit(ep,req)

/* Cancel an I/O request previously sent to an endpoint */

#define EP_CANCEL(ep,req) ep->ops->cancel(ep,req)

/* Stall or resume an endpoint */

#define EP_STALL(ep) ep->ops->stall(ep,FALSE)
#define EP_RESUME(ep) ep->ops->stall(ep,TRUE)

/* USB Device Driver Helpers ********************************************************/

/* Allocate an endpoint:
 *
 *   epphy  - 7-bit physical endpoint number (without diretion bit).  Zero means
 *            that any endpoint matching the other requirements will suffice.
 *   in     - TRUE: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *            USB_EP_ATTR_XFER_INT}
 */

#define DEV_ALLOCEP(dev,epno,in,type) dev->ops->allocep(dev, epphy, in, type)

/* Release an endpoint */

#define DEV_FREEP(dev,epno,in) dev->ops->allocep(dev, ep)

/* Returns the current frame number */

#define DEV_GETFRAME(dev) dev->ops->getframe(dev)

/* Tries to wake up the host connected to this device */

#define DEV_WAKEUP(dev) dev->ops->wakeup(dev)

/* Sets the device selfpowered feature */

#define DEV_SETSELFPOWERED(dev) dev->ops->selfpowered(dev, TRUE)

/* Clears the device selfpowered feature */

#define DEV_CLRSELFPOWERED(dev) dev->ops->selfpowered(dev, FALSE)

/* Software-controlled connect to USB host */

#define DEV_CONNECT(dev) dev->ops->pullup ? dev->ops->pullup(dev, TRUE) : -EOPNOTSUPP

/* Software-controlled disconnect from USB host */

#define DEV_DISCONNECT(dev) dev->ops->pullup ? dev->ops->pullup(dev, FALSE) : -EOPNOTSUPP

/* USB Class Driver Helpsers ********************************************************/

/* Invoked when the driver is bound to a USB device driver */

#define CLASS_BIND(drvr,dev) (drvr)->ops.bind(dev)

/* Invoked when the driver is unbound from a USB device driver */

#define CLASS_UNBIND(drvr,dev) (drvr)->ops.unbind(dev)

/* Invoked after all transfers have been stopped, when the host is disconnected. */

#define CLASS_DISCONNECT(drvr.dev) (drvr)->ops->disconnect(dev)

/* Invoked for ep0 control requests */

#define CLASS_SETUP(drvr,dev,req) (drvr)->ops.setup(dev, req)

/* Invoked on USB suspend. */

#define CLASS_SUSPEND(drvr,dev) (drvr)->ops.suspend ? (drvr)->ops->suspend(dev) : (void)

/* Invoked on USB resume */

#define CLASS_RESUME(drvr,dev) (drvr)->ops.resume ? (drvr)->ops->resume(dev) : (void)

/* Device speeds */

#define USB_SPEED_UNKNOWN     0 /* Transfer rate not yet set */
#define USB_SPEED_LOW         1 /* USB 1.1 */
#define USB_SPEED_FULL        2 /* USB 1.1 */
#define USB_SPEED_HIGH        3 /* USB 2.0 */
#define USB_SPEED_VARIABLE    4 /* Wireless USB 2.5 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* USB Controller Structures ********************************************************/

/* struct usbdev_req_s - describes one i/o request */

struct usbdev_ep_s;
struct usbdev_req_s
{
  char   *buf;     /* Call: Buffer used for data; Return: Unchanged */
  uint16  len;     /* Call: Total length of data in buf; Return: Unchanged */
  uint16  xfrd;    /* Call: zero; Return: Bytes transferred so far */
  sint16  result;  /* Call: zero; Return: Result of transfer (O or -errno) */

  /* Callback when the transfer completes */

  void  (*callback)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);
  void   *private; /* Used only by callee */
};

/* Endpoint-specific interface to USB controller hardware. */

struct usbdev_epops_s
{
  /* Configure/enable and disable endpoint */

  int (*configure)(FAR struct usbdev_ep_s *ep, FAR const struct usb_epdesc_s *desc);
  int (*disable)(FAR struct usbdev_ep_s *ep);

  /* Allocate and free I/O requests */

  FAR struct usbdev_req_s *(*allocreq)(FAR struct usbdev_ep_s *ep);
  void (*freereq)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);

  /* Allocate and free I/O buffers */

#ifdef CONFIG_ARCH_USBDEV_DMA
  FAR void *(*allocbuffer)(FAR struct usbdev_ep_s *ep, uint16 nbytes);
  void (*freebuffer)(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif

  /* Submit and cancel I/O requests */

  int (*submit)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);
  int (*cancel)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);

  /* Stall or resume an endpoint */

  int (*stall)(FAR struct usbdev_ep_s *ep, boolean resume);
};

/* Representation of one USB endpoint */

struct usbdev_ep_s
{
  const struct usbdev_epops_s *ops; /* Endpoint operations */
  uint16 maxpacket;                 /* Maximum packet size for this endpoint */
  void  *private;                   /* For use by class driver */
};

/* struct usbdev_s represents a usb device */

struct usbdev_s;
struct usbdev_ops_s
{
  /* Allocate and free endpoints */

  FAR struct usbdev_ep_s *(*allocep)(FAR struct usbdev_s *dev, ubyte epphy, boolean in, ubyte eptype);
  void (*freeep)(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);

  /* Get the frame number from the last SOF */

  int (*getframe)(FAR struct usbdev_s *dev);

  /* Hardware specific features */

  int (*wakeup)(FAR struct usbdev_s *dev);
  int (*selfpowered)(FAR struct usbdev_s *dev, boolean selfpowered);
  int (*pullup)(FAR struct usbdev_s *dev,  boolean enable);

  /* Device-specific I/O command support */

  int (*ioctl)(FAR struct usbdev_s *dev, unsigned code, unsigned long param);
};

struct usbdev_s
{
  const struct usbdev_ops_s *ops; /* Access to hardware specific features */
  struct usbdev_ep_s *ep0;        /* Endpoint zero */
  ubyte  speed;                   /* Current speed of host connection */
  ubyte  dualspeed:1;             /* 1:supports high and full speed operation */
};

/* USB Device Class Implementations *************************************************/

struct usbdevclass_driverops_s
{
  int  (*bind)(FAR struct usbdev_s *dev, void *private);
  void (*unbind)(FAR struct usbdev_s *dev);
  int  (*setup)(FAR struct usbdev_s *dev, const struct usb_ctrlreq_s *req);
  void (*disconnect)(FAR struct usbdev_s *dev);
  void (*suspend)(FAR struct usbdev_s *dev);
  void (*resume)(FAR struct usbdev_s *dev);
};

struct usbdevclass_driver_s
{
  const struct usbdevclass_driverops_s ops;
  ubyte speed;                    /* Highest speed that the driver handles */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* USB Controller Structures ********************************************************/

/************************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ************************************************************************************/

EXTERN int usbdev_register(FAR struct usbdevclass_driver_s *driver);

/************************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 ************************************************************************************/

EXTERN int usbdev_unregister(FAR struct usbdevclass_driver_s *driver);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_USBDEV_H */
