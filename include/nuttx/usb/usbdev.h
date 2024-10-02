/****************************************************************************
 * include/nuttx/usb/usbdev.h
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

#ifndef __INCLUDE_NUTTX_USB_USBDEV_H
#define __INCLUDE_NUTTX_USB_USBDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/usb/pl2303.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Endpoint helpers *********************************************************/

/* Configure endpoint, making it usable.
 * The class driver may deallocate or re-use the 'desc' structure after
 * returning:
 *
 * ep   - the struct usbdev_ep_s instance obtained from allocep()
 * desc - A struct usb_epdesc_s instance describing the endpoint
 * last - true if this is the last endpoint to be configured.  Some hardware
 *        needs to take special action when all of the endpoints have been
 *        configured.
 */

#define EP_CONFIGURE(ep,desc,last) (ep)->ops->configure(ep,desc,last)

/* The endpoint will no longer be used */

#define EP_DISABLE(ep)             (ep)->ops->disable(ep)

/* Submit an I/O request to the endpoint */

#define EP_SUBMIT(ep,req)          (ep)->ops->submit(ep,req)

/* Cancel an I/O request previously sent to an endpoint */

#define EP_CANCEL(ep,req)          (ep)->ops->cancel(ep,req)

/* Stall or resume an endpoint */

#define EP_STALL(ep)               (ep)->ops->stall(ep,false)
#define EP_RESUME(ep)              (ep)->ops->stall(ep,true)

/* Check the endpoint interrupt status, call interrupt handler
 * if the transfer is done. This is used for polling mode.
 */

#define EP_POLL(ep) \
  do { if ((ep)->ops->poll) (ep)->ops->poll(ep); } while (0)

/* USB Device Driver Helpers ************************************************/

/* Allocate an endpoint:
 *
 *   ep     - 7-bit logical endpoint number (direction bit ignored).
 *            Zero means that any endpoint matching the other requirements
 *            will suffice.
 *            The assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 */

#define DEV_ALLOCEP(dev,ep,in,type) (dev)->ops->allocep(dev,ep,in,type)

/* Release an endpoint */

#define DEV_FREEEP(dev,ep)         (dev)->ops->freeep(dev,ep)

/* Returns the current frame number */

#define DEV_GETFRAME(dev)          (dev)->ops->getframe(dev)

/* Tries to wake up the host connected to this device */

#define DEV_WAKEUP(dev)            (dev)->ops->wakeup(dev)

/* Sets the device selfpowered feature */

#define DEV_SETSELFPOWERED(dev)    (dev)->ops->selfpowered(dev,true)

/* Clears the device selfpowered feature */

#define DEV_CLRSELFPOWERED(dev)    (dev)->ops->selfpowered(dev, false)

/* Software-controlled connect to USB host. All USB class drivers need to
 * call DEV_CONNECT() when they are ready to be enumerated.  That is, (1)
 * initially when bound to the USB driver, and (2) after a USB reset.
 */

#define DEV_CONNECT(dev)           (dev)->ops->pullup ? (dev)->ops->pullup(dev,true) : -EOPNOTSUPP

/* Software-controlled disconnect from USB host */

#define DEV_DISCONNECT(dev)        (dev)->ops->pullup ? (dev)->ops->pullup(dev,false) : -EOPNOTSUPP

/* USB Class Driver Helpers *************************************************/

/* All may be called from interrupt handling logic except bind() and
 * unbind()
 */

/* Invoked when the driver is bound to a USB device driver. */

#define CLASS_BIND(drvr,dev)      (drvr)->ops->bind(drvr,dev)

/* Invoked when the driver is unbound from a USB device driver */

#define CLASS_UNBIND(drvr,dev)    (drvr)->ops->unbind(drvr,dev)

/* Invoked after all transfers have been stopped, when the host is
 * disconnected.
 */

#define CLASS_DISCONNECT(drvr,dev) (drvr)->ops->disconnect(drvr,dev)

/* Invoked for ep0 control requests */

#define CLASS_SETUP(drvr,dev,ctrl,dataout,outlen) \
  (drvr)->ops->setup(drvr,dev,ctrl,dataout,outlen)

/* Invoked on USB suspend. */

#define CLASS_SUSPEND(drvr,dev)   \
  do { if ((drvr)->ops->suspend) (drvr)->ops->suspend(drvr,dev); } while (0)

/* Invoked on USB resume */

#define CLASS_RESUME(drvr,dev)  \
  do { if ((drvr)->ops->resume) (drvr)->ops->resume(drvr,dev); } while (0)

/* Maximum size of a request buffer */

#define USBDEV_MAXREQUEUST        UINT16_MAX

/* Request flags */

#define USBDEV_REQFLAGS_NULLPKT   1 /* Bit 0: Terminate w/short packet; null packet if necessary */
                                    /* Bits 1-7: Available */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* USB Controller Structures ************************************************/

struct usbdev_strdesc_s
{
  uint8_t         id;
  FAR const char *string;
};

struct usbdev_strdescs_s
{
  uint16_t                            language;
  FAR const struct usbdev_strdesc_s  *strdesc;
};

struct usbdev_devdescs_s
{
  FAR const struct usb_cfgdesc_s     *cfgdesc;
  FAR const struct usbdev_strdescs_s *strdescs;
  FAR const struct usb_devdesc_s     *devdesc;
#ifdef CONFIG_USBDEV_DUALSPEED
  FAR const struct usb_qualdesc_s    *qualdesc;
#endif
};

struct usbdev_epinfo_s
{
  struct usb_epdesc_s desc;
  uint16_t            reqnum;
  uint16_t            fssize;
#ifdef CONFIG_USBDEV_DUALSPEED
  uint16_t            hssize;
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  uint16_t            sssize;
  struct usb_ss_epcompdesc_s compdesc;
#endif
};

/* usbdev_devinfo_s - describes the low level bindings of an usb device */

struct usbdev_devinfo_s
{
  FAR const char *name;
  int ninterfaces; /* Number of interfaces in the configuration */
  int ifnobase;    /* Offset to Interface-IDs */

  int nstrings;    /* Number of Strings */
  int strbase;     /* Offset to String Numbers */

  int nendpoints;  /* Number of Endpoints referenced in the following allay */
  int epno[5];     /* Array holding the endpoint configuration for this device */
  FAR const struct usbdev_epinfo_s **epinfos;
};

struct usbdevclass_driver_s;
struct composite_devdesc_s
{
  CODE int16_t (*mkconfdesc)(FAR uint8_t *buf,
                             FAR struct usbdev_devinfo_s *devinfo,
                             uint8_t speed, uint8_t type);
  CODE int (*mkstrdesc)(uint8_t id, FAR struct usb_strdesc_s *strdesc);
  CODE int (*classobject)(int minor,
                          FAR struct usbdev_devinfo_s *devinfo,
                          FAR struct usbdevclass_driver_s **classdev);
  CODE void (*uninitialize)(FAR struct usbdevclass_driver_s *classdev);

  int nconfigs;    /* Number of configurations supported */
  int configid;    /* The only supported configuration ID */

  int cfgdescsize; /* The size of the config descriptor */
  int minor;

#ifdef CONFIG_COMPOSITE_MSFT_OS_DESCRIPTORS
  uint8_t msft_compatible_id[8];
  uint8_t msft_sub_id[8];
#endif

  struct usbdev_devinfo_s devinfo;
};

/* struct usbdev_req_s - describes one i/o request */

struct usbdev_ep_s;
struct usbdev_req_s
{
  FAR uint8_t *buf; /* Call: Buffer used for data; Return: Unchanged */
  uint8_t  flags;   /* See USBDEV_REQFLAGS_* definitions */
  size_t len;       /* Call: Total length of data in buf; Return: Unchanged */
  size_t xfrd;      /* Call: zero; Return: Bytes transferred so far */
  int16_t  result;  /* Call: zero; Return: Result of transfer (O or -errno) */

  /* Callback when the transfer completes */

  CODE void (*callback)(FAR struct usbdev_ep_s *ep,
                        FAR struct usbdev_req_s *req);
  FAR void  *priv; /* Used only by callee */
};

/* Endpoint-specific interface to USB controller hardware. */

struct usbdev_epops_s
{
  /* Configure/enable and disable endpoint */

  CODE int (*configure)(FAR struct usbdev_ep_s *ep,
                        FAR const struct usb_epdesc_s *desc, bool last);
  CODE int (*disable)(FAR struct usbdev_ep_s *ep);

  /* Allocate and free I/O requests */

  CODE FAR struct usbdev_req_s *(*allocreq)(FAR struct usbdev_ep_s *ep);
  CODE void (*freereq)(FAR struct usbdev_ep_s *ep,
                       FAR struct usbdev_req_s *req);

  /* Allocate and free I/O buffers */

#ifdef CONFIG_USBDEV_DMA
  CODE FAR void *(*allocbuffer)(FAR struct usbdev_ep_s *ep, uint16_t nbytes);
  CODE void (*freebuffer)(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif

  /* Submit and cancel I/O requests */

  CODE int (*submit)(FAR struct usbdev_ep_s *ep,
                     FAR struct usbdev_req_s *req);
  CODE int (*cancel)(FAR struct usbdev_ep_s *ep,
                     FAR struct usbdev_req_s *req);

  /* Stall or resume an endpoint */

  CODE int (*stall)(FAR struct usbdev_ep_s *ep, bool resume);

  /* Check the endpoint interrupt status, call interrupt handler
   * if the transfer is done. This is used for polling mode.
   */

  CODE void (*poll)(FAR struct usbdev_ep_s *ep);
};

/* Representation of one USB endpoint */

struct usbdev_ep_s
{
  FAR const struct usbdev_epops_s *ops; /* Endpoint operations */
  uint8_t  eplog;                       /* Logical endpoint address */
  uint16_t maxpacket;                   /* Maximum packet size for this endpoint */
  FAR void *priv;                       /* For use by class driver */
  FAR void *fs;                         /* USB fs device this ep belongs */
};

/* struct usbdev_s represents a usb device */

struct usbdev_s;
struct usbdev_ops_s
{
  /* Allocate and free endpoints */

  CODE FAR struct usbdev_ep_s *(*allocep)(FAR struct usbdev_s *dev,
                                          uint8_t epphy, bool in,
                                          uint8_t eptype);
  CODE void (*freeep)(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);

  /* Get the frame number from the last SOF */

  CODE int (*getframe)(FAR struct usbdev_s *dev);

  /* Hardware specific features */

  CODE int (*wakeup)(FAR struct usbdev_s *dev);
  CODE int (*selfpowered)(FAR struct usbdev_s *dev, bool selfpowered);
  CODE int (*pullup)(FAR struct usbdev_s *dev,  bool enable);

  /* Device-specific I/O command support */

  CODE int (*ioctl)(FAR struct usbdev_s *dev, unsigned code,
                    unsigned long param);
};

struct usbdev_s
{
  FAR const struct usbdev_ops_s *ops; /* Access to hardware specific features */
  FAR struct usbdev_ep_s *ep0;        /* Endpoint zero */
  uint8_t speed;                      /* Current speed of the host connection */
  uint8_t dualspeed:1;                /* 1:supports high and full speed operation */
};

/* USB Device Class Implementations *****************************************/

struct usbdevclass_driverops_s
{
  CODE int  (*bind)(FAR struct usbdevclass_driver_s *driver,
                    FAR struct usbdev_s *dev);
  CODE void (*unbind)(FAR struct usbdevclass_driver_s *driver,
                      FAR struct usbdev_s *dev);
  CODE int  (*setup)(FAR struct usbdevclass_driver_s *driver,
                     FAR struct usbdev_s *dev,
                     FAR const struct usb_ctrlreq_s *ctrl,
                     FAR uint8_t *dataout, size_t outlen);
  CODE void (*disconnect)(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);
  CODE void (*suspend)(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev);
  CODE void (*resume)(FAR struct usbdevclass_driver_s *driver,
                      FAR struct usbdev_s *dev);
};

struct usbdevclass_driver_s
{
  FAR const struct usbdevclass_driverops_s *ops;
  uint8_t speed;                  /* Highest speed that the driver handles */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

FAR struct usbdev_req_s *usbdev_allocreq(FAR struct usbdev_ep_s *ep,
                                         size_t len);

/****************************************************************************
 * Name: usbdev_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

void usbdev_freereq(FAR struct usbdev_ep_s *ep,
                    FAR struct usbdev_req_s *req);

/****************************************************************************
 * Name: usbdev_copy_devdesc
 *
 * Description:
 *   Copies the requested device Description into the dest buffer given.
 *   Returns the number of Bytes filled in (USB_SIZEOF_DEVDESC).
 *   This function is provided by various classes.
 *
 ****************************************************************************/

int usbdev_copy_devdesc(FAR void *dest,
                        FAR const struct usb_devdesc_s *src,
                        uint8_t speed);

/****************************************************************************
 * Name: usbdev_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *   This function is provided by various classes.
 *
 ****************************************************************************/

int usbdev_copy_epdesc(FAR struct usb_epdesc_s *epdesc,
                       uint8_t epno, uint8_t speed,
                       FAR const struct usbdev_epinfo_s *epinfo);

/****************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(FAR struct usbdevclass_driver_s *driver);

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB
 *   host, it will first disconnect().
 *   The driver is also requested to unbind() and clean up any device state,
 *  before this procedure finally returns.
 *
 ****************************************************************************/

int usbdev_unregister(FAR struct usbdevclass_driver_s *driver);

/****************************************************************************
 * Name: usbdev_dma_alloc and usbdev_dma_free
 *
 * Description:
 *   The USB class driver allocates packet I/O buffers for data transfer by
 *   calling the driver allocbuffer() and freebuffer() methods.  Those
 *   methods are only available if CONFIG_USBDEV_DMA is defined in the
 *   system configuration.
 *
 *   If CONFIG_USBDEV_DMAMEMORY is also defined in the NuttX configuration,
 *   then the driver implementations of the allocbuffer() and freebuffer()
 *   methods may use board-specific usbdev_dma_alloc() and usbdev_dma_free().
 *   If CONFIG_USBDEV_DMA and CONFIG_USBDEV_DMAMEMORY are both defined,
 *   then the board-specific logic must provide the functions
 *   usbdev_dma_alloc() and usbdev_dma_free() as prototyped below:
 *   usbdev_dma_alloc() will allocate DMA-capable memory of the specified
 *   size; usbdev_dma_free() is the corresponding function that will be
 *   called to free the DMA-capable memory.
 *
 *   This functions may be simple wrappers around gran_alloc() and
 *   gran_free() (See nuttx/mm/gran.h).  Note that the gran_free() function
 *   does require the size of the allocation to be freed; that would need
 *   to be managed in the board-specific logic.
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_DMA) && defined(CONFIG_USBDEV_DMAMEMORY)
FAR void *usbdev_dma_alloc(size_t size);
void usbdev_dma_free(FAR void *memory);
#endif

/****************************************************************************
 * Name: up_usbdev_sof_irq
 *
 * Description:
 *   If CONFIG_USBDEV_SOFINTERRUPT is enabled, board logic must provide
 *   this function. It gets called in interrupt mode by USB device code
 *   every time start-of-frame USB packet is received from host.
 *
 ****************************************************************************/
#ifdef CONFIG_USBDEV_SOFINTERRUPT
void usbdev_sof_irq(FAR struct usbdev_s *dev, uint16_t frameno);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBDEV_H */
