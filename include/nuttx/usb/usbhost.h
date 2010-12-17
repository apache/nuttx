/************************************************************************************
 * include/nuttx/usb/usbhost.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   "Universal Serial Bus Mass Storage Class, Specification Overview,"
 *   Revision 1.2,  USB Implementer's Forum, June 23, 2003.
 *
 *   "Universal Serial Bus Mass Storage Class, Bulk-Only Transport,"
 *   Revision 1.0, USB Implementer's Forum, September 31, 1999.
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

#ifndef __NUTTX_USB_USBHOST_H
#define __NUTTX_USB_USBHOST_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Name: CLASS_CREATE
 *
 * Description:
 *   This macro will call the create() method of struct usbhost_registry_s.  The create()
 *   method is a callback into the class implementation.  It is used to (1) create
 *   a new instance of the USB host class state and to (2) bind a USB host driver
 *   "session" to the class instance.  Use of this create() method will support
 *   environments where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   reg - The USB host class registry entry previously obtained from a call to
 *     usbhost_findclass().
 *   drvr - An instance of struct usbhost_driver_s that the class implementation will
 *     "bind" to its state structure and will subsequently use to communicate with
 *     the USB host driver.
 *   id - In the case where the device supports multiple base classes, subclasses, or
 *     protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate with the
 *   USB host class.  NULL is returned on failure; this function will fail only if
 *   the drvr input parameter is NULL or if there are insufficient resources to
 *   create another USB host class instance.
 *
 * Assumptions:
 *   If this function is called from an interrupt handler, it will be unable to
 *   allocate memory and CONFIG_USBHOST_NPREALLOC should be defined to be a value
 *   greater than zero specify a number of pre-allocated class structures.
 *
 ************************************************************************************/

#define CLASS_CREATE(reg, drvr, id) ((reg)->create(drvr))

/************************************************************************************
 * Name: CLASS_CONFIGDESC
 *
 * Description:
 *   This macro will call the configdesc() method of struct usbhost_class_s.  This
 *   method is a callback into the class implementation.  It is used to provide the
 *   device's configuration descriptor to the class so that the class may initialize
 *   properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ************************************************************************************/

#define CLASS_CONFIGDESC(class,configdesc,desclen) ((class)->configdesc(class,configdesc,desclen))

/************************************************************************************
 * Name: CLASS_COMPLETE
 *
 * Description:
 *   This macro will call the complete() method of struct usbhost_class_s.  In the
 *   interface with the USB host drivers, the class will queue USB IN/OUT
 *   transactions.  The enqueuing function will return and the transactions will be
 *   performed asynchrounously.  When the transaction completes, the USB host driver
 *   will call this function in order to inform the class that the transaction has
 *   completed and to provide any response data.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   response - Response data buffer
 *   resplen - Number of bytes of data in the response data buffer.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ************************************************************************************/

#define CLASS_COMPLETE(class,response,resplen) (class)->complete(class,response,resplen))

/************************************************************************************
 * Name: CLASS_DISCONNECTED
 *
 * Description:
 *   This macro will call the disconnected() method of struct usbhost_class_s.  This
 *   method is a callback into the class implementation.  It is used to inform the
 *   class that the USB device has been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ************************************************************************************/

#define CLASS_DISCONNECTED(class) ((class)->disconnected(class))

/************************************************************************************
 * Name: DRVR_ENUMERATE
 *
 * Description:
 *   Enumerate the connected device.  This function will enqueue the
 *   enumeration process.  As part of this enumeration process, the driver
 *   will (1) get the device's configuration descriptor, (2) extract the class
 *   ID info from the configuration descriptor, (3) call usbhost_findclass()
 *   to find the class that supports this device, (4) call the create()
 *   method on the struct usbhost_registry_s interface to get a class
 *   instance, and finally (5) call the configdesc() method of the struct
 *   usbhost_class_s interface.  After that, the class is in charge of the
 *   sequence of operations.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_ENUMERATE(drvr) ((drvr)->enumerate(drvr))

/************************************************************************************
 * Name: DRVR_TRANSFER
 *
 * Description:
 *   Enqueue a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  The transfer will
 *   be performed asynchronously.  When the transfer completes, the USB host
 *   driver will call he complete() method of the struct usbhost_class_s
 *   interface.  Only one transfer may be queued; this function cannot be
 *   again until the class complete() method has been called.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_TRANSFER(drvr,ed,buffer,buflen) ((drvr)->transfer(drvr,ed,buffer,buflen))

/************************************************************************************
 * Name: DRVR_ALLOC
 *
 * Description:
 *   Some hardware supports special memory in which transfer descriptors can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the transfer descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_ALLOC(drvr,buffer,maxlen) ((drvr)->alloc(drvr,buffer,maxlen))

/************************************************************************************
 * Name: DRVR_FREE
 *
 * Description:
 *   Some hardware supports special memory in which transfer descriptors can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   transfer descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_FREE(drvr,buffer) ((drvr)->free(drvr,buffer))

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This struct contains all of the information that is needed to associate a device
 * this is connected via a USB port to a class.
 */

struct usbhost_id_s
{
  uint8_t  base;     /* Base device class code (see USB_CLASS_* defines in usb.h) */
  uint8_t  subclass; /* Sub-class, depends on base class. Eg., See USBSTRG_SUBCLASS_* */
  uint8_t  proto;    /* Protocol, depends on base class. Eg., See USBSTRG_PROTO_* */
  uint16_t vid;      /* Vendor ID (for vendor/product specific devices) */
  uint16_t pid;      /* Product ID (for vendor/product specific devices) */
};

/* The struct usbhost_registry_s type describes information that is kept in the the
 * USB host registry.  USB host class implementations register this information so
 * that USB host drivers can later find the class that matches the device that is
 * connected to the USB port.
 */

struct usbhost_driver_s; /* Forward reference to the driver state structure */
struct usbhost_class_s;  /* Forward reference to the class state structure */
struct usbhost_registry_s
{
  /* This field is used to implement a singly-link registry structure.  Because of
   * the presence of this link, provides of structy usbhost_registry_s instances must
   * provide those instances in write-able memory (RAM).
   */

  struct usbhost_registry_s     *flink;

  /* This is a callback into the class implementation.  It is used to (1) create
   * a new instance of the USB host class state and to (2) bind a USB host driver
   * "session" to the class instance.  Use of this create() method will support
   * environments where there may be multiple USB ports and multiple USB devices
   * simultaneously connected (see the CLASS_CREATE() macro above).
   */
 
  FAR struct usbhost_class_s     *(*create)(FAR struct usbhost_driver_s *drvr,
                                           FAR const struct usbhost_id_s *id);

  /* This information uniquely identifies the USB host class implementation that
   * goes with a specific USB device.
   */

  uint8_t                       nids;  /* Number of IDs in the id[] array */
  FAR const struct usbhost_id_s *id;   /* An array of ID info. Actual dimension is nids */
};

/* struct usbhost_class_s provides access from the USB host driver to the USB host
 * class implementation.
 */

struct usbhost_class_s
{
  /* Provides the configuration descriptor to the class.  The configuration
   * descriptor contains critical information needed by the class in order to
   * initialize properly (such as endpoint selections).
   */

  int (*configdesc)(FAR struct usbhost_class_s *class, FAR const uint8_t *configdesc, int desclen);

  /* This method must be called by the USB host driver whenever a transfer
   * completes.
   */

  int (*complete)(FAR struct usbhost_class_s *class, FAR const uint8_t *response, int resplen);

  /* This method informs the class that the USB device has been disconnected. */

  int (*disconnected)(FAR struct usbhost_class_s *class);
};

/* struct usbhost_driver_s provides access to the USB host driver from the
 * USB host class implementation.
 */

struct usbhost_epdesc_s;
struct usbhost_driver_s
{
  /* Enumerate the connected device.  This function will enqueue the
   * enumeration process.  As part of this enumeration process, the driver
   * will (1) get the device's configuration descriptor, (2) extract the class
   * ID info from the configuration descriptor, (3) call usbhost_findclass()
   * to find the class that supports this device, (4) call the create()
   * method on the struct usbhost_registry_s interface to get a class
   * instance, and finally (5) call the configdesc() method of the struct
   * usbhost_class_s interface.  After that, the class is in charge of the
   * sequence of operations.
   */

  int (*enumerate)(FAR struct usbhost_driver_s *drvr);

  /* Enqueue a request to handle a transfer descriptor.  This method will
   * enqueue the transfer request and return immediately.  The transfer will
   * be performed asynchronously.  When the transfer completes, the USB host
   * driver will call he complete() method of the struct usbhost_class_s
   * interface.  Only one transfer may be queued; this function cannot be
   * again until the class complete() method has been called.
   */

  int (*transfer)(FAR struct usbhost_driver_s *drvr,
                  FAR struct usbhost_epdesc_s *ed,
                  FAR uint8_t *buffer, size_t buflen);

  /* Some hardware supports special memory in which transfer descriptors can
   * be accessed more efficiently.  The following methods provide a mechanism
   * to allocate and free the transfer descriptor memory.  If the underlying
   * hardware does not support such "special" memory, these functions may
   * simply map to malloc and free.
   */

  int (*alloc)(FAR struct usbhost_driver_s *drvr,
               FAR uint8_t **buffer, FAR size_t *maxlen);
  int (*free)(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);

  /* Receive control information */

  int (*rcvctrl)(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_epdesc_s *ed);
};

/* This structure describes one endpoint */

struct usbhost_epdesc_s
{
  uint8_t  addr;         /* Endpoint address */
  bool     in;           /* Direction: TRUE = IN */
  uint16_t mxpacketsize; /* Max packetsize */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: usbhost_registerclass
 *
 * Description:
 *   Register a USB host class implementation.  The caller provides an instance of
 *   struct usbhost_registry_s that contains all of the information that will be
 *   needed later to (1) associate the USB host class implementation with a connected
 *   USB device, and (2) to obtain and bind a struct usbhost_class_s instance for
 *   the device.
 *
 * Input Parameters:
 *   class - An write-able instance of struct usbhost_registry_s that will be
 *     maintained in a registry.
 *
 * Returned Values:
 *   On success, this function will return zero (OK).  Otherwise, a negated errno
 *   value is returned.
 *
 ************************************************************************************/

EXTERN int usbhost_registerclass(struct usbhost_registry_s *class);

/************************************************************************************
 * Name: usbhost_findclass
 *
 * Description:
 *   Find a USB host class implementation previously registered by
 *   usbhost_registerclass().  On success, an instance of struct usbhost_registry_s
 *   will be returned.  That instance will contain all of the information that will
 *   be needed to obtain and bind a struct usbhost_class_s instance for the device.
 *
 * Input Parameters:
 *   id - Identifies the USB device class that has connect to the USB host.
 *
 * Returned Values:
 *   On success this function will return a non-NULL instance of struct
 *   usbhost_registry_s.  NULL will be returned on failure.  This function can only
 *   fail if (1) id is NULL, or (2) no USB host class is registered that matches the
 *   device class ID.
 *
 ************************************************************************************/

EXTERN const struct usbhost_registry_s *usbhost_findclass(const struct usbhost_id_s *id);

/****************************************************************************
 * Name: usbhost_storageinit
 *
 * Description:
 *   Initialize the USB host storage class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

EXTERN int usbhost_storageinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __NUTTX_USB_USBHOST_H */
