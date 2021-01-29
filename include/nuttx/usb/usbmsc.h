/****************************************************************************
 * include/nuttx/usb/usbmsc.h
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

#ifndef __INCLUDE_NUTTX_USB_USBMSC_H
#define __INCLUDE_NUTTX_USB_USBMSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Information about the device needed in usbdev_devinfo_s */

#define USBMSC_CONFIGID        (1) /* The only supported configuration ID */
#define USBMSC_NENDPOINTS      (2) /* Number of endpoints in the interface  */

#define USBMSC_EP_BULKIN_IDX   (0)
#define USBMSC_EP_BULKOUT_IDX  (1)

#define USBMSC_NCONFIGS        (1) /* Number of configurations supported */
#define USBMSC_NINTERFACES     (1) /* Number of interfaces in the configuration */

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Public Functions Definitions
 ****************************************************************************/

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
 *   0 on success; a negated errno on failure.  The returned handle value is
 *   an untyped equivalent to the usbmsc_classobject().
 *
 ****************************************************************************/

int usbmsc_configure(unsigned int nluns, FAR void **handle);

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

int usbmsc_bindlun(FAR void *handle,
                   FAR const char *drvrpath,
                   unsigned int lunno,
                   off_t startsector,
                   size_t nsectors,
                   bool readonly);

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

int usbmsc_unbindlun(FAR void *handle, unsigned int lunno);

/****************************************************************************
 * Name: usbmsc_exportluns
 *
 * Description:
 *   After all of the LUNs have been bound, this function may be called in
 *   order to export those LUNs in the USB storage device.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure().
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

int usbmsc_exportluns(FAR void *handle);

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

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBMSC_COMPOSITE)
struct usbdevclass_driver_s;
int usbmsc_classobject(FAR void *handle,
                       FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: usbmsc_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  The handle is the USB MSC
 *   class' device object.  This is the same value as returned by
 *    usbmsc_classobject (typed) or by usbmsc_configure (untyped).
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure()
 *     (or usbmsc_classobject()).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbmsc_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: usbmsc_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration structure.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBMSC_COMPOSITE)
struct composite_devdesc_s;
void usbmsc_get_composite_devdesc(FAR struct composite_devdesc_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBMSC_H */
