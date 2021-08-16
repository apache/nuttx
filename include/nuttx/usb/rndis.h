/****************************************************************************
 * include/nuttx/usb/rndis.h
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

#ifndef __INCLUDE_NUTTX_USB_RNDIS_H
#define __INCLUDE_NUTTX_USB_RNDIS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Indexes for devinfo.epno[] array.
 * Used for composite device configuration.
 */

#define RNDIS_EP_INTIN_IDX      (0)
#define RNDIS_EP_BULKIN_IDX     (1)
#define RNDIS_EP_BULKOUT_IDX    (2)

/* Endpoint configuration ***************************************************/

#define RNDIS_MKEPINTIN(desc)     (USB_DIR_IN | (desc)->epno[RNDIS_EP_INTIN_IDX])
#define RNDIS_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)

#define RNDIS_MKEPBULKIN(desc)    (USB_DIR_IN | (desc)->epno[RNDIS_EP_BULKIN_IDX])
#define RNDIS_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define RNDIS_MKEPBULKOUT(desc)   ((desc)->epno[RNDIS_EP_BULKOUT_IDX])
#define RNDIS_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

#ifndef CONFIG_RNDIS_EPINTIN_FSSIZE
#  define CONFIG_RNDIS_EPINTIN_FSSIZE 16
#endif

#ifndef CONFIG_RNDIS_EPINTIN_HSSIZE
#  define CONFIG_RNDIS_EPINTIN_HSSIZE 16
#endif

#ifndef CONFIG_RNDIS_EPBULKIN_FSSIZE
#  define CONFIG_RNDIS_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_RNDIS_EPBULKIN_HSSIZE
#  define CONFIG_RNDIS_EPBULKIN_HSSIZE 512
#endif

#ifndef CONFIG_RNDIS_EPBULKOUT_FSSIZE
#  define CONFIG_RNDIS_EPBULKOUT_FSSIZE 64
#endif

#ifndef CONFIG_RNDIS_EPBULKOUT_HSSIZE
#  define CONFIG_RNDIS_EPBULKOUT_HSSIZE 512
#endif

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
 * Name: usbdev_rndis_initialize
 *
 * Description:
 *   Register RNDIS USB device interface.
 *   Register the corresponding network driver to NuttX and bring up the
 *   network.
 *
 * Input Parameters:
 *   mac_address: an array of 6 bytes which make the MAC address of the host
 *                side of the network.
 *
 * Returned Value:
 *   0 on success; -errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_RNDIS_COMPOSITE
int usbdev_rndis_initialize(FAR const uint8_t *mac_address);
#endif

/****************************************************************************
 * Name: usbdev_rndis_set_host_mac_addr
 *
 * Description:
 *   Set host MAC address. Mainly for use with composite devices where
 *   the MAC cannot be given directly to usbdev_rndis_initialize().
 *
 * Input Parameters:
 *   netdev:      pointer to the network interface. Can be obtained from
 *                e.g. netdev_findbyname().
 *
 *   mac_address: pointer to an array of six octets which is the MAC address
 *                of the host side of the interface. May be NULL to use the
 *                default MAC address.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

int usbdev_rndis_set_host_mac_addr(FAR struct net_driver_s *netdev,
                                   FAR const uint8_t *mac_address);

/****************************************************************************
 * Name: usbdev_rndis_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RNDIS_COMPOSITE
void usbdev_rndis_get_composite_devdesc(struct composite_devdesc_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_RNDIS_H */
