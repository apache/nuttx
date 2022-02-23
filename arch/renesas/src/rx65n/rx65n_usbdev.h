/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_usbdev.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_USBDEV_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_USBDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <nuttx/config.h>
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CDC_CLASS_DATA_LENGTH                       (7)
#define RW_DATA_LEN                                 (1)
#define BULK_IN_PIPE                                (1)
#define INT_IN_PIPE                                 (6)
#define INT_OUT_PIPE                                (7)
#define BULK_OUT_PIPE                               (2)

/* The below endpoint numbers are assigned by
 * NuttX Class Driver, during endpoint
 * initialization.
 *
 */

#define BULK_IN_EPNUM                               (2)
#define INT_IN_EPNUM                                (1)
#define BULK_OUT_EPNUM                              (3)
#define INT_OUT_EPNUM                               (4)

/* USB_BMREQUESTTYPERECIP   0x001Fu(b4-0) */
#define USB_DEVICE                                (0x0000u)
#define USB_INTERFACE                             (0x0001u)
#define USB_ENDPOINT                              (0x0002u)
#define USB_OTHER                                 (0x0003u)

#define USB_DEV_REMOTE_WAKEUP                     (0x0001u)
#define USB_TEST_MODE                             (0x0002u)
#define USB_TEST_RESERVED                         (0x4000u)   /* Reserved */
#define USB_TEST_VSTMODES                         (0xC000u)   /* VendorSpecific test modes */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_USBDEV_H */
