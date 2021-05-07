/****************************************************************************
 * arch/arm/include/cxd56xx/usbdev.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_USBDEV_H
#define __ARCH_ARM_INCLUDE_CXD56XX_USBDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* BOARDIOC_USBDEV_SETNOTIFYSIG signal value ********************************/

#define USBDEV_STATE_DETACH       0
#define USBDEV_STATE_ATTACH       1

/* The BOARDIOC_USBDEV_SETNOTIFYSIG signal output the VBUS connection state
 * and supply current value to the signal handler argument (sival_int).
 *
 * Please use the following macros.
 *
 * - USBDEV_CONNECTED : Get VBUS connection state.
 * - USBDEV_POWER_CURRENT : Get VBUS supply current.
 */

#define USBDEV_CONNECTED(x)       (0xffff & ((x)>>16))
#define USBDEV_POWER_CURRENT(x)   (0xffff & (x))

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_USBDEV_H */
