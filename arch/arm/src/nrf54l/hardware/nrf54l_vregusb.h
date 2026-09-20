/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_vregusb.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_VREGUSB_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_VREGUSB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_VREGUSB_TASKS_START_OFFSET         0x0000 /* Start USB regulator */
#define NRF54L_VREGUSB_TASKS_STOP_OFFSET          0x0004 /* Stop USB regulator */
#define NRF54L_VREGUSB_EVENTS_VBUSDETECTED_OFFSET 0x0104 /* VBUS detected */
#define NRF54L_VREGUSB_EVENTS_VBUSREMOVED_OFFSET  0x0110 /* VBUS removed */
#define NRF54L_VREGUSB_INTEN_OFFSET               0x0300 /* Enable or disable interrupts */
#define NRF54L_VREGUSB_INTENSET_OFFSET            0x0304 /* Enable interrupts */
#define NRF54L_VREGUSB_INTENCLR_OFFSET            0x0308 /* Disable interrupts */
#define NRF54L_VREGUSB_INTPEND_OFFSET             0x030c /* Pending interrupts */
#define NRF54L_VREGUSB_STATUS_OFFSET              0x0400 /* VBUS detection status */

/* Register addresses *******************************************************/

#define NRF54L_VREGUSB_TASKS_START         (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_TASKS_START_OFFSET)
#define NRF54L_VREGUSB_TASKS_STOP          (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_TASKS_STOP_OFFSET)
#define NRF54L_VREGUSB_EVENTS_VBUSDETECTED (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_EVENTS_VBUSDETECTED_OFFSET)
#define NRF54L_VREGUSB_EVENTS_VBUSREMOVED  (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_EVENTS_VBUSREMOVED_OFFSET)
#define NRF54L_VREGUSB_INTEN               (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_INTEN_OFFSET)
#define NRF54L_VREGUSB_INTENSET            (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_INTENSET_OFFSET)
#define NRF54L_VREGUSB_INTENCLR            (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_INTENCLR_OFFSET)
#define NRF54L_VREGUSB_INTPEND             (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_INTPEND_OFFSET)
#define NRF54L_VREGUSB_STATUS              (NRF54L_VREGUSB_BASE + NRF54L_VREGUSB_STATUS_OFFSET)

/* Register bit definitions *************************************************/

#define VREGUSB_TASKS_START                      (1 << 0)
#define VREGUSB_TASKS_STOP                       (1 << 0)
#define VREGUSB_EVENTS_VBUSDETECTED              (1 << 0)
#define VREGUSB_EVENTS_VBUSREMOVED               (1 << 0)
#define VREGUSB_INT_VBUSDETECTED                 (1 << 1)
#define VREGUSB_INT_VBUSREMOVED                  (1 << 4)

/* STATUS is specified by Nordic's VREGUSB driver, but absent from the MDK
 * register structure.
 */

#define VREGUSB_STATUS_VBUSDET                   (1 << 2)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_VREGUSB_H */
