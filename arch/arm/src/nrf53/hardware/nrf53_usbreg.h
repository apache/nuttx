/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_usbreg.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_USBREG_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_USBREG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_USBREG_EVENTS_USBDETECTED_OFFSET 0x000100  /* Voltage supply detected on VBUS */
#define NRF53_USBREG_EVENTS_USBREMOVED_OFFSET  0x000104  /* Voltage supply removed from VBUS*/
#define NRF53_USBREG_EVENTS_USBPWRRDY_OFFSET   0x000108  /* USB 3.3 V supply ready */
#define NRF53_USBREG_INTEN_OFFSET              0x000300  /* Enable or disable interrrupt */
#define NRF53_USBREG_INTENSET_OFFSET           0x000304  /* Enable interrrupt */
#define NRF53_USBREG_INTENCLR_OFFSET           0x000308  /* Disable interrrupt */
#define NRF53_USBREG_USBREGSTATUS_OFFSET       0x000400  /* USB supply status */

/* Register definitions *****************************************************/

#define NRF53_USBREG_EVENTS_USBDETECTED (NRF53_USBREG_BASE + NRF53_USBREG_EVENTS_USBDETECTED_OFFSET)
#define NRF53_USBREG_EVENTS_USBREMOVED  (NRF53_USBREG_BASE + NRF53_USBREG_EVENTS_USBREMOVED_OFFSET)
#define NRF53_USBREG_EVENTS_USBPWRRDY   (NRF53_USBREG_BASE + NRF53_USBREG_EVENTS_USBPWRRDY_OFFSET)
#define NRF53_USBREG_INTEN              (NRF53_USBREG_BASE + NRF53_USBREG_INTEN_OFFSET)
#define NRF53_USBREG_INTENSET           (NRF53_USBREG_BASE + NRF53_USBREG_INTENSET_OFFSET)
#define NRF53_USBREG_INTENCLR           (NRF53_USBREG_BASE + NRF53_USBREG_INTENCLR_OFFSET)
#define NRF53_USBREG_USBREGSTATUS       (NRF53_USBREG_BASE + NRF53_USBREG_USBREGSTATUS_OFFSET)

/* Register bit definitions *************************************************/

/* USBREGSTATUS */

#define USBREG_USBREGSTATUS_VBUSDETECT  (1 << 0)   /* Vbus Present */
#define USBREG_USBREGSTATUS_OUTPUTRDY   (1 << 1)   /* Ready */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_USBREG_H */
