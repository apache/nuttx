/****************************************************************************
 * arch/arm/src/chip/hardware/nrf52_ecb.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_ECB_TASKS_STARTECB_OFFSET   0x000000  /* Start ECB block encrypt */
#define NRF52_ECB_TASKS_STOPECB_OFFSET    0x000004  /* Abort a possible executing ECB operation */
#define NRF52_ECB_EVENTS_ENDECB_OFFSET    0x000100  /* ECB block encrypt complete */
#define NRF52_ECB_EVENTS_ERRORECB_OFFSET  0x000104  /* ECB block encrypt aborted because of a STOPECB task or due to an error */
#define NRF52_ECB_INTENSET_OFFSET         0x000304  /* Enable interrupt */
#define NRF52_ECB_INTENCLR_OFFSET         0x000308  /* Disable interrupt */
#define NRF52_ECB_ECBDATAPTR_OFFSET       0x000504  /* ECB block encrypt memory pointers */

/* Register definitions *****************************************************/

#define NRF52_ECB_TASKS_STARTECB   (NRF52_ECB_BASE + NRF52_ECB_TASKS_STARTECB_OFFSET)
#define NRF52_ECB_TASKS_STOPECB    (NRF52_ECB_BASE + NRF52_ECB_TASKS_STOPECB_OFFSET)
#define NRF52_ECB_EVENTS_ENDECB    (NRF52_ECB_BASE + NRF52_ECB_EVENTS_ENDECB_OFFSET)
#define NRF52_ECB_EVENTS_ERRORECB  (NRF52_ECB_BASE + NRF52_ECB_EVENTS_ERRORECB_OFFSET)
#define NRF52_ECB_INTENSET         (NRF52_ECB_BASE + NRF52_ECB_INTENSET_OFFSET)
#define NRF52_ECB_INTENCLR         (NRF52_ECB_BASE + NRF52_ECB_INTENCLR_OFFSET)
#define NRF52_ECB_ECBDATAPTR       (NRF52_ECB_BASE + NRF52_ECB_ECBDATAPTR_OFFSET)

/* Register bit definitions *************************************************/

#define NRF52_ECB_INTENSET_ENDECB    (1 << 0)  /* Read: Enabled */
#define NRF52_ECB_INTENSET_ERRORECB  (1 << 1)  /* Read: Enabled */

#define NRF52_ECB_INTENCLR_ENDECB    (1 << 0)  /* Read: Enabled */
#define NRF52_ECB_INTENCLR_ERRORECB  (1 << 1)  /* Read: Enabled */

#endif // __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H
