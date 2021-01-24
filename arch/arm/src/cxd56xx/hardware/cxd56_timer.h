/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_timer.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_TIMER_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/cxd56xx/chip.h>

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses *******************************************************/

#define CXD56_TIMER0_BASE       (CXD56_TIMER_BASE)
#define CXD56_TIMER1_BASE       (CXD56_TIMER_BASE + 0x0020)
#define CXD56_TIMER_LOAD        (0x0000) /* Load register */
#define CXD56_TIMER_VALUE       (0x0004) /* Value register [RO] */
#define CXD56_TIMER_CONTROL     (0x0008) /* Control register */
#define CXD56_TIMER_INTCLR      (0x000C) /* Clear Interrupt register [WO] */
#define CXD56_TIMER_RIS         (0x0010) /* Raw Interrupt Status register [RO] */
#define CXD56_TIMER_MIS         (0x0014) /* Interrupt Status register [RO] */
#define CXD56_TIMER_BGLOAD      (0x0018) /* Background Load register [RO] */
#define CXD56_TIMER_ITCR        (0x0F00) /* Integration Test Control register */
#define CXD56_TIMER_ITOP        (0x0F04) /* Integration Test Output register [WO] */
#define CXD56_TIMER_PERIPHID0   (0x0FE0) /* Peripheral ID0 register [RO] */
#define CXD56_TIMER_PERIPHID1   (0x0FE4) /* Peripheral ID1 register [RO] */
#define CXD56_TIMER_PERIPHID2   (0x0FE8) /* Peripheral ID2 register [RO] */
#define CXD56_TIMER_PERIPHID3   (0x0FFC) /* Peripheral ID3 register [RO] */
#define CXD56_TIMER_PCELLID0    (0x0FF0) /* PrimeCell ID0 register [RO] */
#define CXD56_TIMER_PCELLID1    (0x0FF4) /* PrimeCell ID1 register [RO] */
#define CXD56_TIMER_PCELLID2    (0x0FF8) /* PrimeCell ID2 register [RO] */
#define CXD56_TIMER_PCELLID3    (0x0FFC) /* PrimeCell ID3 register [RO] */

/* Register bit definitions *************************************************/

/* Control Register */

#define TIMERCTRL_ENABLE        (0x1u << 7)
#define TIMERCTRL_DISABLE       (0x0u << 7)
#define TIMERCTRL_PERIODIC      (0x1u << 6)
#define TIMERCTRL_FREERUN       (0x0u << 6)
#define TIMERCTRL_INTENABLE     (0x1u << 5)
#define TIMERCTRL_INTDISABLE    (0x0u << 5)
#define TIMERCTRL_DIV_256       (0x2u << 2)
#define TIMERCTRL_DIV_16        (0x1u << 2)
#define TIMERCTRL_DIV_1         (0x0u << 2)
#define TIMERCTRL_SIZE_32BIT    (0x1u << 1)
#define TIMERCTRL_SIZE_16BIT    (0x0u << 1)
#define TIMERCTRL_MODE_ONESHOT  (0x1u << 0)
#define TIMERCTRL_MODE_WRAP     (0x0u << 0)

/* Interrupt Register */

#define TIMER_INTERRUPT         (0x1u << 0)

/* Integration Test Control register */

#define TIMERITCR_ENABLE        (0x1u << 0)

/* Integration Test Output register */

#define TIMERITOP_TIMINT1       (0x1u << 0)
#define TIMERITOP_TIMINT2       (0x1u << 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_TIMER_H */
