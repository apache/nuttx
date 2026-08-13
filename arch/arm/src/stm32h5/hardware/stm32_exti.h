/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_EXTI_RTSR1_OFFSET      0x0000  /* Rising Trigger Selection 1       */
#define STM32_EXTI_FTSR1_OFFSET      0x0004  /* Falling Trigger Selection 1      */
#define STM32_EXTI_SWIER1_OFFSET     0x0008  /* Software Interrupt Event 1       */
#define STM32_EXTI_RPR1_OFFSET       0x000c  /* Rising Edge Pending 1            */
#define STM32_EXTI_FPR1_OFFSET       0x0010  /* Falling Edge Pending 1           */
#define STM32_EXTI_SECCFGR1_OFFSET   0x0014  /* Security Configuration 1         */
#define STM32_EXTI_PRIVCFGR1_OFFSET  0x0018  /* Privilege Configuration 1        */
#define STM32_EXTI_RTSR2_OFFSET      0x0020  /* Rising Trigger Selection 2       */
#define STM32_EXTI_FTSR2_OFFSET      0x0024  /* Falling Trigger Selection 2      */
#define STM32_EXTI_SWIER2_OFFSET     0x0028  /* Software Interrupt Event 2       */
#define STM32_EXTI_RPR2_OFFSET       0x002c  /* Rising Edge Pending 2            */
#define STM32_EXTI_FPR2_OFFSET       0x0030  /* Falling Edge Pending 2           */
#define STM32_EXTI_SECCFGR2_OFFSET   0x0034  /* Security Configuration 2         */
#define STM32_EXTI_PRIVCFGR2_OFFSET  0x0038  /* Privilege Configuration 2        */
#define STM32_EXTI_EXTICR1_OFFSET    0x0060  /* External Interrupt Selection 1   */
#define STM32_EXTI_EXTICR2_OFFSET    0x0064  /* External Interrupt Selection 2   */
#define STM32_EXTI_EXTICR3_OFFSET    0x0068  /* External Interrupt Selection 3   */
#define STM32_EXTI_EXTICR4_OFFSET    0x006C  /* External Interrupt Selection 4   */
#define STM32_EXTI_LOCKR_OFFSET      0x0070  /* Lock                             */
#define STM32_EXTI_IMR1_OFFSET       0x0080  /* CPU Wakeup with Interrupt Mask 1 */
#define STM32_EXTI_EMR1_OFFSET       0x0084  /* CPU Wakeup with Event Mask 1     */
#define STM32_EXTI_IMR2_OFFSET       0x0090  /* CPU Wakeup with Interrupt Mask 2 */
#define STM32_EXTI_EMR2_OFFSET       0x0094  /* CPU Wakeup with Event Mask 2     */

/* Register Addresses *******************************************************/

#define STM32_EXTI_RTSR1      (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1      (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1     (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_RPR1       (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1       (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_SECCFGR1   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR1_OFFSET)
#define STM32_EXTI_PRIVCFGR1  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR1_OFFSET)
#define STM32_EXTI_RTSR2      (STM32_EXTI_BASE + STM32_EXTI_RTSR2_OFFSET)
#define STM32_EXTI_FTSR2      (STM32_EXTI_BASE + STM32_EXTI_FTSR2_OFFSET)
#define STM32_EXTI_SWIER2     (STM32_EXTI_BASE + STM32_EXTI_SWIER2_OFFSET)
#define STM32_EXTI_RPR2       (STM32_EXTI_BASE + STM32_EXTI_RPR2_OFFSET)
#define STM32_EXTI_FPR2       (STM32_EXTI_BASE + STM32_EXTI_FPR2_OFFSET)
#define STM32_EXTI_SECCFGR2   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR2_OFFSET)
#define STM32_EXTI_PRIVCFGR2  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR2_OFFSET)
#define STM32_EXTI_EXTICR1    (STM32_EXTI_BASE + STM32_EXTI_EXTICR1_OFFSET)
#define STM32_EXTI_EXTICR2    (STM32_EXTI_BASE + STM32_EXTI_EXTICR2_OFFSET)
#define STM32_EXTI_EXTICR3    (STM32_EXTI_BASE + STM32_EXTI_EXTICR3_OFFSET)
#define STM32_EXTI_EXTICR4    (STM32_EXTI_BASE + STM32_EXTI_EXTICR4_OFFSET)
#define STM32_EXTI_LOCKR      (STM32_EXTI_BASE + STM32_EXTI_LOCKR_OFFSET)
#define STM32_EXTI_IMR1       (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1       (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)
#define STM32_EXTI_IMR2       (STM32_EXTI_BASE + STM32_EXTI_IMR2_OFFSET)
#define STM32_EXTI_EMR2       (STM32_EXTI_BASE + STM32_EXTI_EMR2_OFFSET)

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_EXTI_H */
