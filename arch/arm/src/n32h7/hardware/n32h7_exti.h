/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_exti.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32_EXTI_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/n32h7_memorymap.h"

#if defined(CONFIG_N32H7_N32H76X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Register Index Calculation (Each register manages 32 EXTI lines) */
#define N32_EXTI_INDEX(n)        ((n) >> 5)               /* Calculate register index (0-1) */
#define N32_EXTI_SHIFT(n)        ((n) & 0x1f)             /* Calculate bit shift (0-31) */
#define N32_EXTI_MASK(n)         (1 << N32_EXTI_SHIFT(n)) /* Generate bit mask */

/* Dynamic Offset Calculation (For RT/FT/SWIE register groups) */
#define N32_EXTI_RT_CFG_OFFSET(n)      (0x0000 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_FT_CFG_OFFSET(n)      (0x0020 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_SWIE_OFFSET(n)        (0x0040 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M7IMASK_OFFSET(n)     (0x0060 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M4IMASK_OFFSET(n)     (0x0080 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M7EMASK_OFFSET(n)     (0x00A0 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M4EMASK_OFFSET(n)     (0x00C0 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M7PEND_OFFSET(n)      (0x00E0 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M4PEND_OFFSET(n)      (0x0100 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M7IMASK_DRC_OFFSET(n) (0x0120 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M4IMASK_DRC_OFFSET(n) (0x0140 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M7EMASK_DRC_OFFSET(n) (0x0160 + (N32_EXTI_INDEX(n) * 4))
#define N32_EXTI_M4EMASK_DRC_OFFSET(n) (0x0180 + (N32_EXTI_INDEX(n) * 4))

/* Static Offset Definitions */
#define N32_EXTI_RT_CFG0_OFFSET      0x0000  /* Rising Trigger Config 0 (0-31) */
#define N32_EXTI_RT_CFG1_OFFSET      0x0004  /* Rising Trigger Config 1 (32-51) */
#define N32_EXTI_FT_CFG0_OFFSET      0x0020  /* Falling Trigger Config 0 (0-31) */
#define N32_EXTI_FT_CFG1_OFFSET      0x0024  /* Falling Trigger Config 1 (32-51) */
#define N32_EXTI_SWIE0_OFFSET        0x0040  /* SW Interrupt Enable 0 (0-31) */
#define N32_EXTI_SWIE1_OFFSET        0x0044  /* SW Interrupt Enable 1 (32-51) */
#define N32_EXTI_M7IMASK0_OFFSET     0x0060  /* CM7 Interrupt Mask 0 (0-31) */
#define N32_EXTI_M7IMASK1_OFFSET     0x0064  /* CM7 Interrupt Mask 1 (32-51) */
#define N32_EXTI_M4IMASK0_OFFSET     0x0080  /* CM4 Interrupt Mask 0 (0-31) */
#define N32_EXTI_M4IMASK1_OFFSET     0x0084  /* CM4 Interrupt Mask 1 (32-51) */
#define N32_EXTI_M7EMASK0_OFFSET     0x00A0  /* CM7 Event Mask 0 (0-31) */
#define N32_EXTI_M7EMASK1_OFFSET     0x00A4  /* CM7 Event Mask 1 (32-51) */
#define N32_EXTI_M4EMASK0_OFFSET     0x00C0  /* CM4 Event Mask 0 (0-31) */
#define N32_EXTI_M4EMASK1_OFFSET     0x00C4  /* CM4 Event Mask 1 (32-51) */
#define N32_EXTI_M7PEND0_OFFSET      0x00E0  /* CM7 Pending 0 (0-31) */
#define N32_EXTI_M7PEND1_OFFSET      0x00E4  /* CM7 Pending 1 (32-51) */
#define N32_EXTI_M4PEND0_OFFSET      0x0100  /* CM4 Pending 0 (0-31) */
#define N32_EXTI_M4PEND1_OFFSET      0x0104  /* CM4 Pending 1 (32-51) */
#define N32_EXTI_M7IMASK0_DRC_OFFSET 0x0120  /* CM7 Direct Event Int Mask 0 (0-31) */
#define N32_EXTI_M7IMASK1_DRC_OFFSET 0x0124  /* CM7 Direct Event Int Mask 1 (32-58) */
#define N32_EXTI_M4IMASK0_DRC_OFFSET 0x0140  /* CM4 Direct Event Int Mask 0 (0-31) */
#define N32_EXTI_M4IMASK1_DRC_OFFSET 0x0144  /* CM4 Direct Event Int Mask 1 (32-58) */
#define N32_EXTI_M7EMASK0_DRC_OFFSET 0x0160  /* CM7 Direct Event Mask 0 (0-31) */
#define N32_EXTI_M7EMASK1_DRC_OFFSET 0x0164  /* CM7 Direct Event Mask 1 (32-58) */
#define N32_EXTI_M4EMASK0_DRC_OFFSET 0x0180  /* CM4 Direct Event Mask 0 (0-31) */
#define N32_EXTI_M4EMASK1_DRC_OFFSET 0x0184  /* CM4 Direct Event Mask 1 (32-58) */
#define N32_EXTI_TSSEL_OFFSET        0x01C0  /* Timestamp Select */

/* Register Addresses *******************************************************/

/* Dynamic Address Calculation (For RT/FT/SWIE register groups) */
#define N32H7_EXTI_RT_CFG(n)         (N32_EXTI_WKUP_BASE + N32_EXTI_RT_CFG_OFFSET(n))
#define N32H7_EXTI_FT_CFG(n)         (N32_EXTI_WKUP_BASE + N32_EXTI_FT_CFG_OFFSET(n))
#define N32H7_EXTI_SWIE(n)           (N32_EXTI_WKUP_BASE + N32_EXTI_SWIE_OFFSET(n))
#define N32H7_EXTI_M7IMASK(n)        (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK_OFFSET(n))
#define N32H7_EXTI_M4IMASK(n)        (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK_OFFSET(n))
#define N32H7_EXTI_M7EMASK(n)        (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK_OFFSET(n))
#define N32H7_EXTI_M4EMASK(n)        (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK_OFFSET(n))
#define N32H7_EXTI_M7PEND(n)         (N32_EXTI_WKUP_BASE + N32_EXTI_M7PEND_OFFSET(n))
#define N32H7_EXTI_M4PEND(n)         (N32_EXTI_WKUP_BASE + N32_EXTI_M4PEND_OFFSET(n))
#define N32H7_EXTI_M7IMASK_DRC(n)    (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK_DRC_OFFSET(n))
#define N32H7_EXTI_M4IMASK_DRC(n)    (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK_DRC_OFFSET(n))
#define N32H7_EXTI_M7EMASK_DRC(n)    (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK_DRC_OFFSET(n))
#define N32H7_EXTI_M4EMASK_DRC(n)    (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK_DRC_OFFSET(n))

/* Static Address Definitions */
#define N32_EXTI_RT_CFG0             (N32_EXTI_WKUP_BASE + N32_EXTI_RT_CFG0_OFFSET)
#define N32_EXTI_RT_CFG1             (N32_EXTI_WKUP_BASE + N32_EXTI_RT_CFG1_OFFSET)
#define N32_EXTI_FT_CFG0             (N32_EXTI_WKUP_BASE + N32_EXTI_FT_CFG0_OFFSET)
#define N32_EXTI_FT_CFG1             (N32_EXTI_WKUP_BASE + N32_EXTI_FT_CFG1_OFFSET)
#define N32_EXTI_SWIE0               (N32_EXTI_WKUP_BASE + N32_EXTI_SWIE0_OFFSET)
#define N32_EXTI_SWIE1               (N32_EXTI_WKUP_BASE + N32_EXTI_SWIE1_OFFSET)
#define N32_EXTI_M7IMASK0            (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK0_OFFSET)
#define N32_EXTI_M7IMASK1            (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK1_OFFSET)
#define N32_EXTI_M4IMASK0            (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK0_OFFSET)
#define N32_EXTI_M4IMASK1            (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK1_OFFSET)
#define N32_EXTI_M7EMASK0            (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK0_OFFSET)
#define N32_EXTI_M7EMASK1            (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK1_OFFSET)
#define N32_EXTI_M4EMASK0            (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK0_OFFSET)
#define N32_EXTI_M4EMASK1            (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK1_OFFSET)
#define N32_EXTI_M7PEND0             (N32_EXTI_WKUP_BASE + N32_EXTI_M7PEND0_OFFSET)
#define N32_EXTI_M7PEND1             (N32_EXTI_WKUP_BASE + N32_EXTI_M7PEND1_OFFSET)
#define N32_EXTI_M4PEND0             (N32_EXTI_WKUP_BASE + N32_EXTI_M4PEND0_OFFSET)
#define N32_EXTI_M4PEND1             (N32_EXTI_WKUP_BASE + N32_EXTI_M4PEND1_OFFSET)
#define N32_EXTI_M7IMASK0_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK0_DRC_OFFSET)
#define N32_EXTI_M7IMASK1_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M7IMASK1_DRC_OFFSET)
#define N32_EXTI_M4IMASK0_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK0_DRC_OFFSET)
#define N32_EXTI_M4IMASK1_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M4IMASK1_DRC_OFFSET)
#define N32_EXTI_M7EMASK0_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK0_DRC_OFFSET)
#define N32_EXTI_M7EMASK1_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M7EMASK1_DRC_OFFSET)
#define N32_EXTI_M4EMASK0_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK0_DRC_OFFSET)
#define N32_EXTI_M4EMASK1_DRC        (N32_EXTI_WKUP_BASE + N32_EXTI_M4EMASK1_DRC_OFFSET)
#define N32_EXTI_TSSEL               (N32_EXTI_WKUP_BASE + N32_EXTI_TSSEL_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define EXTI_EVENT(n)               N32_EXTI_MASK(n)

/* EXTI_RT_CFGx / EXTI_FT_CFGx */
#define EXTI_CFG_ENABLE                1
#define EXTI_CFG_DISABLE               0

/* EXTI_SWIEx */
#define EXTI_SWIE_TRIGGER              1

/* Mask Registers (MxIMASK, MxEMASK, etc.) */
#define EXTI_MASK_UNMASKED             1
#define EXTI_MASK_MASKED               0

/* Pending Registers (MxPEND) */
#define EXTI_PEND_ACTIVE               1
#define EXTI_PEND_INACTIVE             0

/* EXTI_TSSEL */
#define EXTI_TSSEL_SHIFT               (8)
#define EXTI_TSSEL_MASK                (0xF << EXTI_TSSEL_SHIFT)
#define EXTI_TSSEL_LINE(n)             (((n) & 0xF) << EXTI_TSSEL_SHIFT)

/* EXTI event input mapping *************************************************/

#define EXTI_EVENT_EXTI(n)             (1 << (n))/* Base EXTI line (0-15) */

/* Peripheral Specific Event Lines (16-31) */
#define EXTI_EVENT_PVDAVD               16        /* PVD and AVD event */
#define EXTI_EVENT_RTC_ALARM            17        /* RTC alarm event */
#define EXTI_EVENT_RTC_MULTI            18        /* RTC intrusion/LSECSS/LSECSS overflow/LSICSS */
#define EXTI_EVENT_RTC_WAKEUP           19        /* RTC wakeup timer event */
#define EXTI_EVENT_COMP1                20        /* Comparator1 output */
#define EXTI_EVENT_COMP2                21        /* Comparator2 output */
#define EXTI_EVENT_COMP3                22        /* Comparator3 output */
#define EXTI_EVENT_SDMMC1_WAKEUP        24        /* SDMMC1 wakeup event */
#define EXTI_EVENT_SDMMC2_WAKEUP        25        /* SDMMC2 wakeup event */
#define EXTI_EVENT_LPUART1_WAKEUP       49        /* LPUART1 wakeup event */
#define EXTI_EVENT_DCMUA_INT            51        /* DCMUA interrupt event */
#define EXTI_EVENT_LPUART2_WAKEUP       52        /* LPUART2 wakeup event */
#define EXTI_EVENT_DCMUB_INT            54        /* DCMUB interrupt event */
#define EXTI_EVENT_CM7_AHBS_ABORT       55        /* CM7 AHBS_ABORT event */
#define EXTI_EVENT_CM7_AHBSRDY_ERROR    56        /* CM7 AHBSRDY_ERROR event */
#define EXTI_EVENT_USB1_HS_WAKEUP       62        /* USB1_HS wakeup event */
#define EXTI_EVENT_USB2_HS_WAKEUP       63        /* USB2_HS wakeup event */
#define EXTI_EVENT_CM7_CACHE_READ_CER   64        /* CM7 cache read correctable error event */
#define EXTI_EVENT_CM7_CACHE_READ_FER   65        /* CM7 cache read fatal error event */
#define EXTI_EVENT_LPTIM1_WAKEUP        66        /* LPTIM1 wakeup event */
#define EXTI_EVENT_LPTIM2_WAKEUP        67        /* LPTIM2 wakeup event */
#define EXTI_EVENT_LPTIM3_WAKEUP        68        /* LPTIM3 wakeup event */
#define EXTI_EVENT_LPTIM4_WAKEUP        69        /* LPTIM4 wakeup event */
#define EXTI_EVENT_WKUP1                70        /* Wakeup pin 1 event */
#define EXTI_EVENT_WKUP2                71        /* Wakeup pin 2 event */
#define EXTI_EVENT_WKUP3                72        /* Wakeup pin 3 event */
#define EXTI_EVENT_WKUP4                73        /* Wakeup pin 4 event */
#define EXTI_EVENT_WKUP5                74        /* Wakeup pin 5 event */
#define EXTI_EVENT_WKUP6                75        /* Wakeup pin 6 event */
#define EXTI_EVENT_RCC_INT              76        /* RCC interrupt event */
#define EXTI_EVENT_SEMA1_INT            77        /* SEMA1 interrupt event */
#define EXTI_EVENT_SEMA2_INT            78        /* SEMA2 interrupt event */
#define EXTI_EVENT_CM4_SEV_INT          79        /* Cortex-M4 SEV interrupt event */
#define EXTI_EVENT_CM7_SEV_INT          80        /* Cortex-M7 SEV interrupt event */
#define EXTI_EVENT_WWDG1_RESET          81        /* WWDG1 reset */
#define EXTI_EVENT_WWDG2_RESET          82        /* WWDG2 reset */
#define EXTI_EVENT_ETH1_WAKEUP          83        /* ETHERNET1 wakeup(LPI+PMT) */
#define EXTI_EVENT_ETH2_WAKEUP          84        /* ETHERNET2 wakeup(LPI+PMT) */
#define EXTI_EVENT_HSECSS_INT           85        /* HSECSS interrupt event */
#define EXTI_EVENT_LPTIMER5_WAKEUP      86        /* LPTIMER5 wakeup event */
#define EXTI_EVENT_DSI_ERROR            87        /* DSI error event */
#define EXTI_EVENT_BKP_EMC              88        /* BKP EMC event */
#define EXTI_EVENT_VDDD_EMC             89        /* VDDD EMC event */

#endif /* CONFIG_ARCH_CHIP_N32H7 */
#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32_EXTI_H */
