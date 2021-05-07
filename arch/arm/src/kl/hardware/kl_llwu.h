/****************************************************************************
 * arch/arm/src/kl/hardware/kl_llwu.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_LLWU_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_LLWU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KL_LLWU_PE1_OFFSET         0x0000 /* LLWU Pin Enable 1 Register */
#define KL_LLWU_PE2_OFFSET         0x0001 /* LLWU Pin Enable 2 Register */
#define KL_LLWU_PE3_OFFSET         0x0002 /* LLWU Pin Enable 3 Register */
#define KL_LLWU_PE4_OFFSET         0x0003 /* LLWU Pin Enable 4 Register */
#define KL_LLWU_ME_OFFSET          0x0004 /* LLWU Module Enable Register */
#define KL_LLWU_F1_OFFSET          0x0005 /* LLWU Flag 1 Register */
#define KL_LLWU_F2_OFFSET          0x0006 /* LLWU Flag 2 Register */
#define KL_LLWU_F3_OFFSET          0x0007 /* LLWU Flag 3 Register */
#define KL_LLWU_CS_OFFSET          0x0008 /* LLWU Control and Status Register */

/* Register Addresses *******************************************************/

#define KL_LLWU_PE1                (KL_LLWU_BASE+KL_LLWU_PE1_OFFSET)
#define KL_LLWU_PE2                (KL_LLWU_BASE+KL_LLWU_PE2_OFFSET)
#define KL_LLWU_PE3                (KL_LLWU_BASE+KL_LLWU_PE3_OFFSET)
#define KL_LLWU_PE4                (KL_LLWU_BASE+KL_LLWU_PE4_OFFSET)
#define KL_LLWU_ME                 (KL_LLWU_BASE+KL_LLWU_ME_OFFSET)
#define KL_LLWU_F1                 (KL_LLWU_BASE+KL_LLWU_F1_OFFSET)
#define KL_LLWU_F2                 (KL_LLWU_BASE+KL_LLWU_F2_OFFSET)
#define KL_LLWU_F3                 (KL_LLWU_BASE+KL_LLWU_F3_OFFSET)
#define KL_LLWU_CS                 (KL_LLWU_BASE+KL_LLWU_CS_OFFSET)

/* Register Bit Definitions *************************************************/

/* LLWU Pin Enable 1 Register */

#define LLWU_PE1_WUPE0_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P0 */
#define LLWU_PE1_WUPE0_MASK        (3 << LLWU_PE1_WUPE0_SHIFT)
#  define LLWU_PE1_WUPE0_DISABLED  (0 << LLWU_PE1_WUPE0_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE0_RISING    (1 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE0_FALLING   (2 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE0_BOTH      (3 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE1_WUPE1_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P1 */
#define LLWU_PE1_WUPE1_MASK        (3 << LLWU_PE1_WUPE1_SHIFT)
#  define LLWU_PE1_WUPE1_DISABLED  (0 << LLWU_PE1_WUPE1_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE1_RISING    (1 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE1_FALLING   (2 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE1_BOTH      (3 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE1_WUPE2_SHIFT       (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P2 */
#define LLWU_PE1_WUPE2_MASK        (3 << LLWU_PE1_WUPE2_SHIFT)
#  define LLWU_PE1_WUPE2_DISABLED  (0 << LLWU_PE1_WUPE2_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE2_RISING    (1 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE2_FALLING   (2 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE2_BOTH      (3 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE1_WUPE3_SHIFT       (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P3 */
#define LLWU_PE1_WUPE3_MASK        (3 << LLWU_PE1_WUPE3_SHIFT)
#  define LLWU_PE1_WUPE3_DISABLED  (0 << LLWU_PE1_WUPE3_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE3_RISING    (1 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE3_FALLING   (2 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE3_BOTH      (3 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 2 Register */

#define LLWU_PE2_WUPE4_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P4 */
#define LLWU_PE2_WUPE4_MASK        (3 << LLWU_PE2_WUPE4_SHIFT)
#  define LLWU_PE2_WUPE4_DISABLED  (0 << LLWU_PE2_WUPE4_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE4_RISING    (1 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE4_FALLING   (2 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE4_BOTH      (3 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE2_WUPE5_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P5 */
#define LLWU_PE2_WUPE5_MASK        (3 << LLWU_PE2_WUPE5_SHIFT)
#  define LLWU_PE2_WUPE5_DISABLED  (0 << LLWU_PE2_WUPE5_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE5_RISING    (1 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE5_FALLING   (2 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE5_BOTH      (3 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE2_WUPE6_SHIFT       (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P6 */
#define LLWU_PE2_WUPE6_MASK        (3 << LLWU_PE2_WUPE6_SHIFT)
#  define LLWU_PE2_WUPE6_DISABLED  (0 << LLWU_PE2_WUPE6_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE6_RISING    (1 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE6_FALLING   (2 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE6_BOTH      (3 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE2_WUPE7_SHIFT       (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P7 */
#define LLWU_PE2_WUPE7_MASK        (3 << LLWU_PE2_WUPE7_SHIFT)
#  define LLWU_PE2_WUPE7_DISABLED  (0 << LLWU_PE2_WUPE7_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE7_RISING    (1 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE7_FALLING   (2 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE7_BOTH      (3 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 3 Register */

#define LLWU_PE3_WUPE8_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P8 */
#define LLWU_PE3_WUPE8_MASK        (3 << LLWU_PE3_WUPE8_SHIFT)
#  define LLWU_PE3_WUPE8_DISABLED  (0 << LLWU_PE3_WUPE8_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE8_RISING    (1 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE8_FALLING   (2 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE8_BOTH      (3 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE3_WUPE9_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P9 */
#define LLWU_PE3_WUPE9_MASK        (3 << LLWU_PE3_WUPE9_SHIFT)
#  define LLWU_PE3_WUPE9_DISABLED  (0 << LLWU_PE3_WUPE9_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE9_RISING    (1 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE9_FALLING   (2 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE9_BOTH      (3 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE3_WUPE10_SHIFT      (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P10 */
#define LLWU_PE3_WUPE10_MASK       (3 << LLWU_PE3_WUPE10_SHIFT)
#  define LLWU_PE3_WUPE10_DISABLED (0 << LLWU_PE3_WUPE10_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE10_RISING   (1 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE10_FALLING  (2 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE10_BOTH     (3 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE3_WUPE11_SHIFT      (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P11 */
#define LLWU_PE3_WUPE11_MASK       (3 << LLWU_PE3_WUPE11_SHIFT)
#  define LLWU_PE3_WUPE11_DISABLED (0 << LLWU_PE3_WUPE11_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE11_RISING   (1 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE11_FALLING  (2 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE11_BOTH     (3 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 4 Register */

#define LLWU_PE4_WUPE12_SHIFT      (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P12 */
#define LLWU_PE4_WUPE12_MASK       (3 << LLWU_PE4_WUPE12_SHIFT)
#  define LLWU_PE4_WUPE12_DISABLED (0 << LLWU_PE4_WUPE12_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE12_RISING   (1 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE12_FALLING  (2 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE12_BOTH     (3 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE4_WUPE13_SHIFT      (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P13 */
#define LLWU_PE4_WUPE13_MASK       (3 << LLWU_PE4_WUPE13_SHIFT)
#  define LLWU_PE4_WUPE13_DISABLED (0 << LLWU_PE4_WUPE13_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE13_RISING   (1 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE13_FALLING  (2 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE13_BOTH     (3 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE4_WUPE14_SHIFT      (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P14 */
#define LLWU_PE4_WUPE14_MASK       (3 << LLWU_PE4_WUPE14_SHIFT)
#  define LLWU_PE4_WUPE14_DISABLED (0 << LLWU_PE4_WUPE14_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE14_RISING   (1 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE14_FALLING  (2 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE14_BOTH     (3 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for any change */

#define LLWU_PE4_WUPE15_SHIFT      (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P15 */
#define LLWU_PE4_WUPE15_MASK       (3 << LLWU_PE4_WUPE15_SHIFT)
#  define LLWU_PE4_WUPE15_DISABLED (0 << LLWU_PE4_WUPE15_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE15_RISING   (1 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE15_FALLING  (2 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE15_BOTH     (3 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for any change */

/* LLWU Module Enable Register */

#define LLWU_ME_WUME(n)            (1 << (n))
#define LLWU_ME_WUME0              (1 << 0)  /* Bit 0:  Wakeup Module Enable for Module 0 */
#define LLWU_ME_WUME1              (1 << 1)  /* Bit 1:  Wakeup Module Enable for Module 1 */
#define LLWU_ME_WUME2              (1 << 2)  /* Bit 2:  Wakeup Module Enable for Module 2 */
#define LLWU_ME_WUME3              (1 << 3)  /* Bit 3:  Wakeup Module Enable for Module 3 */
#define LLWU_ME_WUME4              (1 << 4)  /* Bit 4:  Wakeup Module Enable for Module 4 */
#define LLWU_ME_WUME5              (1 << 5)  /* Bit 5:  Wakeup Module Enable for Module 5 */
#define LLWU_ME_WUME6              (1 << 6)  /* Bit 6:  Wakeup Module Enable for Module 6 */
#define LLWU_ME_WUME7              (1 << 7)  /* Bit 7:  Wakeup Module Enable for Module 7 */

/* LLWU Flag 1 Register */

#define LLWU_F1_WUF(n)             (1 << (n))
#define LLWU_F1_WUF0               (1 << 0)  /* Bit 0:  Wakeup Flag for LLWU_P0 */
#define LLWU_F1_WUF1               (1 << 1)  /* Bit 1:  Wakeup Flag for LLWU_P1 */
#define LLWU_F1_WUF2               (1 << 2)  /* Bit 2:  Wakeup Flag for LLWU_P2 */
#define LLWU_F1_WUF3               (1 << 3)  /* Bit 3:  Wakeup Flag for LLWU_P3 */
#define LLWU_F1_WUF4               (1 << 4)  /* Bit 4:  Wakeup Flag for LLWU_P4 */
#define LLWU_F1_WUF5               (1 << 5)  /* Bit 5:  Wakeup Flag for LLWU_P5 */
#define LLWU_F1_WUF6               (1 << 6)  /* Bit 6:  Wakeup Flag for LLWU_P6 */
#define LLWU_F1_WUF7               (1 << 7)  /* Bit 7:  Wakeup Flag for LLWU_P7 */

/* LLWU Flag 2 Register */

#define LLWU_F2_WUF(n)             (1 << ((n)-8))
#define LLWU_F2_WUF8               (1 << 8)  /* Bit 0:  Wakeup Flag for LLWU_P8 */
#define LLWU_F2_WUF9               (1 << 9)  /* Bit 1:  Wakeup Flag for LLWU_P9 */
#define LLWU_F2_WUF10              (1 << 10) /* Bit 2:  Wakeup Flag for LLWU_P10 */
#define LLWU_F2_WUF11              (1 << 11) /* Bit 3:  Wakeup Flag for LLWU_P11 */
#define LLWU_F2_WUF12              (1 << 12) /* Bit 4:  Wakeup Flag for LLWU_P12 */
#define LLWU_F2_WUF13              (1 << 13) /* Bit 5:  Wakeup Flag for LLWU_P13 */
#define LLWU_F2_WUF14              (1 << 14) /* Bit 6:  Wakeup Flag for LLWU_P14 */
#define LLWU_F2_WUF15              (1 << 15) /* Bit 7:  Wakeup Flag for LLWU_P15 */

/* LLWU Flag 3 Register */

#define LLWU_F3_MWUF(n)            (1 << (n))
#define LLWU_F3_MWUF0              (1 << 0)  /* Bit 0:  Wakeup flag for module 0 */
#define LLWU_F3_MWUF1              (1 << 1)  /* Bit 1:  Wakeup flag for module 1 */
#define LLWU_F3_MWUF2              (1 << 2)  /* Bit 2:  Wakeup flag for module 2 */
#define LLWU_F3_MWUF3              (1 << 3)  /* Bit 3:  Wakeup flag for module 3 */
#define LLWU_F3_MWUF4              (1 << 4)  /* Bit 4:  Wakeup flag for module 4 */
#define LLWU_F3_MWUF5              (1 << 5)  /* Bit 5:  Wakeup flag for module 5 */
#define LLWU_F3_MWUF6              (1 << 6)  /* Bit 6:  Wakeup flag for module 6 */
#define LLWU_F3_MWUF7              (1 << 7)  /* Bit 7:  Wakeup flag for module 7 */

/* LLWU Control and Status Register */

#define LLWU_CS_ACKISO             (1 << 7)  /* Bit 7:  Acknowledge Isolation */
                                             /* Bits 2-6: Reserved */
#define LLWU_CS_FLTEP              (1 << 1)  /* Bit 1:  Digital Filter on External Pin */
#define LLWU_CS_FLTR               (1 << 0)  /* Bit 0:  Digital Filter on RESET Pin */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_LLWU_H */
