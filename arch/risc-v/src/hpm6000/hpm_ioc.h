/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_ioc.h
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

 #ifndef __ARCH_RISCV_SRC_HPM6000_HPM_IOC_H
 #define __ARCH_RISCV_SRC_HPM6000_HPM_IOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/hpm_memorymap.h"
#include "hardware/hpm_ioc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16-bit Encoding:
 *
 *   AAAA ARRR ODDD LSST
 */

/* Peripheral Alternate Function:
 * AAAA A... .... ....
 */

#define PAD_ALT_SHIFT          (11)      /* Bits 11-15: Peripheral alternate function */
#define PAD_ALT_MASK           (0x1f << PAD_ALT_SHIFT)
#  define PAD_ALT0             (0 << PAD_ALT_SHIFT)  
#  define PAD_ALT1             (1 << PAD_ALT_SHIFT)
#  define PAD_ALT2             (2 << PAD_ALT_SHIFT)
#  define PAD_ALT3             (3 << PAD_ALT_SHIFT)
#  define PAD_ALT4             (4 << PAD_ALT_SHIFT)
#  define PAD_ALT5             (5 << PAD_ALT_SHIFT)
#  define PAD_ALT6             (6 << PAD_ALT_SHIFT)
#  define PAD_ALT7             (7 << PAD_ALT_SHIFT)
#  define PAD_ALT8             (8 << PAD_ALT_SHIFT)
#  define PAD_ALT9             (9 << PAD_ALT_SHIFT)
#  define PAD_ALT10            (10 << PAD_ALT_SHIFT)
#  define PAD_ALT11            (11 << PAD_ALT_SHIFT)
#  define PAD_ALT12            (12 << PAD_ALT_SHIFT)
#  define PAD_ALT13            (13 << PAD_ALT_SHIFT)
#  define PAD_ALT14            (14 << PAD_ALT_SHIFT)
#  define PAD_ALT15            (15 << PAD_ALT_SHIFT)
#  define PAD_ALT16            (16 << PAD_ALT_SHIFT)
#  define PAD_ALT17            (17 << PAD_ALT_SHIFT)
#  define PAD_ALT18            (18 << PAD_ALT_SHIFT)
#  define PAD_ALT19            (19 << PAD_ALT_SHIFT)
#  define PAD_ALT20            (20 << PAD_ALT_SHIFT)
#  define PAD_ALT21            (21 << PAD_ALT_SHIFT)
#  define PAD_ALT22            (22 << PAD_ALT_SHIFT)
#  define PAD_ALT23            (23 << PAD_ALT_SHIFT)
#  define PAD_ALT24            (24 << PAD_ALT_SHIFT)
#  define PAD_ALT25            (25 << PAD_ALT_SHIFT)
#  define PAD_ALT26            (26 << PAD_ALT_SHIFT)
#  define PAD_ALT27            (27 << PAD_ALT_SHIFT)
#  define PAD_ALT28            (28 << PAD_ALT_SHIFT)
#  define PAD_ALT29            (29 << PAD_ALT_SHIFT)
#  define PAD_ALT30            (30 << PAD_ALT_SHIFT)
#  define PAD_ALT31            (31 << PAD_ALT_SHIFT)

/* Output Pull Up/Down:
 *
 *   .... RRRR .... ....
 */

#define _PAD_PULLTYPE_SHIFT   (8)        /* Bits 8: Pull up/down type */
#define _PAD_PULLTYPE_MASK    (1 << _PAD_PULLTYPE_SHIFT)
#  define _PAD_PULL_KEEP      (0 << _PAD_PULLTYPE_SHIFT) /* Output determined by keeper */
#  define _PAD_PULL_ENABLE    (1 << _PAD_PULLTYPE_SHIFT) /* Output pulled up or down */

#define _PAD_PULLDESC_SHIFT   (9)       /* Bits 9-10: Pull up/down description */
#define _PAD_PULLDESC_MASK    (3 << _PAD_PULLDESC_SHIFT)
#  define _PAD_PULL_UP_22K    (PULL_UP_22K    << _PAD_PULLDESC_SHIFT) /* Pull up with 22 KOhm resister */
#  define _PAD_PULL_UP_47K    (PULL_UP_47K    << _PAD_PULLDESC_SHIFT) /* Pull up with 47 KOhm resister */
#  define _PAD_PULL_UP_100K   (PULL_UP_100K   << _PAD_PULLDESC_SHIFT) /* Pull up with 100 KOhm resister */
#  define _PAD_PULL_DOWN_100K (PULL_DOWN_100K << _PAD_PULLDESC_SHIFT) /* Pull down with 100 KOhm resister */

#define PAD_PULL_SHIFT        (8)        /* Bits 8-10: Pull up/down selection */
#define PAD_PULL_MASK         (15 << PAD_PULL_SHIFT)
#  define PAD_PULL_KEEP       _PAD_PULL_KEEP
#  define PAD_PULL_UP_22K     (_PAD_PULL_ENABLE | _PAD_PULL_UP_22K)
#  define PAD_PULL_UP_47K     (_PAD_PULL_ENABLE | _PAD_PULL_UP_47K)
#  define PAD_PULL_UP_100K    (_PAD_PULL_ENABLE | _PAD_PULL_UP_100K)
#  define PAD_PULL_DOWN_100K  (_PAD_PULL_ENABLE | _PAD_PULL_DOWN_100K)

/* Open Drain Output:
 *
 *   .... .... O... ....
 */

#define PAD_CMOS_OUTPUT       (0)       /* Bit 7: 0=CMOS output */
#define PAD_OPENDRAIN         (1 << 7)  /* Bit 7: 1=Enable open-drain output */

/* Output Drive Strength:
 *
 *   .... .... .DDD ....
 */

#define PAD_DRIVE_SHIFT       (4)       /* Bits 4-6: Output Drive Strength */
#define PAD_DRIVE_MASK        (7 << PAD_DRIVE_SHIFT)
#  define PAD_DRIVE_260OHM    (DRIVE_260OHM << PAD_DRIVE_SHIFT) /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define PAD_DRIVE_130OHM    (DRIVE_130OHM << PAD_DRIVE_SHIFT) /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define PAD_DRIVE_88OHM     (DRIVE_88OHM  << PAD_DRIVE_SHIFT) /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define PAD_DRIVE_65OHM     (DRIVE_65OHM  << PAD_DRIVE_SHIFT) /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define PAD_DRIVE_52OHM     (DRIVE_52OHM  << PAD_DRIVE_SHIFT) /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define PAD_DRIVE_43OHM     (DRIVE_43OHM  << PAD_DRIVE_SHIFT) /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define PAD_DRIVE_37OHM     (DRIVE_37OHM  << PAD_DRIVE_SHIFT) /* 20 Ohm @3.3V, 33 Ohm @1.8V */

/* Analog pin
 *
 *   .... .... .DDD ....
 */
#define FUNC_ANALOG_SHIFT     (4)
#define FUNC_ANALOG_MASK      (7 << FUNC_ANALOG_SHIFT)
#define FUNC_ANALOG           (1 << FUNC_ANALOG_SHIFT)

/* Output Slew Rate:
 *
 *   .... .... .... L...
 */

#define PAD_SLEW_SLOW         (0)       /* Bit 3: 0=Slow Slew Rate */
#define PAD_SLEW_FAST         (1 << 3)  /* Bit 3: 1=Fast Slew Rate */

/* Output Speed:
 *
 *   .... .... .... .SS.
 */

#define PAD_SPEED_SHIFT       (1)       /* Bits 1-2: Speed */
#define PAD_SPEED_MASK        (3 << PAD_SPEED_SHIFT)
#  define PAD_SPEED_SLOW      (SPEED_SLOW   << PAD_SPEED_SHIFT) /* Low frequency (50 MHz) */
#  define PAD_SPEED_MEDIUM    (SPEED_MEDIUM << PAD_SPEED_SHIFT) /* Medium frequency (100, 150 MHz) */
#  define PAD_SPEED_FAST      (SPEED_FAST   << PAD_SPEED_SHIFT) /* Fast frequency */
#  define PAD_SPEED_MAX       (SPEED_MAX    << PAD_SPEED_SHIFT) /* Maximum frequency (100, 150, 200 MHz) */

/* Input Schmitt Trigger:
 *
 *   .... .... .... ...T
 */

#define PAD_CMOS_INPUT        (0)       /* Bit 0: 0=CMOS input */
#define PAD_SCHMITT_TRIGGER   (1 << 0)  /* Bit 0: 1=Enable Schmitt trigger if input */

#define IOC_FUNC_CTL_(port, pin)          (HPM_IOC_BASE + port * 0x0100 + pin * 0x0008 + 0x0000)
#define IOC_PAD_CTL_(port, pin)           (HPM_IOC_BASE + port * 0x0100 + pin * 0x0008 + 0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the IOMUX encoding */

typedef uint16_t ioc_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

unsigned int hpm_iocpad_map(unsigned int iocpad);
int hpm_iocpad_configure(uintptr_t padctl, ioc_pinset_t ioset);

#endif /* __ARCH_RISCV_SRC_HPM6000_HPM_IOC_H */
