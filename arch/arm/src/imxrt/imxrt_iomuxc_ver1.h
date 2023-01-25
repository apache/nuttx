/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver1.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER1_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16-bit Encoding:
 *
 *   .... RRRR ODDD LSST
 */

/* Output Pull Up/Down:
 *
 *   .... RRRR .... ....
 */

#define _IOMUX_PULLTYPE_SHIFT             (8)       /* Bits 8-9:   Pull up/down type */
#define _IOMUX_PULLTYPE_MASK              (0x03 << _IOMUX_PULLTYPE_SHIFT)
#  define _IOMUX_PULL_NONE                (0x00 << _IOMUX_PULLTYPE_SHIFT) /* Pull/keeper disabled */
#  define _IOMUX_PULL_KEEP                (0x01 << _IOMUX_PULLTYPE_SHIFT) /* Output determined by keeper */
#  define _IOMUX_PULL_ENABLE              (0x02 << _IOMUX_PULLTYPE_SHIFT) /* Output pulled up or down */

#define _IOMUX_PULLDESC_SHIFT             (10)      /* Bits 10-11: Pull up/down description */
#define _IOMUX_PULLDESC_MASK              (0x03 << _IOMUX_PULLDESC_SHIFT)
#  define _IOMUX_PULL_UP_22K              (PULL_UP_22K    << _IOMUX_PULLDESC_SHIFT) /* Pull up with 22 KOhm resister */
#  define _IOMUX_PULL_UP_47K              (PULL_UP_47K    << _IOMUX_PULLDESC_SHIFT) /* Pull up with 47 KOhm resister */
#  define _IOMUX_PULL_UP_100K             (PULL_UP_100K   << _IOMUX_PULLDESC_SHIFT) /* Pull up with 100 KOhm resister */
#  define _IOMUX_PULL_DOWN_100K           (PULL_DOWN_100K << _IOMUX_PULLDESC_SHIFT) /* Pull down with 100 KOhm resister */

#define IOMUX_PULL_SHIFT                  (8)       /* Bits 8-11:  Pull up/down selection */
#define IOMUX_PULL_MASK                   (0x0f << IOMUX_PULL_SHIFT)
#  define IOMUX_PULL_NONE                 (_IOMUX_PULL_NONE)
#  define IOMUX_PULL_KEEP                 (_IOMUX_PULL_KEEP)
#  define IOMUX_PULL_UP_22K               (_IOMUX_PULL_ENABLE | _IOMUX_PULL_UP_22K)
#  define IOMUX_PULL_UP_47K               (_IOMUX_PULL_ENABLE | _IOMUX_PULL_UP_47K)
#  define IOMUX_PULL_UP_100K              (_IOMUX_PULL_ENABLE | _IOMUX_PULL_UP_100K)
#  define IOMUX_PULL_DOWN_100K            (_IOMUX_PULL_ENABLE | _IOMUX_PULL_DOWN_100K)

/* Open Drain Output:
 *
 *   .... .... O... ....
 */

#define IOMUX_CMOS_OUTPUT                 (0)       /* Bit 7: 0=CMOS output */
#define IOMUX_OPENDRAIN                   (1 << 7)  /* Bit 7: 1=Enable open-drain output */

/* Output Drive Strength:
 *
 *   .... .... .DDD ....
 */

#define IOMUX_DRIVE_SHIFT                 (4)       /* Bits 4-6: Output Drive Strength */
#define IOMUX_DRIVE_MASK                  (0x07 << IOMUX_DRIVE_SHIFT)
#  define IOMUX_DRIVE_HIZ                 (DRIVE_HIZ    << IOMUX_DRIVE_SHIFT) /* HI-Z */
#  define IOMUX_DRIVE_260OHM              (DRIVE_260OHM << IOMUX_DRIVE_SHIFT) /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define IOMUX_DRIVE_130OHM              (DRIVE_130OHM << IOMUX_DRIVE_SHIFT) /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define IOMUX_DRIVE_90OHM               (DRIVE_90OHM  << IOMUX_DRIVE_SHIFT) /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define IOMUX_DRIVE_60OHM               (DRIVE_60OHM  << IOMUX_DRIVE_SHIFT) /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define IOMUX_DRIVE_50OHM               (DRIVE_50OHM  << IOMUX_DRIVE_SHIFT) /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define IOMUX_DRIVE_40OHM               (DRIVE_40OHM  << IOMUX_DRIVE_SHIFT) /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define IOMUX_DRIVE_33OHM               (DRIVE_33OHM  << IOMUX_DRIVE_SHIFT) /* 20 Ohm @3.3V, 33 Ohm @1.8V */

/* Output Slew Rate:
 *
 *   .... .... .... L...
 */

#define IOMUX_SLEW_SLOW                   (0)       /* Bit 3: 0=Slow Slew Rate */
#define IOMUX_SLEW_FAST                   (1 << 3)  /* Bit 3: 1=Fast Slew Rate */

/* Output Speed:
 *
 *   .... .... .... .SS.
 */

#define IOMUX_SPEED_SHIFT                 (1)       /* Bits 1-2: Speed */
#define IOMUX_SPEED_MASK                  (3 << IOMUX_SPEED_SHIFT)
#  define IOMUX_SPEED_LOW                 (SPEED_LOW    << IOMUX_SPEED_SHIFT) /* Low frequency (50 MHz) */
#  define IOMUX_SPEED_MEDIUM              (SPEED_MEDIUM << IOMUX_SPEED_SHIFT) /* Medium frequency (100, 150 MHz) */
#  define IOMUX_SPEED_MAX                 (SPEED_MAX    << IOMUX_SPEED_SHIFT) /* Maximum frequency (100, 150, 200 MHz) */

/* Input Schmitt Trigger:
 *
 *   .... .... .... ...T
 */

#define IOMUX_CMOS_INPUT                  (0)       /* Bit 0: 0=CMOS input */
#define IOMUX_SCHMITT_TRIGGER             (1 << 0)  /* Bit 0: 1=Enable Schmitt trigger if input */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the IOMUX encoding */

typedef uint16_t iomux_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_map
 *
 * Description:
 *   This function map a Pad Mux register index to the corresponding Pad
 *   Control register index.
 *
 ****************************************************************************/

unsigned int imxrt_padmux_map(unsigned int padmux);

/****************************************************************************
 * Name: imxrt_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int imxrt_iomux_configure(uintptr_t padctl, iomux_pinset_t ioset);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER1_H */
