/****************************************************************************
 * arch/arm/src/rm57/rm57_gio.h
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

/* Adapted from tms570_gio.h. RM57L843 only has 2 GIO ports (A, B) vs.
 * TMS570's up to 8; the pin-encoding bit layout is otherwise identical
 * Hercules GIO IP.
 */

#ifndef __ARCH_ARM_SRC_RM57_RM57_GIO_H
#define __ARCH_ARM_SRC_RM57_RM57_GIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "hardware/rm57_gio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to rm57_configgio() ************************************/

/* 16-bit Encoding:
 *
 *   .... .... M.CC IIOV PPP. .BBB
 */

/* Input/Output mode:  .... .... M... .... .... .... */

#define GIO_MODE_SHIFT            (15)        /* Bit 15: GIO mode */
#define GIO_MODE_MASK             (1 << GIO_MODE_SHIFT)
#  define GIO_INPUT               (0 << GIO_MODE_SHIFT) /* GIO Input */
#  define GIO_OUTPUT              (1 << GIO_MODE_SHIFT) /* GIO Output */

/* Pin configuration bits (pull-up/down): .... .... ..CC .... .... .... */

#define GIO_CFG_SHIFT             (12)        /* Bits 12-14: GIO configuration bits */
#define GIO_CFG_MASK              (3 << GIO_CFG_SHIFT)
#  define GIO_CFG_DEFAULT         (0 << GIO_CFG_SHIFT) /* Default, no attribute */
#  define GIO_CFG_PULLUP          (1 << GIO_CFG_SHIFT) /* Internal pull-up */
#  define GIO_CFG_PULLDOWN        (2 << GIO_CFG_SHIFT) /* Internal pull-down */

/* Interrupt modes: .... .... .... II.. .... .... */

#define GIO_INT_SHIFT             (10)        /* Bits 10-11: GIO interrupt bits */
#define GIO_INT_MASK              (3 << GIO_INT_SHIFT)
#  define GIO_INT_NONE            (0 << GIO_INT_SHIFT)
#  define GIO_INT_RISING          (1 << GIO_INT_SHIFT)
#  define GIO_INT_FALLING         (2 << GIO_INT_SHIFT)
#  define GIO_INT_BOTHEDGES       (3 << GIO_INT_SHIFT)

/* Open drain select (output only): .... .... .... ..O. .... .... */

#define GIO_OPENDRAIN             (1 << 9)    /* Bit 9: Open drain mode */

/* Initial output value (output only): .... .... .... ...V .... .... */

#define GIO_OUTPUT_SET            (1 << 8)    /* Bit 8: Initial value of output */
#define GIO_OUTPUT_CLEAR          (0)

/* GIO port: .... .... .... .... PPP. .... */

#define GIO_PORT_SHIFT            (5)         /* Bit 5-7:  Port number */
#define GIO_PORT_MASK             (7 << GIO_PORT_SHIFT)
#  define GIO_PORT_GIOA           (0 << GIO_PORT_SHIFT)
#  define GIO_PORT_GIOB           (1 << GIO_PORT_SHIFT)

/* GIO pin: .... .... .... .... .... .BBB */

#define GIO_PIN_SHIFT             (0)         /* Bits 0-2: GIO number: 0-7 */
#define GIO_PIN_MASK              (7 << GIO_PIN_SHIFT)
#  define GIO_PIN0                (0  << GIO_PIN_SHIFT)
#  define GIO_PIN1                (1  << GIO_PIN_SHIFT)
#  define GIO_PIN2                (2  << GIO_PIN_SHIFT)
#  define GIO_PIN3                (3  << GIO_PIN_SHIFT)
#  define GIO_PIN4                (4  << GIO_PIN_SHIFT)
#  define GIO_PIN5                (5  << GIO_PIN_SHIFT)
#  define GIO_PIN6                (6  << GIO_PIN_SHIFT)
#  define GIO_PIN7                (7  << GIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef uint16_t gio_pinset_t;

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uintptr_t rm57_gio_base(gio_pinset_t cfgset)
{
  int port = (cfgset & GIO_PORT_MASK) >> GIO_PORT_SHIFT;
  return RM57_GIO_PORTBASE(port);
}

static inline int rm57_gio_port(gio_pinset_t cfgset)
{
  return (cfgset & GIO_PORT_MASK) >> GIO_PORT_SHIFT;
}

static inline int rm57_gio_pin(gio_pinset_t cfgset)
{
  return (cfgset & GIO_PIN_MASK) >> GIO_PIN_SHIFT;
}

static inline int rm57_gio_pinmask(gio_pinset_t cfgset)
{
  return 1 << ((cfgset & GIO_PIN_MASK) >> GIO_PIN_SHIFT);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_gio_initialize
 *
 * Description:
 *   Take the GIO block out of reset and assure that it is ready for use.
 *
 ****************************************************************************/

int rm57_gio_initialize(void);

/****************************************************************************
 * Name: rm57_configgio
 *
 * Description:
 *   Configure a GIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int rm57_configgio(gio_pinset_t cfgset);

/****************************************************************************
 * Name: rm57_giowrite
 *
 * Description:
 *   Write one or zero to the selected GIO pin
 *
 ****************************************************************************/

void rm57_giowrite(gio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: rm57_gioread
 *
 * Description:
 *   Read one or zero from the selected GIO pin
 *
 ****************************************************************************/

bool rm57_gioread(gio_pinset_t pinset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RM57_RM57_GIO_H */
