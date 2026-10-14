/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_gio.h
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

/* Register layout for RM57L843 GIO, per TI's HALCoGen HL_reg_gio.h -
 * matches TMS570's GIO layout/address (0xfff7bc00).
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_GIO_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_GIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RM57_NPORTS                2   /* GIO Port A, Port B */

/* GIO Module Register Offsets **********************************************/

#define RM57_GIO_GCR0_OFFSET      0x0000 /* Global Control Register */
#define RM57_GIO_INTDET_OFFSET    0x0008 /* Interrupt Detect Register */
#define RM57_GIO_POL_OFFSET       0x000c /* Interrupt Polarity Register */
#define RM57_GIO_ENASET_OFFSET    0x0010 /* Interrupt Enable Set Register */
#define RM57_GIO_ENACLR_OFFSET    0x0014 /* Interrupt Enable Clear Register */
#define RM57_GIO_LVLSET_OFFSET    0x0018 /* Interrupt Priority Set Register */
#define RM57_GIO_LVLCLR_OFFSET    0x001c /* Interrupt Priority Clear Register */
#define RM57_GIO_FLG_OFFSET       0x0020 /* Interrupt Flag Register */
#define RM57_GIO_OFF1_OFFSET      0x0024 /* Interrupt Offset 1 Register */
#define RM57_GIO_OFF2_OFFSET      0x0028 /* Interrupt Offset 2 Register */
#define RM57_GIO_EMU1_OFFSET      0x002c /* Emulation 1 Register */
#define RM57_GIO_EMU2_OFFSET      0x0030 /* Emulation 2 Register */

/* GIO Port Register Offsets (relative to each gioPORTx base) ***************/

#define RM57_GIO_DIR_OFFSET       0x0000 /* Data Direction Register */
#define RM57_GIO_DIN_OFFSET       0x0004 /* Data Input Register */
#define RM57_GIO_DOUT_OFFSET      0x0008 /* Data Output Register */
#define RM57_GIO_DSET_OFFSET      0x000c /* Data Output Set Register */
#define RM57_GIO_DCLR_OFFSET      0x0010 /* Data Output Clear Register */
#define RM57_GIO_PDR_OFFSET       0x0014 /* Open Drain Register */
#define RM57_GIO_PULDIS_OFFSET    0x0018 /* Pullup Disable Register */
#define RM57_GIO_PSL_OFFSET       0x001c /* Pull Up/Down Selection Register */

/* Register Addresses *******************************************************/

#define RM57_GIO_GCR0             (RM57_GIO_BASE + RM57_GIO_GCR0_OFFSET)
#define RM57_GIO_INTDET           (RM57_GIO_BASE + RM57_GIO_INTDET_OFFSET)
#define RM57_GIO_POL              (RM57_GIO_BASE + RM57_GIO_POL_OFFSET)
#define RM57_GIO_ENASET           (RM57_GIO_BASE + RM57_GIO_ENASET_OFFSET)
#define RM57_GIO_ENACLR           (RM57_GIO_BASE + RM57_GIO_ENACLR_OFFSET)
#define RM57_GIO_LVLSET           (RM57_GIO_BASE + RM57_GIO_LVLSET_OFFSET)
#define RM57_GIO_LVLCLR           (RM57_GIO_BASE + RM57_GIO_LVLCLR_OFFSET)
#define RM57_GIO_FLG              (RM57_GIO_BASE + RM57_GIO_FLG_OFFSET)
#define RM57_GIO_OFF1             (RM57_GIO_BASE + RM57_GIO_OFF1_OFFSET)
#define RM57_GIO_OFF2             (RM57_GIO_BASE + RM57_GIO_OFF2_OFFSET)
#define RM57_GIO_EMU1             (RM57_GIO_BASE + RM57_GIO_EMU1_OFFSET)
#define RM57_GIO_EMU2             (RM57_GIO_BASE + RM57_GIO_EMU2_OFFSET)

/* Port n base address: Port A and Port B are 0x20 bytes apart */
#define RM57_GIO_PORTBASE(n)      (RM57_GIO_PORTA_BASE + ((n) << 5))

/* GIO Interrupt Enable Set/Clear and Priority Set/Clear Registers: one byte
 * per port, one bit per pin within the byte.  The Set and Clear registers of
 * a pair share the same layout; writing a 1 acts, writing a 0 is ignored.
 */

#define GIO_ENASET_PORT_SHIFT(p)  ((p) << 3)
#define GIO_ENASET_PORT_PIN(p,n)  (1 << (GIO_ENASET_PORT_SHIFT(p) + (n)))
#define GIO_ENACLR_PORT_SHIFT(p)  ((p) << 3)
#define GIO_ENACLR_PORT_PIN(p,n)  (1 << (GIO_ENACLR_PORT_SHIFT(p) + (n)))
#define GIO_LVLSET_PORT_SHIFT(p)  ((p) << 3)
#define GIO_LVLSET_PORT_PIN(p,n)  (1 << (GIO_LVLSET_PORT_SHIFT(p) + (n)))
#define GIO_LVLCLR_PORT_SHIFT(p)  ((p) << 3)
#define GIO_LVLCLR_PORT_PIN(p,n)  (1 << (GIO_LVLCLR_PORT_SHIFT(p) + (n)))

/* GIO Interrupt Polarity / Detect / Flag Registers: same one-byte-per-port,
 * one-bit-per-pin layout as the ENA/LVL registers above.  A GIOFLG bit is
 * cleared by writing a 1 to it (or by reading the corresponding offset
 * register); writing a 0 has no effect.
 */

#define GIO_POL_PORT_SHIFT(p)     ((p) << 3)
#define GIO_POL_PORT_PIN(p,n)     (1 << (GIO_POL_PORT_SHIFT(p) + (n)))
#define GIO_INTDET_PORT_SHIFT(p)  ((p) << 3)
#define GIO_INTDET_PORT_PIN(p,n)  (1 << (GIO_INTDET_PORT_SHIFT(p) + (n)))
#define GIO_FLG_PORT_SHIFT(p)     ((p) << 3)
#define GIO_FLG_PORT_PIN(p,n)     (1 << (GIO_FLG_PORT_SHIFT(p) + (n)))

/* Register Bit-Field Definitions *******************************************/

/* Global Control Register */

/* Bit 0: Take GIO module out of reset */
#define GIO_GCR0_RESET (1 << 0)

/* Interrupt Offset Registers (OFF1/OFF2): 0 means no interrupt pending;
 * a nonzero value is (port << 3 | pin) + 1 for the highest-priority
 * pending pin.  OFF1 reports the level A (high level) pins, OFF2 the
 * level B (low level) pins.  Reading an offset register clears it, the
 * matching EMUn mirror, and the corresponding GIOFLG bit, so the register
 * can be read in a loop until it returns zero to drain every pending pin.
 * EMU1/EMU2 are non-destructive mirrors intended for debugger reads.
 */

#define GIO_OFF_NONE   (0x00)    /* No interrupt pending */

/* Per-pin bitmasks for GIOA0-GIOA7/GIOB0-GIOB7 are computed at the
 * pinset-encoding level (see rm57_gio_pinmask() / GIO_PINn in
 * rm57_gio.h), not duplicated here.
 */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_GIO_H */
