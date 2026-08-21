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

/* Interrupt Priority Clear Register */
#define RM57_GIO_LVLCLR_OFFSET 0x001c
#define RM57_GIO_FLG_OFFSET       0x0020 /* Interrupt Flag Register */
#define RM57_GIO_OFF1_OFFSET      0x0024 /* Interrupt Offset A Register */
#define RM57_GIO_OFF2_OFFSET      0x0028 /* Interrupt Offset B Register */
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

/* Port n base address: Port A and Port B are 0x20 bytes apart
 * (gioPORT_t is 8 32-bit registers), matching
 * RM57_GIO_PORTA_BASE/RM57_GIO_PORTB_BASE in rm57l843_memorymap.h
 */

#define RM57_GIO_PORTBASE(n)      (RM57_GIO_PORTA_BASE + ((n) << 5))
#define RM57_GIO_DIR(n)           (RM57_GIO_PORTBASE(n) + RM57_GIO_DIR_OFFSET)
#define RM57_GIO_DIN(n)           (RM57_GIO_PORTBASE(n) + RM57_GIO_DIN_OFFSET)
#define RM57_GIO_DOUT(n) (RM57_GIO_PORTBASE(n) + RM57_GIO_DOUT_OFFSET)
#define RM57_GIO_DSET(n) (RM57_GIO_PORTBASE(n) + RM57_GIO_DSET_OFFSET)
#define RM57_GIO_DCLR(n) (RM57_GIO_PORTBASE(n) + RM57_GIO_DCLR_OFFSET)
#define RM57_GIO_PDR(n)           (RM57_GIO_PORTBASE(n) + RM57_GIO_PDR_OFFSET)
#define RM57_GIO_PULDIS(n) (RM57_GIO_PORTBASE(n) + RM57_GIO_PULDIS_OFFSET)
#define RM57_GIO_PSL(n)           (RM57_GIO_PORTBASE(n) + RM57_GIO_PSL_OFFSET)

/* GIO Interrupt Enable Clear / Priority Clear Register: one byte per
 * port, one bit per pin within the byte
 */

#define GIO_ENACLR_PORT_SHIFT(p)  ((p) << 3)
#define GIO_ENACLR_PORT_PIN(p,n)  (1 << (GIO_ENACLR_PORT_SHIFT(p) + (n)))
#define GIO_LVLCLR_PORT_SHIFT(p)  ((p) << 3)
#define GIO_LVLCLR_PORT_PIN(p,n)  (1 << (GIO_LVLCLR_PORT_SHIFT(p) + (n)))

/* Register Bit-Field Definitions *******************************************/

/* Global Control Register */

/* Bit 0: Take GIO module out of reset */
#define GIO_GCR0_RESET (1 << 0)

/* Per-pin bitmasks for GIOA0-GIOA7/GIOB0-GIOB7 are computed at the
 * pinset-encoding level (see rm57_gio_pinmask() / GIO_PINn in
 * rm57_gio.h), not duplicated here.
 */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_GIO_H */
