/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_vim.h
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

/* References:
 * TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller,
 * Technical Reference Manual, Texas Instruments,
 * Literature Number: SPNU517A, September 2013
 */

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_VIM_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_VIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIM_REGNDX(ch)                  ((ch) >> 5)
#define VIM_REGBIT(ch)                  ((ch) & 31)

/* Register Offsets *********************************************************/

/* Register Offsets relative to the VIM Parity Frame */

#define TMS570_VIM_PARFLG_OFFSET        0x00ec /* Interrupt Vector Table Parity Flag Register */
#define TMS570_VIM_PARCTL_OFFSET        0x00f0 /* Interrupt Vector Table Parity Control Register */
#define TMS570_VIM_ADDERR_OFFSET        0x00f4 /* Address Parity Error Register */
#define TMS570_VIM_FBPARERR_OFFSET      0x00f8 /* Fall-Back Address Parity Error Register */

/* Register Offsets relative to the VIM Frame */

#define TMS570_VIM_IRQINDEX_OFFSET      0x0000 /* IRQ Index Offset Vector Register */
#define TMS570_VIM_FIQINDEX_OFFSET      0x0004 /* FIQ Index Offset Vector Register */
#define TMS570_VIM_FIRQPR_OFFSET(n)     (0x0010 + ((n) << 2))
#  define TMS570_VIM_FIRQPR0_OFFSET     0x0010 /* FIQ/IRQ Program Control Register 0 */
#  define TMS570_VIM_FIRQPR1_OFFSET     0x0014 /* FIQ/IRQ Program Control Register 1 */
#  define TMS570_VIM_FIRQPR2_OFFSET     0x0018 /* FIQ/IRQ Program Control Register 2 */
#define TMS570_VIM_INTREQ_OFFSET(n)     (0x0020 + ((n) << 2))
#  define TMS570_VIM_INTREQ0_OFFSET     0x0020 /* Pending Interrupt Read Location Register 0 */
#  define TMS570_VIM_INTREQ1_OFFSET     0x0024 /* Pending Interrupt Read Location Register 1 */
#  define TMS570_VIM_INTREQ2_OFFSET     0x0028 /* Pending Interrupt Read Location Register 2 */
#define TMS570_VIM_REQENASET_OFFSET(n)  (0x0030 + ((n) << 2))
#  define TMS570_VIM_REQENASET0_OFFSET  0x0030 /* Interrupt Enable Set Register 0 */
#  define TMS570_VIM_REQENASET1_OFFSET  0x0034 /* Interrupt Enable Set Register 1 */
#  define TMS570_VIM_REQENASET2_OFFSET  0x0038 /* Interrupt Enable Set Register 2 */
#define TMS570_VIM_REQENACLR_OFFSET(n)  (0x0040 + ((n) << 2))
#  define TMS570_VIM_REQENACLR0_OFFSET  0x0040 /* Interrupt Enable Clear Register 0 */
#  define TMS570_VIM_REQENACLR1_OFFSET  0x0044 /* Interrupt Enable Clear Register 1 */
#  define TMS570_VIM_REQENACLR2_OFFSET  0x0048 /* Interrupt Enable Clear Register 2 */
#define TMS570_VIM_WAKEENASET_OFFSET(n) (0x0050 + ((n) << 2))
#  define TMS570_VIM_WAKEENASET0_OFFSET 0x0050 /* Wake-up Enable Set Register 0 */
#  define TMS570_VIM_WAKEENASET1_OFFSET 0x0054 /* Wake-up Enable Set Register 1 */
#  define TMS570_VIM_WAKEENASET2_OFFSET 0x0058 /* Wake-up Enable Set Register 2 */
#define TMS570_VIM_WAKEENACLR_OFFSET(n) (0x0060 + ((n) << 2))
#  define TMS570_VIM_WAKEENACLR0_OFFSET 0x0060 /* Wake-up Enable Clear Register 0 */
#  define TMS570_VIM_WAKEENACLR1_OFFSET 0x0064 /* Wake-up Enable Clear Register 1 */
#  define TMS570_VIM_WAKEENACLR2_OFFSET 0x0068 /* Wake-up Enable Clear Register 2 */
#define TMS570_VIM_IRQVECREG_OFFSET     0x0070 /* IRQ Interrupt Vector Register */
#define TMS570_VIM_FIQVECREG_OFFSET     0x0074 /* FIQ Interrupt Vector Register */
#define TMS570_VIM_CAPEVT_OFFSET        0x0078 /* Capture Event Register */

/* 0x0080-0x00dc VIM Interrupt Control Register */

#define TMS570_VIM_CHANCTRL_INDEX(n)    ((n) >> 2)
#define TMS570_VIM_CHANCTRL_OFFSET(n)   (0x0080 << (TMS570_VIM_CHANCTRL_INDEX(n) << 2))

/* Register Addresses *******************************************************/

/* VIM Parity Frame Registers */

#define TMS570_VIM_PARFLG               (TMS570_VIMPAR_BASE+TMS570_VIM_PARFLG_OFFSET)
#define TMS570_VIM_PARCTL               (TMS570_VIMPAR_BASE+TMS570_VIM_PARCTL_OFFSET)
#define TMS570_VIM_ADDERR               (TMS570_VIMPAR_BASE+TMS570_VIM_ADDERR_OFFSET)
#define TMS570_VIM_FBPARERR             (TMS570_VIMPAR_BASE+TMS570_VIM_FBPARERR_OFFSET)

/* VIM Frame Registers */

#define TMS570_VIM_IRQINDEX             (TMS570_VIM_BASE+TMS570_VIM_IRQINDEX_OFFSET)
#define TMS570_VIM_FIQINDEX             (TMS570_VIM_BASE+TMS570_VIM_FIQINDEX_OFFSET)
#define TMS570_VIM_FIRQPR(n)            (TMS570_VIM_BASE+TMS570_VIM_FIRQPR_OFFSET(n))
#  define TMS570_VIM_FIRQPR0            (TMS570_VIM_BASE+TMS570_VIM_FIRQPR0_OFFSET)
#  define TMS570_VIM_FIRQPR1            (TMS570_VIM_BASE+TMS570_VIM_FIRQPR1_OFFSET)
#  define TMS570_VIM_FIRQPR2            (TMS570_VIM_BASE+TMS570_VIM_FIRQPR2_OFFSET)
#define TMS570_VIM_INTREQ(n)            (TMS570_VIM_BASE+TMS570_VIM_INTREQ_OFFSET(n))
#  define TMS570_VIM_INTREQ0            (TMS570_VIM_BASE+TMS570_VIM_INTREQ0_OFFSET)
#  define TMS570_VIM_INTREQ1            (TMS570_VIM_BASE+TMS570_VIM_INTREQ1_OFFSET)
#  define TMS570_VIM_INTREQ2            (TMS570_VIM_BASE+TMS570_VIM_INTREQ2_OFFSET)
#define TMS570_VIM_REQENASET(n)         (TMS570_VIM_BASE+TMS570_VIM_REQENASET_OFFSET(n))
#  define TMS570_VIM_REQENASET0         (TMS570_VIM_BASE+TMS570_VIM_REQENASET0_OFFSET)
#  define TMS570_VIM_REQENASET1         (TMS570_VIM_BASE+TMS570_VIM_REQENASET1_OFFSET)
#  define TMS570_VIM_REQENASET2         (TMS570_VIM_BASE+TMS570_VIM_REQENASET2_OFFSET)
#define TMS570_VIM_REQENACLR(n)         (TMS570_VIM_BASE+TMS570_VIM_REQENACLR_OFFSET(n))
#  define TMS570_VIM_REQENACLR0         (TMS570_VIM_BASE+TMS570_VIM_REQENACLR0_OFFSET)
#  define TMS570_VIM_REQENACLR1         (TMS570_VIM_BASE+TMS570_VIM_REQENACLR1_OFFSET)
#  define TMS570_VIM_REQENACLR2         (TMS570_VIM_BASE+TMS570_VIM_REQENACLR2_OFFSET)
#define TMS570_VIM_WAKEENASET(n)        (TMS570_VIM_BASE+TMS570_VIM_WAKEENASET_OFFSET(n))
#  define TMS570_VIM_WAKEENASET0        (TMS570_VIM_BASE+TMS570_VIM_WAKEENASET0_OFFSET)
#  define TMS570_VIM_WAKEENASET1        (TMS570_VIM_BASE+TMS570_VIM_WAKEENASET1_OFFSET)
#  define TMS570_VIM_WAKEENASET2        (TMS570_VIM_BASE+TMS570_VIM_WAKEENASET2_OFFSET)
#define TMS570_VIM_WAKEENACLR(n)        (TMS570_VIM_BASE+TMS570_VIM_WAKEENACLR_OFFSET(n))
#  define TMS570_VIM_WAKEENACLR0        (TMS570_VIM_BASE+TMS570_VIM_WAKEENACLR0_OFFSET)
#  define TMS570_VIM_WAKEENACLR1        (TMS570_VIM_BASE+TMS570_VIM_WAKEENACLR1_OFFSET)
#  define TMS570_VIM_WAKEENACLR2        (TMS570_VIM_BASE+TMS570_VIM_WAKEENACLR2_OFFSET)
#define TMS570_VIM_IRQVECREG            (TMS570_VIM_BASE+TMS570_VIM_IRQVECREG_OFFSET)
#define TMS570_VIM_FIQVECREG            (TMS570_VIM_BASE+TMS570_VIM_FIQVECREG_OFFSET)
#define TMS570_VIM_CAPEVT               (TMS570_VIM_BASE+TMS570_VIM_CAPEVT_OFFSET)

/* 0x0080-0x00dc VIM Interrupt Control Register */

#define TMS570_VIM_CHANCTRL(n)          (TMS570_VIM_BASE+TMS570_VIM_CHANCTRL_OFFSET(n))

/* Register Bit-Field Definitions *******************************************/

/* Interrupt Vector Table Parity Flag Register */
#define VIM_PARFLG_

/* Interrupt Vector Table Parity Control Register */
#define VIM_PARCTL_

/* Address Parity Error Register */
#define VIM_ADDERR_

/* Fall-Back Address Parity Error Register */
#define VIM_FBPARERR_

/* IRQ Index Offset Vector Register */

#define VIM_IRQINDEX_MASK               (0x000000ff)  /* IRQ index vector */

/* FIQ Index Offset Vector Register */

#define VIM_FIQINDEX_MASK               (0x000000ff)  /* FIQ index vector */

/* FIQ/IRQ Program Control Register 0 */
#define VIM_FIRQPR0_

/* FIQ/IRQ Program Control Register 1 */
#define VIM_FIRQPR1_

/* FIQ/IRQ Program Control Register 2 */
#define VIM_FIRQPR2_

/* Pending Interrupt Read Location Register 0 */
#define VIM_INTREQ0_

/* Pending Interrupt Read Location Register 1 */
#define VIM_INTREQ1_

/* Pending Interrupt Read Location Register 2 */
#define VIM_INTREQ2_

/* Interrupt Enable Set Register 0 */
#define VIM_REQENASET0_

/* Interrupt Enable Set Register 1 */
#define VIM_REQENASET1_

/* Interrupt Enable Set Register 2 */
#define VIM_REQENASET2_

/* Interrupt Enable Clear Register 0 */
#define VIM_REQENACLR0_

/* Interrupt Enable Clear Register 1 */
#define VIM_REQENACLR1_

/* Interrupt Enable Clear Register 2 */
#define VIM_REQENACLR2_

/* Wake-up Enable Set Register 0 */
#define VIM_WAKEENASET0_

/* Wake-up Enable Set Register 1 */
#define VIM_WAKEENASET1_

/* Wake-up Enable Set Register 2 */
#define VIM_WAKEENASET2_

/* Wake-up Enable Clear Register 0 */
#define VIM_WAKEENACLR0_

/* Wake-up Enable Clear Register 1 */
#define VIM_WAKEENACLR1_

/* Wake-up Enable Clear Register 2 */
#define VIM_WAKEENACLR2_

/* IRQ Interrupt Vector Register */
#define VIM_IRQVECREG_

/* FIQ Interrupt Vector Register */
#define VIM_FIQVECREG_

/* Capture Event Register */
#define VIM_CAPEVT_

/* 0x0080-0x00dc VIM Interrupt Control Register */

#define VIM_CHANCTRL_SHIFT(n)           (((n) & 3) << 3)
#define VIM_CHANCTRL_MASK(n)            (0xff << VIM_CHANCTRL_SHIFT(n))
#  define VIM_CHANCTRL(n,v)             ((uint32_t)(v) << VIM_CHANCTRL_SHIFT(n))

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_VIM_H */
