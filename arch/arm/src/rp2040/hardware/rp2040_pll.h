/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_pll.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PLL_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_PLL_CS_OFFSET         0x000000  /* Control and Status GENERAL CONSTRAINTS: Reference clock frequency min=5MHz, max=800MHz Feedback divider min=16, max=320 VCO frequency min=400MHz, max=1600MHz */
#define RP2040_PLL_PWR_OFFSET        0x000004  /* Controls the PLL power modes. */
#define RP2040_PLL_FBDIV_INT_OFFSET  0x000008  /* Feedback divisor (note: this PLL does not support fractional division) */
#define RP2040_PLL_PRIM_OFFSET       0x00000c  /* Controls the PLL post dividers for the primary output (note: this PLL does not have a secondary output) the primary output is driven from VCO divided by postdiv1*postdiv2 */

/* Register definitions (PLL_SYS) *******************************************/

#define RP2040_PLL_SYS_CS         (RP2040_PLL_SYS_BASE + RP2040_PLL_CS_OFFSET)
#define RP2040_PLL_SYS_PWR        (RP2040_PLL_SYS_BASE + RP2040_PLL_PWR_OFFSET)
#define RP2040_PLL_SYS_FBDIV_INT  (RP2040_PLL_SYS_BASE + RP2040_PLL_FBDIV_INT_OFFSET)
#define RP2040_PLL_SYS_PRIM       (RP2040_PLL_SYS_BASE + RP2040_PLL_PRIM_OFFSET)

/* Register definitions (PLL_USB) *******************************************/

#define RP2040_PLL_USB_CS         (RP2040_PLL_USB_BASE + RP2040_PLL_CS_OFFSET)
#define RP2040_PLL_USB_PWR        (RP2040_PLL_USB_BASE + RP2040_PLL_PWR_OFFSET)
#define RP2040_PLL_USB_FBDIV_INT  (RP2040_PLL_USB_BASE + RP2040_PLL_FBDIV_INT_OFFSET)
#define RP2040_PLL_USB_PRIM       (RP2040_PLL_USB_BASE + RP2040_PLL_PRIM_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_PLL_CS_LOCK              (1 << 31) /* PLL is locked */
#define RP2040_PLL_CS_BYPASS            (1 << 8)  /* Passes the reference clock to the output instead of the divided VCO. The VCO continues to run so the user can switch between the reference clock and the divided VCO but the output will glitch when doing so. */
#define RP2040_PLL_CS_REFDIV_MASK       (0x3f)    /* Divides the PLL input reference clock. Behaviour is undefined for div=0. PLL output will be unpredictable during refdiv changes, wait for lock=1 before using it. */

#define RP2040_PLL_PWR_VCOPD            (1 << 5)  /* PLL VCO powerdown To save power set high when PLL output not required or bypass=1. */
#define RP2040_PLL_PWR_POSTDIVPD        (1 << 3)  /* PLL post divider powerdown To save power set high when PLL output not required or bypass=1. */
#define RP2040_PLL_PWR_DSMPD            (1 << 2)  /* PLL DSM powerdown Nothing is achieved by setting this low. */
#define RP2040_PLL_PWR_PD               (1 << 0)  /* PLL powerdown To save power set high when PLL output not required. */

#define RP2040_PLL_FBDIV_INT_MASK       (0xfff)  /* see ctrl reg description for constraints */

#define RP2040_PLL_PRIM_POSTDIV1_SHIFT  (16)  /* divide by 1-7 */
#define RP2040_PLL_PRIM_POSTDIV1_MASK   (0x07 << RP2040_PLL_PRIM_POSTDIV1_SHIFT)
#define RP2040_PLL_PRIM_POSTDIV2_SHIFT  (12)  /* divide by 1-7 */
#define RP2040_PLL_PRIM_POSTDIV2_MASK   (0x07 << RP2040_PLL_PRIM_POSTDIV2_SHIFT)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PLL_H */
