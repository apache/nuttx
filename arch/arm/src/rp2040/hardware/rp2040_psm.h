/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_psm.h
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

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PSM_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PSM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_PSM_FRCE_ON_OFFSET   0x000000  /* Force block out of reset (i.e. power it on) */
#define RP2040_PSM_FRCE_OFF_OFFSET  0x000004  /* Force into reset (i.e. power it off) */
#define RP2040_PSM_WDSEL_OFFSET     0x000008  /* Set to 1 if this peripheral should be reset when the watchdog fires. */
#define RP2040_PSM_DONE_OFFSET      0x00000c  /* Indicates the peripheral's registers are ready to access. */

/* Register definitions *****************************************************/

#define RP2040_PSM_FRCE_ON   (RP2040_PSM_BASE + RP2040_PSM_FRCE_ON_OFFSET)
#define RP2040_PSM_FRCE_OFF  (RP2040_PSM_BASE + RP2040_PSM_FRCE_OFF_OFFSET)
#define RP2040_PSM_WDSEL     (RP2040_PSM_BASE + RP2040_PSM_WDSEL_OFFSET)
#define RP2040_PSM_DONE      (RP2040_PSM_BASE + RP2040_PSM_DONE_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_PSM_PROC1                 (1 << 16)
#define RP2040_PSM_PROC0                 (1 << 15)
#define RP2040_PSM_SIO                   (1 << 14)
#define RP2040_PSM_VREG_AND_CHIP_RESET   (1 << 13)
#define RP2040_PSM_XIP                   (1 << 12)
#define RP2040_PSM_SRAM5                 (1 << 11)
#define RP2040_PSM_SRAM4                 (1 << 10)
#define RP2040_PSM_SRAM3                 (1 << 9)
#define RP2040_PSM_SRAM2                 (1 << 8)
#define RP2040_PSM_SRAM1                 (1 << 7)
#define RP2040_PSM_SRAM0                 (1 << 6)
#define RP2040_PSM_ROM                   (1 << 5)
#define RP2040_PSM_BUSFABRIC             (1 << 4)
#define RP2040_PSM_RESETS                (1 << 3)
#define RP2040_PSM_CLOCKS                (1 << 2)
#define RP2040_PSM_XOSC                  (1 << 1)
#define RP2040_PSM_ROSC                  (1 << 0)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PSM_H */
