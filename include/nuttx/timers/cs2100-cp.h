/********************************************************************************************
 * include/nuttx/timers/cs2100-cp.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_TIMERS_CS2100_CP_H
#define __INCLUDE_NUTTX_TIMERS_CS2100_CP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/i2c.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Addresses ***********************************************************************/

#define CS2100_DEVID                       0x01      /* Device ID and Revision */
#define CS2100_DEVCTL                      0x02      /* Device Control */
#define CS2100_DEVCFG1                     0x03      /* Device Configuration 1 */
#define CS2100_GBLCFG                      0x05      /* Global Configuration */
#define CS2100_RATIO0                      0x06      /* Ratio: Bits 24-31 */
#define CS2100_RATIO1                      0x07      /* Ratio: Bits 16-23 */
#define CS2100_RATIO2                      0x08      /* Ratio: Bits  8-15 */
#define CS2100_RATIO3                      0x09      /* Ratio: Bits  0- 7 */
#define CS2100_FNCCFG1                     0x16      /* Function Configuration 1 */
#define CS2100_FNCCFG2                     0x17      /* Function Configuration 2 */
#define CS2100_FNCCFG3                     0x1e      /* Function Configuration 3 */

/* Register Bit Field Definitions ***********************************************************/

/* Device ID and Revision */

#define CS2100_DEVID_REVISION_SHIFT        (0)       /* Bits 0-2: Device REVISION */
#define CS2100_DEVID_REVISION_MASK         (7 << CS2100_DEVID_REVISION_SHIFT)
#define CS2100_DEVID_DEVICE_SHIFT          (3)       /* Bits 3-7: Device ID */
#define CS2100_DEVID_DEVICE_MASK           (31 << CS2100_DEVID_DEVICE_SHIFT)
#  define CS2100_DEVID_DEVICE              (0 << CS2100_DEVID_DEVICE_SHIFT)

/* Device Control */

#define CS2100_DEVCTL_CLKOUTDIS            (1 << 0)  /* Bit 0:  CLK_OUT disable */
#define CS2100_DEVCTL_AUXOUTDIS            (1 << 1)  /* Bit 1:  AUX_OUT disable */
#define CS2100_DEVCTL_UNLOCK               (1 << 31) /* Bit 31: Unlock PLL */

/* Device Configuration 1 */

#define CS2100_DEVCFG1_ENDEVCFG1           (1 << 0)  /* Bit 0: Enable 1 */
#define CS2100_DEVCFG1_AUXOUTSRC_SHIFT     (1)       /* Bits 1-2: Source of AUX_OUT signal */
#define CS2100_DEVCFG1_AUXOUTSRC_MASK      (3 << CS2100_DEVCFG1_AUXOUTSRC_SHIFT)
#  define CS2100_DEVCFG1_AUXOUTSRC_REFCLK  (0 << CS2100_DEVCFG1_AUXOUTSRC_SHIFT) /* RefClk */
#  define CS2100_DEVCFG1_AUXOUTSRC_CLKIN   (1 << CS2100_DEVCFG1_AUXOUTSRC_SHIFT) /* CLK_IN */
#  define CS2100_DEVCFG1_AUXOUTSRC_CLKOUT  (2 << CS2100_DEVCFG1_AUXOUTSRC_SHIFT) /* CLK_OUT */
#  define CS2100_DEVCFG1_AUXOUTSRC_PLLLOCK (3 << CS2100_DEVCFG1_AUXOUTSRC_SHIFT) /* PLL Lock Status Indicator*/
#define CS2100_DEVCFG1_RMODSEL_SHIFT       (5)       /* Bit 5-7: Selects R-Mod value */
#define CS2100_DEVCFG1_RMODSEL_MASK        (7 << CS2100_DEVCFG1_RMODSEL_SHIFT)
#  define CS2100_DEVCFG1_RMODSEL_NONE      (0 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Left-shift R-value by 0 (x 1) */
#  define CS2100_DEVCFG1_RMODSEL_MUL2      (1 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Left-shift R-value by 1 (x 2) */
#  define CS2100_DEVCFG1_RMODSEL_MUL4      (2 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Left-shift R-value by 2 (x 4) */
#  define CS2100_DEVCFG1_RMODSEL_MUL8      (3 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Left-shift R-value by 3 (x 8) */
#  define CS2100_DEVCFG1_RMODSEL_DIV2      (4 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Right-shift R-value by 1 (÷ 2) */
#  define CS2100_DEVCFG1_RMODSEL_DIV4      (5 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Right-shift R-value by 2 (÷ 4) */
#  define CS2100_DEVCFG1_RMODSEL_DIV8      (6 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Right-shift R-value by 3 (÷ 8) */
#  define CS2100_DEVCFG1_RMODSEL_DIV16     (7 << CS2100_DEVCFG1_RMODSEL_SHIFT) /* Right-shift R-value by 4 (÷ 16) */

/* Global Configuration */

#define CS2100_GBLCFG_ENDEVCFG2            (1 << 0)  /* Bit 0: Enable 2 */
#define CS2100_GBLCFG_FREEZE               (1 << 3)  /* Bit 3: Mods not applied until unfrozen */

/* Ratio: Bits 0-31 (4 x 8-bit values) */

/* Function Configuration 1 */

#define CS2100_FNCCFG1_REFCLKDIV_SHIFT     (3)       /* Bits 3-4: Reference Clock Input Divider */
#define CS2100_FNCCFG1_REFCLKDIV_MASK      (3 << CS2100_FNCCFG1_REFCLKDIV_SHIFT)
#  define CS2100_FNCCFG1_REFCLKDIV_DIV4    (3 << CS2100_FNCCFG1_REFCLKDIV_SHIFT) /* ÷ 4. 32 MHz to 75 MHz (50 MHz with XTI) */
#  define CS2100_FNCCFG1_REFCLKDIV_DIV2    (3 << CS2100_FNCCFG1_REFCLKDIV_SHIFT) /* ÷ 2. 16 MHz to 37.5 MHz */
#  define CS2100_FNCCFG1_REFCLKDIV_NONE    (3 << CS2100_FNCCFG1_REFCLKDIV_SHIFT) /* ÷ 1. 8 MHz to 18.75 MHz */

#define CS2100_FNCCFG1_AUXLOCKCFG_MASK     (1 << 6)  /* Bit 6:  AUX PLL Lock Output Configuration */
#  define CS2100_FNCCFG1_AUXLOCKCFG_PP     (0 << 6)  /*   0=Push-Pull */
#  define CS2100_FNCCFG1_AUXLOCKCFG_PP     (1 << 6)  /*   1=Open Drain */
#define CS2100_FNCCFG1_CLKSKIPEN           (1 << 7)  /* Bit 7:  Clock Skip Enable */

/* Function Configuration 2 */

#define CS2100_FNCCFG2_LFRATIOCFG          (1 << 3)  /* Bit 3:  Low-Frequency Ratio Configuration */
#define CS2100_FNCCFG2_CLKOUTUNL           (1 << 4)  /* Bit 4:  Enable PLL Clock Output on Unlock */

/* Function Configuration 3 */

#define CS2100_FNCCFG3_CLKINBW_SHIFT       (4)       /* Bits 4-6: Clock Input Bandwidth */
#define CS2100_FNCCFG3_CLKINBW_MASK        (7 << CS2100_FNCCFG3_CLKINBW_SHIFT)
#  define CS2100_FNCCFG3_CLKINBW_1HZ       (0 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 1 Hz */
#  define CS2100_FNCCFG3_CLKINBW_2HZ       (1 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 2 Hz */
#  define CS2100_FNCCFG3_CLKINBW_4HZ       (2 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 4 Hz */
#  define CS2100_FNCCFG3_CLKINBW_8HZ       (3 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 8 Hz */
#  define CS2100_FNCCFG3_CLKINBW_16HZ      (4 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 16 Hz */
#  define CS2100_FNCCFG3_CLKINBW_32HZ      (5 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 32 Hz */
#  define CS2100_FNCCFG3_CLKINBW_64HZ      (6 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 64 Hz */
#  define CS2100_FNCCFG3_CLKINBW_128HZ     (7 << CS2100_FNCCFG3_CLKINBW_SHIFT) /* 128 Hz */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif  /* __INCLUDE_NUTTX_TIMERS_CS2100_CP_H */
