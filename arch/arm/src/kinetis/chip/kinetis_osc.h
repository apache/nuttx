/********************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_osc.h
 *
 *   Copyright (C) 2011, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_KINETIS_CHIHP_KINETIS_OSC_H
#define __ARCH_ARM_SRC_KINETIS_CHIHP_KINETIS_OSC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_OSC_CR_OFFSET  0x0000 /* OSC Control Register */
#define KINETIS_OSC_DIV_OFFSET 0x0002 /* OSC CLock divider register */

/* Register Addresses ***********************************************************************/

#define KINETIS_OSC_CR         (KINETIS_OSC_BASE+KINETIS_OSC_CR_OFFSET)
#define KINETIS_OSC_DIV        (KINETIS_OSC_BASE+KINETIS_OSC_DIV_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* OSC Control Register (8-bit) */

#define OSC_CR_ERCLKEN         (1 << 7)  /* Bit 7:  External Reference Enable */
                                         /* Bit 6:  Reserved */
#define OSC_CR_EREFSTEN        (1 << 5)  /* Bit 5:  External Reference Stop Enable */
                                         /* Bit 4:  Reserved */
#define OSC_CR_SC2P            (1 << 3)  /* Bit 3:  Oscillator 2 pF Capacitor Load Configure */
#define OSC_CR_SC4P            (1 << 2)  /* Bit 2:  Oscillator 4 pF Capacitor Load Configure */
#define OSC_CR_SC8P            (1 << 1)  /* Bit 1:  Oscillator 8 pF Capacitor Load Configure */
#define OSC_CR_SC16P           (1 << 0)  /* Bit 0:  Oscillator 16 pF Capacitor Load Configure */

/* OSC Control Register (8-bit) */
                                         /* Bits 0-5:  Reserved */
#define OSC_DIV_ERPS_SHIFT      6        /* Bits 6-7:  ERCLK prescaler */
#define OSC_DIV_ERPS_MASK       (3 << OSC_DIV_ERPS_SHIFT)
#  define OSC_DIV_ERPS_DIV1     (0 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 1 */
#  define OSC_DIV_ERPS_DIV2     (1 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 2 */
#  define OSC_DIV_ERPS_DIV3     (2 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 4 */
#  define OSC_DIV_ERPS_DIV8     (3 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 8 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIHP_KINETIS_OSC_H */
