/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_gint.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

#define LPC54_GINT_CTRL_OFFSET         0x0000  /* GPIO grouped interrupt control */
#define LPC54_GINT_PORT_POL0_OFFSET    0x0020  /* GPIO grouped interrupt port 0 polarity */
#define LPC54_GINT_PORT_POL0_OFFSET    0x0024  /* GPIO grouped interrupt port 1 polarity */
#define LPC54_GINT_PORT_ENA0_OFFSET    0x0040  /* GPIO grouped interrupt port 0 enable */
#define LPC54_GINT_PORT_ENA1_OFFSET    0x0044  /* GPIO grouped interrupt port 1 enable */

/* Register addresses *******************************************************************************/

#define LPC54_GINT0_CTRL               (LPC54_GINT0_BASE + LPC54_GINT_CTRL_OFFSET)
#define LPC54_GINT0_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL0_OFFSET)
#define LPC54_GINT0_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL1_OFFSET)
#define LPC54_GINT0_PORT_ENA0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA0_OFFSET)
#define LPC54_GINT0_PORT_ENA1          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA1_OFFSET)

#define LPC54_GINT1_CTRL               (LPC54_GINT0_BASE + LPC54_GINT_CTRL_OFFSET)
#define LPC54_GINT1_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL0_OFFSET)
#define LPC54_GINT1_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL1_OFFSET)
#define LPC54_GINT1_PORT_ENA0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA0_OFFSET)
#define LPC54_GINT1_PORT_ENA1          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA1_OFFSET)

/* Register bit definitions *************************************************************************/

/* GPIO grouped interrupt control */

#define GINT_CTRL_INT                  (1 << 0)  /* Bit 0:  Group interrupt status */
#define GINT_CTRL_COMB                 (1 << 1)  /* Bit 1:  Combine enabled inputs for group interrupt 0 */
#define GINT_CTRL_TRIG                 (1 << 2)  /* Bit 2"  Group interrupt trigger 0 */

/* GPIO grouped interrupt port 0/1 polarity */

#define GINT_PORT_POL0(n)              (1 << (n)) /* Configure pin polarity of port0 pins for group interrupt */
#define GINT_PORT_POL1(n)              (1 << (n)) /* Configure pin polarity of port1 pins for group interrupt */

/* GPIO grouped interrupt port 0/1 enable */

#define GINT_PORT_ENA0(n)              (1 << (n)) /* Enable port0 pin for group interrupt */
#define GINT_PORT_ENA1(n)              (1 << (n)) /* Enable port1 pin for group interrupt */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H */
