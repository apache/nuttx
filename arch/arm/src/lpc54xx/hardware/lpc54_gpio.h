/************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc54_flexcomm.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GPIO_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/lpc54_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define LPC54_GPIO_PORT0            0
#define LPC54_GPIO_PORT1            1
#define LPC54_GPIO_PORT2            2
#define LPC54_GPIO_PORT3            3
#define LPC54_GPIO_PORT4            4
#define LPC54_GPIO_PORT5            5
#define LPC54_GPIO_NPORTS           6

/* Register offsets *****************************************************************/
/* Byte and word access to individual pins */

#define LPC54_GPIO_B_OFFSET(p)      (0x0000 + (p))
#define LPC54_GPIO_W_OFFSET(p)      (0x1000 + ((p) << 2))

/* Word access to individual port regisers */

#define LPC54_GPIO_PORT_OFFSET(n)   ((n) << 2)
#define LPC54_GPIO_DIR_OFFSET(n)    (0x2000 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_MASK_OFFSET(n)   (0x2080 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_PIN_OFFSET(n)    (0x2100 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_MPIN_OFFSET(n)   (0x2180 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_SET_OFFSET(n)    (0x2200 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_CLR_OFFSET(n)    (0x2280 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_NOT_OFFSET(n)    (0x2300 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_DIRSET_OFFSET(n) (0x2380 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_DIRCLR_OFFSET(n) (0x2400 + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_DIRNOT_OFFSET(n) (0x2480 + LPC54_GPIO_PORT_OFFSET(n))

/* Register addresses ***************************************************************/

/* Byte and word access to individual pins */

#define LPC54_GPIO_B(p)             (LPC54_GPIO_BASE + LPC54_GPIO_B_OFFSET(p))
#define LPC54_GPIO_W(p)             (LPC54_GPIO_BASE + LPC54_GPIO_W_OFFSET(p))

/* Word access to individual port regisers */

#define LPC54_GPIO_PORT(n)          (LPC54_GPIO_BASE + LPC54_GPIO_PORT_OFFSET(n))
#define LPC54_GPIO_DIR(n)           (LPC54_GPIO_BASE + LPC54_GPIO_DIR_OFFSET(n))
#define LPC54_GPIO_MASK(n)          (LPC54_GPIO_BASE + LPC54_GPIO_MASK_OFFSET(n))
#define LPC54_GPIO_PIN(n)           (LPC54_GPIO_BASE + LPC54_GPIO_PIN_OFFSET(n))
#define LPC54_GPIO_MPIN(n)          (LPC54_GPIO_BASE + LPC54_GPIO_MPIN_OFFSET(n))
#define LPC54_GPIO_SET(n)           (LPC54_GPIO_BASE + LPC54_GPIO_SET_OFFSET(n))
#define LPC54_GPIO_CLR(n)           (LPC54_GPIO_BASE + LPC54_GPIO_CLR_OFFSET(n))
#define LPC54_GPIO_NOT(n)           (LPC54_GPIO_BASE + LPC54_GPIO_NOT_OFFSET(n))
#define LPC54_GPIO_DIRSET(n)        (LPC54_GPIO_BASE + LPC54_GPIO_DIRSET_OFFSET(n))
#define LPC54_GPIO_DIRCLR(n)        (LPC54_GPIO_BASE + LPC54_GPIO_DIRCLR_OFFSET(n))
#define LPC54_GPIO_DIRNOT(n)        (LPC54_GPIO_BASE + LPC54_GPIO_DIRNOT_OFFSET(n))

/* Register bit definitions *********************************************************/

/* Port registers are all bit arrays with one bit corresponding each of the 32 pins
 * of the port.
 */

#define GPIO_PORT_BIT(n)            (1 << ((n) & 31))

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GPIO_H */
