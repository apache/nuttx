/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_gpio.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC11XX_LPC11_CHIP_GPIO_H
#define __ARCH_ARM_SRC_LPC11XX_LPC11_CHIP_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* GPIO block register offsets ******************************************************/

#define LPC11_GPIO0_OFFSET                0x00000
#define LPC11_GPIO1_OFFSET                0x10000
#define LPC11_GPIO2_OFFSET                0x20000
#define LPC11_GPIO3_OFFSET                0x30000
#define LPC11_GPIO_OFFSET(n)              (0x10000*(n))

#define LPC11_GPIO_DATA_OFFSET            0x3FFC
#define LPC11_GPIO_DIR_OFFSET             0x8000  /* GPIO Port Direction control */
#define LPC11_GPIO_IS_OFFSET              0x8004  /* Interrupt Sense register */
#define LPC11_GPIO_IBE_OFFSET             0x8008  /* Interrupt Both Edges register */
#define LPC11_GPIO_IEV_OFFSET             0x800c  /* Interrupt Event register */
#define LPC11_GPIO_IE_OFFSET              0x8010  /* Interrupt Mask register */
#define LPC11_GPIO_RIS_OFFSET             0x8014  /* Raw interrupt status register */
#define LPC11_GPIO_MIS_OFFSET             0x8018  /* Masked interrupt status register */
#define LPC11_GPIO_IC_OFFSET              0x801c  /* Interrupt clear register */

/* Register addresses ***************************************************************/
/* GPIO block register addresses ****************************************************/

#define LPC11_GPIOn_BASE(n)               (LPC11_GPIO_BASE+LPC11_GPIO_OFFSET(n))
#define LPC11_GPIO0_BASE                  (LPC11_GPIO_BASE+LPC11_GPIO0_OFFSET)
#define LPC11_GPIO1_BASE                  (LPC11_GPIO_BASE+LPC11_GPIO1_OFFSET)
#define LPC11_GPIO2_BASE                  (LPC11_GPIO_BASE+LPC11_GPIO2_OFFSET)
#define LPC11_GPIO3_BASE                  (LPC11_GPIO_BASE+LPC11_GPIO3_OFFSET)

#define LPC11_GPIO_DIR(n)                 (LPC11_GPIO_BASE(n)+LPC11_GPIO_DIR_OFFSET)  /* GPIO Port Direction register */
#define LPC11_GPIO_IS(n)                  (LPC11_GPIO_BASE(n)+LPC11_GPIO_IS_OFFSET)   /* GPIO Interrupt Sense register */
#define LPC11_GPIO_IBE(n)                 (LPC11_GPIO_BASE(n)+LPC11_GPIO_IBE_OFFSET)  /* GPIO Interrupt Both Edges sense register */
#define LPC11_GPIO_IEV(n)                 (LPC11_GPIO_BASE(n)+LPC11_GPIO_IVE_OFFSET)  /* GPIO Interrupt Event register */
#define LPC11_GPIO_IE(n)                  (LPC11_GPIO_BASE(n)+LPC11_GPIO_IE_OFFSET)   /* GPIO Interrupt Mask register */
#define LPC11_GPIO_RIS(n)                 (LPC11_GPIO_BASE(n)+LPC11_GPIO_RIS_OFFSET)  /* GPIO Raw Interrupt Status register */
#define LPC11_GPIO_MIS(n)                 (LPC11_GPIO_BASE(n)+LPC11_GPIO_MIS_OFFSET)  /* GPIO Masked Interrupt Status register */
#define LPC11_GPIO_IC(n)                  (LPC11_GPIO_BASE(n)+LPC11_GPIO_IC_OFFSET)   /* GPIO Interrupt Clear register */

#define LPC11_GPIO0_DATA                  (LPC11_GPIO0_BASE+LPC11_GPIO_DATA_OFFSET)
#define LPC11_GPIO0_DIR                   (LPC11_GPIO0_BASE+LPC11_GPIO_DIR_OFFSET)
#define LPC11_GPIO0_IS                    (LPC11_GPIO0_BASE+LPC11_GPIO_IS_OFFSET)
#define LPC11_GPIO0_IBE                   (LPC11_GPIO0_BASE+LPC11_GPIO_IBE_OFFSET)
#define LPC11_GPIO0_IEV                   (LPC11_GPIO0_BASE+LPC11_GPIO_IVE_OFFSET)
#define LPC11_GPIO0_IE                    (LPC11_GPIO0_BASE+LPC11_GPIO_IE_OFFSET)
#define LPC11_GPIO0_RIS                   (LPC11_GPIO0_BASE+LPC11_GPIO_RIS_OFFSET)
#define LPC11_GPIO0_MIS                   (LPC11_GPIO0_BASE+LPC11_GPIO_MIS_OFFSET)
#define LPC11_GPIO0_IC                    (LPC11_GPIO0_BASE+LPC11_GPIO_IC_OFFSET)

#define LPC11_GPIO1_DATA                  (LPC11_GPIO1_BASE+LPC11_GPIO_DATA_OFFSET)
#define LPC11_GPIO1_DIR                   (LPC11_GPIO1_BASE+LPC11_GPIO_DIR_OFFSET)
#define LPC11_GPIO1_IS                    (LPC11_GPIO1_BASE+LPC11_GPIO_IS_OFFSET)
#define LPC11_GPIO1_IBE                   (LPC11_GPIO1_BASE+LPC11_GPIO_IBE_OFFSET)
#define LPC11_GPIO1_IEV                   (LPC11_GPIO1_BASE+LPC11_GPIO_IVE_OFFSET)
#define LPC11_GPIO1_IE                    (LPC11_GPIO1_BASE+LPC11_GPIO_IE_OFFSET)
#define LPC11_GPIO1_RIS                   (LPC11_GPIO1_BASE+LPC11_GPIO_RIS_OFFSET)
#define LPC11_GPIO1_MIS                   (LPC11_GPIO1_BASE+LPC11_GPIO_MIS_OFFSET)
#define LPC11_GPIO1_IC                    (LPC11_GPIO1_BASE+LPC11_GPIO_IC_OFFSET)

#define LPC11_GPIO2_DATA                  (LPC11_GPIO2_BASE+LPC11_GPIO_DATA_OFFSET)
#define LPC11_GPIO2_DIR                   (LPC11_GPIO2_BASE+LPC11_GPIO_DIR_OFFSET)
#define LPC11_GPIO2_IS                    (LPC11_GPIO2_BASE+LPC11_GPIO_IS_OFFSET)
#define LPC11_GPIO2_IBE                   (LPC11_GPIO2_BASE+LPC11_GPIO_IBE_OFFSET)
#define LPC11_GPIO2_IEV                   (LPC11_GPIO2_BASE+LPC11_GPIO_IVE_OFFSET)
#define LPC11_GPIO2_IE                    (LPC11_GPIO2_BASE+LPC11_GPIO_IE_OFFSET)
#define LPC11_GPIO2_RIS                   (LPC11_GPIO2_BASE+LPC11_GPIO_RIS_OFFSET)
#define LPC11_GPIO2_MIS                   (LPC11_GPIO2_BASE+LPC11_GPIO_MIS_OFFSET)
#define LPC11_GPIO2_IC                    (LPC11_GPIO2_BASE+LPC11_GPIO_IC_OFFSET)

#define LPC11_GPIO3_DATA                  (LPC11_GPIO3_BASE+LPC11_GPIO_DATA_OFFSET)
#define LPC11_GPIO3_DIR                   (LPC11_GPIO3_BASE+LPC11_GPIO_DIR_OFFSET)
#define LPC11_GPIO3_IS                    (LPC11_GPIO3_BASE+LPC11_GPIO_IS_OFFSET)
#define LPC11_GPIO3_IBE                   (LPC11_GPIO3_BASE+LPC11_GPIO_IBE_OFFSET)
#define LPC11_GPIO3_IEV                   (LPC11_GPIO3_BASE+LPC11_GPIO_IVE_OFFSET)
#define LPC11_GPIO3_IE                    (LPC11_GPIO3_BASE+LPC11_GPIO_IE_OFFSET)
#define LPC11_GPIO3_RIS                   (LPC11_GPIO3_BASE+LPC11_GPIO_RIS_OFFSET)
#define LPC11_GPIO3_MIS                   (LPC11_GPIO3_BASE+LPC11_GPIO_MIS_OFFSET)
#define LPC11_GPIO3_IC                    (LPC11_GPIO3_BASE+LPC11_GPIO_IC_OFFSET)

/* Register bit definitions *********************************************************/
/* GPIO block register bit definitions **********************************************/

#define GPIO(n)                           (1 << (n)) /* n=0,1,..11 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_LPC11_CHIP_GPIO_H */
