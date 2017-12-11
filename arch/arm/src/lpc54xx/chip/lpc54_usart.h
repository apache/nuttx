/*****************************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc54_usart.h
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
 *****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_USART_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_USART_H

/*****************************************************************************************************
 * Included Files
 *****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/*****************************************************************************************************
 * Pre-processor Definitions
 *****************************************************************************************************/

#define LPC54_USART_FIFO_DEPTH          8       /* All FIFOs are 16x8-bits */

/* USART Register Offsets ***************************************************************************/

/* Registers for the USART function: */

#define LPC54_USART_CFG_OFFSET          0x0000  /* USART Configuration register */
#define LPC54_USART_CTL_OFFSET          0x0004  /* USART Control register */
#define LPC54_USART_STAT_OFFSET         0x0008  /* USART Status register */
#define LPC54_USART_INTENSET_OFFSET     0x000c  /* USART Interrupt Enable read and Set register */
#define LPC54_USART_INTENCLR_OFFSET     0x0010  /* USART Interrupt Enable Clear register */
#define LPC54_USART_BRG_OFFSET          0x0020  /* USART Baud Rate Generator register */
#define LPC54_USART_INTSTAT_OFFSET      0x0024  /* USART Interrupt status register */
#define LPC54_USART_OSR_OFFSET          0x0028  /* USART Oversample selection register */

/* Registers for FIFO control and data access: */

#define LPC54_USART_FIFOCFG_OFFSET      0x0e00  /* FIFO configuration and enable register */
#define LPC54_USART_FIFOSTAT_OFFSET     0x0e04  /* FIFO status register */
#define LPC54_USART_FIFOTRIG_OFFSET     0x0e08  /* FIFO trigger settings for interrupt and DMA request */
#define LPC54_USART_FIFOINTENSET_OFFSET 0x0e10  /* FIFO interrupt enable set (enable) and read register */
#define LPC54_USART_FIFOINTENCLR_OFFSET 0x0e14  /* FIFO interrupt enable clear (disable) and read register */
#define LPC54_USART_FIFOINTSTAT_OFFSET  0x0e18  /* FIFO interrupt status register */
#define LPC54_USART_FIFOWR_OFFSET       0x0e20  /* FIFO write data */
#define LPC54_USART_FIFORD_OFFSET       0x0e30  /* FIFO read data */
#define LPC54_USART_FIFORDNOPOP_OFFSET  0x0e40  /* FIFO data read with no FIFO pop */

/* ID register: */

#define LPC54_USART_ID_OFFSET           0x0ffc  /* USART module Identification */

/* USART Register Adreesses **************************************************************************/

#define LPC54_USART0_CFG                (LPC54_FLEXCOMM0_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART0_CTL                (LPC54_FLEXCOMM0_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART0_STAT               (LPC54_FLEXCOMM0_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART0_INTENSET           (LPC54_FLEXCOMM0_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART0_INTENCLR           (LPC54_FLEXCOMM0_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART0_BRG                (LPC54_FLEXCOMM0_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART0_INTSTAT            (LPC54_FLEXCOMM0_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART0_OSR                (LPC54_FLEXCOMM0_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART0_FIFOCFG            (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART0_FIFOSTAT           (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART0_FIFOTRIG           (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART0_FIFOINTENSET       (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART0_FIFOINTENCLR       (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART0_FIFOINTSTAT        (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART0_FIFOWR             (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART0_FIFORD             (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART0_FIFORDNOPOP        (LPC54_FLEXCOMM0_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART0_ID                 (LPC54_FLEXCOMM0_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART1_CFG                (LPC54_FLEXCOMM1_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART1_CTL                (LPC54_FLEXCOMM1_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART1_STAT               (LPC54_FLEXCOMM1_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART1_INTENSET           (LPC54_FLEXCOMM1_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART1_INTENCLR           (LPC54_FLEXCOMM1_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART1_BRG                (LPC54_FLEXCOMM1_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART1_INTSTAT            (LPC54_FLEXCOMM1_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART1_OSR                (LPC54_FLEXCOMM1_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART1_FIFOCFG            (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART1_FIFOSTAT           (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART1_FIFOTRIG           (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART1_FIFOINTENSET       (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART1_FIFOINTENCLR       (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART1_FIFOINTSTAT        (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART1_FIFOWR             (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART1_FIFORD             (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART1_FIFORDNOPOP        (LPC54_FLEXCOMM1_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART1_ID                 (LPC54_FLEXCOMM1_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART2_CFG                (LPC54_FLEXCOMM2_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART2_CTL                (LPC54_FLEXCOMM2_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART2_STAT               (LPC54_FLEXCOMM2_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART2_INTENSET           (LPC54_FLEXCOMM2_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART2_INTENCLR           (LPC54_FLEXCOMM2_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART2_BRG                (LPC54_FLEXCOMM2_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART2_INTSTAT            (LPC54_FLEXCOMM2_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART2_OSR                (LPC54_FLEXCOMM2_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART2_FIFOCFG            (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART2_FIFOSTAT           (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART2_FIFOTRIG           (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART2_FIFOINTENSET       (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART2_FIFOINTENCLR       (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART2_FIFOINTSTAT        (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART2_FIFOWR             (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART2_FIFORD             (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART2_FIFORDNOPOP        (LPC54_FLEXCOMM2_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART2_ID                 (LPC54_FLEXCOMM2_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART3_CFG                (LPC54_FLEXCOMM3_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART3_CTL                (LPC54_FLEXCOMM3_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART3_STAT               (LPC54_FLEXCOMM3_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART3_INTENSET           (LPC54_FLEXCOMM3_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART3_INTENCLR           (LPC54_FLEXCOMM3_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART3_BRG                (LPC54_FLEXCOMM3_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART3_INTSTAT            (LPC54_FLEXCOMM3_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART3_OSR                (LPC54_FLEXCOMM3_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART3_FIFOCFG            (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART3_FIFOSTAT           (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART3_FIFOTRIG           (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART3_FIFOINTENSET       (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART3_FIFOINTENCLR       (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART3_FIFOINTSTAT        (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART3_FIFOWR             (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART3_FIFORD             (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART3_FIFORDNOPOP        (LPC54_FLEXCOMM3_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART3_ID                 (LPC54_FLEXCOMM3_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART4_CFG                (LPC54_FLEXCOMM4_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART4_CTL                (LPC54_FLEXCOMM4_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART4_STAT               (LPC54_FLEXCOMM4_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART4_INTENSET           (LPC54_FLEXCOMM4_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART4_INTENCLR           (LPC54_FLEXCOMM4_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART4_BRG                (LPC54_FLEXCOMM4_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART4_INTSTAT            (LPC54_FLEXCOMM4_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART4_OSR                (LPC54_FLEXCOMM4_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART4_FIFOCFG            (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART4_FIFOSTAT           (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART4_FIFOTRIG           (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART4_FIFOINTENSET       (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART4_FIFOINTENCLR       (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART4_FIFOINTSTAT        (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART4_FIFOWR             (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART4_FIFORD             (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART4_FIFORDNOPOP        (LPC54_FLEXCOMM4_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART4_ID                 (LPC54_FLEXCOMM4_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART5_CFG                (LPC54_FLEXCOMM5_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART5_CTL                (LPC54_FLEXCOMM5_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART5_STAT               (LPC54_FLEXCOMM5_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART5_INTENSET           (LPC54_FLEXCOMM5_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART5_INTENCLR           (LPC54_FLEXCOMM5_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART5_BRG                (LPC54_FLEXCOMM5_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART5_INTSTAT            (LPC54_FLEXCOMM5_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART5_OSR                (LPC54_FLEXCOMM5_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART5_FIFOCFG            (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART5_FIFOSTAT           (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART5_FIFOTRIG           (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART5_FIFOINTENSET       (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART5_FIFOINTENCLR       (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART5_FIFOINTSTAT        (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART5_FIFOWR             (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART5_FIFORD             (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART5_FIFORDNOPOP        (LPC54_FLEXCOMM5_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART5_ID                 (LPC54_FLEXCOMM5_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART6_CFG                (LPC54_FLEXCOMM6_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART6_CTL                (LPC54_FLEXCOMM6_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART6_STAT               (LPC54_FLEXCOMM6_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART6_INTENSET           (LPC54_FLEXCOMM6_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART6_INTENCLR           (LPC54_FLEXCOMM6_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART6_BRG                (LPC54_FLEXCOMM6_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART6_INTSTAT            (LPC54_FLEXCOMM6_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART6_OSR                (LPC54_FLEXCOMM6_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART6_FIFOCFG            (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART6_FIFOSTAT           (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART6_FIFOTRIG           (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART6_FIFOINTENSET       (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART6_FIFOINTENCLR       (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART6_FIFOINTSTAT        (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART6_FIFOWR             (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART6_FIFORD             (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART6_FIFORDNOPOP        (LPC54_FLEXCOMM6_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART6_ID                 (LPC54_FLEXCOMM6_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART7_CFG                (LPC54_FLEXCOMM7_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART7_CTL                (LPC54_FLEXCOMM7_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART7_STAT               (LPC54_FLEXCOMM7_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART7_INTENSET           (LPC54_FLEXCOMM7_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART7_INTENCLR           (LPC54_FLEXCOMM7_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART7_BRG                (LPC54_FLEXCOMM7_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART7_INTSTAT            (LPC54_FLEXCOMM7_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART7_OSR                (LPC54_FLEXCOMM7_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART7_FIFOCFG            (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART7_FIFOSTAT           (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART7_FIFOTRIG           (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART7_FIFOINTENSET       (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART7_FIFOINTENCLR       (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART7_FIFOINTSTAT        (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART7_FIFOWR             (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART7_FIFORD             (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART7_FIFORDNOPOP        (LPC54_FLEXCOMM7_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART7_ID                 (LPC54_FLEXCOMM7_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART8_CFG                (LPC54_FLEXCOMM8_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART8_CTL                (LPC54_FLEXCOMM8_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART8_STAT               (LPC54_FLEXCOMM8_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART8_INTENSET           (LPC54_FLEXCOMM8_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART8_INTENCLR           (LPC54_FLEXCOMM8_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART8_BRG                (LPC54_FLEXCOMM8_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART8_INTSTAT            (LPC54_FLEXCOMM8_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART8_OSR                (LPC54_FLEXCOMM8_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART8_FIFOCFG            (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART8_FIFOSTAT           (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART8_FIFOTRIG           (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART8_FIFOINTENSET       (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART8_FIFOINTENCLR       (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART8_FIFOINTSTAT        (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART8_FIFOWR             (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART8_FIFORD             (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART8_FIFORDNOPOP        (LPC54_FLEXCOMM8_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART8_ID                 (LPC54_FLEXCOMM8_BASE + LPC54_USART_ID_OFFSET)

#define LPC54_USART9_CFG                (LPC54_FLEXCOMM9_BASE + LPC54_USART_CFG_OFFSET)
#define LPC54_USART9_CTL                (LPC54_FLEXCOMM9_BASE + LPC54_USART_CTL_OFFSET)
#define LPC54_USART9_STAT               (LPC54_FLEXCOMM9_BASE + LPC54_USART_STAT_OFFSET)
#define LPC54_USART9_INTENSET           (LPC54_FLEXCOMM9_BASE + LPC54_USART_INTENSET_OFFSET)
#define LPC54_USART9_INTENCLR           (LPC54_FLEXCOMM9_BASE + LPC54_USART_INTENCLR_OFFSET)
#define LPC54_USART9_BRG                (LPC54_FLEXCOMM9_BASE + LPC54_USART_BRG_OFFSET)
#define LPC54_USART9_INTSTAT            (LPC54_FLEXCOMM9_BASE + LPC54_USART_INTSTAT_OFFSET)
#define LPC54_USART9_OSR                (LPC54_FLEXCOMM9_BASE + LPC54_USART_OSR_OFFSET)
#define LPC54_USART9_FIFOCFG            (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOCFG_OFFSET)
#define LPC54_USART9_FIFOSTAT           (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOSTAT_OFFSET)
#define LPC54_USART9_FIFOTRIG           (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOTRIG_OFFSET)
#define LPC54_USART9_FIFOINTENSET       (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOINTENSET_OFFSET)
#define LPC54_USART9_FIFOINTENCLR       (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOINTENCLR_OFFSET)
#define LPC54_USART9_FIFOINTSTAT        (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOINTSTAT_OFFSET)
#define LPC54_USART9_FIFOWR             (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFOWR_OFFSET)
#define LPC54_USART9_FIFORD             (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFORD_OFFSET)
#define LPC54_USART9_FIFORDNOPOP        (LPC54_FLEXCOMM9_BASE + LPC54_USART_FIFORDNOPOP_OFFSET)
#define LPC54_USART9_ID                 (LPC54_FLEXCOMM9_BASE + LPC54_USART_ID_OFFSET)

/* USART Register Bitfield Definitions ***************************************************************/

/* USART Configuration register */

#define USART_CFG_ENABLE                (1 << 0)  /* Bit 0  USART Enable */
#define USART_CFG_DATALEN_SHIFT         (2)       /* Bits 2-3: Selects the data size for the USART */
#define USART_CFG_DATALEN_MASK          (3 << USART_CFG_DATALEN_SHIFT)
#  define USART_CFG_DATALEN_7BIT        (0 << USART_CFG_DATALEN_SHIFT) /* 7 bit Data length */
#  define USART_CFG_DATALEN_8BIT        (1 << USART_CFG_DATALEN_SHIFT) /* 8 bit Data length */
#  define USART_CFG_DATALEN_9BIT        (2 << USART_CFG_DATALEN_SHIFT) /* 9 bit data lengt */
#define USART_CFG_PARITYSEL_SHIFT       (4)       /* Bits 4-5: Selects what type of parity is used by the USART */
#define USART_CFG_PARITYSEL_MASK        (3 << USART_CFG_PARITYSEL_SHIFT)
#  define USART_CFG_PARITYSEL_NONE      (0 << USART_CFG_PARITYSEL_SHIFT) /* No parity */
#  define USART_CFG_PARITYSEL_EVEN      (2 << USART_CFG_PARITYSEL_SHIFT) /* Even parity */
#  define USART_CFG_PARITYSEL_ODD       (3 << USART_CFG_PARITYSEL_SHIFT) /* Odd parity */
#define USART_CFG_STOPLEN               (1 << 6)  /* Bit 6  Number of stop bits appended to transmitted data */
#define USART_CFG_MODE32K               (1 << 7)  /* Bit 7  Selects standard or 32 kHz clocking mode */
#define USART_CFG_LINMODE               (1 << 8)  /* Bit 8  LIN break mode enable */
#define USART_CFG_CTSEN                 (1 << 9)  /* Bit 9  CTS Enable */
#define USART_CFG_SYNCEN                (1 << 11) /* Bit 11 Selects synchronous or asynchronous operation */
#define USART_CFG_CLKPOL                (1 << 12) /* Bit 12 Selects clock polarity and sampling edge of RX data */
#define USART_CFG_SYNCMST               (1 << 14) /* Bit 14 Synchronous mode Master select */
#define USART_CFG_LOOP                  (1 << 15) /* Bit 15 Selects data loopback mode */
#define USART_CFG_OETA                  (1 << 18) /* Bit 18 Output Enable Turnaround time enable for RS-485 operation */
#define USART_CFG_AUTOADDR              (1 << 19) /* Bit 19 Automatic Address matching enable */
#define USART_CFG_OESEL                 (1 << 20) /* Bit 20 Output Enable Select */
#define USART_CFG_OEPOL                 (1 << 21) /* Bit 21 Output Enable Polarity */
#define USART_CFG_RXPOL                 (1 << 22) /* Bit 22 Receive data polarity */
#define USART_CFG_TXPOL                 (1 << 23) /* Bit 23 Transmit data polarity */

/* USART Control register */

#define USART_CTL_TXBRKEN               (1 << 1)  /* Bit 1:  Break Enable */
#define USART_CTL_ADDRDET               (1 << 2)  /* Bit 2:  Enable address detect mode */
#define USART_CTL_TXDIS                 (1 << 6)  /* Bit 6:  Transmit Disable */
#define USART_CTL_CC                    (1 << 8)  /* Bit 8:  Continuous Clock generation */
#define USART_CTL_CLRCCONRX             (1 << 9)  /* Bit 9:  Clear Continuous Clock */
#define USART_CTL_AUTOBAUD              (1 << 16) /* Bit 16: Autobaud enable */

/* USART Status register, USART Interrupt Enable read and Set register, and USART Interrupt Enable Clear register */

#define USART_INTSTAT_RXIDLE            (1 << 1)  /* Bit 1:  Receiver Idle (Status only) */
#define USART_INT_TXIDLE                (1 << 3)  /* Bit 3:  Transmitter Idle */
#define USART_INTSTAT_CTS               (1 << 4)  /* Bit 4:  State of the CTS signal (Status only) */
#define USART_INT_DELTACTS              (1 << 5)  /* Bit 5:  Change in the state of CTS flag */
#define USART_INT_TXDIS                 (1 << 6)  /* Bit 6:  Transmitter Disabled Status flag */
#define USART_INTSTAT_RXBRK             (1 << 10) /* Bit 10: Received Break (Status only) */
#define USART_INT_DELTARXBRK            (1 << 11) /* Bit 11: Change in the state of receiver break detection */
#define USART_INT_START                 (1 << 12) /* Bit 12: Start detected on the receiver input */
#define USART_INT_FRAMERR               (1 << 13) /* Bit 13: Framing Error interrupt flag */
#define USART_INT_PARITYER              (1 << 14) /* Bit 14: Parity Error interrupt flag */
#define USART_INT_RXNOISE               (1 << 15) /* Bit 15: Received Noise interrupt flag */
#define USART_INT_ABERR                 (1 << 16) /* Bit 16: Auto baud Error */

/* USART Baud Rate Generator register  */

#define USART_BRG_SHIFT                 (0)       /* Bits 0-15: BAUD rate divisor */
#define USART_BRG_MASK                  (0xffff << USART_OSR_SHIFT)
#  define USART_BRG(n)                  ((uint32)((n)-1) << USART_OSR_SHIFT)

/* USART Oversample selection register */

#define USART_OSR_SHIFT                 (0)       /* Bits 0-3: Oversample Selection Value. */
#define USART_OSR_MASK                  (15 << USART_OSR_SHIFT)
#  define USART_OSR(n)                  ((uint32)((n)-1) << USART_OSR_SHIFT)

/* FIFO configuration and enable register */

#define USART_FIFOCFG_ENABLETX          (1 << 0)  /* Bit 0:  Enable the transmit FIFO */
#define USART_FIFOCFG_ENABLERX          (1 << 1)  /* Bit 1:  Enable the receive FIFO */
#define USART_FIFOCFG_SIZE_SHIFT        (4)       /* Bits 4-5:  FIFO size configuration */
#define USART_FIFOCFG_SIZE_MASK         (3 << USART_FIFOCFG_SIZE_SHIFT)
#  define USART_FIFOCFG_SIZE_16x8       (0 << USART_FIFOCFG_SIZE_SHIFT) /* FIFO is 16 entries x 8 bits */
#define USART_FIFOCFG_DMATX             (1 << 12) /* Bit 12: DMA configuration for transmit */
#define USART_FIFOCFG_DMARX             (1 << 13) /* Bit 13: DMA configuration for receive */
#define USART_FIFOCFG_WAKETX            (1 << 14) /* Bit 14: Wake-up for transmit FIFO level */
#define USART_FIFOCFG_WAKERX            (1 << 15) /* Bit 15: Wake-up for receive FIFO level */
#define USART_FIFOCFG_EMPTYTX           (1 << 16) /* Bit 16: Empty command for the transmit FIFO */
#define USART_FIFOCFG_EMPTYRX           (1 << 17) /* Bit 17: Empty command for the receive FIFO */

/* FIFO status register */

#define USART_FIFOSTAT_TXERR            (1 << 0)  /* Bit 0  TX FIFO error */
#define USART_FIFOSTAT_RXERR            (1 << 1)  /* Bit 1  RX FIFO error */
#define USART_FIFOSTAT_PERINT           (1 << 3)  /* Bit 3  Peripheral interrupt */
#define USART_FIFOSTAT_TXEMPTY          (1 << 4)  /* Bit 4  Transmit FIFO empty */
#define USART_FIFOSTAT_TXNOTFULL        (1 << 5)  /* Bit 5  Transmit FIFO not full */
#define USART_FIFOSTAT_RXNOTEMPTY       (1 << 6)  /* Bit 6  Receive FIFO not empty */
#define USART_FIFOSTAT_RXFULL           (1 << 7)  /* Bit 7  Receive FIFO full */
#define USART_FIFOSTAT_TXLVL_SHIFT      (8)       /* Bits 8-12: Transmit FIFO current level */
#define USART_FIFOSTAT_TXLVL_MASK       (31 << USART_FIFOSTAT_TXLVL_SHIFT)
#define USART_FIFOSTAT_RXLVL_SHIFT      (16)      /* Bits 16-20: Receive FIFO current level */
#define USART_FIFOSTAT_RXLVL_MASK       (31 << USART_FIFOSTAT_RXLVL_SHIFT)

/* FIFO trigger settings for interrupt and DMA request */

#define USART_FIFOTRIG_TXLVLENA         (1 << 0)  /* Bit 0:  Transmit FIFO level trigger enable */
#define USART_FIFOTRIG_RXLVLENA         (1 << 1)  /* Bit 1:  Receive FIFO level trigger enable */
#define USART_FIFOTRIG_TXLVL_SHIFT      (8)       /* Bits 8-11: Transmit FIFO level trigger point */
#define USART_FIFOTRIG_TXLVL_MASK       (15 << USART_FIFOTRIG_TXLVL_SHIFT)
#  define USART_FIFOTRIG_TXLVL(n)       ((uint32_t)(n) << USART_FIFOTRIG_TXLVL_SHIFT) /* Interrupt when n entries */
#  define USART_FIFOTRIG_TXLVL_EMPTY    USART_FIFOTRIG_TXLVL(0)
#  define USART_FIFOTRIG_TXLVL_NOTFULL  USART_FIFOTRIG_TXLVL(15)
#define USART_FIFOTRIG_RXLVL_SHIFT      (16)      /* Bits 16-19: Receive FIFO level trigger point */
#define USART_FIFOTRIG_RXLVL_MASK       (15 << USART_FIFOTRIG_RXLVL_SHIFT)
#  define USART_FIFOTRIG_RXLVL(n)       ((uint32_t)(n) << USART_FIFOTRIG_RXLVL_SHIFT) /* Interrupt when n+1 entries */
#  define USART_FIFOTRIG_RXLVL_NOTEMPY   USART_FIFOTRIG_RXLVL(0)
#  define USART_FIFOTRIG_RXLVL_FULL      USART_FIFOTRIG_RXLVL(15)

/* FIFO interrupt status register, FIFO interrupt enable set (enable), and read register and FIFO interrupt enable
 * clear (disable) and read register
 */

#define USART_FIFOINT_TXERR             (1 << 0)  /* Bit 0:  Transmit FIFO error interrupt */
#define USART_FIFOINT_RXERR             (1 << 1)  /* Bit 1:  Receive ERROR error interrupt */
#define USART_FIFOINT_TXLVL             (1 << 2)  /* Bit 2:  Transmit FIFO level interrupt */
#define USART_FIFOINT_RXLVL             (1 << 3)  /* Bit 3:  Receive FIFO level interrupt */
#define USART_FIFOINTSTAT_PERINT        (1 << 4)  /* Bit 4:  Peripheral interrupt (Status only) */

#define USART_FIFOINT_ALL               0x0000000f

/* FIFO write data */

#define USART_FIFOWR_TXDATA_SHIFT       (0)       /* Bits 0-8:  Transmit data to the FIFO */
#define USART_FIFOWR_TXDATA_MASK        (0x1ff << USART_FIFOWR_TXDATA_SHIFT)
#  define USART_FIFOWR_TXDATA(n)        ((uint32_t)(n) << USART_FIFOWR_TXDATA_SHIFT)

/* FIFO read data register and FIFO data read with no FIFO pop register */

#define USART_FIFORD_RXDATA_SHIFT       (0)       /* Bits 0-8:  Received data from the FIFO */
#define USART_FIFORD_RXDATA_MASK        (0x1ff << USART_FIFOWR_TXDATA_SHIFT)

#define USART_FIFORD_FRAMERR            (1 << 13) /* Bit 13: Framing Error status flag */
#define USART_FIFORD_PARITYERR          (1 << 14) /* Bit 14: Parity Error status flag */
#define USART_FIFORD_RXNOISE            (1 << 15) /* Bit 15: Received Noise flag */

/* USART module Identification */

#define USART_ID_APERTURE_SHIFT         (0)       /* Bits 0-7:   Aperture encoded as (aperture size/4K) -1 */
#define USART_ID_APERTURE_MASKX         (0xff << USART_ID_APERTURE_SHIFT)
#define USART_ID_MINOR_SHIFT            (8)       /* Bits 8-11:  Minor revision of module implementation */
#define USART_ID_MINOR_MASKX            (15 << USART_ID_MINOR_SHIFT)
#define USART_ID_MAJOR_SHIFT            (12)      /* Bits 12-15: Major revision of module implementation */
#define USART_ID_MAJOR_MASKX            (15 << USART_ID_MAJOR_SHIFT)
#define USART_ID_ID_SHIFT               (16)      /* Bits 16-31: ID Unique module identifier for this IP block */
#define USART_ID_ID_MASKX               (0xffff << USART_ID_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_USART_H */
