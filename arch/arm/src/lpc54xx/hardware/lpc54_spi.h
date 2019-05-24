/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_spi.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SPI_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SPI_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

/* Registers for the SPI function */

#define LPC54_SPI_CFG_OFFSET          0x0400  /* SPI Configuration register */
#define LPC54_SPI_DLY_OFFSET          0x0404  /* SPI Delay register */
#define LPC54_SPI_STAT_OFFSET         0x0408  /* SPI Status */
#define LPC54_SPI_INTENSET_OFFSET     0x040c  /* SPI Interrupt Enable read and set */
#define LPC54_SPI_INTENCLR_OFFSET     0x0410  /* SPI Interrupt Enable Clear */
#define LPC54_SPI_DIV_OFFSET          0x0424  /* SPI clock Divider */
#define LPC54_SPI_INTSTAT_OFFSET      0x0428  /* SPI Interrupt Status */

/* Registers for FIFO control and data access */

#define LPC54_SPI_FIFOCFG_OFFSET      0x0e00  /* FIFO configuration and enable register */
#define LPC54_SPI_FIFOSTAT_OFFSET     0x0e04  /* FIFO status register */
#define LPC54_SPI_FIFOTRIG_OFFSET     0x0e08  /* FIFO trigger level settings for interrupt and DMA request */
#define LPC54_SPI_FIFOINTENSET_OFFSET 0x0e10  /* FIFO interrupt enable set (enable) and read register */
#define LPC54_SPI_FIFOINTENCLR_OFFSET 0x0e14  /* FIFO interrupt enable clear (disable) and read register */
#define LPC54_SPI_FIFOINTSTAT_OFFSET  0x0e18  /* FIFO interrupt status register */
#define LPC54_SPI_FIFOWR_OFFSET       0x0e20  /* FIFO write data */
#define LPC54_SPI_FIFORD_OFFSET       0x0e30  /* FIFO read data */
#define LPC54_SPI_FIFORDNOPOP_OFFSET  0x0e40  /* FIFO data read with no FIFO pop */

/* ID register */

#define LPC54_SPI_ID_OFFSET           0x0ffc  /* SPI module Identification */

/* Register addresses *******************************************************************************/

#define LPC54_SPI0_CFG                (LPC54_FLEXCOMM0_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI0_DLY                (LPC54_FLEXCOMM0_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI0_STAT               (LPC54_FLEXCOMM0_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI0_INTENSET           (LPC54_FLEXCOMM0_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI0_INTENCLR           (LPC54_FLEXCOMM0_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI0_DIV                (LPC54_FLEXCOMM0_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI0_INTSTAT            (LPC54_FLEXCOMM0_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI0_FIFOCFG            (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI0_FIFOSTAT           (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI0_FIFOTRIG           (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI0_FIFOINTENSET       (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI0_FIFOINTENCLR       (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI0_FIFOINTSTAT        (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI0_FIFOWR             (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI0_FIFORD             (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI0_FIFORDNOPOP        (LPC54_FLEXCOMM0_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI0_ID                 (LPC54_FLEXCOMM0_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI1_CFG                (LPC54_FLEXCOMM1_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI1_DLY                (LPC54_FLEXCOMM1_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI1_STAT               (LPC54_FLEXCOMM1_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI1_INTENSET           (LPC54_FLEXCOMM1_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI1_INTENCLR           (LPC54_FLEXCOMM1_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI1_DIV                (LPC54_FLEXCOMM1_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI1_INTSTAT            (LPC54_FLEXCOMM1_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI1_FIFOCFG            (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI1_FIFOSTAT           (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI1_FIFOTRIG           (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI1_FIFOINTENSET       (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI1_FIFOINTENCLR       (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI1_FIFOINTSTAT        (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI1_FIFOWR             (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI1_FIFORD             (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI1_FIFORDNOPOP        (LPC54_FLEXCOMM1_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI1_ID                 (LPC54_FLEXCOMM1_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI2_CFG                (LPC54_FLEXCOMM2_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI2_DLY                (LPC54_FLEXCOMM2_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI2_STAT               (LPC54_FLEXCOMM2_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI2_INTENSET           (LPC54_FLEXCOMM2_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI2_INTENCLR           (LPC54_FLEXCOMM2_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI2_DIV                (LPC54_FLEXCOMM2_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI2_INTSTAT            (LPC54_FLEXCOMM2_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI2_FIFOCFG            (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI2_FIFOSTAT           (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI2_FIFOTRIG           (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI2_FIFOINTENSET       (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI2_FIFOINTENCLR       (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI2_FIFOINTSTAT        (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI2_FIFOWR             (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI2_FIFORD             (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI2_FIFORDNOPOP        (LPC54_FLEXCOMM2_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI2_ID                 (LPC54_FLEXCOMM2_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI3_CFG                (LPC54_FLEXCOMM3_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI3_DLY                (LPC54_FLEXCOMM3_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI3_STAT               (LPC54_FLEXCOMM3_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI3_INTENSET           (LPC54_FLEXCOMM3_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI3_INTENCLR           (LPC54_FLEXCOMM3_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI3_DIV                (LPC54_FLEXCOMM3_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI3_INTSTAT            (LPC54_FLEXCOMM3_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI3_FIFOCFG            (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI3_FIFOSTAT           (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI3_FIFOTRIG           (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI3_FIFOINTENSET       (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI3_FIFOINTENCLR       (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI3_FIFOINTSTAT        (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI3_FIFOWR             (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI3_FIFORD             (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI3_FIFORDNOPOP        (LPC54_FLEXCOMM3_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI3_ID                 (LPC54_FLEXCOMM3_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI4_CFG                (LPC54_FLEXCOMM4_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI4_DLY                (LPC54_FLEXCOMM4_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI4_STAT               (LPC54_FLEXCOMM4_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI4_INTENSET           (LPC54_FLEXCOMM4_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI4_INTENCLR           (LPC54_FLEXCOMM4_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI4_DIV                (LPC54_FLEXCOMM4_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI4_INTSTAT            (LPC54_FLEXCOMM4_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI4_FIFOCFG            (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI4_FIFOSTAT           (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI4_FIFOTRIG           (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI4_FIFOINTENSET       (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI4_FIFOINTENCLR       (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI4_FIFOINTSTAT        (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI4_FIFOWR             (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI4_FIFORD             (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI4_FIFORDNOPOP        (LPC54_FLEXCOMM4_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI4_ID                 (LPC54_FLEXCOMM4_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI5_CFG                (LPC54_FLEXCOMM5_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI5_DLY                (LPC54_FLEXCOMM5_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI5_STAT               (LPC54_FLEXCOMM5_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI5_INTENSET           (LPC54_FLEXCOMM5_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI5_INTENCLR           (LPC54_FLEXCOMM5_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI5_DIV                (LPC54_FLEXCOMM5_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI5_INTSTAT            (LPC54_FLEXCOMM5_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI5_FIFOCFG            (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI5_FIFOSTAT           (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI5_FIFOTRIG           (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI5_FIFOINTENSET       (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI5_FIFOINTENCLR       (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI5_FIFOINTSTAT        (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI5_FIFOWR             (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI5_FIFORD             (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI5_FIFORDNOPOP        (LPC54_FLEXCOMM5_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI5_ID                 (LPC54_FLEXCOMM5_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI6_CFG                (LPC54_FLEXCOMM6_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI6_DLY                (LPC54_FLEXCOMM6_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI6_STAT               (LPC54_FLEXCOMM6_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI6_INTENSET           (LPC54_FLEXCOMM6_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI6_INTENCLR           (LPC54_FLEXCOMM6_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI6_DIV                (LPC54_FLEXCOMM6_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI6_INTSTAT            (LPC54_FLEXCOMM6_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI6_FIFOCFG            (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI6_FIFOSTAT           (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI6_FIFOTRIG           (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI6_FIFOINTENSET       (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI6_FIFOINTENCLR       (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI6_FIFOINTSTAT        (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI6_FIFOWR             (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI6_FIFORD             (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI6_FIFORDNOPOP        (LPC54_FLEXCOMM6_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI6_ID                 (LPC54_FLEXCOMM6_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI7_CFG                (LPC54_FLEXCOMM7_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI7_DLY                (LPC54_FLEXCOMM7_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI7_STAT               (LPC54_FLEXCOMM7_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI7_INTENSET           (LPC54_FLEXCOMM7_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI7_INTENCLR           (LPC54_FLEXCOMM7_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI7_DIV                (LPC54_FLEXCOMM7_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI7_INTSTAT            (LPC54_FLEXCOMM7_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI7_FIFOCFG            (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI7_FIFOSTAT           (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI7_FIFOTRIG           (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI7_FIFOINTENSET       (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI7_FIFOINTENCLR       (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI7_FIFOINTSTAT        (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI7_FIFOWR             (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI7_FIFORD             (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI7_FIFORDNOPOP        (LPC54_FLEXCOMM7_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI7_ID                 (LPC54_FLEXCOMM7_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI8_CFG                (LPC54_FLEXCOMM8_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI8_DLY                (LPC54_FLEXCOMM8_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI8_STAT               (LPC54_FLEXCOMM8_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI8_INTENSET           (LPC54_FLEXCOMM8_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI8_INTENCLR           (LPC54_FLEXCOMM8_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI8_DIV                (LPC54_FLEXCOMM8_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI8_INTSTAT            (LPC54_FLEXCOMM8_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI8_FIFOCFG            (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI8_FIFOSTAT           (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI8_FIFOTRIG           (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI8_FIFOINTENSET       (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI8_FIFOINTENCLR       (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI8_FIFOINTSTAT        (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI8_FIFOWR             (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI8_FIFORD             (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI8_FIFORDNOPOP        (LPC54_FLEXCOMM8_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI8_ID                 (LPC54_FLEXCOMM8_BASE + LPC54_SPI_ID_OFFSET)

#define LPC54_SPI9_CFG                (LPC54_FLEXCOMM9_BASE + LPC54_SPI_CFG_OFFSET)
#define LPC54_SPI9_DLY                (LPC54_FLEXCOMM9_BASE + LPC54_SPI_DLY_OFFSET)
#define LPC54_SPI9_STAT               (LPC54_FLEXCOMM9_BASE + LPC54_SPI_STAT_OFFSET)
#define LPC54_SPI9_INTENSET           (LPC54_FLEXCOMM9_BASE + LPC54_SPI_INTENSET_OFFSET)
#define LPC54_SPI9_INTENCLR           (LPC54_FLEXCOMM9_BASE + LPC54_SPI_INTENCLR_OFFSET)
#define LPC54_SPI9_DIV                (LPC54_FLEXCOMM9_BASE + LPC54_SPI_DIV_OFFSET)
#define LPC54_SPI9_INTSTAT            (LPC54_FLEXCOMM9_BASE + LPC54_SPI_INTSTAT_OFFSET)
#define LPC54_SPI9_FIFOCFG            (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOCFG_OFFSET)
#define LPC54_SPI9_FIFOSTAT           (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOSTAT_OFFSET)
#define LPC54_SPI9_FIFOTRIG           (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOTRIG_OFFSET)
#define LPC54_SPI9_FIFOINTENSET       (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOINTENSET_OFFSET)
#define LPC54_SPI9_FIFOINTENCLR       (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOINTENCLR_OFFSET)
#define LPC54_SPI9_FIFOINTSTAT        (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOINTSTAT_OFFSET)
#define LPC54_SPI9_FIFOWR             (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFOWR_OFFSET)
#define LPC54_SPI9_FIFORD             (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFORD_OFFSET)
#define LPC54_SPI9_FIFORDNOPOP        (LPC54_FLEXCOMM9_BASE + LPC54_SPI_FIFORDNOPOP_OFFSET)
#define LPC54_SPI9_ID                 (LPC54_FLEXCOMM9_BASE + LPC54_SPI_ID_OFFSET)

/* Register bit definitions *************************************************************************/

/* SPI Configuration register */

#define SPI_CFG_ENABLE                (1 << 0)  /* Bit 0:  SPI enable */
#define SPI_CFG_MASTER                (1 << 2)  /* Bit 2:  Master mode select */
#define SPI_CFG_LSBF                  (1 << 3)  /* Bit 3:  LSB First mode enable */
#define SPI_CFG_CPHA                  (1 << 4)  /* Bit 4:  Clock Phase select */
#define SPI_CFG_CPOL                  (1 << 5)  /* Bit 5:  Clock Polarity select */
#define SPI_CFG_LOOP                  (1 << 7)  /* Bit 7:  Loopback mode enable */
#define SPI_CFG_SPOL0                 (1 << 8)  /* Bit 8:  SSEL0 Polarity select */
#define SPI_CFG_SPOL1                 (1 << 9)  /* Bit 9:  SSEL1 Polarity select */
#define SPI_CFG_SPOL2                 (1 << 10) /* Bit 10: SSEL2 Polarity select */
#define SPI_CFG_SPOL3                 (1 << 11) /* Bit 11: SSEL3 Polarity select */

/* SPI Delay register */

#define SPI_DLY_PRE_DELAY_SHIFT       (0)       /* Bits 0-3: Time between SSEL assertion and data transfer */
#define SPI_DLY_PRE_DELAY_MASK        (15 << SPI_DLY_PRE_DELAY_SHIFT)
#  define SPI_DLY_PRE_DELAY(n)        ((uint32_t)(n) << SPI_DLY_PRE_DELAY_SHIFT)
#define SPI_DLY_POST_DELAY_SHIFT      (4)       /* Bits 4-7: Time between tdata transfer and SSEL deassertion */
#define SPI_DLY_POST_DELAY_MASK       (15 << SPI_DLY_POST_DELAY_SHIFT)
#  define SPI_DLY_POST_DELAY(n)       ((uint32_t)(n) << SPI_DLY_POST_DELAY_SHIFT)
#define SPI_DLY_FRAME_DELAY_SHIFT     (8)       /* Bits 8-11: Minimum amount of time between frames */
#define SPI_DLY_FRAME_DELAY_MASK      (15 << SPI_DLY_FRAME_DELAY_SHIFT)
#  define SPI_DLY_FRAME_DELAY(n)      ((uint32_t)(n) << SPI_DLY_FRAME_DELAY_SHIFT)
#define SPI_DLY_TRANSFER_DELAY_SHIFT  (12)      /* Bits 12-15: Time SSEL deasserted between transfers */
#define SPI_DLY_TRANSFER_DELAY_MASK   (15 << SPI_DLY_TRANSFER_DELAY_SHIFT)
#  define SPI_DLY_TRANSFER_DELAY(n)   ((uint32_t)(n) << SPI_DLY_TRANSFER_DELAY_SHIFT)

/* SPI Status register */

#define SPI_STAT_SSA                  (1 << 4)  /* Bit 4:  Slave Select Assert */
#define SPI_STAT_SSD                  (1 << 5)  /* Bit 5:  Slave Select Deassert */
#define SPI_STAT_STALLED              (1 << 6)  /* Bit 6:  Stalled status flag */
#define SPI_STAT_ENDTRANSFER          (1 << 7)  /* Bit 7:  End Transfer control bit */
#define SPI_STAT_MSTIDLE              (1 << 8)  /* Bit 8:  Master idle status flag */

/* SPI Interrupt Enable read and set, SPI Interrupt Enable Clear, and SPI Interrupt Status */

#define SPI_INT_SSA                   (1 << 4)  /* Bit 4:  Slave select assert interrupt */
#define SPI_INT_SSD                   (1 << 5)  /* Bit 5:  Slave select deassert interrupt */
#define SPI_INT_MSTIDLE               (1 << 8)  /* Bit 8:  Master idle interrupt */

/* SPI clock Divider */

#define SPI_DIV_SHIFT                 (0)       /* Bits 0-15: Rate divider value */
#define SPI_DIV_MASK                  (0xffff << SPI_DIV_SHIFT)
#  define SPI_DIV(n)                  ((uint32_t)((n)-1) << SPI_DIV_SHIFT)

/* FIFO configuration and enable register */

#define SPI_FIFOCFG_ENABLETX          (1 << 0)  /* Bit 0:  Enable the transmit FIFO) */
#define SPI_FIFOCFG_ENABLERX          (1 << 1)  /* Bit 1:  Enable the receive FIFO) */
#define SPI_FIFOCFG_SIZE_SHIFT        (4)       /* Bits 4-5:  FIFO size configuration (read-only) */
#define SPI_FIFOCFG_SIZE_MASK         (3 << SPI_FIFOCFG_SIZE_SHIFT)
#  define SPI_FIFOCFG_SIZE_8x16       (1 << SPI_FIFOCFG_SIZE_SHIFT) /* FIFO is configured as 8 entries of 16 bits */
#define SPI_FIFOCFG_DMATX             (1 << 12) /* Bit 12: DMA configuration for transmit */
#define SPI_FIFOCFG_DMARX             (1 << 13) /* Bit 13: DMA configuration for receive */
#define SPI_FIFOCFG_WAKETX            (1 << 14) /* Bit 14: Wake-up for transmit FIFO level */
#define SPI_FIFOCFG_WAKERX            (1 << 15) /* Bit 15: Wake-up for receive FIFO level */
#define SPI_FIFOCFG_EMPTYTX           (1 << 16) /* Bit 16: Empty command for the transmit FIFO) */
#define SPI_FIFOCFG_EMPTYRX           (1 << 17) /* Bit 17: Empty command for the receive FIFO) */

/* FIFO status register */

#define SPI_FIFOSTAT_TXERR            (1 << 0)  /* Bit 0:  TX FIFO error */
#define SPI_FIFOSTAT_RXERR            (1 << 1)  /* Bit 1:  RX FIFO error */
#define SPI_FIFOSTAT_PERINT           (1 << 3)  /* Bit 3:  Peripheral interrupt */
#define SPI_FIFOSTAT_TXEMPTY          (1 << 4)  /* Bit 4:  Transmit FIFO empty */
#define SPI_FIFOSTAT_TXNOTFULL        (1 << 5)  /* Bit 5:  Transmit FIFO not full */
#define SPI_FIFOSTAT_RXNOTEMPTY       (1 << 6)  /* Bit 6:  Receive FIFO not empty */
#define SPI_FIFOSTAT_RXFULL           (1 << 7)  /* Bit 7:  Receive FIFO full */
#define SPI_FIFOSTAT_TXLVL_SHIFT      (8)       /* Bits 8-12:  Transmit FIFO current level */
#define SPI_FIFOSTAT_TXLVL_MASK       (31 << SPI_FIFOSTAT_TXLVL_SHIFT)
#define SPI_FIFOSTAT_RXLVL_SHIFT      (16)      /* Bits 16-20: Receive FIFO current level */
#define SPI_FIFOSTAT_RXLVL_MASK       (31 << SPI_FIFOSTAT_RXLVL_SHIFT)

/* FIFO trigger level settings for interrupt and DMA request */

#define SPI_FIFOTRIG_TXLVLENA         (1 << 0)  /* Bit 0:  Transmit FIFO level trigger enable */
#define SPI_FIFOTRIG_RXLVLENA         (1 << 1)  /* Bit 1:  Receive FIFO level trigger enable */
#define SPI_FIFOTRIG_TXLVL_SHIFT      (8)       /* Bits 8-11: Transmit FIFO level trigger point */
#define SPI_FIFOTRIG_TXLVL_MASK       (15 << SPI_FIFOTRIG_TXLVL_SHIFT)
#  define SPI_FIFOTRIG_TXLVL(n)       ((uint32_t)(n) << SPI_FIFOTRIG_TXLVL_SHIFT)
#  define SPI_FIFOTRIG_TXLVL_EMPTY    (0 << SPI_FIFOTRIG_TXLVL_SHIFT)
#  define SPI_FIFOTRIG_TXLVL_NOTFULL  (7 << SPI_FIFOTRIG_TXLVL_SHIFT)
#define SPI_FIFOTRIG_RXLVL_SHIFT      (16)      /* Bits 16-19: Receive FIFO level trigger point */
#define SPI_FIFOTRIG_RXLVL_MASK       (15 << SPI_FIFOTRIG_RXLVL_SHIFT)
#  define SPI_FIFOTRIG_RXLVL(n)       ((uint32_t)((n)-1) << SPI_FIFOTRIG_RXLVL_SHIFT)
#  define SPI_FIFOTRIG_RXLVL_NOTEMPTY (0 << SPI_FIFOTRIG_RXLVL_SHIFT)
#  define SPI_FIFOTRIG_RXLVL_FULL     (7 << SPI_FIFOTRIG_RXLVL_SHIFT)

/* FIFO interrupt enable set (enable) and read register, FIFO interrupt enable clear (disable)
 * and read register, and FIFO interrupt status register
 */

#define SPI_FIFOINT_TXERR             (1 << 0)  /* Bit 0:  Transmit error interrupt */
#define SPI_FIFOINT_RXERR             (1 << 1)  /* Bit 1:  Receive error interrupt */
#define SPI_FIFOINT_TXLVL             (1 << 2)  /* Bit 2:  Tx FIFO level reached interrupt */
#define SPI_FIFOINT_RXLVL             (1 << 3)  /* Bit 3:  Rx FIFO level reached interrupt */
#define SPI_FIFOINTSTAT_PERINT        (1 << 4)  /* Bit 4:  Peripheral interrupt (status only) */

/* FIFO write data */

#define SPI_FIFOWR_TXDATA_SHIFT       (0)       /* Bits 0-15: Transmit data to the FIFO */
#define SPI_FIFOWR_TXDATA_MASK        (0xffff << SPI_FIFOWR_TXDATA_SHIFT)
#  define SPI_FIFOWR_TXDATA(n)        ((uint32_t)(n) << SPI_FIFOWR_TXDATA_SHIFT)
#define SPI_FIFOWR_TXSSELN_SHIFT      (16)      /* Bits 16-19: Transmit Slave Selects */
#define SPI_FIFOWR_TXSSELN_MASK       (15 << SPI_FIFOWR_TXSSELN_SHIFT)
#  define SPI_FIFOWR_TXSSELN_ALL      (15 << SPI_FIFOWR_TXSSELN_SHIFT)
#  define SPI_FIFOWR_TXSSEL0N         (1 << 16) /* Bit 16: Transmit Slave Select */
#  define SPI_FIFOWR_TXSSEL1N         (1 << 17) /* Bit 17: Transmit Slave Select */
#  define SPI_FIFOWR_TXSSEL2N         (1 << 18) /* Bit 18: Transmit Slave Select */
#  define SPI_FIFOWR_TXSSEL3N         (1 << 19) /* Bit 19: Transmit Slave Select */
#define SPI_FIFOWR_EOT                (1 << 20) /* Bit 20: End of Transfer */
#define SPI_FIFOWR_EOF                (1 << 21) /* Bit 21: End of Frame */
#define SPI_FIFOWR_RXIGNORE           (1 << 22) /* Bit 22: Receive Ignore */
#define SPI_FIFOWR_LEN_SHIFT          (24)      /* Bits 24-27: Data Length */
#define SPI_FIFOWR_LEN_MASK           (15 << SPI_FIFOWR_LEN_SHIFT)
#  define SPI_FIFOWR_LEN(n)           ((uint32_t)((n)-1) << SPI_FIFOWR_LEN_SHIFT)

/* FIFO read data and FIFO data read with no FIFO pop */

#define SPI_FIFORD_RXDATA_SHIFT       (0)       /* Bits 0-15:  Received data from the FIFO */
#define SPI_FIFORD_RXDATA_MASK        (0xffff << SPI_FIFORD_RXDATA_SHIFT)
#define SPI_FIFORD_RXSSELN_SHIFT      (16)      /* Bits 16-19:  Slave Selects for receive */
#define SPI_FIFORD_RXSSELN_MASK       (15 << SPI_FIFORD_RXSSELN_SHIFT)
#  define SPI_FIFORD_RXSSEL0N         (1 << 16) /* Bit 16: Slave Select for receive */
#  define SPI_FIFORD_RXSSEL1N         (1 << 17) /* Bit 17: Slave Select for receive */
#  define SPI_FIFORD_RXSSEL2N         (1 << 18) /* Bit 18: Slave Select for receive */
#  define SPI_FIFORD_RXSSEL3N         (1 << 19) /* Bit 19: Slave Select for receive */
#define SPI_FIFORD_SOT                (1 << 20) /* Bit 20: Start of Transfer flag */

/* SPI module Identification */

#define SPI_ID_APERTURE_SHIFT         (0)       /* Bits 0-7:   Aperture encoded as (aperture size/4K) -1 */
#define SPI_ID_APERTURE_MASK          (0xff << SPI_ID_APERTURE_SHIFT)
#define SPI_ID_MINORREV_SHIFT         (8)       /* Bits 8-11:  Minor revision of module implementation */
#define SPI_ID_MINORREV_MASK          (15 << SPI_ID_MINORREV_SHIFT)
#define SPI_ID_MAJORREV_SHIFT         (12)      /* Bits 12-15: Major revision of module implementation */
#define SPI_ID_MAJORREV_MASK          (15 << SPI_ID_MAJORREV_SHIFT)
#define SPI_ID_ID_SHIFT               (15)      /* Bits 16-31: Unique module identifier for this IP block */
#define SPI_ID_ID_MASK                (0xffff << SPI_ID_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SPI_H */
