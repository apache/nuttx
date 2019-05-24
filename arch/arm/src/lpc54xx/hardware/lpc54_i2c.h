/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_i2c.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_I2C_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_I2C_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

/* Shared I2C registers */

#define LPC54_I2C_CFG_OFFSET        0x0800  /* Configuration for shared functions */
#define LPC54_I2C_STAT_OFFSET       0x0804  /* Status register for shared functions */
#define LPC54_I2C_INTENSET_OFFSET   0x0808  /* Interrupt enable set and read */
#define LPC54_I2C_INTENCLR_OFFSET   0x080c  /* Interrupt enable clear */
#define LPC54_I2C_TIMEOUT_OFFSET    0x0810  /* Time-out value */
#define LPC54_I2C_CLKDIV_OFFSET     0x0814  /* Clock pre-divider for the entire I2C interface */
#define LPC54_I2C_INTSTAT_OFFSET    0x0818  /* Interrupt status register for shared functions */

/* Master function registers */

#define LPC54_I2C_MSTCTL_OFFSET     0x0820  /* Master control */
#define LPC54_I2C_MSTTIME_OFFSET    0x0824  /* Master timing configuration */
#define LPC54_I2C_MSTDAT_OFFSET     0x0828  /* Combined Master receiver and transmitter data */

/* Slave function registers */

#define LPC54_I2C_SLVCTL_OFFSET     0x0840  /* Slave control */
#define LPC54_I2C_SLVDAT_OFFSET     0x0844  /* Combined Slave receiver and transmitter data */
#define LPC54_I2C_SLVADR0_OFFSET    0x0848  /* Slave address 0 */
#define LPC54_I2C_SLVADR1_OFFSET    0x084c  /* Slave address 1 */
#define LPC54_I2C_SLVADR2_OFFSET    0x0850  /* Slave address 2 */
#define LPC54_I2C_SLVADR3_OFFSET    0x0854  /* Slave address 3 */
#define LPC54_I2C_SLVQUAL0_OFFSET   0x0858  /* Slave qualification for address 0 */

/* Monitor function registers */

#define LPC54_I2C_MONRXDAT_OFFSET   0x0880  /* Monitor receiver data */

/* ID register */

#define LPC54_I2C_ID_OFFSET         0x0ffc  /* I2C module Identification */

/* Register addresses *******************************************************************************/

#define LPC54_I2C0_CFG              (LPC54_FLEXCOMM0_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C0_STAT             (LPC54_FLEXCOMM0_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C0_INTENSET         (LPC54_FLEXCOMM0_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C0_INTENCLR         (LPC54_FLEXCOMM0_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C0_TIMEOUT          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C0_CLKDIV           (LPC54_FLEXCOMM0_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C0_INTSTAT          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C0_MSTCTL           (LPC54_FLEXCOMM0_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C0_MSTTIME          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C0_MSTDAT           (LPC54_FLEXCOMM0_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C0_SLVCTL           (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C0_SLVDAT           (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C0_SLVADR0          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C0_SLVADR1          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C0_SLVADR2          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C0_SLVADR3          (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C0_SLVQUAL0         (LPC54_FLEXCOMM0_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C0_MONRXDAT         (LPC54_FLEXCOMM0_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C0_ID               (LPC54_FLEXCOMM0_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C1_CFG              (LPC54_FLEXCOMM1_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C1_STAT             (LPC54_FLEXCOMM1_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C1_INTENSET         (LPC54_FLEXCOMM1_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C1_INTENCLR         (LPC54_FLEXCOMM1_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C1_TIMEOUT          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C1_CLKDIV           (LPC54_FLEXCOMM1_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C1_INTSTAT          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C1_MSTCTL           (LPC54_FLEXCOMM1_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C1_MSTTIME          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C1_MSTDAT           (LPC54_FLEXCOMM1_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C1_SLVCTL           (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C1_SLVDAT           (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C1_SLVADR0          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C1_SLVADR1          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C1_SLVADR2          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C1_SLVADR3          (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C1_SLVQUAL0         (LPC54_FLEXCOMM1_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C1_MONRXDAT         (LPC54_FLEXCOMM1_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C1_ID               (LPC54_FLEXCOMM1_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C2_CFG              (LPC54_FLEXCOMM2_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C2_STAT             (LPC54_FLEXCOMM2_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C2_INTENSET         (LPC54_FLEXCOMM2_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C2_INTENCLR         (LPC54_FLEXCOMM2_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C2_TIMEOUT          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C2_CLKDIV           (LPC54_FLEXCOMM2_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C2_INTSTAT          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C2_MSTCTL           (LPC54_FLEXCOMM2_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C2_MSTTIME          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C2_MSTDAT           (LPC54_FLEXCOMM2_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C2_SLVCTL           (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C2_SLVDAT           (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C2_SLVADR0          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C2_SLVADR1          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C2_SLVADR2          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C2_SLVADR3          (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C2_SLVQUAL0         (LPC54_FLEXCOMM2_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C2_MONRXDAT         (LPC54_FLEXCOMM2_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C2_ID               (LPC54_FLEXCOMM2_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C3_CFG              (LPC54_FLEXCOMM3_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C3_STAT             (LPC54_FLEXCOMM3_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C3_INTENSET         (LPC54_FLEXCOMM3_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C3_INTENCLR         (LPC54_FLEXCOMM3_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C3_TIMEOUT          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C3_CLKDIV           (LPC54_FLEXCOMM3_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C3_INTSTAT          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C3_MSTCTL           (LPC54_FLEXCOMM3_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C3_MSTTIME          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C3_MSTDAT           (LPC54_FLEXCOMM3_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C3_SLVCTL           (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C3_SLVDAT           (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C3_SLVADR0          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C3_SLVADR1          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C3_SLVADR2          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C3_SLVADR3          (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C3_SLVQUAL0         (LPC54_FLEXCOMM3_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C3_MONRXDAT         (LPC54_FLEXCOMM3_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C3_ID               (LPC54_FLEXCOMM3_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C4_CFG              (LPC54_FLEXCOMM4_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C4_STAT             (LPC54_FLEXCOMM4_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C4_INTENSET         (LPC54_FLEXCOMM4_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C4_INTENCLR         (LPC54_FLEXCOMM4_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C4_TIMEOUT          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C4_CLKDIV           (LPC54_FLEXCOMM4_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C4_INTSTAT          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C4_MSTCTL           (LPC54_FLEXCOMM4_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C4_MSTTIME          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C4_MSTDAT           (LPC54_FLEXCOMM4_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C4_SLVCTL           (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C4_SLVDAT           (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C4_SLVADR0          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C4_SLVADR1          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C4_SLVADR2          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C4_SLVADR3          (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C4_SLVQUAL0         (LPC54_FLEXCOMM4_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C4_MONRXDAT         (LPC54_FLEXCOMM4_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C4_ID               (LPC54_FLEXCOMM4_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C5_CFG              (LPC54_FLEXCOMM5_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C5_STAT             (LPC54_FLEXCOMM5_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C5_INTENSET         (LPC54_FLEXCOMM5_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C5_INTENCLR         (LPC54_FLEXCOMM5_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C5_TIMEOUT          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C5_CLKDIV           (LPC54_FLEXCOMM5_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C5_INTSTAT          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C5_MSTCTL           (LPC54_FLEXCOMM5_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C5_MSTTIME          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C5_MSTDAT           (LPC54_FLEXCOMM5_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C5_SLVCTL           (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C5_SLVDAT           (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C5_SLVADR0          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C5_SLVADR1          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C5_SLVADR2          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C5_SLVADR3          (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C5_SLVQUAL0         (LPC54_FLEXCOMM5_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C5_MONRXDAT         (LPC54_FLEXCOMM5_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C5_ID               (LPC54_FLEXCOMM5_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C6_CFG              (LPC54_FLEXCOMM6_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C6_STAT             (LPC54_FLEXCOMM6_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C6_INTENSET         (LPC54_FLEXCOMM6_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C6_INTENCLR         (LPC54_FLEXCOMM6_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C6_TIMEOUT          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C6_CLKDIV           (LPC54_FLEXCOMM6_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C6_INTSTAT          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C6_MSTCTL           (LPC54_FLEXCOMM6_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C6_MSTTIME          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C6_MSTDAT           (LPC54_FLEXCOMM6_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C6_SLVCTL           (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C6_SLVDAT           (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C6_SLVADR0          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C6_SLVADR1          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C6_SLVADR2          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C6_SLVADR3          (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C6_SLVQUAL0         (LPC54_FLEXCOMM6_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C6_MONRXDAT         (LPC54_FLEXCOMM6_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C6_ID               (LPC54_FLEXCOMM6_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C7_CFG              (LPC54_FLEXCOMM7_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C7_STAT             (LPC54_FLEXCOMM7_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C7_INTENSET         (LPC54_FLEXCOMM7_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C7_INTENCLR         (LPC54_FLEXCOMM7_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C7_TIMEOUT          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C7_CLKDIV           (LPC54_FLEXCOMM7_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C7_INTSTAT          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C7_MSTCTL           (LPC54_FLEXCOMM7_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C7_MSTTIME          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C7_MSTDAT           (LPC54_FLEXCOMM7_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C7_SLVCTL           (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C7_SLVDAT           (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C7_SLVADR0          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C7_SLVADR1          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C7_SLVADR2          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C7_SLVADR3          (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C7_SLVQUAL0         (LPC54_FLEXCOMM7_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C7_MONRXDAT         (LPC54_FLEXCOMM7_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C7_ID               (LPC54_FLEXCOMM7_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C8_CFG              (LPC54_FLEXCOMM8_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C8_STAT             (LPC54_FLEXCOMM8_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C8_INTENSET         (LPC54_FLEXCOMM8_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C8_INTENCLR         (LPC54_FLEXCOMM8_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C8_TIMEOUT          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C8_CLKDIV           (LPC54_FLEXCOMM8_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C8_INTSTAT          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C8_MSTCTL           (LPC54_FLEXCOMM8_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C8_MSTTIME          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C8_MSTDAT           (LPC54_FLEXCOMM8_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C8_SLVCTL           (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C8_SLVDAT           (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C8_SLVADR0          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C8_SLVADR1          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C8_SLVADR2          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C8_SLVADR3          (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C8_SLVQUAL0         (LPC54_FLEXCOMM8_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C8_MONRXDAT         (LPC54_FLEXCOMM8_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C8_ID               (LPC54_FLEXCOMM8_BASE + LPC54_I2C_ID_OFFSET

#define LPC54_I2C9_CFG              (LPC54_FLEXCOMM9_BASE + LPC54_I2C_CFG_OFFSET)
#define LPC54_I2C9_STAT             (LPC54_FLEXCOMM9_BASE + LPC54_I2C_STAT_OFFSET)
#define LPC54_I2C9_INTENSET         (LPC54_FLEXCOMM9_BASE + LPC54_I2C_INTENSET_OFFSET)
#define LPC54_I2C9_INTENCLR         (LPC54_FLEXCOMM9_BASE + LPC54_I2C_INTENCLR_OFFSET)
#define LPC54_I2C9_TIMEOUT          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_TIMEOUT_OFFSET)
#define LPC54_I2C9_CLKDIV           (LPC54_FLEXCOMM9_BASE + LPC54_I2C_CLKDIV_OFFSET)
#define LPC54_I2C9_INTSTAT          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_INTSTAT_OFFSET)
#define LPC54_I2C9_MSTCTL           (LPC54_FLEXCOMM9_BASE + LPC54_I2C_MSTCTL_OFFSET)
#define LPC54_I2C9_MSTTIME          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_MSTTIME_OFFSET)
#define LPC54_I2C9_MSTDAT           (LPC54_FLEXCOMM9_BASE + LPC54_I2C_MSTDAT_OFFSET)
#define LPC54_I2C9_SLVCTL           (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVCTL_OFFSET)
#define LPC54_I2C9_SLVDAT           (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVDAT_OFFSET)
#define LPC54_I2C9_SLVADR0          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVADR0_OFFSET)
#define LPC54_I2C9_SLVADR1          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVADR1_OFFSET)
#define LPC54_I2C9_SLVADR2          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVADR2_OFFSET)
#define LPC54_I2C9_SLVADR3          (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVADR3_OFFSET)
#define LPC54_I2C9_SLVQUAL0         (LPC54_FLEXCOMM9_BASE + LPC54_I2C_SLVQUAL0_OFFSET)
#define LPC54_I2C9_MONRXDAT         (LPC54_FLEXCOMM9_BASE + LPC54_I2C_MONRXDAT_OFFSET)
#define LPC54_I2C9_ID               (LPC54_FLEXCOMM9_BASE + LPC54_I2C_ID_OFFSET

/* Register bit definitions *************************************************************************/

/* Configuration for shared functions */

#define I2C_CFG_MSTEN               (1 << 0)  /* Bit 0:  Master enable */
#define I2C_CFG_SLVEN               (1 << 1)  /* Bit 1:  Slave enable */
#define I2C_CFG_MONEN               (1 << 2)  /* Bit 2:  Monitor enable */
#define I2C_CFG_TIMEOUTEN           (1 << 3)  /* Bit 3:  I2C bus time-out enable */
#define I2C_CFG_MONCLKSTR           (1 << 4)  /* Bit 4:  Monitor function clock stretching */
#define I2C_CFG_HSCAPABLE           (1 << 5)  /* Bit 5:  High-speed mode capable enable */

#define I2C_CFG_ALLENABLES          0x1f

/* Status, set and write, and clear register for shared functions */
/* Master function state codes (MSTSTATE) */

#define I2C_MASTER_STATE_IDLE       (0)       /* Idle */
#define I2C_MASTER_STATE_RXAVAIL    (1)       /* Received data is available (Master Receiver mode) */
#define I2C_MASTER_STATE_TXOK       (2)       /* Data can be transmitted (Master Transmitter mode) */
#define I2C_MASTER_STATE_ADDRNAK    (3)       /* Slave NACKed address */
#define I2C_MASTER_STATE_DATANAK    (4)       /* Slave NACKed transmitted data */

/* Slave function state codes (SLVSTATE) */

#define I2C_SLAVE_STATE_ADDR        (0)       /* Address plus R/W received */
#define I2C_SLAVE_STATE_RXAVAIL     (1)       /* Received data is available (Slave Receiver mode) */
#define I2C_SLAVE_STATE_TXOK        (2)       /* Data can be transmitted (Slave Transmitter mode) */

/* Interrupt status, set and read, and clear registers */

#define I2C_INT_MSTPENDING          (1 << 0)  /* Bit 0  Master Pending interrupt */
#define I2C_STAT_MSTSTATE_SHIFT     (1)       /* Bits 1-3: Master State code (status only) */
#define I2C_STAT_MSTSTATE_MASK      (7 << I2C_STAT_MSTSTATE_SHIFT)
#  define I2C_STAT_MSTSTATE_IDLE    (0 << I2C_STAT_MSTSTATE_SHIFT) /* Idle */
#  define I2C_STAT_MSTSTATE_RXAVAIL (1 << I2C_STAT_MSTSTATE_SHIFT) /* Receive ready */
#  define I2C_STAT_MSTSTATE_TXOK    (2 << I2C_STAT_MSTSTATE_SHIFT) /* Transmit ready */
#  define I2C_STAT_MSTSTATE_ADDRNAK (3 << I2C_STAT_MSTSTATE_SHIFT) /* NACK Address */
#  define I2C_STAT_MSTSTATE_DATANAK (4 << I2C_STAT_MSTSTATE_SHIFT) /* NACK Data */
#define I2C_INT_MSTARBLOSS          (1 << 4)  /* Bit 4:  Master Arbitration Loss interrupt */
#define I2C_INT_MSTSTSTPERR         (1 << 6)  /* Bit 6:  Master Start/Stop Error interrupt */
#define I2C_INT_SLVPENDING          (1 << 8)  /* Bit 8:  Slave Pending interrupt */
#define I2C_STAT_SLVSTATE_SHIFT     (9)       /* Bits 9-10: Slave State code (status only) */
#define I2C_STAT_SLVSTATE_MASK      (3 << I2C_STAT_SLVSTATE_SHIFT)
#  define I2C_STAT_SLVSTATE_ADDR    (0 << I2C_STAT_SLVSTATE_SHIFT) /* Slave address */
#  define I2C_STAT_SLVSTATE_RXAVAIL (1 << I2C_STAT_SLVSTATE_SHIFT) /* Slave receive */
#  define I2C_STAT_SLVSTATE_TXOK    (2 << I2C_STAT_SLVSTATE_SHIFT) /* Slave transmit */
#define I2C_INT_SLVNOTSTR           (1 << 11) /* Bit 11: Slave Not Stretching interrupt */
#define I2C_STAT_SLVIDX_SHIFT       (12)      /* Bits 12-13: Slave address match Index (status only) */
#define I2C_STAT_SLVIDX_MASK        (3 << I2C_STAT_SLVIDX_SHIFT)
#  define I2C_STAT_SLVIDX_ADDR0     (0 << I2C_STAT_SLVIDX_SHIFT) /* Slave address 0 was matched */
#  define I2C_STAT_SLVIDX_ADDR1     (1 << I2C_STAT_SLVIDX_SHIFT) /* Slave address 1 was matched */
#  define I2C_STAT_SLVIDX_ADDR2     (2 << I2C_STAT_SLVIDX_SHIFT) /* Slave address 2 was matched */
#  define I2C_STAT_SLVIDX_ADDR3     (3 << I2C_STAT_SLVIDX_SHIFT) /* Slave address 3 was matched */
#define I2C_STAT_SLVSEL             (1 << 14) /* Bit 14: Slave selected flag (Slave only) */
#define I2C_INT_SLVDESEL            (1 << 15) /* Bit 15: Slave Deselect interrupt */
#define I2C_INT_MONRDY              (1 << 16) /* Bit 16: Monitor data Ready interrupt */
#define I2C_INT_MONOV               (1 << 17) /* Bit 17: Monitor Overrun interrupt */
#define I2C_STAT_MONACTIVE          (1 << 18) /* Bit 18: Monitor Active flag (status only) */
#define I2C_INT_MONIDLE             (1 << 19) /* Bit 19: Monitor Idle interrupt */
#define I2C_INT_EVENTTIMEOUT        (1 << 24) /* Bit 24: Event time-out interrupt */
#define I2C_INT_SCLTIMEOUT          (1 << 25) /* Bit 25: SCL time-out interrupt */

#define I2C_INT_MSTPENDING          (1 << 0)  /* Bit 0   Master Pending interrupt */
#define I2C_INT_MSTARBLOSS          (1 << 4)  /* Bit 4:  Master Arbitration Loss interrupt */
#define I2C_INT_MSTSTSTPERR         (1 << 6)  /* Bit 6:  Master Start/Stop Error interrupt */
#define I2C_INT_SLVPENDING          (1 << 8)  /* Bit 8:  Slave Pending interrupt */
#define I2C_INT_SLVNOTSTR           (1 << 11) /* Bit 11: Slave Not Stretching interrupt */
#define I2C_INT_SLVDESEL            (1 << 15) /* Bit 15: Slave Deselect interrupt */
#define I2C_INT_MONRDY              (1 << 16) /* Bit 16: Monitor data Ready interrupt */
#define I2C_INT_MONOV               (1 << 17) /* Bit 17: Monitor Overrun interrupt */
#define I2C_INT_MONIDLE             (1 << 19) /* Bit 19: Monitor Idle interrupt */
#define I2C_INT_EVENTTIMEOUT        (1 << 24) /* Bit 24: Event time-out interrupt */
#define I2C_INT_SCLTIMEOUT          (1 << 25) /* Bit 25: SCL time-out interrupt */

#define I2C_INT_ALL                 0x030b8951

/* Time-out value */

#define I2C_TIMEOUT_SHIFT           (0)       /* Bits 0-15: Time out value
                                               * Bits 0-3 hardwired to 0xff */
#define I2C_TIMEOUT_MASK            (0xffff << I2C_TIMEOUT_SHIFT)
#  define I2C_TIMEOUT(n)            ((uint32_t)((n)-1) << I2C_TIMEOUT_SHIFT)

/* Clock pre-divider for the entire I2C interface */

#define I2C_CLKDIV_SHIFT            (0)       /* Bits 0-15: I2C clock divider */
#define I2C_CLKDIV_MASK             (0xffff << I2C_CLKDIV_SHIFT)
#  define I2C_CLKDIV(n)             ((uint32_t)((n)-1) << I2C_CLKDIV_SHIFT)

/* Master control */

#define I2C_MSTCTL_MSTCONTINUE      (1 << 0)  /* Bit 0:  Master Continue */
#define I2C_MSTCTL_MSTSTART         (1 << 1)  /* Bit 1:  Master Start control */
#define I2C_MSTCTL_MSTSTOP          (1 << 2)  /* Bit 2:  Master Stop control */
#define I2C_MSTCTL_MSTDMA           (1 << 3)  /* Bit 3:  Master DMA enable */

/* Master timing configuration */

#define I2C_MSTTIME_SCLLOW_SHIFT    (0)       /* Bits 0-2  Master SCL Low time */
#define I2C_MSTTIME_SCLLOW_MASK     (7 << I2C_MSTTIME_SCLLOW_SHIFT)
#  define I2C_MSTTIME_SCLLOW(n)     ((uint32_t)((n)-2) << I2C_MSTTIME_SCLLOW_SHIFT)
#define I2C_MSTTIME_SCLHIGH_SHIFT   (4)       /* Bits 4-6  Master SCL High time */
#define I2C_MSTTIME_SCLHIGH_MASK    (7 << I2C_MSTTIME_SCLHIGH_SHIFT)
#  define I2C_MSTTIME_SCLHIGH(n)    ((uint32_t)((n)-2) << I2C_MSTTIME_SCLHIGH_SHIFT)

/* Combined Master receiver and transmitter data */

#define I2C_MSTDAT_SHIFT            (0)       /* Bits 0-7:  Master function data */
#define I2C_MSTDAT_MASK             (0xff << I2C_MSTDAT_SHIFT)
#  define I2C_MSTDAT(n)             ((uint32_t)(n) << I2C_MSTDAT_SHIFT)

/* Slave control */
#define I2C_SLVCTL_
/* Combined Slave receiver and transmitter data */
#define I2C_SLVDAT_
/* Slave address 0 */
#define I2C_SLVADR0_
/* Slave address 1 */
#define I2C_SLVADR1_
/* Slave address 2 */
#define I2C_SLVADR2_
/* Slave address 3 */
#define I2C_SLVADR3_
/* Slave qualification for address 0 */
#define I2C_SLVQUAL0_
/* Monitor receiver data */
#define I2C_MONRXDAT_
/* I2C module Identification */
#define I2C_ID_

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_I2C_H */
