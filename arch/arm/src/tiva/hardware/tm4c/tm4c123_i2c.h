/************************************************************************************
 * arch/arm/src/tiva/hardware/tm4c/tm4c123_i2c.h
 *
 *   Copyright (C) 2009, 2013-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C123_I2C_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C123_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* I2C Register Offsets *************************************************************/

/* I2C Master */

#define TIVA_I2CM_SA_OFFSET            0x0000 /* I2C Master Slave Address */
#define TIVA_I2CM_CS_OFFSET            0x0004 /* I2C Master Control/Status */
#define TIVA_I2CM_DR_OFFSET            0x0008 /* I2C Master Data */
#define TIVA_I2CM_TPR_OFFSET           0x000c /* I2C Master Timer Period */
#define TIVA_I2CM_IMR_OFFSET           0x0010 /* I2C Master Interrupt Mask */
#define TIVA_I2CM_RIS_OFFSET           0x0014 /* I2C Master Raw Interrupt Status */
#define TIVA_I2CM_MIS_OFFSET           0x0018 /* I2C Master Masked Interrupt Status */
#define TIVA_I2CM_ICR_OFFSET           0x001c /* I2C Master Interrupt Clear */
#define TIVA_I2CM_CR_OFFSET            0x0020 /* I2C Master Configuration */
#define TIVA_I2CM_CLKOCNT_OFFSET       0x0024 /* I2C Master Clock Low Timeout Count */
#define TIVA_I2CM_BMON_OFFSET          0x002c /* I2C Master Bus Monitor */

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM_CR2_OFFSET         0x0038 /* I2C Master Configuration 2 */
#endif

/* I2C Slave */

#define TIVA_I2CS_OAR_OFFSET           0x0800 /* I2C Slave Own Address */
#define TIVA_I2CS_CSR_OFFSET           0x0804 /* I2C Slave Control/Status */
#define TIVA_I2CS_DR_OFFSET            0x0808 /* I2C Slave Data */
#define TIVA_I2CS_IMR_OFFSET           0x080c /* I2C Slave Interrupt Mask */
#define TIVA_I2CS_RIS_OFFSET           0x0810 /* I2C Slave Raw Interrupt Status */
#define TIVA_I2CS_MIS_OFFSET           0x0814 /* I2C Slave Masked Interrupt Status */
#define TIVA_I2CS_ICR_OFFSET           0x0818 /* I2C Slave Interrupt Clear */
#define TIVA_I2CS_SOAR2_OFFSET         0x081c /* I2C Slave Own Address 2 */
#define TIVA_I2CS_ACKCTL_OFFSET        0x0820 /* I2C Slave ACK Control */

/* I2C Status and control */

#define TIVA_I2CSC_PP_OFFSET           0x0fc0 /* I2C Peripheral Properties */
#define TIVA_I2CSC_PC_OFFSET           0x0fc4 /* I2C Peripheral Configuration */

/* I2C Register Addresses ***********************************************************/

#if TIVA_NI2C > 0

/* I2C0 Master */

#define TIVA_I2CM0_SA                  (TIVA_I2C0_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM0_CS                  (TIVA_I2C0_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM0_DR                  (TIVA_I2C0_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM0_TPR                 (TIVA_I2C0_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM0_IMR                 (TIVA_I2C0_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM0_RIS                 (TIVA_I2C0_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM0_MIS                 (TIVA_I2C0_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM0_ICR                 (TIVA_I2C0_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM0_CR                  (TIVA_I2C0_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM0_CLKOCNT             (TIVA_I2C0_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM0_BMON                (TIVA_I2C0_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM0_CR2               (TIVA_I2C0_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C0 Slave */

#define TIVA_I2CS0_OAR                 (TIVA_I2C0_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS0_CSR                 (TIVA_I2C0_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS0_DR                  (TIVA_I2C0_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS0_IMR                 (TIVA_I2C0_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS0_RIS                 (TIVA_I2C0_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS0_MIS                 (TIVA_I2C0_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS0_ICR                 (TIVA_I2C0_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS0_SOAR2               (TIVA_I2C0_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS0_ACKCTL              (TIVA_I2C0_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C0 Status and control */

#define TIVA_I2CSC0_PP                 (TIVA_I2C0_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC0_PC                 (TIVA_I2C0_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 0 */

#if TIVA_NI2C > 1

/* I2C1 Master */

#define TIVA_I2CM1_SA                  (TIVA_I2C1_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM1_CS                  (TIVA_I2C1_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM1_DR                  (TIVA_I2C1_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM1_TPR                 (TIVA_I2C1_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM1_IMR                 (TIVA_I2C1_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM1_RIS                 (TIVA_I2C1_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM1_MIS                 (TIVA_I2C1_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM1_ICR                 (TIVA_I2C1_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM1_CR                  (TIVA_I2C1_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM1_CLKOCNT             (TIVA_I2C1_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM1_BMON                (TIVA_I2C1_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM1_CR2               (TIVA_I2C1_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C1 Slave */

#define TIVA_I2CS1_OAR                 (TIVA_I2C1_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS1_CSR                 (TIVA_I2C1_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS1_DR                  (TIVA_I2C1_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS1_IMR                 (TIVA_I2C1_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS1_RIS                 (TIVA_I2C1_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS1_MIS                 (TIVA_I2C1_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS1_ICR                 (TIVA_I2C1_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS1_SOAR2               (TIVA_I2C1_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS1_ACKCTL              (TIVA_I2C1_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C1 Status and control */

#define TIVA_I2CSC1_PP                 (TIVA_I2C1_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC1_PC                 (TIVA_I2C1_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 1 */

#if TIVA_NI2C > 2

/* I2C2 Master */

#define TIVA_I2CM2_SA                  (TIVA_I2C2_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM2_CS                  (TIVA_I2C2_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM2_DR                  (TIVA_I2C2_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM2_TPR                 (TIVA_I2C2_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM2_IMR                 (TIVA_I2C2_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM2_RIS                 (TIVA_I2C2_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM2_MIS                 (TIVA_I2C2_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM2_ICR                 (TIVA_I2C2_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM2_CR                  (TIVA_I2C2_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM2_CLKOCNT             (TIVA_I2C2_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM2_BMON                (TIVA_I2C2_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM2_CR2               (TIVA_I2C2_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C2 Slave */

#define TIVA_I2CS2_OAR                 (TIVA_I2C2_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS2_CSR                 (TIVA_I2C2_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS2_DR                  (TIVA_I2C2_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS2_IMR                 (TIVA_I2C2_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS2_RIS                 (TIVA_I2C2_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS2_MIS                 (TIVA_I2C2_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS2_ICR                 (TIVA_I2C2_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS2_SOAR2               (TIVA_I2C2_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS2_ACKCTL              (TIVA_I2C2_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C2 Status and control */

#define TIVA_I2CSC2_PP                 (TIVA_I2C2_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC2_PC                 (TIVA_I2C2_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 2 */

#if TIVA_NI2C > 3

/* I2C3 Master */

#define TIVA_I2CM3_SA                  (TIVA_I2C3_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM3_CS                  (TIVA_I2C3_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM3_DR                  (TIVA_I2C3_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM3_TPR                 (TIVA_I2C3_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM3_IMR                 (TIVA_I2C3_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM3_RIS                 (TIVA_I2C3_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM3_MIS                 (TIVA_I2C3_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM3_ICR                 (TIVA_I2C3_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM3_CR                  (TIVA_I2C3_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM3_CLKOCNT             (TIVA_I2C3_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM3_BMON                (TIVA_I2C3_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM3_CR2               (TIVA_I2C3_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C3 Slave */

#define TIVA_I2CS3_OAR                 (TIVA_I2C3_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS3_CSR                 (TIVA_I2C3_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS3_DR                  (TIVA_I2C3_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS3_IMR                 (TIVA_I2C3_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS3_RIS                 (TIVA_I2C3_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS3_MIS                 (TIVA_I2C3_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS3_ICR                 (TIVA_I2C3_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS3_SOAR2               (TIVA_I2C3_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS3_ACKCTL              (TIVA_I2C3_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C3 Status and control */

#define TIVA_I2CSC3_PP                 (TIVA_I2C3_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC3_PC                 (TIVA_I2C3_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 3 */

#if TIVA_NI2C > 4

/* I2C4 Master */

#define TIVA_I2CM4_SA                  (TIVA_I2C4_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM4_CS                  (TIVA_I2C4_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM4_DR                  (TIVA_I2C4_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM4_TPR                 (TIVA_I2C4_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM4_IMR                 (TIVA_I2C4_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM4_RIS                 (TIVA_I2C4_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM4_MIS                 (TIVA_I2C4_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM4_ICR                 (TIVA_I2C4_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM4_CR                  (TIVA_I2C4_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM4_CLKOCNT             (TIVA_I2C4_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM4_BMON                (TIVA_I2C4_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM4_CR2               (TIVA_I2C4_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C4 Slave */

#define TIVA_I2CS4_OAR                 (TIVA_I2C4_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS4_CSR                 (TIVA_I2C4_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS4_DR                  (TIVA_I2C4_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS4_IMR                 (TIVA_I2C4_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS4_RIS                 (TIVA_I2C4_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS4_MIS                 (TIVA_I2C4_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS4_ICR                 (TIVA_I2C4_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS4_SOAR2               (TIVA_I2C4_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS4_ACKCTL              (TIVA_I2C4_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C4 Status and control */

#define TIVA_I2CSC4_PP                 (TIVA_I2C4_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC4_PC                 (TIVA_I2C4_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 4 */

#if TIVA_NI2C > 5

/* I2C5 Master */

#define TIVA_I2CM5_SA                  (TIVA_I2C5_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM5_CS                  (TIVA_I2C5_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM5_DR                  (TIVA_I2C5_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM5_TPR                 (TIVA_I2C5_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM5_IMR                 (TIVA_I2C5_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM5_RIS                 (TIVA_I2C5_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM5_MIS                 (TIVA_I2C5_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM5_ICR                 (TIVA_I2C5_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM5_CR                  (TIVA_I2C5_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM5_CLKOCNT             (TIVA_I2C5_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM5_BMON                (TIVA_I2C5_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM5_CR2               (TIVA_I2C5_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C5 Slave */

#define TIVA_I2CS5_OAR                 (TIVA_I2C5_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS5_CSR                 (TIVA_I2C5_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS5_DR                  (TIVA_I2C5_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS5_IMR                 (TIVA_I2C5_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS5_RIS                 (TIVA_I2C5_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS5_MIS                 (TIVA_I2C5_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS5_ICR                 (TIVA_I2C5_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS5_SOAR2               (TIVA_I2C5_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS5_ACKCTL              (TIVA_I2C5_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C Status and control */

#define TIVA_I2CSC5_PP                 (TIVA_I2C5_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC5_PC                 (TIVA_I2C5_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 5 */

#if TIVA_NI2C > 6

/* I2C6 Master */

#define TIVA_I2CM6_SA                  (TIVA_I2C6_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM6_CS                  (TIVA_I2C6_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM6_DR                  (TIVA_I2C6_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM6_TPR                 (TIVA_I2C6_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM6_IMR                 (TIVA_I2C6_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM6_RIS                 (TIVA_I2C6_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM6_MIS                 (TIVA_I2C6_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM6_ICR                 (TIVA_I2C6_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM6_CR                  (TIVA_I2C6_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM6_CLKOCNT             (TIVA_I2C6_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM6_BMON                (TIVA_I2C6_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM6_CR2               (TIVA_I2C6_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C6 Slave */

#define TIVA_I2CS6_OAR                 (TIVA_I2C6_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS6_CSR                 (TIVA_I2C6_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS6_DR                  (TIVA_I2C6_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS6_IMR                 (TIVA_I2C6_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS6_RIS                 (TIVA_I2C6_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS6_MIS                 (TIVA_I2C6_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS6_ICR                 (TIVA_I2C6_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS6_SOAR2               (TIVA_I2C6_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS6_ACKCTL              (TIVA_I2C6_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C Status and control */

#define TIVA_I2CSC6_PP                 (TIVA_I2C6_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC6_PC                 (TIVA_I2C6_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 6 */

#if TIVA_NI2C > 7

/* I2C7 Master */

#define TIVA_I2CM7_SA                  (TIVA_I2C7_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM7_CS                  (TIVA_I2C7_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM7_DR                  (TIVA_I2C7_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM7_TPR                 (TIVA_I2C7_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM7_IMR                 (TIVA_I2C7_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM7_RIS                 (TIVA_I2C7_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM7_MIS                 (TIVA_I2C7_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM7_ICR                 (TIVA_I2C7_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM7_CR                  (TIVA_I2C7_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM7_CLKOCNT             (TIVA_I2C7_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM7_BMON                (TIVA_I2C7_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM7_CR2               (TIVA_I2C7_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C7 Slave */

#define TIVA_I2CS7_OAR                 (TIVA_I2C7_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS7_CSR                 (TIVA_I2C7_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS7_DR                  (TIVA_I2C7_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS7_IMR                 (TIVA_I2C7_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS7_RIS                 (TIVA_I2C7_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS7_MIS                 (TIVA_I2C7_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS7_ICR                 (TIVA_I2C7_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS7_SOAR2               (TIVA_I2C7_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS7_ACKCTL              (TIVA_I2C7_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C Status and control */

#define TIVA_I2CSC7_PP                 (TIVA_I2C7_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC7_PC                 (TIVA_I2C7_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 7 */

#if TIVA_NI2C > 8

/* I2C8 Master */

#define TIVA_I2CM8_SA                  (TIVA_I2C8_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM8_CS                  (TIVA_I2C8_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM8_DR                  (TIVA_I2C8_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM8_TPR                 (TIVA_I2C8_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM8_IMR                 (TIVA_I2C8_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM8_RIS                 (TIVA_I2C8_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM8_MIS                 (TIVA_I2C8_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM8_ICR                 (TIVA_I2C8_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM8_CR                  (TIVA_I2C8_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM8_CLKOCNT             (TIVA_I2C8_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM8_BMON                (TIVA_I2C8_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM8_CR2               (TIVA_I2C8_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C8 Slave */

#define TIVA_I2CS8_OAR                 (TIVA_I2C8_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS8_CSR                 (TIVA_I2C8_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS8_DR                  (TIVA_I2C8_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS8_IMR                 (TIVA_I2C8_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS8_RIS                 (TIVA_I2C8_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS8_MIS                 (TIVA_I2C8_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS8_ICR                 (TIVA_I2C8_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS8_SOAR2               (TIVA_I2C8_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS8_ACKCTL              (TIVA_I2C8_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C Status and control */

#define TIVA_I2CSC8_PP                 (TIVA_I2C8_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC8_PC                 (TIVA_I2C8_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 8 */

#if TIVA_NI2C > 9

/* I2C9 Master */

#define TIVA_I2CM9_SA                  (TIVA_I2C9_BASE + TIVA_I2CM_SA_OFFSET)
#define TIVA_I2CM9_CS                  (TIVA_I2C9_BASE + TIVA_I2CM_CS_OFFSET)
#define TIVA_I2CM9_DR                  (TIVA_I2C9_BASE + TIVA_I2CM_DR_OFFSET)
#define TIVA_I2CM9_TPR                 (TIVA_I2C9_BASE + TIVA_I2CM_TPR_OFFSET)
#define TIVA_I2CM9_IMR                 (TIVA_I2C9_BASE + TIVA_I2CM_IMR_OFFSET)
#define TIVA_I2CM9_RIS                 (TIVA_I2C9_BASE + TIVA_I2CM_RIS_OFFSET)
#define TIVA_I2CM9_MIS                 (TIVA_I2C9_BASE + TIVA_I2CM_MIS_OFFSET)
#define TIVA_I2CM9_ICR                 (TIVA_I2C9_BASE + TIVA_I2CM_ICR_OFFSET)
#define TIVA_I2CM9_CR                  (TIVA_I2C9_BASE + TIVA_I2CM_CR_OFFSET)
#define TIVA_I2CM9_CLKOCNT             (TIVA_I2C9_BASE + TIVA_I2CM_CLKOCNT_OFFSET)
#define TIVA_I2CM9_BMON                (TIVA_I2C9_BASE + TIVA_I2CM_BMON_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_I2CM9_CR2               (TIVA_I2C9_BASE + TIVA_I2CM_CR2_OFFSET)
#endif

/* I2C9 Slave */

#define TIVA_I2CS9_OAR                 (TIVA_I2C9_BASE + TIVA_I2CS_OAR_OFFSET)
#define TIVA_I2CS9_CSR                 (TIVA_I2C9_BASE + TIVA_I2CS_CSR_OFFSET)
#define TIVA_I2CS9_DR                  (TIVA_I2C9_BASE + TIVA_I2CS_DR_OFFSET)
#define TIVA_I2CS9_IMR                 (TIVA_I2C9_BASE + TIVA_I2CS_IMR_OFFSET)
#define TIVA_I2CS9_RIS                 (TIVA_I2C9_BASE + TIVA_I2CS_RIS_OFFSET)
#define TIVA_I2CS9_MIS                 (TIVA_I2C9_BASE + TIVA_I2CS_MIS_OFFSET)
#define TIVA_I2CS9_ICR                 (TIVA_I2C9_BASE + TIVA_I2CS_ICR_OFFSET)
#define TIVA_I2CS9_SOAR2               (TIVA_I2C9_BASE + TIVA_I2CS_SOAR2_OFFSET)
#define TIVA_I2CS9_ACKCTL              (TIVA_I2C9_BASE + TIVA_I2CS_ACKCTL_OFFSET)

/* I2C Status and control */

#define TIVA_I2CSC9_PP                 (TIVA_I2C9_BASE + TIVA_I2CSC_PP_OFFSET)
#define TIVA_I2CSC9_PC                 (TIVA_I2C9_BASE + TIVA_I2CSC_PC_OFFSET)

#endif /* TIVA_NI2C > 9 */

/* I2C_Register Bit Definitions *****************************************************/

/* I2C Master Slave Address (I2CM_SA) */

#define I2CM_SA_RS                     (1 << 0)  /* Bit 0:  Receive/Send */
#define I2CM_SA_SA_SHIFT               1         /* Bits 7-1: I2C Slave Address */
#define I2CM_SA_SA_MASK                (0x7f << I2CM_SA_SA_SHIFT)

/* I2C Master Control/Status (I2CM_CS) */

#define I2CM_CS_BUSY                   (1 << 0)  /* Bit 0:  I2C Busy (read) */
#define I2CM_CS_ERROR                  (1 << 1)  /* Bit 1:  Error in last bus operation (read) */
#define I2CM_CS_ADRACK                 (1 << 2)  /* Bit 2:  Acknowledge Address (read) */
#define I2CM_CS_DATACK                 (1 << 3)  /* Bit 3:  Acknowledge Data (read) */
#define I2CM_CS_ARBLST                 (1 << 4)  /* Bit 4:  Arbitration Lost (read) */
#define I2CM_CS_IDLE                   (1 << 5)  /* Bit 5:  I2C Idle (read) */
#define I2CM_CS_BUSBSY                 (1 << 6)  /* Bit 6:  Bus Busy (read) */
#define I2CM_CS_CLKTO                  (1 << 7)  /* Bit 7:  Clock Timeout Error (read) */

#define I2CM_CS_RUN                    (1 << 0)  /* Bit 0:  I2C Master Enable (write) */
#define I2CM_CS_START                  (1 << 1)  /* Bit 1:  Generate START (write) */
#define I2CM_CS_STOP                   (1 << 2)  /* Bit 2:  Generate STOP (write) */
#define I2CM_CS_ACK                    (1 << 3)  /* Bit 3:  Data Acknowledge Enable (write) */
#define I2CM_CS_HS                     (1 << 4)  /* Bit 4:  High-Speed Enable (write) */

/* I2C Master Data (I2CM_DR) */

#define I2CM_DR_SHIFT                  (0)       /* Bits 7-0: Data transferred */
#define I2CM_DR_MASK                   (0xff << I2CM_DR_SHIFT)

/* I2C Master Timer Period (I2CM_TPR) */

#define I2CM_TPR_SHIFT                 (0)      /* Bits 6-0: SCL Clock Period */
#define I2CM_TPR_MASK                  (0x7f << I2CM_TPR_SHIFT)
#define I2CM_TPR_HS                    (1 << 7) /* Bit 7:  High-Speed Enable (write) */

/* I2C Master Interrupt Mask (I2CM_IMR) */

#define I2CM_IMR_MIM                   (1 << 0)  /* Bit 0:  Master Interrupt Mask */
#define I2CM_IMR_CLKIM                 (1 << 1)  /* Bit 1:  Clock Timeout Interrupt Mask */

/* I2C Master Raw Interrupt Status (I2CM_RIS) */

#define I2CM_RIS_MRIS                  (1 << 0)  /* Bit 0:  Master Raw Interrupt Status */
#define I2CM_RIS_CLKRIS                (1 << 1)  /* Bit 1:  Clock Timeout Raw Interrupt Status */

/* I2C Master Masked Interrupt Status (I2CM_MIS) */

#define I2CM_MIS_MMIS                  (1 << 0)  /* Bit 0:  Maseter Masked Interrupt Status */
#define I2CM_MIS_CLKMIS                (1 << 1)  /* Bit 1:  Clock Timeout Masked Interrupt Status */

/* I2C Master Masked Interrupt Status (I2CM_ICR) */

#define I2CM_ICR_MIC                   (1 << 0)  /* Bit 0:  Master Masked Interrupt Clear */
#define I2CM_ICR_CLKC                  (1 << 1)  /* Bit 1:  Clock Timeout Interrupt Clear */

/* I2C Master Configuration (I2CM_CR) */

#define I2CM_CR_LPBK                   (1 << 0)  /* Bit 0:: I2C Loopback */
#define I2CM_CR_MFE                    (1 << 4)  /* Bit 4:  I2C Master Function Enable */
#define I2CM_CR_SFE                    (1 << 5)  /* Bit 5:  I2C Slave Function Enable */

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define I2CM_CR_GFE                  (1 << 6)  /* Bit 6:  I2C Glitch Filter Enable */
#endif

/* I2C Master Clock Low Timeout Count */

#define I2CM_CLKOCNT_CNTL_SHIFT        (0)       /* Bits 7-0: I2C Master Count */
#define I2CM_CLKOCNT_CNTL_MASK         (0xff << I2CM_CLKOCNT_CNTL_SHIFT)

/* I2C Master Configuration */

#define I2CM_BMON_SCL                  (1 << 0)  /* Bit 0:  II2C SCL Status */
#define I2CM_BMON_SCA                  (1 << 1)  /* Bit 1:  II2C SDA Status */

/* I2C Master Configuration 2 */

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define I2CM_CR2_GFPW_SHIFT          (4)       /* I2C Glitch Filter Pulse Width */
#  define I2CM_CR2_GFPW_MASK           (7 << I2CM_CR2_GFPW_SHIFT)
#    define I2CM_CR2_GFPW_BYPASS       (0 << I2CM_CR2_GFPW_SHIFT) /* Bypass */
#    define I2CM_CR2_GFPW_1CLK         (1 << I2CM_CR2_GFPW_SHIFT) /* 1 clock */
#    define I2CM_CR2_GFPW_2CLKS        (2 << I2CM_CR2_GFPW_SHIFT) /* 2 clocks */
#    define I2CM_CR2_GFPW_3CLKS        (3 << I2CM_CR2_GFPW_SHIFT) /* 3 clocks */
#    define I2CM_CR2_GFPW_4CLKS        (4 << I2CM_CR2_GFPW_SHIFT) /* 4 clocks */
#    define I2CM_CR2_GFPW_8CLKS        (5 << I2CM_CR2_GFPW_SHIFT) /* 8 clocks */
#    define I2CM_CR2_GFPW_16CLKS       (6 << I2CM_CR2_GFPW_SHIFT) /* 16 clocks */
#    define I2CM_CR2_GFPW_31CLKS       (7 << I2CM_CR2_GFPW_SHIFT) /* 31 clocks */
#endif

/* I2C Slave Own Address (I2CS_OAR) */

#define I2CS_OAR_MASK                  0x7f      /* Bits 6-0: I2C Slave Own Address */

/* I2C Slave Control/Status (I2CS_CSR) */

#define I2CS_CSR_RREQ                  (1 << 0)  /* Bit 0:  Receive Request (read) */
#define I2CS_CSR_TREQ                  (1 << 1)  /* Bit 1:  Transmit Request (read) */
#define I2CS_CSR_FBR                   (1 << 2)  /* Bit 2:  First Byte Received (read) */
#define I2CS_CSR_OAR2SEL               (1 << 3)  /* Bit 3:  OAR2 Address Matched (read) */

#define I2CS_CSR_DA                    (1 << 0)  /* Bit 0:  Device Active (write) */

/* I2C Slave Data (I2CS_DR) */

#define I2CS_DR_SHIFT                  (0)       /* Bits 7-0: Data for Transfer */
#define I2CS_DR_MASK                   (0xff << I2CS_DR_SHIFT)

/* I2C Slave Interrupt Mask (I2CS_IMR) */

#define I2CS_IMR_DATAIM                (1 << 0)  /* Bit 0:  Data Interrupt Mask */
#define I2CS_IMR_STARTIM               (1 << 1)  /* Bit 1:  Start Condition Interrupt Mask */
#define I2CS_IMR_STOPIM                (1 << 2)  /* Bit 2:  Stop Condition Interrupt Mask */

/* I2C Slave Raw Interrupt Status (I2CS_RIS) */

#define I2CS_RIS_DATARIS               (1 << 0)  /* Bit 0:  Data Raw Interrupt Status */
#define I2CS_RIS_STARTRIS              (1 << 1)  /* Bit 1:  Start Condition Raw Interrupt Status */
#define I2CS_RIS_STOPRIS               (1 << 2)  /* Bit 2:  Stop Condition Raw Interrupt Status */

/* I2C Slave Masked Interrupt Status (I2CS_MIS) */

#define I2CS_MIS_DATAMIS               (1 << 0)  /* Bit 0:  Data Masked Interrupt Status */
#define I2CS_MIS_STARTMIS              (1 << 1)  /* Bit 1:  Start Condition Masked Interrupt Status */
#define I2CS_MIS_STOPMIS               (1 << 2)  /* Bit 2:  Stop Condition Masked Interrupt Status */

/* I2C Slave Interrupt Clear (I2CS_ICR) */

#define I2CS_ICR_DATAIC                (1 << 0)  /* Bit 0:  Data Interrupt Clear */
#define I2CS_ICR_STARTIC               (1 << 1)  /* Bit 1:  Start Condition Interrupt Clear */
#define I2CS_ICR_STOPIC                (1 << 2)  /* Bit 2:  Stop Condition Interrupt Clear */

/* I2C Slave Own Address 2 */

#define I2CS_SOAR2_SHIFT               (0)       /* Bits 0-6: I2C Slave Own Address 2 */
#define I2CS_SOAR2_MASK                (0x7f << I2CS_SOAR2_SHIFT)
#define I2CS_SOAR2_OAR2EN              (1 << 7)  /* Bit 7:  I2C Slave Own Address 2 Enable */

/* I2C Slave ACK Control */

#define I2CS_ACKCTL_ACKOEN             (1 << 0)  /* Bit 0:  I2C Slave ACK Override Enable */
#define I2CS_ACKCTL_ACKOVAL            (1 << 1)  /* Bit 1:  I2C Slave ACK Override Value */

/* I2C Peripheral Properties */

#define I2CSC_PP_HS                    (1 << 0)  /* Bit 0:  High-Speed Capable */

/* I2C Peripheral Configuration */

#define I2CSC_PC_HS                    (1 << 0)  /* Bit 0:  High-Speed Capable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C123_I2C_H */
