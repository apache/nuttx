/********************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_i2c.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_I2C_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_I2C_H

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* I2C register offsets *********************************************************************/

#define TIVA_I2C_SOAR_OFFSET                    0x0000  /* Slave Own Address */
#define TIVA_I2C_SSTAT_OFFSET                   0x0004  /* Slave Status */
#define TIVA_I2C_SCTL_OFFSET                    0x0004  /* Slave Control */
#define TIVA_I2C_SDR_OFFSET                     0x0008  /* Slave Data */
#define TIVA_I2C_SIMR_OFFSET                    0x000c  /* Slave Interrupt Mask */
#define TIVA_I2C_SRIS_OFFSET                    0x0010  /* Slave Raw Interrupt Status */
#define TIVA_I2C_SMIS_OFFSET                    0x0014  /* Slave Masked Interrupt Status */
#define TIVA_I2C_SICR_OFFSET                    0x0018  /* Slave Interrupt Clear */

#define TIVA_I2C_MSA_OFFSET                     0x0800  /* Master Slave Address */
#define TIVA_I2C_MSTAT_OFFSET                   0x0804  /* Master Status */
#define TIVA_I2C_MCTRL_OFFSET                   0x0804  /* Master Control */
#define TIVA_I2C_MDR_OFFSET                     0x0808  /* Master Data */
#define TIVA_I2C_MTPR_OFFSET                    0x080c  /* I2C Master Timer Period */
#define TIVA_I2C_MIMR_OFFSET                    0x0810  /* Master Interrupt Mask */
#define TIVA_I2C_MRIS_OFFSET                    0x0814  /* Master Raw Interrupt Status */
#define TIVA_I2C_MMIS_OFFSET                    0x0818  /* Master Masked Interrupt Status */
#define TIVA_I2C_MICR_OFFSET                    0x081c  /* Master Interrupt Clear */
#define TIVA_I2C_MCR_OFFSET                     0x0820  /* Master Configuration */

/* I2C register addresses *******************************************************************/

#define TIVA_I2C0_SOAR                          (TIVA_I2C0_BASE + TIVA_I2C_SOAR_OFFSET)
#define TIVA_I2C0_SSTAT                         (TIVA_I2C0_BASE + TIVA_I2C_SSTAT_OFFSET)
#define TIVA_I2C0_SCTL                          (TIVA_I2C0_BASE + TIVA_I2C_SCTL_OFFSET)
#define TIVA_I2C0_SDR                           (TIVA_I2C0_BASE + TIVA_I2C_SDR_OFFSET)
#define TIVA_I2C0_SIMR                          (TIVA_I2C0_BASE + TIVA_I2C_SIMR_OFFSET)
#define TIVA_I2C0_SRIS                          (TIVA_I2C0_BASE + TIVA_I2C_SRIS_OFFSET)
#define TIVA_I2C0_SMIS                          (TIVA_I2C0_BASE + TIVA_I2C_SMIS_OFFSET)
#define TIVA_I2C0_SICR                          (TIVA_I2C0_BASE + TIVA_I2C_SICR_OFFSET)

#define TIVA_I2C0_MSA                           (TIVA_I2C0_BASE + TIVA_I2C_MSA_OFFSET)
#define TIVA_I2C0_MSTAT                         (TIVA_I2C0_BASE + TIVA_I2C_MSTAT_OFFSET)
#define TIVA_I2C0_MCTRL                         (TIVA_I2C0_BASE + TIVA_I2C_MCTRL_OFFSET)
#define TIVA_I2C0_MDR                           (TIVA_I2C0_BASE + TIVA_I2C_MDR_OFFSET)
#define TIVA_I2C0_MTPR                          (TIVA_I2C0_BASE + TIVA_I2C_MTPR_OFFSET)
#define TIVA_I2C0_MIMR                          (TIVA_I2C0_BASE + TIVA_I2C_MIMR_OFFSET)
#define TIVA_I2C0_MRIS                          (TIVA_I2C0_BASE + TIVA_I2C_MRIS_OFFSET)
#define TIVA_I2C0_MMIS                          (TIVA_I2C0_BASE + TIVA_I2C_MMIS_OFFSET)
#define TIVA_I2C0_MICR                          (TIVA_I2C0_BASE + TIVA_I2C_MICR_OFFSET)
#define TIVA_I2C0_MCR                           (TIVA_I2C0_BASE + TIVA_I2C_MCR_OFFSET)

/* I2C bitfield definitions *****************************************************************/

/* Slave Own Address */

#define I2C_SOAR_OAR_SHIFT                      (0)       /* Bits 0-6: 2C slave own address */
#define I2C_SOAR_OAR_MASK                       (0x7f << I2C_SOAR_OAR_SHIFT)
#  define I2C_SOAR_OAR(n)                       ((uint32_t)(n) << I2C_SOAR_OAR_SHIFT)

/* Slave Status */

#define I2C_SSTAT_RREQ                          (1 << 0)  /* Bit 0:  Receive request */
#define I2C_SSTAT_TREQ                          (1 << 1)  /* Bit 1:  Transmit request */
#define I2C_SSTAT_FBR                           (1 << 2)  /* Bit 2:  First byte received */

/* Slave Control */

#define I2C_SCTL_DA                             (1 << 0)  /* Bit 0:  Device active */

/* Slave Data */

#define I2C_SDR_DATA_SHIFT                      (0)       /* Bits 0-7: Data for transfer */
#define I2C_SDR_DATA_MASK                       (0xff << I2C_SDR_DATA_SHIFT)
#  define I2C_SDR_DATA(n)                       ((uint32_t)(n) << I2C_SDR_DATA_SHIFT)

/* Slave Interrupt Mask */

#define I2C_SIMR_DATAIM                         (1 << 0)  /* Bit 0:  Data interrupt mask */
#define I2C_SIMR_STARTIM                        (1 << 1)  /* Bit 1:  Start condition interrupt mask */
#define I2C_SIMR_STOPIM                         (1 << 2)  /* Bit 2:  Stop condition interrupt mask */

/* Slave Raw Interrupt Status */

#define I2C_SRIS_DATARIS                        (1 << 0)  /* Bit 0:  Data raw interrupt status */
#define I2C_SRIS_STARTRIS                       (1 << 1)  /* Bit 1:  Start condition raw interrupt status */
#define I2C_SRIS_STOPRIS                        (1 << 2)  /* Bit 2:  Stop condition raw interrupt status */

/* Slave Masked Interrupt Status */

#define I2C_SMIS_DATAMIS                        (1 << 0)  /* Bit 0:  Data masked interrupt status */
#define I2C_SMIS_STARTMIS                       (1 << 1)  /* Bit 1:  Start condition masked interrupt status */
#define I2C_SMIS_STOPMIS                        (1 << 2)  /* Bit 2:  Stop condition masked interrupt status */

/* Slave Interrupt Clear */

#define I2C_SICR_DATAIC                         (1 << 0)  /* Bit 0:  Data interrupt clear */
#define I2C_SICR_STARTIC                        (1 << 1)  /* Bit 1:  Start condition interrupt clear */
#define I2C_SICR_STOPIC                         (1 << 2)  /* Bit 2:  top condition interrupt clear */

/* Master Slave Address */

#define I2C_MSA_RS                              (1 << 0)  /* Bit 0:  Receive or Send */
#  define I2C_MSA_RS_TX                         0
#  define I2C_MSA_RS_RX                         I2C_MSA_RS
#define I2C_MSA_SA_SHIFT                        (1)       /* Bits 1-7: I2C master slave address */
#define I2C_MSA_SA_MASK                         (0x7f << I2C_MSA_SA_SHIFT)
#  define I2C_MSA_SA(n)                         ((uint32_t)(n) << I2C_MSA_SA_SHIFT)

/* Master Status */

#define I2C_MSTAT_BUSY                          (1 << 0)  /* Bit 0:  I2C busy */
#define I2C_MSTAT_ERR                           (1 << 1)  /* Bit 1:  Error */
#define I2C_MSTAT_ADRACK_N                      (1 << 2)  /* Bit 2:  Address Was Not Acknowledged */
#define I2C_MSTAT_DATACK_N                      (1 << 3)  /* Bit 3:  Data Was Not Acknowledged */
#define I2C_MSTAT_ARBLST                        (1 << 4)  /* Bit 4:  Arbitration lost */
#define I2C_MSTAT_IDLE                          (1 << 5)  /* Bit 5:  I2C idle */
#define I2C_MSTAT_BUSBSY                        (1 << 6)  /* Bit 6:  Bus busy */

/* Master Control */

#define I2C_MCTRL_RUN                           (1 << 0)  /* Bit 0:  Enable master transfer */
#define I2C_MCTRL_START                         (1 << 1)  /* Bit 1:  Generate [repeated] START */
#define I2C_MCTRL_STOP                          (1 << 2)  /* Bit 2:  STOP at end of data cycle */
#define I2C_MCTRL_ACK                           (1 << 3)  /* Bit 3:  Data acknowledge enable */

/* Master Data */

#define I2C_MDR_DATA_SHIFT                      (0)       /* Bits 0-7:  */
#define I2C_MDR_DATA_MASK                       (0xff << I2C_MDR_DATA_SHIFT)
#  define I2C_MDR_DATA(n)                       ((uint32_t)(n) << I2C_MDR_DATA_SHIFT)

/* I2C Master Timer Period
 *
 *   SCL_PRD = 2 * (1 + TPR) *( SCL_LP + SCL_HP) * CLK_PRD
 *
 * where:
 *   SCL_PRD - The SCL line period (I2C clock).
 *   TPR     - The timer period register value (range of 1 to 127)
 *   SCL_LP  - The SCL low period (fixed at 6).
 *   SCL_HP  - The SCL high period (fixed at 4).
 *   CLK_PRD - The system clock period in ns.
 */

#define I2C_MTPR_TPR_SHIFT                      (0)       /* Bits 0-6:  SCL clock period */
#define I2C_MTPR_TPR_MASK                       (0x7f << I2C_MTPR_TPR_SHIFT)
#  define I2C_MTPR_TPR(n)                       ((uint32_t)(n) << I2C_MTPR_TPR_SHIFT)
#define I2C_MTPR_TPR_7                          (1 << 7)  /* Bit 7:  Lock TPR */

/* Master Interrupt Mask */

#define I2C_MIMR_IM                             (1 << 0)  /* Bit 0:  Interrupt mask */

/* Master Raw Interrupt Status */

#define I2C_MRIS_RIS                            (1 << 0)  /* Bit 0:  Raw interrupt status */

/* Master Masked Interrupt Status */

#define I2C_MMIS_MIS                            (1 << 0)  /* Bit 0:  Masked interrupt status */

/* Master Interrupt Clear */

#define I2C_MICR_IC                             (1 << 0)  /* Bit 0:  Interrupt clear */

/* Master Configuration */

#define I2C_MCR_LPBK                            (1 << 0)  /* Bit 0:  I2C loopback */
#define I2C_MCR_MFE                             (1 << 4)  /* Bit 4:  I2C master function enable */
#define I2C_MCR_SFE                             (1 << 5)  /* Bit 5:  I2C slave function enable */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_I2C_H */
