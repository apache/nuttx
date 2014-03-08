/************************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_i2c.h
 *
 *   Copyright (C) 2013 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *
 * Derived arch/arm/src/lpc17xx/lpc17_i2c.h
 *
 *   Copyright (C) 2010, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC2378_LPC23XX_I2C_H
#define __ARCH_ARM_SRC_LPC2378_LPC23XX_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* I2C Pin Configuration ************************************************************/

#define I2C0_PCLKSEL_MASK    (0x03 << 14)
#define I2C1_PCLKSEL_MASK    (0x03 << 6)
#define I2C2_PCLKSEL_MASK    (0x03 << 20)
#define I2C0_PCLKSEL         (0x01 << 14)
#define I2C1_PCLKSEL         (0x01 << 6)
#define I2C2_PCLKSEL         (0x01 << 20)

#define I2C0_PINSEL_MASK     (0x0f << 22)   /* P0.27 P0.28  PINSEL1 */
#define I2C1_PINSEL_MASK     (0x0f)         /* P0.0  P0.1   PINSEL0 */
#define I2C2_PINSEL_MASK     (0x0f << 22)   /* P0.10 P0.11  PINSEL0 */
#define I2C0_PINSEL          (0x05 << 22)
#define I2C1_PINSEL          (0x0f)
#define I2C2_PINSEL          (0x0a << 22)

/* I2C Register addresses ***********************************************************/

#define I2C0_CONSET          (I2C0_BASE_ADDR+I2C_CONSET_OFFSET)
#define I2C0_STAT            (I2C0_BASE_ADDR+I2C_STAT_OFFSET)
#define I2C0_DAT             (I2C0_BASE_ADDR+I2C_DAT_OFFSET)
#define I2C0_ADR0            (I2C0_BASE_ADDR+I2C_ADR0_OFFSET)
#define I2C0_SCLH            (I2C0_BASE_ADDR+I2C_SCLH_OFFSET)
#define I2C0_SCLL            (I2C0_BASE_ADDR+I2C_SCLL_OFFSET)
#define I2C0_CONCLR          (I2C0_BASE_ADDR+I2C_CONCLR_OFFSET)

#define I2C1_CONSET          (I2C1_BASE_ADDR+I2C_CONSET_OFFSET)
#define I2C1_STAT            (I2C1_BASE_ADDR+I2C_STAT_OFFSET)
#define I2C1_DAT             (I2C1_BASE_ADDR+I2C_DAT_OFFSET)
#define I2C1_ADR0            (I2C1_BASE_ADDR+I2C_ADR0_OFFSET)
#define I2C1_SCLH            (I2C1_BASE_ADDR+I2C_SCLH_OFFSET)
#define I2C1_SCLL            (I2C1_BASE_ADDR+I2C_SCLL_OFFSET)
#define I2C1_CONCLR          (I2C1_BASE_ADDR+I2C_CONCLR_OFFSET)

#define I2C2_CONSET          (I2C2_BASE_ADDR+I2C_CONSET_OFFSET)
#define I2C2_STAT            (I2C2_BASE_ADDR+I2C_STAT_OFFSET)
#define I2C2_DAT             (I2C2_BASE_ADDR+I2C_DAT_OFFSET)
#define I2C2_ADR0            (I2C2_BASE_ADDR+I2C_ADR0_OFFSET)
#define I2C2_SCLH            (I2C2_BASE_ADDR+I2C_SCLH_OFFSET)
#define I2C2_SCLL            (I2C2_BASE_ADDR+I2C_SCLL_OFFSET)
#define I2C2_CONCLR          (I2C2_BASE_ADDR+I2C_CONCLR_OFFSET)

/* I2C Register bit definitions *****************************************************/
/* I2C Control Set Register */

                                       /* Bits 0-1: Reserved */
#define I2C_CONSET_AA        (1 << 2)  /* Bit 2:  Assert acknowledge flag */
#define I2C_CONSET_SI        (1 << 3)  /* Bit 3:  I2C interrupt flag */
#define I2C_CONSET_STO       (1 << 4)  /* Bit 4:  STOP flag */
#define I2C_CONSET_STA       (1 << 5)  /* Bit 5:  START flag */
#define I2C_CONSET_I2EN      (1 << 6)  /* Bit 6:  I2C interface enable */
                                       /* Bits 7-31: Reserved */
/* I2C Control Clear Register */
                                       /* Bits 0-1: Reserved */
#define I2C_CONCLR_AAC       (1 << 2)  /* Bit 2:  Assert acknowledge Clear bit */
#define I2C_CONCLR_SIC       (1 << 3)  /* Bit 3:  I2C interrupt Clear bit */
                                       /* Bit 4:  Reserved */
#define I2C_CONCLR_STAC      (1 << 5)  /* Bit 5:  START flag Clear bit */
#define I2C_CONCLRT_I2ENC    (1 << 6)  /* Bit 6:  I2C interface Disable bit */
                                       /* Bits 7-31: Reserved */
/* I2C Status Register
 *
 *   See tables 399-402 in the "LPC17xx User Manual" (UM10360), Rev. 01, 4 January
 *   2010, NXP for definitions of status codes.
 */

#define I2C_STAT_MASK        (0xff)    /* Bits 0-7: I2C interface status
                                        *           Bits 0-1 always zero */
                                       /* Bits 8-31: Reserved */
/* I2C Data Register */

#define I2C_DAT_MASK         (0xff)    /* Bits 0-7: I2C data */
                                       /* Bits 8-31: Reserved */

#define I2C_ADR_GC           (1 << 0)  /* Bit 0: GC General Call enable bit */
#define I2C_ADR_ADDR_SHIFT   (1)       /* Bits 1-7: I2C slave address */
#define I2C_ADR_ADDR_MASK    (0x7f << I2C_ADR_ADDR_SHIFT)
                                       /* Bits 8-31: Reserved */
/* I2C Slave address mask registers:
 *
 *   I2C Slave address mask register 0
 *   I2C Slave address mask register 1
 *   I2C Slave address mask register 2
 *   I2C Slave address mask register 3
 */
                                       /* Bit 0: Reserved */
#define I2C_MASK_SHIFT       (1)       /* Bits 1-7: I2C mask bits */
#define I2C_MASK_MASK        (0x7f << I2C_ADR_ADDR_SHIFT)
                                       /* Bits 8-31: Reserved */
/* SCH Duty Cycle Register High Half Word */

#define I2C_SCLH_MASK        (0xffff)  /* Bit 0-15: Count for SCL HIGH time period selection */
                                       /* Bits 16-31: Reserved */
/* SCL Duty Cycle Register Low Half Word */

#define I2C_SCLL_MASK        (0xffff)  /* Bit 0-15: Count for SCL LOW time period selection */
                                       /* Bits 16-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC2378_LPC23XX_I2C_H */
