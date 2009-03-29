/************************************************************************************
 * arch/z80/src/ez80/ez80f91_i2c.h
 * arch/z80/src/chip/ez80f91_i2c.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_Z80_SRC_EZ80_EZ80F91_I2C_H
#define __ARCH_Z80_SRC_EZ80_EZ80F91_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* I2C Registers  ******************************************************************/

/* Provided in ez80f91.h */

/* I2C Register Bit Definitions  ***************************************************/

/* Slave Address Register (SAR) Bit Definitions */

#define I2C_SAR_GCE      (1 << 0)     /* Bit 0: 1=I2C enabled to recognize the General Call Address */
#define I2C_SAR_SLA_SHIFT 1           /* Bits 1-7: 7-bit address or upper 2 bits in 10-bit mode */
#define I2C_SAR_SLA_MASK (0x7f << 1)

/* Extended Slave Address Register (XSAR) Bit Definitions */
/* Bits 0-7: Least significant 8-bits of 10-bit slave address */

/* Data Byte Register (DR) Bit Definitions */
/* Bits 0-7: I2C byte data */

/* Control (CTL) Register Bit Definitions */

#define I2C_CTL_AAK      (1 << 2) /* Bit 2: 1=Acknowledge */
#define I2C_CTL_IFLG     (1 << 3) /* Bit 3: 1=I2C interrupt flag is set */
#define I2C_CTL_STP      (1 << 4) /* Bit 4: 1=Master mode stop-transmit STOP condition on the bus */
#define I2C_CTL_STA      (1 << 5) /* Bit 5: 1=Master mode start-transmit START condition on the bus */
#define I2C_CTL_ENAB     (1 << 6) /* Bit 6: 1=I2C bus (SCL/SDA) is enabled */
#define I2C_CTL_IEN      (1 << 7) /* Bit 7: 1=I2C interrupt is enabled */

/* Status Register (SR) Bit Definitions */

#define I2C_SR_SHIFT     3        /* Bits 3-7: 5-bit status code */
#define I2C_SR_MASK      (0x1c << I2C_SR_SHIFT)

/* Clock Control Register (CCR) Bit Definitions */

#define I2C_CCR_NSHIFT   0        /* Bits 0-2: I2C clock divider exponent */
#define I2C_CCR_NMASK    (0x07 << I2C_CCR_NSHIFT)
#define I2C_CCR_MSHIFT   3        /* Bits 3-6: I2C clock divider scalar value */
#define I2C_CCR_NMASK    (0x0f << I2C_CCR_MSHIFT)

/* Software Reset Register (SRR) Bit Definitions */
/* Writing any value to this register performs a software reset of the I2C module */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif /* __cplusplus */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_Z80_SRC_EZ80_EZ80F91_I2C_H */
