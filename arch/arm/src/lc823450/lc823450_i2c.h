/****************************************************************************
 * arch/arm/src/lc823450/lc823450_i2c.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_I2C_H
#define __ARCH_ARM_SRC_LC823450_LC823450_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define LC823450_I2C0_REGBASE   0x40089000
#define LC823450_I2C1_REGBASE   0x4008A000

#define I2CCTL      (0x0)
#define I2CSTR      (0x4)
#define I2CTXD      (0x8)
#define I2CRXD      (0xC)
#define I2CCKS      (0x10)

/* Register Bitfield Definitions ********************************************/

/* Control register */

#define I2C_CTL_SDAD        (1 << 0)  /* Bit 0: SDA data output */
#define I2C_CTL_SCLD        (1 << 1)  /* Bit 1: SCL data output */
#define I2C_CTL_BC          (1 << 2)  /* Bit 2: Control I2C Bus forcely */
#define I2C_CTL_SCLR        (1 << 3)  /* Bit 3: Clear internal state */
#define I2C_CTL_IREQEN      (1 << 4)  /* Bit 4: Interrupt enable */
#define I2C_CTL_ACK         (1 << 5)  /* Bit 5: ACK bit output enable */
#define I2C_CTL_ST          (1 << 6)  /* Bit 6: Generate Start/Stop condition */
#define I2C_CTL_TRX         (1 << 7)  /* Bit 7: TX or RX enable */
#define I2C_CTL_SRST        (1 << 8)  /* Bit 8: Reset all registers */
#define I2C_CTL_BTRIG       (1 << 12) /* Bit 12: Trigger starting byte transfer/receive */
#define I2C_CTL_FMODE       (1 << 15) /* Bit 15: Fast or Standard mode enable */

/* Status register */

#define I2C_STR_SDAB        (1 << 0)  /* Bit 0: Monitor SDA pin */
#define I2C_STR_SCLB        (1 << 1)  /* Bit 1: Monitor SCL pin */
#define I2C_STR_ACKD        (1 << 5)  /* Bit 5: ACK detection */
#define I2C_STR_IREQ        (1 << 6)  /* Bit 6: Interrupt status */
#define I2C_STR_BBSY        (1 << 7)  /* Bit 7: Bus busy */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct i2c_master_s *lc823450_i2cbus_initialize(int port);
int lc823450_i2cbus_uninitialize(FAR struct i2c_master_s *dev);
void lc823450_i2cbus_changetimeout(FAR struct i2c_master_s *dev, uint32_t timeoms);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_I2C_H */
