/****************************************************************************
 * arch/arm/src/lc823450/lc823450_i2c.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
 * Public Function Prototypes
 ****************************************************************************/

struct i2c_master_s *lc823450_i2cbus_initialize(int port);
int lc823450_i2cbus_uninitialize(struct i2c_master_s *dev);
void lc823450_i2cbus_changetimeout(struct i2c_master_s *dev,
                                   uint32_t timeoms);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_I2C_H */
