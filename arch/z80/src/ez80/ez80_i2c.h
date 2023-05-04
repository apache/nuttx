/****************************************************************************
 * arch/z80/src/ez80/ez80_i2c.h
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

#ifndef __ARCH_Z80_SRC_EZ80_EZ80_I2C_H
#define __ARCH_Z80_SRC_EZ80_EZ80_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Registers  ***********************************************************/

/* Provided in ez80f91.h */

/* I2C Register Bit Definitions  ********************************************/

/* Slave Address Register (SAR) Bit Definitions */

#define I2C_SAR_GCE      (1 << 0)     /* Bit 0: 1=I2C enabled to recognize the
                                       *        General Call Address */
#define I2C_SAR_SLA_SHIFT 1           /* Bits 1-7: 7-bit address or upper 2 bits in
                                       *        10-bit mode */
#define I2C_SAR_SLA_MASK (0x7f << 1)

/* Extended Slave Address Register (XSAR) Bit Definitions */

/* Bits 0-7: Least significant 8-bits of 10-bit slave address */

/* Data Byte Register (DR) Bit Definitions */

/* Bits 0-7: I2C byte data */

/* Control (CTL) Register Bit Definitions */

                                  /* Bits 0-1: Reserved */
#define I2C_CTL_AAK      (1 << 2) /* Bit 2: 1=Acknowledge */
#define I2C_CTL_IFLG     (1 << 3) /* Bit 3: 1=I2C interrupt flag is set */
#define I2C_CTL_STP      (1 << 4) /* Bit 4: 1=Master mode stop-transmit STOP
                                   *        condition on the bus */
#define I2C_CTL_STA      (1 << 5) /* Bit 5: 1=Master mode start-transmit
                                   *        START condition on the bus */
#define I2C_CTL_ENAB     (1 << 6) /* Bit 6: 1=I2C bus (SCL/SDA) is enabled */
#define I2C_CTL_IEN      (1 << 7) /* Bit 7: 1=I2C interrupt is enabled */

/* Status Register (SR) Bit Definitions */

#define I2C_SR_SHIFT     3        /* Bits 3-7: 5-bit status code */
#define I2C_SR_MASK      (0x1c << I2C_SR_SHIFT)

#define I2C_SR_BUSERR      0x00 /* Bus error */
#define I2C_SR_MSTART      0x08 /* START condition transmitted */
#define I2C_SR_MREPSTART   0x10 /* Repeated START condition transmitted */
#define I2C_SR_MADDRWRACK  0x18 /* Address and Write bit transmitted, ACK
                                 * received */
#define I2C_SR_MADDRWR     0x20 /* Address and Write bit transmitted, ACK
                                 * not received */
#define I2C_SR_MDATAWRACK  0x28 /* Data byte transmitted in MASTER mode,
                                 * ACK received */
#define I2C_SR_MDATAWR     0x30 /* Data byte transmitted in MASTER mode,
                                 * ACK not received */
#define I2C_SR_ARBLOST1    0x38 /* Arbitration lost in address or data byte */
#define I2C_SR_MADDRRDACK  0x40 /* Address and Read bit transmitted, ACK
                                 * received */
#define I2C_SR_MADDRRD     0x48 /* Address and Read bit transmitted, ACK no
                                 * received */
#define I2C_SR_MDATARDACK  0x50 /* Data byte received in MASTER mode, ACK
                                 * transmitted */
#define I2C_SR_MDATARDNAK  0x58 /* Data byte received in MASTER mode, NACK
                                 * transmitted */
#define I2C_SR_SADDRWRACK  0x60 /* Slave address and Write bit received, ACK
                                 * transmitted */
#define I2C_SR_ARBLOST2    0x68 /* Arbitration lost in address as master,
                                 * slave address and Write bit received, ACK
                                 * transmitted */
#define I2C_SR_SGCARDACK   0x70 /* General Call address received, ACK
                                 * transmitted */
#define I2C_SR_ARBLOST3    0x78 /* Arbitration lost in address as master,
                                 * General Call address received, ACK
                                 * transmitted */
#define I2C_SR_SDATARDACK  0x80 /* Data byte received after slave address
                                 * received, ACK transmitted */
#define I2C_SR_SDATARDNAK  0x88 /* Data byte received after slave address
                                 * received, NACK transmitted */
#define I2C_SR_SDATAGCAACK 0x90 /* Data byte received after General Call
                                 * received, ACK transmitted */
#define I2C_SR_SDATAGCANAK 0x98 /* Data byte received after General Call
                                 * received, NACK transmitted */
#define I2C_SR_SSTOP       0xa0 /* STOP or repeated START condition received
                                 * in SLAVE mode */
#define I2C_SR_SSADDRRDACK 0xa8 /* Slave address and Read bit received, ACK
                                 * transmitted */
#define I2C_SR_ARBLOST4    0xb0 /* Arbitration lost in address as master,
                                 * slave address and Read bit received, ACK
                                 * transmitted */
#define I2C_SR_SDATAWRACK  0xb8 /* Data byte transmitted in SLAVE mode, ACK
                                 * received */
#define I2C_SR_SDATAWR     0xc0 /* Data byte transmitted in SLAVE mode, ACK
                                 * not received */
#define I2C_SR_SLDATAWR    0xc8 /* Last byte transmitted in SLAVE mode, ACK
                                 * received */
#define I2C_SR_MADDR2WRACK 0xd0 /* Second Address byte and Write bit
                                 * transmitted, ACK received */
#define I2C_SR_MADDR2WR    0xd8 /* Second Address byte and Write bit
                                 * transmitted, ACK not received */
#define I2C_SR_NONE        0xf8 /* No relevant status information, IFLG = 0 */

/* Clock Control Register (CCR) Bit Definitions */

#define I2C_CCR_NSHIFT   0        /* Bits 0-2: I2C clock divider exponent */
#define I2C_CCR_NMASK    (0x07 << I2C_CCR_NSHIFT)
#define I2C_CCR_MSHIFT   3        /* Bits 3-6: I2C clock divider scalar value */
#define I2C_CCR_MMASK    (0x0f << I2C_CCR_MSHIFT)

/* Software Reset Register (SRR) Bit Definitions */

/* Writing any value to this register performs a software reset
 * of the I2C module
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif /* __cplusplus */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct i2c_master_s *ez80_i2cbus_initialize(int port);

/****************************************************************************
 * Name: ez80_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by the ez80_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int ez80_i2cbus_uninitialize(FAR struct i2c_master_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_Z80_SRC_EZ80_EZ80_I2C_H */
