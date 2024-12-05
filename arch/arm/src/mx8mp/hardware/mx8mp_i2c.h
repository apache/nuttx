/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_I2C_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Register Offsets *****************************************************/

#define IADR_OFFSET     0x0000
#define IFDR_OFFSET     0x0004
#define I2CR_OFFSET     0x0008
#define I2SR_OFFSET     0x000c
#define I2DR_OFFSET     0x0010

/* I2C Register Bit Definitions *********************************************/

#define IADR_SHIFT      1
#define IADR_MASK       (0x7f << IADR_SHIFT)

#define IFDR_SHIFT      0
#define IFDR_MASK       (0x3f << IFDR_SHIFT)

#define I2CR_IEN        (1 << 7)  /* enable */
#define I2CR_IIEN       (1 << 6)  /* interrupt enable */
#define I2CR_MSTA       (1 << 5)  /* master/start */
#define I2CR_MTX        (1 << 4)  /* transmit */
#define I2CR_TXAK       (1 << 3)  /* ACK/NACK on reception */
#define I2CR_RSTA       (1 << 2)  /* repeated start */

#define I2SR_ICF        (1 << 7)  /* transfer in progress */
#define I2SR_IAAS       (1 << 6)  /* addressed as a slave */
#define I2SR_IBB        (1 << 5)  /* is busy */
#define I2SR_IAL        (1 << 4)  /* arbitration lost */
#define I2SR_SRW        (1 << 2)  /* slave read/write (if in slave mode) */
#define I2SR_IIF        (1 << 1)  /* interrupt is pending */
#define I2SR_RXAK       (1 << 0)  /* no ack detected */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_I2C_H */
