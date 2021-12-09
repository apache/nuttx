/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_i2c.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_I2C_CTRL_CR2             7
#define MPFS_I2C_CTRL_ENS1            6
#define MPFS_I2C_CTRL_STA             5
#define MPFS_I2C_CTRL_STO             4
#define MPFS_I2C_CTRL_SI              3
#define MPFS_I2C_CTRL_AA              2
#define MPFS_I2C_CTRL_CR1             1
#define MPFS_I2C_CTRL_CR0             0

#define MPFS_I2C_CTRL_CR2_MASK        (1 << 7)
#define MPFS_I2C_CTRL_ENS1_MASK       (1 << 6)
#define MPFS_I2C_CTRL_STA_MASK        (1 << 5)
#define MPFS_I2C_CTRL_STO_MASK        (1 << 4)
#define MPFS_I2C_CTRL_SI_MASK         (1 << 3)
#define MPFS_I2C_CTRL_AA_MASK         (1 << 2)
#define MPFS_I2C_CTRL_CR1_MASK        (1 << 1)
#define MPFS_I2C_CTRL_CR0_MASK        (1 << 0)

#define MPFS_I2C_ST_IDLE              0xF8  /* No activity, I2C bus idle */
#define MPFS_I2C_ST_RESET_ACTIVATED   0xD0  /* Master reset is activated */
#define MPFS_I2C_ST_RX_DATA_NACK      0x58  /* Data received, NACK sent */
#define MPFS_I2C_ST_RX_DATA_ACK       0x50  /* Data received, ACK sent */
#define MPFS_I2C_ST_SLAR_NACK         0x48  /* SLA+R sent, NACK'ed */
#define MPFS_I2C_ST_SLAR_ACK          0x40  /* SLA+R sent, ACK'ed */
#define MPFS_I2C_ST_LOST_ARB          0x38  /* Master lost arbitration */
#define MPFS_I2C_ST_TX_DATA_NACK      0x30  /* Data sent, NACK'ed */
#define MPFS_I2C_ST_TX_DATA_ACK       0x28  /* Data sent, ACK'ed */
#define MPFS_I2C_ST_SLAW_NACK         0x20  /* SLA + W sent, nack received */
#define MPFS_I2C_ST_SLAW_ACK          0x18  /* SLA + W sent, ack received */
#define MPFS_I2C_ST_RESTART           0x10  /* Repeated start */
#define MPFS_I2C_ST_START             0x08  /* Start condition sent */
#define MPFS_I2C_ST_BUS_ERROR         0x00  /* Bus error */

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_H */
