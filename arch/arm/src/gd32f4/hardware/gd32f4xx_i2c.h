/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_i2c.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_I2C_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2Cx(x=0,1,2,3,4,5) definitions */
#define GD32_I2C0_BASE             (GD32_I2C_BASE+0x00000000)      /* I2C0 base address */
#define GD32_I2C1_BASE             (GD32_I2C_BASE+0x00000400)      /* I2C1 base address */
#define GD32_I2C2_BASE             (GD32_I2C_BASE+0x00000800)      /* I2C2 base address */

/* Register Offsets *********************************************************/
#define GD32_I2C_CTL0_OFFSET       0x0000      /* I2C control register 0 offset */
#define GD32_I2C_CTL1_OFFSET       0x0004      /* I2C control register 1 offset */
#define GD32_I2C_SADDR0_OFFSET     0x0008      /* I2C slave address register 0 offset */
#define GD32_I2C_SADDR1_OFFSET     0x000C      /* I2C slave address register 1 offset */
#define GD32_I2C_DATA_OFFSET       0x0010      /* I2C transfer buffer register offset */
#define GD32_I2C_STAT0_OFFSET      0x0014      /* I2C transfer status register 0 offset */
#define GD32_I2C_STAT1_OFFSET      0x0018      /* I2C transfer status register 1 offset */
#define GD32_I2C_CKCFG_OFFSET      0x001C      /* I2C clock configure register offset */
#define GD32_I2C_RT_OFFSET         0x0020      /* I2C rise time register offset */
#define GD32_I2C_FCTL_OFFSET       0x0024      /* I2C filter control register offset */
#define GD32_I2C_SAMCS_OFFSET      0x0080      /* I2C SAM control and status register offset */

/* Register Addresses *******************************************************/

#define GD32_I2C0                  GD32_I2C0_BASE
#define GD32_I2C1                  GD32_I2C1_BASE
#define GD32_I2C2                  GD32_I2C2_BASE

/* I2C registers definitions */
#define GD32_I2C_CTL0(i2cx)        ((i2cx)+GD32_I2C_CTL0_OFFSET)       /* I2C control register 0 */
#define GD32_I2C_CTL1(i2cx)        ((i2cx)+GD32_I2C_CTL1_OFFSET)       /* I2C control register 1 */
#define GD32_I2C_SADDR0(i2cx)      ((i2cx)+GD32_I2C_SADDR0_OFFSET)     /* I2C slave address register 0 */
#define GD32_I2C_SADDR1(i2cx)      ((i2cx)+GD32_I2C_SADDR1_OFFSET)     /* I2C slave address register 1 */
#define GD32_I2C_DATA(i2cx)        ((i2cx)+GD32_I2C_DATA_OFFSET)       /* I2C transfer buffer register */
#define GD32_I2C_STAT0(i2cx)       ((i2cx)+GD32_I2C_STAT0_OFFSET)      /* I2C transfer status register 0 */
#define GD32_I2C_STAT1(i2cx)       ((i2cx)+GD32_I2C_STAT1_OFFSET)      /* I2C transfer status register 1 */
#define GD32_I2C_CKCFG(i2cx)       ((i2cx)+GD32_I2C_CKCFG_OFFSET)      /* I2C clock configure register */
#define GD32_I2C_RT(i2cx)          ((i2cx)+GD32_I2C_RT_OFFSET)         /* I2C rise time register */
#define GD32_I2C_FCTL(i2cx)        ((i2cx)+GD32_I2C_FCTL_OFFSET)       /* I2C filter control register */
#define GD32_I2C_SAMCS(i2cx)       ((i2cx)+GD32_I2C_SAMCS_OFFSET)      /* I2C SAM control and status register */

/* Register Bitfield Definitions ********************************************/

/* Control register 0 */

#define I2C_CTL0_I2CEN             (1 << 0)          /* Bit 0: Peripheral enable */
#define I2C_CTL0_SMBEN             (1 << 1)          /* Bit 1: SMBus mode */
#define I2C_CTL0_SMBSEL            (1 << 3)          /* Bit 3: SMBus type */
#define I2C_CTL0_ARPEN             (1 << 4)          /* Bit 4: ARP enable */
#define I2C_CTL0_PECEN             (1 << 5)          /* Bit 5: PEC enable */
#define I2C_CTL0_GCEN              (1 << 6)          /* Bit 6: General call enable */
#define I2C_CTL0_SS                (1 << 7)          /* Bit 7: Clock stretching disable (Slave mode) */
#define I2C_CTL0_START             (1 << 8)          /* Bit 8: Start generation */
#define I2C_CTL0_STOP              (1 << 9)          /* Bit 9: Stop generation */
#define I2C_CTL0_ACKEN             (1 << 10)         /* Bit 10: Acknowledge enable */
#define I2C_CTL0_POAP              (1 << 11)         /* Bit 11: Acknowledge/PEC position (for data reception) */
#define I2C_CTL0_PECTRANS          (1 << 12)         /* Bit 12: Packet error checking */
#define I2C_CTL0_SALT              (1 << 13)         /* Bit 13: SMBus alert */
#define I2C_CTL0_SRESET            (1 << 15)         /* Bit 15: Software reset */

/* Control register 1 */

#define I2C_CTL1_I2CCLK_SHIFT      (0)               /* Bits 5-0: Peripheral clock frequency */
#define I2C_CTL1_I2CCLK_MASK       (0x3f << I2C_CTL1_I2CCLK_SHIFT)
#define I2C_CTL1_I2CCLK(n)         ((n) << I2C_CTL1_I2CCLK_SHIFT)
#define I2C_CTL1_ERRIE             (1 << 8)          /* Bit 8: Error interrupt enable */
#define I2C_CTL1_EVIE              (1 << 9)          /* Bit 9: Event interrupt enable */
#define I2C_CTL1_BUFIE             (1 << 10)         /* Bit 10: Buffer interrupt enable */
#define I2C_CTL1_DMAON             (1 << 11)         /* Bit 11: DMA requests enable */
#define I2C_CTL1_DMALST            (1 << 12)         /* Bit 12: DMA last transfer */

#define I2C_CTL1_INTS_MASK         (I2C_CTL1_ERRIE|I2C_CTL1_EVIE|I2C_CTL1_BUFIE)

/* Slave address register 0 */

#define I2C_SADDR0_ADDRESS0        (1 << 0)          /* Bit 0: Bit 0 of 10-bit address */
#define I2C_SADDR0_ADDRESS_SHIFT   (1)               /* Bits 7-1: 7-bit address or bits 7:1 of a 10-bit address */
#define I2C_SADDR0_ADDRESS_MASK    (0x7f << I2C_SADDR0_ADDRESS_SHIFT)
#define I2C_SADDR0_ADDRESS_H_SHIFT (8)               /* Bits 9-8: Highest two bits of a 10-bit address */
#define I2C_SADDR0_ADDRESS_H_MASK  (0x3 << I2C_SADDR0_ADDRESS_H_SHIFT)
#define I2C_SADDR0_ADDFORMAT       (1 << 15)         /* Bit 15: Address mode for the I2C slave */

/* Slave address register 1 */

#define I2C_SADDR1_DUADEN          (1 << 0)          /* Bit 0: Dual-Address mode enable */
#define I2C_SADDR1_ADDRESS2_SHIFT  (1)               /* Bits 7-1: Second I2C address for the slave in dual-address mode */
#define I2C_SADDR1_ADDRESS2_MASK   (0x7f << I2C_SADDR1_ADDRESS2_SHIFT)

/* Transfer data register */

#define I2C_DATA_TRB_SHIFT         (0)               /* Bits 7-0: 8-bit data register */
#define I2C_DATA_TRB_MASK          (0x00ff << I2C_DATA_TRB_SHIFT)

/* Transfer status register 0 */

#define I2C_STAT0_SBSEND           (1 << 0)          /* Bit 0: Start bit (master mode) */
#define I2C_STAT0_ADDSEND          (1 << 1)          /* Bit 1: Address sent (master mode)/matched (slave mode) */
#define I2C_STAT0_BTC              (1 << 2)          /* Bit 2: Byte transfer finished */
#define I2C_STAT0_ADD10SEND        (1 << 3)          /* Bit 3: 10-bit header sent (master mode) */
#define I2C_STAT0_STPDET           (1 << 4)          /* Bit 4: Stop detection (slave mode) */
#define I2C_STAT0_RBNE             (1 << 6)          /* Bit 6: Data register not empty (receivers) */
#define I2C_STAT0_TBE              (1 << 7)          /* Bit 7: Data register empty (transmitters) */
#define I2C_STAT0_BERR             (1 << 8)          /* Bit 8: bus error */
#define I2C_STAT0_LOSTARB          (1 << 9)          /* Bit 9: Arbitration lost (master mode) */
#define I2C_STAT0_AERR             (1 << 10)         /* Bit 10: Acknowledge failure */
#define I2C_STAT0_OUERR            (1 << 11)         /* Bit 11: Overrun/underrun */
#define I2C_STAT0_PECERR           (1 << 12)         /* Bit 12: PEC error in reception */
#define I2C_STAT0_SMBTO            (1 << 14)         /* Bit 14: Timeout signal in SMBus mode */
#define I2C_STAT0_SMBALT           (1 << 15)         /* Bit 15: SMBus alert status */

#define I2C_STAT0_ERROR_MASK       (I2C_STAT0_BERR|I2C_STAT0_LOSTARB|I2C_STAT0_AERR|I2C_STAT0_OUERR|\
                                    I2C_STAT0_PECERR|I2C_STAT0_SMBTO|I2C_STAT0_SMBALT)

/* Transfer status register 1 */

#define I2C_STAT1_MASTER           (1 << 0)          /* Bit 0: Master/slave */
#define I2C_STAT1_I2CBSY           (1 << 1)          /* Bit 1: Bus busy */
#define I2C_STAT1_TR               (1 << 2)          /* Bit 2: Transmitter/receiver */
#define I2C_STAT1_RXGC             (1 << 4)          /* Bit 4: General call address (slave mode) */
#define I2C_STAT1_DEFSMB           (1 << 5)          /* Bit 5: SMBus device default address (slave mode) */
#define I2C_STAT1_HSTSMB           (1 << 6)          /* Bit 6: SMBus host header (slave mode) */
#define I2C_STAT1_DUMODF           (1 << 7)          /* Bit 7: Dual flag (slave mode) */
#define I2C_STAT1_PECV_SHIFT       (8)               /* Bits 15-8: Packet error checking value */
#define I2C_STAT1_PECV_MASK        (0xff << I2C_STAT1_PECV_SHIFT)

/* Clock configure register */

#define I2C_CKCFG_CLKC_SHIFT       (0)               /* Bits 11-0: Clock control register in fast/standard mode (master mode) */
#define I2C_CKCFG_CLKC_MASK        (0x0fff << I2C_CKCFG_CLKC_SHIFT)
#define I2C_CKCFG_DTCY             (1 << 14)         /* Bit 14: Fast mode duty Cycle */
#define I2C_CKCFG_FAST             (1 << 15)         /* Bit 15: I2C speed selection in master mode */

/* Rise time register */

#define I2C_RT_RISETIME_SHIFT      (0)               /* Bits 5-0: Maximum rise time in fast/standard mode (master mode) */
#define I2C_RT_RISETIME_MASK       (0x3f << I2C_RT_RISETIME_SHIFT)

/* Filter control register */

#define I2C_FCTL_DF_SHIFT          (0)               /* Bit 3-0: Digital noise filter */
#define I2C_FCTL_DFMASK            (0xf << )I2C_FCTL_DF_SHIFT
#define I2C_FCTL_AFD               (4)               /* Bit 4: Aanalog noise filter disable */

/* SAM control and status register */

#define I2C_SAMCS_SAMEN            (0)               /* Bit 0: SAM_V interface enable */
#define I2C_SAMCS_STOEN            (1)               /* Bit 1: SAM_V interface timeout detect enable */
#define I2C_SAMCS_TFFIE            (4)               /* Bit 4: Txframe fall interrupt enable */
#define I2C_SAMCS_TFRIE            (5)               /* Bit 5: Txframe rise interrupt enable */
#define I2C_SAMCS_RFFIE            (6)               /* Bit 6: Rxframe fall interrupt enable */
#define I2C_SAMCS_RFRIE            (7)               /* Bit 7: Rxframe rise interrupt enable */
#define I2C_SAMCS_TXF              (8)               /* Bit 8: Level of txframe signal */
#define I2C_SAMCS_RXF              (9)               /* Bit 9: Level of rxframe signal */
#define I2C_SAMCS_TFF              (12)              /* Bit 12: Txframe fall flag */
#define I2C_SAMCS_TFR              (13)              /* Bit 13: Txframe rise flag */
#define I2C_SAMCS_RFF              (14)              /* Bit 14: Rxframe fall flag */
#define I2C_SAMCS_RFR              (15)              /* Bit 15: Rxframe rise flag */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_I2C_H */
