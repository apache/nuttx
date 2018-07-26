/********************************************************************************************
 * arch/arm/src/samd5e5/chip/sam_i2c_slave.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_I2C_SLAVE_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_I2C_SLAVE_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* I2C register offsets *********************************************************************/

#define SAM_I2C_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_I2C_CTRLB_OFFSET       0x0004  /* Control B register */
#define SAM_I2C_CTRLC_OFFSET       0x0008  /* Control C register */
#define SAM_I2C_INTENCLR_OFFSET    0x0014  /* Interrupt enable clear register */
#define SAM_I2C_INTENSET_OFFSET    0x0016  /* Interrupt enable set register */
#define SAM_I2C_INTFLAG_OFFSET     0x0018  /* Interrupt flag and status clear register */
#define SAM_I2C_STATUS_OFFSET      0x001a  /* Status register */
#define SAM_I2C_SYNCBUSY_OFFSET    0x001c  /* Synchronization busy register */
#define SAM_I2C_LENGTH_OFFSET      0x0022  /* Length register */
#define SAM_I2C_ADDR_OFFSET        0x0024  /* Address register */
#define SAM_I2C_DATA_OFFSET        0x0028  /* Data register */

/* I2C register addresses *******************************************************************/

#define SAM_I2C0_CTRLA             (SAM_SERCOM0_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C0_CTRLB             (SAM_SERCOM0_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C0_CTRLC             (SAM_SERCOM0_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C0_INTENCLR          (SAM_SERCOM0_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C0_INTENSET          (SAM_SERCOM0_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C0_INTFLAG           (SAM_SERCOM0_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C0_STATUS            (SAM_SERCOM0_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C0_SYNCBUSY          (SAM_SERCOM0_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C0_LENGTH            (SAM_SERCOM0_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C0_ADDR              (SAM_SERCOM0_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C0_DATA              (SAM_SERCOM0_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C1_CTRLA             (SAM_SERCOM1_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C1_CTRLB             (SAM_SERCOM1_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C1_CTRLC             (SAM_SERCOM1_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C1_INTENCLR          (SAM_SERCOM1_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C1_INTENSET          (SAM_SERCOM1_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C1_INTFLAG           (SAM_SERCOM1_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C1_STATUS            (SAM_SERCOM1_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C1_SYNCBUSY          (SAM_SERCOM1_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C1_LENGTH            (SAM_SERCOM1_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C1_ADDR              (SAM_SERCOM1_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C1_DATA              (SAM_SERCOM1_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C2_CTRLA             (SAM_SERCOM2_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C2_CTRLB             (SAM_SERCOM2_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C2_CTRLC             (SAM_SERCOM2_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C2_INTENCLR          (SAM_SERCOM2_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C2_INTENSET          (SAM_SERCOM2_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C2_INTFLAG           (SAM_SERCOM2_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C2_STATUS            (SAM_SERCOM2_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C2_SYNCBUSY          (SAM_SERCOM2_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C2_LENGTH            (SAM_SERCOM2_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C2_ADDR              (SAM_SERCOM2_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C2_DATA              (SAM_SERCOM2_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C3_CTRLA             (SAM_SERCOM3_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C3_CTRLB             (SAM_SERCOM3_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C3_CTRLC             (SAM_SERCOM3_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C3_INTENCLR          (SAM_SERCOM3_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C3_INTENSET          (SAM_SERCOM3_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C3_INTFLAG           (SAM_SERCOM3_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C3_STATUS            (SAM_SERCOM3_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C3_SYNCBUSY          (SAM_SERCOM3_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C3_LENGTH            (SAM_SERCOM3_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C3_ADDR              (SAM_SERCOM3_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C3_DATA              (SAM_SERCOM3_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C4_CTRLA             (SAM_SERCOM4_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C4_CTRLB             (SAM_SERCOM4_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C4_CTRLC             (SAM_SERCOM4_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C4_INTENCLR          (SAM_SERCOM4_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C4_INTENSET          (SAM_SERCOM4_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C4_INTFLAG           (SAM_SERCOM4_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C4_STATUS            (SAM_SERCOM4_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C4_SYNCBUSY          (SAM_SERCOM4_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C4_LENGTH            (SAM_SERCOM4_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C4_ADDR              (SAM_SERCOM4_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C4_DATA              (SAM_SERCOM4_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C5_CTRLA             (SAM_SERCOM5_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C5_CTRLB             (SAM_SERCOM5_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C5_CTRLC             (SAM_SERCOM5_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C5_INTENCLR          (SAM_SERCOM5_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C5_INTENSET          (SAM_SERCOM5_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C5_INTFLAG           (SAM_SERCOM5_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C5_STATUS            (SAM_SERCOM5_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C5_SYNCBUSY          (SAM_SERCOM5_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C5_LENGTH            (SAM_SERCOM5_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C5_ADDR              (SAM_SERCOM5_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C5_DATA              (SAM_SERCOM5_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C6_CTRLA             (SAM_SERCOM6_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C6_CTRLB             (SAM_SERCOM6_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C6_CTRLC             (SAM_SERCOM6_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C6_INTENCLR          (SAM_SERCOM6_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C6_INTENSET          (SAM_SERCOM6_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C6_INTFLAG           (SAM_SERCOM6_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C6_STATUS            (SAM_SERCOM6_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C6_SYNCBUSY          (SAM_SERCOM6_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C6_LENGTH            (SAM_SERCOM6_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C6_ADDR              (SAM_SERCOM6_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C6_DATA              (SAM_SERCOM6_BASE + SAM_I2C_DATA_OFFSET)

#define SAM_I2C7_CTRLA             (SAM_SERCOM7_BASE + SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C7_CTRLB             (SAM_SERCOM7_BASE + SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C7_CTRLC             (SAM_SERCOM7_BASE + SAM_I2C_CTRLC_OFFSET)
#define SAM_I2C7_INTENCLR          (SAM_SERCOM7_BASE + SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C7_INTENSET          (SAM_SERCOM7_BASE + SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C7_INTFLAG           (SAM_SERCOM7_BASE + SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C7_STATUS            (SAM_SERCOM7_BASE + SAM_I2C_STATUS_OFFSET)
#define SAM_I2C7_SYNCBUSY          (SAM_SERCOM7_BASE + SAM_I2C_SYNCBUSY_OFFSET)
#define SAM_I2C7_LENGTH            (SAM_SERCOM7_BASE + SAM_I2C_LENGTH_OFFSET)
#define SAM_I2C7_ADDR              (SAM_SERCOM7_BASE + SAM_I2C_ADDR_OFFSET)
#define SAM_I2C7_DATA              (SAM_SERCOM7_BASE + SAM_I2C_DATA_OFFSET)

/* I2C register bit definitions *************************************************************/

/* Control A register */

#define I2C_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define I2C_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define I2C_CTRLA_MODE_SHIFT       (2)       /* Bits 2-4: Operating Mode */
#define I2C_CTRLA_MODE_MASK        (7 << I2C_CTRLA_MODE_SHIFT)
#  define I2C_CTRLA_MODE_SLAVE     (4 << I2C_CTRLA_MODE_SHIFT) /* I2C slave mode */
#define I2C_CTRLA_RUNSTDBY         (1 << 7)  /* Bit 7:  Run in standby */
#define I2C_CTRLA_PINOUT           (1 << 16) /* Bit 16: Pin usage */
#  define I2C_CTRLA_1WIRE          (0)              /* 4-wire operation disabled */
#  define I2C_CTRLA_4WIRE          I2C_CTRLA_PINOUT /* 4-wire operation enabled */
#define I2C_CTRLA_SDAHOLD_SHIFT    (20)      /* Bits 20-21: SDA Hold Time */
#define I2C_CTRLA_SDAHOLD_MASK     (3 << I2C_CTRLA_SDAHOLD_SHIFT)
#  define I2C_CTRLA_SDAHOLD_DIS    (0 << I2C_CTRLA_SDAHOLD_SHIFT) /* Disabled */
#  define I2C_CTRLA_SDAHOLD_75NS   (1 << I2C_CTRLA_SDAHOLD_SHIFT) /* 50-100ns hold time */
#  define I2C_CTRLA_SDAHOLD_450NS  (2 << I2C_CTRLA_SDAHOLD_SHIFT) /* 300-600ns hold time */
#  define I2C_CTRLA_SDAHOLD_600NS  (3 << I2C_CTRLA_SDAHOLD_SHIFT) /* 400-800ns hold time */
#define I2C_CTRLA_SEXTTOEN         (1 << 23) /* Bit 23: Slave SCL low extend time-out */
#define I2C_CTRLA_SPEED_SHIFT      (24)      /* Bits 24-25: Trnasfer speed */
#define I2C_CTRLA_SPEED_MASK       (3 << I2C_CTRLA_SPEED_SHIFT)
#  define I2C_CTRLA_SPEED_STD      (0 << I2C_CTRLA_SPEED_SHIFT) /* Standard (<=100KHz) fast <=400KHz */
#  define I2C_CTRLA_SPEED_FAST     (1 << I2C_CTRLA_SPEED_SHIFT) /* Fast-mode plase (<=1MHz) */
#  define I2C_CTRLA_SPEED_HIGH     (2 << I2C_CTRLA_SPEED_SHIFT) /* High-speed mode (<=3.4Mhz */
#define I2C_CTRLA_SCLSM            (1 << 27) /* Bit 27: SCL clock stretch mode */
#define I2C_CTRLA_LOWTOUT          (1 << 30) /* Bit 30: SCL Low Time-Out */

/* Control B register */

#define I2C_CTRLB_SMEN             (1 << 8)  /* Bit 8:  Smart Mode Enable */
#define I2C_CTRLB_GCMD             (1 << 9)  /* Bit 9:  PMBus group commend */
#define I2C_CTRLB_AACKEN           (1 << 10) /* Bit 10: Automatic acknowledge enable */
#define I2C_CRLB_AMODE_SHIFT       (14)      /* Bits 14-15: Address Mode */
#define I2C_CRLB_AMODE_MASK        (3 << I2C_CRLB_AMODE_SHIFT)
#  define I2C_CRLB_AMODE_MASK      (0 << I2C_CRLB_AMODE_SHIFT) /* ADDRMASK used to mask ADDR */
#  define I2C_CRLB_AMODE_2ADDRS    (1 << I2C_CRLB_AMODE_SHIFT) /* Slave 2 addresses: ADDR & ADDRMASK */
#  define I2C_CRLB_AMODE_RANGE     (2 << I2C_CRLB_AMODE_SHIFT) /* Slave range of addresses: ADDRMASK-ADDR */
#define I2C_CTRLB_CMD_SHIFT        (16)      /* Bits 16-17: Command */
#define I2C_CTRLB_CMD_MASK         (3 << I2C_CTRLB_CMD_SHIFT)
#  define I2C_CTRLB_CMD_NOACTION   (0 << I2C_CTRLB_CMD_SHIFT) /* No action */
#  define I2C_CTRLB_CMD_WAITSTART  (2 << I2C_CTRLB_CMD_SHIFT) /* ACK (write) wait for START */
#  define I2C_CTRLB_CMD_ACKREAD    (3 << I2C_CTRLB_CMD_SHIFT) /* ACK with read (context dependent) */
#define I2C_CTRLB_ACKACT           (1 << 18) /* Bit 18: Acknowledge Action */
#  define I2C_CTRLB_ACK            (0)              /* Send ACK */
#  define I2C_CTRLB_NCK            I2C_CTRLB_ACKACT /* Send NACK */

/* Control C register */

#define I2C_CTRLC_SDASETUP_SHIFT   (0)       /* Bits 0-3: Inter-Character Spacing */
#define I2C_CTRLC_SDASETUP_MASK    (15 << I2C_CTRLC_SDASETUP_SHIFT)
#  define I2C_CTRLC_SDASETUP(n)    ((uint32_t)(n) << I2C_CTRLC_SDASETUP_SHIFT)
#define I2C_CTRLC_DATA32B          (1 << 24) /* Bit 24: Data 32 Bit */
#  define I2C_CTRLC_DATA32B_8BIT   (0)               /* DATA register is 8-bit */
#  define I2C_CTRLC_DATA32B_32BIT  I2C_CTRLC_DATA32B /* DATA register is 32-bit */

/* Interrupt enable clear, interrupt enable set, interrupt enable set, interrupt flag and
 * status clear registers.
 */

#define I2C_INT_PREC               (1 << 0)  /* Bit 0:  Stop received interrupt */
#define I2C_INT_AMATCH             (1 << 1)  /* Bit 1:  Address match interrupt */
#define I2C_INT_DRDY               (1 << 2)  /* Bit 2:  Data ready interrupt */
#define I2C_INT_ERROR              (1 << 7)  /* Bit 7:  Error interrupt */

#define I2C_INT_ALL                (0x87)

/* Status register */

#define I2C_STATUS_BUSERR          (1 << 0)  /* Bit 0:  Bus Error */
#define I2C_STATUS_COLL            (1 << 1)  /* Bit 1:  Transmit Collision */
#define I2C_STATUS_RXNACK          (1 << 2)  /* Bit 2:  Received Not Acknowledge */
#define I2C_STATUS_DIR             (1 << 3)  /* Bit 3:  Read / Write Direction */
#define I2C_STATUS_SR              (1 << 4)  /* Bit 4:  Repeated Start */
#define I2C_STATUS_LOWTOUT         (1 << 6)  /* Bit 6:  SCL Low Time-out */
#define I2C_STATUS_CLKHOLD         (1 << 7)  /* Bit 7:  Clock Hold */
#define I2C_STATUS_SEXTTOUT        (1 << 9)  /* Bit 9:  Slave SCL Low Extend Time-Out */
#define I2C_STATUS_HS              (1 << 10) /* Bit 10: High-speed */
#define I2C_STATUS_LENERR          (1 << 11) /* Bit 11: Transaction Length Error */

/* Synchronization busy register */

#define I2C_SYNCBUSY_SWRST         (1 << 0)  /* Bit 0:  Software reset synchronization busy */
#define I2C_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  SERCOM enable synchronization busy */
#define I2C_SYNCBUSY_LENGTH        (1 << 4)  /* Bit 3:  LENGTH synchronization busy */

/* Length register */

#define I2C_LENGTH_LEN_SHIFT       (0)       /* Bit 0-7: Data length enable */
#define I2C_LENGTH_LEN_MASK        (0xff << I2C_LENGTH_LEN_SHIFT)
#  define I2C_LENGTHLEN(n)         ((uint32_t)(n) << I2C_LENGTH_LEN_SHIFT)
#define I2C_LENGTH_LENEN           (1 << 8)  /* Bit 8: Data Length Enable */

/* Address register */

#define I2C_ADDR_GENCEN            (1 << 0)  /* Bit 0:  General Call Address Enable */
#define I2C_ADDR_SHIFT             (1)       /* Bits 1-10: Address */
#define I2C_ADDR_MASK              (0x3ff << I2C_ADDR_SHIFT)
#  define I2C_ADDR(n)              ((uint32_t)(n) << I2C_ADDR_SHIFT)
#define I2C_ADDR_TENBITEN          (1 << 15) /* Bit 15: */
#define I2C_ADDRMASK_SHIFT         (17)      /* Bits 17-26: Address Mask */
#define I2C_ADDRMASK_MASK          (0x3ff << I2C_ADDRMASK_SHIFT)
#  define I2C_ADDRMASK(n)          ((uint32_t)(n) << I2C_ADDRMASK_SHIFT)

/* Data register (8- or 32-bit data) */

#define I2C_DATA_MASK              (0xff)    /* Bits 0-7: Data */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_I2C_SLAVE_H */
