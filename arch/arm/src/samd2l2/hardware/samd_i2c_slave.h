/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_i2c_slave.h
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

/* References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J-SAM-12/2013
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_SLAVE_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/samd_sercom.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD20

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C register offsets *****************************************************/

#define SAM_I2C_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_I2C_CTRLB_OFFSET       0x0004  /* Control B register */

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  define SAM_I2C_INTENCLR_OFFSET  0x000c  /* Interrupt enable clear register */
#  define SAM_I2C_INTENSET_OFFSET  0x000d  /* Interrupt enable set register */
#  define SAM_I2C_INTFLAG_OFFSET   0x000e  /* Interrupt flag and status clear register */
#  define SAM_I2C_STATUS_OFFSET    0x0010  /* Status register */
#  define SAM_I2C_ADDR_OFFSET      0x0014  /* Address register */
#  define SAM_I2C_DATA_OFFSET      0x0018  /* Data register */
#elif defined(CONFIG_ARCH_FAMILY_SAMD21)
#  define SAM_I2C_INTENCLR_OFFSET  0x0014  /* Interrupt enable clear register */
#  define SAM_I2C_INTENSET_OFFSET  0x0016  /* Interrupt enable set register */
#  define SAM_I2C_INTFLAG_OFFSET   0x0018  /* Interrupt flag and status clear register */
#  define SAM_I2C_STATUS_OFFSET    0x001a  /* Status register */
#  define SAM_I2C_SYNCBUSY_OFFSET  0x001c  /* Synchronization busy register */
#  define SAM_I2C_ADDR_OFFSET      0x0024  /* Address register */
#  define SAM_I2C_DATA_OFFSET      0x0028  /* Data register */
#endif

/* I2C register addresses ***************************************************/

#define SAM_I2C0_CTRLA             (SAM_SERCOM0_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C0_CTRLB             (SAM_SERCOM0_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C0_INTENCLR          (SAM_SERCOM0_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C0_INTENSET          (SAM_SERCOM0_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C0_INTFLAG           (SAM_SERCOM0_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C0_STATUS            (SAM_SERCOM0_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C0_SYNCBUSY        (SAM_SERCOM0_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C0_ADDR              (SAM_SERCOM0_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C0_DATA              (SAM_SERCOM0_BASE+SAM_I2C_DATA_OFFSET)

#define SAM_I2C1_CTRLA             (SAM_SERCOM1_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C1_CTRLB             (SAM_SERCOM1_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C1_INTENCLR          (SAM_SERCOM1_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C1_INTENSET          (SAM_SERCOM1_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C1_INTFLAG           (SAM_SERCOM1_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C1_STATUS            (SAM_SERCOM1_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C1_SYNCBUSY        (SAM_SERCOM1_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C1_ADDR              (SAM_SERCOM1_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C1_DATA              (SAM_SERCOM1_BASE+SAM_I2C_DATA_OFFSET)

#define SAM_I2C2_CTRLA             (SAM_SERCOM2_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C2_CTRLB             (SAM_SERCOM2_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C2_INTENCLR          (SAM_SERCOM2_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C2_INTENSET          (SAM_SERCOM2_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C2_INTFLAG           (SAM_SERCOM2_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C2_STATUS            (SAM_SERCOM2_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C2_SYNCBUSY        (SAM_SERCOM2_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C2_ADDR              (SAM_SERCOM2_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C2_DATA              (SAM_SERCOM2_BASE+SAM_I2C_DATA_OFFSET)

#define SAM_I2C3_CTRLA             (SAM_SERCOM3_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C3_CTRLB             (SAM_SERCOM3_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C3_INTENCLR          (SAM_SERCOM3_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C3_INTENSET          (SAM_SERCOM3_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C3_INTFLAG           (SAM_SERCOM3_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C3_STATUS            (SAM_SERCOM3_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C3_SYNCBUSY        (SAM_SERCOM3_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C3_ADDR              (SAM_SERCOM3_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C3_DATA              (SAM_SERCOM3_BASE+SAM_I2C_DATA_OFFSET)

#define SAM_I2C4_CTRLA             (SAM_SERCOM4_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C4_CTRLB             (SAM_SERCOM4_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C4_INTENCLR          (SAM_SERCOM4_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C4_INTENSET          (SAM_SERCOM4_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C4_INTFLAG           (SAM_SERCOM4_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C4_STATUS            (SAM_SERCOM4_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C4_SYNCBUSY        (SAM_SERCOM4_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C4_ADDR              (SAM_SERCOM4_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C4_DATA              (SAM_SERCOM4_BASE+SAM_I2C_DATA_OFFSET)

#define SAM_I2C5_CTRLA             (SAM_SERCOM5_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C5_CTRLB             (SAM_SERCOM5_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C5_INTENCLR          (SAM_SERCOM5_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C5_INTENSET          (SAM_SERCOM5_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C5_INTFLAG           (SAM_SERCOM5_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C5_STATUS            (SAM_SERCOM5_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C5_SYNCBUSY        (SAM_SERCOM5_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C5_ADDR              (SAM_SERCOM5_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C5_DATA              (SAM_SERCOM5_BASE+SAM_I2C_DATA_OFFSET)

/* I2C register bit definitions *********************************************/

/* Control A register */

#define I2C_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define I2C_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define I2C_CTRLA_MODE_SHIFT       (2)       /* Bits 2-4: Operating Mode */
#define I2C_CTRLA_MODE_MASK        (7 << I2C_CTRLA_MODE_SHIFT)
#  define I2C_CTRLA_MODE_SLAVE     (4 << I2C_CTRLA_MODE_SHIFT) /* I2C slave mode */

#define I2C_CTRLA_RUNSTDBY         (1 << 7)  /* Bit 7:  Run in standby */
#define I2C_CTRLA_PINOUT           (1 << 16) /* Bit 16: Transmit data pinout */

#  define I2C_CTRLA_1WIRE          (0)              /* 4-wire operation disable */
#  define I2C_CTRLA_4WIRE          I2C_CTRLA_PINOUT /* 4-wire operation enable */

#define I2C_CTRLA_SDAHOLD_SHIFT    (20)      /* Bits 20-21: SDA Hold Time */
#define I2C_CTRLA_SDAHOLD_MASK     (3 << I2C_CTRLA_SDAHOLD_SHIFT)
#  define I2C_CTRLA_SDAHOLD_DIS    (0 << I2C_CTRLA_SDAHOLD_SHIFT) /* Disabled */
#  define I2C_CTRLA_SDAHOLD_75NS   (1 << I2C_CTRLA_SDAHOLD_SHIFT) /* 50-100ns hold time */
#  define I2C_CTRLA_SDAHOLD_450NS  (2 << I2C_CTRLA_SDAHOLD_SHIFT) /* 300-600ns hold time */
#  define I2C_CTRLA_SDAHOLD_600NS  (3 << I2C_CTRLA_SDAHOLD_SHIFT) /* 400-800ns hold time */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_CTRLA_SEXTTOEN       (1 << 23) /* Bit 23: Slave SCL low extend time-out */
#  define I2C_CTRLA_SPEED_SHIFT    (24)      /* Bits 24-25:  Transfer speed */
#  define I2C_CTRLA_SPEED_MASK     (3 << I2C_CTRLA_SPEED_SHIFT)
#    define I2C_CTRLA_SPEED_STANDARD  (0 << I2C_CTRLA_SPEED_SHIFT) /* Standard-mode (<=100 kHz) and Fast-mode (<=400 kHz) */
#    define I2C_CTRLA_SPEED_FAST      (1 << I2C_CTRLA_SPEED_SHIFT) /* Fast-mode Plus (<=1 MHz) */
#    define I2C_CTRLA_SPEED_HIGHSPEED (2 << I2C_CTRLA_SPEED_SHIFT) /* High-speed mode (<=3.4 MHz) */

#  define I2C_CTRLA_SCLSM          (1 << 27) /* Bit 27: SCL clock stretch mode */
#endif

#define I2C_CTRLA_LOWTOUT          (1 << 30)  /* Bit 30: SCL low time-out */

/* Control B register */

#define I2C_CTRLB_SMEN             (1 << 8)  /* Bit 8:  Smart Mode Enable */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_CTRLB_GCMD           (1 << 9)  /* Bit 8:  PMBus group command */
#  define I2C_CTRLB_AACKEN         (1 << 10) /* Bit 10: Automatic acknowledge enable */
#endif

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

/* Interrupt enable clear, interrupt enable set, interrupt enable set,
 * interrupt flag and status clear registers.
 */

#define I2C_INT_PREC               (1 << 0)  /* Bit 0:  Stop received interrupt */
#define I2C_INT_AMATCH             (1 << 1)  /* Bit 1:  Address match interrupt */
#define I2C_INT_DRDY               (1 << 2)  /* Bit 2:  Data ready interrupt */

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  define I2C_INT_ALL              (0x07)
#elif defined(CONFIG_ARCH_FAMILY_SAMD21)
#  define I2C_INT_ERROR            (1 << 7)  /* Bit 7:  Error interrupt */
#  define I2C_INT_ALL              (0x87)
#endif

/* Status register */

#define I2C_STATUS_BUSERR          (1 << 0)  /* Bit 0:  Bus Error */
#define I2C_STATUS_COLL            (1 << 1)  /* Bit 1:  Transmit Collision */
#define I2C_STATUS_RXNACK          (1 << 2)  /* Bit 2:  Received Not Acknowledge */
#define I2C_STATUS_DIR             (1 << 3)  /* Bit 3:  Read / Write Direction */
#define I2C_STATUS_SR              (1 << 4)  /* Bit 4:  Repeated Start */
#define I2C_STATUS_LOWTOUT         (1 << 6)  /* Bit 6:  SCL Low Time-Out */
#define I2C_STATUS_CLKHOLD         (1 << 7)  /* Bit 7:  Clock Hold */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_STATUS_SEXTTOUT      (1 << 9)  /* Bit 9:  Slave SCL low extend time-out */
#  define I2C_STATUS_HS            (1 << 10) /* Bit 10: High speed */
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define I2C_STATUS_SYNCBUSY      (1 << 15) /* Bit 15: Synchronization busy */
#endif

/* Synchronization busy register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_SYNCBUSY_SWRST       (1 << 0)  /* Bit 0: Software reset synchronization busy */
#  define I2C_SYNCBUSY_ENABLE      (1 << 1)  /* Bit 1: SERCOM enable synchronization busy */

#  define I2C_SYNCBUSY_ALL         0x0003
#endif

/* Address register */

#define SPI_ADDR_GENCEN            (1 << 0)  /* Bit 0:  General call address enable */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define SPI_ADDR_SHIFT           (1)       /* Bits 1-7: Address */
#  define SPI_ADDR_MASK            (0x7f << SPI_ADDR_SHIFT)
#    define SPI_ADDR(n)            ((uint32_t)(n) << SPI_ADDR_SHIFT)
#  define SPI_ADDRMASK_SHIFT       (17)      /* Bits 17-23: Address Mask */
#  define SPI_ADDRMASK_MASK        (0x7f << SPI_ADDRMASK_SHIFT)
#    define SPI_ADDRMASK(n)        ((uint32_t)(n) << SPI_ADDRMASK_SHIFT)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SPI_ADDR_SHIFT           (1)       /* Bits 1-10: Address */
#  define SPI_ADDR_MASK            (0x3ff << SPI_ADDR_SHIFT)
#    define SPI_ADDR(n)            ((uint32_t)(n) << SPI_ADDR_SHIFT)
#  define SPI_ADDR_TENBITEN        (1 << 15) /* Bit 15: Ten bit addressing enable */
#  define SPI_ADDRMASK_SHIFT       (17)      /* Bits 17-26: Address Mask */
#  define SPI_ADDRMASK_MASK        (0x3ff << SPI_ADDRMASK_SHIFT)
#    define SPI_ADDRMASK(n)        ((uint32_t)(n) << SPI_ADDRMASK_SHIFT)
#endif

/* Data register */

#define I2C_DATA_MASK              (0xooff)  /* Bits 0-7: Data */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_SLAVE_H */
