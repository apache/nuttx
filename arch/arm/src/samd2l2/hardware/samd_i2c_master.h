/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_i2c_master.h
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
 *   Datasheet", 42129J–SAM–12/2013
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_MASTER_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_MASTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/samd_sercom.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C register offsets *****************************************************/

#define SAM_I2C_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_I2C_CTRLB_OFFSET       0x0004  /* Control B register */

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  define SAM_I2C_BAUD_OFFSET      0x000a  /* Baud register */
#  define SAM_I2C_DBGCTRL_OFFSET   0x0008  /* Debug control register */
#  define SAM_I2C_INTENCLR_OFFSET  0x000c  /* Interrupt enable clear register */
#  define SAM_I2C_INTENSET_OFFSET  0x000d  /* Interrupt enable set register */
#  define SAM_I2C_INTFLAG_OFFSET   0x000e  /* Interrupt flag and status clear register */
#  define SAM_I2C_STATUS_OFFSET    0x0010  /* Status register */
#  define SAM_I2C_ADDR_OFFSET      0x0014  /* Address register */
#  define SAM_I2C_DATA_OFFSET      0x0018  /* Data register */
#elif defined(CONFIG_ARCH_FAMILY_SAMD21)
#  define SAM_I2C_BAUD_OFFSET      0x000c  /* Baud register */
#  define SAM_I2C_INTENCLR_OFFSET  0x0014  /* Interrupt enable clear register */
#  define SAM_I2C_INTENSET_OFFSET  0x0016  /* Interrupt enable set register */
#  define SAM_I2C_INTFLAG_OFFSET   0x0018  /* Interrupt flag and status clear register */
#  define SAM_I2C_STATUS_OFFSET    0x001a  /* Status register */
#  define SAM_I2C_SYNCBUSY_OFFSET  0x001c  /* Synchronization busy register */
#  define SAM_I2C_ADDR_OFFSET      0x0024  /* Address register */
#  define SAM_I2C_DATA_OFFSET      0x0028  /* Data register */
#  define SAM_I2C_DBGCTRL_OFFSET   0x0030  /* Debug control register */
#endif

/* I2C register addresses ***************************************************/

#define SAM_I2C0_CTRLA             (SAM_SERCOM0_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C0_CTRLB             (SAM_SERCOM0_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C0_BAUD              (SAM_SERCOM0_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C0_INTENCLR          (SAM_SERCOM0_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C0_INTENSET          (SAM_SERCOM0_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C0_INTFLAG           (SAM_SERCOM0_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C0_STATUS            (SAM_SERCOM0_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C0_SYNCBUSY        (SAM_SERCOM0_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C0_ADDR              (SAM_SERCOM0_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C0_DATA              (SAM_SERCOM0_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C0_DBGCTRL           (SAM_SERCOM0_BASE+SAM_I2C_DBGCTRL_OFFSET)

#define SAM_I2C1_CTRLA             (SAM_SERCOM1_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C1_CTRLB             (SAM_SERCOM1_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C1_BAUD              (SAM_SERCOM1_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C1_INTENCLR          (SAM_SERCOM1_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C1_INTENSET          (SAM_SERCOM1_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C1_INTFLAG           (SAM_SERCOM1_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C1_STATUS            (SAM_SERCOM1_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C1_SYNCBUSY        (SAM_SERCOM1_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C1_ADDR              (SAM_SERCOM1_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C1_DATA              (SAM_SERCOM1_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C1_DBGCTRL           (SAM_SERCOM1_BASE+SAM_I2C_DBGCTRL_OFFSET)

#define SAM_I2C2_CTRLA             (SAM_SERCOM2_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C2_CTRLB             (SAM_SERCOM2_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C2_BAUD              (SAM_SERCOM2_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C2_INTENCLR          (SAM_SERCOM2_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C2_INTENSET          (SAM_SERCOM2_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C2_INTFLAG           (SAM_SERCOM2_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C2_STATUS            (SAM_SERCOM2_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C2_SYNCBUSY        (SAM_SERCOM2_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C2_ADDR              (SAM_SERCOM2_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C2_DATA              (SAM_SERCOM2_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C2_DBGCTRL           (SAM_SERCOM2_BASE+SAM_I2C_DBGCTRL_OFFSET)

#define SAM_I2C3_CTRLA             (SAM_SERCOM3_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C3_CTRLB             (SAM_SERCOM3_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C3_BAUD              (SAM_SERCOM3_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C3_INTENCLR          (SAM_SERCOM3_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C3_INTENSET          (SAM_SERCOM3_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C3_INTFLAG           (SAM_SERCOM3_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C3_STATUS            (SAM_SERCOM3_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C3_SYNCBUSY        (SAM_SERCOM3_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C3_ADDR              (SAM_SERCOM3_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C3_DATA              (SAM_SERCOM3_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C3_DBGCTRL           (SAM_SERCOM3_BASE+SAM_I2C_DBGCTRL_OFFSET)

#define SAM_I2C4_CTRLA             (SAM_SERCOM4_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C4_CTRLB             (SAM_SERCOM4_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C4_BAUD              (SAM_SERCOM4_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C4_INTENCLR          (SAM_SERCOM4_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C4_INTENSET          (SAM_SERCOM4_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C4_INTFLAG           (SAM_SERCOM4_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C4_STATUS            (SAM_SERCOM4_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C4_SYNCBUSY        (SAM_SERCOM4_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C4_ADDR              (SAM_SERCOM4_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C4_DATA              (SAM_SERCOM4_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C4_DBGCTRL           (SAM_SERCOM4_BASE+SAM_I2C_DBGCTRL_OFFSET)

#define SAM_I2C5_CTRLA             (SAM_SERCOM5_BASE+SAM_I2C_CTRLA_OFFSET)
#define SAM_I2C5_CTRLB             (SAM_SERCOM5_BASE+SAM_I2C_CTRLB_OFFSET)
#define SAM_I2C5_BAUD              (SAM_SERCOM5_BASE+SAM_I2C_BAUD_OFFSET)
#define SAM_I2C5_INTENCLR          (SAM_SERCOM5_BASE+SAM_I2C_INTENCLR_OFFSET)
#define SAM_I2C5_INTENSET          (SAM_SERCOM5_BASE+SAM_I2C_INTENSET_OFFSET)
#define SAM_I2C5_INTFLAG           (SAM_SERCOM5_BASE+SAM_I2C_INTFLAG_OFFSET)
#define SAM_I2C5_STATUS            (SAM_SERCOM5_BASE+SAM_I2C_STATUS_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_I2C5_SYNCBUSY        (SAM_SERCOM5_BASE+SAM_I2C_SYNCBUSY_OFFSET)
#endif

#define SAM_I2C5_ADDR              (SAM_SERCOM5_BASE+SAM_I2C_ADDR_OFFSET)
#define SAM_I2C5_DATA              (SAM_SERCOM5_BASE+SAM_I2C_DATA_OFFSET)
#define SAM_I2C5_DBGCTRL           (SAM_SERCOM5_BASE+SAM_I2C_DBGCTRL_OFFSET)

/* I2C register bit definitions *********************************************/

/* Control A register */

#define I2C_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define I2C_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define I2C_CTRLA_MODE_SHIFT       (2)       /* Bits 2-4: Operating Mode */
#define I2C_CTRLA_MODE_MASK        (7 << I2C_CTRLA_MODE_SHIFT)
#  define I2C_CTRLA_MODE_MASTER    (5 << I2C_CTRLA_MODE_SHIFT) /* I2C master mode */

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
#  define I2C_CTRLA_MEXTTOEN       (1 << 22) /* Bit 22: Master SCL low extend time-out */
#  define I2C_CTRLA_SEXTTOEN       (1 << 23) /* Bit 23: Slave SCL low extend time-out */
#  define I2C_CTRLA_SPEED_SHIFT    (24)      /* Bits 24-25:  Transfer speed */
#  define I2C_CTRLA_SPEED_MASK     (3 << I2C_CTRLA_SPEED_SHIFT)
#    define I2C_CTRLA_SPEED_STANDARD  (0 << I2C_CTRLA_SPEED_SHIFT) /* Standard-mode (<=100 kHz) and Fast-mode (<=400 kHz) */
#    define I2C_CTRLA_SPEED_FAST      (1 << I2C_CTRLA_SPEED_SHIFT) /* Fast-mode Plus (<=1 MHz) */
#    define I2C_CTRLA_SPEED_HIGHSPEED (2 << I2C_CTRLA_SPEED_SHIFT) /* High-speed mode (<=3.4 MHz) */

#  define I2C_CTRLA_SCLSM          (1 << 27) /* Bit 27: SCL clock stretch mode */
#endif

#define I2C_CTRLA_INACTOUT_SHIFT   (28)      /* Bits 28-29: Inactive Time-Out */
#define I2C_CTRLA_INACTOUT_MASK    (7 << I2C_CTRLA_INACTOUT_SHIFT)
#  define I2C_CTRLA_INACTOUT_DIS   (0 << I2C_CTRLA_INACTOUT_SHIFT) /* Disabled */
#  define I2C_CTRLA_INACTOUT_55US  (1 << I2C_CTRLA_INACTOUT_SHIFT) /* 5-6 SCL cycle time-out (50-60µs) */
#  define I2C_CTRLA_INACTOUT_105US (2 << I2C_CTRLA_INACTOUT_SHIFT) /* 10-11 SCL cycle time-out (100-110µs) */
#  define I2C_CTRLA_INACTOUT_205US (3 << I2C_CTRLA_INACTOUT_SHIFT) /* 20-21 SCL cycle time-out (200-210µs) */

#define I2C_CTRLA_LOWTOUT          (1 << 30)  /* Bit 30: SCL Low Time-Out */

/* Control B register */

#define I2C_CTRLB_SMEN             (1 << 8)  /* Bit 8:  Smart Mode Enable */
#define I2C_CTRLB_QCEN             (1 << 9)  /* Bit 9:  Quick Command Enable */
#define I2C_CTRLB_CMD_SHIFT        (16)      /* Bits 16-17: Command */
#define I2C_CTRLB_CMD_MASK         (3 << I2C_CTRLB_CMD_SHIFT)
#  define I2C_CTRLB_CMD_NOACTION   (0 << I2C_CTRLB_CMD_SHIFT) /* No action */
#  define I2C_CTRLB_CMD_ACKREP     (1 << I2C_CTRLB_CMD_SHIFT) /* ACK followed by repeated START */
#  define I2C_CTRLB_CMD_ACKREAD    (2 << I2C_CTRLB_CMD_SHIFT) /* ACK followed by read operation */
#  define I2C_CTRLB_CMD_ACKSTOP    (3 << I2C_CTRLB_CMD_SHIFT) /* ACK followed by STOP */

#define I2C_CTRLB_ACKACT           (1 << 18) /* Bit 18: Acknowledge Action */

#  define I2C_CTRLB_ACK            (0)              /* Send ACK */
#  define I2C_CTRLB_NACK           I2C_CTRLB_ACKACT /* Send NACK */

/* Baud register (16-bit baud value) */

#define I2C_BAUD_SHIFT             (0)       /* Bits 0-7: Master baud rate */
#define I2C_BAUD_MASK              (0xff << I2C_BAUD_SHIFT)
#  define I2C_BAUD(n)              ((uint16_t)(n) << I2C_BAUD_SHIFT)
#define I2C_BAUDLOW_SHIFT          (8)       /* Bits 8-15: Master baud rate low */
#define I2C_BAUDLOW_MASK           (0xff << I2C_BAUDLOW_SHIFT)
#  define I2C_BAUDLOW(n)           (uint16_t)(n) << I2C_BAUDLOW_SHIFT

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_HSBAUD_SHIFT         (16)      /* Bits 16-23: High speed master baud rate */
#  define I2C_HSBAUD_MASK          (0xff << I2C_HSBAUD_SHIFT)
#    define I2C_HSBAUD(n)          ((uint16)(n) << I2C_HSBAUD_SHIFT)
#  define I2C_HSBAUDLOW_SHIFT      (24)      /* Bits 24-31: High speed master baud rate low */
#  define I2C_HSBAUDLOW_MASK       (0xff << I2C_HSBAUDLOW_SHIFT)
#    define I2C_HSBAUDLOW(n)       (uint16)(n) << I2C_HSBAUDLOW_SHIFT
#endif

/* Interrupt enable clear, interrupt enable set, interrupt enable set,
 * interrupt flag and status clear registers.
 */

#define I2C_INT_MB                 (1 << 0)  /* Bit 0:  Master on bus interrupt */
#define I2C_INT_SB                 (1 << 1)  /* Bit 1:  Slave on bus interrupt */

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  define I2C_INT_ALL              (0x03)
#elif defined(CONFIG_ARCH_FAMILY_SAMD21)
#  define I2C_INT_ERROR            (1 << 7)  /* Bit 7:  Error interrupt */
#  define I2C_INT_ALL              (0x03)
#endif

/* Status register */

#define I2C_STATUS_BUSERR          (1 << 0)  /* Bit 0:  Bus Error */
#define I2C_STATUS_ARBLOST         (1 << 1)  /* Bit 1:  Arbitration Lost */
#define I2C_STATUS_RXNACK          (1 << 2)  /* Bit 2:  Received Not Acknowledge */
#define I2C_STATUS_BUSSTATE_SHIFT  (4)       /* Bits 4-5:  Bus State */
#define I2C_STATUS_BUSSTATE_MASK   (3 << I2C_STATUS_BUSSTATE_SHIFT)
#  define I2C_STATUS_BUSSTATE_UNKNOWN (0 << I2C_STATUS_BUSSTATE_SHIFT) /* Unknown to master */
#  define I2C_STATUS_BUSSTATE_IDLE    (1 << I2C_STATUS_BUSSTATE_SHIFT) /* Waiting for transaction */
#  define I2C_STATUS_BUSSTATE_OWNER   (2 << I2C_STATUS_BUSSTATE_SHIFT) /* Master of bus owner */
#  define I2C_STATUS_BUSSTATE_BUSY    (3 << I2C_STATUS_BUSSTATE_SHIFT) /* Other master owns */

#define I2C_STATUS_LOWTOUT         (1 << 6)  /* Bit 6:  SCL Low Time-Out */
#define I2C_STATUS_CLKHOLD         (1 << 7)  /* Bit 7:  Clock Hold */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_STATUS_MEXTTOUT      (1 << 8)  /* Bit 8:  Master SCL low extend time-out */
#  define I2C_STATUS_SEXTTOUT      (1 << 9)  /* Bit 9:  Slave SCL low extend time-out */
#  define I2C_STATUS_LENERR        (1 << 10) /* Bit 10: Transaction length error */
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define I2C_STATUS_SYNCBUSY      (1 << 15) /* Bit 15: Synchronization busy */
#endif

/* Synchronization busy register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_SYNCBUSY_SWRST       (1 << 0)  /* Bit 0: Software reset synchronization busy */
#  define I2C_SYNCBUSY_ENABLE      (1 << 1)  /* Bit 1: SERCOM enable synchronization busy */
#  define I2C_SYNCBUSY_SYSOP       (1 << 2)  /* Bit 2: System operation synchronization busy */

#  define I2C_SYNCBUSY_ALL         0x0007
#endif

/* Address register */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define I2C_ADDR_SHIFT           (0)       /* Bits 0-7: Address */
#  define I2C_ADDR_MASK            (0xff << I2C_ADDR_SHIFT)
#    define I2C_ADDR(n)            ((uint16_t)(n) << I2C_ADDR_SHIFT)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define I2C_ADDR_SHIFT           (0)       /* Bits 0-10: Address */
#  define I2C_ADDR_MASK            (0x7ff << I2C_ADDR_SHIFT)
#    define I2C_ADDR(n)            ((uint32_t)(n) << I2C_ADDR_SHIFT)
#  define I2C_ADDR_LENEN           (1 << 13) /* Bit 13: Transfer length enable */
#  define I2C_ADDR_HS              (1 << 14) /* Bit 14: High speed */
#  define I2C_ADDR_TENBITEN        (1 << 15) /* Bit 15: Ten Bit Addressing Enable */
#  define I2C_ADDR_LEN_SHIFT       (16)      /* Bits 16-23: Transaction Length */
#  define I2C_ADDR_LEN_MASK        (0xff << I2C_ADDR_LEN_SHIFT)
#    define I2C_ADDR_LEN(n)        ((uint32_t)(n) << I2C_ADDR_LEN_SHIFT)
#endif

/* Data register */

#define I2C_DATA_MASK              (0x00ff)  /* Bits 0-7: Data */

/* Debug control register */

#define I2C_DBGCTRL_DBGSTOP        (1 << 0)  /* Bit 0: Debug stop mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_I2C_MASTER_H */
