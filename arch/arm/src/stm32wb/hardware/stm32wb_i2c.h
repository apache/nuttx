/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_i2c.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_I2C_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_I2C_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_I2C_CR1_OFFSET      0x0000 /* Control register 1 (32-bit) */
#define STM32WB_I2C_CR2_OFFSET      0x0004 /* Control register 2 (32-bit) */
#define STM32WB_I2C_OAR1_OFFSET     0x0008 /* Own address register 1 (16-bit) */
#define STM32WB_I2C_OAR2_OFFSET     0x000c /* Own address register 2 (16-bit) */
#define STM32WB_I2C_TIMINGR_OFFSET  0x0010 /* Timing register */
#define STM32WB_I2C_TIMEOUTR_OFFSET 0x0014 /* Timeout register */
#define STM32WB_I2C_ISR_OFFSET      0x0018 /* Interrupt and Status register */
#define STM32WB_I2C_ICR_OFFSET      0x001c /* Interrupt clear register */
#define STM32WB_I2C_PECR_OFFSET     0x0020 /* Packet error checking register */
#define STM32WB_I2C_RXDR_OFFSET     0x0024 /* Receive data register */
#define STM32WB_I2C_TXDR_OFFSET     0x0028 /* Transmit data register */

/* Register Addresses *******************************************************/

#define STM32WB_I2C1_CR1            (STM32WB_I2C1_BASE + STM32WB_I2C_CR1_OFFSET)
#define STM32WB_I2C1_CR2            (STM32WB_I2C1_BASE + STM32WB_I2C_CR2_OFFSET)
#define STM32WB_I2C1_OAR1           (STM32WB_I2C1_BASE + STM32WB_I2C_OAR1_OFFSET)
#define STM32WB_I2C1_OAR2           (STM32WB_I2C1_BASE + STM32WB_I2C_OAR2_OFFSET)
#define STM32WB_I2C1_TIMINGR        (STM32WB_I2C1_BASE + STM32WB_I2C_TIMINGR_OFFSET)
#define STM32WB_I2C1_TIMEOUTR       (STM32WB_I2C1_BASE + STM32WB_I2C_TIMEOUTR_OFFSET)
#define STM32WB_I2C1_ISR            (STM32WB_I2C1_BASE + STM32WB_I2C_ISR_OFFSET)
#define STM32WB_I2C1_ICR            (STM32WB_I2C1_BASE + STM32WB_I2C_ICR_OFFSET)
#define STM32WB_I2C1_PECR           (STM32WB_I2C1_BASE + STM32WB_I2C_PECR_OFFSET)
#define STM32WB_I2C1_RXDR           (STM32WB_I2C1_BASE + STM32WB_I2C_RXDR_OFFSET)
#define STM32WB_I2C1_TXDR           (STM32WB_I2C1_BASE + STM32WB_I2C_TXDR_OFFSET)

#ifdef CONFIG_STM32WB_HAVE_I2C3
#  define STM32WB_I2C3_CR1          (STM32WB_I2C3_BASE + STM32WB_I2C_CR1_OFFSET)
#  define STM32WB_I2C3_CR2          (STM32WB_I2C3_BASE + STM32WB_I2C_CR2_OFFSET)
#  define STM32WB_I2C3_OAR1         (STM32WB_I2C3_BASE + STM32WB_I2C_OAR1_OFFSET)
#  define STM32WB_I2C3_OAR2         (STM32WB_I2C3_BASE + STM32WB_I2C_OAR2_OFFSET)
#  define STM32WB_I2C3_TIMINGR      (STM32WB_I2C3_BASE + STM32WB_I2C_TIMINGR_OFFSET)
#  define STM32WB_I2C3_TIMEOUTR     (STM32WB_I2C3_BASE + STM32WB_I2C_TIMEOUTR_OFFSET)
#  define STM32WB_I2C3_ISR          (STM32WB_I2C3_BASE + STM32WB_I2C_ISR_OFFSET)
#  define STM32WB_I2C3_ICR          (STM32WB_I2C3_BASE + STM32WB_I2C_ICR_OFFSET)
#  define STM32WB_I2C3_PECR         (STM32WB_I2C3_BASE + STM32WB_I2C_PECR_OFFSET)
#  define STM32WB_I2C3_RXDR         (STM32WB_I2C3_BASE + STM32WB_I2C_RXDR_OFFSET)
#  define STM32WB_I2C3_TXDR         (STM32WB_I2C3_BASE + STM32WB_I2C_TXDR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register 1 */

#define I2C_CR1_PE                (1 << 0)  /* Bit 0:  Peripheral Enable */
#define I2C_CR1_TXIE              (1 << 1)  /* Bit 1:  TX Interrupt enable */
#define I2C_CR1_RXIE              (1 << 2)  /* Bit 2:  RX Interrupt enable */
#define I2C_CR1_ADDRIE            (1 << 3)  /* Bit 3:  Address match interrupt enable (slave) */
#define I2C_CR1_NACKIE            (1 << 4)  /* Bit 4:  Not acknowledge received interrupt enable */
#define I2C_CR1_STOPIE            (1 << 5)  /* Bit 5:  STOP detection interrupt enable */
#define I2C_CR1_TCIE              (1 << 6)  /* Bit 6:  Transfer Complete interrupt enable */
#define I2C_CR1_ERRIE             (1 << 7)  /* Bit 7:  Error interrupts enable */
#define I2C_CR1_DNF_SHIFT         (8)       /* Bits 8-11: Digital noise filter */
#define I2C_CR1_DNF_MASK          (0xf << I2C_CR1_DNF_SHIFT)
#  define I2C_CR1_DNF_DISABLE     (0 << I2C_CR1_DNF_SHIFT)
#  define I2C_CR1_DNF(n)          ((n) << I2C_CR1_DNF_SHIFT) /* Up to n * Ti2cclk, n=1..15 */

#define I2C_CR1_ANFOFF            (1 << 12) /* Bit 12: Analog noise filter OFF */
#define I2C_CR1_TXDMAEN           (1 << 14) /* Bit 14: DMA transmission requests enable */
#define I2C_CR1_RXDMAEN           (1 << 15) /* Bit 15: DMA reception requests enable */
#define I2C_CR1_SBC               (1 << 16) /* Bit 16: Slave byte control */
#define I2C_CR1_NOSTRETCH         (1 << 17) /* Bit 17: Clock stretching disable */
#define I2C_CR1_WUPEN             (1 << 18) /* Bit 18: Wakeup from STOP enable */
#define I2C_CR1_GCEN              (1 << 19) /* Bit 19: General call enable */
#define I2C_CR1_SMBHEN            (1 << 20) /* Bit 20: SMBus Host address enable */
#define I2C_CR1_SMBDEN            (1 << 21) /* Bit 21: SMBus Device Default address enable */
#define I2C_CR1_ALERTEN           (1 << 22) /* Bit 22: SMBus alert enable */
#define I2C_CR1_PECEN             (1 << 23) /* Bit 23: PEC enable */

/* Control register 2 */

#define I2C_CR2_SADD10_SHIFT      (0)       /* Bits 0-9: Slave 10-bit address (master) */
#define I2C_CR2_SADD10_MASK       (0x3ff << I2C_CR2_SADD10_SHIFT)
#define I2C_CR2_SADD7_SHIFT       (1)       /* Bits 1-7: Slave 7-bit address (master) */
#define I2C_CR2_SADD7_MASK        (0x7f << I2C_CR2_SADD7_SHIFT)
#define I2C_CR2_RD_WRN            (1 << 10) /* Bit 10: Transfer direction (master) */
#define I2C_CR2_ADD10             (1 << 11) /* Bit 11: 10-bit addressing mode (master) */
#define I2C_CR2_HEAD10R           (1 << 12) /* Bit 12: 10-bit address header only read direction (master) */
#define I2C_CR2_START             (1 << 13) /* Bit 13: Start generation */
#define I2C_CR2_STOP              (1 << 14) /* Bit 14: Stop generation (master) */
#define I2C_CR2_NACK              (1 << 15) /* Bit 15: NACK generation (slave) */
#define I2C_CR2_NBYTES_SHIFT      (16)      /* Bits 16-23: Number of bytes */
#define I2C_CR2_NBYTES_MASK       (0xff << I2C_CR2_NBYTES_SHIFT)
#define I2C_CR2_RELOAD            (1 << 24) /* Bit 24: NBYTES reload mode */
#define I2C_CR2_AUTOEND           (1 << 25) /* Bit 25: Automatic end mode (master) */
#define I2C_CR2_PECBYTE           (1 << 26) /* Bit 26: Packet error checking byte */

/* Own address register 1 */

#define I2C_OAR1_OA1_10_SHIFT     (0)       /* Bits 0-9: 10-bit interface address */
#define I2C_OAR1_OA1_10_MASK      (0x3ff << I2C_OAR1_OA1_10_SHIFT)
#define I2C_OAR1_OA1_7_SHIFT      (1)       /* Bits 1-7: 7-bit interface address */
#define I2C_OAR1_OA1_7_MASK       (0x7f << I2C_OAR1_OA1_7_SHIFT)
#define I2C_OAR1_OA1MODE          (1 << 10) /* Bit 10: Own Address 1 10-bit mode */
#define I2C_OAR1_OA1EN            (1 << 15) /* Bit 15: Own Address 1 enable */

/* Own address register 2 */

#define I2C_OAR2_OA2_SHIFT        (1)       /* Bits 1-7: 7-bit interface address */
#define I2C_OAR2_OA2_MASK         (0x7f << I2C_OAR2_OA2_SHIFT)
#define I2C_OAR2_OA2MSK_SHIFT     (8)       /* Bits 8-10: Own Address 2 masks */
#define I2C_OAR2_OA2MSK_MASK      (0x7 << I2C_OAR2_OA2MSK_SHIFT)
#  define I2C_OAR2_OA2MSK_NONE    (0x0 << I2C_OAR2_OA2MSK_SHIFT) /* No mask */
#  define I2C_OAR2_OA2MSK_2_7     (0x1 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7:2] are compared */
#  define I2C_OAR2_OA2MSK_3_7     (0x2 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7:3] are compared */
#  define I2C_OAR2_OA2MSK_4_7     (0x3 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7:4] are compared */
#  define I2C_OAR2_OA2MSK_5_7     (0x4 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7:5] are compared */
#  define I2C_OAR2_OA2MSK_6_7     (0x5 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7:6] are compared */
#  define I2C_OAR2_OA2MSK_7       (0x6 << I2C_OAR2_OA2MSK_SHIFT) /* Only OA2[7] is compared */
#  define I2C_OAR2_OA2MSK_ALL     (0x7 << I2C_OAR2_OA2MSK_SHIFT) /* All 7-bit addresses acknowledged */

#define I2C_OAR2_OA2EN            (1 << 15) /* Bit 15: Own Address 2 enable */

/* Timing register */

#define I2C_TIMINGR_SCLL_SHIFT    (0)       /* Bits 0-7: SCL low period (master) */
#define I2C_TIMINGR_SCLL_MASK     (0xff << I2C_TIMINGR_SCLL_SHIFT)
#  define I2C_TIMINGR_SCLL(n)     (((n)-1) << I2C_TIMINGR_SCLL_SHIFT) /* tSCLL = n x tPRESC */

#define I2C_TIMINGR_SCLH_SHIFT    (8)       /* Bits 8-15: SCL high period (master) */
#define I2C_TIMINGR_SCLH_MASK     (0xff << I2C_TIMINGR_SCLH_SHIFT)
#  define I2C_TIMINGR_SCLH(n)     (((n)-1) << I2C_TIMINGR_SCLH_SHIFT) /* tSCLH = n x tPRESC */

#define I2C_TIMINGR_SDADEL_SHIFT  (16)      /* Bits 16-19: Data hold time */
#define I2C_TIMINGR_SDADEL_MASK   (0xf << I2C_TIMINGR_SDADEL_SHIFT)
#  define I2C_TIMINGR_SDADEL(n)   ((n) << I2C_TIMINGR_SDADEL_SHIFT) /* tSDADEL= n x tPRESC */

#define I2C_TIMINGR_SCLDEL_SHIFT  (20)      /* Bits 20-23: Data setup time */
#define I2C_TIMINGR_SCLDEL_MASK   (0xf << I2C_TIMINGR_SCLDEL_SHIFT)
#  define I2C_TIMINGR_SCLDEL(n)   (((n)-1) << I2C_TIMINGR_SCLDEL_SHIFT) /* tSCLDEL = n x tPRESC */

#define I2C_TIMINGR_PRESC_SHIFT   (28)      /* Bits 28-31: Timing prescaler */
#define I2C_TIMINGR_PRESC_MASK    (15 << I2C_TIMINGR_PRESC_SHIFT)
#  define I2C_TIMINGR_PRESC(n)    (((n)-1) << I2C_TIMINGR_PRESC_SHIFT) /* tPRESC = n x tI2CCLK */

/* Timeout register */

#define I2C_TIMEOUTR_A_SHIFT      (0)       /* Bits 0-11: Bus Timeout A */
#define I2C_TIMEOUTR_A_MASK       (0xfff << I2C_TIMEOUTR_A_SHIFT)
#  define I2C_TIMEOUTR_A(n)       ((n) << I2C_TIMEOUTR_A_SHIFT)
#define I2C_TIMEOUTR_TIDLE        (1 << 12) /* Bit 12: Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN     (1 << 15) /* Bit 15: Clock timeout enable */
#define I2C_TIMEOUTR_B_SHIFT      (16)      /* Bits 16-27: Bus Timeout B */
#define I2C_TIMEOUTR_B_MASK       (0xfff << I2C_TIMEOUTR_B_SHIFT)
#  define I2C_TIMEOUTR_B(n)       ((n) << I2C_TIMEOUTR_B_SHIFT)
#define I2C_TIMEOUTR_TEXTEN       (1 << 31) /* Bits 31: Extended clock timeout enable */

/* Interrupt and Status register and interrupt clear register */

/* Common interrupt bits */

#define I2C_INT_ADDR              (1 << 3)  /* Bit 3:  Address matched (slave) */
#define I2C_INT_NACK              (1 << 4)  /* Bit 4:  Not Acknowledge received flag */
#define I2C_INT_STOP              (1 << 5)  /* Bit 5:  Stop detection flag */
#define I2C_INT_BERR              (1 << 8)  /* Bit 8:  Bus error */
#define I2C_INT_ARLO              (1 << 9)  /* Bit 9:  Arbitration lost */
#define I2C_INT_OVR               (1 << 10) /* Bit 10: Overrun/Underrun (slave) */
#define I2C_INT_PECERR            (1 << 11) /* Bit 11: PEC Error in reception */
#define I2C_INT_TIMEOUT           (1 << 12) /* Bit 12: Timeout or tLOW detection flag */
#define I2C_INT_ALERT             (1 << 13) /* Bit 13: SMBus alert */

/* Fields unique to the Interrupt and Status register */

#define I2C_ISR_TXE               (1 << 0)  /* Bit 0:  Transmit data register empty (transmitters) */
#define I2C_ISR_TXIS              (1 << 1)  /* Bit 1:  Transmit interrupt status (transmitters) */
#define I2C_ISR_RXNE              (1 << 2)  /* Bit 2:  Receive data register not empty (receivers) */
#define I2C_ISR_TC                (1 << 6)  /* Bit 6:  Transfer Complete (master) */
#define I2C_ISR_TCR               (1 << 7)  /* Bit 7:  Transfer Complete Reload */
#define I2C_ISR_BUSY              (1 << 15) /* Bit 15: Bus busy */
#define I2C_ISR_DIR               (1 << 16) /* Bit 16: Transfer direction (slave) */
#define I2C_ISR_ADDCODE_SHIFT     (17)      /* Bits 17-23: Address match code (slave) */
#define I2C_ISR_ADDCODE_MASK      (0x7f << I2C_ISR_ADDCODE_SHIFT)

#define I2C_ISR_ERRORMASK (I2C_INT_BERR | I2C_INT_ARLO | I2C_INT_OVR | I2C_INT_PECERR | I2C_INT_TIMEOUT)

#define I2C_ICR_CLEARMASK (I2C_INT_ADDR | I2C_INT_NACK | I2C_INT_STOP | I2C_INT_BERR | I2C_INT_ARLO \
                           | I2C_INT_OVR | I2C_INT_PECERR | I2C_INT_TIMEOUT | I2C_INT_ALERT)

/* Packet error checking register */

#define I2C_PECR_MASK             (0xff)

/* Receive data register */

#define I2C_RXDR_MASK             (0xff)

/* Transmit data register */

#define I2C_TXDR_MASK             (0xff)

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_I2C_H */
