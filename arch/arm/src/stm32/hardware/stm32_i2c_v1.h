/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_i2c_v1.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_V1_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_V1_H

/* This file provide definitions for the STM32 I2C IP core 1
 *(F1, F2, F4 and L1)
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_I2C_CR1_OFFSET    0x0000  /* Control register 1 (16-bit) */
#define STM32_I2C_CR2_OFFSET    0x0004  /* Control register 2 (16-bit) */
#define STM32_I2C_OAR1_OFFSET   0x0008  /* Own address register 1 (16-bit) */
#define STM32_I2C_OAR2_OFFSET   0x000c  /* Own address register 2 (16-bit) */
#define STM32_I2C_DR_OFFSET     0x0010  /* Data register (16-bit) */
#define STM32_I2C_SR1_OFFSET    0x0014  /* Status register 1 (16-bit) */
#define STM32_I2C_SR2_OFFSET    0x0018  /* Status register 2 (16-bit) */
#define STM32_I2C_CCR_OFFSET    0x001c  /* Clock control register (16-bit) */
#define STM32_I2C_TRISE_OFFSET  0x0020  /* TRISE Register (16-bit) */
#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
    defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F412)
#  define STM32_I2C_FLTR_OFFSET   0x0024  /* FLTR Register (16-bit) */
#endif

/* Register Addresses *******************************************************/

#if STM32_NI2C > 0
#  define STM32_I2C1_CR1        (STM32_I2C1_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C1_CR2        (STM32_I2C1_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C1_OAR1       (STM32_I2C1_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C1_OAR2       (STM32_I2C1_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C1_DR         (STM32_I2C1_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C1_SR1        (STM32_I2C1_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C1_SR2        (STM32_I2C1_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C1_CCR        (STM32_I2C1_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C1_TRISE      (STM32_I2C1_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C1_FLTR     (STM32_I2C1_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

#if STM32_NI2C > 1
#  define STM32_I2C2_CR1        (STM32_I2C2_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C2_CR2        (STM32_I2C2_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C2_OAR1       (STM32_I2C2_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C2_OAR2       (STM32_I2C2_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C2_DR         (STM32_I2C2_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C2_SR1        (STM32_I2C2_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C2_SR2        (STM32_I2C2_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C2_CCR        (STM32_I2C2_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C2_TRISE      (STM32_I2C2_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C2_FLTR     (STM32_I2C2_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

#if STM32_NI2C > 2
#  define STM32_I2C3_CR1        (STM32_I2C3_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C3_CR2        (STM32_I2C3_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C3_OAR1       (STM32_I2C3_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C3_OAR2       (STM32_I2C3_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C3_DR         (STM32_I2C3_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C3_SR1        (STM32_I2C3_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C3_SR2        (STM32_I2C3_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C3_CCR        (STM32_I2C3_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C3_TRISE      (STM32_I2C3_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C3_FLTR     (STM32_I2C3_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register 1 */

#define I2C_CR1_PE              (1 << 0)  /* Bit 0: Peripheral Enable */
#define I2C_CR1_SMBUS           (1 << 1)  /* Bit 1: SMBus Mode */
#define I2C_CR1_SMBTYPE         (1 << 3)  /* Bit 3: SMBus Type */
#define I2C_CR1_ENARP           (1 << 4)  /* Bit 4: ARP Enable */
#define I2C_CR1_ENPEC           (1 << 5)  /* Bit 5: PEC Enable */
#define I2C_CR1_ENGC            (1 << 6)  /* Bit 6: General Call Enable */
#define I2C_CR1_NOSTRETCH       (1 << 7)  /* Bit 7: Clock Stretching Disable (Slave mode) */
#define I2C_CR1_START           (1 << 8)  /* Bit 8: Start Generation */
#define I2C_CR1_STOP            (1 << 9)  /* Bit 9: Stop Generation */
#define I2C_CR1_ACK             (1 << 10) /* Bit 10: Acknowledge Enable */
#define I2C_CR1_POS             (1 << 11) /* Bit 11: Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC             (1 << 12) /* Bit 12: Packet Error Checking */
#define I2C_CR1_ALERT           (1 << 13) /* Bit 13: SMBus Alert */
#define I2C_CR1_SWRST           (1 << 15) /* Bit 15: Software Reset */

/* Control register 2 */

#define I2C_CR2_FREQ_SHIFT      (0)       /* Bits 5-0: Peripheral Clock Frequency */
#define I2C_CR2_FREQ_MASK       (0x3f << I2C_CR2_FREQ_SHIFT)
#define I2C_CR2_ITERREN         (1 << 8)  /* Bit 8: Error Interrupt Enable */
#define I2C_CR2_ITEVFEN         (1 << 9)  /* Bit 9: Event Interrupt Enable */
#define I2C_CR2_ITBUFEN         (1 << 10) /* Bit 10: Buffer Interrupt Enable */
#define I2C_CR2_DMAEN           (1 << 11) /* Bit 11: DMA Requests Enable */
#define I2C_CR2_LAST            (1 << 12) /* Bit 12: DMA Last Transfer */

#define I2C_CR2_ALLINTS         (I2C_CR2_ITERREN|I2C_CR2_ITEVFEN|I2C_CR2_ITBUFEN)

/* Own address register 1 */

#define I2C_OAR1_ADD0           (1 << 0)  /* Bit 0: Interface Address */
#define I2C_OAR1_ADD8_SHIFT     (1)       /* Bits 7-1: Interface Address */
#define I2C_OAR1_ADD8_MASK      (0x007f << I2C_OAR1_ADD8_SHIFT)
#define I2C_OAR1_ADD10_SHIFT    (1)       /* Bits 9-1: Interface Address (10-bit addressing mode)*/
#define I2C_OAR1_ADD10_MASK     (0x01ff << I2C_OAR1_ADD10_SHIFT)
#define I2C_OAR1_ONE            (1 << 14) /* Bit 14: Must be configured and kept at 1 */
#define I2C_OAR1_ADDMODE        (1 << 15) /* Bit 15: Addressing Mode (Slave mode) */

/* Own address register 2 */

#define I2C_OAR2_ENDUAL         (1 << 0)  /* Bit 0: Dual addressing mode enable */
#define I2C_OAR2_ADD2_SHIFT     (1)       /* Bits 7-1: Interface address */
#define I2C_OAR2_ADD2_MASK      (0x7f << I2C_OAR2_ADD2_SHIFT)

/* Data register */

#define I2C_DR_SHIFT            (0)       /* Bits 7-0: 8-bit Data Register */
#define I2C_DR_MASK             (0x00ff << I2C_DR_SHIFT)

/* Status register 1 */

#define I2C_SR1_SB              (1 << 0)  /* Bit 0: Start Bit (Master mode) */
#define I2C_SR1_ADDR            (1 << 1)  /* Bit 1: Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF             (1 << 2)  /* Bit 2: Byte Transfer Finished */
#define I2C_SR1_ADD10           (1 << 3)  /* Bit 3: 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF           (1 << 4)  /* Bit 4: Stop detection (Slave mode) */
                                          /* Bit 5: Reserved */
#define I2C_SR1_RXNE            (1 << 6)  /* Bit 6: Data Register not Empty (receivers) */
#define I2C_SR1_TXE             (1 << 7)  /* Bit 7: Data Register Empty (transmitters) */
#define I2C_SR1_BERR            (1 << 8)  /* Bit 8: Bus Error */
#define I2C_SR1_ARLO            (1 << 9)  /* Bit 9: Arbitration Lost (master mode) */
#define I2C_SR1_AF              (1 << 10) /* Bit 10: Acknowledge Failure */
#define I2C_SR1_OVR             (1 << 11) /* Bit 11: Overrun/Underrun */
#define I2C_SR1_PECERR          (1 << 12) /* Bit 12: PEC Error in reception */
                                          /* Bit 13: Reserved */
#define I2C_SR1_TIMEOUT         (1 << 14) /* Bit 14: Timeout or Tlow Error */
#define I2C_SR1_SMBALERT        (1 << 15) /* Bit 15: SMBus Alert */

#define I2C_SR1_ERRORMASK       (I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_AF|I2C_SR1_OVR|\
                                 I2C_SR1_PECERR|I2C_SR1_TIMEOUT|I2C_SR1_SMBALERT)

/* Status register 2 */

#define I2C_SR2_MSL             (1 << 0)  /* Bit 0: Master/Slave */
#define I2C_SR2_BUSY            (1 << 1)  /* Bit 1: Bus Busy */
#define I2C_SR2_TRA             (1 << 2)  /* Bit 2: Transmitter/Receiver */
#define I2C_SR2_GENCALL         (1 << 4)  /* Bit 4: General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT      (1 << 5)  /* Bit 5: SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST         (1 << 6)  /* Bit 6: SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF           (1 << 7)  /* Bit 7: Dual Flag (Slave mode) */
#define I2C_SR2_PEC_SHIFT       (8)       /* Bits 15-8: Packet Error Checking Register */
#define I2C_SR2_PEC_MASK        (0xff << I2C_SR2_PEC_SHIFT)

/* Clock control register */

#define I2C_CCR_CCR_SHIFT       (0)       /* Bits 11-0: Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_CCR_MASK        (0x0fff << I2C_CCR_CCR_SHIFT)
#define I2C_CCR_DUTY            (1 << 14) /* Bit 14: Fast Mode Duty Cycle */
#define I2C_CCR_FS              (1 << 15) /* Bit 15: Fast Mode Selection */

/* TRISE Register */

#define I2C_TRISE_SHIFT         (0) /* Bits 5-0: Maximum Rise Time in Fast/Standard mode (Master mode) */
#define I2C_TRISE_MASK          (0x3f << I2C_TRISE_SHIFT)

/* FLTR Register */

#ifdef STM32_I2C_FLTR_OFFSET
#  define I2C_FLTR_ANOFF        (1 << 4)  /* Bit 4: Analog noise filter disable */
#  define I2C_FLTR_DNF_SHIFT    0         /* Bits 0-3: Digital noise filter */
#  define I2C_FLTR_DNF_MASK     (0xf << I2C_FLTR_DNF_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_V1_H */
