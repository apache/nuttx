/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs250t_484_pinmap.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS250T_484_PINMAP_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS250T_484_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IO Mux setting for each IO pad */

#define MSSIO_AF_SDIO        (0x0 << GPIO_AF_SHIFT)
#define MSSIO_AF_EMMC        (0x1 << GPIO_AF_SHIFT)
#define MSSIO_AF_QSPI        (0x2 << GPIO_AF_SHIFT)
#define MSSIO_AF_SPI         (0x3 << GPIO_AF_SHIFT)
#define MSSIO_AF_USB         (0x4 << GPIO_AF_SHIFT)
#define MSSIO_AF_MMUART      (0x5 << GPIO_AF_SHIFT)
#define MSSIO_AF_I2C         (0x6 << GPIO_AF_SHIFT)
#define MSSIO_AF_CAN         (0x7 << GPIO_AF_SHIFT)
#define MSSIO_AF_MDIO        (0x8 << GPIO_AF_SHIFT)
#define MSSIO_AF_MISC        (0x9 << GPIO_AF_SHIFT)
#define MSSIO_AF_RSVD        (0xA << GPIO_AF_SHIFT)
#define MSSIO_AF_GPIO        (0xB << GPIO_AF_SHIFT)
#define MSSIO_AF_FABRIC_TEST (0xC << GPIO_AF_SHIFT)
#define MSSIO_AF_LOW         (0xD << GPIO_AF_SHIFT)
#define MSSIO_AF_HIGH        (0xE << GPIO_AF_SHIFT)
#define MSSIO_AF_TRISTATE    (0xF << GPIO_AF_SHIFT)

/* Each 32-bit register has 2 16-bit configurations for consecutive pins */
#define MSSIO_IO_CFG_CR_SHIFT(pin) (pin & 1 ? 16 : 0)
#define MSSIO_IO_CFG_CR_MASK(pin) (0xFFFF << MSSIO_IO_CFG_CR_SHIFT(pin))

/* First offset register of the bank + (pin / 2) * 4 */
#define MSSIO_IO_CFG_BANK0_CR_OFFSET(pin) (0x00000234 + ((pin >> 1) * 4))
#define MSSIO_IO_CFG_BANK1_CR_OFFSET(pin) (0x00000254 + ((pin >> 1) * 4))

#define MSSIO_IO_CFG_CR(bank, pin) (MPFS_SYSREG_BASE +                         \
                                    (bank == 0 ?                               \
                                     MSSIO_IO_CFG_BANK0_CR_OFFSET(pin) :       \
                                     MSSIO_IO_CFG_BANK1_CR_OFFSET(pin)))

/* Each 32-bit register has 8 4-bit configurations for consecutive pins */
#define MSSIO_MUX_SHIFT(pin) ((pin & 7) * 4)
#define MSSIO_MUX_MASK(pin)  (0xF << MSSIO_MUX_SHIFT(pin))

/* First offset register of the bank + pin / 8 * 4 */
#define MSSIO_MUX_BANK0_REG_OFFSET(pin) (MPFS_SYSREG_IOMUX1_CR_OFFSET +        \
                                         (pin >> 3) * 4)
#define MSSIO_MUX_BANK1_REG_OFFSET(pin) (MPFS_SYSREG_IOMUX3_CR_OFFSET +        \
                                         (pin >> 3) * 4)

#define MSSIO_MUX_BANK_REG_OFFSET(bank,pin) (bank == 0 ?                       \
                                             MSSIO_MUX_BANK0_REG_OFFSET(pin) : \
                                             MSSIO_MUX_BANK1_REG_OFFSET(pin))
#define MSSIO_MUX_BANK_REG(bank,pin) (MPFS_SYSREG_BASE +                       \
                                      MSSIO_MUX_BANK_REG_OFFSET(bank,pin))

/* Default EC configuration for all GPIOS */
#define MSSIO_EC_DEFAULT (0x0428 << GPIO_EC_SHIFT)
#define MSSIO_EC_USB_DEFAULT (0x0829 << GPIO_EC_SHIFT)

/* Basic GPIO definitions for MSSIO */

#if defined(CONFIG_ARCH_CHIP_MPFS250T_FCVG484) || defined(CONFIG_ARCH_CHIP_MPFS250T_FCG484)

/* MSSIO GPIO BANK 0 */

#define MSSIO_GPIO_PAD0_J1  (GPIO_BANK0 | GPIO_PIN0  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD1_K5  (GPIO_BANK0 | GPIO_PIN1  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD2_H1  (GPIO_BANK0 | GPIO_PIN2  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD3_J4  (GPIO_BANK0 | GPIO_PIN3  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD4_K4  (GPIO_BANK0 | GPIO_PIN4  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD5_J7  (GPIO_BANK0 | GPIO_PIN5  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD6_K3  (GPIO_BANK0 | GPIO_PIN6  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD7_H4  (GPIO_BANK0 | GPIO_PIN7  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD8_J6  (GPIO_BANK0 | GPIO_PIN8  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD9_H6  (GPIO_BANK0 | GPIO_PIN9  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD10_J3 (GPIO_BANK0 | GPIO_PIN10 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD11_H2 (GPIO_BANK0 | GPIO_PIN11 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD12_H5 (GPIO_BANK0 | GPIO_PIN12 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD13_J2 (GPIO_BANK0 | GPIO_PIN13 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)

/* MSSIO GPIO BANK 1 */

#define MSSIO_GPIO_PAD14_G2 (GPIO_BANK1 | GPIO_PIN0  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD15_F1 (GPIO_BANK1 | GPIO_PIN1  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD16_G5 (GPIO_BANK1 | GPIO_PIN2  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD17_G4 (GPIO_BANK1 | GPIO_PIN3  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD18_F2 (GPIO_BANK1 | GPIO_PIN4  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD19_E1 (GPIO_BANK1 | GPIO_PIN5  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD20_G3 (GPIO_BANK1 | GPIO_PIN6  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD21_F5 (GPIO_BANK1 | GPIO_PIN7  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD22_D1 (GPIO_BANK1 | GPIO_PIN8  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD23_D2 (GPIO_BANK1 | GPIO_PIN9  | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD24_F6 (GPIO_BANK1 | GPIO_PIN10 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD25_F3 (GPIO_BANK1 | GPIO_PIN11 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD26_C1 (GPIO_BANK1 | GPIO_PIN12 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD27_B1 (GPIO_BANK1 | GPIO_PIN13 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD28_D3 (GPIO_BANK1 | GPIO_PIN14 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD29_C2 (GPIO_BANK1 | GPIO_PIN15 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD30_E5 (GPIO_BANK1 | GPIO_PIN16 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD31_E4 (GPIO_BANK1 | GPIO_PIN17 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD32_B2 (GPIO_BANK1 | GPIO_PIN18 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD33_A2 (GPIO_BANK1 | GPIO_PIN19 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD34_B3 (GPIO_BANK1 | GPIO_PIN20 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD35_A3 (GPIO_BANK1 | GPIO_PIN21 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD36_E3 (GPIO_BANK1 | GPIO_PIN22 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)
#define MSSIO_GPIO_PAD37_D4 (GPIO_BANK1 | GPIO_PIN23 | MSSIO_AF_GPIO | MSSIO_EC_DEFAULT)

/* USB pins */

#define MSSIO_USB_CLK   (GPIO_BANK1 | GPIO_PIN0  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DIR   (GPIO_BANK1 | GPIO_PIN1  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_NXT   (GPIO_BANK1 | GPIO_PIN2  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_STP   (GPIO_BANK1 | GPIO_PIN3  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA0 (GPIO_BANK1 | GPIO_PIN4  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA1 (GPIO_BANK1 | GPIO_PIN5  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA2 (GPIO_BANK1 | GPIO_PIN6  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA3 (GPIO_BANK1 | GPIO_PIN7  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA4 (GPIO_BANK1 | GPIO_PIN8  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA5 (GPIO_BANK1 | GPIO_PIN9  | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA6 (GPIO_BANK1 | GPIO_PIN10 | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)
#define MSSIO_USB_DATA7 (GPIO_BANK1 | GPIO_PIN11 | MSSIO_AF_USB | MSSIO_EC_USB_DEFAULT)

#endif

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS250T_484_PINMAP_H */
