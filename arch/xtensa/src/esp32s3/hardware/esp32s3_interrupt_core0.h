/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_interrupt_core0.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_INTERRUPT_CORE0_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_INTERRUPT_CORE0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DR_REG_INTERRUPT_CORE0_BASE     DR_REG_INTERRUPT_BASE

/* INTERRUPT_CORE0_MAC_INTR_MAP_REG register
 * mac interrupt configuration register
 */

#define INTERRUPT_CORE0_MAC_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x0)

/* INTERRUPT_CORE0_MAC_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map mac interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_MAC_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_MAC_INTR_MAP_M  (INTERRUPT_CORE0_MAC_INTR_MAP_V << INTERRUPT_CORE0_MAC_INTR_MAP_S)
#define INTERRUPT_CORE0_MAC_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_MAC_INTR_MAP_S  0

/* INTERRUPT_CORE0_MAC_NMI_MAP_REG register
 * mac_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_MAC_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x4)

/* INTERRUPT_CORE0_MAC_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map_nmi interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_MAC_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_MAC_NMI_MAP_M  (INTERRUPT_CORE0_MAC_NMI_MAP_V << INTERRUPT_CORE0_MAC_NMI_MAP_S)
#define INTERRUPT_CORE0_MAC_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_MAC_NMI_MAP_S  0

/* INTERRUPT_CORE0_PWR_INTR_MAP_REG register
 * pwr interrupt configuration register
 */

#define INTERRUPT_CORE0_PWR_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x8)

/* INTERRUPT_CORE0_PWR_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pwr interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PWR_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PWR_INTR_MAP_M  (INTERRUPT_CORE0_PWR_INTR_MAP_V << INTERRUPT_CORE0_PWR_INTR_MAP_S)
#define INTERRUPT_CORE0_PWR_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PWR_INTR_MAP_S  0

/* INTERRUPT_CORE0_BB_INT_MAP_REG register
 * bb interrupt configuration register
 */

#define INTERRUPT_CORE0_BB_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xc)

/* INTERRUPT_CORE0_BB_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map bb interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_BB_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_BB_INT_MAP_M  (INTERRUPT_CORE0_BB_INT_MAP_V << INTERRUPT_CORE0_BB_INT_MAP_S)
#define INTERRUPT_CORE0_BB_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_BB_INT_MAP_S  0

/* INTERRUPT_CORE0_BT_MAC_INT_MAP_REG register
 * bb_mac interrupt configuration register
 */

#define INTERRUPT_CORE0_BT_MAC_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x10)

/* INTERRUPT_CORE0_BT_MAC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map bb_mac interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_BT_MAC_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_BT_MAC_INT_MAP_M  (INTERRUPT_CORE0_BT_MAC_INT_MAP_V << INTERRUPT_CORE0_BT_MAC_INT_MAP_S)
#define INTERRUPT_CORE0_BT_MAC_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_BT_MAC_INT_MAP_S  0

/* INTERRUPT_CORE0_BT_BB_INT_MAP_REG register
 * bt_bb interrupt configuration register
 */

#define INTERRUPT_CORE0_BT_BB_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x14)

/* INTERRUPT_CORE0_BT_BB_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map bt_bb interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_BT_BB_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_BT_BB_INT_MAP_M  (INTERRUPT_CORE0_BT_BB_INT_MAP_V << INTERRUPT_CORE0_BT_BB_INT_MAP_S)
#define INTERRUPT_CORE0_BT_BB_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_BT_BB_INT_MAP_S  0

/* INTERRUPT_CORE0_BT_BB_NMI_MAP_REG register
 * bt_bb_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_BT_BB_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x18)

/* INTERRUPT_CORE0_BT_BB_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map bb_bt_nmi interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_BT_BB_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_BT_BB_NMI_MAP_M  (INTERRUPT_CORE0_BT_BB_NMI_MAP_V << INTERRUPT_CORE0_BT_BB_NMI_MAP_S)
#define INTERRUPT_CORE0_BT_BB_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_BT_BB_NMI_MAP_S  0

/* INTERRUPT_CORE0_RWBT_IRQ_MAP_REG register
 * rwbt_irq interrupt configuration register
 */

#define INTERRUPT_CORE0_RWBT_IRQ_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x1c)

/* INTERRUPT_CORE0_RWBT_IRQ_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rwbt_irq interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RWBT_IRQ_MAP    0x0000001f
#define INTERRUPT_CORE0_RWBT_IRQ_MAP_M  (INTERRUPT_CORE0_RWBT_IRQ_MAP_V << INTERRUPT_CORE0_RWBT_IRQ_MAP_S)
#define INTERRUPT_CORE0_RWBT_IRQ_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RWBT_IRQ_MAP_S  0

/* INTERRUPT_CORE0_RWBLE_IRQ_MAP_REG register
 * rwble_irq interrupt configuration register
 */

#define INTERRUPT_CORE0_RWBLE_IRQ_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x20)

/* INTERRUPT_CORE0_RWBLE_IRQ_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rwble_irq interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RWBLE_IRQ_MAP    0x0000001f
#define INTERRUPT_CORE0_RWBLE_IRQ_MAP_M  (INTERRUPT_CORE0_RWBLE_IRQ_MAP_V << INTERRUPT_CORE0_RWBLE_IRQ_MAP_S)
#define INTERRUPT_CORE0_RWBLE_IRQ_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RWBLE_IRQ_MAP_S  0

/* INTERRUPT_CORE0_RWBT_NMI_MAP_REG register
 * rwbt_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_RWBT_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x24)

/* INTERRUPT_CORE0_RWBT_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map mac rwbt_nmi to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RWBT_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_RWBT_NMI_MAP_M  (INTERRUPT_CORE0_RWBT_NMI_MAP_V << INTERRUPT_CORE0_RWBT_NMI_MAP_S)
#define INTERRUPT_CORE0_RWBT_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RWBT_NMI_MAP_S  0

/* INTERRUPT_CORE0_RWBLE_NMI_MAP_REG register
 * rwble_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_RWBLE_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x28)

/* INTERRUPT_CORE0_RWBLE_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rwble_nmi interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RWBLE_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_RWBLE_NMI_MAP_M  (INTERRUPT_CORE0_RWBLE_NMI_MAP_V << INTERRUPT_CORE0_RWBLE_NMI_MAP_S)
#define INTERRUPT_CORE0_RWBLE_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RWBLE_NMI_MAP_S  0

/* INTERRUPT_CORE0_I2C_MST_INT_MAP_REG register
 * i2c_mst interrupt configuration register
 */

#define INTERRUPT_CORE0_I2C_MST_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x2c)

/* INTERRUPT_CORE0_I2C_MST_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map i2c_mst interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_I2C_MST_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_I2C_MST_INT_MAP_M  (INTERRUPT_CORE0_I2C_MST_INT_MAP_V << INTERRUPT_CORE0_I2C_MST_INT_MAP_S)
#define INTERRUPT_CORE0_I2C_MST_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_I2C_MST_INT_MAP_S  0

/* INTERRUPT_CORE0_SLC0_INTR_MAP_REG register
 * slc0 interrupt configuration register
 */

#define INTERRUPT_CORE0_SLC0_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x30)

/* INTERRUPT_CORE0_SLC0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map slc0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SLC0_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_SLC0_INTR_MAP_M  (INTERRUPT_CORE0_SLC0_INTR_MAP_V << INTERRUPT_CORE0_SLC0_INTR_MAP_S)
#define INTERRUPT_CORE0_SLC0_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SLC0_INTR_MAP_S  0

/* INTERRUPT_CORE0_SLC1_INTR_MAP_REG register
 * slc1 interrupt configuration register
 */

#define INTERRUPT_CORE0_SLC1_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x34)

/* INTERRUPT_CORE0_SLC1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map slc1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SLC1_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_SLC1_INTR_MAP_M  (INTERRUPT_CORE0_SLC1_INTR_MAP_V << INTERRUPT_CORE0_SLC1_INTR_MAP_S)
#define INTERRUPT_CORE0_SLC1_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SLC1_INTR_MAP_S  0

/* INTERRUPT_CORE0_UHCI0_INTR_MAP_REG register
 * uhci0 interrupt configuration register
 */

#define INTERRUPT_CORE0_UHCI0_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x38)

/* INTERRUPT_CORE0_UHCI0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map uhci0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_UHCI0_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_UHCI0_INTR_MAP_M  (INTERRUPT_CORE0_UHCI0_INTR_MAP_V << INTERRUPT_CORE0_UHCI0_INTR_MAP_S)
#define INTERRUPT_CORE0_UHCI0_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_UHCI0_INTR_MAP_S  0

/* INTERRUPT_CORE0_UHCI1_INTR_MAP_REG register
 * uhci1 interrupt configuration register
 */

#define INTERRUPT_CORE0_UHCI1_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x3c)

/* INTERRUPT_CORE0_UHCI1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map uhci1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_UHCI1_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_UHCI1_INTR_MAP_M  (INTERRUPT_CORE0_UHCI1_INTR_MAP_V << INTERRUPT_CORE0_UHCI1_INTR_MAP_S)
#define INTERRUPT_CORE0_UHCI1_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_UHCI1_INTR_MAP_S  0

/* INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_REG register
 * gpio_interrupt_pro interrupt configuration register
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x40)

/* INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map gpio_interrupt_pro interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP    0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_M  (INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_V << INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_S)
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_V  0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_S  0

/* INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_REG register
 * gpio_interrupt_pro_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x44)

/* INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map gpio_interrupt_pro_nmi interrupt to one of
 * core0's external interrupt
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_M  (INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_V << INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_S)
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_S  0

/* INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_REG register
 * gpio_interrupt_app interrupt configuration register
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x48)

/* INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map gpio_interrupt_app interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP    0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_M  (INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_V << INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_S)
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_V  0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_MAP_S  0

/* INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_REG register
 * gpio_interrupt_app_nmi interrupt configuration register
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x4c)

/* INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map gpio_interrupt_app_nmi interrupt to one of
 * core0's external interrupt
 */

#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP    0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_M  (INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_V << INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_S)
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_V  0x0000001f
#define INTERRUPT_CORE0_GPIO_INTERRUPT_APP_NMI_MAP_S  0

/* INTERRUPT_CORE0_SPI_INTR_1_MAP_REG register
 * spi_intr_1 interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI_INTR_1_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x50)

/* INTERRUPT_CORE0_SPI_INTR_1_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi_intr_1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI_INTR_1_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_1_MAP_M  (INTERRUPT_CORE0_SPI_INTR_1_MAP_V << INTERRUPT_CORE0_SPI_INTR_1_MAP_S)
#define INTERRUPT_CORE0_SPI_INTR_1_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_1_MAP_S  0

/* INTERRUPT_CORE0_SPI_INTR_2_MAP_REG register
 * spi_intr_2 interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI_INTR_2_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x54)

/* INTERRUPT_CORE0_SPI_INTR_2_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi_intr_2 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI_INTR_2_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_2_MAP_M  (INTERRUPT_CORE0_SPI_INTR_2_MAP_V << INTERRUPT_CORE0_SPI_INTR_2_MAP_S)
#define INTERRUPT_CORE0_SPI_INTR_2_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_2_MAP_S  0

/* INTERRUPT_CORE0_SPI_INTR_3_MAP_REG register
 * spi_intr_3 interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI_INTR_3_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x58)

/* INTERRUPT_CORE0_SPI_INTR_3_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi_intr_3 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI_INTR_3_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_3_MAP_M  (INTERRUPT_CORE0_SPI_INTR_3_MAP_V << INTERRUPT_CORE0_SPI_INTR_3_MAP_S)
#define INTERRUPT_CORE0_SPI_INTR_3_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_3_MAP_S  0

/* INTERRUPT_CORE0_SPI_INTR_4_MAP_REG register
 * spi_intr_4 interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI_INTR_4_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x5c)

/* INTERRUPT_CORE0_SPI_INTR_4_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi_intr_4 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI_INTR_4_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_4_MAP_M  (INTERRUPT_CORE0_SPI_INTR_4_MAP_V << INTERRUPT_CORE0_SPI_INTR_4_MAP_S)
#define INTERRUPT_CORE0_SPI_INTR_4_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI_INTR_4_MAP_S  0

/* INTERRUPT_CORE0_LCD_CAM_INT_MAP_REG register
 * lcd_cam interrupt configuration register
 */

#define INTERRUPT_CORE0_LCD_CAM_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x60)

/* INTERRUPT_CORE0_LCD_CAM_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map lcd_cam interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_LCD_CAM_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_LCD_CAM_INT_MAP_M  (INTERRUPT_CORE0_LCD_CAM_INT_MAP_V << INTERRUPT_CORE0_LCD_CAM_INT_MAP_S)
#define INTERRUPT_CORE0_LCD_CAM_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_LCD_CAM_INT_MAP_S  0

/* INTERRUPT_CORE0_I2S0_INT_MAP_REG register
 * i2s0 interrupt configuration register
 */

#define INTERRUPT_CORE0_I2S0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x64)

/* INTERRUPT_CORE0_I2S0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map i2s0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_I2S0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_I2S0_INT_MAP_M  (INTERRUPT_CORE0_I2S0_INT_MAP_V << INTERRUPT_CORE0_I2S0_INT_MAP_S)
#define INTERRUPT_CORE0_I2S0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_I2S0_INT_MAP_S  0

/* INTERRUPT_CORE0_I2S1_INT_MAP_REG register
 * i2s1 interrupt configuration register
 */

#define INTERRUPT_CORE0_I2S1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x68)

/* INTERRUPT_CORE0_I2S1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map i2s1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_I2S1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_I2S1_INT_MAP_M  (INTERRUPT_CORE0_I2S1_INT_MAP_V << INTERRUPT_CORE0_I2S1_INT_MAP_S)
#define INTERRUPT_CORE0_I2S1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_I2S1_INT_MAP_S  0

/* INTERRUPT_CORE0_UART_INTR_MAP_REG register
 * uart interrupt configuration register
 */

#define INTERRUPT_CORE0_UART_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x6c)

/* INTERRUPT_CORE0_UART_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map uart interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_UART_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_UART_INTR_MAP_M  (INTERRUPT_CORE0_UART_INTR_MAP_V << INTERRUPT_CORE0_UART_INTR_MAP_S)
#define INTERRUPT_CORE0_UART_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_UART_INTR_MAP_S  0

/* INTERRUPT_CORE0_UART1_INTR_MAP_REG register
 * uart1 interrupt configuration register
 */

#define INTERRUPT_CORE0_UART1_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x70)

/* INTERRUPT_CORE0_UART1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map uart1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_UART1_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_UART1_INTR_MAP_M  (INTERRUPT_CORE0_UART1_INTR_MAP_V << INTERRUPT_CORE0_UART1_INTR_MAP_S)
#define INTERRUPT_CORE0_UART1_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_UART1_INTR_MAP_S  0

/* INTERRUPT_CORE0_UART2_INTR_MAP_REG register
 * uart2 interrupt configuration register
 */

#define INTERRUPT_CORE0_UART2_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x74)

/* INTERRUPT_CORE0_UART2_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map uart2 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_UART2_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_UART2_INTR_MAP_M  (INTERRUPT_CORE0_UART2_INTR_MAP_V << INTERRUPT_CORE0_UART2_INTR_MAP_S)
#define INTERRUPT_CORE0_UART2_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_UART2_INTR_MAP_S  0

/* INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_REG register
 * sdio_host interrupt configuration register
 */

#define INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x78)

/* INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map sdio_host interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP    0x0000001f
#define INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_M  (INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_V << INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_S)
#define INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SDIO_HOST_INTERRUPT_MAP_S  0

/* INTERRUPT_CORE0_PWM0_INTR_MAP_REG register
 * pwm0 interrupt configuration register
 */

#define INTERRUPT_CORE0_PWM0_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x7c)

/* INTERRUPT_CORE0_PWM0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pwm0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PWM0_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PWM0_INTR_MAP_M  (INTERRUPT_CORE0_PWM0_INTR_MAP_V << INTERRUPT_CORE0_PWM0_INTR_MAP_S)
#define INTERRUPT_CORE0_PWM0_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PWM0_INTR_MAP_S  0

/* INTERRUPT_CORE0_PWM1_INTR_MAP_REG register
 * pwm1 interrupt configuration register
 */

#define INTERRUPT_CORE0_PWM1_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x80)

/* INTERRUPT_CORE0_PWM1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pwm1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PWM1_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PWM1_INTR_MAP_M  (INTERRUPT_CORE0_PWM1_INTR_MAP_V << INTERRUPT_CORE0_PWM1_INTR_MAP_S)
#define INTERRUPT_CORE0_PWM1_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PWM1_INTR_MAP_S  0

/* INTERRUPT_CORE0_PWM2_INTR_MAP_REG register
 * pwm2 interrupt configuration register
 */

#define INTERRUPT_CORE0_PWM2_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x84)

/* INTERRUPT_CORE0_PWM2_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pwm2 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PWM2_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PWM2_INTR_MAP_M  (INTERRUPT_CORE0_PWM2_INTR_MAP_V << INTERRUPT_CORE0_PWM2_INTR_MAP_S)
#define INTERRUPT_CORE0_PWM2_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PWM2_INTR_MAP_S  0

/* INTERRUPT_CORE0_PWM3_INTR_MAP_REG register
 * pwm3 interrupt configuration register
 */

#define INTERRUPT_CORE0_PWM3_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x88)

/* INTERRUPT_CORE0_PWM3_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pwm3 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PWM3_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PWM3_INTR_MAP_M  (INTERRUPT_CORE0_PWM3_INTR_MAP_V << INTERRUPT_CORE0_PWM3_INTR_MAP_S)
#define INTERRUPT_CORE0_PWM3_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PWM3_INTR_MAP_S  0

/* INTERRUPT_CORE0_LEDC_INT_MAP_REG register
 * ledc interrupt configuration register
 */

#define INTERRUPT_CORE0_LEDC_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x8c)

/* INTERRUPT_CORE0_LEDC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map ledc interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_LEDC_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_LEDC_INT_MAP_M  (INTERRUPT_CORE0_LEDC_INT_MAP_V << INTERRUPT_CORE0_LEDC_INT_MAP_S)
#define INTERRUPT_CORE0_LEDC_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_LEDC_INT_MAP_S  0

/* INTERRUPT_CORE0_EFUSE_INT_MAP_REG register
 * efuse interrupt configuration register
 */

#define INTERRUPT_CORE0_EFUSE_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x90)

/* INTERRUPT_CORE0_EFUSE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map efuse interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_EFUSE_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_EFUSE_INT_MAP_M  (INTERRUPT_CORE0_EFUSE_INT_MAP_V << INTERRUPT_CORE0_EFUSE_INT_MAP_S)
#define INTERRUPT_CORE0_EFUSE_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_EFUSE_INT_MAP_S  0

/* INTERRUPT_CORE0_CAN_INT_MAP_REG register
 * can interrupt configuration register
 */

#define INTERRUPT_CORE0_CAN_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x94)

/* INTERRUPT_CORE0_CAN_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map can interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_CAN_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_CAN_INT_MAP_M  (INTERRUPT_CORE0_CAN_INT_MAP_V << INTERRUPT_CORE0_CAN_INT_MAP_S)
#define INTERRUPT_CORE0_CAN_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CAN_INT_MAP_S  0

/* INTERRUPT_CORE0_USB_INTR_MAP_REG register
 * usb interrupt configuration register
 */

#define INTERRUPT_CORE0_USB_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x98)

/* INTERRUPT_CORE0_USB_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map usb interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_USB_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_USB_INTR_MAP_M  (INTERRUPT_CORE0_USB_INTR_MAP_V << INTERRUPT_CORE0_USB_INTR_MAP_S)
#define INTERRUPT_CORE0_USB_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_USB_INTR_MAP_S  0

/* INTERRUPT_CORE0_RTC_CORE_INTR_MAP_REG register
 * rtc_core interrupt configuration register
 */

#define INTERRUPT_CORE0_RTC_CORE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x9c)

/* INTERRUPT_CORE0_RTC_CORE_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rtc_core interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RTC_CORE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_RTC_CORE_INTR_MAP_M  (INTERRUPT_CORE0_RTC_CORE_INTR_MAP_V << INTERRUPT_CORE0_RTC_CORE_INTR_MAP_S)
#define INTERRUPT_CORE0_RTC_CORE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RTC_CORE_INTR_MAP_S  0

/* INTERRUPT_CORE0_RMT_INTR_MAP_REG register
 * rmt interrupt configuration register
 */

#define INTERRUPT_CORE0_RMT_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xa0)

/* INTERRUPT_CORE0_RMT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rmt interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RMT_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_RMT_INTR_MAP_M  (INTERRUPT_CORE0_RMT_INTR_MAP_V << INTERRUPT_CORE0_RMT_INTR_MAP_S)
#define INTERRUPT_CORE0_RMT_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RMT_INTR_MAP_S  0

/* INTERRUPT_CORE0_PCNT_INTR_MAP_REG register
 * pcnt interrupt configuration register
 */

#define INTERRUPT_CORE0_PCNT_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xa4)

/* INTERRUPT_CORE0_PCNT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map pcnt interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_PCNT_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_PCNT_INTR_MAP_M  (INTERRUPT_CORE0_PCNT_INTR_MAP_V << INTERRUPT_CORE0_PCNT_INTR_MAP_S)
#define INTERRUPT_CORE0_PCNT_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PCNT_INTR_MAP_S  0

/* INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_REG register
 * i2c_ext0 interrupt configuration register
 */

#define INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xa8)

/* INTERRUPT_CORE0_I2C_EXT0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map i2c_ext0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_I2C_EXT0_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_M  (INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_V << INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_S)
#define INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_I2C_EXT0_INTR_MAP_S  0

/* INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_REG register
 * i2c_ext1 interrupt configuration register
 */

#define INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xac)

/* INTERRUPT_CORE0_I2C_EXT1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map i2c_ext1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_I2C_EXT1_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_M  (INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_V << INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_S)
#define INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_I2C_EXT1_INTR_MAP_S  0

/* INTERRUPT_CORE0_SPI2_DMA_INT_MAP_REG register
 * spi2_dma interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI2_DMA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xb0)

/* INTERRUPT_CORE0_SPI2_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi2_dma interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI2_DMA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI2_DMA_INT_MAP_M  (INTERRUPT_CORE0_SPI2_DMA_INT_MAP_V << INTERRUPT_CORE0_SPI2_DMA_INT_MAP_S)
#define INTERRUPT_CORE0_SPI2_DMA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI2_DMA_INT_MAP_S  0

/* INTERRUPT_CORE0_SPI3_DMA_INT_MAP_REG register
 * spi3_dma interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI3_DMA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xb4)

/* INTERRUPT_CORE0_SPI3_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi3_dma interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI3_DMA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI3_DMA_INT_MAP_M  (INTERRUPT_CORE0_SPI3_DMA_INT_MAP_V << INTERRUPT_CORE0_SPI3_DMA_INT_MAP_S)
#define INTERRUPT_CORE0_SPI3_DMA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI3_DMA_INT_MAP_S  0

/* INTERRUPT_CORE0_SPI4_DMA_INT_MAP_REG register
 * spi4_dma interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI4_DMA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xb8)

/* INTERRUPT_CORE0_SPI4_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi4_dma interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SPI4_DMA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI4_DMA_INT_MAP_M  (INTERRUPT_CORE0_SPI4_DMA_INT_MAP_V << INTERRUPT_CORE0_SPI4_DMA_INT_MAP_S)
#define INTERRUPT_CORE0_SPI4_DMA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI4_DMA_INT_MAP_S  0

/* INTERRUPT_CORE0_WDG_INT_MAP_REG register
 * wdg interrupt configuration register
 */

#define INTERRUPT_CORE0_WDG_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xbc)

/* INTERRUPT_CORE0_WDG_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map wdg interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_WDG_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_WDG_INT_MAP_M  (INTERRUPT_CORE0_WDG_INT_MAP_V << INTERRUPT_CORE0_WDG_INT_MAP_S)
#define INTERRUPT_CORE0_WDG_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_WDG_INT_MAP_S  0

/* INTERRUPT_CORE0_TIMER_INT1_MAP_REG register
 * timer_int1 interrupt configuration register
 */

#define INTERRUPT_CORE0_TIMER_INT1_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xc0)

/* INTERRUPT_CORE0_TIMER_INT1_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map timer_int1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TIMER_INT1_MAP    0x0000001f
#define INTERRUPT_CORE0_TIMER_INT1_MAP_M  (INTERRUPT_CORE0_TIMER_INT1_MAP_V << INTERRUPT_CORE0_TIMER_INT1_MAP_S)
#define INTERRUPT_CORE0_TIMER_INT1_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TIMER_INT1_MAP_S  0

/* INTERRUPT_CORE0_TIMER_INT2_MAP_REG register
 * timer_int2 interrupt configuration register
 */

#define INTERRUPT_CORE0_TIMER_INT2_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xc4)

/* INTERRUPT_CORE0_TIMER_INT2_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map timer_int2 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TIMER_INT2_MAP    0x0000001f
#define INTERRUPT_CORE0_TIMER_INT2_MAP_M  (INTERRUPT_CORE0_TIMER_INT2_MAP_V << INTERRUPT_CORE0_TIMER_INT2_MAP_S)
#define INTERRUPT_CORE0_TIMER_INT2_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TIMER_INT2_MAP_S  0

/* INTERRUPT_CORE0_TG_T0_INT_MAP_REG register
 * tg_t0 interrupt configuration register
 */

#define INTERRUPT_CORE0_TG_T0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xc8)

/* INTERRUPT_CORE0_TG_T0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map tg_t0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG_T0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG_T0_INT_MAP_M  (INTERRUPT_CORE0_TG_T0_INT_MAP_V << INTERRUPT_CORE0_TG_T0_INT_MAP_S)
#define INTERRUPT_CORE0_TG_T0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG_T0_INT_MAP_S  0

/* INTERRUPT_CORE0_TG_T1_INT_MAP_REG register
 * tg_t1 interrupt configuration register
 */

#define INTERRUPT_CORE0_TG_T1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xcc)

/* INTERRUPT_CORE0_TG_T1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map tg_t1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG_T1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG_T1_INT_MAP_M  (INTERRUPT_CORE0_TG_T1_INT_MAP_V << INTERRUPT_CORE0_TG_T1_INT_MAP_S)
#define INTERRUPT_CORE0_TG_T1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG_T1_INT_MAP_S  0

/* INTERRUPT_CORE0_TG_WDT_INT_MAP_REG register
 * tg_wdt interrupt configuration register
 */

#define INTERRUPT_CORE0_TG_WDT_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xd0)

/* INTERRUPT_CORE0_TG_WDT_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rg_wdt interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG_WDT_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG_WDT_INT_MAP_M  (INTERRUPT_CORE0_TG_WDT_INT_MAP_V << INTERRUPT_CORE0_TG_WDT_INT_MAP_S)
#define INTERRUPT_CORE0_TG_WDT_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG_WDT_INT_MAP_S  0

/* INTERRUPT_CORE0_TG1_T0_INT_MAP_REG register
 * tg1_t0 interrupt configuration register
 */

#define INTERRUPT_CORE0_TG1_T0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xd4)

/* INTERRUPT_CORE0_TG1_T0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map tg1_t0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG1_T0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG1_T0_INT_MAP_M  (INTERRUPT_CORE0_TG1_T0_INT_MAP_V << INTERRUPT_CORE0_TG1_T0_INT_MAP_S)
#define INTERRUPT_CORE0_TG1_T0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG1_T0_INT_MAP_S  0

/* INTERRUPT_CORE0_TG1_T1_INT_MAP_REG register
 * tg1_t1 interrupt configuration register
 */

#define INTERRUPT_CORE0_TG1_T1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xd8)

/* INTERRUPT_CORE0_TG1_T1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map tg1_t1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG1_T1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG1_T1_INT_MAP_M  (INTERRUPT_CORE0_TG1_T1_INT_MAP_V << INTERRUPT_CORE0_TG1_T1_INT_MAP_S)
#define INTERRUPT_CORE0_TG1_T1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG1_T1_INT_MAP_S  0

/* INTERRUPT_CORE0_TG1_WDT_INT_MAP_REG register
 * tg1_wdt interrupt configuration register
 */

#define INTERRUPT_CORE0_TG1_WDT_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xdc)

/* INTERRUPT_CORE0_TG1_WDT_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map tg1_wdt interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_TG1_WDT_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_TG1_WDT_INT_MAP_M  (INTERRUPT_CORE0_TG1_WDT_INT_MAP_V << INTERRUPT_CORE0_TG1_WDT_INT_MAP_S)
#define INTERRUPT_CORE0_TG1_WDT_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_TG1_WDT_INT_MAP_S  0

/* INTERRUPT_CORE0_CACHE_IA_INT_MAP_REG register
 * cache_ia interrupt configuration register
 */

#define INTERRUPT_CORE0_CACHE_IA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xe0)

/* INTERRUPT_CORE0_CACHE_IA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cache_ia interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_CACHE_IA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_CACHE_IA_INT_MAP_M  (INTERRUPT_CORE0_CACHE_IA_INT_MAP_V << INTERRUPT_CORE0_CACHE_IA_INT_MAP_S)
#define INTERRUPT_CORE0_CACHE_IA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CACHE_IA_INT_MAP_S  0

/* INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG register
 * systimer_target0 interrupt configuration register
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xe4)

/* INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map systimer_target0 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_M  (INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_V << INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_S)
#define INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_S  0

/* INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_REG register
 * systimer_target1 interrupt configuration register
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xe8)

/* INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map systimer_target1 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_M  (INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_V << INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_S)
#define INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET1_INT_MAP_S  0

/* INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_REG register
 * systimer_target2 interrupt configuration register
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xec)

/* INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map systimer_target2 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_M  (INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_V << INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_S)
#define INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SYSTIMER_TARGET2_INT_MAP_S  0

/* INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_REG register
 * spi_mem_reject interrupt configuration register
 */

#define INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xf0)

/* INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map spi_mem_reject interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_M  (INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_V << INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_S)
#define INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SPI_MEM_REJECT_INTR_MAP_S  0

/* INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_REG register
 * dcache_prelaod interrupt configuration register
 */

#define INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xf4)

/* INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dcache_prelaod interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_M  (INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_V << INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_S)
#define INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DCACHE_PRELOAD_INT_MAP_S  0

/* INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_REG register
 * icache_preload interrupt configuration register
 */

#define INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xf8)

/* INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map icache_preload interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_M  (INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_V << INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_S)
#define INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_ICACHE_PRELOAD_INT_MAP_S  0

/* INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_REG register
 * dcache_sync interrupt configuration register
 */

#define INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0xfc)

/* INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dcache_sync interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_M  (INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_V << INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_S)
#define INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DCACHE_SYNC_INT_MAP_S  0

/* INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_REG register
 * icache_sync interrupt configuration register
 */

#define INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x100)

/* INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map icache_sync interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_M  (INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_V << INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_S)
#define INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_ICACHE_SYNC_INT_MAP_S  0

/* INTERRUPT_CORE0_APB_ADC_INT_MAP_REG register
 * apb_adc interrupt configuration register
 */

#define INTERRUPT_CORE0_APB_ADC_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x104)

/* INTERRUPT_CORE0_APB_ADC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map apb_adc interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_APB_ADC_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_APB_ADC_INT_MAP_M  (INTERRUPT_CORE0_APB_ADC_INT_MAP_V << INTERRUPT_CORE0_APB_ADC_INT_MAP_S)
#define INTERRUPT_CORE0_APB_ADC_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_APB_ADC_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_REG register
 * dma_in_ch0 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x108)

/* INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_in_ch0 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_M  (INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_V << INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_REG register
 * dma_in_ch1 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x10c)

/* INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_in_ch1 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_M  (INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_V << INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH1_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_REG register
 * dma_in_ch2 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x110)

/* INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_in_ch2 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_M  (INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_V << INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH2_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_REG register
 * dma_in_ch3 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x114)

/* INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_in_ch3 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_M  (INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_V << INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH3_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_REG register
 * dma_in_ch4 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x118)

/* INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_in_ch4 interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_M  (INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_V << INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_IN_CH4_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_REG register
 * dma_out_ch0 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x11c)

/* INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_out_ch0 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_M  (INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_V << INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_REG register
 * dma_out_ch1 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x120)

/* INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_out_ch1 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_M  (INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_V << INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH1_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_REG register
 * dma_out_ch2 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x124)

/* INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_out_ch2 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_M  (INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_V << INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH2_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_REG register
 * dma_out_ch3 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x128)

/* INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_out_ch3 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_M  (INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_V << INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH3_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_REG register
 * dma_out_ch4 interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x12c)

/* INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map dma_out_ch4 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_M  (INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_V << INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_OUT_CH4_INT_MAP_S  0

/* INTERRUPT_CORE0_RSA_INT_MAP_REG register
 * rsa interrupt configuration register
 */

#define INTERRUPT_CORE0_RSA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x130)

/* INTERRUPT_CORE0_RSA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map rsa interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_RSA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_RSA_INT_MAP_M  (INTERRUPT_CORE0_RSA_INT_MAP_V << INTERRUPT_CORE0_RSA_INT_MAP_S)
#define INTERRUPT_CORE0_RSA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_RSA_INT_MAP_S  0

/* INTERRUPT_CORE0_AES_INT_MAP_REG register
 * aes interrupt configuration register
 */

#define INTERRUPT_CORE0_AES_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x134)

/* INTERRUPT_CORE0_AES_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map aes interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_AES_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_AES_INT_MAP_M  (INTERRUPT_CORE0_AES_INT_MAP_V << INTERRUPT_CORE0_AES_INT_MAP_S)
#define INTERRUPT_CORE0_AES_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_AES_INT_MAP_S  0

/* INTERRUPT_CORE0_SHA_INT_MAP_REG register
 * sha interrupt configuration register
 */

#define INTERRUPT_CORE0_SHA_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x138)

/* INTERRUPT_CORE0_SHA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map sha interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_SHA_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_SHA_INT_MAP_M  (INTERRUPT_CORE0_SHA_INT_MAP_V << INTERRUPT_CORE0_SHA_INT_MAP_S)
#define INTERRUPT_CORE0_SHA_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_SHA_INT_MAP_S  0

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_REG register
 * cpu_intr_from_cpu_0 interrupt configuration register
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x13c)

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cpu_intr_from_cpu_0 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP    0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_M  (INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_V << INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_S)
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_0_MAP_S  0

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_REG register
 * cpu_intr_from_cpu_1 interrupt configuration register
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x140)

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cpu_intr_from_cpu_1 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP    0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_M  (INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_V << INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_S)
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_1_MAP_S  0

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_REG register
 * cpu_intr_from_cpu_2 interrupt configuration register
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x144)

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cpu_intr_from_cpu_2 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP    0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_M  (INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_V << INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_S)
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_2_MAP_S  0

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_REG register
 * cpu_intr_from_cpu_3 interrupt configuration register
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x148)

/* INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cpu_intr_from_cpu_3 interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP    0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_M  (INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_V << INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_S)
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CPU_INTR_FROM_CPU_3_MAP_S  0

/* INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_REG register
 * assist_debug interrupt configuration register
 */

#define INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x14c)

/* INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map assist_debug interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_M  (INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_V << INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_S)
#define INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_ASSIST_DEBUG_INTR_MAP_S  0

/* INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * dma_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x150)

/* INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map dma_pms_monitor_violatile interrupt to one of
 * core0's external interrupt
 */

#define INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_APBPERI_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core0_IRam0_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x154)

/* INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core0_IRam0_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core0_DRam0_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x158)

/* INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core0_DRam0_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core0_PIF_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x15c)

/* INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core0_PIF_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_REG register
 * core0_PIF_pms_monitor_violatile_size interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x160)

/* INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP : R/W;
 * bitpos: [4:0]; default: 16;
 * this register used to map core0_PIF_pms_monitor_violatile_size interrupt
 * to one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_V << INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core1_IRam0_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x164)

/* INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core1_IRam0_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_1_IRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core1_DRam0_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x168)

/* INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core1_DRam0_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_1_DRAM0_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_REG register
 * core1_PIF_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x16c)

/* INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP : R/W; bitpos:
 * [4:0]; default: 16;
 * this register used to map core1_PIF_pms_monitor_violatile interrupt to
 * one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_REG register
 * core1_PIF_pms_monitor_violatile_size interrupt configuration register
 */

#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x170)

/* INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP : R/W;
 * bitpos: [4:0]; default: 16;
 * this register used to map core1_PIF_pms_monitor_violatile_size interrupt
 * to one of core0's external interrupt
 */

#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_M  (INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_V << INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_S)
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CORE_1_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_S  0

/* INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_REG register
 * backup_pms_monitor_violatile interrupt configuration register
 */

#define INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x174)

/* INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP : R/W; bitpos: [4:0];
 * default: 16;
 * this register used to map backup_pms_monitor_violatile interrupt to one
 * of core0's external interrupt
 */

#define INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP    0x0000001f
#define INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_M  (INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_V << INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_S)
#define INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_V  0x0000001f
#define INTERRUPT_CORE0_BACKUP_PMS_VIOLATE_INTR_MAP_S  0

/* INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_REG register
 * cache_core0_acs interrupt configuration register
 */

#define INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x178)

/* INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cache_core0_acs interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_M  (INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_V << INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_S)
#define INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CACHE_CORE0_ACS_INT_MAP_S  0

/* INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_REG register
 * cache_core1_acs interrupt configuration register
 */

#define INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x17c)

/* INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map cache_core1_acs interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_M  (INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_V << INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_S)
#define INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_CACHE_CORE1_ACS_INT_MAP_S  0

/* INTERRUPT_CORE0_USB_DEVICE_INT_MAP_REG register
 * usb_device interrupt configuration register
 */

#define INTERRUPT_CORE0_USB_DEVICE_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x180)

/* INTERRUPT_CORE0_USB_DEVICE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map usb_device interrupt to one of core0's external
 * interrupt
 */

#define INTERRUPT_CORE0_USB_DEVICE_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_USB_DEVICE_INT_MAP_M  (INTERRUPT_CORE0_USB_DEVICE_INT_MAP_V << INTERRUPT_CORE0_USB_DEVICE_INT_MAP_S)
#define INTERRUPT_CORE0_USB_DEVICE_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_USB_DEVICE_INT_MAP_S  0

/* INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_REG register
 * peri_backup interrupt configuration register
 */

#define INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x184)

/* INTERRUPT_CORE0_PERI_BACKUP_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * this register used to map peri_backup interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_PERI_BACKUP_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_M  (INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_V << INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_S)
#define INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_PERI_BACKUP_INT_MAP_S  0

/* INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_REG register
 * dma_extmem_reject interrupt configuration register
 */

#define INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x188)

/* INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * this register used to map dma_extmem_reject interrupt to one of core0's
 * external interrupt
 */

#define INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP    0x0000001f
#define INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_M  (INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_V << INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_S)
#define INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_V  0x0000001f
#define INTERRUPT_CORE0_DMA_EXTMEM_REJECT_INT_MAP_S  0

/* INTERRUPT_CORE0_INTR_STATUS_0_REG register
 * interrupt status register
 */

#define INTERRUPT_CORE0_INTR_STATUS_0_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x18c)

/* INTERRUPT_CORE0_INTR_STATUS_0 : RO; bitpos: [31:0]; default: 0;
 * this register store the status of the first 32 interrupt source
 */

#define INTERRUPT_CORE0_INTR_STATUS_0    0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_0_M  (INTERRUPT_CORE0_INTR_STATUS_0_V << INTERRUPT_CORE0_INTR_STATUS_0_S)
#define INTERRUPT_CORE0_INTR_STATUS_0_V  0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_0_S  0

/* INTERRUPT_CORE0_INTR_STATUS_1_REG register
 * interrupt status register
 */

#define INTERRUPT_CORE0_INTR_STATUS_1_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x190)

/* INTERRUPT_CORE0_INTR_STATUS_1 : RO; bitpos: [31:0]; default: 0;
 * this register store the status of the first 32 interrupt source
 */

#define INTERRUPT_CORE0_INTR_STATUS_1    0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_1_M  (INTERRUPT_CORE0_INTR_STATUS_1_V << INTERRUPT_CORE0_INTR_STATUS_1_S)
#define INTERRUPT_CORE0_INTR_STATUS_1_V  0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_1_S  0

/* INTERRUPT_CORE0_INTR_STATUS_2_REG register
 * interrupt status register
 */

#define INTERRUPT_CORE0_INTR_STATUS_2_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x194)

/* INTERRUPT_CORE0_INTR_STATUS_2 : RO; bitpos: [31:0]; default: 0;
 * this register store the status of the first 32 interrupt source
 */

#define INTERRUPT_CORE0_INTR_STATUS_2    0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_2_M  (INTERRUPT_CORE0_INTR_STATUS_2_V << INTERRUPT_CORE0_INTR_STATUS_2_S)
#define INTERRUPT_CORE0_INTR_STATUS_2_V  0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_2_S  0

/* INTERRUPT_CORE0_INTR_STATUS_3_REG register
 * interrupt status register
 */

#define INTERRUPT_CORE0_INTR_STATUS_3_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x198)

/* INTERRUPT_CORE0_INTR_STATUS_3 : RO; bitpos: [31:0]; default: 0;
 * this register store the status of the first 32 interrupt source
 */

#define INTERRUPT_CORE0_INTR_STATUS_3    0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_3_M  (INTERRUPT_CORE0_INTR_STATUS_3_V << INTERRUPT_CORE0_INTR_STATUS_3_S)
#define INTERRUPT_CORE0_INTR_STATUS_3_V  0xffffffff
#define INTERRUPT_CORE0_INTR_STATUS_3_S  0

/* INTERRUPT_CORE0_CLOCK_GATE_REG register
 * clock gate register
 */

#define INTERRUPT_CORE0_CLOCK_GATE_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x19c)

/* INTERRUPT_CORE0_REG_CLK_EN : R/W; bitpos: [0]; default: 1;
 * this register uesd to control clock-gating interupt martrix
 */

#define INTERRUPT_CORE0_REG_CLK_EN    (BIT(0))
#define INTERRUPT_CORE0_REG_CLK_EN_M  (INTERRUPT_CORE0_REG_CLK_EN_V << INTERRUPT_CORE0_REG_CLK_EN_S)
#define INTERRUPT_CORE0_REG_CLK_EN_V  0x00000001
#define INTERRUPT_CORE0_REG_CLK_EN_S  0

/* INTERRUPT_CORE0_DATE_REG register
 * version register
 */

#define INTERRUPT_CORE0_DATE_REG (DR_REG_INTERRUPT_CORE0_BASE + 0x7fc)

/* INTERRUPT_CORE0_INTERRUPT_REG_DATE : R/W; bitpos: [27:0]; default:
 * 33628928;
 * version register
 */

#define INTERRUPT_CORE0_INTERRUPT_REG_DATE    0x0fffffff
#define INTERRUPT_CORE0_INTERRUPT_REG_DATE_M  (INTERRUPT_CORE0_INTERRUPT_REG_DATE_V << INTERRUPT_CORE0_INTERRUPT_REG_DATE_S)
#define INTERRUPT_CORE0_INTERRUPT_REG_DATE_V  0x0fffffff
#define INTERRUPT_CORE0_INTERRUPT_REG_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_INTERRUPT_CORE0_H */
