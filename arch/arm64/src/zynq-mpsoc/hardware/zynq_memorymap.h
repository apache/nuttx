/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/hardware/zynq_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_ZYNQ_ZYNQ_MPSOC_HARDWARE_ZYNQ_MEMORYMAP_H
#define __ARCH_ARM64_SRC_ZYNQ_ZYNQ_MPSOC_HARDWARE_ZYNQ_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Base Addresses */

#define ZYNQ_MPSOC_UART0_ADDR        0xFF000000 /* UART0 reg base address */
#define ZYNQ_MPSOC_UART1_ADDR        0xFF010000 /* UART1 reg base address */
#define ZYNQ_MPSOC_I2C0_ADDR         0xFF020000 /* I2C0 reg base address  */
#define ZYNQ_MPSOC_I2C1_ADDR         0xFF030000 /* I2C1 reg base address  */
#define ZYNQ_MPSOC_SPI0_ADDR         0xFF040000 /* SPI0 reg base address  */
#define ZYNQ_MPSOC_SPI1_ADDR         0xFF050000 /* SPI1 reg base address  */
#define ZYNQ_MPSOC_CAN0_ADDR         0xFF060000 /* CAN0 reg base address  */
#define ZYNQ_MPSOC_CAN1_ADDR         0xFF070000 /* CAN1 reg base address  */
#define ZYNQ_MPSOC_GPIO_ADDR         0xFF0A0000 /* GPIO reg base address  */
#define ZYNQ_MPSOC_GEM0_ADDR         0xFF0B0000 /* GEM0 reg base address  */
#define ZYNQ_MPSOC_GEM1_ADDR         0xFF0C0000 /* GEM1 reg base address  */
#define ZYNQ_MPSOC_GEM2_ADDR         0xFF0D0000 /* GEM2 reg base address  */
#define ZYNQ_MPSOC_GEM3_ADDR         0xFF0E0000 /* GEM3 reg base address  */
#define ZYNQ_MPSOC_MDIO_ADDR(ch)     0xFF0B0000 /* GEM3 reg base address  */
#define ZYNQ_MPSOC_QSPI_ADDR         0xFF0F0000 /* QSPI reg base address  */
#define ZYNQ_MPSOC_TTC0_ADDR         0xFF110000 /* NAND reg base address  */
#define ZYNQ_MPSOC_TTC1_ADDR         0xFF120000 /* TTC0 reg base address  */
#define ZYNQ_MPSOC_TTC2_ADDR         0xFF130000 /* TTC1 reg base address  */
#define ZYNQ_MPSOC_TTC3_ADDR         0xFF140000 /* TTC2 reg base address  */
#define ZYNQ_MPSOC_SD0_ADDR          0xFF160000 /* SD0 reg base address   */
#define ZYNQ_MPSOC_SD1_ADDR          0xFF170000 /* SD1 reg base address   */
#define ZYNQ_MPSOC_IOU_SLCR_ADDR     0xFF180000 /* IOU reg base address   */
#define ZYNQ_MPSOC_IPIBUF_ADDR       0xFF990000 /* IPI buffer base address */
#define ZYNQ_MPSOC_USB0_ADDR         0xFF9D0000 /* USB0 reg base address   */
#define ZYNQ_MPSOC_USB1_ADDR         0xFF9E0000 /* USB1 reg base address   */
#define ZYNQ_MPSOC_AMS_ADDR          0xFFA50000 /* AMS reg base address    */
#define ZYNQ_MPSOC_PSSYSMON_ADDR     0xFFA50800 /* PS sys monitor reg base */
#define ZYNQ_MPSOC_PLSYSMON_ADDR     0xFFA50C00 /* PL sys monitor reg base */
#define ZYNQ_MPSOC_CSU_SWDT_ADDR     0xFFCB0000 /* CSU wdg mon reg base    */
#define ZYNQ_MPSOC_CRF_APB_CLKC_ADDR 0xFD1A0000
#define ZYNQ_MPSOC_CRL_APB_CLKC_ADDR 0xFF5E0000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_ZYNQ_ZYNQ_MPSOC_HARDWARE_ZYNQ_MEMORYMAP_H */
