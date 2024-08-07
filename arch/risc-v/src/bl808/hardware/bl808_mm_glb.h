/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_mm_glb.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MM_GLB_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MM_GLB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL808_MM_GLB_CLK_CTRL_PERI_OFFSET 0x10

/* Register definitions *****************************************************/

#define BL808_MM_GLB_CLK_CTRL_PERI (BL808_MM_GLB_BASE \
                                   + BL808_MM_GLB_CLK_CTRL_PERI_OFFSET)

/* Register bit definitions *************************************************/

/* CLK_CTRL_PERI ************************************************************/

#define CLK_CTRL_PERI_I2C0_DIV_SHIFT 0
#define CLK_CTRL_PERI_I2C0_DIV_MASK (0xff << CLK_CTRL_PERI_I2C0_DIV_SHIFT)
#define CLK_CTRL_PERI_I2C0_EN_SHIFT 9
#define CLK_CTRL_PERI_UART_DIV_EN_SHIFT 16
#define CLK_CTRL_PERI_UART_DIV_SHIFT 17
#define CLK_CTRL_PERI_UART_DIV_MASK (0x07 << CLK_CTRL_PERI_UART_DIV_SHIFT)
#define CLK_CTRL_PERI_SPI_DIV_EN_SHIFT 23
#define CLK_CTRL_PERI_SPI_DIV_SHIFT 24
#define CLK_CTRL_PERI_SPI_DIV_MASK (0xff << CLK_CTRL_PERI_SPI_DIV_SHIFT)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MM_GLB_H */
