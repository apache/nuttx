/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_glb.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL808_GLB_UART_CFG1_OFFSET    0x154
#define BL808_GLB_UART_CFG2_OFFSET    0x158
#define BL808_GLB_I2C_CFG0_OFFSET     0x180
#define BL808_GLB_SPI_CFG0_OFFSET     0x1b0
#define BL808_GLB_PARM_CFG0_OFFSET    0x510

#define BL808_GPIO_CFG_OFFSET              0x0008c4  /* gpio_cfg0 */

/* Register definitions *****************************************************/

#define BL808_GLB_UART_CFG1 (BL808_GLB_BASE + BL808_GLB_UART_CFG1_OFFSET)
#define BL808_GLB_UART_CFG2 (BL808_GLB_BASE + BL808_GLB_UART_CFG2_OFFSET)
#define BL808_GLB_I2C_CFG0 (BL808_GLB_BASE + BL808_GLB_I2C_CFG0_OFFSET)
#define BL808_GLB_SPI_CFG0 (BL808_GLB_BASE + BL808_GLB_SPI_CFG0_OFFSET)
#define BL808_GLB_PARM_CFG0 (BL808_GLB_BASE + BL808_GLB_PARM_CFG0_OFFSET)

#define BL808_GPIO_CFG(n)        (BL808_GLB_BASE + BL808_GPIO_CFG_OFFSET + 4*n)

/* Register bit definitions *************************************************/

/* UART_CFG registers *******************************************************/

#define UART_CFG_SIG_SEL_SHIFT(n)  ((n % 8) * 4)
#define UART_CFG_SIG_SEL_MASK(n)   (0x0f << UART_CFG_SIG_SEL_SHIFT(n))

/* I2C_CFG0 *****************************************************************/

#define I2C_CFG_CLK_DIV_SHIFT 16
#define I2C_CFG_CLK_DIV_MASK (0xff << I2C_CFG_CLK_DIV_SHIFT)
#define I2C_CFG_CLK_EN (1 << 24)
#define I2C_CFG_CLK_XTAL (1 << 25)

/* SPI_CFG0 *****************************************************************/

#define SPI_CFG_CLK_DIV_SHIFT 0
#define SPI_CFG_CLK_DIV_MASK (0x1f << SPI_CFG_CLK_DIV_SHIFT)
#define SPI_CFG_CLK_EN_SHIFT 8
#define SPI_CFG_CLK_SEL_SHIFT 9
#define SPI_CFG_SWAP_SET_SHIFT 16
#define SPI_CFG_SWAP_SET_MASK (0x0f << SPI_CFG_SWAP_SET_SHIFT);

/* PARM_CFG0 ****************************************************************/

#define PARM_SPI_0_MASTER_MODE_SHIFT 12
#define PARM_MM_SPI_MASTER_MODE_SHIFT 27

/* GPIO_CFG registers *******************************************************/

/* bit definitions from lupyuen's wip-nuttx, branch gpio2 *******************/

#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT           (8)
#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_MASK            (0x0f << GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_OE                       (1 << 6)
#define GPIO_CFGCTL0_GPIO_0_PD                       (1 << 5)
#define GPIO_CFGCTL0_GPIO_0_PU                       (1 << 4)
#define GPIO_CFGCTL0_GPIO_0_DRV_SHIFT                (2)
#define GPIO_CFGCTL0_GPIO_0_DRV_MASK                 (0x03 << GPIO_CFGCTL0_GPIO_0_DRV_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_SMT                      (1 << 1)
#define GPIO_CFGCTL0_GPIO_0_IE                       (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H */
