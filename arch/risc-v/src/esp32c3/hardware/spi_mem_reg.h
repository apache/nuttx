/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/spi_mem_reg.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_SPI_MEM_REG_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_SPI_MEM_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_MEM_CMD_REG(i)          (REG_SPI_MEM_BASE(i) + 0x000)
#define SPI_MEM_CLOCK_GATE_REG(i)   (REG_SPI_MEM_BASE(i) + 0x0DC)

/* SPI_MEM_CLK_EN : R/W ;bitpos:[0] ;default: 1'b1.
 * Register clock gate enable signal. 1: Enable. 0: Disable.
 */

#define SPI_MEM_CLK_EN    (BIT(0))
#define SPI_MEM_CLK_EN_M  (BIT(0))
#define SPI_MEM_CLK_EN_V  0x1
#define SPI_MEM_CLK_EN_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_SPI_MEM_REG_H */
