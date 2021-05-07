/****************************************************************************
 * arch/arm/src/eoss3/hardware/eoss3_memorymap.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_MEMORYMAP_H
#define __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address Blocks ***********************************************************/

#define EOSS3_CODE_BASE      0x00000000
#define EOSS3_SRAM_BASE      0x20000000
#define EOSS3_PERIPH_BASE    0x40000000
#define EOSS3_CORTEX_BASE    0xe0000000

/* Peripheral Base Addresses ************************************************/

#define EOSS3_PERIPH_BASE    0x40000000
#define EOSS3_PKT_FIFO_BASE  0x40002000
#define EOSS3_CLK_BASE       0x40004000
#define EOSS3_PMU_BASE       0x40004400
#define EOSS3_INTR_CTRL_BASE 0x40004800
#define EOSS3_IO_MUX_BASE    0x40004c00
#define EOSS3_MISC_BASE      0x40005000
#define EOSS3_AIP_BASE       0x40005400
#define EOSS3_JTM_BASE       0x40005a00
#define EOSS3_SPT_BASE       0x40005c00
#define EOSS3_A1_BASE        0x40006000
#define EOSS3_SPI_BASE       0x40007000
#define EOSS3_DMA_SPI_BASE   0x40007400
#define EOSS3_EFUSE_BASE     0x40008000
#define EOSS3_I2S_BASE       0x4000b000
#define EOSS3_SDMA_BASE      0x4000c000
#define EOSS3_SDMA_BRG_BASE  0x4000d000
#define EOSS3_SDMA_SRAM_BASE 0x4000f000
#define EOSS3_UART_BASE      0x40010000
#define EOSS3_WDT_BASE       0x40012000
#define EOSS3_TIMER_BASE     0x40013000
#define EOSS3_PIF_BASE       0x40014000
#define EOSS3_AUD_BASE       0x40015000
#define EOSS3_RAMFIFO0_BASE  0x40018000
#define EOSS3_RAMFIFO1_BASE  0x40019000
#define EOSS3_RAMFIFO2_BASE  0x4001a000
#define EOSS3_RAMFIFO3_BASE  0x4001b000
#define EOSS3_FABRIC_BASE    0x40020000
#define EOSS3_FFE_BASE       0x40040000
#define EOSS3_CM_BASE        0x40050000
#define EOSS3_ITM_BASE       0xe0000000
#define EOSS3_DWT_BASE       0xe0001000
#define EOSS3_FBP_BASE       0xe0002000
#define EOSS3_SCS_BASE       0xe000e000
#define EOSS3_TPIU_BASE      0xe0040000
#define EOSS3_DAP_BASE       0xe00ff000

#endif /* __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_MEMORYMAP_H */
