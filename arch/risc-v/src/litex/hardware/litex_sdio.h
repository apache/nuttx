/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_sdio.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_SDIO_H
#define __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/litex_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LITEX_SDIO register offsets **********************************************/

#define LITEX_SDBLOCK2MEM_DMA_BASE_OFFSET   0x0000
#define LITEX_SDBLOCK2MEM_DMA_LENGTH_OFFSET 0x0008
#define LITEX_SDBLOCK2MEM_DMA_ENABLE_OFFSET 0x000C
#define LITEX_SDBLOCK2MEM_DMA_DONE_OFFSET   0x0010
#define LITEX_SDBLOCK2MEM_DMA_LOOP_OFFSET   0x0014
#define LITEX_SDBLOCK2MEM_DMA_OFFSET_OFFSET 0x0018

#define LITEX_SDCORE_CMD_ARGUMENT_OFFSET    0x0000
#define LITEX_SDCORE_CMD_COMMAND_OFFSET     0x0004
#define LITEX_SDCORE_CMD_SEND_OFFSET        0x0008
#define LITEX_SDCORE_CMD_RESPONSE_OFFSET    0x000C
#define LITEX_SDCORE_CMD_EVENT_OFFSET       0x001C
#define LITEX_SDCORE_DATA_EVENT_OFFSET      0x0020
#define LITEX_SDCORE_BLOCK_LENGTH_OFFSET    0x0024
#define LITEX_SDCORE_BLOCK_COUNT_OFFSET     0x0028

#define LITEX_SDIRQ_STATUS_OFFSET           0x0000
#define LITEX_SDIRQ_PENDING_OFFSET          0x0004
#define LITEX_SDIRQ_ENABLE_OFFSET           0x0008

#define LITEX_SDMEM2BLOCK_DMA_BASE_OFFSET   0x0000
#define LITEX_SDMEM2BLOCK_DMA_LENGTH_OFFSET 0x0008
#define LITEX_SDMEM2BLOCK_DMA_ENABLE_OFFSET 0x000C
#define LITEX_SDMEM2BLOCK_DMA_DONE_OFFSET   0x0010
#define LITEX_SDMEM2BLOCK_DMA_LOOP_OFFSET   0x0014
#define LITEX_SDMEM2BLOCK_DMA_OFFSET_OFFSET 0x0018

#define LITEX_SDPHY_CARD_DETECT_OFFSET      0x0000
#define LITEX_SDPHY_CLOCKER_DIVIDER_OFFSET  0x0004
#define LITEX_SDPHY_INIT_INITIALIZE_OFFSET  0x0008
#define LITEX_SDPHY_DATAW_STATUS_OFFSET     0x000C

/* LITEX_SDIO register addresses ********************************************/

#define LITEX_SDBLOCK2MEM_DMA_BASE          (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_BASE_OFFSET)
#define LITEX_SDBLOCK2MEM_DMA_LENGTH        (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_LENGTH_OFFSET)
#define LITEX_SDBLOCK2MEM_DMA_ENABLE        (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_ENABLE_OFFSET)
#define LITEX_SDBLOCK2MEM_DMA_DONE          (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_DONE_OFFSET)
#define LITEX_SDBLOCK2MEM_DMA_LOOP          (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_LOOP_OFFSET)
#define LITEX_SDBLOCK2MEM_DMA_OFFSET        (LITEX_SDBLOCK2MEM_BASE+LITEX_SDBLOCK2MEM_DMA_OFFSET_OFFSET)

#define LITEX_SDCORE_CMD_ARGUMENT           (LITEX_SDCORE_BASE+LITEX_SDCORE_CMD_ARGUMENT_OFFSET)
#define LITEX_SDCORE_CMD_COMMAND            (LITEX_SDCORE_BASE+LITEX_SDCORE_CMD_COMMAND_OFFSET)
#define LITEX_SDCORE_CMD_SEND               (LITEX_SDCORE_BASE+LITEX_SDCORE_CMD_SEND_OFFSET)
#define LITEX_SDCORE_CMD_RESPONSE           (LITEX_SDCORE_BASE+LITEX_SDCORE_CMD_RESPONSE_OFFSET)
#define LITEX_SDCORE_CMD_EVENT              (LITEX_SDCORE_BASE+LITEX_SDCORE_CMD_EVENT_OFFSET)
#define LITEX_SDCORE_DATA_EVENT             (LITEX_SDCORE_BASE+LITEX_SDCORE_DATA_EVENT_OFFSET)
#define LITEX_SDCORE_BLOCK_LENGTH           (LITEX_SDCORE_BASE+LITEX_SDCORE_BLOCK_LENGTH_OFFSET)
#define LITEX_SDCORE_BLOCK_COUNT            (LITEX_SDCORE_BASE+LITEX_SDCORE_BLOCK_COUNT_OFFSET)

#define LITEX_SDIRQ_STATUS                  (LITEX_SDIRQ_BASE+LITEX_SDIRQ_STATUS_OFFSET)
#define LITEX_SDIRQ_PENDING                 (LITEX_SDIRQ_BASE+LITEX_SDIRQ_PENDING_OFFSET)
#define LITEX_SDIRQ_ENABLE                  (LITEX_SDIRQ_BASE+LITEX_SDIRQ_ENABLE_OFFSET)

#define LITEX_SDMEM2BLOCK_DMA_BASE          (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_BASE_OFFSET)
#define LITEX_SDMEM2BLOCK_DMA_LENGTH        (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_LENGTH_OFFSET)
#define LITEX_SDMEM2BLOCK_DMA_ENABLE        (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_ENABLE_OFFSET)
#define LITEX_SDMEM2BLOCK_DMA_DONE          (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_DONE_OFFSET)
#define LITEX_SDMEM2BLOCK_DMA_LOOP          (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_LOOP_OFFSET)
#define LITEX_SDMEM2BLOCK_DMA_OFFSET        (LITEX_SDMEM2BLOCK_BASE+LITEX_SDMEM2BLOCK_DMA_OFFSET_OFFSET)

#define LITEX_SDPHY_CARD_DETECT             (LITEX_SDPHY_BASE+LITEX_SDPHY_CARD_DETECT_OFFSET)
#define LITEX_SDPHY_CLOCKER_DIVIDER         (LITEX_SDPHY_BASE+LITEX_SDPHY_CLOCKER_DIVIDER_OFFSET)
#define LITEX_SDPHY_INIT_INITIALIZER        (LITEX_SDPHY_BASE+LITEX_SDPHY_INIT_INITIALIZE_OFFSET)
#define LITEX_SDPHY_DATAW_STATUS            (LITEX_SDPHY_BASE+LITEX_SDPHY_DATAW_STATUS_OFFSET)

/* LITEX_SDIO register bit definitions **************************************/

#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_SDIO_H */
