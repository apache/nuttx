/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_dma.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_DMA_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_DMA_NUM_CHANNELS (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct mpfs_dma_channel_config
{
  uint64_t src_addr;        /* source address */
  uint64_t dest_addr;       /* destination address */
  uint64_t num_bytes;       /* Number of bytes to be transferred. (Base-2) */
  uint8_t  enable_done_int; /* enable transfer complete interrupt */
  uint8_t  enable_err_int;  /* enable transfer error interrupt */
  uint8_t  repeat;          /* repeat the transaction */
  uint8_t  force_order;     /* Enforces strict ordering by only
                             * allowing one of each transfer type
                             * in-flight at a time */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN int mpfs_dma_setup_transfer(unsigned int channel,
                                   struct mpfs_dma_channel_config *config);
EXTERN int mpfs_dma_set_transaction_size(unsigned int channel,
                                         uint8_t write_size,
                                         uint8_t read_size);
EXTERN int mpfs_dma_start(unsigned int channel);
EXTERN uint32_t mpfs_dma_get_transfer_type(unsigned int channel);
EXTERN uint64_t mpfs_dma_get_bytes_remaining(unsigned int channel);
EXTERN uint64_t mpfs_dma_get_current_destination(unsigned int channel);
EXTERN uint64_t mpfs_dma_get_current_source(unsigned int channel);
EXTERN int mpfs_dma_get_complete_status(unsigned int channel);
EXTERN int mpfs_dma_get_error_status(unsigned int channel);
EXTERN int mpfs_dma_clear_complete_status(unsigned int channel);
EXTERN int mpfs_dma_clear_error_status(unsigned int channel);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_DMA_H */
