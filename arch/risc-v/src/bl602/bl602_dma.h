/****************************************************************************
 * arch/risc-v/src/bl602/bl602_dma.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_DMA_H
#define __ARCH_RISCV_SRC_BL602_BL602_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_DMA_NCHANNELS    4
#define BL602_DMA_INT_EVT_TC  0 /* Interrupt callback status Transfer Complete */
#define BL602_DMA_INT_EVT_ERR 1 /* Interrupt callback status Error */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
struct bl602_dmaregs_s
{
  uint32_t intstatus;         /* Interrupt status. */
  uint32_t inttcstatus;       /* Interrupt terminal count request status. */
  uint32_t inttcclear;        /* Terminal count request clear. */
  uint32_t interrorstatus;    /* Interrupt error status. */
  uint32_t interrclr;         /* Interrupt error clear. */
  uint32_t rawinttcstatus;    /* Status of the terminal count interrupt prior to masking. */
  uint32_t rawinterrorstatus; /* Status of the error interrupt prior to masking. */
  uint32_t enbldchns;         /* Channel enable status. */
  uint32_t softbreq;          /* Software burst request. */
  uint32_t softsreq;          /* Software single request. */
  uint32_t softlbreq;         /* Software last burst request. */
  uint32_t softlsreq;         /* Software last single request. */
  uint32_t top_config;        /* DMA general configuration. */
  uint32_t sync;              /* DMA request asynchronous setting. */
  uint32_t c0srcaddr;         /* Channel DMA source address. */
  uint32_t c0dstaddr;         /* Channel DMA Destination address. */
  uint32_t c0lli;             /* Channel DMA link list. */
  uint32_t c0control;         /* Channel DMA bus control. */
  uint32_t c0config;          /* Channel DMA configuration. */
  uint32_t c1srcaddr;         /* Channel DMA source address. */
  uint32_t c1dstaddr;         /* Channel DMA Destination address. */
  uint32_t c1lli;             /* Channel DMA link list. */
  uint32_t c1control;         /* Channel DMA bus control. */
  uint32_t c1config;          /* Channel DMA configuration. */
  uint32_t c2srcaddr;         /* Channel DMA source address. */
  uint32_t c2dstaddr;         /* Channel DMA Destination address. */
  uint32_t c2lli;             /* Channel DMA link list. */
  uint32_t c2control;         /* Channel DMA bus control. */
  uint32_t c2config;          /* Channel DMA configuration. */
  uint32_t c3srcaddr;         /* Channel DMA source address. */
  uint32_t c3dstaddr;         /* Channel DMA Destination address. */
  uint32_t c3lli;             /* Channel DMA link list. */
  uint32_t c3control;         /* Channel DMA bus control. */
  uint32_t c3config;          /* Channel DMA configuration. */
};
#endif

/* Configuration for the LLI Control entry, defined as matching
 * the DMA channel control register.
 */

begin_packed_struct struct bl602_dma_ctrl_s
{
  uint32_t transfer_size  : 12;
  uint32_t src_burst_size : 3;
  uint32_t dst_burst_size : 3;
  uint32_t src_width      : 3;
  uint32_t dst_width      : 3;
  uint32_t sld            : 1;
  uint32_t rsvd           : 1;
  uint32_t src_increment  : 1;
  uint32_t dst_increment  : 1;
  uint32_t protect        : 3;
  uint32_t tc_int_en      : 1;
} end_packed_struct;

/* Configuration for the LLI packed data.
 * This must be aligned to uint32 pointer.
 */

begin_packed_struct struct aligned_data(4) bl602_lli_ctrl_s
{
  uint32_t src_addr;
  uint32_t dst_addr;
  uint32_t next_lli;                /* Address for next LLI entry */
  struct bl602_dma_ctrl_s dma_ctrl; /* Control register config for entry */
} end_packed_struct;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.
 *
 * Input Parameters:
 *   channel - Refers to the DMA channel
 *   status  - A bit encoded value that provides the completion status.
 *             See the BL602_DMA_INT_EVT_* definitions above.
 *   arg     - A user-provided value.
 */

typedef void (*bl602_dma_callback_t) \
  (uint8_t channel, uint8_t status, void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_dma_channel_request
 *
 * Description:
 *   Allocate a new DMA channel.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0-3: DMA channel
 *   -1: Failed
 *
 ****************************************************************************/

int8_t bl602_dma_channel_request(bl602_dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: bl602_dma_channel_release
 *
 * Description:
 *   Release a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_release(uint8_t channel);

/****************************************************************************
 * Name: bl602_dma_channel_start
 *
 * Description:
 *   Start a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_start(uint8_t channel_id);

/****************************************************************************
 * Name: bl602_dma_channel_stop
 *
 * Description:
 *   Stop a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_stop(uint8_t channel_id);

/****************************************************************************
 * Name: bl602_dma_init
 *
 * Description:
 *   Initialize DMA controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_init(void);

/****************************************************************************
 * Name: bl602_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void bl602_dmasample(struct bl602_dmaregs_s *regs);
#else
#  define bl602_dmasample(regs)
#endif

/****************************************************************************
 * Name: bl602_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void bl602_dmadump(const struct bl602_dmaregs_s *regs, const char *msg);
#else
#  define bl602_dmadump(regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_DMA_H */
