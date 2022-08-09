/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpdma.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LP43_GPDMA_H
#define __ARCH_ARM_SRC_LPC43XX_LP43_GPDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc43_gpdma.h"

#ifdef CONFIG_LPC43_GPDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when
 * CONFIG DEBUG_DMA is selected
 */

#ifdef CONFIG_DEBUG_DMA
struct lpc43_dmaglobalregs_s
{
  /* Global Registers */

  uint32_t intst;       /* DMA Interrupt Status Register */
  uint32_t inttcst;     /* DMA Interrupt Terminal Count Request Status Register */
  uint32_t interrst;    /* DMA Interrupt Error Status Register */
  uint32_t rawinttcst;  /* DMA Raw Interrupt Terminal Count Status Register */
  uint32_t rawinterrst; /* DMA Raw Error Interrupt Status Register */
  uint32_t enbldchns;   /* DMA Enabled Channel Register */
  uint32_t softbreq;    /* DMA Software Burst Request Register */
  uint32_t softsreq;    /* DMA Software Single Request Register */
  uint32_t softlbreq;   /* DMA Software Last Burst Request Register */
  uint32_t softlsreq;   /* DMA Software Last Single Request Register */
  uint32_t config;      /* DMA Configuration Register */
  uint32_t sync;        /* DMA Synchronization Register */
};

struct lpc43_dmachanregs_s
{
  /* Channel Registers */

  uint32_t srcaddr;  /* DMA Channel Source Address Register */
  uint32_t destaddr; /* DMA Channel Destination Address Register */
  uint32_t lli;      /* DMA Channel Linked List Item Register */
  uint32_t control;  /* DMA Channel Control Register */
  uint32_t config;   /* DMA Channel Configuration Register */
};

struct lpc43_dmaregs_s
{
  /* Global Registers */

  struct lpc43_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct lpc43_dmachanregs_s   ch;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lpc43_dmachannel(void);

/****************************************************************************
 * Name: lpc43_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc43_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc43_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: lpc43_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc43_dmasetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                   uint32_t srcaddr, uint32_t destaddr, size_t nbytes);

/****************************************************************************
 * Name: lpc43_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc43_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: lpc43_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc43_dmastop() is called, the DMA channel is
 *   reset and lpc43_dmasetup() must be called before lpc43_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void lpc43_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: lpc43_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc43_dmasample(DMA_HANDLE handle, struct lpc43_dmaregs_s *regs);
#else
#  define lpc43_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: lpc43_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc43_dmadump(DMA_HANDLE handle, const struct lpc43_dmaregs_s *regs,
                   const char *msg);
#else
#  define lpc43_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC43_GPDMA */
#endif /* __ARCH_ARM_SRC_LPC43XX_LP43_GPDMA_H */
