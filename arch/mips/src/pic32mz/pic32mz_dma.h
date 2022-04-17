/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_dma.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_DMA_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_DMA_H

/* General Usage:
 *
 * 1. Allocate a DMA channel
 *
 *     DMA_HANDLE handle;
 *     handle = pic32mz_dma_alloc(chcfg);
 *
 *  where chcfg is the channel's configuration (see struct pic32mz_dma_chcfg)
 *
 * 2. Setup the transfer
 *
 *      struct pic32mz_dma_xfrcfg_s xfrcfg;
 *      xfrcfg.srcaddr  = ...;
 *      xfrcfg.destaddr = ...;
 *      etc.
 *
 *      pic32mz_dma_xfrsetup(handle, xfrcfg);
 *
 * 3. Start the transfer
 *
 *     pic32mz_dma_start(handle, callback, arg);
 *
 *     If a start irq is set this function will only enable the channel.
 *     The transfer will be controlled by the start irq.
 *     If start irq is set to PIC32MZ_DMA_NOIRQ then a force start is
 *     performed.
 *
 * 4. Stop and free the channel
 *
 *    The DMA channel can be aborted if an abort irq is set.
 *    Alternatively, call to pic32mz_dma_stop will force the abort.
 *
 *    pic32mz_dma_free will free the channel and make it available.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is used when setting a channel with no start/abort irq */

#define PIC32MZ_DMA_NOIRQ (NR_IRQS + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/* DMA channel modes, arguments for pic32mz_dma_mode. */

enum pic32mz_dma_chmode_e
{
  PIC32MZ_DMA_MODE_BASIC   = 1 << 0U, /* Basic transfer mode */
  PIC32MZ_DMA_MODE_AUTOEN  = 1 << 1U, /* Channel Auto-Enable mode */
  PIC32MZ_DMA_MODE_PMATCH  = 1 << 2U, /* Pattern Match termination mode */
  PIC32MZ_DMA_MODE_CHCHAIN = 1 << 3U, /* Channel chaining mode */
  PIC32MZ_DMA_MODE_SFM     = 1 << 4U  /* Special Function Module mode */
};

/* Interrupt type arguments for pic32mz_dma_intctrl. */

enum pic32mz_dma_event_e
{
  PIC32MZ_DMA_INT_DISABLE   = 0U,
  PIC32MZ_DMA_INT_ADDRERR   = 1 << 0U,  /* Address error interrupt */
  PIC32MZ_DMA_INT_ABORT     = 1 << 1U,  /* Transfer abort interrupt */
  PIC32MZ_DMA_INT_CELLDONE  = 1 << 2U,  /* Cell transfer complete interrupt */
  PIC32MZ_DMA_INT_BLOCKDONE = 1 << 3U,  /* Block transfer complete interrupt */
  PIC32MZ_DMA_INT_DESTHALF  = 1 << 4U,  /* Destination half full interrupt */
  PIC32MZ_DMA_INT_DESTDONE  = 1 << 5U,  /* Destination done interrupt */
  PIC32MZ_DMA_INT_SRCHALF   = 1 << 6U,  /* Source half full interrupt */
  PIC32MZ_DMA_INT_SRCDONE   = 1 << 7U   /* Source done interrupt */
};

/* This structure holds the channel's configuration */

struct pic32mz_dma_chcfg_s
{
  uint8_t priority; /* Channel's priority (0..3) */
  uint8_t startirq; /* Start event */
  uint8_t abortirq; /* Abort event */
  uint8_t event;    /* Interrupt events (enum pic32mz_dma_event_e) */
  uint8_t mode;     /* Channel's modes (enum pic32mz_dma_chmode_e) */
};

/* This structure holds a transfer's configuration */

struct pic32mz_dma_xfrcfg_s
{
  uint32_t srcaddr;  /* Source address */
  uint32_t destaddr; /* Destination address */
  uint16_t srcsize;  /* Source size */
  uint16_t destsize; /* Destination size */
  uint16_t cellsize; /* Cell size */
};

/* The following is used for sampling DMA registers when CONFIG_DEBUG_DMA
 * is selected
 */

#ifdef CONFIG_DEBUG_DMA
struct pic32mz_dmagblregs_s
{
  /* Global Registers */

  uint32_t con;
  uint32_t stat;
  uint32_t addr;
};

struct pic32mz_dmacrcregs_s
{
  /* CRC Registers */

  uint32_t con;
  uint32_t data;
  uint32_t xor;
};

struct pic32mz_dmachanregs_s
{
  /* Channel Registers */

  uint32_t con;
  uint32_t econ;
  uint32_t intcon;
  uint32_t ssa;
  uint32_t dsa;
  uint32_t ssiz;
  uint32_t dsiz;
  uint32_t sptr;
  uint32_t dptr;
  uint32_t csiz;
  uint32_t cptr;
  uint32_t dat;
};

struct pic32mz_dmaregs_s
{
  /* Global Registers */

  struct pic32mz_dmagblregs_s  gbl;

  /* CRC Registers */

  struct pic32mz_dmacrcregs_s  crc;

  /* Channel Registers */

  struct pic32mz_dmachanregs_s ch;
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_dma_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.
 *   This function can fail only if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE pic32mz_dma_alloc(const struct pic32mz_dma_chcfg_s *cfg);

/****************************************************************************
 * Name: pic32mz_dma_free
 *
 * Description:
 *   Release a DMA channel.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again until
 *   pic32mz_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pic32mz_dma_free(DMA_HANDLE handle);

/****************************************************************************
 * Name: pic32mz_dma_chcfg
 *
 * Description:
 *   Configure a DMA channel.
 *   This config can be done during alloc, however if reconfig is needed,
 *   this functions should be used.
 *
 ****************************************************************************/

int pic32mz_dma_chcfg(DMA_HANDLE handle,
                      const struct pic32mz_dma_chcfg_s *cfg);

/****************************************************************************
 * Name: pic32mz_dma_xfrsetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int pic32mz_dma_xfrsetup(DMA_HANDLE handle,
                         const struct pic32mz_dma_xfrcfg_s *cfg);

/****************************************************************************
 * Name: pic32mz_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int pic32mz_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: pic32mz_dma_stop
 *
 * Description:
 *   Cancel the DMA.
 *   After pic32mz_dma_stop() is called, the DMA channel is reset
 *   and pic32mz_dma_xfrsetup() must be called before pic32mz_dma_start()
 *   can be called again.
 *
 ****************************************************************************/

void pic32mz_dma_stop(DMA_HANDLE handle);

/****************************************************************************
 * Name: pic32mz_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_sample(DMA_HANDLE handle, struct pic32mz_dmaregs_s *regs);
#else
#  define pic32mz_dma_sample(handle,regs)
#endif

/****************************************************************************
 * Name: pic32mz_dma_dump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_dump(DMA_HANDLE handle,
                      const struct pic32mz_dmaregs_s *regs,
                      const char *msg);
#else
#  define pic32mz_dma_dump(handle,regs,msg)
#endif

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_DMA_H */
