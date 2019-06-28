/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz-dma.h
 *
 *   Copyright (C) 2015, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

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
 *     If no start irq is specified then the a force start is performed.
 *
 * 4. Stop and free the channel
 *
 *    The DMA channel can be aborted if an abort irq is set.
 *    Alternatively, call to pic32mz_dma_stop will force the abort.
 *
 *    pic32mz_dma_free will free the channel and make it available.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>

#include "hardware/pic32mz-dma.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Interrupt type arguments for pic32mz_dma_intctrl. */

#define PIC32MZ_DMA_INT_SRCDONE     DMACH_INT_CHSDIE
#define PIC32MZ_DMA_INT_SRCHALF     DMACH_INT_CHSHIE
#define PIC32MZ_DMA_INT_DESTDONE    DMACH_INT_CHDDIE
#define PIC32MZ_DMA_INT_DESTHALF    DMACH_INT_CHDHIE
#define PIC32MZ_DMA_INT_BLOCKDONE   DMACH_INT_CHBCIE
#define PIC32MZ_DMA_INT_CELLDONE    DMACH_INT_CHCCIE
#define PIC32MZ_DMA_INT_ABORT       DMACH_INT_CHTAIE
#define PIC32MZ_DMA_INT_ERR         DMACH_INT_CHERIE
#define PIC32MZ_DMA_INT_DISABLE     (0)

/* This is used when setting a channel with no start/abort event */

#define PIC32MZ_DMA_NOEVENT (NR_IRQS + 1)

/*******************************************************************************
 * Public Types
 *
 ******************************************************************************/

#ifndef __ASSEMBLY__

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

enum pic32mz_dma_chmode_e
{
  PIC32MZ_DMA_MODE_BASIC   = 1 << 0U, /* Basic transfert mode */
  PIC32MZ_DMA_MODE_AUTOEN  = 1 << 1U, /* Channel Auto-Enable mode */
  PIC32MZ_DMA_MODE_PMATCH  = 1 << 2U, /* Pattern Match termination mode */
  PIC32MZ_DMA_MODE_CHCHAIN = 1 << 3U, /* Channel chaining mode */
  PIC32MZ_DMA_MODE_SFM     = 1 << 4U  /* Special Function Module mode */
};

/* This structure holds the channel's configuration */

struct pic32mz_dma_chcfg_s
{
  uint8_t priority;                 /* Channel's priority (0..3) */
  uint8_t startirq;                 /* Start event */
  uint8_t abortirq;                 /* Abort event */
  uint8_t event;                    /* Interrupt event */
  enum pic32mz_dma_chmode_e mode;   /* Channel's mode of operation */
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

/*******************************************************************************
 * Public Data
 ******************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Name: pic32mz_dma_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and gives
 *   the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.
 *   This function can fail only if no DMA channel is available.
 *
 ******************************************************************************/

DMA_HANDLE pic32mz_dma_alloc(const struct pic32mz_dma_chcfg_s *cfg);

/*******************************************************************************
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
 ******************************************************************************/

void pic32mz_dma_free(DMA_HANDLE handle);

/*******************************************************************************
 * Name: pic32mz_dma_xfrsetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ******************************************************************************/

int pic32mz_dma_xfrsetup(DMA_HANDLE handle,
                         FAR const struct pic32mz_dma_xfrcfg_s *cfg);

/*******************************************************************************
 * Name: pic32mz_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 ******************************************************************************/

int pic32mz_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/*******************************************************************************
 * Name: pic32mz_dma_stop
 *
 * Description:
 *   Cancel the DMA.
 *   After pic32mz_dma_stop() is called, the DMA channel is reset
 *   and pic32mz_dma_xfrsetup() must be called before pic32mz_dma_start()
 *   can be called again.
 *
 ******************************************************************************/

void pic32mz_dma_stop(DMA_HANDLE handle);

/*******************************************************************************
 * Name: pic32mz_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 ******************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_sample(DMA_HANDLE handle, struct pic32mz_dmaregs_s *regs);
#else
#  define pic32mz_dma_sample(handle,regs)
#endif

/*******************************************************************************
 * Name: pic32mz_dma_dump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ******************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_dump(DMA_HANDLE handle, const struct pic32mz_dmaregs_s *regs,
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

