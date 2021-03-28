/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_dma.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H
#define __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_dma.h"

#ifdef CONFIG_LPC54_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef void (*dma_callback_t)(int ch, void *arg, int result);

/* The following is used for sampling DMA registers when
 * CONFIG DEBUG_DMA is selected
 */

#ifdef CONFIG_DEBUG_DMA
struct lpc54_dmaglobalregs_s
{
  /* Global Registers */

  uint32_t ctrl;         /* DMA control */
  uint32_t intstat;      /* DMA Interrupt status */
  uint32_t srambase;     /* SRAM address of the channel configuration table */
  uint32_t enableset0;   /* DMA Channel enable read and set */
  uint32_t active0;      /* DMA Channel active status */
  uint32_t busy0;        /* DMA Channel busy status */
  uint32_t errint0;      /* DMA Error interrupt status */
  uint32_t intenset0;    /* DMA Interrupt enable read and set */
  uint32_t inta0;        /* DMA Interrupt A status */
  uint32_t intb0;        /* DMA Interrupt B status */
};

struct lpc54_dmachanregs_s
{
  /* Channel Registers */

  uint32_t cfg;          /* DMA Configuration register */
  uint32_t ctlstat;      /* DMA Control and status register */
  uint32_t xfercfg;      /* DMA Transfer configuration register */
};

struct lpc54_dmaregs_s
{
  /* Global Registers */

  struct lpc54_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct lpc54_dmachanregs_s   ch;
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
 * Name: lpc54_dma_setup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 * Input Parameters:
 *   ch      - DMA channel number
 *   cfg     - The content of the DMA channel configuration register.  See
 *             peripheral channel definitions in chip/lpc54_dma.h.  The
 *             caller must provide all fields:  PERIPHREQEN, TRIGPOL,
 *             TRIGTYPE, TRIGBURST, BURSTPOWER, SRCBURSTWRAP, DSTBURSTWRAP,
 *             and CHPRIORITY.
 *   xfrcfg  - The content of the DMA channel configuration register.  See
 *             peripheral channel definitions in chip/lpc54_dma.h.  The
 *             caller must provide all fields:  WIDTH, SRCINC, and DSTINC.\
 *             All of fields are managed by the DMA driver
 *   trigsrc - See input mux DMA trigger ITRIG_INMUX_* definitions in
 *             chip/lpc54_inputmux.h.
 *   srcaddr  - Source address of the DMA transfer
 *   dstaddr  - Destination address of the DMA transfer
 *   nbytes   - Number of bytes to transfer
 *
 ****************************************************************************/

int lpc54_dma_setup(int ch, uint32_t cfg, uint32_t xfrcfg, uint8_t trigsrc,
                    uintptr_t srcaddr, uintptr_t dstaddr, size_t nbytes);

/****************************************************************************
 * Name: lpc54_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc54_dmastart(int ch, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: lpc54_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc54_dmastop() is called, the DMA channel is
 *   reset and lpc54_dmasetup() must be called before lpc54_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void lpc54_dmastop(int ch);

/****************************************************************************
 * Name: lpc54_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmasample(int ch, struct lpc54_dmaregs_s *regs);
#else
#  define lpc54_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: lpc54_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmadump(int ch, const struct lpc54_dmaregs_s *regs,
                   const char *msg);
#else
#  define lpc54_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC54_DMA */
#endif /* __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H */
