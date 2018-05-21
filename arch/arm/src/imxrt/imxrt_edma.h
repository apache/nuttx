/************************************************************************************
 * arch/arm/src/imxrt/imxrt_dmac.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Portions of the eDMA logic derive from NXP sample code which has a compatible
 * BSD 3-clause license:
 *
 *   Copyright (c) 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2017 NXP
 *   All rights reserved
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include "chip/imxrt_edma.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMACH_HANDLE;
typedef void (*dma_callback_t)(DMACH_HANDLE handle, void *arg, int result);

/* eDMA transfer type */

enum imxrt_edma_xfrtype_e
{
  eDMA_MEMORY2MEMORY = 0,  /* Transfer from memory to memory */
  eDMA_PERIPH2MEMORY,      /* Transfer from peripheral to memory */
  eDMA_MEMORY2PERIPH,      /* Transfer from memory to peripheral */
};

/* This structure holds the source/destination transfer attribute configuration. */

struct imxrt_edma_xfrconfig_s
{
    uint32_t saddr;    /* Source data address. */
    uint32_t daddr;    /* Destination data address. */
    uint8_t  ssize;    /* Source data transfer size (see TCD_ATTR_SIZE_* definitions in chip/. */
    uint8_t  dsize;    /* Destination data transfer size. */
    int16_t soff;      /* Sign-extended offset for current source address. */
    int16_t doff;      /* Sign-extended offset for current destination address. */
    uint16_t iter;     /* Major loop iteration count. */
#ifdef CONFIG_IMXRT_EDMA_EMLIM
    uint16_t nbytes;   /* Bytes to transfer in a minor loop */
#else
    uint32_t nbytes;   /* Bytes to transfer in a minor loop */
#endif
};

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct imxrt_dmaregs_s
{
  uint8_t chan;      /* Sampled channel */

  /* eDMA Global Registers */

  uint32_t cr;       /* Control */
  uint32_t es;       /* Error Status */
  uint32_t erq;      /* Enable Request */
  uint32_t req;      /* Interrupt Request */
  uint32_t err;      /* Error */
  uint32_t hrs;      /* Hardware Request Status */
  uint32_t ears;     /* Enable Asynchronous Request in Stop */

  /* eDMA Channel registers */

  uint8_t dchpri;    /* Channel priority */

  /* eDMA TCD */

  uint32_t saddr;    /* TCD Source Address */
  uint16_t soff;     /* TCD Signed Source Address Offset */
  uint16_t attr;     /* TCD Transfer Attributes */
  uint32_t nbml;     /* TCD Signed Minor Loop Offset / Byte Count */
  uint32_t slast;    /* TCD Last Source Address Adjustment */
  uint32_t daddr;    /* TCD Destination Address */
  uint16_t doff;     /* TCD Signed Destination Address Offset */
  uint16_t citer;    /* TCD Current Minor Loop Link, Major Loop Count */
  uint32_t dlastsga; /* TCD Last Destination Address Adjustment/Scatter Gather Address */
  uint16_t csr;      /* TCD Control and Status */
  uint16_t biter;    /* TCD Beginning Minor Loop Link, Major Loop Count */

  /* DMAMUX registers */

  uint32_t dmamux;   /* Channel configuration */
};
#endif /* CONFIG_DEBUG_DMA */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_dmachannel
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel,
 *   initializes the DMAMUX for the channel, then gives the caller exclusive
 *   access to the DMA channel.
 *
 * Input Parameters:
 *   dmamux - DMAMUX configuration see DMAMUX channel configuration register
 *            bit-field definitions in chip/imxrt_dmamux.h.  Settings include:
 *
 *            DMAMUX_CHCFG_SOURCE     Chip-specific DMA source (required)
 *            DMAMUX_CHCFG_AON        DMA Channel Always Enable (optional)
 *            DMAMUX_CHCFG_TRIG       DMA Channel Trigger Enable (optional)
 *            DMAMUX_CHCFG_ENBL       DMA Mux Channel Enable (required)
 *
 *            A value of zero will disable the DMAMUX channel.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ************************************************************************************/

DMACH_HANDLE imxrt_dmachannel(uint32_t dmamux);

/************************************************************************************
 * Name: imxrt_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must NEVER be
 *   used again until imxrt_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void imxrt_dmafree(DMACH_HANDLE handle);

/****************************************************************************
 * Name: imxrt_tcd_alloc
 *
 * Description:
 *   Allocate an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
struct imxrt_edmatcd_s *imxrt_tcd_alloc(void);
#endif

/****************************************************************************
 * Name: imxrt_tcd_free()
 *
 * Description:
 *   Free an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
void imxrt_tcd_free(struct imxrt_edmatcd_s *tcd);
#endif

/************************************************************************************
 * Name: imxrt_dmach_reset
 *
 * Description:
 *   Sets all TCD registers to default values..
 *
 *   NOTE:  This function enables the auto stop request feature.
 *
 ************************************************************************************/

void imxrt_dmach_reset(DMACH_HANDLE handle);

/*******************************************************************************
 * Name: imxrt_dmach_initconfig
 *
 * Description:
 *   This function initializes the transfer configuration structure according
 *   to the user-provided input configuration.
 *
 * Input Parameters:
 *   saddr     - eDMA transfer source address.
 *   srcwidth  - eDMA transfer source address width(bytes).
 *   daddr     - eDMA transfer destination address.
 *   destwidth - eDMA transfer destination address width(bytes).
 *   reqsize   - eDMA transfer bytes per channel request.
 *   nbytes    - eDMA transfer bytes to be transferred.
 *   type      - eDMA transfer type.
 *   config    - The user configuration structure of type struct
 *               imxrt_edma_xfrconfig_s.
 *
 *   NOTE: The data address and the data width must be consistent. For example,
 *   if the SRC is 4 bytes, the source address must be 4 bytes aligned, or it
 *   results in  source address error (SAE).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int imxrt_dmach_initconfig(void *saddr, uint32_t srcwidth,
                           void *daddr, uint32_t destwidth,
                           uint32_t reqsize, uint32_t nbytes,
                           edma_transfer_type_t type,
                           struct imxrt_edma_xfrconfig_s *config);

/*******************************************************************************
 * Name: imxrt_dmach_setconfig
 *
 * Description:
 *   This function configures the transfer attribute, including source address,
 *   destination address, transfer size, address offset, and so on. It also
 *   configures the scatter gather feature if the user supplies the TCD address.
 *
 *   Example:
 *
 *      edma_transfer_t config;
 *      struct imxrt_edmatcd_s tcd;
 *      config.saddr = ..;
 *      config.daddr = ..;
 *      ...
 *      dmach = imxrt_dmachannel(dmamux);
 *      ...
 *      tcd = imxrt_tcd_alloc(dmach)
 *      ...
 *      imxrt_dmach_setconfig(dmach, &config, &tcd);
 *
 * Input Parameters:
 *   handle  - DMA channel handle created by imxrt_dmachannel()
 *   channel - eDMA channel number.
 *   config  - Pointer to eDMA transfer configuration structure.
 *   next    - Points to a TCD structure previously allocated via
 *             imxrt_tcd_alloc(). 'next' can be NULL if the caller does not
 *             wish to enable scatter/gather feature.
 *
 *   NOTE: If 'next' is not NULL, it means scatter gather feature is enabled
 *         and DREQ bit is cleared in the previous transfer configuration.
 *         That bit was set in imxrt_dmach_reset().
 *
 ******************************************************************************/

#ifdef CONFIG_IMXRT_EDMA_NTCD > 0
void imxrt_dmach_setconfig(DMACH_HANDLE handle,
                           const struct imxrt_edma_xfrconfig_s *config,
                           struct imxrt_edmatcd_s *next);
#else
void imxrt_dmach_setconfig(DMACH_HANDLE handle,
                           const struct imxrt_edma_xfrconfig_s *config);
#endif

/************************************************************************************
 * Name: imxrt_dmasetup
 *
 * Description:
 *   Configure DMA for one Rx (peripheral-to-memory) or Rx (memory-to-peripheral)
 *   transfer of one buffer.
 *
 *   TODO:  This function needs to be called multiple times to handle multiple,
 *   discontinuous transfers.
 *
 ************************************************************************************/

int imxrt_dmasetup(DMACH_HANDLE handle, uint32_t saddr, uint32_t daddr,
                   size_t nbytes, uint32_t chflags);

/************************************************************************************
 * Name: imxrt_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ************************************************************************************/

int imxrt_dmastart(DMACH_HANDLE handle, dma_callback_t callback, void *arg);

/************************************************************************************
 * Name: imxrt_dmastop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmastop() is called, the DMA channel is reset and
 *   imxrt_dmarx/txsetup() must be called before imxrt_dmastart() can be called again
 *
 ************************************************************************************/

void imxrt_dmastop(DMACH_HANDLE handle);

/************************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMACH_HANDLE handle, struct imxrt_dmaregs_s *regs);
#else
#  define imxrt_dmasample(handle,regs)
#endif

/************************************************************************************
 * Name: imxrt_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmadump(const struct imxrt_dmaregs_s *regs, const char *msg);
#else
#  define imxrt_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H */
