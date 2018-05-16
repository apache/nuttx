/************************************************************************************
 * arch/arm/src/imxrt/imxrt_dmac.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

/* DMA ******************************************************************************/

/* Flags used to characterize the DMA channel.  The naming convention is that one
 * side is the peripheral and the other is memory (however, the interface could still
 * be used if, for example, both sides were memory although the naming would be
 * awkward)
 *
 * .... .... .... ....  .... CCCC GGBA DDSS
 *
 * REVISIT:  Initially, only vanilla Rx/Tx DMA block transfers are supported.
 */

/* Source transfer size:
 *
 * .... .... .... ....  .... .... .... ..SS
 */

#define DMACH_FLAG_SSIZE_SHIFT        (0)      /* Bits 0-1: Source transfer size */
#define DMACH_FLAG_SSIZE_MASK         (7 << DMACH_FLAG_SSIZE_SHIFT)
#  define DMACH_FLAG_SSIZE_8BIT       (TCD_ATTR_SIZE_8BIT   << DMACH_FLAG_SSIZE_SHIFT) /* 8-bit */
#  define DMACH_FLAG_SSIZE_16BIT      (TCD_ATTR_SIZE_16BIT  << DMACH_FLAG_SSIZE_SHIFT) /* 16-bit */
#  define DMACH_FLAG_SSIZE_32BIT      (TCD_ATTR_SIZE_32BIT  << DMACH_FLAG_SSIZE_SHIFT) /* 32-bit */
#  define DMACH_FLAG_SSIZE_64BIT      (TCD_ATTR_SIZE_64BIT  << DMACH_FLAG_SSIZE_SHIFT) /* 64-bit */
#  define DMACH_FLAG_SSIZE_256BIT     (TCD_ATTR_SIZE_256BIT << DMACH_FLAG_SSIZE_SHIFT) /* 32-byte burst */

/* Destination transfer size:
 *
 * .... .... .... ....  .... .... .... DD..
 */

#define DMACH_FLAG_DSIZE_SHIFT        (2)      /* Bits 2-3: Destination transfer size */
#define DMACH_FLAG_DSIZE_MASK         (7 << DMACH_FLAG_DSIZE_SHIFT)
#  define EMACH_FLAG_DSIZE_8BIT       (TCD_ATTR_SIZE_8BIT   << DMACH_FLAG_DSIZE_SHIFT) /* 8-bit */
#  define EMACH_FLAG_DSIZE_16BIT      (TCD_ATTR_SIZE_16BIT  << DMACH_FLAG_DSIZE_SHIFT) /* 16-bit */
#  define EMACH_FLAG_DSIZE_32BIT      (TCD_ATTR_SIZE_32BIT  << DMACH_FLAG_DSIZE_SHIFT) /* 32-bit */
#  define EMACH_FLAG_DSIZE_64BIT      (TCD_ATTR_SIZE_64BIT  << DMACH_FLAG_DSIZE_SHIFT) /* 64-bit */
#  define EMACH_FLAG_DSIZE_256BIT     (TCD_ATTR_SIZE_256BIT << DMACH_FLAG_DSIZE_SHIFT) /* 32-byte burst */

/* Arbitration:
 *
 * .... .... .... ....  .... .... ..BA ....
 */

#define DMACH_FLAG_CHRR               (1 << 4)  /* Bit 4:  Round Robin Channel Arbitration */
#define DMACH_FLAG_GRPRR              (1 << 5)  /* Bit 5:  Round Robin Group Arbitration */

/* DMA Priorities:
 *
 * .... .... .... ....  .... CCCC GG.. ....
 */

#define DMACH_FLAG_GPPRI_SHIFT             (6)       /* Bits 6-7: Channel Group Priority */
#define DMACH_FLAG_GRPPRI_MASK             (3 << DMACH_FLAG_GPPRI_SHIFT)
#  define DMACH_FLAG_GRPPRI(n)             ((uint32_t)(n) << DMACH_FLAG_GPPRI_SHIFT)

#define DMACH_FLAG_CHPRI_SHIFT             (8)       /* Bits 8-11: Channel Arbitration Priority */
#define DMACH_FLAG_CHPRI_MASK              (15 << DMACH_FLAG_CHPRI_SHIFT)
#  define DMACH_FLAG_CHPRI(n)              ((uint32_t)(n) << DMACH_FLAG_CHPRI_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

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
  uint32_t ears;     /*  Enable Asynchronous Request in Stop */

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
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel then gives the
 *   caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is the
 *   'peripheral' and the other is 'memory'.  However, the interface could still
 *   be used if, for example, both sides were memory although the naming would be
 *   awkward.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void* DMA
 *   channel handle.  NULL is returned on any failure.
 *
 ************************************************************************************/

DMA_HANDLE imxrt_dmachannel(void);

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

void imxrt_dmafree(DMA_HANDLE handle);

/************************************************************************************
 * Name: imxrt_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This function
 *   may be called multiple times to handle large and/or discontinuous transfers.
 *   Calls to imxrt_dmatxsetup() and imxrt_dmarxsetup() must not be intermixed on the
 *   same transfer, however.
 *
 ************************************************************************************/

int imxrt_dmatxsetup(DMA_HANDLE handle, uint8_t pchan, uint32_t maddr, size_t nbytes,
                     uint32_t chflags);

/************************************************************************************
 * Name: imxrt_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This function
 *   may be called multiple times to handle large and/or discontinuous transfers.
 *   Calls to imxrt_dmatxsetup() and imxrt_dmarxsetup() must not be intermixed on the
 *   same transfer, however.
 *
 ************************************************************************************/

int imxrt_dmarxsetup(DMA_HANDLE handle, uint8_t pchan, uint32_t maddr, size_t nbytes,
                     uint32_t chflags);

/************************************************************************************
 * Name: imxrt_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ************************************************************************************/

int imxrt_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/************************************************************************************
 * Name: imxrt_dmastop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmastop() is called, the DMA channel is reset and
 *   imxrt_dmarx/txsetup() must be called before imxrt_dmastart() can be called again
 *
 ************************************************************************************/

void imxrt_dmastop(DMA_HANDLE handle);

/************************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMA_HANDLE handle, struct imxrt_dmaregs_s *regs);
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
