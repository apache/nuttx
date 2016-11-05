/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-dma.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct pic32mx_dmaglobalregs_s
{
  /* Global Registers */
#warning "Missing definitions"
};

struct pic32mx_dmachanregs_s
{
  /* Channel Registers */
#warning "Missing definitions"
};

struct pic32mx_dmaregs_s
{
  /* Global Registers */

  struct pic32mx_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct pic32mx_dmachanregs_s   ch;
};
#endif

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
 * Name: pic32mx_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmainitilaize(void);
#endif

/************************************************************************************
 * Name: pic32mx_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and gives the
 *  caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel handle.  NULL
 *   is returned on any failure.  This function can fail only if no DMA channel is
 *   available.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
DMA_HANDLE pic32mx_dmachannel(void);
#endif

/************************************************************************************
 * Name: pic32mx_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must NEVER be
 *   used again until pic32mx_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmafree(DMA_HANDLE handle);
#endif

/************************************************************************************
 * Name: pic32mx_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
int pic32mx_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                       uint32_t srcaddr, uint32_t destaddr, size_t nbytes);
#endif

/************************************************************************************
 * Name: pic32mx_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
int pic32mx_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/************************************************************************************
 * Name: pic32mx_dmastop
 *
 * Description:
 *   Cancel the DMA.  After pic32mx_dmastop() is called, the DMA channel is reset
 *   and pic32mx_dmasetup() must be called before pic32mx_dmastart() can be called
 *   again
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmastop(DMA_HANDLE handle);
#endif

/************************************************************************************
 * Name: pic32mx_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
#ifdef CONFIG_DEBUG_DMA
void pic32mx_dmasample(DMA_HANDLE handle, struct pic32mx_dmaregs_s *regs);
#else
#  define pic32mx_dmasample(handle,regs)
#endif
#endif

/************************************************************************************
 * Name: pic32mx_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
#ifdef CONFIG_DEBUG_DMA
void pic32mx_dmadump(DMA_HANDLE handle, const struct pic32mx_dmaregs_s *regs,
                     const char *msg);
#else
#  define pic32mx_dmadump(handle,regs,msg)
#endif
#endif

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_DMA_H */
