/****************************************************************************
 *  arch/arm/src/kinetis/kinetis_dma.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_config.h"
#include "chip.h"
#include "kinetis_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmainitilaize(void)
{

}

/****************************************************************************
 * Name: kinetis_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE kinetis_dmachannel(void)
{
	return NULL;
}

/****************************************************************************
 * Name: kinetis_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kinetis_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmafree(DMA_HANDLE handle)
{

}

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int kinetis_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                  uint32_t srcaddr, uint32_t destaddr, size_t nbytes)
{
  return -1;
}

/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  return -1;
}


/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void kinetis_dmastop(DMA_HANDLE handle)
{
}

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs)
{

}
#endif

/****************************************************************************
 * Name: kinetis_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmadump(DMA_HANDLE handle, const struct kinetis_dmaregs_s *regs,
                const char *msg)
{

}
#endif

