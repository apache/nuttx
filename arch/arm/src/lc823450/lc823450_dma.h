/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dma.h
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_DMA_H
#define __ARCH_ARM_SRC_LC823450_LC823450_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <arch/irq.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_DMA_ITC                (1 << 31)
#define LC823450_DMA_SRCWIDTH_BYTE      (0 << 18)
#define LC823450_DMA_SRCWIDTH_HWORD     (1 << 18)
#define LC823450_DMA_SRCWIDTH_WORD      (2 << 18)
#define LC823450_DMA_DSTWIDTH_BYTE      (0 << 21)
#define LC823450_DMA_DSTWIDTH_HWORD     (1 << 21)
#define LC823450_DMA_DSTWIDTH_WORD      (2 << 21)
#define LC823450_DMA_SRCINC             (1 << 26)
#define LC823450_DMA_DSTINC             (1 << 27)
#define LC823450_DMA_SBS_SHIFT          12
#define LC823450_DMA_DBS_SHIFT          15
#define LC823450_DMA_BS_1               0
#define LC823450_DMA_BS_4               1
#define LC823450_DMA_BS_8               2
#define LC823450_DMA_BS_16              3
#define LC823450_DMA_BS_32              4
#define LC823450_DMA_BS_64              5
#define LC823450_DMA_BS_128             6
#define LC823450_DMA_BS_256             7

#define LC823450_DMA_TRANSSIZE_MASK     (0xfff << 0)
#define LC823450_DMA_MAX_TRANSSIZE      0xff0

/* HighPriority */

#define DMA_CHANNEL_SIOTX               0
#define DMA_CHANNEL_UART1RX             1
#define DMA_CHANNEL_UART1TX             2
#define DMA_CHANNEL_USBDEV              3
#define DMA_CHANNEL_AUDIOWR             4
#define DMA_CHANNEL_AUDIORD             5
#if 0
#define DMA_CHANNEL_???                 6
#endif
#define DMA_CHANNEL_VIRTUAL             7
#define DMA_CHANNEL_NUM                 8

#define DMA_REQUEST_UART0RX             0
#define DMA_REQUEST_UART0TX             1
#define DMA_REQUEST_UART1RX             2
#define DMA_REQUEST_UART1TX             3
#define DMA_REQUEST_UART2RX             4
#define DMA_REQUEST_UART2TX             5
#define DMA_REQUEST_SIORX               6
#define DMA_REQUEST_SIOTX               7
#define DMA_REQUEST_AUDIOBUF0           8
#define DMA_REQUEST_AUDIOBUF1           9
#define DMA_REQUEST_AUDIOBUF2           10
#define DMA_REQUEST_AUDIOBUF3           11
#define DMA_REQUEST_AUDIOBUF4           12
#define DMA_REQUEST_AUDIOBUF5           13
#define DMA_REQUEST_AUDIOBUF6           14
#define DMA_REQUEST_AUDIOBUF7           15
#define DMA_REQUEST_USBDEV              22

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lc823450_dma_llist
{
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t nextlli;
  uint32_t ctrl;
};

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

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
 * Public Functions
 ****************************************************************************/

void lc823450_dmaconfigure(uint8_t dmarequest, bool alternate);
DMA_HANDLE lc823450_dmachannel(int ch);
void lc823450_dmafree(DMA_HANDLE handle);
void lc823450_dmarequest(DMA_HANDLE handle, uint8_t dmarequest);
int lc823450_dmasetup(DMA_HANDLE handle, uint32_t control,
                      uint32_t srcaddr, uint32_t destaddr, size_t nxfrs);
int lc823450_dmallsetup(DMA_HANDLE handle, uint32_t control,
                        uint32_t srcaddr, uint32_t destaddr,
                        size_t nxfrs, uint32_t llist);
void lc823450_dmareauest_dir(DMA_HANDLE handle, uint8_t dmarequest,
                             int m2p);

int lc823450_dmastart(DMA_HANDLE handle, dma_callback_t callback,
                      void *arg);
void lc823450_dmastop(DMA_HANDLE handle);
int lc823450_dmaremain(DMA_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /*  __ARCH_ARM_SRC_LC823450_LC823450_DMA_H */
