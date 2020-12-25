/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dma.h
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
 * Public Function Prototypes
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

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_DMA_H */
