/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_dma_alloc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/mm/gran.h>
#include <nuttx/compiler.h>

#include "imxrt1180-evk.h"

#if defined(CONFIG_GRAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* On the imxrt1180-evk M33 image, .data/.bss/heap live in the CM33's
 * private System TCM, which the USB controller's DMA engine cannot reach
 * (it isn't part of the TRDC-controlled shared memory map).  The USB
 * class driver's request buffers (imxrt_epallocbuffer()/usbdev_dma_alloc())
 * must therefore come from a separate, DMA-reachable pool.  We put that
 * pool in the same OCRAM2 `.dmamemory` linker section already used for the
 * USB controller's endpoint queue heads/transfer descriptors and EP0
 * buffer (see arch/arm/src/imxrt/imxrt_usbdev.c and
 * boards/arm/imxrt/imxrt1180-evk/scripts/flash-m33.ld).
 *
 * The pool must be big enough for all buffers that can be concurrently
 * allocated by the bound class driver.  For CDC/ACM with the default
 * config (4 read + 4 write requests) this is roughly:
 *   1 x CDCACM_MXDESCLEN         (ctrlreq,   64B)
 *   CONFIG_CDCACM_NRDREQS x CONFIG_CDCACM_BULKOUT_REQLEN (4 x 512 = 2048B)
 *   CONFIG_CDCACM_NWRREQS x CONFIG_CDCACM_BULKIN_REQLEN  (4 x  96 =  384B)
 * i.e. ~2.5KB; round up generously to leave headroom for granule/alignment
 * overhead and other endpoints.
 */

#define IMXRT_DMA_POOL_SIZE   (6 * 1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE g_dma_allocator;

static uint8_t g_dma_heap[IMXRT_DMA_POOL_SIZE]
                          locate_data(".dmamemory")
                          aligned_data(32);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_dma_alloc_init
 *
 * Description:
 *   Initialize the DMA-reachable memory pool used to back board USB
 *   device request buffers.  Called once from board bringup, before any
 *   USB device activity.
 *
 ****************************************************************************/

int imxrt_dma_alloc_init(void)
{
  g_dma_allocator = gran_initialize(g_dma_heap,
                                     sizeof(g_dma_heap),
                                     5,  /* 32-byte granule */
                                     5); /* 32-byte alignment */

  return g_dma_allocator == NULL ? -ENOMEM : OK;
}

/****************************************************************************
 * Name: usbdev_dma_alloc
 *
 * Description:
 *   Allocate a DMA-reachable buffer for use by a USB device driver.
 *
 *   gran_free() needs to know the size of the allocation being freed, but
 *   usbdev_dma_free() is not given a size (see include/nuttx/usb/usbdev.h).
 *   We work around this by prepending a small header that records the
 *   total granule allocation size.
 *
 ****************************************************************************/

struct usbdma_header_s
{
  size_t allocsize;
};

void *usbdev_dma_alloc(size_t size)
{
  struct usbdma_header_s *hdr;
  size_t allocsize;

  DEBUGASSERT(g_dma_allocator != NULL);

  allocsize = size + sizeof(struct usbdma_header_s);
  hdr = gran_alloc(g_dma_allocator, allocsize);
  if (hdr == NULL)
    {
      return NULL;
    }

  hdr->allocsize = allocsize;
  return (void *)(hdr + 1);
}

/****************************************************************************
 * Name: usbdev_dma_free
 *
 * Description:
 *   Free a DMA-reachable buffer previously allocated with
 *   usbdev_dma_alloc().
 *
 ****************************************************************************/

void usbdev_dma_free(void *mem)
{
  struct usbdma_header_s *hdr;

  DEBUGASSERT(g_dma_allocator != NULL);

  if (mem == NULL)
    {
      return;
    }

  hdr = ((struct usbdma_header_s *)mem) - 1;
  gran_free(g_dma_allocator, hdr, hdr->allocsize);
}

#endif /* CONFIG_GRAN */
