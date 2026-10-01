/****************************************************************************
 * arch/arm/src/imxrt/imxrt_dtcm.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_DTCM_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_DTCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The M7's DTCM and symmetrically the M33's System TCM are reachable by
 * other bus-master initiators via a system-bus alias window.
 */

#if defined(CONFIG_ARCH_CHIP_MIMXRT1189CVM8C) || \
    defined(CONFIG_ARCH_CHIP_MIMXRT1189CVM8C_CM33)
#  define USE_DTCM_SHADOW_ADDRESSING 1
#  ifdef CONFIG_ARCH_CORTEXM33
#    define DTCM_SIZE                (128 * 1024)
#    define DTCM_SHADOW_ADDRESS      0x20200000ul
#  elif defined(CONFIG_ARCH_CORTEXM7)
#    define DTCM_SIZE                (256 * 1024)
#    define DTCM_SHADOW_ADDRESS      0x20400000ul
#  endif
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_dma_address
 *
 * Description:
 *   Translate a CPU buffer address into the address a DMA-capable bus
 *   master must be given to reach the same physical memory.  A buffer in
 *   DTCM is translated to its system-bus alias (see the
 *   DTCM_SHADOW_ADDRESS comment above); any other address (OCRAM, flash,
 *   etc.) is already DMA-reachable and is returned unchanged.
 *
 * Input Parameters:
 *   buffer       - The CPU-view buffer address
 *   nbytes       - The size of the transfer, in bytes
 *   dma_address  - Set, on success, to the address to program into the
 *                  DMA engine
 *
 * Returned Value:
 *   true on success; false if buffer+nbytes would run past the end of
 *   DTCM (and therefore past the end of its alias window too).
 *
 ****************************************************************************/

static inline bool imxrt_dma_address(const void *buffer, size_t nbytes,
                                     uint32_t *dma_address)
{
  uintptr_t address = (uintptr_t)buffer;

#ifdef USE_DTCM_SHADOW_ADDRESSING
  if (address >= IMXRT_DTCM_BASE &&
      address - IMXRT_DTCM_BASE < DTCM_SIZE)
    {
      uintptr_t offset = address - IMXRT_DTCM_BASE;

      if (nbytes > DTCM_SIZE - offset)
        {
          return false;
        }

      address = DTCM_SHADOW_ADDRESS + offset;
    }
#endif

  *dma_address = (uint32_t)address;
  return true;
}

/****************************************************************************
 * Name: imxrt_dma_cpu_address
 *
 * Description:
 *   Translate a DMA-visible address (as produced by imxrt_dma_address())
 *   back into the CPU-view address of the same physical memory.  An
 *   address within the DTCM shadow alias window is translated back to its
 *   DTCM address; any other address is already CPU-reachable and is
 *   returned unchanged.
 *
 * Input Parameters:
 *   dma_address  - The DMA-visible address
 *
 * Returned Value:
 *   The corresponding CPU-view address.
 *
 ****************************************************************************/

static inline uintptr_t imxrt_dma_cpu_address(uint32_t dma_address)
{
  uintptr_t address = dma_address;

#ifdef USE_DTCM_SHADOW_ADDRESSING
  if (address >= DTCM_SHADOW_ADDRESS &&
      address - DTCM_SHADOW_ADDRESS < DTCM_SIZE)
    {
      address = IMXRT_DTCM_BASE + address - DTCM_SHADOW_ADDRESS;
    }
#endif

  return address;
}

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_DTCM_H */
