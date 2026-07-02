/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_textheap.c
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
 * Text-heap allocator for the RP2350.
 *
 * Dynamically loaded code (e.g. a "lifted arm" copied in from NFS) must be
 * placed in memory that the CPU can both write and then execute.  On the
 * RP2350 the on-chip SRAM (0x20000000) is normal, executable memory, and
 * this port runs without the MPU enabled (CONFIG_ARM_MPU=n), so RAM is not
 * marked no-execute.  The general-purpose kernel heap therefore already
 * satisfies the requirements of a text heap, and these wrappers simply
 * forward to it (the same approach used by arch/arm/src/cxd56xx).
 *
 * Because the instruction and data views of this memory are the same
 * address, ARCH_HAVE_TEXT_HEAP_SEPARATE_DATA_ADDRESS is not selected and
 * up_textheap_data_address() is not required.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_ARCH_USE_TEXT_HEAP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   Allocate aligned, executable memory for dynamically loaded code.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  return kmm_memalign(align, size);
}

/****************************************************************************
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory previously returned by up_textheap_memalign().
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  if (p != NULL)
    {
      kmm_free(p);
    }
}

/****************************************************************************
 * Name: up_textheap_heapmember
 *
 * Description:
 *   Test if the given address was allocated from the text heap.
 *
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  return kmm_heapmember(p);
}

#endif /* CONFIG_ARCH_USE_TEXT_HEAP */
