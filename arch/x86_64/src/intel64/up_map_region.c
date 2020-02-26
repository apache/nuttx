/****************************************************************************
 * arch/x86/src/intel64/up_map_region.c
 *
 *   Copyright (C) 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Chung-Fan Yang <sonic.tw.tp@gmail.com>
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

#include <debug.h>
#include <nuttx/irq.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_map_region
 *
 * Description:
 *   Map a memory region as 1:1 by MMU
 *
 ****************************************************************************/

int up_map_region(void* base, int size, int flags)
{

  /* Round to page boundary */
  uint64_t bb = (uint64_t)base & ~(PAGE_SIZE - 1);

  /* Increase size if the base address is rounded off */
  size += (uint64_t)base - bb;
  uint64_t num_of_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

  uint64_t entry;
  uint64_t curr;

  if(bb > 0xFFFFFFFF) return -1; //Only < 4GB can be mapped

  curr = bb;
  for(int i = 0; i < num_of_pages; i++)
    {
      entry = (curr >> 12) & 0x7ffffff;

      pt[entry] = curr | flags;
      curr += PAGE_SIZE;
    }

  return 0;
}
