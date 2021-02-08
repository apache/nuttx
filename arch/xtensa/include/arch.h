/****************************************************************************
 * arch/xtensa/include/arch.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/arch.h
 */

#ifndef __ARCH_XTENSA_INCLUDE_ARCH_H
#define __ARCH_XTENSA_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_getsp
 ****************************************************************************/

static inline uint32_t xtensa_getsp(void)
{
  register uint32_t sp;

  __asm__ __volatile__
  (
    "mov %0, sp\n"
    : "=r" (sp)
  );

  return sp;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_XTENSA_USE_SEPARATE_IMEM
struct mallinfo; /* Forward reference, see malloc.h */

/****************************************************************************
 * Name: xtensa_imm_initialize
 *
 * Description:
 *   Initialize the internal heap.
 *
 ****************************************************************************/

void xtensa_imm_initialize(void);

/****************************************************************************
 * Name: xtensa_imm_malloc
 *
 * Description:
 *   Allocate memory from the internal heap.
 *
 ****************************************************************************/

FAR void *xtensa_imm_malloc(size_t size);

/****************************************************************************
 * Name: xtensa_imm_calloc
 *
 * Description:
 *   Calculates the size of the allocation and
 *   allocate memory the internal heap.
 *
 ****************************************************************************/

FAR void *xtensa_imm_calloc(size_t n, size_t elem_size);

/****************************************************************************
 * Name: xtensa_imm_realloc
 *
 * Description:
 *   Reallocate memory from the internal heap.
 *
 ****************************************************************************/

FAR void *xtensa_imm_realloc(void *ptr, size_t size);

/****************************************************************************
 * Name: xtensa_imm_zalloc
 *
 * Description:
 *   Allocate and zero memory from the internal heap.
 *
 ****************************************************************************/

FAR void *xtensa_imm_zalloc(size_t size);

/****************************************************************************
 * Name: xtensa_imm_free
 *
 * Description:
 *   Free memory from the internal heap.
 *
 ****************************************************************************/

void      xtensa_imm_free(FAR void *mem);

/****************************************************************************
 * Name: xtensa_imm_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

FAR void *xtensa_imm_memalign(size_t alignment, size_t size);

/****************************************************************************
 * Name: xtensa_imm_heapmember
 *
 * Description:
 *   Check if an address lies in the internal heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the internal heap.  false if not
 *
 ****************************************************************************/

bool      xtensa_imm_heapmember(FAR void *mem);

/****************************************************************************
 * Name: xtensa_imm_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

int       xtensa_imm_mallinfo(FAR struct mallinfo *info);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_INCLUDE_ARCH_H */
