/****************************************************************************
 * arch/arm/src/stm32f7/stm32_dtcm.h
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
 *           Bob Feretich <bob.feretich@rafresearch.com>
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_DTCM_H
#define __ARCH_ARM_SRC_STM32F7_STM32_DTCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mm/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Assume that we can support the DTCM heap */

#define HAVE_DTCM_HEAP 1

/* The STM32 F7 have DTCM memory */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX) \
   || defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX)
#  define DTCM_START 0x20000000
#  define DTCM_END   0x20010000
#elif defined(CONFIG_STM32F7_STM32F76XX) || defined(CONFIG_STM32F7_STM32F77XX)
#  define DTCM_START 0x20000000
#  define DTCM_END   0x20020000
#else
#  undef HAVE_DTCM_HEAP
#endif

/* In order to use the DTCM heap, it had to have been excluded from the main
 * heap.
 */

#ifndef CONFIG_STM32F7_DTCMEXCLUDE
#  undef HAVE_DTCM_HEAP
#endif

/* Can we support the DTCM heap? */

#ifdef HAVE_DTCM_HEAP

/* dtcm_initialize must be called early in initialization in order to
 * initialize the DTCM heap.
 */

#define dtcm_initialize() \
  mm_initialize(&g_dtcm_heap, (FAR void *)DTCM_START, DTCM_END-DTCM_START)

/* The dtcm_addregion interface could be used if, for example, you want to
 * add some other memory region to the DTCM heap.  I don't really know why
 * you might want to do that, but the functionality is essentially free.
 */

#define dtcm_addregion(b,s) mm_addregion(&g_dtcm_heap, b, s);

/* Then, once g_dtcm_heap has been setup by dtcm_initialize(), these memory
 * allocators can be used just like the standard memory allocators.
 */

#define dtcm_malloc(s)      mm_malloc(&g_dtcm_heap, s)
#define dtcm_zalloc(s)      mm_zalloc(&g_dtcm_heap, s)
#define dtcm_calloc(n,s)    mm_calloc(&g_dtcm_heap, n,s)
#define dtcm_free(p)        mm_free(&g_dtcm_heap, p)
#define dtcm_realloc(p,s)   mm_realloc(&g_dtcm_heap, p, s)
#define dtcm_memalign(a,s)  mm_memalign(&g_dtcm_heap, a, s)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN struct mm_heap_s g_dtcm_heap;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* HAVE_DTCM_HEAP */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_DTCM_H */
