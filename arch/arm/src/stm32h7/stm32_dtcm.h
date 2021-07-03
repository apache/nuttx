/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dtcm.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_DTCM_H
#define __ARCH_ARM_SRC_STM32H7_STM32_DTCM_H

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

/* The STM32 H7 have DTCM memory */

#define DTCM_START 0x20000000
#define DTCM_END   0x20020000

/* In order to use the DTCM heap, it had to have been excluded from the main
 * heap.
 */

#ifndef CONFIG_STM32H7_DTCMEXCLUDE
#  undef HAVE_DTCM_HEAP
#endif

/* Can we support the DTCM heap? */

#ifdef HAVE_DTCM_HEAP

/* dtcm_initialize must be called early in initialization in order to
 * initialize the DTCM heap.
 */

#define dtcm_initialize() \
  g_dtcm_heap = mm_initialize("dtcm", (FAR void *)DTCM_START, DTCM_END-DTCM_START)

/* The dtcm_addregion interface could be used if, for example, you want to
 * add some other memory region to the DTCM heap.  I don't really know why
 * you might want to do that, but the functionality is essentially free.
 */

#define dtcm_addregion(b,s) mm_addregion(g_dtcm_heap, b, s);

/* Then, once g_dtcm_heap has been setup by dtcm_initialize(), these memory
 * allocators can be used just like the standard memory allocators.
 */

#define dtcm_malloc(s)      mm_malloc(g_dtcm_heap, s)
#define dtcm_zalloc(s)      mm_zalloc(g_dtcm_heap, s)
#define dtcm_calloc(n,s)    mm_calloc(g_dtcm_heap, n,s)
#define dtcm_free(p)        mm_free(g_dtcm_heap, p)
#define dtcm_realloc(p,s)   mm_realloc(g_dtcm_heap, p, s)
#define dtcm_memalign(a,s)  mm_memalign(g_dtcm_heap, a, s)

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

EXTERN FAR struct mm_heap_s *g_dtcm_heap;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* HAVE_DTCM_HEAP */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_DTCM_H */
