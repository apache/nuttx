/****************************************************************************
 * mm/kasan/kasan.h
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

#ifndef __MM_KASAN_KASAN_H
#define __MM_KASAN_KASAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MM_KASAN
#  define kasan_poison(addr, size)
#  define kasan_unpoison(addr, size)
#  define kasan_register(addr, size)
#endif

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

#ifdef CONFIG_MM_KASAN

/****************************************************************************
 * Name: kasan_poison
 *
 * Description:
 *   Mark the memory range as inaccessible
 *
 * Input Parameters:
 *   addr - range start address
 *   size - range size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_poison(FAR const void *addr, size_t size);

/****************************************************************************
 * Name: kasan_unpoison
 *
 * Description:
 *   Mark the memory range as accessible
 *
 * Input Parameters:
 *   addr - range start address
 *   size - range size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_unpoison(FAR const void *addr, size_t size);

/****************************************************************************
 * Name: kasan_register
 *
 * Description:
 *   Monitor the memory range for invalid access check
 *
 * Input Parameters:
 *   addr - range start address
 *   size - range size
 *
 * Returned Value:
 *   None.
 *
 * Note:
 *   The size is shrinked for the shadow region
 *
 ****************************************************************************/

void kasan_register(FAR void *addr, FAR size_t *size);

#endif /* CONFIG_MM_KASAN */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MM_KASAN_KASAN_H */
