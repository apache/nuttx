/****************************************************************************
 * include/nuttx/mm/kasan.h
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

#ifndef __INCLUDE_NUTTX_MM_KASAN_H
#define __INCLUDE_NUTTX_MM_KASAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MM_KASAN
#  define kasan_poison(addr, size)
#  define kasan_unpoison(addr, size) addr
#  define kasan_register(addr, size)
#  define kasan_unregister(addr)
#  define kasan_reset_tag(addr) addr
#  define kasan_start()
#  define kasan_stop()
#  define kasan_debugpoint(t,a,s) 0
#  define kasan_init_early()
#else

#  define kasan_init_early() kasan_stop()

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
 *   Return tagged address
 *
 ****************************************************************************/

FAR void *kasan_unpoison(FAR const void *addr, size_t size);

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

/****************************************************************************
 * Name: kasan_unregister
 *
 * Description:
 *   Stop monitoring the memory range
 *
 * Input Parameters:
 *   addr - range start address
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_unregister(FAR void *addr);

/****************************************************************************
 * Name: kasan_reset_tag
 *
 * Input Parameters:
 *   addr - The address of the memory to reset the tag.
 *
 * Returned Value:
 *   Unlabeled address
 *
 ****************************************************************************/

FAR void *kasan_reset_tag(FAR const void *addr);

/****************************************************************************
 * Name: kasan_start
 *
 * Description:
 *   Let kasan start check.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_start(void);

/****************************************************************************
 * Name: kasan_stop
 *
 * Description:
 *   Stop kasan check, setup g_region_init variable.
 *   This used for some platfroms clear bss late, and error use kasan before
 *   called kasan_register().
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_stop(void);

/****************************************************************************
 * Name: kasan_debugpoint
 *
 * Description:
 *   Monitor the memory range for invalid access check
 *
 * Input Parameters:
 *   type - DEBUGPOINT_NONE         : remove
 *          DEBUGPOINT_WATCHPOINT_RO: read
 *          DEBUGPOINT_WATCHPOINT_WO: write
 *          DEBUGPOINT_WATCHPOINT_RW: read/write
 *   addr - range start address
 *   size - range size
 *
 * Returned Value:
 *   If the setting is successful, it returns 0, otherwise it
 *   returns an error code.
 *
 ****************************************************************************/

int kasan_debugpoint(int type, FAR void *addr, size_t size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MM_KASAN */

#endif /* __INCLUDE_NUTTX_MM_KASAN_H */
