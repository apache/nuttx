/****************************************************************************
 * arch/arm/src/armv7-r/l2cc.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_R_L2CC_H
#define __ARCH_ARM_SRC_ARMV7_R_L2CC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_L2CACHE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_l2ccinitialize
 *
 * Description:
 *   One time configuration of the L2 cache.  The L2 cache will be enabled
 *   upon return.
 *
 * Input Parameters:
 *   None.  The L2 cache configuration is controlled by configuration
 *   settings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Prototyped in arm_internal.h */
void arm_l2ccinitialize(void);
#endif

/****************************************************************************
 * Name: l2cc_get_linesize
 *
 * Description:
 *    Get L2CC-P310 L2 cache linesize
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    L2 cache linesize
 *
 ****************************************************************************/

uint32_t l2cc_get_linesize(void);

/****************************************************************************
 * Name: l2cc_enable
 *
 * Description:
 *    Re-enable the L2CC-P310 L2 cache by setting the enable bit in the
 *    Control Register (CR)
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

void l2cc_enable(void);

/****************************************************************************
 * Name: l2cc_disable
 *
 * Description:
 *    Disable the L2 cache
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

void l2cc_disable(void);

/****************************************************************************
 * Name: l2cc_sync
 *
 * Description:
 *   Drain the L2 cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_sync(void);

/****************************************************************************
 * Name: l2cc_invalidate_all
 *
 * Description:
 *   Invalidate the entire L2 cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_invalidate_all(void);

/****************************************************************************
 * Name: l2cc_invalidate
 *
 * Description:
 *   Invalidate a range of addresses in the L2 cache
 *
 * Input Parameters:
 *   startaddr - The first address to be invalidated
 *   endaddr   - The last address to be invalidated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_invalidate(uintptr_t startaddr, uintptr_t endaddr);

/****************************************************************************
 * Name: l2cc_clean_all
 *
 * Description:
 *   Clean the entire L2 cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_clean_all(void);

/****************************************************************************
 * Name: l2cc_clean
 *
 * Description:
 *   Clean a range of address within the L2 cache.
 *
 * Input Parameters:
 *   startaddr - The first address to be cleaned
 *   endaddr   - The last address to be cleaned
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_clean(uintptr_t startaddr, uintptr_t endaddr);

/****************************************************************************
 * Name: l2cc_flush_all
 *
 * Description:
 *   Flush the entire L2 cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_flush_all(void);

/****************************************************************************
 * Name: l2cc_flush
 *
 * Description:
 *   Flush a range of address within the L2 cache.
 *
 * Input Parameters:
 *   startaddr - The first address to be flushed
 *   endaddr   - The last address to be flushed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void l2cc_flush(uint32_t startaddr, uint32_t endaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#else /* CONFIG_ARCH_L2CACHE */
  /* Provide simple definitions to concentrate the inline conditional
   * compilation in one place.
   */

#  define l2cc_get_linesize() 0
#  define l2cc_enable()
#  define l2cc_disable()
#  define l2cc_sync()
#  define l2cc_invalidate_all()
#  define l2cc_invalidate(s,e)
#  define l2cc_clean_all()
#  define l2cc_clean(s,e)
#  define l2cc_flush_all()
#  define l2cc_flush(s,e)

#endif /* CONFIG_ARCH_L2CACHE */
#endif /* __ARCH_ARM_SRC_ARMV7_R_L2CC_H */
