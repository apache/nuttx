/****************************************************************************
 * arch/arm/src/armv7-a/l2cc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_L2CC_H
#define __ARCH_ARM_SRC_ARMV7_A_L2CC_H

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
 * Name: up_l2ccinitialize
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

#if 0 /* Prototyped in up_internal.h */
void up_l2ccinitialize(void);
#endif

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
#endif  /* __ARCH_ARM_SRC_ARMV7_A_L2CC_H */
