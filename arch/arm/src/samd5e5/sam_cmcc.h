/************************************************************************************
 * arch/arm/src/samd5e5/sam_cmcc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_SAMD5E5_CMCC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: sam_cmcc_enable
 *
 * Description:
 *   Enable the Cortex-M Cache Controller
 *
 ****************************************************************************/

void sam_cmcc_enable(void);

/****************************************************************************
 * Name: sam_cmcc_disable
 *
 * Description:
 *   Disable the Cortex-M Cache Controller
 *
 ****************************************************************************/

void sam_cmcc_disable(void);

/****************************************************************************
 * Name: sam_cmcc_invalidate
 *
 * Description:
 *   Invalidate a range of addresses.  Note:  These addresses should be
 *   aligned with the beginning and end of cache lines.  Otherwise, values
 *   at the edges of the region will also be invalidated!
 *
 ****************************************************************************/

void sam_cmcc_invalidate(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: sam_cmcc_invalidateall
 *
 * Description:
 *   Invalidate the entire cache
 *
 ****************************************************************************/

void sam_cmcc_invalidateall(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#else /* CONFIG_SAMD5E5_CMCC */

/* Stubs so that we don't have to put condition compilation in driver source */

#  define sam_cmcc_invalidate(start, end)
#  define sam_cmcc_invalidateall()

#endif /* CONFIG_SAMD5E5_CMCC */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H */
