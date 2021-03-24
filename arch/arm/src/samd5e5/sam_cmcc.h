/****************************************************************************
 * arch/arm/src/samd5e5/sam_cmcc.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_SAMD5E5_CMCC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
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

/* Stubs so that we don't have to put condition compilation in driver
 * source
 */

#  define sam_cmcc_invalidate(start, end)
#  define sam_cmcc_invalidateall()

#endif /* CONFIG_SAMD5E5_CMCC */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_CMCC_H */
