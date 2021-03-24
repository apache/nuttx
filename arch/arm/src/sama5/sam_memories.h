/****************************************************************************
 * arch/arm/src/sama5/sam_memories.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_MEMORIES_H
#define __ARCH_ARM_SRC_SAMA5_SAM_MEMORIES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: sam_physregaddr
 *
 * Description:
 *   Give the virtual address of a register, return the physical address of
 *   the register
 *
 ****************************************************************************/

uintptr_t sam_physregaddr(uintptr_t virtregaddr);

/****************************************************************************
 * Name: sam_physramaddr
 *
 * Description:
 *   Give the virtual address of a RAM memory location, return the physical
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t sam_physramaddr(uintptr_t vramaddr);

/****************************************************************************
 * Name: sam_virtramaddr
 *
 * Description:
 *   Give the physical address of a RAM memory location, return the virtual
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t sam_virtramaddr(uintptr_t physramaddr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_MEMORIES_H */
