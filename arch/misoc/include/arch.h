/****************************************************************************
 * arch/misoc/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_MISOC_INCLUDE_ARCH_H
#define __ARCH_MISOC_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: misoc_getsp
 ****************************************************************************/

static inline uint32_t misoc_getsp(void)
{
  register uint32_t sp;

  __asm__ __volatile__("addi %0, sp, 0" : "=r" (sp));

  return sp;
}

#endif /* __ARCH_MISOC_INCLUDE_ARCH_H */
