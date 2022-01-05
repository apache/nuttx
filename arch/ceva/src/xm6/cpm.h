/****************************************************************************
 * arch/ceva/src/xm6/cpm.h
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

#ifndef __ARCH_CEVA_SRC_XM6_CPM_H
#define __ARCH_CEVA_SRC_XM6_CPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <vec-c.h>

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

static inline uint32_t getcpm(uintptr_t addr)
{
  return in(cpm, (const volatile uint32_t *)addr);
}

static inline void putcpm(uintptr_t addr, uint32_t value)
{
  out(cpm, value, (volatile uint32_t *)addr);
}

static inline void modifycpm(uintptr_t addr, uint32_t clearbits,
                             uint32_t setbits)
{
  putcpm(addr, (getcpm(addr) & ~clearbits) | setbits);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_CEVA_SRC_XM6_CPM_H */
