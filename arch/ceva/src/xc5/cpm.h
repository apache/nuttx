/****************************************************************************
 * arch/ceva/src/xc5/cpm.h
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

#ifndef __ARCH_CEVA_SRC_XC5_CPM_H
#define __ARCH_CEVA_SRC_XC5_CPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Inline functions
 ****************************************************************************/
#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

uint32_t getcpm(uintptr_t addr);
void putcpm(uintptr_t addr, uint32_t value);
void modifycpm(uintptr_t addr, uint32_t clearbits, uint32_t setbits);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_CEVA_SRC_XC5_CPM_H */
