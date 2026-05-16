/****************************************************************************
 * arch/sim/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_SIM_INCLUDE_ARCH_H
#define __ARCH_SIM_INCLUDE_ARCH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* _sinit and _einit mark the beginning and end of the C++ constructor
 * array.  They are mapped to platform-specific linker symbols:
 *   macOS:  section$start$__DATA_CONST$__mod_init_func /
 *           section$end$__DATA_CONST$__mod_init_func (Mach-O auto)
 * On macOS, the section type flags are patched post-link to prevent
 * dyld from auto-running constructors before NuttX is initialized.
 */

#ifdef CONFIG_HAVE_CXXINITIALIZE
#  if defined(CONFIG_HOST_MACOS)
extern void (*_sinit[])(void)
  __asm("section$start$__DATA_CONST$__mod_init_func");
extern void (*_einit[])(void)
  __asm("section$end$__DATA_CONST$__mod_init_func");
#  endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_SIM_INCLUDE_ARCH_H */
