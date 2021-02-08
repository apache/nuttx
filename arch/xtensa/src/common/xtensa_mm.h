/****************************************************************************
 * arch/xtensa/src/common/xtensa_mm.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_MM_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_MM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
#  define UMM_MALLOC(s)      xtensa_imm_malloc(s)
#  define UMM_MEMALIGN(a,s)  xtensa_imm_memalign(a,s)
#  define UMM_FREE(p)        xtensa_imm_free(p)
#  define UMM_HEAPMEMEBER(p) xtensa_imm_heapmember(p)
#else
#  define UMM_MALLOC(s)      kumm_malloc(s)
#  define UMM_MEMALIGN(a,s)  kumm_memalign(a,s)
#  define UMM_FREE(p)        kumm_free(p)
#  define UMM_HEAPMEMEBER(p) umm_heapmember(p)
#endif

#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_MM_H */
