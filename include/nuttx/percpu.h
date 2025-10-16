/****************************************************************************
 * include/nuttx/percpu.h
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

#ifndef __INCLUDE_NUTTX_PERCPU_H
#define __INCLUDE_NUTTX_PERCPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_HAVE_MULTICPU
#  define up_cpu_index() 0
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

/****************************************************************************
 * Name: up_this_cpu
 *
 * Description:
 *   Return the logical core number. Default implementation is 1:1 mapping,
 *   i.e. physical=logical.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_HAVE_CPUID_MAPPING
#  define up_this_cpu() up_cpu_index()
#endif

/* These are macros to access the current CPU and the current task on a CPU.
 * These macros are intended to support a future SMP implementation.
 */

#ifdef CONFIG_SMP
#  define this_cpu()                   up_this_cpu()
#else
#  define this_cpu()                   (0)
#endif

#ifdef CONFIG_PERCPU_ARRAY
#  define DEFINE_PER_CPU(t, v)         t v[CONFIG_NCPUS]
#  define DEFINE_PER_CPU_BSS(t, v)     t v[CONFIG_NCPUS]
#  define DECLARE_PER_CPU(t, v)        extern t v[CONFIG_NCPUS]
#  define per_cpu_var_smp(v, c)        v[c]
#  define this_cpu_var(v)              v[this_cpu()]
#elif defined(CONFIG_PERCPU_SECTION)
#  define DEFINE_PER_CPU(t, v)         locate_data(".data..percpu") t v
#  define DEFINE_PER_CPU_BSS(t, v)     locate_data(".bss..percpu") t v
#  define DECLARE_PER_CPU(t, v)        extern t v;

/* Compile time percpu .data & .bss size */

extern char _percpu_size[];
#  define PERCPU_SIZE                  (size_t)_percpu_size

/* For BMP case per_cpu_var is not supported */

#  define per_cpu_var_smp(v, c)        (*(FAR typeof(v) *)((unsigned long)&(v) + PERCPU_SIZE * (c)))
#  define this_cpu_var(v)              (*(FAR typeof(v) *)((unsigned long)&(v) + PERCPU_SIZE * up_cpu_index()))
#endif

#ifdef CONFIG_SMP
#  define DEFINE_PER_CPU_SMP(t, v)     DEFINE_PER_CPU(t, v)
#  define DEFINE_PER_CPU_BSS_SMP(t, v) DEFINE_PER_CPU_BSS(t, v)
#  define DECLARE_PER_CPU_SMP(t, v)    DECLARE_PER_CPU(t, v)
#  define this_cpu_var_smp(v)          this_cpu_var(v)
#else
#  define DEFINE_PER_CPU_SMP(t, v)     t v[1]
#  define DEFINE_PER_CPU_BSS_SMP(t, v) t v[1]
#  define DECLARE_PER_CPU_SMP(t, v)    extern t v[1]
#  define this_cpu_var_smp(v)          v[0]
#endif

#ifdef CONFIG_BMP
#  define DEFINE_PER_CPU_BMP(t, v)     DEFINE_PER_CPU(t, v)
#  define DEFINE_PER_CPU_BSS_BMP(t, v) DEFINE_PER_CPU_BSS(t, v)
#  define DECLARE_PER_CPU_BMP(t, v)    DECLARE_PER_CPU(t, v)
#  define this_cpu_var_bmp(v)          this_cpu_var(v)
#else
#  define DEFINE_PER_CPU_BMP(t, v)     t v
#  define DEFINE_PER_CPU_BSS_BMP(t, v) t v
#  define DECLARE_PER_CPU_BMP(t, v)    extern t v
#  define this_cpu_var_bmp(v)          v
#endif

#endif
