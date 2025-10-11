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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#endif
