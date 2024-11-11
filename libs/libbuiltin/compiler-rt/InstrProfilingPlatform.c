/****************************************************************************
 * libs/libbuiltin/compiler-rt/InstrProfilingPlatform.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>

#include "InstrProfiling.h"
#include "InstrProfilingInternal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char __start__llvm_prf_names[];
extern char __end__llvm_prf_names[];
extern char __start__llvm_prf_data[];
extern char __end__llvm_prf_data[];
extern char __start__llvm_prf_cnts[];
extern char __end__llvm_prf_cnts[];

COMPILER_RT_VISIBILITY ValueProfNode *CurrentVNode = 0;
COMPILER_RT_VISIBILITY ValueProfNode *EndVNode = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

COMPILER_RT_VISIBILITY
void __llvm_profile_register_function(void *Data_)
{
}

COMPILER_RT_VISIBILITY
void __llvm_profile_register_names_function(void *NamesStart,
                                            uint64_t NamesSize)
{
}

COMPILER_RT_VISIBILITY
const __llvm_profile_data *__llvm_profile_begin_data(void)
{
  return (const __llvm_profile_data *)__start__llvm_prf_data;
}

COMPILER_RT_VISIBILITY
const __llvm_profile_data *__llvm_profile_end_data(void)
{
  return (const __llvm_profile_data *)__end__llvm_prf_data;
}

COMPILER_RT_VISIBILITY
const char *__llvm_profile_begin_names(void)
{
  return (const char *)__start__llvm_prf_names;
}

COMPILER_RT_VISIBILITY
const char *__llvm_profile_end_names(void)
{
  return (const char *)__end__llvm_prf_names;
}

COMPILER_RT_VISIBILITY
char *__llvm_profile_begin_counters(void)
{
  return (char *)__start__llvm_prf_cnts;
}

COMPILER_RT_VISIBILITY
char *__llvm_profile_end_counters(void)
{
  return (char *)__end__llvm_prf_cnts;
}

COMPILER_RT_VISIBILITY
char *__llvm_profile_begin_bitmap(void)
{
  return 0;
}

COMPILER_RT_VISIBILITY
char *__llvm_profile_end_bitmap(void)
{
  return 0;
}

COMPILER_RT_VISIBILITY
uint32_t *__llvm_profile_begin_orderfile(void)
{
  return 0;
}

COMPILER_RT_VISIBILITY
ValueProfNode *__llvm_profile_begin_vnodes(void)
{
  return 0;
}

COMPILER_RT_VISIBILITY
ValueProfNode *__llvm_profile_end_vnodes(void)
{
  return 0;
}

COMPILER_RT_VISIBILITY int __llvm_write_binary_ids(ProfDataWriter *Writer)
{
  return 0;
}
