/****************************************************************************
 * libs/libc/machine/arm64/gnu/mcount.c
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

#include <sys/gmon.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline noinstrument_function void *strip_pac(void *p)
{
  register void *ra asm ("x30") = (p);
  asm ("hint 7 // xpaclri" : "+r"(ra));
  return ra;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _mcount
 *
 * Description:
 *   This is the ARM64 mcount function.  It is called by the profiling
 *   logic to record the call.
 *
 * Input Parameters:
 *   frompc - The address of the calling instruction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

noinstrument_function
void _mcount(void *frompc)
{
  mcount_internal((uintptr_t)strip_pac(frompc),
                  (uintptr_t)return_address(0));
}

