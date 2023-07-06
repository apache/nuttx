/****************************************************************************
 * arch/arm64/include/setjmp.h
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

#ifndef __ARCH_ARM64_INCLUDE_SETJUMP_H
#define __ARCH_ARM64_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct setjmp_buf_s
{
  uint64_t x19;
  uint64_t x20;
  uint64_t x21;
  uint64_t x22;
  uint64_t x23;
  uint64_t x24;
  uint64_t x25;
  uint64_t x26;
  uint64_t x27;
  uint64_t x28;
  uint64_t x29;
  uint64_t x30;
  uint64_t x16;
  uint64_t gap;

#ifdef CONFIG_ARCH_FPU
  float    q8;
  float    q9;
  float    q10;
  float    q11;
  float    q12;
  float    q13;
  float    q14;
  float    q15;
#endif
};

/* Traditional typedef for setjmp_buf */

typedef struct setjmp_buf_s jmp_buf[1];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) noreturn_function;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM64_INCLUDE_SETJUMP_H */
