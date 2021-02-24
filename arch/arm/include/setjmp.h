/****************************************************************************
 * arch/arm/include/setjmp.h
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

#ifndef __ARCH_ARM_INCLUDE_SETJUMP_H
#define __ARCH_ARM_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if defined(CONFIG_ARCH_ARMV7M) || defined(CONFIG_ARCH_ARMV8M)
struct setjmp_buf_s
{
  /* Note: core registers r0-r3 are caller-saved */

  unsigned r4;
  unsigned r5;
  unsigned r6;
  unsigned r7;
  unsigned r8;
  unsigned r9;
  unsigned r10;
  unsigned r11;
  unsigned ip; /* this is really sp */
  unsigned lr;

#ifdef CONFIG_ARCH_FPU
  /* note: FPU registers s0-s15 are caller-saved */

  float    s16;
  float    s17;
  float    s18;
  float    s19;
  float    s20;
  float    s21;
  float    s22;
  float    s23;
  float    s24;
  float    s25;
  float    s26;
  float    s27;
  float    s28;
  float    s29;
  float    s30;
  float    s31;

  unsigned fpscr;
#endif
};

/* Traditional typedef for setjmp_buf */

typedef struct setjmp_buf_s jmp_buf[1];

#else
#  error "setjmp() not compiled!"
#endif /* CONFIG_ARCH_ARMV7M */

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

#endif /* __ARCH_ARM_INCLUDE_SETJUMP_H */
