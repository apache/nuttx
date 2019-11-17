/****************************************************************************
 * arch/arm/include/setjmp.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: David S. Alessio <David@DSA.Consulting>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_SETJUMP_H
#define __ARCH_ARM_INCLUDE_SETJUMP_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ARMV7M
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

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) noreturn_function;

#endif /* __ARCH_ARM_INCLUDE_SETJUMP_H */
