/****************************************************************************
 * arch/risc-v/include/setjmp.h
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

#ifndef __ARCH_RISCV_INCLUDE_SETJUMP_H
#define __ARCH_RISCV_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct setjmp_buf_s
{
  uintptr_t regs[14];

  /* Float callee register : fs0-fs11 */
#ifdef CONFIG_ARCH_QPFPU
  long double fregs[12];
#elif defined(CONFIG_ARCH_DPFPU)
  double fregs[12];
#elif defined(CONFIG_ARCH_FPU)
  float fregs[12];
#endif
};

/* Traditional typedef for setjmp_buf */

typedef struct setjmp_buf_s jmp_buf[1];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) noreturn_function;

#endif
