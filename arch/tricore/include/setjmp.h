/****************************************************************************
 * arch/tricore/include/setjmp.h
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

#ifndef __ARCH_TRICORE_INCLUDE_SETJUMP_H
#define __ARCH_TRICORE_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define JB_LPCXI    0
#define JB_LA11     1
#define JB_A2       2
#define JB_A3       3
#define JB_D0       4
#define JB_D1       5
#define JB_D2       6
#define JB_D3       7
#define JB_A4       8
#define JB_A5       9
#define JB_A6       10
#define JB_A7       11
#define JB_D4       12
#define JB_D5       13
#define JB_D6       14
#define JB_D7       15
#define JB_UA11     16
#define JB_REG_NUM  17

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct setjmp_buf_s
{
  uintptr_t regs[JB_REG_NUM];
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
void longjmp(jmp_buf env, int val);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_SETJUMP_H */
