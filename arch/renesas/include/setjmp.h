/****************************************************************************
 * arch/renesas/include/setjmp.h
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

#ifndef __ARCH_RENESAS_INCLUDE_SETJUMP_H
#define __ARCH_RENESAS_INCLUDE_SETJUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if defined(CONFIG_ARCH_RENESAS_RX)
struct setjmp_buf_s
{
  unsigned regs[10];
};

/* Traditional typedef for setjmp_buf */

typedef struct setjmp_buf_s jmp_buf[1];

#else
#  error "setjmp() not compiled!"
#endif /* CONFIG_ARCH_RX... */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) noreturn_function;

#endif /* __ARCH_RENESAS_INCLUDE_SETJUMP_H */
