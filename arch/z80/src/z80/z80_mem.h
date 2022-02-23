/************************************************************************************
 * arch/z80/src/z80/z80_mem.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_SRC_Z80_Z80_MEM_H
#define __ARCH_Z80_SRC_Z80_Z80_MEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Locate the IDLE thread stack at the end of RAM. */

#define CONFIG_STACK_END   CONFIG_RAM_SIZE
#define CONFIG_STACK_BASE  (CONFIG_STACK_END - CONFIG_IDLETHREAD_STACKSIZE)

/* The heap then extends from the linker determined beginning of the heap (s__HEAP).
 * to the bottom of the IDLE thread stack.  NOTE:  The symbol s__HEAP is not
 * accessible from C because it does not begin with the _ character.  g_heapbase
 * is defined in z80_head.asm to provide that value to the C code.
 */

#define CONFIG_HEAP1_END   CONFIG_STACK_BASE
#define CONFIG_HEAP1_BASE  g_heapbase

/************************************************************************************
 * Public variables
 ************************************************************************************/

/* This is the bottom of the heap as provided by the linker symbol s__HEAP. NOTE:
 * The symbol s__HEAP is not accessible from C because it does not begin with the _
 * character.  g_heapbase is defined in z80_head.asm to provide that value to the C
 * code.
 */

extern const uint16_t g_heapbase;

#endif /* __ARCH_Z80_SRC_Z80_Z80_MEM_H */
