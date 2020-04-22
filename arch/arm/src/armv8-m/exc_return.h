/************************************************************************************
 * arch/arm/src/armv8-m/exc_return.h
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_EXC_RETURN_H
#define __ARCH_ARM_SRC_ARMV8_M_EXC_RETURN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The processor saves an EXC_RETURN value to the LR on exception entry. The
 * exception mechanism relies on this value to detect when the processor has
 * completed an exception handler.
 *
 * Bits [31:28] of an EXC_RETURN value are always 1.  When the processor loads a
 * value matching this pattern to the PC it detects that the operation is a not
 * a normal branch operation and instead, that the exception is complete.
 * Therefore, it starts the exception return sequence.
 *
 * Bits[4:0] of the EXC_RETURN value indicate the required return stack and eventual
 * processor mode.  The remaining bits of the EXC_RETURN value should be set to 1.
 */

/* EXC_RETURN_BASE: Bits that are always set in an EXC_RETURN value. */

#define EXC_RETURN_BASE          0xffffffe1

/* EXC_RETURN_PROCESS_STACK: The exception saved (and will restore) the hardware
 * context using the process stack pointer (if not set, the context was saved
 * using the main stack pointer)
 */

#define EXC_RETURN_PROCESS_STACK (1 << 2)

/* EXC_RETURN_THREAD_MODE: The exception will return to thread mode (if not set,
 * return stays in handler mode)
 */

#define EXC_RETURN_THREAD_MODE   (1 << 3)

/* EXC_RETURN_STD_CONTEXT: The state saved on the stack does not include the
 * volatile FP registers and FPSCR.  If this bit is clear, the state does include
 * these registers.
 */

#define EXC_RETURN_STD_CONTEXT   (1 << 4)

/* EXC_RETURN_HANDLER: Return to handler mode. Exception return gets state from
 * the main stack. Execution uses MSP after return.
 */

#define EXC_RETURN_HANDLER       0xfffffff1

/* EXC_RETURN_PRIVTHR: Return to privileged thread mode. Exception return gets
 * state from the main stack. Execution uses MSP after return.
 */

#if !defined(CONFIG_ARMV8M_LAZYFPU) && defined(CONFIG_ARCH_FPU)
#  define EXC_RETURN_PRIVTHR     (EXC_RETURN_BASE | EXC_RETURN_THREAD_MODE)
#else
#  define EXC_RETURN_PRIVTHR     (EXC_RETURN_BASE | EXC_RETURN_STD_CONTEXT | \
                                  EXC_RETURN_THREAD_MODE)
#endif

/* EXC_RETURN_UNPRIVTHR: Return to unprivileged thread mode. Exception return gets
 * state from the process stack. Execution uses PSP after return.
 */

#if !defined(CONFIG_ARMV8M_LAZYFPU) && defined(CONFIG_ARCH_FPU)
#  define EXC_RETURN_UNPRIVTHR   (EXC_RETURN_BASE | EXC_RETURN_THREAD_MODE | \
                                  EXC_RETURN_PROCESS_STACK)
#else
#  define EXC_RETURN_UNPRIVTHR   (EXC_RETURN_BASE | EXC_RETURN_STD_CONTEXT | \
                                  EXC_RETURN_THREAD_MODE | EXC_RETURN_PROCESS_STACK)
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_ARMV8_M_EXC_RETURN_H */
