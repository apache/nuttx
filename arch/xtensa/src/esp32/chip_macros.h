/****************************************************************************
 * arch/xtensa/src/esp32/chip_macros.h
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H
#define __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ESP32_PID) && defined(CONFIG_BUILD_PROTECTED)
#include "hardware/esp32_pid.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the name of the section containing the Xtensa low level handlers
 * that is used by the board linker scripts.
 */

#define HANDLER_SECTION .iram1

#if defined(CONFIG_ESP32_PID) && defined(CONFIG_BUILD_PROTECTED)

/* Definitions for the PIDs reserved for Kernel and Userspace */

#  define PIDCTRL_PID_KERNEL            0   /* Privileged */

#ifdef CONFIG_ESP32_USER_DATA_EXTMEM

/* Allocating user data in External RAM is currently limited to only using
 * privileged PIDs (0 and 1).
 */

#  define PIDCTRL_PID_USER              1   /* Privileged */
#else
#  define PIDCTRL_PID_USER              5   /* Non-privileged */
#endif

/* Macros for privilege handling with the PID Controller peripheral */

#define xtensa_saveprivilege(regs,var)     ((var) = (regs)[REG_INT_CTX])
#define xtensa_restoreprivilege(regs,var)  ((regs)[REG_INT_CTX] = (var))

#define xtensa_lowerprivilege(regs) ((regs)[REG_INT_CTX] = PIDCTRL_PID_USER)
#define xtensa_raiseprivilege(regs) ((regs)[REG_INT_CTX] = PIDCTRL_PID_KERNEL)

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
  .global	g_cpu_intstack_top
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 15 */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Macro to get the current core ID. Only uses the reg given as an argument.
 * Reading PRID on the ESP32 gives us 0xCDCD on the PRO processor (0)
 * and 0xABAB on the APP CPU (1). We can distinguish between the two by
 * checking bit 13: it's 1 on the APP and 0 on the PRO processor.
 */

    .macro getcoreid reg
    rsr.prid \reg
    extui \reg,\reg,13,1
    .endm

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the correct interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
    .macro setintstack tmp1 tmp2
    getcoreid \tmp1                   /* tmp1 = Core ID (0 or 1) */
    movi  \tmp2, g_cpu_intstack_top   /* tmp2 = Array of stack pointers */
    addx4 \tmp2, \tmp1, \tmp2         /* tmp2 = tmp2 + (tmp1 << 2) */
    l32i  a1, \tmp2, 0                /* a1   = *tmp2 */
    .endm
#endif

/****************************************************************************
 * Name: get_prev_pid
 *
 * Description:
 *   Retrieve PID information from interruptee.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   out    - Temporary and output register
 *
 * Exit Conditions:
 *   PID value to be returned will be written to "out" register.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_PID
    .macro get_prev_pid level out
    movi    \out, PIDCTRL_FROM_1_REG + (\level - 1) * 0x4
    l32i    \out, \out, 0
    extui   \out, \out, 0, 3
    .endm
#endif

/****************************************************************************
 * Name: set_next_pid
 *
 * Description:
 *   Configure the PID Controller for the new execution context.
 *
 * Entry Conditions:
 *   in     - PID to be set
 *   tmp    - Temporary register
 *
 * Exit Conditions:
 *   Register "in" has been trashed.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_PID
    .macro set_next_pid in tmp
    movi    \tmp, PIDCTRL_PID_NEW_REG
    s32i    \in, \tmp, 0           /* Set new PID */

    movi    \tmp, PIDCTRL_PID_DELAY_REG
    movi    \in, 0x0
    s32i    \in, \tmp, 0           /* Set delay (cycles) for PID change */

    movi    \tmp, PIDCTRL_PID_CONFIRM_REG
    movi    \in, 0x1
    s32i    \in, \tmp, 0           /* Confirm change to the new PID */
    .endm
#endif

/****************************************************************************
 * Name: exception_entry_hook
 *
 * Description:
 *   Perform chip-specific exception entry operations.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   reg_sp - Stack pointer
 *   tmp    - Temporary register
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_GENERAL_EXCEPTION_HOOKS
    .macro exception_entry_hook level reg_sp tmp

  /* Save PID information from interruptee when handling User (Level 1) and
   * Software-triggered interrupts (Level 3).
   */

    .ifeq (\level - 1) & (\level - 3)
    get_prev_pid \level \tmp
    s32i    \tmp, \reg_sp, (4 * REG_INT_CTX) /* Save PID into context area */
    .endif
    .endm
#endif

/****************************************************************************
 * Name: exception_exit_hook
 *
 * Description:
 *   Perform chip-specific exception exit operations.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   reg_sp - Stack pointer
 *   tmp1   - Temporary register 1
 *   tmp2   - Temporary register 2
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_GENERAL_EXCEPTION_HOOKS
    .macro exception_exit_hook level reg_sp tmp1 tmp2

  /* Configure the PID Controller for the new execution context before
   * returning from User (Level 1) and Software-triggered interrupts
   * (Level 3).
   */

    .ifeq (\level - 1) & (\level - 3)
    l32i    \tmp1, \reg_sp, (4 * REG_INT_CTX)
    set_next_pid \tmp1 \tmp2
    .endif
    .endm
#endif

#endif /* __ASSEMBLY */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H */
