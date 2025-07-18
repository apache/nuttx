/****************************************************************************
 * boards/arm64/fvp-v8r/fvp-armv8r/kernel/fvp_userspace.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <nuttx/userspace.h>

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NUTTX_USERSPACE
#  error "CONFIG_NUTTX_USERSPACE not defined"
#endif

#if CONFIG_NUTTX_USERSPACE != 0x100000
#  error "CONFIG_NUTTX_USERSPACE must be 0x100000 to match memory.ld"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These 'addresses' of these values are setup by the linker script. */

extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */
static void sig_trampoline(void) naked_function;
static void sig_trampoline(void)
{
  __asm__ __volatile__
  (
    " sub sp, sp, #16\n"  /* Create a stack frame to hold LR */
    " str lr, [sp, #0]\n" /* Save LR on the stack */
    " mov ip0, x0\n"      /* IP=sighand */
    " mov x0, x1\n"       /* R0=signo */
    " mov x1, x2\n"       /* R1=info */
    " mov x2, x3\n"       /* R2=ucontext */
    " blr ip0\n"          /* Call the signal handler */
    " ldr lr, [sp, #0]\n" /* Recover LR in R2 */
    " add sp, sp, #16\n"  /* Destroy the stack frame */
    " mov x0, %0\n"       /* SYS_signal_handler_return */
    " svc %1\n"           /* Return from the SYSCALL */
    :
    : "i"(SYS_signal_handler_return),
      "i"(SYS_syscall)
    :
  );
}

const struct userspace_s userspace locate_data(".userspace") =
{
  /* General memory map */

  .us_entrypoint    = CONFIG_INIT_ENTRYPOINT,
  .us_textstart     = (uintptr_t)_stext,
  .us_textend       = (uintptr_t)_etext,
  .us_datasource    = (uintptr_t)_eronly,
  .us_datastart     = (uintptr_t)_sdata,
  .us_dataend       = (uintptr_t)_edata,
  .us_bssstart      = (uintptr_t)_sbss,
  .us_bssend        = (uintptr_t)_ebss,

  /* Memory manager heap structure */

  .us_heap          = &g_mmheap,

  /* Task/thread startup routines */

  .task_startup     = nxtask_startup,

  /* Signal handler trampoline */

  .signal_handler   = (void *)sig_trampoline,

  /* User-space work queue support (declared in include/nuttx/wqueue.h) */

#ifdef CONFIG_LIBC_USRWORK
  .work_usrstart    = work_usrstart,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
