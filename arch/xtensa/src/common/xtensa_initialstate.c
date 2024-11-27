/****************************************************************************
 * arch/xtensa/src/common/xtensa_initialstate.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <arch/irq.h>
#include <arch/xtensa/core.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_corebits.h>
#include <arch/xtensa/xtensa_coproc.h>

#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_THREAD_LOCAL
#  define TCB_SIZE 8
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern int _thread_local_start;
extern int _thread_local_end;
extern int _rodata_reserved_start;
extern int _rodata_reserved_align;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of the
 *   new TCB.
 *
 *   This function must setup the initial architecture registers and/or stack
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
#ifdef CONFIG_SCHED_THREAD_LOCAL
  const uint32_t base = ALIGN_UP((uint32_t)&_rodata_reserved_align,
                                 TCB_SIZE);
#endif

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      tcb->stack_alloc_ptr = g_idlestack;
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

#ifdef CONFIG_STACK_COLORATION
      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      xtensa_stack_color(tcb->stack_alloc_ptr, 0);
#endif /* CONFIG_STACK_COLORATION */
      return;
    }

  /* Initialize the context registers to stack top */

  xcp->regs = (void *)((uint32_t)tcb->stack_base_ptr +
                                 tcb->adj_stack_size -
                                 XCPTCONTEXT_SIZE);

  /* Initialize the xcp registers */

  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  /* Set initial values of registers */

  xcp->regs[REG_PC] = (uint32_t)tcb->start;           /* Task entrypoint                */
  xcp->regs[REG_A0] = 0;                              /* To terminate GDB backtrace     */
  xcp->regs[REG_A1] = (uint32_t)tcb->stack_base_ptr + /* Physical top of stack frame    */
                                tcb->adj_stack_size;

  /* Each task access the TLS variables using the THREADPTR register plus an
   * offset to obtain the address of the variable.
   *
   * TLS layout at link-time (on flash), where 0xNNN is the offset that the
   * linker calculates to a particular TLS variable:
   *
   * LOW ADDRESS
   *         |---------------------------|   Linker Symbols
   *         | Section                   |   --------------
   *         | .flash.rodata             |
   *      0x0|---------------------------| <- _rodata_reserved_start
   *       ^ | Other Data                |
   *       | |---------------------------| <- _thread_local_start
   *       | | .tdata                    | ^              ^
   *       | |                           | |              |
   *       | |                           | | Offset from  |
   *       | |                           | | TLS start    |
   *       | |                           | |              | tls_area_size
   *       V |                           | V              |
   *   0xNNN | int example;              |                |
   *         |                           |                |
   *         |                           |                |
   *         | .tbss                     |                V
   *         |---------------------------| <- _thread_local_end
   *         | Other data                |
   *         | ...                       |
   *         |---------------------------|
   * HIGH ADDRESS
   *
   * Consider the TLS variable `example`. Its location is calculated as:
   *
   *   &example = &_rodata_reserved_start + 0xNNN
   *
   * And the offset 0xNNN can be calculated as:
   *
   *   0xNNN = (&_thread_local_start - &_rodata_reserved_start) +
   *           Offset from TLS start
   *
   * Consider the following diagram for the stack layout:
   *
   * Note: the following diagram shows the stack layout for a particular task
   * and the memory grows towards the stack base (lower memory addresses).
   *
   * HIGH ADDRESS
   *         |---------------------------| <- Top of the stack
   *         |                           |
   *         |                           |
   *         |---------------------------| <- Base of the stack
   *         | .tbss (*)                 |
   *         |                           |
   *         |                           |
   *         | int example;              |
   *       ^ |                           | ^
   *       | |                           | | Offset from TLS start
   *       | | .tdata (*)                | V
   *       | |---------------------------| <- Start of the TLS area
   * 0xNNN | |                           | ^
   *       | |                           | |
   *       | |           ...             | | (_thread_local_start -
   *       | |                           | |   _rodata_reserved_start)
   *       | |                           | |  + align_up(TCB_SIZE,
   *       | |                           | |             tls_section_align)
   *       | |                           | |
   *       | |                           | V
   *       V |                           | <- threadptr register's value
   *
   * LOW ADDRESS
   *
   * At run-time, each task accesses the TLS variables using the THREADPTR
   * register plus an offset to obtain the address of the variable.
   *
   *   &example = THREADPTR + 0xNNN (calculated at link-time)
   *
   * Similarly, example can be accessed as:
   *
   *   &example = Start of the TLS area + Offset from TLS start
   *
   * The start of the TLS area is given by (check xtensa_tls.c):
   *
   *   Start of the TLS area = tcb->stack_alloc_ptr +
   *                           sizeof(struct tls_info_s)
   *
   * Calculate the THREADPTR register's initialization value based on the
   * link-time offset and the TLS area allocated on the stack.
   *
   *   THREADPTR = &example - 0xNNN
   *   THREADPTR = (tcb->stack_alloc_ptr + sizeof(struct tls_info_s)) +
   *               Offset from TLS start - 0xNNN
   *
   * And, finally, based on 0xNNN calculated at link-time:
   *
   *  THREADPTR = (tcb->stack_alloc_ptr + sizeof(struct tls_info_s)) -
   *              (&_thread_local_start - &_rodata_reserved_start) - base
   *
   * Note: Xtensa is slightly different compared to the RISC-V port as there
   * is an implicit aligned TCB_SIZE added to the offset.
   *  - "offset = address - tls_section_vma +
   *              align_up(TCB_SIZE, tls_section_align)"
   *    - TCB_SIZE is hardcoded to 8
   * Refer to https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=bfd/
   * elf32-xtensa.c;h=f078cbde7146675fd2ed82eb5102ae2596c20775;hb=HEAD#l1830
   */

#ifdef CONFIG_SCHED_THREAD_LOCAL
  xcp->regs[REG_THREADPTR] = (uintptr_t)tcb->stack_alloc_ptr +
                              sizeof(struct tls_info_s) -
                             ((uint32_t)&_thread_local_start -
                              (uint32_t)&_rodata_reserved_start) - base;
#endif

  /* Set initial PS to int level 0, user mode. */

#ifdef __XTENSA_CALL0_ABI__
  xcp->regs[REG_PS] = PS_UM;

#else
  /* For windowed ABI set WOE and CALLINC (pretend task was 'call4'd). */

  xcp->regs[REG_PS] = PS_UM | PS_WOE | PS_CALLINC(1);
#endif
}
