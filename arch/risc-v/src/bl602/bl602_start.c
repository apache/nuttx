/****************************************************************************
 * arch/risc-v/src/bl602/bl602_start.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <syslog.h>

#ifdef CONFIG_STACK_OVERFLOW_CHECK
#  include "sched/sched.h"
#endif

#include "riscv_internal.h"
#include "chip.h"

#include "bl602_boot2.h"
#include "hardware/bl602_hbn.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) riscv_lowputc(c)
#else
#define showprogress(c)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Dont change the name of variable, since we refer this
 * g_boot2_partition_table in linker script
 */

static struct boot2_partition_table_s g_boot2_partition_table used_data;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void bl602_lowsetup(void);
extern void exception_common(void);
extern void bl602_boardinitialize(void);

/****************************************************************************
 * Name: boot2_get_flash_addr
 ****************************************************************************/

uint32_t noinstrument_function boot2_get_flash_addr(void)
{
  extern uint8_t __boot2_flash_cfg_src[];

  return (uint32_t)(__boot2_flash_cfg_src +
                    (sizeof(g_boot2_partition_table.table.entries[0]) *
                     g_boot2_partition_table.table.table.entry_cnt));
}

#ifdef CONFIG_STACK_OVERFLOW_CHECK
void noinstrument_function locate_code(".tcm_code")
__cyg_profile_func_enter(void *this_fn, void *call_site)
{
  register uintptr_t *sp;
  register uintptr_t *stack_base;

  __asm__("add %0, x0, sp" : "=r"(sp));
  __asm__("add %0, x0, s11" : "=r"(stack_base));

  if (sp < stack_base)
    {
      struct tcb_s *rtcb;

      __asm volatile("csrc mstatus, 8");
      __asm__("li s11, 0");

      /* get current task */

      rtcb = running_task();

      syslog(LOG_EMERG,
             "task %s stack overflow detected! base:0x%x >= sp:0x%x\n",
             get_task_name(rtcb),
             stack_base,
             sp);

      /* PANIC(); */

      while (1)
        ;
    }
}

void noinstrument_function locate_code(".tcm_code")
__cyg_profile_func_exit(void *this_fn, void *call_site)
{
}
#endif

/****************************************************************************
 * Name: bfl_main
 ****************************************************************************/

void bfl_main(void)
{
  /* Configure FPU */

  riscv_fpuconfig();

  /* set interrupt vector */

  asm volatile("csrw mtvec, %0" ::"r"((uintptr_t)exception_common + 2));

  /* Configure the UART so we can get debug output */

  bl602_lowsetup();

  /* HBN Config AON pad input and SMT */

  modifyreg32(BL602_HBN_IRQ_MODE, 0, HBN_IRQ_MODE_REG_AON_PAD_IE_SMT);

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  /* Do board initialization */

  bl602_boardinitialize();

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  while (1)
    ;
}
