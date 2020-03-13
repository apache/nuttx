/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_lowsetup.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called from intel64_head BEFORE starting the operating system in order
 *   perform any necessary, early initialization.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
  /* we should be in long mode at this point*/

  /* GDT is loaded with 64bit GDT  */

  /* Paging is enabled*/

  /* Setup pointers for accessing Page table and GDT in high address */

  pdpt = &pdpt_low + X86_64_LOAD_OFFSET;
  pd   = &pd_low   + X86_64_LOAD_OFFSET;
  pt   = &pt_low   + X86_64_LOAD_OFFSET;

  ist64     = (struct ist_s *)((void *)&ist64_low       + X86_64_LOAD_OFFSET);
  gdt64     = (struct gdt_entry_s *)((void *)&gdt64_low + X86_64_LOAD_OFFSET);

  /* reload the GDTR with mapped high memory address */

  setgdt(gdt64, (uintptr_t)(&gdt64_low_end - &gdt64_low) - 1);

  /* Do some checking on CPU compatibilities */

  x86_64_check_and_enable_capability();

  /* perform board-specific initializations */

  x86_64_boardinitialize();

  /* Early serial driver initialization */

  up_earlyserialinit();

  x86_64_timer_calibrate_freq();

#ifdef CONFIG_LIB_SYSCALL
  enable_syscall();
#endif
}

