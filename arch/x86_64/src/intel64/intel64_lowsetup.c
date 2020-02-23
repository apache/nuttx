/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_lowsetup.c
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

  ist64     = &ist64_low     + X86_64_LOAD_OFFSET;
  gdt64     = &gdt64_low     + X86_64_LOAD_OFFSET;
  gdt64_ist = &gdt64_ist_low + X86_64_LOAD_OFFSET;

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

