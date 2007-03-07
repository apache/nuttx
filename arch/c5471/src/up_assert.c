/************************************************************
 * up_assert.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/irq.h>
#include "os_internal.h"
#include "up_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

static void _up_assert(int errorcode) /* __attribute__ ((noreturn)) */
{
  /* Are we in an interrupt handler or the idle task? */

  if (current_regs || ((_TCB*)g_readytorun.head)->pid == 0)
    {
       (void)irqsave();
        for(;;);
    }
  else
    {
      exit(errorcode);
    }
}

/************************************************************
 * Public Funtions
 ************************************************************/

/************************************************************
 * Name: up_assert
 ************************************************************/

void up_assert(const ubyte *filename, int lineno)
{
  dbg("Assertion failed at file:%s line: %d\n",
      filename, lineno);
  _up_assert(EXIT_FAILURE);
}

/************************************************************
 * Name: up_assert_code
 ************************************************************/

void up_assert_code(const ubyte *filename, int lineno, int errorcode)
{
  dbg("Assertion failed at file:%s line: %d error code: %d\n",
       filename, lineno, errorcode);
  _up_assert(errorcode);
}
