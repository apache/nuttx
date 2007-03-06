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
#include <debug.h>
#include <8052.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include "up_internal.h"
#include "up_mem.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

#ifdef CONFIG_FRAME_DUMP
static void _up_puthex(ubyte hex) __naked
{
  hex; /* To avoid unreferenced argument warning */
  _asm
        mov     a, dpl
        ljmp    PM2_ENTRY_PHEX
  _endasm;
}

static void _up_putspace(void) __naked
{
  _asm
        mov     a, #0x20
        ljmp    PM2_ENTRY_COUT
  _endasm;
}

static void _up_putcolon(void) __naked
{
  _asm
        mov     a, #0x3a
        lcall	PM2_ENTRY_COUT
  _endasm;
}

static void _up_putnl(void) __naked
{
  _asm
	ljmp	PM2_ENTRY_NEWLINE
  _endasm;
}

static void _up_puts(__code char *ptr)
{
  for (; *ptr; ptr++)
    {
       up_putc(*ptr);
    }
}

static void _up_dump16(__code char *ptr, ubyte msb, ubyte lsb)
{
  _up_puts(ptr);
  _up_puthex(msb);
  _up_puthex(lsb);
  _up_putnl();
}

static void _up_dump8(__code char *ptr, ubyte b)
{
  _up_puts(ptr);
  _up_puthex(b);
  _up_putnl();
}
#endif

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: up_dumpstack
 ************************************************************/

#ifdef CONFIG_FRAME_DUMP
void up_dumpstack(void)
{
  NEAR ubyte *start = (NEAR ubyte *)(UP_STACK_BASE & 0xf0);
  NEAR ubyte *end   = (NEAR ubyte *)SP;
  ubyte i;

  while (start < end)
    {
      _up_puthex((ubyte)start);
      _up_putcolon();

      for (i = 0; i < 8; i++)
        {
          _up_putspace();
          _up_puthex(*start);
          start++;
        }
      _up_putnl();
    }
}
#endif

/************************************************************
 * Name: up_dumpframe
 ************************************************************/

#ifdef CONFIG_FRAME_DUMP
void up_dumpframe(FAR struct xcptcontext *context)
{
  FAR ubyte *start = &context->stack[context->nbytes - FRAME_SIZE];
  _up_dump16(" RET  ", start[FRAME_RETMS], start[FRAME_RETLS]);
  _up_dump8(" IE   ", start[FRAME_IE]);
  _up_dump16(" DPTR ", start[FRAME_DPH], start[FRAME_DPL]);
  _up_dump8(" PSW  ", start[FRAME_PSW]);
}
#endif
