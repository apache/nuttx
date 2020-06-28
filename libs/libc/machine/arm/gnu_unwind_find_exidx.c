/****************************************************************************
 * libs/libc/machine/arm/gnu_unwind_find_exidx.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/elf.h>
#include <unwind.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct __EIT_entry
{
  _uw fnoffset;
  _uw content;
} __EIT_entry;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static __EIT_entry *__exidx_start_elf;
static __EIT_entry *__exidx_end_elf;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern __EIT_entry __exidx_start;
extern __EIT_entry __exidx_end;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  __gnu_Unwind_Find_exidx
 *
 * Description:
 *    This function is called (if exists) by the gcc generated unwind
 *    run-time in order to retrieve an alternative .ARM.exidx Exception
 *    index section.
 *    This is the case for an ELF module loaded by the elf binary loader.
 *    It is needed to support exception handling for loadable ELF modules.
 *
 *    NOTES:
 *
 *     1. The section to be searched is chosen by the address of the calling
 *        site: if we are in a runtime loaded ELF, the code will be executed
 *        in ram ( > 0x20000000 ) otherwise we will  be executing code  from
 *        flash (0x08000000) (running nuttx from ram will break this logic)
 *
 *     2. __exidx_start  and  __exidx_end refers to main nuttx elf image and
 *        are defined in its linker script.
 *
 *     2. __exidx_start_elf  and  __exidx_end_elf refers  to the elf module
 *        loaded by the elf binary loader, and are initialized at run-time.
 *
 *     3. TODO: if nuttx itself is running from ram, this logic will not work
 *
 *     4. TODO: in order to support multiple elf modules running at the same
 *        time, this error logic needs to be extended to store multiple
 *        start/end ranges that refers to the loaded binaries.
 *
 ****************************************************************************/

int up_init_exidx(Elf_Addr start, Elf_Word size)
{
  __exidx_start_elf = (__EIT_entry *)start;
  __exidx_end_elf   = (__EIT_entry *)(start + size);
  return 0;
}

_Unwind_Ptr __gnu_Unwind_Find_exidx(_Unwind_Ptr return_address, int *nrecp)
{
  if (return_address < 0x20000000)
    {
      *nrecp = &__exidx_end - &__exidx_start;
      return (_Unwind_Ptr)&__exidx_start;
    }
  else
    {
      *nrecp = __exidx_end_elf - __exidx_start_elf;
      return (_Unwind_Ptr)__exidx_start_elf;
    }
}
