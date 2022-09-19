/****************************************************************************
 * libs/libc/machine/arm/gnu_unwind_find_exidx.c
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

#include <nuttx/elf.h>
#include <unwind.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static __EIT_entry *__exidx_start_elf;
static __EIT_entry *__exidx_end_elf;

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
      *nrecp = __exidx_end - __exidx_start;
      return (_Unwind_Ptr)__exidx_start;
    }
  else
    {
      *nrecp = __exidx_end_elf - __exidx_start_elf;
      return (_Unwind_Ptr)__exidx_start_elf;
    }
}
