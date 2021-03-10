/****************************************************************************
 * libs/libc/machine/sim/arch_elf.c
 *
 *   Copyright (C) 2012, 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define R_386_32        1
#define R_386_PC32      2

#define ELF_BITS        32
#define ELF_ARCH        EM_386

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the ELF file is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the ELF loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the ELF file.
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

bool up_checkarch(FAR const Elf32_Ehdr *hdr)
{
  return hdr->e_machine == EM_386 || hdr->e_machine == EM_486;
}

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform an architecture-specific ELF relocation.  Every architecture
 *   that uses the ELF loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *         There are a few relocation types for a few architectures that do
 *         not require symbol information.  For those, this value will be
 *         NULL.  Implementations of these functions must be able to handle
 *         that case.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

int up_relocate(FAR const Elf32_Rel *rel, FAR const Elf32_Sym *sym,
                uintptr_t addr)
{
  FAR uint32_t *ptr = (FAR uint32_t *)addr;

  /* All relocations depend upon having valid symbol information. */

  if (sym == NULL)
    {
      return -EINVAL;
    }

  /* Handle the relocation by relocation type */

  switch (ELF32_R_TYPE(rel->r_info))
    {
     case R_386_32:
       *ptr += sym->st_value;
       break;

     case R_386_PC32:
       *ptr += sym->st_value - (uint32_t)((uintptr_t)ptr);
       break;

     default:
       berr("ERROR: Unsupported type %u\n", ELF32_R_TYPE(rel->r_info));
       return -EINVAL;
    }

  return OK;
}

int up_relocateadd(FAR const Elf32_Rela *rel, FAR const Elf32_Sym *sym,
                   uintptr_t addr)
{
  berr("ERROR: Not supported\n");
  return -ENOSYS;
}
