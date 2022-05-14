/****************************************************************************
 * libs/libc/machine/sim/arch_elf.c
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

bool up_checkarch(const Elf32_Ehdr *hdr)
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

int up_relocate(const Elf32_Rel *rel, const Elf32_Sym *sym, uintptr_t addr)
{
  uint32_t *ptr = (uint32_t *)addr;

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

int up_relocateadd(const Elf32_Rela *rel, const Elf32_Sym *sym,
                   uintptr_t addr)
{
  berr("ERROR: Not supported\n");
  return -ENOSYS;
}
