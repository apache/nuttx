/****************************************************************************
 * libs/libc/machine/sim/arch_elf64.c
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

/* References:
 *    AMD64 ABI Draft 0.98
 *    http://refspecs.linuxbase.org/elf/x86_64-abi-0.98.pdf
 *    4.4 Relocation
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define R_X86_64_NONE   0
#define R_X86_64_64     1
#define R_X86_64_PC32   2
#define R_X86_64_PLT32  4
#define R_X86_64_32     10
#define R_X86_64_32S    11
#define R_X86_64_PC64   24

/****************************************************************************
 * Private Data Types
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

bool up_checkarch(const Elf64_Ehdr *ehdr)
{
  /* Make sure it's an x86_64 executable */

  if (ehdr->e_machine != EM_X86_64)
    {
      berr("ERROR: Not for x86_64: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that 64-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS64)
    {
      berr("ERROR: Need 64-bit objects: e_ident[EI_CLASS]=%02x\n",
           ehdr->e_ident[EI_CLASS]);
      return false;
    }

  /* Verify endian-ness */

#ifdef CONFIG_ENDIAN_BIG
#error x86_64 is LE
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      berr("ERROR: Wrong endian-ness: e_ident[EI_DATA]=%02x\n",
           ehdr->e_ident[EI_DATA]);
      return false;
    }

  /* TODO:  Check ABI here. */

  return true;
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

int up_relocate(const Elf64_Rel *rel, const Elf64_Sym *sym, uintptr_t addr)
{
  berr("Not implemented\n");
  return -ENOSYS;
}

int up_relocateadd(const Elf64_Rela *rel, const Elf64_Sym *sym,
                   uintptr_t addr)
{
  unsigned int relotype;
  uint64_t value;

  relotype = ELF64_R_TYPE(rel->r_info);

  if (sym == NULL && relotype != R_X86_64_NONE)
    {
      return -EINVAL;
    }

  /* A ... The addend (rel->r_addend)
   * P ... The location (addr)
   * L ... The PLT location (sym->st_value)
   * S ... symbol value (sym->st_value)
   */

  switch (relotype)
    {
      case R_X86_64_64: /* S + A */
        *(uint64_t *)addr = sym->st_value + rel->r_addend;
        break;
      case R_X86_64_PC32:  /* S + A - P */
      case R_X86_64_PLT32: /* L + A - P */
        value = sym->st_value + rel->r_addend - addr;
        if ((int32_t)value != value)
          {
            berr("ERROR: Out of range relocation: %d\n", relotype);
            return -EINVAL;
          }

        *(uint32_t *)addr = value;
        break;
      case R_X86_64_PC64:  /* S + A - P */
        value = sym->st_value + rel->r_addend - addr;
        *(uint64_t *)addr = value;
        break;
      case R_X86_64_32:  /* S + A */
      case R_X86_64_32S: /* S + A */
        value = sym->st_value + rel->r_addend;
        if ((relotype == R_X86_64_32 && (uint32_t)value != value) ||
            (relotype == R_X86_64_32S && (int32_t)value != value))
          {
            berr("ERROR: Out of range relocation: %d\n", relotype);
            return -EINVAL;
          }

        *(uint32_t *)addr = value;
        break;
      default:
        berr("ERROR: Unsupported relocation: %d\n", relotype);
        return -EINVAL;
    }

  return OK;
}
