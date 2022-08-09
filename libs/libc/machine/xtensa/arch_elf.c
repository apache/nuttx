/****************************************************************************
 * libs/libc/machine/xtensa/arch_elf.c
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

#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <arch/elf.h>
#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool is_l32r(const unsigned char *p)
{
  return (p[0] & 0xf) == 1;
}

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

bool up_checkarch(const Elf32_Ehdr *ehdr)
{
  /* Make sure it's an Xtensa executable */

  if (ehdr->e_machine != EM_XTENSA)
    {
      berr("ERROR: Not for Xtensa: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that 32-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
    {
      berr("ERROR: Need 32-bit objects: e_ident[EI_CLASS]=%02x\n",
           ehdr->e_ident[EI_CLASS]);
      return false;
    }

  /* Verify endian-ness */

#ifdef CONFIG_ENDIAN_BIG
#error not implemented
  if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB)
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      berr("ERROR: Wrong endian-ness: e_ident[EI_DATA]=%02x\n",
           ehdr->e_ident[EI_DATA]);
      return false;
    }

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

int up_relocate(const Elf32_Rel *rel, const Elf32_Sym *sym, uintptr_t addr)
{
  unsigned int relotype;

  /* All relocations except NONE depend upon having valid symbol
   * information.
   */

  relotype = ELF32_R_TYPE(rel->r_info);
  if (sym == NULL && relotype != R_XTENSA_NONE)
    {
      return -EINVAL;
    }

  /* Handle the relocation by relocation type */

  switch (relotype)
    {
    case R_XTENSA_NONE:
      {
        /* No relocation */
      }
      break;

    default:
      berr("ERROR: Unsupported relocation: %u\n", relotype);
      return -EINVAL;
    }

  return OK;
}

int up_relocateadd(const Elf32_Rela *rel, const Elf32_Sym *sym,
                   uintptr_t addr)
{
  unsigned int relotype;
  unsigned char *p;
  uint32_t value;

  /* All relocations except NONE depend upon having valid symbol
   * information.
   */

  relotype = ELF32_R_TYPE(rel->r_info);
  if (sym == NULL)
    {
      if (relotype != R_XTENSA_NONE)
        {
          return -EINVAL;
        }
    }
  else
    {
      value = sym->st_value + rel->r_addend;
    }

  /* Handle the relocation by relocation type */

  switch (relotype)
    {
    case R_XTENSA_NONE:
      break;

    case R_XTENSA_32:
      (*(uint32_t *)addr) += value;
      break;

    case R_XTENSA_ASM_EXPAND:
      bwarn("WARNING: Ignoring RELA relocation R_XTENSA_ASM_EXPAND %u\n",
          relotype);
      break;

    case R_XTENSA_SLOT0_OP:
      p = (unsigned char *)addr;
      if (is_l32r(p))
        {
          /* Xtensa ISA:
           * L32R forms a virtual address by adding the 16-bit one-extended
           * constant value encoded in the instruction word shifted left by
           * two to the address of the L32R plus three with the two least
           * significant bits cleared.
           */

          uintptr_t base = (addr + 3) & ~3;
          uint16_t imm = (value - base) >> 2;
          if (base + (0xfffc0000 | ((uint32_t)imm << 2)) != value)
            {
              berr("ERROR: Out of range rellocation at %p\n", p);
              return -EINVAL;
            }

          p[1] = imm & 0xff;
          p[2] = (imm >> 8) & 0xff;
          break;
        }

      bwarn("WARNING: Ignoring RELA relocation R_XTENSA_SLOT0_OP %u\n",
            relotype);
      break;

    default:
      berr("ERROR: RELA relocation %u not supported\n", relotype);
      return -EINVAL;
    }

  return OK;
}
