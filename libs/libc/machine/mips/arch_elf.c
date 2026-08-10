/****************************************************************************
 * libs/libc/machine/mips/arch_elf.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <assert.h>

#include <nuttx/elf.h>

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
  /* Make sure it's a MIPS executable */

  if (ehdr->e_machine != EM_MIPS)
    {
      berr("ERROR: Not for MIPS: e_machine=%04x\n", ehdr->e_machine);
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
  if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB)
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      berr("ERROR: Wrong endian-ness: e_ident[EI_DATA]=%02x\n",
           ehdr->e_ident[EI_DATA]);
      return false;
    }

  /* Make sure the entry point address is properly aligned */

  if ((ehdr->e_entry & 3) != 0)
    {
      berr("ERROR: Entry point is not properly aligned: %08" PRIx32 "\n",
           ehdr->e_entry);
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
 *   rel       - The relocation type
 *   sym       - The ELF symbol structure containing the fully resolved
 *               value.  There are a few relocation types for a few
 *               architectures that do not require symbol information.
 *               For those, this value will be NULL.  Implementations of
 *               these functions must be able to handle that case.
 *   addr      - The address that requires the relocation.
 *   arch_data - Pointer to architecture specific elf data container.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

int up_relocate(const Elf_Rel *rel, const Elf_Sym *sym, uintptr_t addr,
                void *arch_data)
{
  /* Variable names correspond to the MIPS ABI specification symbols
   * (A, AHI, AHL, P, S) in lowercase.
   */

  Elf32_Addr *p = (Elf32_Addr *)addr;
  Elf_Word r_type = ELF_R_TYPE(rel->r_info);
  Elf_Addr s = (sym != NULL) ? sym->st_value : 0;
  arch_elfdata_t *data = (arch_elfdata_t *)arch_data;

  if ((sym == NULL && r_type != R_MIPS_NONE) || !data)
    {
      return -EINVAL;
    }

  switch (r_type)
    {
      case R_MIPS_NONE:
        break;

      case R_MIPS_32:
        *p = s + *p;
        break;

      case R_MIPS_26:
        {
          Elf_Addr a = *p & 0x03ffffff;
          Elf_Addr val = ((a << 2) | ((Elf_Addr)p & 0xf0000000)) + s;
          val = (val >> 2) & 0x03ffffff;
          *p &= 0xfc000000;
          *p |= val;
        }
        break;

      case R_MIPS_HI16:
        if (data->pos >= MIPS_HI16_COUNT)
          {
            return -EINVAL;
          }

        data->ahi = *p;
        data->hi[data->pos].ahi = *p;
        data->hi[data->pos].p = p;

        data->pos++;

        break;

      case R_MIPS_LO16:
        {
          int i;
          int16_t a = *p;
          Elf_Addr ahl = (data->ahi << 16) + a;
          Elf32_Addr iword = *p & 0xffff0000;
          iword |= (uint16_t)(ahl + s);
          *p = iword;

          for (i = 0; i < data->pos; i++)
            {
              ahl = (data->hi[i].ahi << 16) + a;

              iword = *(data->hi[i].p) & 0xffff0000;
              iword |= ((ahl + s) - (int16_t)(ahl + s)) >> 16;
              *(data->hi[i].p) = iword;
            }

          data->pos = 0;
        }
        break;

      default:
        berr("ERROR: Unsupported relocation: %" PRIu32 "\n",
             ELF_R_TYPE(rel->r_info));
        return -EINVAL;
    }

  return OK;
}

int up_relocateadd(const Elf32_Rela *rela, const Elf32_Sym *sym,
                   uintptr_t addr, void *arch_data)
{
  berr("Not implemented\n");
  PANIC();
  return -ENOSYS;
}

