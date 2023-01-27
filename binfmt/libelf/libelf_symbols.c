/****************************************************************************
 * binfmt/libelf/libelf_symbols.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_symname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should
 *            only happen if the file is corrupted).
 *   ESRCH - Symbol has no name
 *
 ****************************************************************************/

static int elf_symname(FAR struct elf_loadinfo_s *loadinfo,
                       FAR const Elf_Sym *sym)
{
  FAR uint8_t *buffer;
  off_t  offset;
  size_t readlen;
  size_t bytesread;
  int ret;

  /* Get the file offset to the string that is the name of the symbol.  The
   * st_name member holds an offset into the file's symbol string table.
   */

  if (sym->st_name == 0)
    {
      berr("Symbol has no name\n");
      return -ESRCH;
    }

  offset = loadinfo->shdr[loadinfo->strtabidx].sh_offset + sym->st_name;

  /* Loop until we get the entire symbol name into memory */

  bytesread = 0;

  for (; ; )
    {
      /* Get the number of bytes to read */

      readlen = loadinfo->buflen - bytesread;
      if (offset + readlen > loadinfo->filelen)
        {
          if (loadinfo->filelen <= offset)
            {
              berr("At end of file\n");
              return -EINVAL;
            }

          readlen = loadinfo->filelen - offset;
        }

      /* Read that number of bytes into the array */

      buffer = &loadinfo->iobuffer[bytesread];
      ret = elf_read(loadinfo, buffer, readlen, offset + bytesread);
      if (ret < 0)
        {
          berr("elf_read failed: %d\n", ret);
          return ret;
        }

      bytesread += readlen;

      /* Did we read the NUL terminator? */

      if (memchr(buffer, '\0', readlen) != NULL)
        {
          /* Yes, the buffer contains a NUL terminator. */

          return OK;
        }

      /* No.. then we have to read more */

      ret = elf_reallocbuffer(loadinfo, CONFIG_ELF_BUFFERINCR);
      if (ret < 0)
        {
          berr("elf_reallocbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_findsymtab(FAR struct elf_loadinfo_s *loadinfo)
{
  int i;

  /* Find the symbol table section header and its associated string table */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      if (loadinfo->shdr[i].sh_type == SHT_SYMTAB)
        {
          loadinfo->symtabidx = i;
          loadinfo->strtabidx = loadinfo->shdr[i].sh_link;
          break;
        }
    }

  /* Verify that there is a symbol and string table */

  if (loadinfo->symtabidx == 0)
    {
      berr("No symbols in ELF file\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: elf_readsym
 *
 * Description:
 *   Read the ELF symbol structure at the specified index into memory.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   index    - Symbol table index
 *   sym      - Location to return the table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_readsym(FAR struct elf_loadinfo_s *loadinfo, int index,
                FAR Elf_Sym *sym)
{
  FAR Elf_Shdr *symtab = &loadinfo->shdr[loadinfo->symtabidx];
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (symtab->sh_size / sizeof(Elf_Sym)))
    {
      berr("Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = symtab->sh_offset + sizeof(Elf_Sym) * index;

  /* And, finally, read the symbol table entry into memory */

  return elf_read(loadinfo, (FAR uint8_t *)sym, sizeof(Elf_Sym), offset);
}

/****************************************************************************
 * Name: elf_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should
 *            only happen if the file is corrupted).
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int elf_symvalue(FAR struct elf_loadinfo_s *loadinfo, FAR Elf_Sym *sym,
                 FAR const struct symtab_s *exports, int nexports)
{
  FAR const struct symtab_s *symbol;
  uintptr_t secbase;
  int ret;

  switch (sym->st_shndx)
    {
    case SHN_COMMON:
      {
        /* NuttX ELF modules should be compiled with -fno-common. */

        berr("SHN_COMMON: Re-compile with -fno-common\n");
        return -ENOSYS;
      }

    case SHN_ABS:
      {
        /* st_value already holds the correct value */

        binfo("SHN_ABS: st_value=%08lx\n", (long)sym->st_value);
        return OK;
      }

    case SHN_UNDEF:
      {
        /* Get the name of the undefined symbol */

        ret = elf_symname(loadinfo, sym);
        if (ret < 0)
          {
            /* There are a few relocations for a few architectures that do
             * no depend upon a named symbol.  We don't know if that is the
             * case here, but return and special error to the caller to
             * indicate the nameless symbol.
             */

            berr("SHN_UNDEF: Failed to get symbol name: %d\n", ret);
            return ret;
          }

        /* Check if the base code exports a symbol of this name */

        symbol = symtab_findbyname(exports, (FAR char *)loadinfo->iobuffer,
                                   nexports);
        if (!symbol)
          {
            berr("SHN_UNDEF: Exported symbol \"%s\" not found\n",
                 loadinfo->iobuffer);
            return -ENOENT;
          }

        /* Yes... add the exported symbol value to the ELF symbol table
         * entry
         */

        binfo("SHN_UNDEF: name=%s "
              "%08" PRIxPTR "+%08" PRIxPTR "=%08" PRIxPTR "\n",
              loadinfo->iobuffer, (uintptr_t)sym->st_value,
              (uintptr_t)symbol->sym_value,
              (uintptr_t)(sym->st_value + (uintptr_t)symbol->sym_value));

        sym->st_value += ((uintptr_t)symbol->sym_value);
      }
      break;

    default:
      {
        secbase = loadinfo->shdr[sym->st_shndx].sh_addr;

        binfo("Other: %08" PRIxPTR "+%08" PRIxPTR "=%08" PRIxPTR "\n",
              (uintptr_t)sym->st_value, secbase,
              (uintptr_t)(sym->st_value + secbase));

        sym->st_value += secbase;
      }
      break;
    }

  return OK;
}
