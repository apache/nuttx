/****************************************************************************
 * libs/libc/modlib/modlib_symbols.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <elf32.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>
#include <nuttx/binfmt/symtab.h>

#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Amount to reallocate buffer when buffer is full */

#ifndef CONFIG_MODLIB_BUFFERINCR
#  define CONFIG_MODLIB_BUFFERINCR 32
#endif

/* Return values search for exported modules */

#define SYM_NOT_FOUND 0
#define SYM_FOUND     1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mod_exportinfo_s
{
  FAR const char *name;              /* Symbol name to find */
  FAR struct module_s *modp;         /* The module that needs the symbol */
  FAR const struct symtab_s *symbol; /* Symbol info returned (if found) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_symname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should only
 *            happen if the file is corrupted).
 *   ESRCH - Symbol has no name
 *
 ****************************************************************************/

static int modlib_symname(FAR struct mod_loadinfo_s *loadinfo,
                          FAR const Elf32_Sym *sym)
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
      berr("ERROR: Symbol has no name\n");
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
              berr("ERROR: At end of file\n");
              return -EINVAL;
            }

          readlen = loadinfo->filelen - offset;
        }

      /* Read that number of bytes into the array */

      buffer = &loadinfo->iobuffer[bytesread];
      ret = modlib_read(loadinfo, buffer, readlen, offset);
      if (ret < 0)
        {
          berr("ERROR: modlib_read failed: %d\n", ret);
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

      ret = modlib_reallocbuffer(loadinfo, CONFIG_MODLIB_BUFFERINCR);
      if (ret < 0)
        {
          berr("ERROR: mod_reallocbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Name: modlib_symcallback
 *
 * Description:
 *   modlib_registry_foreach() callback function.  Test if the provided module,
 *   modp, exports the symbol of interest.  If so, return that symbol value
 *   and setup the module dependency relationship.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int modlib_symcallback(FAR struct module_s *modp, FAR void *arg)
{
  FAR struct mod_exportinfo_s *exportinfo = (FAR struct mod_exportinfo_s *)arg;
  int ret;

  /* Check if this module exports a symbol of that name */

#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
  exportinfo->symbol = symtab_findorderedbyname(modp->modinfo.exports,
                                                exportinfo->name,
                                                modp->modinfo.nexports);
#else
  exportinfo->symbol = symtab_findbyname(modp->modinfo.exports,
                                         exportinfo->name,
                                         modp->modinfo.nexports);
#endif

   if (exportinfo->symbol != NULL)
     {
       /* Yes.. save the dependency relationship and return SYM_FOUND to
        * stop the traversal.
        */

       ret = modlib_depend(exportinfo->modp, modp);
       if (ret < 0)
         {
           berr("ERROR: modlib_depend failed: %d\n", ret);
           return ret;
         }

       return SYM_FOUND;
     }

   return SYM_NOT_FOUND;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_findsymtab(FAR struct mod_loadinfo_s *loadinfo)
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
      berr("ERROR: No symbols in ELF file\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: modlib_readsym
 *
 * Description:
 *   Read the ELFT symbol structure at the specfied index into memory.
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

int modlib_readsym(FAR struct mod_loadinfo_s *loadinfo, int index,
                   FAR Elf32_Sym *sym)
{
  FAR Elf32_Shdr *symtab = &loadinfo->shdr[loadinfo->symtabidx];
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (symtab->sh_size / sizeof(Elf32_Sym)))
    {
      berr("ERROR: Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = symtab->sh_offset + sizeof(Elf32_Sym) * index;

  /* And, finally, read the symbol table entry into memory */

  return modlib_read(loadinfo, (FAR uint8_t *)sym, sizeof(Elf32_Sym), offset);
}

/****************************************************************************
 * Name: modlib_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should only
 *            happen if the file is corrupted).
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int modlib_symvalue(FAR struct module_s *modp,
                    FAR struct mod_loadinfo_s *loadinfo, FAR Elf32_Sym *sym)
{
  FAR const struct symtab_s *symbol;
  struct mod_exportinfo_s exportinfo;
  uintptr_t secbase;
  int ret;

  switch (sym->st_shndx)
    {
    case SHN_COMMON:
      {
        /* NuttX ELF modules should be compiled with -fno-common. */

        berr("ERROR: SHN_COMMON: Re-compile with -fno-common\n");
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

        ret = modlib_symname(loadinfo, sym);
        if (ret < 0)
          {
            /* There are a few relocations for a few architectures that do
             * no depend upon a named symbol.  We don't know if that is the
             * case here, but return and special error to the caller to
             * indicate the nameless symbol.
             */

            berr("ERROR: SHN_UNDEF: Failed to get symbol name: %d\n", ret);
            return ret;
          }

        /* First check if the symbol is exported by an installed module.
         * Newest modules are installed at the head of the list.  Therefore,
         * if the symbol is exported by numerous modules, then the most
         * recently installed will take precedence.
         */

        exportinfo.name   = (FAR const char *)loadinfo->iobuffer;
        exportinfo.modp   = modp;
        exportinfo.symbol = NULL;

        ret = modlib_registry_foreach(modlib_symcallback, (FAR void *)&exportinfo);
        if (ret < 0)
          {
            berr("ERROR: modlib_symcallback failed: \n", ret);
            return ret;
          }

        symbol = exportinfo.symbol;

        /* If the symbol is not exported by any module, then check if the
         * base code exports a symbol of this name.
         */

        if (symbol == NULL)
          {
#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
            symbol = symtab_findorderedbyname(g_modlib_symtab, exportinfo.name,
                                              g_modlib_nsymbols);
#else
            symbol = symtab_findbyname(g_modlib_symtab, exportinfo.name,
                                       g_modlib_nsymbols);
#endif
          }

        /* Was the symbol found from any exporter? */

        if (symbol == NULL)
          {
            berr("ERROR: SHN_UNDEF: Exported symbol \"%s\" not found\n",
                 loadinfo->iobuffer);
            return -ENOENT;
          }

        /* Yes... add the exported symbol value to the ELF symbol table entry */

        binfo("SHN_ABS: name=%s %08x+%08x=%08x\n",
              loadinfo->iobuffer, sym->st_value, symbol->sym_value,
              sym->st_value + symbol->sym_value);

        sym->st_value += (Elf32_Word)((uintptr_t)symbol->sym_value);
      }
      break;

    default:
      {
        secbase = loadinfo->shdr[sym->st_shndx].sh_addr;

        binfo("Other: %08x+%08x=%08x\n",
              sym->st_value, secbase, sym->st_value + secbase);

        sym->st_value += secbase;
      }
      break;
    }

  return OK;
}
