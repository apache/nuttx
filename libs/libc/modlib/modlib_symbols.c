/****************************************************************************
 * libs/libc/modlib/modlib_symbols.c
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/symtab.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

struct eptable_s
{
  FAR uint8_t *epname;      /* Name of global symbol */
  FAR void    *epaddr;      /* Address of global symbol */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct eptable_s global_table[];
extern int nglobals;

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
 *   EINVAL - There is something inconsistent in the symbol table (should
 *            only happen if the file is corrupted).
 *   ESRCH - Symbol has no name
 *
 ****************************************************************************/

static int modlib_symname(FAR struct mod_loadinfo_s *loadinfo,
                          FAR const Elf_Sym *sym, Elf_Off sh_offset)
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

  /* Allocate an I/O buffer.  This buffer is used by mod_symname() to
   * accumulate the variable length symbol name.
   */

  ret = modlib_allocbuffer(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  offset = sh_offset + sym->st_name;

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

      offset += CONFIG_MODLIB_BUFFERINCR;
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Name: modlib_symcallback
 *
 * Description:
 *   modlib_registry_foreach() callback function.  Test if the provided
 *   module, modp, exports the symbol of interest.  If so, return that symbol
 *   value and setup the module dependency relationship.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int modlib_symcallback(FAR struct module_s *modp, FAR void *arg)
{
  FAR struct mod_exportinfo_s *exportinfo = (FAR struct mod_exportinfo_s *)
                                            arg;

  /* Check if this module exports a symbol of that name */

  exportinfo->symbol = symtab_findbyname(modp->modinfo.exports,
                                         exportinfo->name,
                                         modp->modinfo.nexports);

  if (exportinfo->symbol != NULL)
    {
      /* Yes.. save the dependency relationship and return SYM_FOUND to
       * stop the traversal.
       */

#if CONFIG_MODLIB_MAXDEPEND > 0
      int ret = modlib_depend(exportinfo->modp, modp);
      if (ret < 0)
        {
          berr("ERROR: modlib_depend failed: %d\n", ret);
          return ret;
        }
#endif

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

int modlib_readsym(FAR struct mod_loadinfo_s *loadinfo, int index,
                   FAR Elf_Sym *sym, FAR Elf_Shdr *symtab)
{
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (symtab->sh_size / sizeof(Elf_Sym)))
    {
      berr("ERROR: Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = symtab->sh_offset + sizeof(Elf_Sym) * index;

  /* And, finally, read the symbol table entry into memory */

  return modlib_read(loadinfo, (FAR uint8_t *)sym, sizeof(Elf_Sym), offset);
}

/****************************************************************************
 * Name: modlib_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   modp      - Module state information
 *   loadinfo  - Load state information
 *   sym       - Symbol table entry (value might be undefined)
 *   sh_offset - Offset of strtab
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

int modlib_symvalue(FAR struct module_s *modp,
                    FAR struct mod_loadinfo_s *loadinfo, FAR Elf_Sym *sym,
                    Elf_Off sh_offset)
{
  FAR const struct symtab_s *symbol;
  struct mod_exportinfo_s exportinfo;
  uintptr_t secbase;
  int nsymbols;
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

        ret = modlib_symname(loadinfo, sym, sh_offset);
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

        ret = modlib_registry_foreach(modlib_symcallback,
                                      (FAR void *)&exportinfo);
        if (ret < 0)
          {
            berr("ERROR: modlib_symcallback failed: %d\n", ret);
            return ret;
          }

        symbol = exportinfo.symbol;

        /* If the symbol is not exported by any module, then check if the
         * base code exports a symbol of this name.
         */

        if (symbol == NULL)
          {
            modlib_getsymtab(&symbol, &nsymbols);
            symbol = symtab_findbyname(symbol, exportinfo.name,
                                       nsymbols);
          }

        /* Was the symbol found from any exporter? */

        if (symbol == NULL)
          {
            berr("ERROR: SHN_UNDEF: Exported symbol \"%s\" not found\n",
                 loadinfo->iobuffer);
            return -ENOENT;
          }

        /* Yes... add the exported symbol value to the ELF symbol tablei
         * entry
         */

        binfo("SHN_UNDEF: name=%s "
              "%08" PRIxPTR "+%08" PRIxPTR "=%08" PRIxPTR "\n",
              loadinfo->iobuffer,
              (uintptr_t)sym->st_value, (uintptr_t)symbol->sym_value,
              (uintptr_t)(sym->st_value + symbol->sym_value));

        sym->st_value += ((uintptr_t)symbol->sym_value);
      }
      break;

    default:
      {
        secbase = loadinfo->shdr[sym->st_shndx].sh_addr;

        binfo("Other[%d]: %08" PRIxPTR "+%08" PRIxPTR "=%08" PRIxPTR "\n",
              sym->st_shndx,
              (uintptr_t)sym->st_value, secbase,
              (uintptr_t)(sym->st_value + secbase));

        sym->st_value += secbase;
      }
      break;
    }

  return OK;
}

/****************************************************************************
 * Name: modlib_insertsymtab
 *
 * Description:
 *   Insert a symbol into the modules exportinfo array.
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Load state information
 *   shdr     - Symbol table section header
 *   sym      - Symbol table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table
 *            (should only happen if the file is corrupted).
 *
 ****************************************************************************/

int modlib_insertsymtab(FAR struct module_s *modp,
                        struct mod_loadinfo_s *loadinfo,
                        FAR Elf_Shdr *shdr, FAR Elf_Sym *sym)
{
  FAR struct symtab_s *symbol;
  FAR Elf_Shdr *strtab = &loadinfo->shdr[shdr->sh_link];
  int ret = 0;
  int i;
  int j;
  int nsym;
  int symcount;

  if (modp->modinfo.exports != NULL)
    {
      bwarn("Module export information already present - replacing");
      modlib_freesymtab((FAR void *)modp);
    }

  /* Count the "live" symbols */

  nsym = shdr->sh_size / sizeof(Elf_Sym);
  for (i = 0, symcount = 0; i < nsym; i++)
    {
      if (sym[i].st_name != 0)
          symcount++;
    }

  if (symcount > 0)
    {
      modp->modinfo.exports = symbol =
                              loadinfo->exported =
                              lib_malloc(sizeof(*symbol) * symcount);
      if (modp->modinfo.exports)
        {
          /* Build out module's symbol table */

          modp->modinfo.nexports = symcount;
          for (i = 0, j = 0; i < nsym; i++)
            {
              if (sym[i].st_name != 0)
                {
                  ret = modlib_symname(loadinfo, &sym[i], strtab->sh_offset);
                  if (ret < 0)
                    {
                      lib_free((FAR void *)modp->modinfo.exports);
                      modp->modinfo.exports = NULL;
                      return ret;
                    }

                  symbol[j].sym_name =
                      strdup((FAR char *)loadinfo->iobuffer);
                  symbol[j].sym_value =
                      (FAR const void *)(uintptr_t)sym[i].st_value;
                  j++;
                }
            }
        }
      else
        {
          berr("Unable to get memory for exported symbols table");
          ret = -ENOMEM;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: findep
 *
 * Description:
 *   Binary search comparison function
 *
 * Input Parameters:
 *   c1 - Comparand 1
 *   c2 - Comparand 2
 *
 ****************************************************************************/

static int findep(FAR const void *c1, FAR const void *c2)
{
  FAR const struct eptable_s *m1 = (FAR const struct eptable_s *)c1;
  FAR const struct eptable_s *m2 = (FAR const struct eptable_s *)c2;
  return strcmp((FAR const char *)m1->epname, (FAR const char *)m2->epname);
}

/****************************************************************************
 * Name: modlib_findglobal
 *
 * Description:
 *   Find a symbol in our library entry point table
 *
 * Input Parameters:
 *   modp     - Module state information
 *
 ****************************************************************************/

void *modlib_findglobal(FAR struct module_s *modp,
                        FAR struct mod_loadinfo_s *loadinfo,
                        FAR Elf_Shdr *shdr, FAR Elf_Sym *sym)
{
  FAR Elf_Shdr *strtab = &loadinfo->shdr[shdr->sh_link];
  int ret;
  struct eptable_s key;
  FAR struct eptable_s *res;

  ret = modlib_symname(loadinfo, sym, strtab->sh_offset);
  if (ret < 0)
    {
      return NULL;
    }

  key.epname = loadinfo->iobuffer;
  res = bsearch(&key, global_table, nglobals,
                sizeof(struct eptable_s), findep);
  if (res != NULL)
    {
      return res->epaddr;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: modlib_freesymtab
 *
 * Description:
 *   Free a symbol table
 *
 * Input Parameters:
 *   modp     - Module state information
 *
 ****************************************************************************/

void modlib_freesymtab(FAR struct module_s *modp)
{
  FAR const struct symtab_s *symbol;
  int i;

  if ((symbol = modp->modinfo.exports) != NULL)
    {
      for (i = 0; i < modp->modinfo.nexports; i++)
        {
          lib_free((FAR void *)symbol[i].sym_name);
        }

      lib_free((FAR void *)symbol);
    }
}
