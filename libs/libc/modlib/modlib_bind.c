/****************************************************************************
 * libs/libc/modlib/modlib_bind.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/elf.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* REVISIT:  This naming breaks the NuttX coding standard, but is consistent
 * with legacy naming of other ELF types.
 */

typedef struct
{
  dq_entry_t      entry;
  Elf_Sym         sym;
  int             idx;
} Elf_SymCache;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_readrels
 *
 * Description:
 *   Read the (ELF_Rel structure * buffer count) into memory.
 *
 ****************************************************************************/

static inline int modlib_readrels(FAR struct mod_loadinfo_s *loadinfo,
                                  FAR const Elf_Shdr *relsec,
                                  int index, FAR Elf_Rel *rels,
                                  int count)
{
  off_t offset;
  int size;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (relsec->sh_size / sizeof(Elf_Rel)))
    {
      berr("ERROR: Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = sizeof(Elf_Rel) * index;
  size   = sizeof(Elf_Rel) * count;
  if (offset + size > relsec->sh_size)
    {
      size = relsec->sh_size - offset;
    }

  /* And, finally, read the symbol table entry into memory */

  return modlib_read(loadinfo, (FAR uint8_t *)rels, size,
                     relsec->sh_offset + offset);
}

/****************************************************************************
 * Name: modlib_readrelas
 *
 * Description:
 *   Read the (ELF_Rela structure * buffer count) into memory.
 *
 ****************************************************************************/

static inline int modlib_readrelas(FAR struct mod_loadinfo_s *loadinfo,
                                   FAR const Elf_Shdr *relsec,
                                   int index, FAR Elf_Rela *relas,
                                   int count)
{
  off_t offset;
  int size;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (relsec->sh_size / sizeof(Elf_Rela)))
    {
      berr("ERROR: Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = sizeof(Elf_Rela) * index;
  size   = sizeof(Elf_Rela) * count;
  if (offset + size > relsec->sh_size)
    {
      size = relsec->sh_size - offset;
    }

  /* And, finally, read the symbol table entry into memory */

  return modlib_read(loadinfo, (FAR uint8_t *)relas, size,
                     relsec->sh_offset + offset);
}

/****************************************************************************
 * Name: modlib_relocate and modlib_relocateadd
 *
 * Description:
 *   Perform all relocations associated with a section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int modlib_relocate(FAR struct module_s *modp,
                           FAR struct mod_loadinfo_s *loadinfo, int relidx)
{
  FAR Elf_Shdr *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rel  *rels;
  FAR Elf_Rel  *rel;
  FAR Elf_SymCache *cache;
  FAR Elf_Sym  *sym;
  FAR dq_entry_t *e;
  dq_queue_t      q;
  uintptr_t       addr;
  int             symidx;
  int             ret;
  int             i;
  int             j;

  rels = lib_malloc(CONFIG_MODLIB_RELOCATION_BUFFERCOUNT * sizeof(Elf_Rel));
  if (!rels)
    {
      berr("Failed to allocate memory for elf relocation rels\n");
      return -ENOMEM;
    }

  dq_init(&q);

  /* Examine each relocation in the section.  'relsec' is the section
   * containing the relations.  'dstsec' is the section containing the data
   * to be relocated.
   */

  ret = OK;

  for (i = j = 0; i < relsec->sh_size / sizeof(Elf_Rel); i++)
    {
      /* Read the relocation entry into memory */

      rel = &rels[i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT];

      if (!(i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT))
        {
          ret = modlib_readrels(loadinfo, relsec, i, rels,
                                CONFIG_MODLIB_RELOCATION_BUFFERCOUNT);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read relocation entry: %d\n",
                   relidx, i, ret);
              break;
            }
        }

      /* Get the symbol table index for the relocation.  This is contained
       * in a bit-field within the r_info element.
       */

      symidx = ELF_R_SYM(rel->r_info);

      /* First try the cache */

      sym = NULL;
      for (e = dq_peek(&q); e; e = dq_next(e))
        {
          cache = (FAR Elf_SymCache *)e;
          if (cache->idx == symidx)
            {
              dq_rem(&cache->entry, &q);
              dq_addfirst(&cache->entry, &q);
              sym = &cache->sym;
              break;
            }
        }

      /* If the symbol was not found in the cache, we will need to read the
       * symbol from the file.
       */

      if (sym == NULL)
        {
          if (j < CONFIG_MODLIB_SYMBOL_CACHECOUNT)
            {
              cache = lib_malloc(sizeof(Elf_SymCache));
              if (!cache)
                {
                  berr("Failed to allocate memory for elf symbols\n");
                  ret = -ENOMEM;
                  break;
                }

              j++;
            }
          else
            {
              cache = (FAR Elf_SymCache *)dq_remlast(&q);
            }

          sym = &cache->sym;

          /* Read the symbol table entry into memory */

          ret = modlib_readsym(loadinfo, symidx, sym);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              lib_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = modlib_symvalue(modp, loadinfo, sym);
          if (ret < 0)
            {
              /* The special error -ESRCH is returned only in one condition:
               * The symbol has no name.
               *
               * There are a few relocations for a few architectures that do
               * no depend upon a named symbol.  We don't know if that is the
               * case here, but we will use a NULL symbol pointer to indicate
               * that case to up_relocate().  That function can then do what
               * is best.
               */

              if (ret == -ESRCH)
                {
                  berr("ERROR: Section %d reloc %d: "
                       "Undefined symbol[%d] has no name: %d\n",
                       relidx, i, symidx, ret);
                }
              else
                {
                  berr("ERROR: Section %d reloc %d: "
                       "Failed to get value of symbol[%d]: %d\n",
                       relidx, i, symidx, ret);
                  lib_free(cache);
                  break;
                }
            }

          cache->idx = symidx;
          dq_addfirst(&cache->entry, &q);
        }

      if (sym->st_shndx == SHN_UNDEF && sym->st_name == 0)
        {
          sym = NULL;
        }

      /* Calculate the relocation address. */

      if (rel->r_offset + sizeof(uint32_t) > dstsec->sh_size)
        {
          berr("ERROR: Section %d reloc %d: "
               "Relocation address out of range, "
               "offset %" PRIuPTR " size %ju\n",
               relidx, i, (uintptr_t)rel->r_offset,
               (uintmax_t)dstsec->sh_size);
          ret = -EINVAL;
          break;
        }

      addr = dstsec->sh_addr + rel->r_offset;

      /* Now perform the architecture-specific relocation */

      ret = up_relocate(rel, sym, addr);
      if (ret < 0)
        {
          berr("ERROR: Section %d reloc %d: Relocation failed: %d\n",
               relidx, i, ret);
          break;
        }
    }

  lib_free(rels);
  while ((e = dq_peek(&q)))
    {
      dq_rem(e, &q);
      lib_free(e);
    }

  return ret;
}

static int modlib_relocateadd(FAR struct module_s *modp,
                           FAR struct mod_loadinfo_s *loadinfo, int relidx)
{
  FAR Elf_Shdr *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rela *relas;
  FAR Elf_Rela *rela;
  FAR Elf_SymCache *cache;
  FAR Elf_Sym  *sym;
  FAR dq_entry_t *e;
  dq_queue_t      q;
  uintptr_t       addr;
  int             symidx;
  int             ret;
  int             i;
  int             j;

  relas = lib_malloc(CONFIG_MODLIB_RELOCATION_BUFFERCOUNT *
                     sizeof(Elf_Rela));
  if (!relas)
    {
      berr("Failed to allocate memory for elf relocation relas\n");
      return -ENOMEM;
    }

  dq_init(&q);

  /* Examine each relocation in the section.  'relsec' is the section
   * containing the relations.  'dstsec' is the section containing the data
   * to be relocated.
   */

  ret = OK;

  for (i = j = 0; i < relsec->sh_size / sizeof(Elf_Rela); i++)
    {
      /* Read the relocation entry into memory */

      rela = &relas[i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT];

      if (!(i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT))
        {
          ret = modlib_readrelas(loadinfo, relsec, i, relas,
                                 CONFIG_MODLIB_RELOCATION_BUFFERCOUNT);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read relocation entry: %d\n",
                   relidx, i, ret);
              break;
            }
        }

      /* Get the symbol table index for the relocation.  This is contained
       * in a bit-field within the r_info element.
       */

      symidx = ELF_R_SYM(rela->r_info);

      /* First try the cache */

      sym = NULL;
      for (e = dq_peek(&q); e; e = dq_next(e))
        {
          cache = (FAR Elf_SymCache *)e;
          if (cache->idx == symidx)
            {
              dq_rem(&cache->entry, &q);
              dq_addfirst(&cache->entry, &q);
              sym = &cache->sym;
              break;
            }
        }

      /* If the symbol was not found in the cache, we will need to read the
       * symbol from the file.
       */

      if (sym == NULL)
        {
          if (j < CONFIG_MODLIB_SYMBOL_CACHECOUNT)
            {
              cache = lib_malloc(sizeof(Elf_SymCache));
              if (!cache)
                {
                  berr("Failed to allocate memory for elf symbols\n");
                  ret = -ENOMEM;
                  break;
                }

              j++;
            }
          else
            {
              cache = (FAR Elf_SymCache *)dq_remlast(&q);
            }

          sym = &cache->sym;

          /* Read the symbol table entry into memory */

          ret = modlib_readsym(loadinfo, symidx, sym);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              lib_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = modlib_symvalue(modp, loadinfo, sym);
          if (ret < 0)
            {
              /* The special error -ESRCH is returned only in one condition:
               * The symbol has no name.
               *
               * There are a few relocations for a few architectures that do
               * no depend upon a named symbol.  We don't know if that is the
               * case here, but we will use a NULL symbol pointer to indicate
               * that case to up_relocate().  That function can then do what
               * is best.
               */

              if (ret == -ESRCH)
                {
                  berr("ERROR: Section %d reloc %d: "
                       "Undefined symbol[%d] has no name: %d\n",
                       relidx, i, symidx, ret);
                }
              else
                {
                  berr("ERROR: Section %d reloc %d: "
                       "Failed to get value of symbol[%d]: %d\n",
                       relidx, i, symidx, ret);
                  lib_free(cache);
                  break;
                }
            }

          cache->idx = symidx;
          dq_addfirst(&cache->entry, &q);
        }

      if (sym->st_shndx == SHN_UNDEF && sym->st_name == 0)
        {
          sym = NULL;
        }

      /* Calculate the relocation address. */

      if (rela->r_offset + sizeof(uint32_t) > dstsec->sh_size)
        {
          berr("ERROR: Section %d reloc %d: "
               "Relocation address out of range, "
               "offset %" PRIuPTR " size %ju\n",
               relidx, i, (uintptr_t)rela->r_offset,
               (uintmax_t)dstsec->sh_size);
          ret = -EINVAL;
          break;
        }

      addr = dstsec->sh_addr + rela->r_offset;

      /* Now perform the architecture-specific relocation */

      ret = up_relocateadd(rela, sym, addr);
      if (ret < 0)
        {
          berr("ERROR: Section %d reloc %d: Relocation failed: %d\n",
               relidx, i, ret);
          break;
        }
    }

  lib_free(relas);
  while ((e = dq_peek(&q)))
    {
      dq_rem(e, &q);
      lib_free(e);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by
 *   modlib_setsymtab().
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_bind(FAR struct module_s *modp,
                FAR struct mod_loadinfo_s *loadinfo)
{
  int ret;
  int i;

  /* Find the symbol and string tables */

  ret = modlib_findsymtab(loadinfo);
  if (ret < 0)
    {
      return ret;
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

  /* Process relocations in every allocated section */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      /* Get the index to the relocation section */

      int infosec = loadinfo->shdr[i].sh_info;
      if (infosec >= loadinfo->ehdr.e_shnum)
        {
          continue;
        }

      /* Make sure that the section is allocated.  We can't relocate
       * sections that were not loaded into memory.
       */

      if ((loadinfo->shdr[infosec].sh_flags & SHF_ALLOC) == 0)
        {
          continue;
        }

      /* Process the relocations by type */

      if (loadinfo->shdr[i].sh_type == SHT_REL)
        {
          ret = modlib_relocate(modp, loadinfo, i);
        }
      else if (loadinfo->shdr[i].sh_type == SHT_RELA)
        {
          ret = modlib_relocateadd(modp, loadinfo, i);
        }

      if (ret < 0)
        {
          break;
        }
    }

  /* Ensure that the I and D caches are coherent before starting the newly
   * loaded module by cleaning the D cache (i.e., flushing the D cache
   * contents to memory and invalidating the I cache).
   */

  up_coherent_dcache(loadinfo->textalloc, loadinfo->textsize);
  up_coherent_dcache(loadinfo->datastart, loadinfo->datasize);

  return ret;
}
