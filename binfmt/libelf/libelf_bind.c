/****************************************************************************
 * binfmt/libelf/libelf_bind.c
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/elf.h>
#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT have to
 * be defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
# define elf_dumpbuffer(m,b,n) binfodumpbuffer(m,b,n)
#else
# define elf_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct elf_symcache_s
{
  dq_entry_t    entry;
  Elf_Sym       sym;
  int           idx;
};

typedef struct elf_symcache_s elf_symcache_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_readrels
 *
 * Description:
 *   Read the (ELF_Rel structure * buffer count) into memory.
 *
 ****************************************************************************/

static inline int elf_readrels(FAR struct elf_loadinfo_s *loadinfo,
                               FAR const Elf_Shdr *relsec,
                               int index, FAR Elf_Rel *rels,
                               int count)
{
  off_t offset;
  int size;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (relsec->sh_size / sizeof(Elf_Rel)))
    {
      berr("Bad relocation symbol index: %d\n", index);
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

  return elf_read(loadinfo, (FAR uint8_t *)rels, size,
                  relsec->sh_offset + offset);
}

/****************************************************************************
 * Name: elf_readrelas
 *
 * Description:
 *   Read the (ELF_Rela structure * buffer count) into memory.
 *
 ****************************************************************************/

static inline int elf_readrelas(FAR struct elf_loadinfo_s *loadinfo,
                                FAR const Elf_Shdr *relsec,
                                int index, FAR Elf_Rela *relas,
                                int count)
{
  off_t offset;
  int size;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (relsec->sh_size / sizeof(Elf_Rela)))
    {
      berr("Bad relocation symbol index: %d\n", index);
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

  return elf_read(loadinfo, (FAR uint8_t *)relas, size,
                  relsec->sh_offset + offset);
}

/****************************************************************************
 * Name: elf_relocate and elf_relocateadd
 *
 * Description:
 *   Perform all relocations associated with a section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int elf_relocate(FAR struct elf_loadinfo_s *loadinfo, int relidx,
                        FAR const struct symtab_s *exports, int nexports)
{
  FAR Elf_Shdr         *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr         *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rel          *rels;
  FAR Elf_Rel          *rel;
  FAR elf_symcache_t   *cache;
  FAR Elf_Sym          *sym;
  FAR dq_entry_t       *e;
  dq_queue_t            q;
  uintptr_t             addr;
  int                   symidx;
  int                   ret;
  int                   i;
  int                   j;

  rels = kmm_malloc(CONFIG_ELF_RELOCATION_BUFFERCOUNT * sizeof(Elf_Rel));
  if (rels == NULL)
    {
      berr("Failed to allocate memory for elf relocation\n");
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

      rel = &rels[i % CONFIG_ELF_RELOCATION_BUFFERCOUNT];

      if (!(i % CONFIG_ELF_RELOCATION_BUFFERCOUNT))
        {
          ret = elf_readrels(loadinfo, relsec, i, rels,
                             CONFIG_ELF_RELOCATION_BUFFERCOUNT);
          if (ret < 0)
            {
              berr("Section %d reloc %d: "
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
          cache = (FAR elf_symcache_t *)e;
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
          if (j < CONFIG_ELF_SYMBOL_CACHECOUNT)
            {
              cache = kmm_malloc(sizeof(elf_symcache_t));
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
              cache = (FAR elf_symcache_t *)dq_remlast(&q);
            }

          sym = &cache->sym;

          /* Read the symbol table entry into memory */

          ret = elf_readsym(loadinfo, symidx, sym);
          if (ret < 0)
            {
              berr("Section %d reloc %d: Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              kmm_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = elf_symvalue(loadinfo, sym, exports, nexports);
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
                  berr("Section %d reloc %d: "
                       "Undefined symbol[%d] has no name: %d\n",
                       relidx, i, symidx, ret);
                }
              else
                {
                  berr("Section %d reloc %d: "
                       "Failed to get value of symbol[%d]: %d\n",
                       relidx, i, symidx, ret);
                  kmm_free(cache);
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

      if (rel->r_offset < 0 ||
          rel->r_offset > dstsec->sh_size - sizeof(uint32_t))
        {
          berr("Section %d reloc %d: Relocation address out of range, "
               "offset %" PRIdPTR " size %jd\n",
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

  kmm_free(rels);
  while ((e = dq_peek(&q)))
    {
      dq_rem(e, &q);
      kmm_free(e);
    }

  return ret;
}

static int elf_relocateadd(FAR struct elf_loadinfo_s *loadinfo, int relidx,
                           FAR const struct symtab_s *exports, int nexports)
{
  FAR Elf_Shdr         *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr         *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rela         *relas;
  FAR Elf_Rela         *rela;
  FAR elf_symcache_t   *cache;
  FAR Elf_Sym          *sym;
  FAR dq_entry_t       *e;
  dq_queue_t            q;
  uintptr_t             addr;
  int                   symidx;
  int                   ret;
  int                   i;
  int                   j;

  relas = kmm_malloc(CONFIG_ELF_RELOCATION_BUFFERCOUNT * sizeof(Elf_Rela));
  if (relas == NULL)
    {
      berr("Failed to allocate memory for elf relocation\n");
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

      rela = &relas[i % CONFIG_ELF_RELOCATION_BUFFERCOUNT];

      if (!(i % CONFIG_ELF_RELOCATION_BUFFERCOUNT))
        {
          ret = elf_readrelas(loadinfo, relsec, i, relas,
                              CONFIG_ELF_RELOCATION_BUFFERCOUNT);
          if (ret < 0)
            {
              berr("Section %d reloc %d: "
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
          cache = (FAR elf_symcache_t *)e;
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
          if (j < CONFIG_ELF_SYMBOL_CACHECOUNT)
            {
              cache = kmm_malloc(sizeof(elf_symcache_t));
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
              cache = (FAR elf_symcache_t *)dq_remlast(&q);
            }

          sym = &cache->sym;

          /* Read the symbol table entry into memory */

          ret = elf_readsym(loadinfo, symidx, sym);
          if (ret < 0)
            {
              berr("Section %d reloc %d: Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              kmm_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = elf_symvalue(loadinfo, sym, exports, nexports);
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
                  berr("Section %d reloc %d: "
                       "Undefined symbol[%d] has no name: %d\n",
                       relidx, i, symidx, ret);
                }
              else
                {
                  berr("Section %d reloc %d: "
                       "Failed to get value of symbol[%d]: %d\n",
                       relidx, i, symidx, ret);
                  kmm_free(cache);
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

      if (rela->r_offset < 0 ||
          rela->r_offset > dstsec->sh_size)
        {
          berr("Section %d reloc %d: Relocation address out of range, "
               "offset %" PRIdPTR " size %jd\n",
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

  kmm_free(relas);
  while ((e = dq_peek(&q)))
    {
      dq_rem(e, &q);
      kmm_free(e);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_bind(FAR struct elf_loadinfo_s *loadinfo,
             FAR const struct symtab_s *exports, int nexports)
{
#ifdef CONFIG_ARCH_ADDRENV
  int status;
#endif
  int ret;
  int i;

  /* Find the symbol and string tables */

  ret = elf_findsymtab(loadinfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate an I/O buffer.  This buffer is used by elf_symname() to
   * accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      berr("elf_allocbuffer failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* If CONFIG_ARCH_ADDRENV=y, then the loaded ELF lies in a virtual address
   * space that may not be in place now.  elf_addrenv_select() will
   * temporarily instantiate that address space.
   */

  ret = elf_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Process relocations in every allocated section */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      /* Get the index to the relocation section */

      int infosec = loadinfo->shdr[i].sh_info;
      if (infosec >= loadinfo->ehdr.e_shnum)
        {
          continue;
        }

      /* Make sure that the section is allocated.  We can't relocated
       * sections that were not loaded into memory.
       */

      if ((loadinfo->shdr[infosec].sh_flags & SHF_ALLOC) == 0)
        {
          continue;
        }

      /* Process the relocations by type */

      if (loadinfo->shdr[i].sh_type == SHT_REL)
        {
          ret = elf_relocate(loadinfo, i, exports, nexports);
        }
      else if (loadinfo->shdr[i].sh_type == SHT_RELA)
        {
          ret = elf_relocateadd(loadinfo, i, exports, nexports);
        }

      if (ret < 0)
        {
          break;
        }
    }

#if defined(CONFIG_ARCH_ADDRENV)
  /* Ensure that the I and D caches are coherent before starting the newly
   * loaded module by cleaning the D cache (i.e., flushing the D cache
   * contents to memory and invalidating the I cache).
   */

#if 0 /* REVISIT... has some problems */
  up_addrenv_coherent(&loadinfo->addrenv);
#else
  up_coherent_dcache(loadinfo->textalloc, loadinfo->textsize);
  up_coherent_dcache(loadinfo->dataalloc, loadinfo->datasize);
#endif

  /* Restore the original address environment */

  status = elf_addrenv_restore(loadinfo);
  if (status < 0)
    {
      berr("ERROR: elf_addrenv_restore() failed: %d\n", status);
      if (ret == OK)
        {
          ret = status;
        }
    }

#else
  /* Ensure that the I and D caches are coherent before starting the newly
   * loaded module by cleaning the D cache (i.e., flushing the D cache
   * contents to memory and invalidating the I cache).
   */

  up_coherent_dcache(loadinfo->textalloc, loadinfo->textsize);
  up_coherent_dcache(loadinfo->dataalloc, loadinfo->datasize);

#endif

  return ret;
}
