/****************************************************************************
 * libs/libc/modlib/modlib_bind.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/elf.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I_REL   0    /* Index into relxxx[] arrays for relocations */
#define I_PLT   1    /* ... for PLTs */
#define N_RELS  2    /* Number of relxxx[] indexes */

#ifdef ARCH_ELFDATA
#  define ARCH_ELFDATA_DEF  arch_elfdata_t arch_data; \
                            memset(&arch_data, 0, sizeof(arch_elfdata_t))
#  define ARCH_ELFDATA_PARM &arch_data
#else
#  define ARCH_ELFDATA_DEF
#  define ARCH_ELFDATA_PARM NULL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* REVISIT:  This naming breaks the NuttX coding standard, but is consistent
 * with legacy naming of other ELF types.
 */

typedef struct
{
  dq_entry_t entry;
  Elf_Sym    sym;
  int        idx;
} Elf_SymCache;

struct
{
  int stroff;           /* offset to string table */
  int symoff;           /* offset to symbol table */
  int lsymtab;          /* size of symbol table */
  int relentsz[2];      /* size of relocation entry */
  int reloff[2];        /* offset to the relocation section */
  int relsz[2];         /* size of relocation table */
  int relrela[2];       /* type of relocation type - 0: DT_REL / 1: DT_RELA */
} reldata;

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
                           FAR struct mod_loadinfo_s *loadinfo, int relidx,
                           FAR const struct symtab_s *exports, int nexports)
{
  FAR Elf_Shdr     *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr     *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rel      *rels;
  FAR Elf_Rel      *rel;
  FAR Elf_SymCache *cache;
  FAR Elf_Sym      *sym;
  FAR dq_entry_t   *e;
  dq_queue_t        q;
  uintptr_t         addr;
  int               symidx;
  int               ret = OK;
  int               i;
  int               j;

  /* Define potential architecture specific elf data container */

  ARCH_ELFDATA_DEF;

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

          ret = modlib_readsym(loadinfo, symidx, sym,
                               &loadinfo->shdr[loadinfo->symtabidx]);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              lib_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = modlib_symvalue(modp, loadinfo, sym,
                  loadinfo->shdr[loadinfo->strtabidx].sh_offset,
                  exports, nexports);
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

      if (loadinfo->gotindex >= 0)
        {
          if (sym->st_shndx == SHN_UNDEF)
            {
              /* Symbol type is undefined, we need to set the address
               * to the value of the symbol.
               */

              FAR Elf_Shdr *gotsec = &loadinfo->shdr[loadinfo->gotindex];
              FAR uintptr_t *gotaddr = (FAR uintptr_t *)(gotsec->sh_addr +
                *((FAR uintptr_t *)(dstsec->sh_addr + rel->r_offset)));

              *gotaddr = sym->st_value;
              continue;
            }

          if ((dstsec->sh_flags & SHF_WRITE) == 0)
            {
              /* Skip relocations for read-only sections */

              continue;
            }

          /* Use the GOT to store the address */

          if (rel->r_offset - dstsec->sh_offset >
              dstsec->sh_size)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Relocation address out of range, "
                   "offset %" PRIuPTR " size %ju\n",
                   relidx, i, (uintptr_t)rel->r_offset,
                   (uintmax_t)dstsec->sh_size);
              ret = -EINVAL;
              break;
            }

          addr = dstsec->sh_addr + rel->r_offset - dstsec->sh_offset;
          if (ELF_ST_TYPE(sym->st_info) == STT_SECTION)
            {
              /* Symbol type is section, we need clear the address
               * and keep the original value.
               */

              *(FAR uintptr_t *)addr -=
                 loadinfo->shdr[sym->st_shndx].sh_offset;
            }
          else
            {
              /* Normal symbol, just keep it zero */

              *(FAR uintptr_t *)addr = 0;
            }
        }
      else
        {
          if (rel->r_offset > dstsec->sh_size)
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
        }

      /* Now perform the architecture-specific relocation */

      ret = up_relocate(rel, sym, addr, ARCH_ELFDATA_PARM);
      if (ret < 0)
        {
          berr("ERROR: Section %d reloc %d: Relocation failed: %d\n",
               relidx, i, ret);
          break;
        }
    }

  lib_free(rels);
  while ((e = dq_peek(&q)) != NULL)
    {
      dq_rem(e, &q);
      lib_free(e);
    }

  return ret;
}

static int modlib_relocateadd(FAR struct module_s *modp,
                              FAR struct mod_loadinfo_s *loadinfo,
                              int relidx,
                              FAR const struct symtab_s *exports,
                              int nexports)
{
  FAR Elf_Shdr     *relsec = &loadinfo->shdr[relidx];
  FAR Elf_Shdr     *dstsec = &loadinfo->shdr[relsec->sh_info];
  FAR Elf_Rela     *relas;
  FAR Elf_Rela     *rela;
  FAR Elf_SymCache *cache;
  FAR Elf_Sym      *sym;
  FAR dq_entry_t   *e;
  dq_queue_t        q;
  uintptr_t         addr;
  int               symidx;
  int               ret = OK;
  int               i;
  int               j;

  /* Define potential architecture specific elf data container */

  ARCH_ELFDATA_DEF;

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

          ret = modlib_readsym(loadinfo, symidx, sym,
                               &loadinfo->shdr[loadinfo->symtabidx]);
          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: "
                   "Failed to read symbol[%d]: %d\n",
                   relidx, i, symidx, ret);
              lib_free(cache);
              break;
            }

          /* Get the value of the symbol (in sym.st_value) */

          ret = modlib_symvalue(modp, loadinfo, sym,
                           loadinfo->shdr[loadinfo->strtabidx].sh_offset,
                           exports, nexports);
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

      if (rela->r_offset < 0 ||
          rela->r_offset > dstsec->sh_size)
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

      ret = up_relocateadd(rela, sym, addr, ARCH_ELFDATA_PARM);
      if (ret < 0)
        {
          berr("ERROR: Section %d reloc %d: Relocation failed: %d\n",
               relidx, i, ret);
          break;
        }
    }

  lib_free(relas);
  while ((e = dq_peek(&q)) != NULL)
    {
      dq_rem(e, &q);
      lib_free(e);
    }

  return ret;
}

/****************************************************************************
 * Name: modlib_relocatedyn
 *
 * Description:
 *   Perform all relocations associated with a dynamic section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int modlib_relocatedyn(FAR struct module_s *modp,
                              FAR struct mod_loadinfo_s *loadinfo,
                              int relidx)
{
  FAR Elf_Shdr *shdr = &loadinfo->shdr[relidx];
  FAR Elf_Shdr *symhdr;
  FAR Elf_Dyn  *dyn = NULL;
  FAR Elf_Rel  *rels = NULL;
  FAR Elf_Rel  *rel;
  FAR Elf_Rela *relas = NULL;
  FAR Elf_Rela *rela;
  FAR Elf_Sym  *sym = NULL;
  uintptr_t     addr;
  int           ret;
  int           i;
  int           idx_rel;
  int           idx_sym;

  /* Define potential architecture specific elf data container */

  ARCH_ELFDATA_DEF;

  dyn = lib_malloc(shdr->sh_size);
  if (dyn == NULL)
    {
      berr("Failed to allocate memory for elf dynamic section\n");
      return -ENOMEM;
    }

  ret = modlib_read(loadinfo, (FAR uint8_t *)dyn, shdr->sh_size,
                    shdr->sh_offset);
  if (ret < 0)
    {
      berr("Failed to read dynamic section header");
      lib_free(dyn);
      return ret;
    }

  /* Assume DT_RELA to get maximum size required */

  rels = lib_zalloc(CONFIG_MODLIB_RELOCATION_BUFFERCOUNT * sizeof(Elf_Rela));
  if (!rels)
    {
      berr("Failed to allocate memory for elf relocation rels\n");
      lib_free(dyn);
      return -ENOMEM;
    }

  memset((FAR void *)&reldata, 0, sizeof(reldata));
  relas = (FAR Elf_Rela *)rels;

  for (i = 0; dyn[i].d_tag != DT_NULL; i++)
    {
      switch (dyn[i].d_tag)
        {
          case DT_REL:
            reldata.reloff[I_REL] = dyn[i].d_un.d_val;
            break;
          case DT_RELSZ:
            reldata.relsz[I_REL] = dyn[i].d_un.d_val;
            break;
          case DT_RELENT:
            reldata.relentsz[I_REL] = dyn[i].d_un.d_val;
            break;
          case DT_SYMTAB:
            reldata.symoff = dyn[i].d_un.d_val;
            break;
          case DT_STRTAB:
            reldata.stroff = dyn[i].d_un.d_val;
            break;
          case DT_JMPREL:
            reldata.reloff[I_PLT] = dyn[i].d_un.d_val;
            break;
          case DT_PLTRELSZ:
            reldata.relsz[I_PLT] = dyn[i].d_un.d_val;
            break;
          case DT_PLTREL:
            if (dyn[i].d_un.d_val == DT_REL)
              {
                reldata.relentsz[I_PLT] = sizeof(Elf_Rel);
                reldata.relrela[I_PLT] = 0;
              }
            else
              {
                reldata.relentsz[I_PLT] = sizeof(Elf_Rela);
                reldata.relrela[I_PLT] = 1;
              }
            break;
        }
    }

  symhdr = &loadinfo->shdr[loadinfo->dsymtabidx];
  sym = lib_malloc(symhdr->sh_size);
  if (!sym)
    {
      berr("Error obtaining storage for dynamic symbol table");
      lib_free(rels);
      lib_free(dyn);
      return -ENOMEM;
    }

  ret = modlib_read(loadinfo, (FAR uint8_t *)sym, symhdr->sh_size,
                    symhdr->sh_offset);
  if (ret < 0)
    {
      berr("Error reading dynamic symbol table - %d", ret);
      lib_free(sym);
      lib_free(rels);
      lib_free(dyn);
      return ret;
    }

  reldata.lsymtab = reldata.stroff - reldata.symoff;

  for (idx_rel = 0; idx_rel < N_RELS; idx_rel++)
    {
      int lrelent;

      if ((reldata.relsz[idx_rel] == 0) || (reldata.reloff[idx_rel] == 0))
        {
          continue;
        }

      /* Examine each relocation in the .rel.* section. */

      ret = OK;
      lrelent = reldata.relsz[idx_rel] / reldata.relentsz[idx_rel];

      for (i = 0; i < lrelent; i++)
        {
          /* Process each relocation entry
           * - we cheat by using the fact the 1st two fields of Elf_Rel
           *   and Elf_Rela are identical so can do things based on the
           *   former until it's important
           */

          if (reldata.relrela[idx_rel] == 0)
            {
              rel = &rels[i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT];
              rela = (Elf_Rela *)rel;  /* Just to keep the compiler happy */
            }
          else
            {
              rela = &relas[i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT];
              rel = (Elf_Rel *)rela;
            }

          if (!(i % CONFIG_MODLIB_RELOCATION_BUFFERCOUNT))
            {
              size_t relsize = (sizeof(Elf_Rela) *
                               CONFIG_MODLIB_RELOCATION_BUFFERCOUNT);

              if (reldata.relsz[idx_rel] < relsize)
                {
                  relsize = reldata.relsz[idx_rel];
                }

              ret = modlib_read(loadinfo, (FAR uint8_t *)rels,
                                relsize,
                                reldata.reloff[idx_rel] +
                                i * sizeof(Elf_Rel));

              if (ret < 0)
                {
                  berr("ERROR: Section %d reloc %d:"
                       "Failed to read relocation entry: %d\n",
                       relidx, i, ret);
                  break;
                }
            }

          /* Now perform the architecture-specific relocation */

          if ((idx_sym = ELF_R_SYM(rel->r_info)) != 0)
            {
              /* We have an external reference */

              if (sym[idx_sym].st_shndx == SHN_UNDEF)
                {
                    FAR void *ep;

                    ep = modlib_findglobal(modp, loadinfo, symhdr,
                                           &sym[idx_sym]);
                    if ((ep == NULL) && (ELF_ST_BIND(sym[idx_sym].st_info)
                        != STB_WEAK))
                      {
                        berr("ERROR: Unable to resolve addr of ext ref %s\n",
                             loadinfo->iobuffer);
                        ret = -EINVAL;
                        lib_free(sym);
                        lib_free(rels);
                        lib_free(dyn);
                        return ret;
                      }

                    addr = rel->r_offset + loadinfo->textalloc;

                    if (reldata.relrela[idx_rel] == 1)
                      {
                        addr += rela->r_addend;
                      }

                    *(FAR uintptr_t *)addr = (uintptr_t)ep;
                }
            }
          else
            {
              Elf_Sym dynsym =
                {
                  0
                };

              addr = rel->r_offset - loadinfo->datasec + loadinfo->datastart;

              if (reldata.relrela[idx_rel] == 1)
                {
                  addr += rela->r_addend;
                }

              if ((*(FAR uint32_t *)addr) < loadinfo->datasec)
                {
                  dynsym.st_value = *(FAR uint32_t *)addr +
                                    loadinfo->textalloc;
                }
              else
                {
                  dynsym.st_value = *(FAR uint32_t *)addr -
                                    loadinfo->datasec + loadinfo->datastart;
                }

              ret = up_relocate(rel, &dynsym, addr, ARCH_ELFDATA_PARM);
            }

          if (ret < 0)
            {
              berr("ERROR: Section %d reloc %d: Relocation failed: %d\n",
                   relidx, i, ret);
              lib_free(sym);
              lib_free(rels);
              lib_free(dyn);
              return ret;
            }
        }
    }

  lib_free(sym);
  lib_free(rels);
  lib_free(dyn);

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
 *   exports  - The table of exported symbols
 *   nexports - The number of symbols in the exports table
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_bind(FAR struct module_s *modp,
                FAR struct mod_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports)
{
  int ret;
  int i;

#ifdef CONFIG_ARCH_ADDRENV
  /* If CONFIG_ARCH_ADDRENV=y, then the loaded ELF lies in a virtual address
   * space that may not be in place now.  modlib_addrenv_select() will
   * temporarily instantiate that address space.
   */

  if (loadinfo->addrenv != NULL)
    {
      ret = modlib_addrenv_select(loadinfo);
      if (ret < 0)
        {
          berr("ERROR: modlib_addrenv_select() failed: %d\n", ret);
          return ret;
        }
    }
#endif

  /* Find the symbol and string tables */

  ret = modlib_findsymtab(loadinfo);
  if (ret < 0)
    {
      return ret;
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

      if (loadinfo->ehdr.e_type == ET_DYN)
        {
          modp->dynamic = 1;
          switch (loadinfo->shdr[i].sh_type)
            {
              case SHT_DYNAMIC:
                ret = modlib_relocatedyn(modp, loadinfo, i);
                break;
              case SHT_DYNSYM:
                loadinfo->dsymtabidx = i;
                break;
              case SHT_INIT_ARRAY:
                loadinfo->initarr = loadinfo->shdr[i].sh_addr -
                                    loadinfo->datasec +
                                    loadinfo->datastart;
                loadinfo->ninit = loadinfo->shdr[i].sh_size /
                                  sizeof(uintptr_t);
                break;
              case SHT_FINI_ARRAY:
                loadinfo->finiarr = loadinfo->shdr[i].sh_addr -
                                    loadinfo->datasec +
                                    loadinfo->datastart;
                loadinfo->nfini = loadinfo->shdr[i].sh_size /
                                  sizeof(uintptr_t);
                break;
              case SHT_PREINIT_ARRAY:
                loadinfo->preiarr = loadinfo->shdr[i].sh_addr -
                                    loadinfo->datasec +
                                    loadinfo->datastart;
                loadinfo->nprei = loadinfo->shdr[i].sh_size /
                                  sizeof(uintptr_t);
                break;
            }

          if (ret < 0)
            {
              return ret;
            }
        }
      else
        {
          modp->dynamic = 0;

          /* Make sure that the section is allocated.  We can't
           * relocate sections that were not loaded into memory.
           */

          if ((loadinfo->shdr[i].sh_flags & SHF_ALLOC) == 0 &&
              (loadinfo->shdr[i].sh_flags & SHF_INFO_LINK) == 0)
            {
              continue;
            }

          /* Process the relocations by type */

          switch (loadinfo->shdr[i].sh_type)
            {
              case SHT_REL:
                if ((loadinfo->shdr[infosec].sh_flags & SHF_ALLOC) == 0)
                  {
                    continue;
                  }

                ret = modlib_relocate(modp, loadinfo, i, exports, nexports);
                break;
              case SHT_RELA:
                if ((loadinfo->shdr[infosec].sh_flags & SHF_ALLOC) == 0)
                  {
                    continue;
                  }

                ret = modlib_relocateadd(modp, loadinfo, i, exports,
                                         nexports);
                break;
              case SHT_INIT_ARRAY:
                loadinfo->initarr = loadinfo->shdr[i].sh_addr;
                loadinfo->ninit = loadinfo->shdr[i].sh_size /
                                  sizeof(uintptr_t);
                break;
              case SHT_FINI_ARRAY:
                loadinfo->finiarr = loadinfo->shdr[i].sh_addr;
                loadinfo->nfini = loadinfo->shdr[i].sh_size /
                                  sizeof(uintptr_t);
                break;
            }
        }

      if (ret < 0)
        {
          return ret;
        }
    }

  modp->xipbase = loadinfo->xipbase;

  /* Ensure that the I and D caches are coherent before starting the newly
   * loaded module by cleaning the D cache (i.e., flushing the D cache
   * contents to memory and invalidating the I cache).
   */

  if (loadinfo->textsize > 0)
    {
      up_coherent_dcache(loadinfo->textalloc, loadinfo->textsize);
    }

  if (loadinfo->datasize > 0)
    {
      up_coherent_dcache(loadinfo->datastart, loadinfo->datasize);
    }

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  for (i = 0; loadinfo->ehdr.e_type == ET_REL && i < loadinfo->ehdr.e_shnum;
       i++)
    {
      if (loadinfo->sectalloc[i] == 0)
        {
          continue;
        }

      up_coherent_dcache(loadinfo->sectalloc[i], loadinfo->shdr[i].sh_size);
    }
#endif

#ifdef CONFIG_ARCH_ADDRENV
  if (loadinfo->addrenv != NULL)
    {
      int status = modlib_addrenv_restore(loadinfo);
      if (status < 0)
        {
          berr("ERROR: modlib_addrenv_restore() failed: %d\n", status);
          if (ret == OK)
            {
              ret = status;
            }
        }
    }
#endif

  return ret;
}
