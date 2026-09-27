/****************************************************************************
 * tools/nxflat/nxflat_elf.c
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
 * Reading the object is the one thing mknxflat and ldnxflat both do, so they
 * do it here.  The two also used to disagree about byte order: one swapped
 * the fields it read, the other refused anything but little endian.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "nxflat_elf.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_swap;            /* The object disagrees with this host */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t swap16(uint16_t v)
{
  return g_swap ? (uint16_t)((v >> 8) | (v << 8)) : v;
}

static uint32_t swap32(uint32_t v)
{
  if (!g_swap)
    {
      return v;
    }

  return ((v >> 24) & 0x000000ff) | ((v >> 8) & 0x0000ff00) |
         ((v << 8) & 0x00ff0000) | ((v << 24) & 0xff000000);
}

static void elf_fail(const char *program, const char *path, const char *why)
{
  fprintf(stderr, "%s: %s: %s\n", program, path, why);
  exit(2);
}

/****************************************************************************
 * Name: normalise
 *
 * Description:
 *   Put every table that describes the object into the host's byte order.
 *   The section contents are not touched: they belong to the target.
 *
 ****************************************************************************/

static void normalise(struct nxflat_elf_s *elf)
{
  size_t i;
  size_t j;

  elf->ehdr->e_type      = swap16(elf->ehdr->e_type);
  elf->ehdr->e_machine   = swap16(elf->ehdr->e_machine);
  elf->ehdr->e_entry     = swap32(elf->ehdr->e_entry);
  elf->ehdr->e_shoff     = swap32(elf->ehdr->e_shoff);
  elf->ehdr->e_shentsize = swap16(elf->ehdr->e_shentsize);
  elf->ehdr->e_shnum     = swap16(elf->ehdr->e_shnum);
  elf->ehdr->e_shstrndx  = swap16(elf->ehdr->e_shstrndx);

  elf->shdr = (struct elf32_shdr_s *)(elf->img + elf->ehdr->e_shoff);

  for (i = 0; i < elf->ehdr->e_shnum; i++)
    {
      struct elf32_shdr_s *s = &elf->shdr[i];

      s->sh_name      = swap32(s->sh_name);
      s->sh_type      = swap32(s->sh_type);
      s->sh_flags     = swap32(s->sh_flags);
      s->sh_addr      = swap32(s->sh_addr);
      s->sh_offset    = swap32(s->sh_offset);
      s->sh_size      = swap32(s->sh_size);
      s->sh_link      = swap32(s->sh_link);
      s->sh_info      = swap32(s->sh_info);
      s->sh_addralign = swap32(s->sh_addralign);
      s->sh_entsize   = swap32(s->sh_entsize);
    }

  /* The symbol and relocation tables describe the object too.  Everything
   * else is left alone.
   */

  for (i = 0; i < elf->ehdr->e_shnum; i++)
    {
      struct elf32_shdr_s *s = &elf->shdr[i];

      if (s->sh_type == SHT_SYMTAB && s->sh_entsize == sizeof(*elf->syms))
        {
          struct elf32_sym_s *syms =
            (struct elf32_sym_s *)(elf->img + s->sh_offset);

          for (j = 0; j < s->sh_size / sizeof(*syms); j++)
            {
              syms[j].st_name  = swap32(syms[j].st_name);
              syms[j].st_value = swap32(syms[j].st_value);
              syms[j].st_size  = swap32(syms[j].st_size);
              syms[j].st_shndx = swap16(syms[j].st_shndx);
            }
        }
      else if (s->sh_type == SHT_REL &&
               s->sh_entsize == sizeof(struct elf32_rel_s))
        {
          struct elf32_rel_s *rel =
            (struct elf32_rel_s *)(elf->img + s->sh_offset);

          for (j = 0; j < s->sh_size / sizeof(*rel); j++)
            {
              rel[j].r_offset = swap32(rel[j].r_offset);
              rel[j].r_info   = swap32(rel[j].r_info);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nxflat_elf_read(struct nxflat_elf_s *elf, const char *path,
                     const char *program)
{
  struct stat st;
  uint16_t probe = 1;
  bool host_le;
  int symidx = -1;
  size_t i;
  int fd;

  memset(elf, 0, sizeof(*elf));

  fd = open(path, O_RDONLY);
  if (fd < 0 || fstat(fd, &st) < 0)
    {
      elf_fail(program, path, strerror(errno));
    }

  elf->size = (size_t)st.st_size;
  elf->img  = malloc(elf->size);
  if (elf->img == NULL)
    {
      elf_fail(program, path, "out of memory");
    }

  if (read(fd, elf->img, elf->size) != (ssize_t)elf->size)
    {
      elf_fail(program, path, strerror(errno));
    }

  close(fd);

  if (elf->size < sizeof(struct elf32_ehdr_s) ||
      memcmp(elf->img, "\177ELF", 4) != 0)
    {
      elf_fail(program, path, "not an ELF file");
    }

  elf->ehdr = (struct elf32_ehdr_s *)elf->img;

  if (elf->ehdr->e_ident[4] != ELFCLASS32)
    {
      elf_fail(program, path, "not a 32-bit ELF file");
    }

  if (elf->ehdr->e_ident[5] != ELFDATA2LSB &&
      elf->ehdr->e_ident[5] != ELFDATA2MSB)
    {
      elf_fail(program, path, "has no byte order");
    }

  elf->littleendian = elf->ehdr->e_ident[5] == ELFDATA2LSB;

  host_le = *(const unsigned char *)&probe != 0;
  g_swap  = host_le != elf->littleendian;

  normalise(elf);

  if (elf->ehdr->e_shnum == 0 ||
      elf->ehdr->e_shentsize != sizeof(struct elf32_shdr_s))
    {
      elf_fail(program, path, "has no usable section header table");
    }

  elf->shstrtab = (const char *)
                  (elf->img + elf->shdr[elf->ehdr->e_shstrndx].sh_offset);

  /* The static symbol table.  There is no dynamic one: ld -r does not make
   * .dynsym, and NXFLAT has no use for it -- imports reach a module through
   * the array mknxflat generates.
   */

  for (i = 0; i < elf->ehdr->e_shnum; i++)
    {
      if (elf->shdr[i].sh_type == SHT_SYMTAB)
        {
          symidx = (int)i;
          break;
        }
    }

  if (symidx < 0)
    {
      elf_fail(program, path, "has no symbol table");
    }

  if (elf->shdr[symidx].sh_entsize != sizeof(struct elf32_sym_s))
    {
      elf_fail(program, path, "has symbols of an unexpected size");
    }

  if (elf->shdr[symidx].sh_link >= elf->ehdr->e_shnum)
    {
      elf_fail(program, path, "has a symbol table with no names");
    }

  elf->syms   = (struct elf32_sym_s *)
                (elf->img + elf->shdr[symidx].sh_offset);
  elf->nsyms  = elf->shdr[symidx].sh_size / sizeof(struct elf32_sym_s);
  elf->strtab = (const char *)
                (elf->img + elf->shdr[elf->shdr[symidx].sh_link].sh_offset);
}

const char *nxflat_elf_symname(const struct nxflat_elf_s *elf,
                               const struct elf32_sym_s *sym)
{
  return sym->st_name != 0 ? elf->strtab + sym->st_name : "";
}
