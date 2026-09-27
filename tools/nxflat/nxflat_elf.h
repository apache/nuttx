/****************************************************************************
 * tools/nxflat/nxflat_elf.h
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

#ifndef __TOOLS_NXFLAT_NXFLAT_ELF_H
#define __TOOLS_NXFLAT_NXFLAT_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ELF32, declared here and not taken from a host elf.h: macOS has none, and
 * a tool that writes a target's binary should not use the host's idea of the
 * format.
 */

#define ELFCLASS32              1
#define ELFDATA2LSB             1
#define ELFDATA2MSB             2

#define ET_REL                  1
#define EM_ARM                  40

#define SHT_SYMTAB              2
#define SHT_NOBITS              8
#define SHT_REL                 9

#define SHF_WRITE               0x1
#define SHF_ALLOC               0x2
#define SHF_EXECINSTR           0x4

#define SHN_UNDEF               0
#define SHN_COMMON              0xfff2

#define STT_NOTYPE              0
#define STT_OBJECT              1
#define STT_FUNC                2

#define ELF32_R_SYM(i)          ((i) >> 8)
#define ELF32_R_TYPE(i)         ((i) & 0xff)
#define ELF32_ST_TYPE(i)        ((i) & 0x0f)
#define ELF32_ST_BIND(i)        ((i) >> 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct elf32_ehdr_s
{
  unsigned char e_ident[16];
  uint16_t e_type;
  uint16_t e_machine;
  uint32_t e_version;
  uint32_t e_entry;
  uint32_t e_phoff;
  uint32_t e_shoff;
  uint32_t e_flags;
  uint16_t e_ehsize;
  uint16_t e_phentsize;
  uint16_t e_phnum;
  uint16_t e_shentsize;
  uint16_t e_shnum;
  uint16_t e_shstrndx;
};

struct elf32_shdr_s
{
  uint32_t sh_name;
  uint32_t sh_type;
  uint32_t sh_flags;
  uint32_t sh_addr;
  uint32_t sh_offset;
  uint32_t sh_size;
  uint32_t sh_link;
  uint32_t sh_info;
  uint32_t sh_addralign;
  uint32_t sh_entsize;
};

struct elf32_sym_s
{
  uint32_t st_name;
  uint32_t st_value;
  uint32_t st_size;
  unsigned char st_info;
  unsigned char st_other;
  uint16_t st_shndx;
};

struct elf32_rel_s
{
  uint32_t r_offset;
  uint32_t r_info;
};

/* One object, read and put into the host's byte order.
 *
 * The tables that describe it -- headers, symbols, relocation entries --
 * are normalised, so a tool reads a field without asking whose byte order
 * it is in.  Section contents are not: they are the target's bytes, and are
 * written out again as they were read.
 */

struct nxflat_elf_s
{
  uint8_t *img;                /* The file */
  size_t   size;
  bool     littleendian;       /* The byte order the object's contents are in */
  struct elf32_ehdr_s *ehdr;
  struct elf32_shdr_s *shdr;   /* e_shnum of them */
  const char *shstrtab;
  struct elf32_sym_s  *syms;   /* The static symbol table */
  size_t      nsyms;
  const char *strtab;          /* The names those symbols point into */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_elf_read
 *
 * Description:
 *   Read one ELF32 object and locate its section headers, symbol table and
 *   string tables.  A bad file is reported against program and the tool
 *   exits; neither tool has anything else to try.
 *
 ****************************************************************************/

void nxflat_elf_read(struct nxflat_elf_s *elf, const char *path,
                     const char *program);

/****************************************************************************
 * Name: nxflat_elf_symname
 *
 * Description:
 *   The name of a symbol, or "" for one that has none.
 *
 ****************************************************************************/

const char *nxflat_elf_symname(const struct nxflat_elf_s *elf,
                               const struct elf32_sym_s *sym);

#endif /* __TOOLS_NXFLAT_NXFLAT_ELF_H */
