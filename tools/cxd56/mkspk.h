/****************************************************************************
 * tools/cxd56/mkspk.h
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

#include "clefia.h"
#include "elf32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EI_MAG0            0      /* File identification */
#define EI_MAG1            1
#define EI_MAG2            2
#define EI_MAG3            3

#define SHT_SYMTAB         2
#define SHT_STRTAB         3

#define PT_LOAD            1

#define alignup(x, a) (((x) + ((a) - 1)) & ~((a) - 1))
#define swap(a, b) { (a) ^= (b); (b) ^= (a); (a) ^= (b); }

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spk_header
  {
    uint8_t magic[4];
    uint8_t cpu;
    uint8_t reserved[11];
    uint32_t entry;
    uint32_t stack;
    uint16_t core;
    uint16_t binaries;
    uint16_t phoffs;
    uint16_t mode;
  };

struct spk_prog_info
  {
    uint32_t load_address;
    uint32_t offset;
    uint32_t size;
    uint32_t memsize;
  };

struct elf_file
  {
    Elf32_Ehdr *ehdr;
    Elf32_Phdr *phdr;
    Elf32_Shdr *shdr;
    Elf32_Sym *symtab;
    int nsyms;
    char *shstring;
    char *string;
    char *data;
  };
