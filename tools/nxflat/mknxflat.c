/****************************************************************************
 * tools/nxflat/mknxflat.c
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
 * mknxflat generates the "thunk" assembly file for an NXFLAT module: one
 * stub per imported function, plus the import name string table and the
 * per-process __dyninfo array the loader fills in at load time.
 *
 * This is a port of the tool from the NuttX buildroot NXFLAT toolchain.
 * The one substantive change is that the symbol table is read from the ELF
 * file directly rather than through libbfd.  libbfd is GPL, which an Apache
 * project cannot depend on, and it is awkward to obtain besides -- but the
 * dependency was never deep: the upstream tool used it only to open the
 * file and enumerate symbols, never to relocate or rewrite anything.
 *
 * The emitted text is unchanged.  The format strings live in the .def
 * files, which are carried here byte-for-byte from upstream.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "nxflat_thunk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define dbg(format, ...) \
  do \
    { \
      if (verbose) \
        { \
          printf(format, ##__VA_ARGS__); \
        } \
    } \
  while (0)

/* Just enough of the ELF32 ABI to walk a symbol table.  Spelled out here
 * rather than pulled from <elf.h> so the tool builds on any host.
 */

#define EI_NIDENT       16
#define ELFCLASS32      1
#define ELFDATA2LSB     1
#define ELFDATA2MSB     2

#define SHT_SYMTAB      2
#define SHT_DYNSYM      11

#define SHN_UNDEF       0

#define STB_WEAK        2
#define STT_OBJECT      1

#define ELF_ST_BIND(i)  ((i) >> 4)
#define ELF_ST_TYPE(i)  ((i) & 0x0f)

#define MAX_EXPORT_NAMES 1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct elf32_ehdr_s
{
  unsigned char e_ident[EI_NIDENT];
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

/* One imported symbol, in symbol table order */

struct import_s
{
  const char *name;
  int   is_object;
  int   is_weak;
};

typedef int (*namefunc_type)(const char *name, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Command line settings (counters but treated like booleans) */

static int verbose = 0;
static int weak_imports = 0;
static int dsyms = 0;

/* Characteristics of things */

static int calls_nonreturning_functions = 0;

/* Names of things */

static const char *program_name = NULL;
static const char *elf_filename = NULL;
static const char *out_filename = NULL;

/* The selected architecture's thunk format strings */

static const struct nxflat_thunk_s *thunk = NULL;

/* The imported symbols, in symbol table order */

static struct import_s *imports = NULL;
static long number_undefined = 0;

static int counter;

/* Big-endian input?  ARM is normally little-endian but big-endian ARM
 * exists, so honour EI_DATA rather than assuming.
 */

static int need_swap = 0;

/****************************************************************************
 * Private constant data
 ****************************************************************************/

/* This is the list of names of libc and libpthread functions that
 * do not return.  These may require some special handling -- at a
 * minimum, they must tie up resources that can only be released
 * when the function returns.
 */

static const char *const nonreturners[] =
{
  "abort",                      /* Never returns */
  "exit",                       /* Never returns */
  "_exit",                      /* Never returns */
  "longjmp",                    /* Never returns */
  "_longjmp",                   /* Never returns */
  "pthread_exit",               /* Never returns */
  "siglongjmp",                 /* Never returns */
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: swap16 / swap32
 ****************************************************************************/

static uint16_t swap16(uint16_t v)
{
  return need_swap ? (uint16_t)((v >> 8) | (v << 8)) : v;
}

static uint32_t swap32(uint32_t v)
{
  if (!need_swap)
    {
      return v;
    }

  return ((v & 0x000000fful) << 24) | ((v & 0x0000ff00ul) << 8) |
         ((v & 0x00ff0000ul) >> 8)  | ((v & 0xff000000ul) >> 24);
}

/****************************************************************************
 * Name: xread
 *
 * Description:
 *   Read exactly nbytes at an absolute offset, or die.
 *
 ****************************************************************************/

static void xread(int fd, void *buffer, size_t nbytes, off_t offset)
{
  ssize_t nread;

  if (lseek(fd, offset, SEEK_SET) == (off_t)-1)
    {
      fprintf(stderr, "%s: seek to %ld failed: %s\n",
              elf_filename, (long)offset, strerror(errno));
      exit(2);
    }

  while (nbytes > 0)
    {
      nread = read(fd, buffer, nbytes);
      if (nread < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          fprintf(stderr, "%s: read failed: %s\n",
                  elf_filename, strerror(errno));
          exit(2);
        }
      else if (nread == 0)
        {
          fprintf(stderr, "%s: unexpected end of file\n", elf_filename);
          exit(2);
        }

      buffer  = (char *)buffer + nread;
      nbytes -= nread;
    }
}

/****************************************************************************
 * Name: load_imports
 *
 * Description:
 *   Collect every undefined, non-object symbol from the ELF file, in symbol
 *   table order.
 *
 *   The selection rule is the upstream one.  Symbol typing is not
 *   trustworthy here: imported functions are frequently emitted as
 *   STT_NOTYPE rather than STT_FUNC, while a weakly defined *object* does
 *   show up as an undefined object.  So rather than looking for functions,
 *   this takes everything undefined that is not explicitly an object.  A
 *   genuinely undefined object would be an error, and is left to the link.
 *
 ****************************************************************************/

static void load_imports(void)
{
  struct elf32_ehdr_s ehdr;
  struct elf32_shdr_s *shdrs;
  struct elf32_sym_s *syms;
  char *strtab;
  int wanted = dsyms ? SHT_DYNSYM : SHT_SYMTAB;
  int symidx = -1;
  size_t nsyms;
  size_t strsize;
  size_t i;
  uint16_t probe;
  int host_le;
  int obj_le;
  int fd;

  fd = open(elf_filename, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "%s: cannot open: %s\n",
              elf_filename, strerror(errno));
      exit(2);
    }

  xread(fd, &ehdr, sizeof(ehdr), 0);

  if (memcmp(ehdr.e_ident, "\177ELF", 4) != 0)
    {
      fprintf(stderr, "%s: not an ELF file\n", elf_filename);
      exit(2);
    }

  if (ehdr.e_ident[4] != ELFCLASS32)
    {
      fprintf(stderr, "%s: not a 32-bit ELF file\n", elf_filename);
      exit(2);
    }

  /* Decide whether the host and the object disagree about byte order */

  probe   = 1;
  host_le = *(const unsigned char *)&probe;
  obj_le  = (ehdr.e_ident[5] == ELFDATA2LSB);

  need_swap = (host_le != obj_le);

  /* Re-read the fields that mattered now that byte order is known */

  ehdr.e_shoff     = swap32(ehdr.e_shoff);
  ehdr.e_shnum     = swap16(ehdr.e_shnum);
  ehdr.e_shentsize = swap16(ehdr.e_shentsize);

  if (ehdr.e_shnum == 0 || ehdr.e_shentsize != sizeof(struct elf32_shdr_s))
    {
      fprintf(stderr, "%s: no usable section header table\n", elf_filename);
      exit(2);
    }

  shdrs = malloc((size_t)ehdr.e_shnum * sizeof(struct elf32_shdr_s));
  if (shdrs == NULL)
    {
      fprintf(stderr, "Failed to allocate section headers\n");
      exit(3);
    }

  xread(fd, shdrs, (size_t)ehdr.e_shnum * sizeof(struct elf32_shdr_s),
        ehdr.e_shoff);

  for (i = 0; i < ehdr.e_shnum; i++)
    {
      shdrs[i].sh_type    = swap32(shdrs[i].sh_type);
      shdrs[i].sh_offset  = swap32(shdrs[i].sh_offset);
      shdrs[i].sh_size    = swap32(shdrs[i].sh_size);
      shdrs[i].sh_link    = swap32(shdrs[i].sh_link);
      shdrs[i].sh_entsize = swap32(shdrs[i].sh_entsize);

      if ((int)shdrs[i].sh_type == wanted && symidx < 0)
        {
          symidx = (int)i;
        }
    }

  if (symidx < 0)
    {
      fprintf(stderr, "%s: no %s section\n", elf_filename,
              dsyms ? "dynamic symbol table" : "symbol table");
      exit(2);
    }

  if (shdrs[symidx].sh_entsize != sizeof(struct elf32_sym_s))
    {
      fprintf(stderr, "%s: unexpected symbol entry size\n", elf_filename);
      exit(2);
    }

  nsyms = shdrs[symidx].sh_size / sizeof(struct elf32_sym_s);

  syms = malloc(shdrs[symidx].sh_size);
  if (syms == NULL)
    {
      fprintf(stderr, "Failed to allocate symbol table\n");
      exit(3);
    }

  xread(fd, syms, shdrs[symidx].sh_size, shdrs[symidx].sh_offset);

  /* The linked string table holds the names */

  if (shdrs[symidx].sh_link >= ehdr.e_shnum)
    {
      fprintf(stderr, "%s: symbol table has no string table\n",
              elf_filename);
      exit(2);
    }

  strsize = shdrs[shdrs[symidx].sh_link].sh_size;
  strtab  = malloc(strsize + 1);
  if (strtab == NULL)
    {
      fprintf(stderr, "Failed to allocate string table\n");
      exit(3);
    }

  xread(fd, strtab, strsize, shdrs[shdrs[symidx].sh_link].sh_offset);
  strtab[strsize] = '\0';

  close(fd);

  imports = calloc(nsyms + 1, sizeof(struct import_s));
  if (imports == NULL)
    {
      fprintf(stderr, "Failed to allocate import table\n");
      exit(3);
    }

  /* The ABI marker goes first, so that every module has at least one
   * import and the loader can tell what it was built for.  It is a marker
   * rather than a real import: nothing calls it and no board exports it --
   * the loader matches it by name and skips resolution.
   */

  imports[0].name      = NXFLAT_ABI_SYMBOL;
  imports[0].is_object = 0;
  imports[0].is_weak   = 0;
  number_undefined     = 1;

  for (i = 0; i < nsyms; i++)
    {
      uint32_t st_name  = swap32(syms[i].st_name);
      uint32_t st_value = swap32(syms[i].st_value);
      uint16_t st_shndx = swap16(syms[i].st_shndx);
      unsigned char info = syms[i].st_info;

      if (st_shndx != SHN_UNDEF || st_value != 0 || st_name == 0 ||
          st_name >= strsize)
        {
          continue;
        }

      if (ELF_ST_TYPE(info) == STT_OBJECT)
        {
          /* An undefined object is not something a thunk can stand in
           * for; leave it to the link to complain.
           */

          continue;
        }

      imports[number_undefined].name = &strtab[st_name];
      imports[number_undefined].is_object = 0;
      imports[number_undefined].is_weak =
        (ELF_ST_BIND(info) == STB_WEAK);
      number_undefined++;
    }

  free(shdrs);
  free(syms);

  dbg("Found %ld undefined symbols\n", number_undefined);
}

/****************************************************************************
 * Name: traverse_undefined_functions
 ****************************************************************************/

static int traverse_undefined_functions(void *arg, namefunc_type fn)
{
  long i;

  for (i = 0; i < number_undefined; i++)
    {
      /* Is it imported as a "weak" symbol?  If so, we will process the
       * symbol only if we were requested to do so from the command line.
       */

      if (imports[i].is_weak && weak_imports == 0)
        {
          continue;
        }

      if (fn(imports[i].name, arg) != 0)
        {
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: put_string
 ****************************************************************************/

static void put_string(int fd, const char *string)
{
  ssize_t bytes_available = strlen(string);
  ssize_t bytes_written = write(fd, string, bytes_available);

  if (bytes_written < 0)
    {
      fprintf(stderr,
              "Failed to write %ld bytes of string to output, errno=%d\n",
              (long)bytes_available, errno);
      exit(5);
    }
  else if (bytes_written != bytes_available)
    {
      fprintf(stderr, "Only wrote %ld of %ld bytes of string to output\n",
              (long)bytes_written, (long)bytes_available);
      exit(6);
    }
}

/****************************************************************************
 * Name: does_not_return_name
 ****************************************************************************/

static int does_not_return_name(const char *func_name)
{
  int i;

  for (i = 0; nonreturners[i] != NULL; i++)
    {
      if (strcmp(func_name, nonreturners[i]) == 0)
        {
          return 1;
        }
    }

  return 0;
}

static int check_nonreturning(const char *func_name, void *arg)
{
  if (does_not_return_name(func_name))
    {
      calls_nonreturning_functions = 1;
    }

  return 0;
}

/****************************************************************************
 * Name: put_import_name_strtab / put_dynimport_decl / ...
 *
 * Description:
 *   The four emission passes.  Each walks the import list in the same
 *   order, so the %04d counters line up across passes.
 *
 ****************************************************************************/

static int put_import_name(const char *func_name, void *arg)
{
  char buffer[4096];
  int fd = *(int *)arg;

  snprintf(buffer, sizeof(buffer), thunk->import_name_strtab_format,
           counter, counter, counter, func_name, counter, counter);
  put_string(fd, buffer);
  counter++;
  return 0;
}

static int put_dynimport_decl(const char *func_name, void *arg)
{
  char buffer[4096];
  int fd = *(int *)arg;

  snprintf(buffer, sizeof(buffer), thunk->dynimport_decl_format,
           counter, func_name, counter);
  put_string(fd, buffer);
  counter++;
  return 0;
}

static int put_dynimport_array(const char *func_name, void *arg)
{
  char buffer[4096];
  int fd = *(int *)arg;

  snprintf(buffer, sizeof(buffer), thunk->dynimport_array_format,
           counter, func_name, counter, counter, counter);
  put_string(fd, buffer);
  counter++;
  return 0;
}

static int put_dyncall(const char *func_name, void *arg)
{
  char buffer[4096];
  int fd = *(int *)arg;
  const char *format;

  if (does_not_return_name(func_name))
    {
      format = thunk->nonreturning_dyncall_format;
    }
  else
    {
      format = thunk->dyncall_format;
    }

  snprintf(buffer, sizeof(buffer), format,
           func_name, func_name, func_name, func_name,
           counter, counter, counter, func_name, func_name);
  put_string(fd, buffer);
  counter++;
  return 0;
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s [options] <elf-filename>\n\n", program_name);
  fprintf(stderr, "Where options are one or more of the following.  Note\n");
  fprintf(stderr, "that a space is always required between the\n");
  fprintf(stderr, "option and any following arguments.\n\n");
  fprintf(stderr, "  -a <arch>\n");
  fprintf(stderr, "     Instruction set of the module: arm or thumb2\n");
  fprintf(stderr, "     [thumb2]\n");
  fprintf(stderr, "  -d Use dynamic symbol table. [symtab]\n");
  fprintf(stderr, "  -o <out-filename>\n");
  fprintf(stderr, "     Output to <out-filename> [stdout]\n");
  fprintf(stderr, "  -v Verbose output [no output]\n");
  fprintf(stderr, "  -w Import weakly declared functions, i.e., weakly\n");
  fprintf(stderr, "     declared functions are expected to be\n");
  fprintf(stderr, "     provided at load-time [not imported]\n");
  fprintf(stderr, "\n");
  exit(1);
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(int argc, char **argv)
{
  const char *arch = "thumb2";
  int opt;

  program_name = argv[0];

  while ((opt = getopt(argc, argv, "a:do:vw")) != -1)
    {
      switch (opt)
        {
          case 'a':
            arch = optarg;
            break;

          case 'd':
            dsyms++;
            break;

          case 'o':
            out_filename = optarg;
            break;

          case 'v':
            verbose++;
            break;

          case 'w':
            weak_imports++;
            break;

          default:
            show_usage();
            break;
        }
    }

  if (strcmp(arch, "thumb2") == 0)
    {
      thunk = &g_thunk_thumb2;
    }
  else if (strcmp(arch, "arm") == 0)
    {
      thunk = &g_thunk_arm;
    }
  else
    {
      fprintf(stderr, "Unrecognized architecture '%s'\n\n", arch);
      show_usage();
    }

  if (optind >= argc)
    {
      fprintf(stderr, "No ELF file provided\n\n");
      show_usage();
    }

  elf_filename = argv[optind];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  int fd = 1;

  parse_args(argc, argv);

  if (out_filename != NULL)
    {
      fd = open(out_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (fd < 0)
        {
          fprintf(stderr, "Failed to open %s: %s\n", out_filename,
                  strerror(errno));
          exit(4);
        }
    }

  load_imports();

  traverse_undefined_functions(NULL, check_nonreturning);

  /* Output the thunk file in the same order the upstream tool used:
   * prologue, import name string table, the __dyninfo declarations, the
   * __dyninfo array, then the call thunks.
   */

  put_string(fd, thunk->file_prologue);
  put_string(fd, thunk->import_prologue);

  put_string(fd, thunk->import_name_strtab_prologue);
  counter = 0;
  traverse_undefined_functions(&fd, put_import_name);

  put_string(fd, thunk->dynimport_decl_prologue);
  counter = 0;
  traverse_undefined_functions(&fd, put_dynimport_decl);

  put_string(fd, thunk->dynimport_array_prologue);
  counter = 0;
  traverse_undefined_functions(&fd, put_dynimport_array);
  put_string(fd, thunk->dynimport_array_epilogue);

  put_string(fd, thunk->dyncall_decl_prologue);
  counter = 0;
  traverse_undefined_functions(&fd, put_dyncall);

  put_string(fd, thunk->file_epilogue);

  if (fd != 1)
    {
      close(fd);
    }

  return 0;
}
