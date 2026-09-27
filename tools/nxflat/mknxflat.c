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
 * The reading itself is in nxflat_elf.c, which ldnxflat uses as well.
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

#include "nxflat_elf.h"
#include "nxflat_thunk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STB_WEAK                2
#define MAX_EXPORT_NAMES        1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One imported symbol, in symbol table order */

struct import_s
{
  const char *name;
  int   is_object;
  int   is_weak;
};

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
typedef int (*namefunc_type)(const char *name, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Command line settings (counters but treated like booleans) */

static int verbose = 0;
static int weak_imports = 0;

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
  struct nxflat_elf_s elf;
  size_t strsize = 0;
  size_t i;

  nxflat_elf_read(&elf, elf_filename, program_name);

  /* The size of the string table the symbols point into, which bounds
   * st_name.  nxflat_elf_read() has located the table itself.
   */

  for (i = 0; i < elf.ehdr->e_shnum; i++)
    {
      if (elf.shdr[i].sh_type == SHT_SYMTAB)
        {
          strsize = elf.shdr[elf.shdr[i].sh_link].sh_size;
          break;
        }
    }

  imports = malloc((elf.nsyms + 1) * sizeof(struct import_s));
  if (imports == NULL)
    {
      fprintf(stderr, "Failed to allocate imports\n");
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

  for (i = 0; i < elf.nsyms; i++)
    {
      const struct elf32_sym_s *sym = &elf.syms[i];
      unsigned char info = sym->st_info;

      if (sym->st_shndx != SHN_UNDEF || sym->st_value != 0 ||
          sym->st_name == 0 || sym->st_name >= strsize)
        {
          continue;
        }

      if (ELF32_ST_TYPE(info) == STT_OBJECT)
        {
          /* An undefined object is not something a thunk can stand in
           * for; leave it to the link to complain.
           */

          continue;
        }

      imports[number_undefined].name      = elf.strtab + sym->st_name;
      imports[number_undefined].is_object = 0;
      imports[number_undefined].is_weak   =
        (ELF32_ST_BIND(info) == STB_WEAK);
      number_undefined++;
    }

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

  while ((opt = getopt(argc, argv, "a:o:vw")) != -1)
    {
      switch (opt)
        {
          case 'a':
            arch = optarg;
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
