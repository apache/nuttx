/****************************************************************************
 * binfmt/elf.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/elf.h>

#include "libelf/libelf.h"

#ifdef CONFIG_ELF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT
 * have to be defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifndef CONFIG_ELF_STACKSIZE
#  define CONFIG_ELF_STACKSIZE 2048
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
#  define elf_dumpbuffer(m,b,n) binfodumpbuffer(m,b,n)
#else
#  define elf_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp,
                          FAR const char *filename,
                          FAR const struct symtab_s *exports,
                          int nexports);
#ifdef CONFIG_ELF_COREDUMP
static int elf_dumpbinary(FAR struct memory_region_s *regions,
                          FAR struct lib_outstream_s *stream,
                          pid_t pid);
#endif
#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_BINFMT)
static void elf_dumploadinfo(FAR struct elf_loadinfo_s *loadinfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_elfbinfmt =
{
  NULL,             /* next */
  elf_loadbinary,   /* load */
  NULL,             /* unload */
#ifdef CONFIG_ELF_COREDUMP
  elf_dumpbinary,   /* coredump */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_dumploadinfo
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_BINFMT)
static void elf_dumploadinfo(FAR struct elf_loadinfo_s *loadinfo)
{
  int i;

  binfo("LOAD_INFO:\n");
  binfo("  textalloc:    %08lx\n", (long)loadinfo->textalloc);
  binfo("  dataalloc:    %08lx\n", (long)loadinfo->dataalloc);
  binfo("  textsize:     %ld\n",   (long)loadinfo->textsize);
  binfo("  datasize:     %ld\n",   (long)loadinfo->datasize);
  binfo("  textalign:    %zu\n",   loadinfo->textalign);
  binfo("  dataalign:    %zu\n",   loadinfo->dataalign);
  binfo("  filelen:      %ld\n",   (long)loadinfo->filelen);
#ifdef CONFIG_BINFMT_CONSTRUCTORS
  binfo("  ctoralloc:    %08lx\n", (long)loadinfo->ctoralloc);
  binfo("  ctors:        %08lx\n", (long)loadinfo->ctors);
  binfo("  nctors:       %d\n",    loadinfo->nctors);
  binfo("  dtoralloc:    %08lx\n", (long)loadinfo->dtoralloc);
  binfo("  dtors:        %08lx\n", (long)loadinfo->dtors);
  binfo("  ndtors:       %d\n",    loadinfo->ndtors);
#endif
  binfo("  symtabidx:    %d\n",    loadinfo->symtabidx);
  binfo("  strtabidx:    %d\n",    loadinfo->strtabidx);

  binfo("ELF Header:\n");
  binfo("  e_ident:      %02x %02x %02x %02x\n",
    loadinfo->ehdr.e_ident[0], loadinfo->ehdr.e_ident[1],
    loadinfo->ehdr.e_ident[2], loadinfo->ehdr.e_ident[3]);
  binfo("  e_type:       %04x\n",  loadinfo->ehdr.e_type);
  binfo("  e_machine:    %04x\n",  loadinfo->ehdr.e_machine);
  binfo("  e_version:    %08x\n",  loadinfo->ehdr.e_version);
  binfo("  e_entry:      %08lx\n", (long)loadinfo->ehdr.e_entry);
  binfo("  e_phoff:      %ju\n",   (uintmax_t)loadinfo->ehdr.e_phoff);
  binfo("  e_shoff:      %ju\n",   (uintmax_t)loadinfo->ehdr.e_shoff);
  binfo("  e_flags:      %08x\n" , loadinfo->ehdr.e_flags);
  binfo("  e_ehsize:     %d\n",    loadinfo->ehdr.e_ehsize);
  binfo("  e_phentsize:  %d\n",    loadinfo->ehdr.e_phentsize);
  binfo("  e_phnum:      %d\n",    loadinfo->ehdr.e_phnum);
  binfo("  e_shentsize:  %d\n",    loadinfo->ehdr.e_shentsize);
  binfo("  e_shnum:      %d\n",    loadinfo->ehdr.e_shnum);
  binfo("  e_shstrndx:   %d\n",    loadinfo->ehdr.e_shstrndx);

  if (loadinfo->shdr && loadinfo->ehdr.e_shnum > 0)
    {
      for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
        {
          FAR Elf_Shdr *shdr = &loadinfo->shdr[i];
          binfo("Sections %d:\n", i);
          binfo("  sh_name:      %08x\n",  shdr->sh_name);
          binfo("  sh_type:      %08x\n",  shdr->sh_type);
          binfo("  sh_flags:     %08jx\n", (uintmax_t)shdr->sh_flags);
          binfo("  sh_addr:      %08jx\n", (uintmax_t)shdr->sh_addr);
          binfo("  sh_offset:    %ju\n",   (uintmax_t)shdr->sh_offset);
          binfo("  sh_size:      %ju\n",   (uintmax_t)shdr->sh_size);
          binfo("  sh_link:      %d\n",    shdr->sh_link);
          binfo("  sh_info:      %d\n",    shdr->sh_info);
          binfo("  sh_addralign: %ju\n",   (uintmax_t)shdr->sh_addralign);
          binfo("  sh_entsize:   %ju\n",   (uintmax_t)shdr->sh_entsize);
        }
    }
}
#else
#  define elf_dumploadinfo(i)
#endif

/****************************************************************************
 * Name: elf_dumpentrypt
 ****************************************************************************/

#ifdef CONFIG_ELF_DUMPBUFFER
static void elf_dumpentrypt(FAR struct binary_s *binp,
                            FAR struct elf_loadinfo_s *loadinfo)
{
#ifdef CONFIG_ARCH_ADDRENV
  int ret;

  /* If CONFIG_ARCH_ADDRENV=y, then the loaded ELF lies in a virtual address
   * space that may not be in place now.  elf_addrenv_select() will
   * temporarily instantiate that address space.
   */

  ret = elf_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_select() failed: %d\n", ret);
      return;
    }
#endif

  elf_dumpbuffer("Entry code", (FAR const uint8_t *)binp->entrypt,
                 MIN(loadinfo->textsize - loadinfo->ehdr.e_entry, 512));

#ifdef CONFIG_ARCH_ADDRENV
  /* Restore the original address environment */

  ret = elf_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_restore() failed: %d\n", ret);
    }
#endif
}
#else
#  define elf_dumpentrypt(b,l)
#endif

/****************************************************************************
 * Name: elf_loadbinary
 *
 * Description:
 *   Verify that the file is an ELF binary and, if so, load the ELF
 *   binary into memory
 *
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp,
                          FAR const char *filename,
                          FAR const struct symtab_s *exports,
                          int nexports)
{
  struct elf_loadinfo_s loadinfo;  /* Contains globals for libelf */
  int                   ret;

  binfo("Loading file: %s\n", filename);

  /* Initialize the ELF library to load the program binary. */

  ret = elf_init(filename, &loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("Failed to initialize for load of ELF program: %d\n", ret);
      goto errout_with_init;
    }

  /* Load the program binary */

  ret = elf_load(&loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_init;
    }

  /* Bind the program to the exported symbol table */

  if (loadinfo.ehdr.e_type == ET_REL)
    {
      ret = elf_bind(&loadinfo, exports, nexports);
      if (ret != 0)
        {
          berr("Failed to bind symbols program binary: %d\n", ret);
          goto errout_with_load;
        }

      binp->entrypt = (main_t)(loadinfo.textalloc + loadinfo.ehdr.e_entry);
    }
  else if (loadinfo.ehdr.e_type == ET_EXEC)
    {
      if (nexports > 0)
        {
          berr("Cannot bind exported symbols to a "\
                                    "fully linked executable\n");
          ret = -ENOEXEC;
          goto errout_with_load;
        }

      /* The entrypoint for a fully linked executable can be found directly */

      binp->entrypt = (main_t)(loadinfo.ehdr.e_entry);
    }

  else
    {
        berr("Unexpected elf type %d\n", loadinfo.ehdr.e_type);
        ret = -ENOEXEC;
    }

  /* Return the load information */

  binp->stacksize = CONFIG_ELF_STACKSIZE;

  /* Add the ELF allocation to the alloc[] only if there is no address
   * environment.  If there is an address environment, it will automatically
   * be freed when the function exits
   *
   * REVISIT:  If the module is loaded then unloaded, wouldn't this cause
   * a memory leak?
   */

#ifdef CONFIG_ARCH_ADDRENV
  /* Save the address environment in the binfmt structure.  This will be
   * needed when the module is executed.
   */

  binp->addrenv = loadinfo.addrenv;

#else
  binp->alloc[0]  = (FAR void *)loadinfo.textalloc;
  binp->alloc[1]  = (FAR void *)loadinfo.dataalloc;
#ifdef CONFIG_BINFMT_CONSTRUCTORS
  binp->alloc[2]  = loadinfo.ctoralloc;
  binp->alloc[3]  = loadinfo.dtoralloc;
#endif
#endif

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Save information about constructors and destructors. */

  binp->ctors     = loadinfo.ctors;
  binp->nctors    = loadinfo.nctors;

  binp->dtors     = loadinfo.dtors;
  binp->ndtors    = loadinfo.ndtors;
#endif

  elf_dumpentrypt(binp, &loadinfo);
  elf_uninit(&loadinfo);
  return OK;

errout_with_load:
  elf_unload(&loadinfo);
errout_with_init:
  elf_uninit(&loadinfo);
  return ret;
}

/****************************************************************************
 * Name: elf_dumpbinary
 *
 * Description:
 *   Generat the core dump stream as ELF structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ELF_COREDUMP
static int elf_dumpbinary(FAR struct memory_region_s *regions,
                          FAR struct lib_outstream_s *stream,
                          pid_t pid)
{
  struct elf_dumpinfo_s dumpinfo;

  dumpinfo.regions = regions;
  dumpinfo.stream  = stream;
  dumpinfo.pid     = pid;

  return elf_coredump(&dumpinfo);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_initialize
 *
 * Description:
 *   In order to use the ELF binary format, this function must be called
 *   during system initialization to register the ELF binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  binfo("Registering ELF\n");

  ret = register_binfmt(&g_elfbinfmt);
  if (ret != 0)
    {
      berr("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: elf_uninitialize
 *
 * Description:
 *   Unregister the ELF binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void elf_uninitialize(void)
{
  unregister_binfmt(&g_elfbinfmt);
}

#endif /* CONFIG_ELF */
