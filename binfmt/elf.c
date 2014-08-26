/****************************************************************************
 * binfmt/elf.c
 *
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <elf32.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/elf.h>

#include "libelf/libelf.h"

#ifdef CONFIG_ELF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_BINFMT have to be
 * defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifndef CONFIG_ELF_STACKSIZE
#  define CONFIG_ELF_STACKSIZE 2048
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
# define elf_dumpbuffer(m,b,n) bvdbgdumpbuffer(m,b,n)
#else
# define elf_dumpbuffer(m,b,n)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp);
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
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
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_dumploadinfo
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void elf_dumploadinfo(FAR struct elf_loadinfo_s *loadinfo)
{
  int i;

  bdbg("LOAD_INFO:\n");
  bdbg("  textalloc:    %08lx\n", (long)loadinfo->textalloc);
  bdbg("  dataalloc:    %08lx\n", (long)loadinfo->dataalloc);
  bdbg("  textsize:     %ld\n",   (long)loadinfo->textsize);
  bdbg("  datasize:     %ld\n",   (long)loadinfo->datasize);
  bdbg("  filelen:      %ld\n",   (long)loadinfo->filelen);
#ifdef CONFIG_BINFMT_CONSTRUCTORS
  bdbg("  ctoralloc:    %08lx\n", (long)loadinfo->ctoralloc);
  bdbg("  ctors:        %08lx\n", (long)loadinfo->ctors);
  bdbg("  nctors:       %d\n",    loadinfo->nctors);
  bdbg("  dtoralloc:    %08lx\n", (long)loadinfo->dtoralloc);
  bdbg("  dtors:        %08lx\n", (long)loadinfo->dtors);
  bdbg("  ndtors:       %d\n",    loadinfo->ndtors);
#endif
  bdbg("  filfd:        %d\n",    loadinfo->filfd);
  bdbg("  symtabidx:    %d\n",    loadinfo->symtabidx);
  bdbg("  strtabidx:    %d\n",    loadinfo->strtabidx);

  bdbg("ELF Header:\n");
  bdbg("  e_ident:      %02x %02x %02x %02x\n",
    loadinfo->ehdr.e_ident[0], loadinfo->ehdr.e_ident[1],
    loadinfo->ehdr.e_ident[2], loadinfo->ehdr.e_ident[3]);
  bdbg("  e_type:       %04x\n",  loadinfo->ehdr.e_type);
  bdbg("  e_machine:    %04x\n",  loadinfo->ehdr.e_machine);
  bdbg("  e_version:    %08x\n",  loadinfo->ehdr.e_version);
  bdbg("  e_entry:      %08lx\n", (long)loadinfo->ehdr.e_entry);
  bdbg("  e_phoff:      %d\n",    loadinfo->ehdr.e_phoff);
  bdbg("  e_shoff:      %d\n",    loadinfo->ehdr.e_shoff);
  bdbg("  e_flags:      %08x\n" , loadinfo->ehdr.e_flags);
  bdbg("  e_ehsize:     %d\n",    loadinfo->ehdr.e_ehsize);
  bdbg("  e_phentsize:  %d\n",    loadinfo->ehdr.e_phentsize);
  bdbg("  e_phnum:      %d\n",    loadinfo->ehdr.e_phnum);
  bdbg("  e_shentsize:  %d\n",    loadinfo->ehdr.e_shentsize);
  bdbg("  e_shnum:      %d\n",    loadinfo->ehdr.e_shnum);
  bdbg("  e_shstrndx:   %d\n",    loadinfo->ehdr.e_shstrndx);

  if (loadinfo->shdr && loadinfo->ehdr.e_shnum > 0)
    {
      for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
        {
          FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];
          bdbg("Sections %d:\n", i);
          bdbg("  sh_name:      %08x\n", shdr->sh_name);
          bdbg("  sh_type:      %08x\n", shdr->sh_type);
          bdbg("  sh_flags:     %08x\n", shdr->sh_flags);
          bdbg("  sh_addr:      %08x\n", shdr->sh_addr);
          bdbg("  sh_offset:    %d\n",   shdr->sh_offset);
          bdbg("  sh_size:      %d\n",   shdr->sh_size);
          bdbg("  sh_link:      %d\n",   shdr->sh_link);
          bdbg("  sh_info:      %d\n",   shdr->sh_info);
          bdbg("  sh_addralign: %d\n",   shdr->sh_addralign);
          bdbg("  sh_entsize:   %d\n",   shdr->sh_entsize);
        }
    }
}
#else
# define elf_dumploadinfo(i)
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
      bdbg("ERROR: elf_addrenv_select() failed: %d\n", ret);
      return;
    }
#endif

  elf_dumpbuffer("Entry code", (FAR const uint8_t*)binp->entrypt,
                 MIN(loadinfo->textsize - loadinfo->ehdr.e_entry, 512));

#ifdef CONFIG_ARCH_ADDRENV
  /* Restore the original address environment */

  ret = elf_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_addrenv_restore() failed: %d\n", ret);
    }
#endif
}
#else
# define elf_dumpentrypt(b,l)
#endif

/****************************************************************************
 * Name: elf_loadbinary
 *
 * Description:
 *   Verify that the file is an ELF binary and, if so, load the ELF
 *   binary into memory
 *
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp)
{
  struct elf_loadinfo_s loadinfo;  /* Contains globals for libelf */
  int                   ret;

  bvdbg("Loading file: %s\n", binp->filename);

  /* Initialize the ELF library to load the program binary. */

  ret = elf_init(binp->filename, &loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to initialize for load of ELF program: %d\n", ret);
      goto errout;
    }

  /* Load the program binary */

  ret = elf_load(&loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_init;
    }

  /* Bind the program to the exported symbol table */

  ret = elf_bind(&loadinfo, binp->exports, binp->nexports);
  if (ret != 0)
    {
      bdbg("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  /* Return the load information */

  binp->entrypt   = (main_t)(loadinfo.textalloc + loadinfo.ehdr.e_entry);
  binp->stacksize = CONFIG_ELF_STACKSIZE;

  /* Add the ELF allocation to the alloc[] only if there is no address
   * environment.  If there is an address environment, it will automatically
   * be freed when the function exits
   *
   * REVISIT:  If the module is loaded then unloaded, wouldn't this cause
   * a memory leak?
   */

#ifdef CONFIG_ARCH_ADDRENV
#  warning "REVISIT"
#else
  binp->alloc[0]  = (FAR void *)loadinfo.textalloc;
#endif

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Save information about constructors.  NOTE:  destructors are not
   * yet supported.
   */

  binp->alloc[1]  = loadinfo.ctoralloc;
  binp->ctors     = loadinfo.ctors;
  binp->nctors    = loadinfo.nctors;

  binp->alloc[2]  = loadinfo.dtoralloc;
  binp->dtors     = loadinfo.dtors;
  binp->ndtors    = loadinfo.ndtors;
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Save the address environment in the binfmt structure.  This will be
   * needed when the module is executed.
   */

  up_addrenv_clone(&loadinfo.addrenv, &binp->addrenv);
#endif

  elf_dumpentrypt(binp, &loadinfo);
  elf_uninit(&loadinfo);
  return OK;

errout_with_load:
  elf_unload(&loadinfo);
errout_with_init:
  elf_uninit(&loadinfo);
errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_initialize
 *
 * Description:
 *   ELF support is built unconditionally.  However, in order to
 *   use this binary format, this function must be called during system
 *   initialization in order to register the ELF binary format.
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

  bvdbg("Registering ELF\n");

  ret = register_binfmt(&g_elfbinfmt);
  if (ret != 0)
    {
      bdbg("Failed to register binfmt: %d\n", ret);
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

