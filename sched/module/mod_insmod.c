/****************************************************************************
 * sched/module/mod_insmod.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/module.h>

#include "module/module.h"

#ifdef CONFIG_MODULE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_BINFMT have to be
 * defined or CONFIG_MODULE_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_MODULE_DUMPBUFFER
#endif

#ifdef CONFIG_MODULE_DUMPBUFFER
# define mod_dumpbuffer(m,b,n) svdbgdumpbuffer(m,b,n)
#else
# define mod_dumpbuffer(m,b,n)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_dumploadinfo
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void mod_dumploadinfo(FAR struct mod_loadinfo_s *loadinfo)
{
  int i;

  sdbg("LOAD_INFO:\n");
  sdbg("  textalloc:    %08lx\n", (long)loadinfo->textalloc);
  sdbg("  datastart:    %08lx\n", (long)loadinfo->datastart);
  sdbg("  textsize:     %ld\n",   (long)loadinfo->textsize);
  sdbg("  datasize:     %ld\n",   (long)loadinfo->datasize);
  sdbg("  filelen:      %ld\n",   (long)loadinfo->filelen);
  sdbg("  filfd:        %d\n",    loadinfo->filfd);
  sdbg("  symtabidx:    %d\n",    loadinfo->symtabidx);
  sdbg("  strtabidx:    %d\n",    loadinfo->strtabidx);

  sdbg("ELF Header:\n");
  sdbg("  e_ident:      %02x %02x %02x %02x\n",
    loadinfo->ehdr.e_ident[0], loadinfo->ehdr.e_ident[1],
    loadinfo->ehdr.e_ident[2], loadinfo->ehdr.e_ident[3]);
  sdbg("  e_type:       %04x\n",  loadinfo->ehdr.e_type);
  sdbg("  e_machine:    %04x\n",  loadinfo->ehdr.e_machine);
  sdbg("  e_version:    %08x\n",  loadinfo->ehdr.e_version);
  sdbg("  e_entry:      %08lx\n", (long)loadinfo->ehdr.e_entry);
  sdbg("  e_phoff:      %d\n",    loadinfo->ehdr.e_phoff);
  sdbg("  e_shoff:      %d\n",    loadinfo->ehdr.e_shoff);
  sdbg("  e_flags:      %08x\n" , loadinfo->ehdr.e_flags);
  sdbg("  e_ehsize:     %d\n",    loadinfo->ehdr.e_ehsize);
  sdbg("  e_phentsize:  %d\n",    loadinfo->ehdr.e_phentsize);
  sdbg("  e_phnum:      %d\n",    loadinfo->ehdr.e_phnum);
  sdbg("  e_shentsize:  %d\n",    loadinfo->ehdr.e_shentsize);
  sdbg("  e_shnum:      %d\n",    loadinfo->ehdr.e_shnum);
  sdbg("  e_shstrndx:   %d\n",    loadinfo->ehdr.e_shstrndx);

  if (loadinfo->shdr && loadinfo->ehdr.e_shnum > 0)
    {
      for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
        {
          FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];
          sdbg("Sections %d:\n", i);
          sdbg("  sh_name:      %08x\n", shdr->sh_name);
          sdbg("  sh_type:      %08x\n", shdr->sh_type);
          sdbg("  sh_flags:     %08x\n", shdr->sh_flags);
          sdbg("  sh_addr:      %08x\n", shdr->sh_addr);
          sdbg("  sh_offset:    %d\n",   shdr->sh_offset);
          sdbg("  sh_size:      %d\n",   shdr->sh_size);
          sdbg("  sh_link:      %d\n",   shdr->sh_link);
          sdbg("  sh_info:      %d\n",   shdr->sh_info);
          sdbg("  sh_addralign: %d\n",   shdr->sh_addralign);
          sdbg("  sh_entsize:   %d\n",   shdr->sh_entsize);
        }
    }
}
#else
# define mod_dumploadinfo(i)
#endif

/****************************************************************************
 * Name: mod_dumpinitializer
 ****************************************************************************/

#ifdef CONFIG_MODULE_DUMPBUFFER
static void mod_dumpinitializer(mod_initializer_t initializer,
                                FAR struct mod_loadinfo_s *loadinfo)
{
  mod_dumpbuffer("Initializer code", (FAR const uint8_t *)initializer,
                 MIN(loadinfo->textsize - loadinfo->ehdr.e_entry, 512));
}
#else
# define mod_dumpinitializer(b,l)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: insmod
 *
 * Description:
 *   Verify that the file is an ELF module binary and, if so, load the
 *   module into kernel memory and initialize it for use.
 *
 *   NOTE: mod_setsymtab had to have been called in board-specific OS logic
 *   prior to calling this function from application logic (perhaps via
 *   boardctl(BOARDIOC_OS_SYMTAB).  Otherwise, insmod will be unable to
 *   resolve symbols in the OS module.
 *
 * Input Parameters:
 *
 *   filename   - Full path to the module binary to be loaded
 *   modulename - The name that can be used to refer to the module after
 *     it has been loaded.
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int insmod(FAR const char *filename, FAR const char *modulename)
{
  struct mod_loadinfo_s loadinfo;
  FAR struct module_s *modp;
  mod_initializer_t initializer;
  int ret;

  DEBUGASSERT(filename != NULL && modulename != NULL);
  svdbg("Loading file: %s\n", filename);

  /* Get exclusive access to the module registry */

  mod_registry_lock();

  /* Check if this module is already installed */

  if (mod_registry_find(modulename) != NULL)
    {
      mod_registry_unlock();
      ret = -EEXIST;
      goto errout_with_lock;
    }

  /* Initialize the ELF library to load the program binary. */

  ret = mod_initialize(filename, &loadinfo);
  mod_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      sdbg("ERROR: Failed to initialize to load module: %d\n", ret);
      goto errout_with_lock;
    }

  /* Allocate a module registry entry to hold the module data */

  modp = (FAR struct module_s *)kmm_zalloc(sizeof(struct module_s));
  if (ret != 0)
    {
      sdbg("Failed to initialize for load of ELF program: %d\n", ret);
      goto errout_with_loadinfo;
    }

  /* Save the module name in the registry entry */

  strncpy(modp->modulename, modulename, MODULENAME_MAX);

  /* Load the program binary */

  ret = mod_load(&loadinfo);
  mod_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      sdbg("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_registry_entry;
    }

  /* Bind the program to the kernel symbol table */

  ret = mod_bind(&loadinfo);
  if (ret != 0)
    {
      sdbg("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  /* Return the load information */

  modp->alloc       = (FAR void *)loadinfo.textalloc;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->textsize    = loadinfo.textsize;
  modp->datasize    = loadinfo.datasize;
#endif

  /* Get the module initializer entry point */

  initializer = (mod_initializer_t)(loadinfo.textalloc + loadinfo.ehdr.e_entry);
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->initializer = initializer;
#endif
  mod_dumpinitializer(initializer, &loadinfo);

  /* Call the module initializer */

  ret = initializer(&modp->uninitializer, &modp->arg);
  if (ret < 0)
    {
      sdbg("Failed to initialize the module: %d\n", ret);
      goto errout_with_load;
    }

  /* Add the new module entry to the registry */

  mod_registry_add(modp);

  mod_uninitialize(&loadinfo);
  mod_registry_unlock();
  return OK;

errout_with_load:
  mod_unload(&loadinfo);
errout_with_registry_entry:
  kmm_free(modp);
errout_with_loadinfo:
  mod_uninitialize(&loadinfo);
errout_with_lock:
  mod_registry_unlock();
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_MODULE */
