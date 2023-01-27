/****************************************************************************
 * sched/module/mod_insmod.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#ifdef CONFIG_MODULE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_dumploadinfo
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
static void mod_dumploadinfo(FAR struct mod_loadinfo_s *loadinfo)
{
  int i;

  binfo("LOAD_INFO:\n");
  binfo("  textalloc:    %08lx\n", (long)loadinfo->textalloc);
  binfo("  datastart:    %08lx\n", (long)loadinfo->datastart);
  binfo("  textsize:     %ld\n",   (long)loadinfo->textsize);
  binfo("  datasize:     %ld\n",   (long)loadinfo->datasize);
  binfo("  textalign:    %zu\n",   loadinfo->textalign);
  binfo("  dataalign:    %zu\n",   loadinfo->dataalign);
  binfo("  filelen:      %ld\n",   (long)loadinfo->filelen);
  binfo("  filfd:        %d\n",    loadinfo->filfd);
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
  binfo("  e_flags:      %08x\n",  loadinfo->ehdr.e_flags);
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
# define mod_dumploadinfo(i)
#endif

/****************************************************************************
 * Name: mod_dumpinitializer
 ****************************************************************************/

#ifdef CONFIG_MODLIB_DUMPBUFFER
static void mod_dumpinitializer(mod_initializer_t initializer,
                                FAR struct mod_loadinfo_s *loadinfo)
{
  modlib_dumpbuffer("Initializer code", (FAR const uint8_t *)initializer,
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
 *   NOTE: modlib_setsymtab() had to have been called in board-specific OS
 *   logic prior to calling this function from application logic (perhaps via
 *   boardctl(BOARDIOC_OS_SYMTAB).  Otherwise, insmod will be unable to
 *   resolve symbols in the OS module.
 *
 * Input Parameters:
 *
 *   filename - Full path to the module binary to be loaded
 *   modname  - The name that can be used to refer to the module after
 *     it has been loaded.
 *
 * Returned Value:
 *   A non-NULL module handle that can be used on subsequent calls to other
 *   module interfaces is returned on success.  If insmod() was unable to
 *   load the module insmod() will return a NULL handle and the errno
 *   variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *insmod(FAR const char *filename, FAR const char *modname)
{
  struct mod_loadinfo_s loadinfo;
  FAR struct module_s *modp;
  mod_initializer_t initializer;
  int ret;

  DEBUGASSERT(filename != NULL && modname != NULL);
  binfo("Loading file: %s\n", filename);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Check if this module is already installed */

  if (modlib_registry_find(modname) != NULL)
    {
      ret = -EEXIST;
      goto errout_with_lock;
    }

  /* Initialize the ELF library to load the program binary. */

  ret = modlib_initialize(filename, &loadinfo);
  mod_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("ERROR: Failed to initialize to load module: %d\n", ret);
      goto errout_with_loadinfo;
    }

  /* Allocate a module registry entry to hold the module data */

  modp = (FAR struct module_s *)kmm_zalloc(sizeof(struct module_s));
  if (modp == NULL)
    {
      berr("Failed to allocate struct module_s\n");
      goto errout_with_loadinfo;
    }

#ifdef HAVE_MODLIB_NAMES
  /* Save the module name in the registry entry */

  strlcpy(modp->modname, modname, sizeof(modp->modname));
#endif

  /* Load the program binary */

  ret = modlib_load(&loadinfo);
  mod_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      binfo("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_registry_entry;
    }

  /* Bind the program to the kernel symbol table */

  ret = modlib_bind(modp, &loadinfo);
  if (ret != 0)
    {
      binfo("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  /* Save the load information */

  modp->textalloc   = (FAR void *)loadinfo.textalloc;
  modp->dataalloc   = (FAR void *)loadinfo.datastart;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->textsize    = loadinfo.textsize;
  modp->datasize    = loadinfo.datasize;
#endif

  /* Get the module initializer entry point */

  initializer = (mod_initializer_t)(loadinfo.textalloc +
                                    loadinfo.ehdr.e_entry);
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->initializer = initializer;
#endif
  mod_dumpinitializer(initializer, &loadinfo);

  /* Call the module initializer */

  ret = initializer(&modp->modinfo);
  if (ret < 0)
    {
      binfo("Failed to initialize the module: %d\n", ret);
      goto errout_with_load;
    }

  /* Add the new module entry to the registry */

  modlib_registry_add(modp);

  modlib_uninitialize(&loadinfo);
  modlib_registry_unlock();
  return (FAR void *)modp;

errout_with_load:
  modlib_unload(&loadinfo);
  modlib_undepend(modp);
errout_with_registry_entry:
  kmm_free(modp);
errout_with_loadinfo:
  modlib_uninitialize(&loadinfo);
errout_with_lock:
  modlib_registry_unlock();
  set_errno(-ret);
  return NULL;
}

#endif /* CONFIG_MODULE */
