/****************************************************************************
 * libs/libc/modlib/modlib_insert.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <sys/param.h>

#include <nuttx/lib/lib.h>
#include <nuttx/lib/modlib.h>

#include "modlib.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_dumploadinfo
 *
 * Description:
 *  Dump the load information to debug output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
void modlib_dumploadinfo(FAR struct mod_loadinfo_s *loadinfo)
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
#  ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
          if (loadinfo->ehdr.e_type == ET_REL)
            {
              binfo("  sh_alloc:     %08jx\n",
                    (uintmax_t)loadinfo->sectalloc[i]);
            }
#  endif

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

/****************************************************************************
 * Name: modlib_dumpmodule
 ****************************************************************************/

void modlib_dumpmodule(FAR struct module_s *modp)
{
  binfo("Module:\n");
  binfo("  modname:      %s\n", modp->modname);
  binfo("  textalloc:    %08lx\n", (long)modp->textalloc);
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  binfo("  dataalloc:    %08lx\n", (long)modp->dataalloc);
  binfo("  textsize:     %ld\n",   (long)modp->textsize);
#endif

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  binfo("  sectalloc:    %p\n", modp->sectalloc);
  binfo("  nsect:          %ld\n", (long)modp->nsect);
  for (int i = 0; i < modp->nsect; i++)
    {
      binfo("    sectalloc[%d]:    %p\n", i, modp->sectalloc[i]);
    }

#endif

#if CONFIG_MODLIB_MAXDEPEND > 0
  binfo("  dependents:   %d\n",    modp->dependents);
  for (int i = 0; i < modp->dependents; i++)
    {
      binfo("%d    %s\n", i, modp->dependencies[i]->modname);
      modlib_dumpmodule(modp->dependencies[i]);
    }
#endif

  binfo("  finiarr:      %08lx\n", (long)modp->finiarr);
  binfo("  nfini:        %d\n",    modp->nfini);
}

#endif
/****************************************************************************
 * Name: elf_dumpentrypt
 ****************************************************************************/

#ifdef CONFIG_MODLIB_DUMPBUFFER
void modlib_dumpentrypt(FAR struct mod_loadinfo_s *loadinfo)
{
  FAR const uint8_t *entry;
#ifdef CONFIG_ARCH_ADDRENV
  int ret;

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
          return;
        }
    }
#endif

  if (loadinfo->ehdr.e_type == ET_REL)
    {
      entry = (FAR const uint8_t *)
        ((uintptr_t)loadinfo->textalloc + loadinfo->ehdr.e_entry);
    }
  else if (loadinfo->ehdr.e_type == ET_EXEC)
    {
      entry = (FAR const uint8_t *)loadinfo->ehdr.e_entry;
    }
  else
    {
      entry = (FAR const uint8_t *)loadinfo->textalloc;
    }

  modlib_dumpbuffer("Entry code", entry,
                    MIN(loadinfo->textsize - loadinfo->ehdr.e_entry, 512));

#ifdef CONFIG_ARCH_ADDRENV
  /* Restore the original address environment */

  if (loadinfo->addrenv != NULL)
    {
      ret = modlib_addrenv_restore(loadinfo);
      if (ret < 0)
        {
          berr("ERROR: modlib_addrenv_restore() failed: %d\n", ret);
        }
    }
#endif
}
#endif

/****************************************************************************
 * Name: modlib_loadsymtab
 *
 * Description:
 *   Load the symbol table into memory.
 *
 ****************************************************************************/

static int modlib_loadsymtab(FAR struct module_s *modp,
                             FAR struct mod_loadinfo_s *loadinfo)
{
  FAR Elf_Shdr *symhdr = &loadinfo->shdr[loadinfo->symtabidx];
  FAR Elf_Sym *sym = lib_malloc(symhdr->sh_size);
  int ret;
  int i;

  if (sym == NULL)
    {
      return -ENOMEM;
    }

  ret = modlib_read(loadinfo, (FAR uint8_t *)sym, symhdr->sh_size,
                    symhdr->sh_offset);

  if (ret < 0)
    {
      berr("Failed to read symbol table\n");
      lib_free(sym);
      return ret;
    }

  for (i = 0; i < symhdr->sh_size / sizeof(Elf_Sym); i++)
    {
      if (sym[i].st_shndx != SHN_UNDEF &&
          sym[i].st_shndx < loadinfo->ehdr.e_shnum)
        {
          FAR Elf_Shdr *s = &loadinfo->shdr[sym[i].st_shndx];

          sym[i].st_value = sym[i].st_value + s->sh_addr;
        }
    }

  ret = modlib_insertsymtab(modp, loadinfo, symhdr, sym);
  lib_free(sym);
  if (ret != 0)
    {
      binfo("Failed to export symbols program binary: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: modlib_insert
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
 *   module interfaces is returned on success.  If modlib_insert() was
 *   unable to load the module modlib_insert() will return a NULL handle
 *   and the errno variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *modlib_insert(FAR const char *filename, FAR const char *modname)
{
  FAR const struct symtab_s *exports;
  struct mod_loadinfo_s loadinfo;
  FAR struct module_s *modp;
  FAR void (**array)(void);
  int nexports;
  int ret;
  int i;

  DEBUGASSERT(filename != NULL && modname != NULL);
  binfo("Loading file: %s\n", filename);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Check if this module is already installed */

#ifdef HAVE_MODLIB_NAMES
  if (modlib_registry_find(modname) != NULL)
    {
      modlib_registry_unlock();
      set_errno(EEXIST);
      return NULL;
    }
#endif

  /* Initialize the ELF library to load the program binary. */

  ret = modlib_initialize(filename, &loadinfo);
  modlib_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("ERROR: Failed to initialize to load module: %d\n", ret);
      goto errout_with_loadinfo;
    }

  /* Allocate a module registry entry to hold the module data */

  modp = lib_zalloc(sizeof(struct module_s));
  if (modp == NULL)
    {
      berr("Failed to allocate struct module_s\n");
      ret = -ENOMEM;
      goto errout_with_loadinfo;
    }

#ifdef HAVE_MODLIB_NAMES
  /* Save the module name in the registry entry */

  strlcpy(modp->modname, modname, sizeof(modp->modname));
#endif

  /* Load the program binary */

  ret = modlib_load(&loadinfo);
  modlib_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      binfo("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_registry_entry;
    }

  /* Get the symbol table */

  modlib_getsymtab(&exports, &nexports);

  /* Bind the program to the kernel symbol table */

  ret = modlib_bind(modp, &loadinfo, exports, nexports);
  if (ret != 0)
    {
      binfo("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  ret = modlib_loadsymtab(modp, &loadinfo);
  if (ret != 0)
    {
      binfo("Failed to load symbol table: %d\n", ret);
      goto errout_with_load;
    }

  /* Save the load information */

  modp->textalloc = (FAR void *)loadinfo.textalloc;
  modp->dataalloc = (FAR void *)loadinfo.datastart;
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  modp->sectalloc = (FAR void **)loadinfo.sectalloc;
  modp->nsect = loadinfo.ehdr.e_shnum;
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->textsize  = loadinfo.textsize;
  modp->datasize  = loadinfo.datasize;
#endif

  /* Call the module initializer */

  switch (loadinfo.ehdr.e_type)
    {
      case ET_REL :
      case ET_DYN :

          /* Process any preinit_array entries */

          array = (FAR void (**)(void))loadinfo.preiarr;
          for (i = 0; i < loadinfo.nprei; i++)
            {
              array[i]();
            }

          /* Process any init_array entries */

          array = (FAR void (**)(void))loadinfo.initarr;
          for (i = 0; i < loadinfo.ninit; i++)
            {
              array[i]();
            }

          modp->initarr = loadinfo.initarr;
          modp->ninit = loadinfo.ninit;
          modp->finiarr = loadinfo.finiarr;
          modp->nfini = loadinfo.nfini;
          break;
    }

  /* Add the new module entry to the registry */

  modlib_registry_add(modp);

  modlib_uninitialize(&loadinfo);
  modlib_registry_unlock();
  return modp;

errout_with_load:
  modlib_unload(&loadinfo);
#if CONFIG_MODLIB_MAXDEPEND > 0
  modlib_undepend(modp);
#endif
errout_with_registry_entry:
  lib_free(modp);
errout_with_loadinfo:
  modlib_uninitialize(&loadinfo);
  modlib_registry_unlock();
  set_errno(-ret);
  return NULL;
}
