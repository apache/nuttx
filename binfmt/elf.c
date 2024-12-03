/****************************************************************************
 * binfmt/elf.c
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
#include <nuttx/kmalloc.h>

#ifdef CONFIG_ELF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT
 * have to be defined or CONFIG_MODLIB_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined(CONFIG_DEBUG_BINFMT)
#  undef CONFIG_MODLIB_DUMPBUFFER
#endif

#ifndef CONFIG_ELF_STACKSIZE
#  define CONFIG_ELF_STACKSIZE 2048
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp,
                          FAR const char *filename,
                          FAR const struct symtab_s *exports,
                          int nexports);

static int elf_unloadbinary(FAR struct binary_s *binp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_elfbinfmt =
{
  NULL,             /* next */
  elf_loadbinary,   /* load */
  elf_unloadbinary, /* unload */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  struct mod_loadinfo_s loadinfo;
  int ret;

  binfo("Loading file: %s\n", filename);

  /* Initialize the ELF library to load the program binary. */

  ret = modlib_initialize(filename, &loadinfo);
  modlib_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("Failed to initialize to load ELF program binary: %d\n", ret);
      return ret;
    }

  /* Load the program binary */

  ret = modlib_load_with_addrenv(&loadinfo);
  modlib_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      berr("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_init;
    }

  /* Bind the program to the exported symbol table */

  if (loadinfo.ehdr.e_type == ET_REL || loadinfo.gotindex >= 0)
    {
      ret = modlib_bind(&binp->mod, &loadinfo, exports, nexports);
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
          berr("Cannot bind exported symbols to a "
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
      goto errout_with_load;
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
#  ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  if (loadinfo.ehdr.e_type == ET_REL)
    {
      binp->mod.sectalloc = (FAR void *)loadinfo.sectalloc;
      binp->mod.nsect     = loadinfo.ehdr.e_shnum;
    }
#  endif

  binp->mod.textalloc = (FAR void *)loadinfo.textalloc;
  binp->mod.dataalloc = (FAR void *)loadinfo.datastart;
#  ifdef CONFIG_BINFMT_CONSTRUCTORS
  binp->mod.initarr = loadinfo.initarr;
  binp->mod.finiarr = loadinfo.finiarr;
#  endif
#endif

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Save information about constructors and destructors. */

  binp->mod.initarr = loadinfo.initarr;
  binp->mod.ninit   = loadinfo.ninit;

  binp->mod.finiarr = loadinfo.finiarr;
  binp->mod.nfini   = loadinfo.nfini;
#endif

#ifdef CONFIG_SCHED_USER_IDENTITY
  /* Save IDs and mode from file system */

  binp->uid  = loadinfo.fileuid;
  binp->gid  = loadinfo.filegid;
  binp->mode = loadinfo.filemode;
#endif

  modlib_dumpentrypt(&loadinfo);
#ifdef CONFIG_PIC
  if (loadinfo.gotindex >= 0)
    {
      FAR struct dspace_s *dspaces = kmm_zalloc(sizeof(struct dspace_s));

      if (dspaces == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_load;
        }

      dspaces->region = (FAR void *)loadinfo.shdr[loadinfo.gotindex].sh_addr;
      dspaces->crefs = 1;
      binp->picbase = (FAR void *)dspaces;
    }
#endif

  modlib_uninitialize(&loadinfo);
  return OK;

errout_with_load:
  modlib_unload(&loadinfo);
errout_with_init:
  modlib_uninitialize(&loadinfo);
  return ret;
}

/****************************************************************************
 * Name: elf_unloadbinary
 *
 * Description:
 *   Unload the ELF binary that was loaded into memory by elf_loadbinary.
 *
 ****************************************************************************/

static int elf_unloadbinary(FAR struct binary_s *binp)
{
  binfo("Unloading %p\n", binp);
  modlib_uninit(&binp->mod);

  return OK;
}

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
