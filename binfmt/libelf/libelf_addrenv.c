/****************************************************************************
 * binfmt/libelf/libelf_addrenv.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <sys/mman.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_TEXT_WRE (PROT_READ | PROT_WRITE | PROT_EXEC)
#define ELF_TEXT_WRD (PROT_READ | PROT_EXEC)

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_addrenv_alloc
 *
 * Description:
 *   Allocate memory for the ELF image (textalloc and dataalloc). If
 *   CONFIG_ARCH_ADDRENV=n, textalloc will be allocated using kmm_zalloc()
 *   and dataalloc will be a offset from textalloc.  If
 *   CONFIG_ARCH_ADDRENV=y, then textalloc and dataalloc will be allocated
 *   using up_addrenv_create().  In either case, there will be a unique
 *   instance of textalloc and dataalloc (and stack) for each instance of a
 *   process.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   textsize - The size (in bytes) of the .text address environment needed
 *     for the ELF image (read/execute).
 *   datasize - The size (in bytes) of the .bss/.data address environment
 *     needed for the ELF image (read/write).
 *   heapsize - The initial size (in bytes) of the heap address environment
 *     needed by the task.  This region may be read/write only.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int elf_addrenv_alloc(FAR struct elf_loadinfo_s *loadinfo, size_t textsize,
                      size_t datasize, size_t heapsize)
{
#ifdef CONFIG_ARCH_ADDRENV
  FAR struct arch_addrenv_s *addrenv;
  FAR void *vtext;
  FAR void *vdata;
  int ret;

  /* Create an address environment for the new ELF task */

  loadinfo->addrenv = addrenv_allocate();
  if (!loadinfo->addrenv)
    {
      return -ENOMEM;
    }

  /* Start creating the address environment sections */

  addrenv = &loadinfo->addrenv->addrenv;

  ret = up_addrenv_create(textsize, datasize, heapsize, addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_create failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Get the virtual address associated with the start of the address
   * environment.  This is the base address that we will need to use to
   * access the ELF image (but only if the address environment has been
   * selected.
   */

  ret = up_addrenv_vtext(addrenv, &vtext);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_vtext failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  ret = up_addrenv_vdata(addrenv, textsize, &vdata);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_vdata failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  loadinfo->textalloc = (uintptr_t)vtext;
  loadinfo->dataalloc = (uintptr_t)vdata;

  return OK;

errout_with_addrenv:
  addrenv_drop(loadinfo->addrenv, false);
  return ret;
#else
  /* Allocate memory to hold the ELF image */

#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
  loadinfo->textalloc = (uintptr_t)
                         up_textheap_memalign(loadinfo->textalign,
                                              textsize);
#else
  loadinfo->textalloc = (uintptr_t)kumm_memalign(loadinfo->textalign,
                                                 textsize);
#endif

  if (!loadinfo->textalloc)
    {
      return -ENOMEM;
    }

  if (loadinfo->datasize > 0)
    {
      loadinfo->dataalloc = (uintptr_t)kumm_memalign(loadinfo->dataalign,
                                                     datasize);
      if (!loadinfo->dataalloc)
        {
          return -ENOMEM;
        }
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: elf_addrenv_select
 *
 * Description:
 *   Temporarily select the task's address environment.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int elf_addrenv_select(FAR struct elf_loadinfo_s *loadinfo)
{
  int ret;

  /* Instantiate the new address environment */

  ret = addrenv_select(loadinfo->addrenv, &loadinfo->oldenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_select failed: %d\n", ret);
      return ret;
    }

  /* Allow write access to .text */

  ret = up_addrenv_mprot(&loadinfo->addrenv->addrenv, loadinfo->textalloc,
                         loadinfo->textsize, ELF_TEXT_WRE);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_text_enable_write failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: elf_addrenv_restore
 *
 * Description:
 *   Restore the address environment before elf_addrenv_select() was called..
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int elf_addrenv_restore(FAR struct elf_loadinfo_s *loadinfo)
{
  int ret;

  /* Remove write access to .text */

  ret = up_addrenv_mprot(&loadinfo->addrenv->addrenv, loadinfo->textalloc,
                         loadinfo->textsize, ELF_TEXT_WRD);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_text_disable_write failed: %d\n", ret);
      return ret;
    }

  /* Restore the old address environment */

  ret = addrenv_restore(loadinfo->oldenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_restore failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: elf_addrenv_free
 *
 * Description:
 *   Release the address environment previously created by
 *   elf_addrenv_alloc().  This function  is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *   After the module has been started, the address environment will
 *   automatically be freed when the module exits.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void elf_addrenv_free(FAR struct elf_loadinfo_s *loadinfo)
{
#ifdef CONFIG_ARCH_ADDRENV

  /* Free the address environment */

  addrenv_drop(loadinfo->addrenv, false);
#else

  if (loadinfo->textalloc != 0)
    {
#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
      up_textheap_free((FAR void *)loadinfo->textalloc);
#else
      kumm_free((FAR void *)loadinfo->textalloc);
#endif
    }

  if (loadinfo->dataalloc != 0)
    {
      kumm_free((FAR void *)loadinfo->dataalloc);
    }

#endif

  /* Clear out all indications of the allocated address environment */

  loadinfo->textalloc = 0;
  loadinfo->dataalloc = 0;
  loadinfo->textsize  = 0;
  loadinfo->datasize  = 0;
}
