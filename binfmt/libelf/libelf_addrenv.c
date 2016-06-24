/****************************************************************************
 * binfmt/libelf/libelf_addrenv.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 *   CONFIG_ARCH_ADDRENV=n, textalloc will be allocated using kmm_zalloc() and
 *   dataalloc will be a offset from textalloc.  If CONFIG_ARCH_ADDRENV-y, then
 *   textalloc and dataalloc will be allocated using up_addrenv_create().  In
 *   either case, there will be a unique instance of textalloc and dataalloc
 *   (and stack) for each instance of a process.
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
  FAR void *vtext;
  FAR void *vdata;
  int ret;

  /* Create an address environment for the new ELF task */

  ret = up_addrenv_create(textsize, datasize, heapsize, &loadinfo->addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_create failed: %d\n", ret);
      return ret;
    }

  /* Get the virtual address associated with the start of the address
   * environment.  This is the base address that we will need to use to
   * access the ELF image (but only if the address environment has been
   * selected.
   */

  ret = up_addrenv_vtext(&loadinfo->addrenv, &vtext);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_vtext failed: %d\n", ret);
      return ret;
    }

  ret = up_addrenv_vdata(&loadinfo->addrenv, textsize, &vdata);
  if (ret < 0)
    {
      berr("ERROR: up_adup_addrenv_vdatadrenv_vtext failed: %d\n", ret);
      return ret;
    }

  loadinfo->textalloc = (uintptr_t)vtext;
  loadinfo->dataalloc = (uintptr_t)vdata;
  return OK;
#else
  /* Allocate memory to hold the ELF image */

  loadinfo->textalloc = (uintptr_t)kumm_zalloc(textsize + datasize);
  if (!loadinfo->textalloc)
    {
      return -ENOMEM;
    }

  loadinfo->dataalloc = loadinfo->textalloc + textsize;
  return OK;
#endif
}

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
  int ret;

  /* Free the address environment */

  ret = up_addrenv_destroy(&loadinfo->addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_destroy failed: %d\n", ret);
    }
#else
  /* If there is an allocation for the ELF image, free it */

  if (loadinfo->textalloc != 0)
    {
      kumm_free((FAR void *)loadinfo->textalloc);
    }
#endif

  /* Clear out all indications of the allocated address environment */

  loadinfo->textalloc = 0;
  loadinfo->dataalloc = 0;
  loadinfo->textsize  = 0;
  loadinfo->datasize  = 0;
}
