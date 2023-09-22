/****************************************************************************
 * libs/libc/modlib/modlib_loadhdrs.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_loadhdrs
 *
 * Description:
 *   Loads program and section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_loadhdrs(FAR struct mod_loadinfo_s *loadinfo)
{
  size_t shdrsize;
  size_t phdrsize;
  int ret;

  DEBUGASSERT(loadinfo->shdr == NULL);
  DEBUGASSERT(loadinfo->phdr == NULL);

  /* Verify that there are sections */

  if (loadinfo->ehdr.e_shnum < 1)
    {
      berr("ERROR: No sections(?)\n");
      return -EINVAL;
    }

  /* Get the total size of the section header table */

  shdrsize = (size_t)loadinfo->ehdr.e_shentsize *
             (size_t)loadinfo->ehdr.e_shnum;
  if (loadinfo->ehdr.e_shoff + shdrsize > loadinfo->filelen)
    {
      berr("ERROR: Insufficient space in file for section header table\n");
      return -ESPIPE;
    }

  /* Allocate memory to hold a working copy of the sector header table */

  loadinfo->shdr = (FAR Elf_Shdr *)lib_malloc(shdrsize);
  if (!loadinfo->shdr)
    {
      berr("ERROR: Failed to allocate the section header table. Size: %zu\n",
           shdrsize);
      return -ENOMEM;
    }

  /* Read the section header table into memory */

  ret = modlib_read(loadinfo, (FAR uint8_t *)loadinfo->shdr, shdrsize,
                    loadinfo->ehdr.e_shoff);
  if (ret < 0)
    {
      berr("ERROR: Failed to read section header table: %d\n", ret);
    }

  if (loadinfo->ehdr.e_phnum > 0)
    {
      /* Get the total size of the program header table */

      phdrsize = (size_t)loadinfo->ehdr.e_phentsize *
                 (size_t)loadinfo->ehdr.e_phnum;
      if (loadinfo->ehdr.e_phoff + phdrsize > loadinfo->filelen)
        {
          berr("ERROR: Insufficent space for program header table\n");
          return -ESPIPE;
        }

      /* Allocate memory to hold a working copy of the program header table */

      loadinfo->phdr = (FAR Elf_Phdr *)lib_malloc(phdrsize);
      if (!loadinfo->phdr)
        {
          berr("ERROR: Failed to allocate the program header table."
               "Size: %zu\n", phdrsize);
          return -ENOMEM;
        }

      /* Read the program header table into memory */

      ret = modlib_read(loadinfo, (FAR uint8_t *)loadinfo->phdr, phdrsize,
                        loadinfo->ehdr.e_phoff);
      if (ret < 0)
        {
          berr("ERROR: Failed to read program header table: %d\n", ret);
        }
    }
  else
    {
      loadinfo->phdr = NULL;
    }

  return ret;
}
