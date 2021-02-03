/****************************************************************************
 * binfmt/libelf/libelf_unload.c
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

#include <stdlib.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/elf.h>

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
 * Name: elf_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of elf_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_unload(struct elf_loadinfo_s *loadinfo)
{
  /* Free all working buffers */

  elf_freebuffers(loadinfo);

  /* Release memory holding the relocated ELF image */

  elf_addrenv_free(loadinfo);

  /* Release memory used to hold static constructors and destructors */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
#ifndef CONFIG_ARCH_ADDRENV
  if (loadinfo->ctoralloc != 0)
    {
      kumm_free(loadinfo->ctoralloc);
    }

  if (loadinfo->dtoralloc != 0)
    {
      kumm_free(loadinfo->dtoralloc);
    }
#endif

  loadinfo->ctoralloc = NULL;
  loadinfo->ctors     = NULL;
  loadinfo->nctors    = 0;

  loadinfo->dtoralloc = NULL;
  loadinfo->dtors     = NULL;
  loadinfo->ndtors    = 0;
#endif

  return OK;
}
