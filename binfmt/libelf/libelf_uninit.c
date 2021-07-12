/****************************************************************************
 * binfmt/libelf/libelf_uninit.c
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

#include <unistd.h>
#include <debug.h>
#include <errno.h>

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
 * Name: elf_uninit
 *
 * Description:
 *   Releases any resources committed by elf_init().  This essentially
 *   undoes the actions of elf_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_uninit(struct elf_loadinfo_s *loadinfo)
{
  /* Free all working buffers */

  elf_freebuffers(loadinfo);

  /* Close the ELF file */

  if (loadinfo->file.f_inode)
    {
      file_close(&loadinfo->file);
    }

  return OK;
}

/****************************************************************************
 * Name: elf_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_freebuffers(struct elf_loadinfo_s *loadinfo)
{
  /* Release all working allocations  */

  if (loadinfo->shdr)
    {
      kmm_free((FAR void *)loadinfo->shdr);
      loadinfo->shdr      = NULL;
    }

  if (loadinfo->iobuffer)
    {
      kmm_free((FAR void *)loadinfo->iobuffer);
      loadinfo->iobuffer  = NULL;
      loadinfo->buflen    = 0;
    }

  return OK;
}
