/****************************************************************************
 * libs/libc/elf/elf_uninit.c
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

#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/lib/elf.h>

#include "libc.h"
#include "elf/elf.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: libelf_uninitialize
 *
 * Description:
 *   Releases any resources committed by libelf_initialize().  This
 *   essentially undoes the actions of libelf_initialize.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libelf_uninitialize(FAR struct mod_loadinfo_s *loadinfo)
{
  /* Free all working buffers */

  libelf_freebuffers(loadinfo);

  /* Close the ELF file */

  if (loadinfo->filfd >= 0)
    {
      _NX_CLOSE(loadinfo->filfd);
      loadinfo->filfd = -1;
    }

  return OK;
}

/****************************************************************************
 * Name: libelf_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libelf_freebuffers(FAR struct mod_loadinfo_s *loadinfo)
{
  /* Release all working allocations  */

  if (loadinfo->shdr != NULL)
    {
      lib_free(loadinfo->shdr);
      loadinfo->shdr = NULL;
    }

  if (loadinfo->phdr != NULL)
    {
      lib_free(loadinfo->phdr);
      loadinfo->phdr = NULL;
    }

  if (loadinfo->iobuffer != NULL)
    {
      lib_free(loadinfo->iobuffer);
      loadinfo->iobuffer = NULL;
      loadinfo->buflen   = 0;
    }

  return OK;
}
