/****************************************************************************
 * binfmt/libnxflat/libnxflat_unload.c
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

#include <sys/mman.h>

#include <stdlib.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/nxflat.h>

#include "libnxflat.h"

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
 * Name: nxflat_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of nxflat_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_unload(FAR struct nxflat_loadinfo_s *loadinfo)
{
  /* Release the memory segments */

  /* Release the I-Space mmap'ed file */

  if (loadinfo->ispace)
    {
      file_munmap((FAR void *)loadinfo->ispace, loadinfo->isize);
      loadinfo->ispace = 0;
    }

  /* Release the D-Space address environment */

  nxflat_addrenv_free(loadinfo);
  return OK;
}
