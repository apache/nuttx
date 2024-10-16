/****************************************************************************
 * libs/libc/modlib/modlib_unload.c
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

#include <stdlib.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of modlib_load().  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_unload(FAR struct mod_loadinfo_s *loadinfo)
{
  /* Free all working buffers */

  modlib_freebuffers(loadinfo);

#ifdef CONFIG_ARCH_ADDRENV
  if (loadinfo->addrenv != NULL)
    {
      modlib_addrenv_free(loadinfo);
    }
  else
#endif
  /* Release memory holding the relocated ELF image */

  /* ET_DYN has a single allocation so we only free textalloc */

  if (loadinfo->ehdr.e_type != ET_DYN)
    {
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
      int i;

      for (i = 0; loadinfo->sectalloc[i] != 0 &&
                  i < loadinfo->ehdr.e_shnum; i++)
        {
#  ifdef CONFIG_ARCH_USE_TEXT_HEAP
          if (up_textheap_heapmember((FAR void *)loadinfo->sectalloc[i]))
            {
              up_textheap_free((FAR void *)loadinfo->sectalloc[i]);
            }
          else
#  endif

#  ifdef CONFIG_ARCH_USE_DATA_HEAP
          if (up_dataheap_heapmember((FAR void *)loadinfo->sectalloc[i]))
            {
              up_dataheap_free((FAR void *)loadinfo->sectalloc[i]);
            }
          else
#  endif
            {
              lib_free((FAR void *)loadinfo->sectalloc[i]);
            }
        }

      lib_free(loadinfo->sectalloc);
#else
      if (loadinfo->textalloc != 0 && loadinfo->xipbase == 0)
        {
#  if defined(CONFIG_ARCH_USE_TEXT_HEAP)
          up_textheap_free((FAR void *)loadinfo->textalloc);
#  else
          lib_free((FAR void *)loadinfo->textalloc);
#  endif
        }

      if (loadinfo->datastart != 0)
        {
#  if defined(CONFIG_ARCH_USE_DATA_HEAP)
          up_dataheap_free((FAR void *)loadinfo->datastart);
#  else
          lib_free((FAR void *)loadinfo->datastart);
#  endif
        }
#endif
    }
  else
    {
      lib_free((FAR void *)loadinfo->textalloc);
    }

  /* Clear out all indications of the allocated address environment */

  loadinfo->textalloc = 0;
  loadinfo->datastart = 0;
  loadinfo->textsize  = 0;
  loadinfo->datasize  = 0;

  return OK;
}
