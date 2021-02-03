/****************************************************************************
 * binfmt/libelf/libelf_ctors.c
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

#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/elf.h>

#include "libelf.h"

#ifdef CONFIG_BINFMT_CONSTRUCTORS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: elf_loadctors
 *
 * Description:
 *  Load pointers to static constructors into an in-memory array.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loadctors(FAR struct elf_loadinfo_s *loadinfo)
{
  FAR Elf_Shdr *shdr;
  size_t ctorsize;
  int ctoridx;
  int ret;
  int i;

  DEBUGASSERT(loadinfo->ctors == NULL);

  /* Allocate an I/O buffer if necessary.  This buffer is used by
   * elf_sectname() to accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      berr("elf_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  /* Find the index to the section named ".ctors."  NOTE:  On old ABI system,
   * .ctors is the name of the section containing the list of constructors;
   * On newer systems, the similar section is called .init_array.  It is
   * expected that the linker script will force the section name to be
   * ".ctors" in either case.
   */

  ctoridx = elf_findsection(loadinfo, ".ctors");
  if (ctoridx < 0)
    {
      /* This may not be a failure.  -ENOENT indicates that the file has no
       * static constructor section.
       */

      binfo("elf_findsection .ctors section failed: %d\n", ctoridx);
      return ret == -ENOENT ? OK : ret;
    }

  /* Now we can get a pointer to the .ctor section in the section header
   * table.
   */

  shdr = &loadinfo->shdr[ctoridx];

  /* Get the size of the .ctor section and the number of constructors that
   * will need to be called.
   */

  ctorsize         = shdr->sh_size;
  loadinfo->nctors = ctorsize / sizeof(binfmt_ctor_t);

  binfo("ctoridx=%d ctorsize=%d sizeof(binfmt_ctor_t)=%d nctors=%d\n",
        ctoridx, ctorsize,  sizeof(binfmt_ctor_t), loadinfo->nctors);

  /* Check if there are any constructors.  It is not an error if there
   * are none.
   */

  if (loadinfo->nctors > 0)
    {
      /* Check an assumption that we made above */

      DEBUGASSERT(shdr->sh_size == loadinfo->nctors * sizeof(binfmt_ctor_t));

      /* In the old ABI, the .ctors section is not allocated.  In that case,
       * we need to allocate memory to hold the .ctors and then copy the
       * from the file into the allocated memory.
       *
       * SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

      if ((shdr->sh_flags & SHF_ALLOC) == 0)
        {
          /* Allocate memory to hold a copy of the .ctor section */

          loadinfo->ctoralloc = (binfmt_ctor_t *)kumm_malloc(ctorsize);
          if (!loadinfo->ctoralloc)
            {
              berr("Failed to allocate memory for .ctors\n");
              return -ENOMEM;
            }

          loadinfo->ctors = (binfmt_ctor_t *)loadinfo->ctoralloc;

          /* Read the section header table into memory */

          ret = elf_read(loadinfo, (FAR uint8_t *)loadinfo->ctors, ctorsize,
                         shdr->sh_offset);
          if (ret < 0)
            {
              berr("Failed to allocate .ctors: %d\n", ret);
              return ret;
            }

          /* Fix up all of the .ctor addresses.  Since the addresses
           * do not lie in allocated memory, there will be no relocation
           * section for them.
           */

          for (i = 0; i < loadinfo->nctors; i++)
            {
              FAR uintptr_t *ptr = (uintptr_t *)
                   ((FAR void *)(&loadinfo->ctors)[i]);

              binfo("ctor %d: "
                    "%08" PRIxPTR " + %08" PRIxPTR " = %08" PRIxPTR "\n",
                    i, *ptr, (uintptr_t)loadinfo->textalloc,
                    (uintptr_t)(*ptr + loadinfo->textalloc));

              *ptr += loadinfo->textalloc;
            }
        }
      else
        {
          /* Save the address of the .ctors (actually, .init_array) where
           * it was loaded into memory.  Since the .ctors lie in allocated
           * memory, they will be relocated via the normal mechanism.
           */

          loadinfo->ctors = (binfmt_ctor_t *)shdr->sh_addr;
        }
    }

  return OK;
}

#endif /* CONFIG_BINFMT_CONSTRUCTORS */
