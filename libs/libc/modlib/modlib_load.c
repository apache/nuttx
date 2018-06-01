/****************************************************************************
 * libs/libc/modlib/modlib_load.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <elf32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_ALIGN_MASK   ((1 << CONFIG_MODLIB_ALIGN_LOG2) - 1)
#define ELF_ALIGNUP(a)   (((unsigned long)(a) + ELF_ALIGN_MASK) & ~ELF_ALIGN_MASK)
#define ELF_ALIGNDOWN(a) ((unsigned long)(a) & ~ELF_ALIGN_MASK)

#ifndef MAX
#  define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#  define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_elfsize
 *
 * Description:
 *   Calculate total memory allocation for the ELF file.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static void modlib_elfsize(struct mod_loadinfo_s *loadinfo)
{
  size_t textsize;
  size_t datasize;
  int i;

  /* Accumulate the size each section into memory that is marked SHF_ALLOC */

  textsize = 0;
  datasize = 0;

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];

      /* SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

      if ((shdr->sh_flags & SHF_ALLOC) != 0)
        {
          /* SHF_WRITE indicates that the section address space is write-
           * able
           */

          if ((shdr->sh_flags & SHF_WRITE) != 0)
            {
              datasize += ELF_ALIGNUP(shdr->sh_size);
            }
          else
            {
              textsize += ELF_ALIGNUP(shdr->sh_size);
            }
        }
    }

  /* Save the allocation size */

  loadinfo->textsize = textsize;
  loadinfo->datasize = datasize;
}

/****************************************************************************
 * Name: modlib_loadfile
 *
 * Description:
 *   Read the section data into memory. Section addresses in the shdr[] are
 *   updated to point to the corresponding position in the memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int modlib_loadfile(FAR struct mod_loadinfo_s *loadinfo)
{
  FAR uint8_t *text;
  FAR uint8_t *data;
  FAR uint8_t **pptr;
  int ret;
  int i;

  /* Read each section into memory that is marked SHF_ALLOC + SHT_NOBITS */

  binfo("Loaded sections:\n");
  text = (FAR uint8_t *)loadinfo->textalloc;
  data = (FAR uint8_t *)loadinfo->datastart;

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf32_Shdr *shdr = &loadinfo->shdr[i];

      /* SHF_ALLOC indicates that the section requires memory during
       * execution */

      if ((shdr->sh_flags & SHF_ALLOC) == 0)
        {
          continue;
        }

      /* SHF_WRITE indicates that the section address space is write-
       * able
       */

      if ((shdr->sh_flags & SHF_WRITE) != 0)
        {
          pptr = &data;
        }
      else
        {
          pptr = &text;
        }

      /* SHT_NOBITS indicates that there is no data in the file for the
       * section.
       */

      if (shdr->sh_type != SHT_NOBITS)
        {
          /* Read the section data from sh_offset to the memory region */

          ret = modlib_read(loadinfo, *pptr, shdr->sh_size, shdr->sh_offset);
          if (ret < 0)
            {
              berr("ERROR: Failed to read section %d: %d\n", i, ret);
              return ret;
            }
        }

      /* If there is no data in an allocated section, then the allocated
       * section must be cleared.
       */

      else
        {
          memset(*pptr, 0, shdr->sh_size);
        }

      /* Update sh_addr to point to copy in memory */

      binfo("%d. %08lx->%08lx\n", i,
            (unsigned long)shdr->sh_addr, (unsigned long)*pptr);

      shdr->sh_addr = (uintptr_t)*pptr;

      /* Setup the memory pointer for the next time through the loop */

      *pptr += ELF_ALIGNUP(shdr->sh_size);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_load
 *
 * Description:
 *   Loads the binary into memory, allocating memory, performing relocations
 *   and initializing the data and bss segments.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_load(FAR struct mod_loadinfo_s *loadinfo)
{
  int ret;

  binfo("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->filfd >= 0);

  /* Load section headers into memory */

  ret = modlib_loadshdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadshdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Determine total size to allocate */

  modlib_elfsize(loadinfo);

  /* Allocate (and zero) memory for the ELF file. */

  /* Allocate memory to hold the ELF image */

  loadinfo->textalloc = (uintptr_t)lib_zalloc(loadinfo->textsize + loadinfo->datasize);
  if (!loadinfo->textalloc)
    {
      berr("ERROR: Failed to allocate memory for the module\n");
      ret = -ENOMEM;
      goto errout_with_buffers;
    }

  loadinfo->datastart = loadinfo->textalloc + loadinfo->textsize;

  /* Load ELF section data into memory */

  ret = modlib_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadfile failed: %d\n", ret);
      goto errout_with_buffers;
    }

  return OK;

  /* Error exits */

errout_with_buffers:
  modlib_unload(loadinfo);
  return ret;
}
