/****************************************************************************
 * binfmt/libelf/libelf_load.c
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

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <elf32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/binfmt/elf.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_ALIGN_MASK   ((1 << CONFIG_ELF_ALIGN_LOG2) - 1)
#define ELF_ALIGNUP(a)   (((unsigned long)(a) + ELF_ALIGN_MASK) & ~ELF_ALIGN_MASK)
#define ELF_ALIGNDOWN(a) ((unsigned long)(a) & ~ELF_ALIGN_MASK)

#ifndef MAX
#  define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#  define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_elfsize
 *
 * Description:
 *   Calculate total memory allocation for the ELF file.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static void elf_elfsize(struct elf_loadinfo_s *loadinfo)
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
 * Name: elf_loadfile
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

static inline int elf_loadfile(FAR struct elf_loadinfo_s *loadinfo)
{
  FAR uint8_t *text;
  FAR uint8_t *data;
  FAR uint8_t **pptr;
  int ret;
  int i;

  /* Read each section into memory that is marked SHF_ALLOC + SHT_NOBITS */

  bvdbg("Loaded sections:\n");
  text = (FAR uint8_t*)loadinfo->textalloc;
  data = (FAR uint8_t*)loadinfo->dataalloc;

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

          ret = elf_read(loadinfo, *pptr, shdr->sh_size, shdr->sh_offset);
          if (ret < 0)
            {
              bdbg("ERROR: Failed to read section %d: %d\n", i, ret);
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

      bvdbg("%d. %08lx->%08lx\n", i,
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
 * Name: elf_load
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

int elf_load(FAR struct elf_loadinfo_s *loadinfo)
{
  size_t heapsize;
#ifdef CONFIG_UCLIBCXX_EXCEPTION
  int exidx;
#endif
  int ret;

  bvdbg("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->filfd >= 0);

  /* Load section headers into memory */

  ret = elf_loadshdrs(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_loadshdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Determine total size to allocate */

  elf_elfsize(loadinfo);

  /* Determine the heapsize to allocate.  heapsize is ignored if there is
   * no address environment because the heap is a shared resource in that
   * case.  If there is no dynamic stack then heapsize must at least as big
   * as the fixed stack size since the stack will be allocated from the heap
   * in that case.
   */

#if !defined(CONFIG_ARCH_ADDRENV)
  heapsize = 0;
#elif defined(CONFIG_ARCH_STACK_DYNAMIC)
  heapsize = ARCH_HEAP_SIZE;
#else
  heapsize = MIN(ARCH_HEAP_SIZE, CONFIG_ELF_STACKSIZE);
#endif

  /* Allocate (and zero) memory for the ELF file. */

  ret = elf_addrenv_alloc(loadinfo, loadinfo->textsize, loadinfo->datasize, heapsize);
  if (ret < 0)
    {
      bdbg("ERROR: elf_addrenv_alloc() failed: %d\n", ret);
      goto errout_with_buffers;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* If CONFIG_ARCH_ADDRENV=y, then the loaded ELF lies in a virtual address
   * space that may not be in place now.  elf_addrenv_select() will
   * temporarily instantiate that address space.
   */

  ret = elf_addrenv_select(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_addrenv_select() failed: %d\n", ret);
      goto errout_with_buffers;
    }
#endif

  /* Load ELF section data into memory */

  ret = elf_loadfile(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_loadfile failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Load static constructors and destructors. */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  ret = elf_loadctors(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_loadctors failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  ret = elf_loaddtors(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_loaddtors failed: %d\n", ret);
      goto errout_with_addrenv;
    }
#endif

#ifdef CONFIG_UCLIBCXX_EXCEPTION
  exidx = elf_findsection(loadinfo, CONFIG_ELF_EXIDX_SECTNAME);
  if (exidx < 0)
    {
      bvdbg("elf_findsection: Exception Index section not found: %d\n", exidx);
    }
  else
    {
      up_init_exidx(loadinfo->shdr[exidx].sh_addr, loadinfo->shdr[exidx].sh_size);
    }
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Restore the original address environment */

  ret = elf_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      bdbg("ERROR: elf_addrenv_restore() failed: %d\n", ret);
      goto errout_with_buffers;
    }
#endif

  return OK;

  /* Error exits */

errout_with_addrenv:
#ifdef CONFIG_ARCH_ADDRENV
  (void)elf_addrenv_restore(loadinfo);
#endif

errout_with_buffers:
  elf_unload(loadinfo);
  return ret;
}

