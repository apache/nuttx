/****************************************************************************
 * binfmt/libelf/libelf_load.c
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

#include <sys/param.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/elf.h>
#include <nuttx/binfmt/elf.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_ALIGN_MASK   ((1 << CONFIG_ELF_ALIGN_LOG2) - 1)
#define ELF_ALIGNUP(a)   (((unsigned long)(a) + ELF_ALIGN_MASK) & ~ELF_ALIGN_MASK)
#define ELF_ALIGNDOWN(a) ((unsigned long)(a) & ~ELF_ALIGN_MASK)

/* _ALIGN_UP: 'a' is assumed to be a power of two */

#define _ALIGN_UP(v, a)  (((v) + ((a) - 1)) & ~((a) - 1))

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

static void elf_elfsize(FAR struct elf_loadinfo_s *loadinfo)
{
  size_t textsize = 0;
  size_t datasize = 0;
  int i;

  /* Accumulate the size each section into memory that is marked SHF_ALLOC */

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf_Shdr *shdr = &loadinfo->shdr[i];

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
              datasize = _ALIGN_UP(datasize, shdr->sh_addralign);
              datasize += ELF_ALIGNUP(shdr->sh_size);
              if (loadinfo->dataalign < shdr->sh_addralign)
                {
                  loadinfo->dataalign = shdr->sh_addralign;
                }
            }
          else
            {
              textsize = _ALIGN_UP(textsize, shdr->sh_addralign);
              textsize += ELF_ALIGNUP(shdr->sh_size);
              if (loadinfo->textalign < shdr->sh_addralign)
                {
                  loadinfo->textalign = shdr->sh_addralign;
                }
            }
        }
    }

  /* Save the allocation size */

  loadinfo->textsize = textsize;
  loadinfo->datasize = datasize;
}

#ifdef CONFIG_ELF_LOADTO_LMA
/****************************************************************************
 * Name: elf_vma2lma
 *
 * Description:
 *   Convert section`s VMA to LMA according to PhysAddr(p_paddr) of
 *   Program Header.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int elf_vma2lma(FAR struct elf_loadinfo_s *loadinfo,
                       FAR Elf_Shdr *shdr, FAR Elf_Addr *lma)
{
  int i;

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      FAR Elf_Phdr *phdr = &loadinfo->phdr[i];

      if (shdr->sh_addr >= phdr->p_vaddr &&
          shdr->sh_addr < phdr->p_vaddr + phdr->p_memsz)
        {
          *lma = phdr->p_paddr + shdr->sh_addr - phdr->p_vaddr;
          return 0;
        }
    }

  return -ENOENT;
}
#endif

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
  FAR uint8_t *text = (FAR uint8_t *)loadinfo->textalloc;
  FAR uint8_t *data = (FAR uint8_t *)loadinfo->dataalloc;
  FAR uint8_t **pptr;
  int ret;
  int i;

  /* Read each section into memory that is marked SHF_ALLOC + SHT_NOBITS */

  binfo("Loaded sections:\n");

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR Elf_Shdr *shdr = &loadinfo->shdr[i];

      /* SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

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

      if (*pptr == NULL)
        {
          if (shdr->sh_type != SHT_NOBITS)
            {
              Elf_Addr addr = shdr->sh_addr;

#ifdef CONFIG_ELF_LOADTO_LMA
              ret = elf_vma2lma(loadinfo, shdr, &addr);
              if (ret < 0)
                {
                  berr("ERROR: Failed to convert addr %d: %d\n", i, ret);
                  return ret;
                }
#endif

              /* Read the section data from sh_offset to specified region */

              ret = elf_read(loadinfo, (FAR uint8_t *)addr,
                             shdr->sh_size, shdr->sh_offset);
              if (ret < 0)
                {
                  berr("ERROR: Failed to read section %d: %d\n", i, ret);
                  return ret;
                }
            }

#ifndef CONFIG_ELF_LOADTO_LMA
          /* If there is no data in an allocated section, then the
           * allocated section must be cleared.
           */

          else
            {
              memset((FAR uint8_t *)shdr->sh_addr, 0, shdr->sh_size);
            }
#endif

          continue;
        }

      *pptr = (FAR uint8_t *)_ALIGN_UP((uintptr_t)*pptr, shdr->sh_addralign);

      /* SHT_NOBITS indicates that there is no data in the file for the
       * section.
       */

      if (shdr->sh_type != SHT_NOBITS)
        {
          /* Read the section data from sh_offset to the memory region */

          ret = elf_read(loadinfo, *pptr, shdr->sh_size, shdr->sh_offset);
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
  /* Determine the heapsize to allocate.  heapsize is ignored if there is
   * no address environment because the heap is a shared resource in that
   * case.  If there is no dynamic stack then heapsize must at least as big
   * as the fixed stack size since the stack will be allocated from the heap
   * in that case.
   */

#if !defined(CONFIG_ARCH_ADDRENV)
  size_t heapsize = 0;
#elif defined(CONFIG_ARCH_STACK_DYNAMIC)
  size_t heapsize = ARCH_HEAP_SIZE;
#else
  size_t heapsize = MAX(ARCH_HEAP_SIZE, CONFIG_ELF_STACKSIZE);
#endif
#ifdef CONFIG_ELF_EXIDX_SECTNAME
  int exidx;
#endif
  int ret;

  binfo("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->file.f_inode);

  /* Load program headers into memory */

  ret = elf_loadphdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_loadphdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Load section headers into memory */

  ret = elf_loadshdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_loadshdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Determine total size to allocate */

  elf_elfsize(loadinfo);

  /* Allocate (and zero) memory for the ELF file. */

  ret = elf_addrenv_alloc(loadinfo, loadinfo->textsize, loadinfo->datasize,
                          heapsize);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_alloc() failed: %d\n", ret);
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
      berr("ERROR: elf_addrenv_select() failed: %d\n", ret);
      goto errout_with_buffers;
    }
#endif

  /* Load ELF section data into memory */

  ret = elf_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_loadfile failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Load static constructors and destructors. */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  ret = elf_loadctors(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_loadctors failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  ret = elf_loaddtors(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_loaddtors failed: %d\n", ret);
      goto errout_with_addrenv;
    }
#endif

#ifdef CONFIG_ELF_EXIDX_SECTNAME
  exidx = elf_findsection(loadinfo, CONFIG_ELF_EXIDX_SECTNAME);
  if (exidx < 0)
    {
      binfo("elf_findsection: Exception Index section not found: %d\n",
            exidx);
    }
  else
    {
      up_init_exidx(loadinfo->shdr[exidx].sh_addr,
                    loadinfo->shdr[exidx].sh_size);
    }
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Restore the original address environment */

  ret = elf_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_restore() failed: %d\n", ret);
      goto errout_with_buffers;
    }
#endif

  return OK;

  /* Error exits */

errout_with_addrenv:
#ifdef CONFIG_ARCH_ADDRENV
  elf_addrenv_restore(loadinfo);
#endif

errout_with_buffers:
  elf_unload(loadinfo);
  return ret;
}
