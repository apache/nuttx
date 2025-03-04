/****************************************************************************
 * libs/libc/modlib/modlib_load.c
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

#include <sys/param.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lib/modlib.h>
#include <nuttx/fs/ioctl.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_ALIGN_MASK   ((1 << CONFIG_MODLIB_ALIGN_LOG2) - 1)
#define ELF_ALIGNUP(a)   (((unsigned long)(a) + ELF_ALIGN_MASK) & ~ELF_ALIGN_MASK)
#define ELF_ALIGNDOWN(a) ((unsigned long)(a) & ~ELF_ALIGN_MASK)

/* _ALIGN_UP: 'a' is assumed to be a power of two */

#define _ALIGN_UP(v, a)  (((v) + ((a) - 1)) & ~((a) - 1))

#ifdef CONFIG_ARCH_USE_TEXT_HEAP
#  define buffer_data_address(p) \
            (FAR uint8_t *)up_textheap_data_address((FAR void *)p)
#else
#  define buffer_data_address(p) ((FAR uint8_t *)p)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
static int modlib_section_alloc(FAR struct mod_loadinfo_s *loadinfo,
                                FAR Elf_Shdr *shdr, uint8_t idx)
{
  if (loadinfo->ehdr.e_type == ET_DYN)
    {
      return -EINVAL;
    }

  if (loadinfo->sectalloc == NULL)
    {
      /* Allocate memory info for all sections */

      loadinfo->sectalloc = lib_zalloc(sizeof(uintptr_t) *
                                       loadinfo->ehdr.e_shnum);
      if (loadinfo->sectalloc == NULL)
        {
          return -ENOMEM;
        }
    }

  modlib_sectname(loadinfo, shdr);
  if ((shdr->sh_flags & SHF_WRITE) != 0)
    {
#  ifdef CONFIG_ARCH_USE_DATA_HEAP
      loadinfo->sectalloc[idx] = (uintptr_t)
                                 up_dataheap_memalign(
                                   (FAR const char *)loadinfo->iobuffer,
                                                     shdr->sh_addralign,
                                                     shdr->sh_size);
#  else
      loadinfo->sectalloc[idx] = (uintptr_t)lib_memalign(shdr->sh_addralign,
                                                        shdr->sh_size);
#  endif

      if (loadinfo->datastart == 0)
        {
          loadinfo->datastart = loadinfo->sectalloc[idx];
        }
    }
  else if (loadinfo->xipbase != 0)
    {
      loadinfo->sectalloc[idx] = loadinfo->xipbase + shdr->sh_offset;
      if (loadinfo->textalloc == 0)
        {
          loadinfo->textalloc = loadinfo->sectalloc[idx];
        }
    }
  else
    {
#  ifdef CONFIG_ARCH_USE_TEXT_HEAP
      loadinfo->sectalloc[idx] = (uintptr_t)
                                 up_textheap_memalign(
                                   (FAR const char *)loadinfo->iobuffer,
                                                     shdr->sh_addralign,
                                                     shdr->sh_size);
#  else
      loadinfo->sectalloc[idx] = (uintptr_t)
                                  lib_memalign(shdr->sh_addralign,
                                               shdr->sh_size);
#  endif

      if (loadinfo->textalloc == 0)
        {
          loadinfo->textalloc = loadinfo->sectalloc[idx];
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: modlib_elfsize
 *
 * Description:
 *   Calculate total memory allocation for the ELF file.
 *
 ****************************************************************************/

static void modlib_elfsize(FAR struct mod_loadinfo_s *loadinfo, bool alloc)
{
  size_t textsize = 0;
  size_t datasize = 0;
  int i;

  /* Accumulate the size each section into memory that is marked SHF_ALLOC
   * if CONFIG_ARCH_USE_SEPARATED_SECTION is enabled, allocate
   * (and zero) memory for the each section.
   */

  if (loadinfo->ehdr.e_type == ET_DYN)
    {
      for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
        {
          FAR Elf_Phdr *phdr = &loadinfo->phdr[i];
          FAR void *textaddr = NULL;

          if (phdr->p_type == PT_LOAD)
            {
              if (phdr->p_flags & PF_X)
                {
                  textsize += phdr->p_memsz;
                  textaddr = (FAR void *)(uintptr_t)phdr->p_vaddr;
                }
              else
                {
                  datasize += phdr->p_memsz;
                  loadinfo->datasec = phdr->p_vaddr;
                  loadinfo->segpad  = phdr->p_vaddr -
                                      ((uintptr_t)textaddr + textsize);
                }
            }
        }
    }
  else
    {
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

              if ((shdr->sh_flags & SHF_WRITE) != 0
#ifdef CONFIG_ARCH_HAVE_TEXT_HEAP_WORD_ALIGNED_READ
                  || (shdr->sh_flags & SHF_EXECINSTR) == 0
#endif
                  )
                {
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
                  if (alloc && modlib_section_alloc(loadinfo, shdr, i) >= 0)
                    {
                      continue;
                    }
#endif

                  datasize = _ALIGN_UP(datasize, shdr->sh_addralign);
                  datasize += ELF_ALIGNUP(shdr->sh_size);
                  if (loadinfo->dataalign < shdr->sh_addralign)
                    {
                      loadinfo->dataalign = shdr->sh_addralign;
                    }
                }
              else
                {
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
                  if (alloc && modlib_section_alloc(loadinfo, shdr, i) >= 0)
                    {
                      continue;
                    }
#endif

                  textsize = _ALIGN_UP(textsize, shdr->sh_addralign);
                  textsize += ELF_ALIGNUP(shdr->sh_size);
                  if (loadinfo->textalign < shdr->sh_addralign)
                    {
                      loadinfo->textalign = shdr->sh_addralign;
                    }
                }
            }
        }
    }

  /* Save the allocation size */

  loadinfo->textsize = textsize;
  loadinfo->datasize = datasize;
}

#ifdef CONFIG_MODLIB_LOADTO_LMA
/****************************************************************************
 * Name: modlib_vma2lma
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

static int modlib_vma2lma(FAR struct mod_loadinfo_s *loadinfo,
                          FAR Elf_Shdr *shdr, FAR Elf_Addr *lma)
{
  int i;

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      FAR Elf_Phdr *phdr = &loadinfo->phdr[i];

      if (shdr->sh_addr >= phdr->p_vaddr &&
          shdr->sh_addr + shdr->sh_size <= phdr->p_vaddr + phdr->p_memsz &&
          shdr->sh_offset >= phdr->p_offset &&
          shdr->sh_offset <= phdr->p_offset + phdr->p_filesz)
        {
          *lma = phdr->p_paddr + shdr->sh_addr - phdr->p_vaddr;
          return OK;
        }
    }

  return -ENOENT;
}
#endif

/****************************************************************************
 * Name: modlib_set_emptysect_vma
 *
 * Description:
 *   Set VMA for empty and unallocated sections, some relocations might
 *   depend on this.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void modlib_set_emptysect_vma(FAR struct mod_loadinfo_s *loadinfo,
                                    int section)
{
  FAR Elf_Shdr *shdr = &loadinfo->shdr[section];

  /* Set the section as data or text, depending on SHF_WRITE */

  if ((shdr->sh_flags & SHF_WRITE) != 0
#ifdef CONFIG_ARCH_HAVE_TEXT_HEAP_WORD_ALIGNED_READ
      || (shdr->sh_flags & SHF_EXECINSTR) == 0
#endif
      )
    {
      shdr->sh_addr = loadinfo->datastart;
    }
  else
    {
      shdr->sh_addr = loadinfo->textalloc;
    }
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
  FAR uint8_t *text = (FAR uint8_t *)loadinfo->textalloc;
  FAR uint8_t *data = (FAR uint8_t *)loadinfo->datastart;
  int ret;
  int i;

  /* Read each PT_LOAD area into memory */

  binfo("Loading sections - text: %p.%zx data: %p.%zx\n",
        text, loadinfo->textsize, data, loadinfo->datasize);

  if (loadinfo->ehdr.e_type == ET_DYN)
    {
      for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
        {
          FAR Elf_Phdr *phdr = &loadinfo->phdr[i];

          if (phdr->p_type == PT_LOAD)
            {
              if (phdr->p_flags & PF_X)
                {
                  ret = modlib_read(loadinfo, buffer_data_address(text),
                                    phdr->p_filesz,
                                    phdr->p_offset);
                }
              else
                {
                  size_t bsssize = phdr->p_memsz - phdr->p_filesz;
                  ret = modlib_read(loadinfo, data, phdr->p_filesz,
                                    phdr->p_offset);
                  memset(data + phdr->p_filesz, 0, bsssize);
                }

              if (ret < 0)
                {
                  berr("ERROR: Failed to read section %d: %d\n", i, ret);
                  return ret;
                }
            }
        }
    }
  else
    {
      for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
        {
          FAR Elf_Shdr *shdr = &loadinfo->shdr[i];
          FAR uint8_t **pptr = NULL;

          /* SHF_ALLOC indicates that the section requires memory during
           * execution
           */

          if ((shdr->sh_flags & SHF_ALLOC) == 0 || shdr->sh_size == 0)
            {
              /* Set the VMA regardless */

              modlib_set_emptysect_vma(loadinfo, i);
              continue;
            }

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
          if (loadinfo->ehdr.e_type == ET_REL ||
              loadinfo->ehdr.e_type == ET_EXEC)
            {
              pptr = (FAR uint8_t **)&loadinfo->sectalloc[i];
            }
#endif

          if (pptr == NULL)
            {
              /* SHF_WRITE indicates that the section address space is
               * writeable
               */

              if ((shdr->sh_flags & SHF_WRITE) != 0
#ifdef CONFIG_ARCH_HAVE_TEXT_HEAP_WORD_ALIGNED_READ
                  || (shdr->sh_flags & SHF_EXECINSTR) == 0
#endif
                  )
                {
                  pptr = &data;
                }
              else
                {
                  pptr = &text;
                }

              if (loadinfo->xipbase == 0)
                {
                  /* If xipbase is not set, align the address
                   * xipbase is set, the address can't be aligned
                   */

                  *pptr = (FAR uint8_t *)_ALIGN_UP((uintptr_t)*pptr,
                                                   shdr->sh_addralign);
                }
            }

          if ((shdr->sh_flags & SHF_WRITE) == 0 && loadinfo->xipbase != 0)
            {
              goto skipload;
            }

          /* SHT_NOBITS indicates that there is no data in the file for the
           * section.
           */

          if (shdr->sh_type != SHT_NOBITS)
            {
#ifdef CONFIG_MODLIB_LOADTO_LMA
              ret = modlib_vma2lma(loadinfo, shdr, (FAR Elf_Addr *)pptr);
              if (ret < 0)
                {
                  berr("ERROR: Failed to convert addr %d: %d\n", i, ret);
                  return ret;
                }
#endif

              /* Read the section data from sh_offset to the memory region */

              ret = modlib_read(loadinfo, buffer_data_address(*pptr),
                                shdr->sh_size, shdr->sh_offset);
              if (ret < 0)
                {
                  berr("ERROR: Failed to read section %d: %d\n", i, ret);
                  return ret;
                }
            }

          /* If there is no data in an allocated section, then the allocated
           * section must be cleared.
           */

#ifndef CONFIG_MODLIB_LOADTO_LMA
          else if (*pptr != NULL)
            {
              memset(*pptr, 0, shdr->sh_size);
            }
#endif

skipload:

          /* Update sh_addr to point to copy in memory */

          binfo("%d. %08lx->%08lx\n", i,
                (unsigned long)shdr->sh_addr, (unsigned long)*pptr);

          /* Use offset to remember the original file address */

          shdr->sh_offset = (uintptr_t)shdr->sh_addr;
          shdr->sh_addr = (uintptr_t)*pptr;

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
          if (loadinfo->ehdr.e_type != ET_REL)
            {
              *pptr += ELF_ALIGNUP(shdr->sh_size);
            }
#else
          /* Setup the memory pointer for the next time through the loop */

          *pptr += ELF_ALIGNUP(shdr->sh_size);
#endif
        }
    }

  /* Update GOT table */

  if (loadinfo->gotindex >= 0)
    {
      FAR Elf_Shdr *gotshdr = &loadinfo->shdr[loadinfo->gotindex];
      FAR uintptr_t *got = (FAR uintptr_t *)gotshdr->sh_addr;
      FAR uintptr_t *end = got + gotshdr->sh_size / sizeof(uintptr_t);

      for (; got < end; got++)
        {
          for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
            {
              FAR Elf_Shdr *shdr = &loadinfo->shdr[i];

              if ((shdr->sh_flags & SHF_ALLOC) == 0)
                {
                  continue;
                }

              if (*got >= shdr->sh_offset &&
                  *got < shdr->sh_offset + shdr->sh_size)
                {
                  *got += shdr->sh_addr - shdr->sh_offset;
                }
            }
        }
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

  /* Load section and program headers into memory */

  ret = modlib_loadhdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadhdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  loadinfo->gotindex = modlib_findsection(loadinfo, ".got");
  if (loadinfo->gotindex >= 0)
    {
      binfo("GOT section found! index %d\n", loadinfo->gotindex);
      if (ioctl(loadinfo->filfd, FIOC_XIPBASE,
                (unsigned long)&loadinfo->xipbase) >= 0)
        {
          binfo("can use xipbase %zu\n", loadinfo->xipbase);
        }
    }

  /* Determine total size to allocate */

  modlib_elfsize(loadinfo, true);

  /* Allocate (and zero) memory for the ELF file. */

  /* Allocate memory to hold the ELF image */

  /* For Dynamic shared objects the relative positions between
   * text and data must be maintained due to references to the
   * GOT. Therefore we cannot do two different allocations.
   */

#ifndef CONFIG_MODLIB_LOADTO_LMA

  if (loadinfo->ehdr.e_type == ET_REL || loadinfo->ehdr.e_type == ET_EXEC)
    {
#  ifndef CONFIG_ARCH_USE_SEPARATED_SECTION
      if (loadinfo->xipbase != 0)
        {
          loadinfo->textalloc = loadinfo->xipbase +
                                loadinfo->shdr[1].sh_offset;
        }
      else if (loadinfo->textsize > 0)
        {
#    ifdef CONFIG_ARCH_USE_TEXT_HEAP
          loadinfo->textalloc = (uintptr_t)
                                up_textheap_memalign(loadinfo->textalign,
                                                     loadinfo->textsize +
                                                     loadinfo->segpad);
#    else
          loadinfo->textalloc = (uintptr_t)lib_memalign(loadinfo->textalign,
                                                        loadinfo->textsize +
                                                        loadinfo->segpad);
#    endif
          if (!loadinfo->textalloc)
            {
              berr("ERROR: Failed to allocate memory for the module text\n");
              ret = -ENOMEM;
              goto errout_with_buffers;
            }
        }

      if (loadinfo->datasize > 0)
        {
#    ifdef CONFIG_ARCH_USE_DATA_HEAP
          loadinfo->datastart = (uintptr_t)
                                 up_dataheap_memalign(loadinfo->dataalign,
                                                      loadinfo->datasize);
#    else
          loadinfo->datastart = (uintptr_t)lib_memalign(loadinfo->dataalign,
                                                        loadinfo->datasize);
#    endif
          if (!loadinfo->datastart)
            {
              berr("ERROR: Failed to allocate memory for the module data\n");
              ret = -ENOMEM;
              goto errout_with_buffers;
            }
        }
#  endif
    }
  else if (loadinfo->ehdr.e_type == ET_DYN)
    {
      loadinfo->textalloc = (uintptr_t)lib_memalign(loadinfo->textalign,
                                                    loadinfo->textsize +
                                                    loadinfo->datasize +
                                                    loadinfo->segpad);

      if (!loadinfo->textalloc)
        {
          berr("ERROR: Failed to allocate memory for the module\n");
          ret = -ENOMEM;
          goto errout_with_buffers;
        }

      loadinfo->datastart = loadinfo->textalloc +
                            loadinfo->textsize +
                            loadinfo->segpad;
    }

#endif /* CONFIG_MODLIB_LOADTO_LMA */

  /* Load ELF section data into memory */

  ret = modlib_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadfile failed: %d\n", ret);
      goto errout_with_buffers;
    }

#ifdef CONFIG_MODLIB_EXIDX_SECTNAME
  ret = modlib_findsection(loadinfo, CONFIG_MODLIB_EXIDX_SECTNAME);
  if (ret < 0)
    {
      binfo("modlib_findsection: Exception Index section not found: %d\n",
            ret);
    }
  else
    {
      up_init_exidx(loadinfo->shdr[ret].sh_addr,
                    loadinfo->shdr[ret].sh_size);
    }
#endif

  return OK;

  /* Error exits */

errout_with_buffers:
  modlib_unload(loadinfo);
  return ret;
}

/****************************************************************************
 * Name: modlib_load_with_addrenv
 *
 * Description:
 *   Loads the binary into memory, use the address environment to load the
 *   binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int modlib_load_with_addrenv(FAR struct mod_loadinfo_s *loadinfo)
{
  int ret;

  binfo("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->filfd >= 0);

  /* Load section and program headers into memory */

  ret = modlib_loadhdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadhdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  loadinfo->gotindex = modlib_findsection(loadinfo, ".got");
  if (loadinfo->gotindex >= 0)
    {
      binfo("GOT section found! index %d\n", loadinfo->gotindex);
      if (ioctl(loadinfo->filfd, FIOC_XIPBASE,
                (unsigned long)&loadinfo->xipbase) >= 0)
        {
          binfo("can use xipbase %zu\n", loadinfo->xipbase);
        }
    }

  /* Determine total size to allocate */

  modlib_elfsize(loadinfo, false);

  ret = modlib_addrenv_alloc(loadinfo, loadinfo->textsize,
                             loadinfo->datasize);
  if (ret < 0)
    {
      berr("ERROR: Failed to create address environment: %d\n", ret);
      goto errout_with_buffers;
    }

  /* If CONFIG_ARCH_ADDRENV=y, then the loaded ELF lies in a virtual address
   * space that may not be in place now.  elf_addrenv_select() will
   * temporarily instantiate that address space.
   */

  ret = modlib_addrenv_select(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: elf_addrenv_select() failed: %d\n", ret);
      goto errout_with_buffers;
    }

  ret = modlib_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadfile failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Restore the original address environment */

  ret = modlib_addrenv_restore(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_addrenv_restore() failed: %d\n", ret);
      goto errout_with_buffers;
    }

  return OK;

errout_with_addrenv:
  modlib_addrenv_restore(loadinfo);

errout_with_buffers:
  modlib_unload(loadinfo);
  return ret;
}
#endif
