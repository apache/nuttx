/****************************************************************************
 * binfmt/libelf/libelf_sections.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
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
 * Name: elf_sectname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_sectname(FAR struct elf_loadinfo_s *loadinfo,
                               FAR const Elf_Shdr *shdr)
{
  FAR Elf_Shdr *shstr;
  off_t  offset;
  size_t bytesread = 0;
  int shstrndx;
  int ret;

  /* Get the section header table index of the entry associated with the
   * section name string table. If the file has no section name string table,
   * this member holds the value SH_UNDEF.
   */

  shstrndx = loadinfo->ehdr.e_shstrndx;
  if (shstrndx == SHN_UNDEF)
    {
      berr("No section header string table\n");
      return -EINVAL;
    }

  /* Allocate an I/O buffer if necessary.  This buffer is used by
   * elf_sectname() to accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      berr("elf_allocbuffer failed: %d\n", ret);
      return ret;
    }

  /* Get the section name string table section header */

  shstr = &loadinfo->shdr[shstrndx];

  /* Get the file offset to the string that is the name of the section. This
   * is the sum of:
   *
   *   shstr->sh_offset: The file offset to the first byte of the section
   *     header string table data.
   *   shdr->sh_name: The offset to the name of the section in the section
   *     name table
   */

  offset = shstr->sh_offset + shdr->sh_name;

  /* Loop until we get the entire section name into memory */

  for (; ; )
    {
      FAR uint8_t *buffer = &loadinfo->iobuffer[bytesread];
      size_t readlen = loadinfo->buflen - bytesread;

      /* Get the number of bytes to read */

      if (offset + readlen > loadinfo->filelen)
        {
          if (loadinfo->filelen <= offset)
            {
              berr("At end of file\n");
              return -EINVAL;
            }

          readlen = loadinfo->filelen - offset;
        }

      /* Read that number of bytes into the array */

      ret = elf_read(loadinfo, buffer, readlen, offset + bytesread);
      if (ret < 0)
        {
          berr("Failed to read section name\n");
          return ret;
        }

      bytesread += readlen;

      /* Did we read the NUL terminator? */

      if (memchr(buffer, '\0', readlen) != NULL)
        {
          /* Yes, the buffer contains a NUL terminator. */

          return OK;
        }

      /* No.. then we have to read more */

      ret = elf_reallocbuffer(loadinfo, CONFIG_ELF_BUFFERINCR);
      if (ret < 0)
        {
          berr("elf_reallocbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_loadphdrs
 *
 * Description:
 *   Loads program headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loadphdrs(FAR struct elf_loadinfo_s *loadinfo)
{
  size_t phdrsize;
  int ret;

  DEBUGASSERT(loadinfo->phdr == NULL);

  /* Verify that there are programs */

  if (loadinfo->ehdr.e_phnum < 1)
    {
      binfo("No programs(?)\n");
      return 0;
    }

  /* Get the total size of the program header table */

  phdrsize = (size_t)loadinfo->ehdr.e_phentsize *
             (size_t)loadinfo->ehdr.e_phnum;
  if (loadinfo->ehdr.e_phoff + phdrsize > loadinfo->filelen)
    {
      berr("Insufficient space in file for program header table\n");
      return -ESPIPE;
    }

  /* Allocate memory to hold a working copy of the program header table */

  loadinfo->phdr = (FAR FAR Elf_Phdr *)kmm_malloc(phdrsize);
  if (!loadinfo->phdr)
    {
      berr("Failed to allocate the program header table. Size: %ld\n",
           (long)phdrsize);
      return -ENOMEM;
    }

  /* Read the program header table into memory */

  ret = elf_read(loadinfo, (FAR uint8_t *)loadinfo->phdr, phdrsize,
                 loadinfo->ehdr.e_phoff);
  if (ret < 0)
    {
      berr("Failed to read program header table: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: elf_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loadshdrs(FAR struct elf_loadinfo_s *loadinfo)
{
  size_t shdrsize;
  int ret;

  DEBUGASSERT(loadinfo->shdr == NULL);

  /* Verify that there are sections */

  if (loadinfo->ehdr.e_shnum < 1)
    {
      berr("No sections(?)\n");
      return -EINVAL;
    }

  /* Get the total size of the section header table */

  shdrsize = (size_t)loadinfo->ehdr.e_shentsize *
             (size_t)loadinfo->ehdr.e_shnum;
  if (loadinfo->ehdr.e_shoff + shdrsize > loadinfo->filelen)
    {
      berr("Insufficient space in file for section header table\n");
      return -ESPIPE;
    }

  /* Allocate memory to hold a working copy of the sector header table */

  loadinfo->shdr = kmm_malloc(shdrsize);
  if (!loadinfo->shdr)
    {
      berr("Failed to allocate the section header table. Size: %zu\n",
           shdrsize);
      return -ENOMEM;
    }

  /* Read the section header table into memory */

  ret = elf_read(loadinfo, (FAR uint8_t *)loadinfo->shdr, shdrsize,
                 loadinfo->ehdr.e_shoff);
  if (ret < 0)
    {
      berr("Failed to read section header table: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: elf_findsection
 *
 * Description:
 *   A section by its name.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sectname - Name of the section to find
 *
 * Returned Value:
 *   On success, the index to the section is returned; A negated errno value
 *   is returned on failure.
 *
 ****************************************************************************/

int elf_findsection(FAR struct elf_loadinfo_s *loadinfo,
                    FAR const char *sectname)
{
  int i;

  /* Search through the shdr[] array in loadinfo for a section named
   * 'sectname'
   */

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      FAR const Elf_Shdr *shdr = &loadinfo->shdr[i];

      /* Get the name of this section */

      int ret = elf_sectname(loadinfo, shdr);
      if (ret < 0)
        {
          berr("elf_sectname failed: %d\n", ret);
          return ret;
        }

      /* Check if the name of this section is 'sectname' */

      binfo("%d. Comparing \"%s\" and .\"%s\"\n",
            i, loadinfo->iobuffer, sectname);

      if (strcmp((FAR const char *)loadinfo->iobuffer, sectname) == 0)
        {
          /* We found it... return the index */

          return i;
        }
    }

  /* We failed to find a section with this name. */

  return -ENOENT;
}
