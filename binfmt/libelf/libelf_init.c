/****************************************************************************
 * binfmt/libelf/libelf_init.c
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

#include <sys/stat.h>

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/binfmt/elf.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT have to
 * be defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
# define elf_dumpbuffer(m,b,n) binfodumpbuffer(m,b,n)
#else
# define elf_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_filelen
 *
 * Description:
 *  Get the size of the ELF file
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_filelen(FAR struct elf_loadinfo_s *loadinfo,
                              FAR const char *filename)
{
  struct stat buf;
  int ret;

  /* Get the file stats */

  ret = nx_stat(filename, &buf, 1);
  if (ret < 0)
    {
      berr("Failed to stat file: %d\n", ret);
      return ret;
    }

  /* Verify that it is a regular file */

  if (!S_ISREG(buf.st_mode))
    {
      berr("Not a regular file.  mode: %d\n", buf.st_mode);
      return -ENOENT;
    }

  /* TODO:  Verify that the file is readable.  Not really important because
   * we will detect this when we try to open the file read-only.
   */

  /* Return the size of the file in the loadinfo structure */

  loadinfo->filelen = buf.st_size;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_init
 *
 * Description:
 *   This function is called to configure the library to process an ELF
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_init(FAR const char *filename, FAR struct elf_loadinfo_s *loadinfo)
{
  int ret;

  binfo("filename: %s loadinfo: %p\n", filename, loadinfo);

  /* Clear the load info structure */

  memset(loadinfo, 0, sizeof(struct elf_loadinfo_s));

  /* Get the length of the file. */

  ret = elf_filelen(loadinfo, filename);
  if (ret < 0)
    {
      berr("elf_filelen failed: %d\n", ret);
      return ret;
    }

  /* Open the binary file for reading (only) */

  ret = file_open(&loadinfo->file, filename, O_RDONLY);
  if (ret < 0)
    {
      berr("Failed to open ELF binary %s: %d\n", filename, ret);
      return ret;
    }

  /* Read the ELF ehdr from offset 0 */

  ret = elf_read(loadinfo, (FAR uint8_t *)&loadinfo->ehdr,
                 sizeof(Elf_Ehdr), 0);
  if (ret < 0)
    {
      berr("Failed to read ELF header: %d\n", ret);
      return ret;
    }

  elf_dumpbuffer("ELF header", (FAR const uint8_t *)&loadinfo->ehdr,
                 sizeof(Elf_Ehdr));

  /* Verify the ELF header */

  ret = elf_verifyheader(&loadinfo->ehdr);
  if (ret < 0)
    {
      /* This may not be an error because we will be called to attempt
       * loading EVERY binary.  If elf_verifyheader() does not recognize
       * the ELF header, it will -ENOEXEC which simply informs the system
       * that the file is not an ELF file.  elf_verifyheader() will return
       * other errors if the ELF header is not correctly formed.
       */

      berr("Bad ELF header: %d\n", ret);
      return ret;
    }

  return OK;
}
