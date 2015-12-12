/****************************************************************************
 * sched/module/mod_init.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <elf32.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/module.h>

#include "module.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_MODULE_DUMPBUFFER have to
 * be defined or CONFIG_MODULE_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_MODULE_DUMPBUFFER)
#  undef CONFIG_MODULE_DUMPBUFFER
#endif

#ifdef CONFIG_MODULE_DUMPBUFFER
# define mod_dumpbuffer(m,b,n) svdbgdumpbuffer(m,b,n)
#else
# define mod_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_filelen
 *
 * Description:
 *  Get the size of the ELF file
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int mod_filelen(FAR struct mod_loadinfo_s *loadinfo,
                              FAR const char *filename)
{
  struct stat buf;
  int ret;

  /* Get the file stats */

  ret = stat(filename, &buf);
  if (ret < 0)
    {
      int errval = errno;
      sdbg("Failed to stat file: %d\n", errval);
      return -errval;
    }

  /* Verify that it is a regular file */

  if (!S_ISREG(buf.st_mode))
    {
      sdbg("Not a regular file.  mode: %d\n", buf.st_mode);
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
 * Name: mod_initialize
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

int mod_initialize(FAR const char *filename,
                   FAR struct mod_loadinfo_s *loadinfo)
{
  int ret;

  svdbg("filename: %s loadinfo: %p\n", filename, loadinfo);

  /* Clear the load info structure */

  memset(loadinfo, 0, sizeof(struct mod_loadinfo_s));

  /* Get the length of the file. */

  ret = mod_filelen(loadinfo, filename);
  if (ret < 0)
    {
      sdbg("mod_filelen failed: %d\n", ret);
      return ret;
    }

  /* Open the binary file for reading (only) */

  loadinfo->filfd = open(filename, O_RDONLY);
  if (loadinfo->filfd < 0)
    {
      int errval = errno;
      sdbg("Failed to open ELF binary %s: %d\n", filename, errval);
      return -errval;
    }

  /* Read the ELF ehdr from offset 0 */

  ret = mod_read(loadinfo, (FAR uint8_t *)&loadinfo->ehdr,
                    sizeof(Elf32_Ehdr), 0);
  if (ret < 0)
    {
      sdbg("Failed to read ELF header: %d\n", ret);
      return ret;
    }

  mod_dumpbuffer("ELF header", (FAR const uint8_t *)&loadinfo->ehdr,
                    sizeof(Elf32_Ehdr));

  /* Verify the ELF header */

  ret = mod_verifyheader(&loadinfo->ehdr);
  if (ret < 0)
    {
      /* This may not be an error because we will be called to attempt loading
       * EVERY binary.  If mod_verifyheader() does not recognize the ELF header,
       * it will -ENOEXEC whcih simply informs the system that the file is not an
       * ELF file.  mod_verifyheader() will return other errors if the ELF header
       * is not correctly formed.
       */

      sdbg("Bad ELF header: %d\n", ret);
      return ret;
    }

  return OK;
}
