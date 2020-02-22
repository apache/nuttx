/****************************************************************************
 * binfmt/libelf/libelf_read.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/binfmt/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef ELF_DUMP_READDATA       /* Define to dump all file data read */

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_dumpreaddata
 ****************************************************************************/

#if defined(ELF_DUMP_READDATA)
static inline void elf_dumpreaddata(FAR char *buffer, int buflen)
{
  FAR uint32_t *buf32 = (FAR uint32_t *)buffer;
  int i;
  int j;

  for (i = 0; i < buflen; i += 32)
    {
      syslog(LOG_DEBUG, "%04x:", i);
      for (j = 0; j < 32; j += sizeof(uint32_t))
        {
          syslog(LOG_DEBUG, "  %08x", *buf32++);
        }

      syslog(LOG_DEBUG, "\n");
    }
}
#else
#  define elf_dumpreaddata(b,n)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'.  The data is
 *   read into 'buffer.' If 'buffer' is part of the ELF address environment,
 *   then the caller is responsible for assuring that that address
 *   environment is in place before calling this function (i.e., that
 *   elf_addrenv_select() has been called if CONFIG_ARCH_ADDRENV=y).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_read(FAR struct elf_loadinfo_s *loadinfo, FAR uint8_t *buffer,
             size_t readsize, off_t offset)
{
  ssize_t nbytes;      /* Number of bytes read */
  off_t   rpos;        /* Position returned by lseek */

  binfo("Read %ld bytes from offset %ld\n", (long)readsize, (long)offset);

  /* Loop until all of the requested data has been read. */

  while (readsize > 0)
    {
      /* Seek to the next read position */

      rpos = lseek(loadinfo->filfd, offset, SEEK_SET);
      if (rpos != offset)
        {
          int errval = get_errno();
          berr("Failed to seek to position %lu: %d\n",
               (unsigned long)offset, errval);
          return -errval;
        }

      /* Read the file data at offset into the user buffer */

      nbytes = nx_read(loadinfo->filfd, buffer, readsize);
      if (nbytes < 0)
        {
          /* EINTR just means that we received a signal */

          if (nbytes != -EINTR)
            {
              berr("Read from offset %lu failed: %d\n",
                   (unsigned long)offset, (int)nbytes);
              return nbytes;
            }
        }
      else if (nbytes == 0)
        {
          berr("Unexpected end of file\n");
          return -ENODATA;
        }
      else
        {
          readsize -= nbytes;
          buffer   += nbytes;
          offset   += nbytes;
        }
    }

  elf_dumpreaddata(buffer, readsize);
  return OK;
}
