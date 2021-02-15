/****************************************************************************
 * binfmt/libelf/libelf_read.c
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

      rpos = file_seek(&loadinfo->file, offset, SEEK_SET);
      if (rpos != offset)
        {
          berr("Failed to seek to position %lu: %d\n",
               (unsigned long)offset, (int)rpos);
          return rpos;
        }

      /* Read the file data at offset into the user buffer */

      nbytes = file_read(&loadinfo->file, buffer, readsize);
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
