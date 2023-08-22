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
#include <inttypes.h>
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

#ifdef ELF_DUMP_READDATA
static inline void elf_dumpreaddata(FAR char *buffer, size_t buflen)
{
  FAR uint32_t *buf32 = (FAR uint32_t *)buffer;
  size_t i;
  size_t j;

  for (i = 0; i < buflen; i += 32)
    {
      syslog(LOG_DEBUG, "%04zx:", i);
      for (j = 0; j < 32; j += sizeof(uint32_t))
        {
          syslog(LOG_DEBUG, " %08" PRIx32, *buf32++);
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
  size_t  nsize = readsize;   /* Bytes to read from the object file */
  ssize_t nbytes;             /* Number of bytes read */
  off_t   rpos;               /* Position returned by lseek */
  int     ret = OK;           /* Return value */

#ifdef CONFIG_ARCH_USE_COPY_SECTION
  FAR uint8_t *dest = buffer; /* Destination address - `buffer` */

  /* Redirect `buffer` to temporary allocated memory */

  buffer = kmm_malloc(readsize);
  if (buffer == NULL)
    {
      berr("ERROR: Failed to allocate memory\n");
      return -ENOMEM;
    }
#endif

  binfo("Read %zu bytes from offset %" PRIdOFF "\n", readsize, offset);

  /* Loop until all of the requested data has been read. */

  while (readsize > 0)
    {
      /* Seek to the next read position */

      rpos = file_seek(&loadinfo->file, offset, SEEK_SET);
      if (rpos != offset)
        {
          berr("Failed to seek to position %" PRIdOFF ": %" PRIdOFF "\n",
               offset, rpos);
          ret = rpos;
          goto errout;
        }

      /* Read the file data at offset into the user buffer */

      nbytes = file_read(&loadinfo->file,
                         buffer + nsize - readsize, readsize);
      if (nbytes < 0)
        {
          /* EINTR just means that we received a signal */

          if (nbytes != -EINTR)
            {
              berr("Read from offset %" PRIdOFF " failed: %zd\n",
                   offset, nbytes);
              ret = nbytes;
              goto errout;
            }
        }
      else if (nbytes == 0)
        {
          berr("Unexpected end of file\n");
          ret = -ENODATA;
          goto errout;
        }
      else
        {
          readsize -= nbytes;
          offset   += nbytes;
        }
    }

#ifdef CONFIG_ARCH_USE_COPY_SECTION
  /* Copy the requested data from temporary memory to destination */

  ret = up_copy_section(dest, buffer, nsize);
  if (ret < 0)
    {
      berr("ERROR: Failed to copy section at offset %"PRIdOFF"\n", offset);
      goto errout;
    }
#endif

  elf_dumpreaddata(buffer, nsize);

errout:
#ifdef CONFIG_ARCH_USE_COPY_SECTION
  /* Free the temporary memory */

  kmm_free(buffer);
#endif

  return ret;
}
