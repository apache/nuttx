/****************************************************************************
 * libs/libc/modlib/modlib_read.c
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
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef ELF_DUMP_READDATA       /* Define to dump all file data read */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_dumpreaddata
 ****************************************************************************/

#ifdef ELF_DUMP_READDATA
#  define modlib_dumpreaddata(b,n) binfodumpbuffer("modlib_read",b,n)
#else
#  define modlib_dumpreaddata(b,n)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'.  The data is
 *   read into 'buffer.'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_read(FAR struct mod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
                size_t readsize, off_t offset)
{
  size_t  nsize = readsize;
  ssize_t nbytes;      /* Number of bytes read */
  off_t   rpos;        /* Position returned by lseek */
  int     errval;

  binfo("Read %zu bytes from offset %" PRIdOFF "\n", readsize, offset);

  /* Loop until all of the requested data has been read. */

  /* Seek to the read position */

  rpos = _NX_SEEK(loadinfo->filfd, offset, SEEK_SET);
  if (rpos != offset)
    {
      errval = _NX_GETERRNO(rpos);
      berr("ERROR: Failed to seek to position %" PRIdOFF ": %d\n",
           offset, errval);
      return -errval;
    }

  while (readsize > 0)
    {
      /* Read the file data at offset into the user buffer */

      nbytes = _NX_READ(loadinfo->filfd,
                        buffer + nsize - readsize, readsize);
      if (nbytes < 0)
        {
          errval = _NX_GETERRNO(nbytes);

          /* EINTR just means that we received a signal */

          if (errval != EINTR)
            {
              berr("ERROR: Read from offset %" PRIdOFF " failed: %d\n",
                   (off_t)(offset + nsize - readsize), errval);
              return -errval;
            }
        }
      else if (nbytes == 0)
        {
          berr("ERROR: Unexpected end of file\n");
          return -ENODATA;
        }
      else
        {
          readsize -= nbytes;
        }
    }

  modlib_dumpreaddata(buffer, nsize);
  return OK;
}
