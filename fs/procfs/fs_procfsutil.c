/****************************************************************************
 * fs/procfs/fs_procfsutil.c
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
#include <stdio.h>
#include <string.h>

#include <nuttx/fs/procfs.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: procfs_memcpy
 *
 * Description:
 *   procfs/ file data may be read by the user with different user buffer
 *   sizes to receive the data.  If the amount of data to be returned is
 *   large or if the callers receive buffer is small, then multiple read
 *   operations will be required.
 *
 *   If multiple read operations are required, then each read operation will
 *   be identical accept that file position (f_pos) will be incremented with
 *   each read:  f_pos must be incremented by the read method after each
 *   read operation to provide the 'offset' for the next read.
 *
 *   procfs_memcpy() is a helper function.  Each read() method should
 *   provide data in a local data buffer ('src' and 'srclen').  This
 *   will transfer the data to the user receive buffer ('dest' and
 *   'destlen'), respecting both (1) the size of the destination buffer so
 *   that it will write beyond the user receiver and (1) the file position,
 *   'offset'.
 *
 *   This function will skip over data until the under of bytes specified
 *   by 'offset' have been skipped.  Then it will transfer data from the
 *   the procfs/ 'src' buffer into the user receive buffer.  No more than
 *   'destlen' bytes will be transferred.
 *
 * Input Parameters:
 *   src     - The address of the intermediate procfs/ buffer containing the
 *             data to be returned.
 *   srclen  - The number of bytes of data in the 'src' buffer
 *   dest    - The address of the user's receive buffer.
 *   destlen - The size (in bytes) of the user's receive buffer.
 *   offset  - On input, this is the number of bytes to skip before returning
 *             data;  If bytes were skipped, this offset will be decremented.
 *             Data will not be transferred until this offset decrements to
 *             zero.
 *
 * Returned Value:
 *   The number of bytes actually transferred into the user's receive buffer.
 *
 ****************************************************************************/

size_t procfs_memcpy(FAR const char *src, size_t srclen,
                     FAR char *dest, size_t destlen,
                     off_t *offset)
{
  size_t copysize;
  size_t lnoffset;

  /* Will this line take us past the offset? */

  lnoffset = *offset;
  if (srclen < lnoffset)
    {
      /* No... decrement the offset and return without doing anything */

      *offset -= srclen;
      return 0;
    }

  /* Handle the remaining offset */

  srclen -= lnoffset;
  src    += lnoffset;
  *offset = 0;

  /* Copy the line into the user destination buffer */

  copysize = MIN(srclen, destlen);
  memcpy(dest, src, copysize);
  return copysize;
}

/****************************************************************************
 * Name: procfs_snprintf
 *
 * Description:
 *   This function is same with snprintf, except return values.
 *   If buf has no enough space and output was truncated due to size limit,
 *   snprintf:        return formatted string len.
 *   procfs_snprintf: return string len which has written to buf.
 *
 * Input Parameters:
 *   Same with snprintf
 *
 * Returned Value:
 *   See Description.
 *
 ****************************************************************************/

int procfs_snprintf(FAR char *buf, size_t size,
                    FAR const IPTR char *format, ...)
{
  va_list ap;
  int n;
  va_start(ap, format);
  n = vsnprintf(buf, size, format, ap);
  va_end(ap);
  return n < size - 1 ? n : size - 1;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
