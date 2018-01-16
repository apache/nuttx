/****************************************************************************
 * fs/procfs/fs_procfsutil.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <string.h>

#include <nuttx/fs/procfs.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) ((a > b) ? a : b)
#endif

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
 *   will transfer the data to the user receive buffer ('dest' and 'destlen'),
 *   respecting both (1) the size of the destination buffer so that it will
 *   write beyond the user receiver and (1) the file position, 'offset'.
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

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
