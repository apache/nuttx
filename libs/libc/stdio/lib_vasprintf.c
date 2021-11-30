/****************************************************************************
 * libs/libc/stdio/lib_vasprintf.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vasprintf
 *
 * Description:
 *   This function is similar to vsprintf, except that it dynamically
 *   allocates a string (as with malloc) to hold the output, instead of
 *   putting the output in a buffer you allocate in advance.  The ptr
 *   argument should be the address of a char * object, and a successful
 *   call to vasprintf stores a pointer to the newly allocated string at that
 *   location.
 *
 * Returned Value:
 *   The returned value is the number of characters allocated for the buffer,
 *   or less than zero if an error occurred. Usually this means that the
 *   buffer could not be allocated.
 *
 ****************************************************************************/

int vasprintf(FAR char **ptr, FAR const IPTR char *fmt, va_list ap)
{
  struct lib_outstream_s nulloutstream;
  struct lib_memoutstream_s memoutstream;

  /* On some architectures, va_list is really a pointer to a structure on
   * the stack. And the va_arg builtin will modify that instance of va_list.
   * Since vasprintf traverse the parameters in the va_list twice, the
   * va_list will be altered in this first cases and the second usage will
   * fail. This is a known issue with x86_64.
   */

#ifdef va_copy
  va_list ap2;
#endif
  FAR char *buf;
  int nbytes;

  DEBUGASSERT(ptr && fmt);

#ifdef va_copy
  va_copy(ap2, ap);
#endif

  /* First, use a nullstream to get the size of the buffer.  The number
   * of bytes returned may or may not include the null terminator.
   */

  lib_nulloutstream(&nulloutstream);
  lib_vsprintf((FAR struct lib_outstream_s *)&nulloutstream, fmt, ap);

  /* Then allocate a buffer to hold that number of characters, adding one
   * for the null terminator.
   */

  buf = (FAR char *)lib_malloc(nulloutstream.nput + 1);
  if (!buf)
    {
#ifdef va_copy
      va_end(ap2);
#endif
      return ERROR;
    }

  /* Initialize a memory stream to write into the allocated buffer.  The
   * memory stream will reserve one byte at the end of the buffer for the
   * null terminator and will not report this in the number of output bytes.
   */

  lib_memoutstream((FAR struct lib_memoutstream_s *)&memoutstream,
                   buf, nulloutstream.nput + 1);

  /* Then let lib_vsprintf do it's real thing */

#ifdef va_copy
  nbytes = lib_vsprintf((FAR struct lib_outstream_s *)&memoutstream.public,
                        fmt, ap2);
  va_end(ap2);
#else
  nbytes = lib_vsprintf((FAR struct lib_outstream_s *)&memoutstream.public,
                        fmt, ap);
#endif

  /* Return a pointer to the string to the caller.  NOTE: the memstream put()
   * method has already added the NUL terminator to the end of the string
   * (not included in the nput count).
   *
   * Hmmm.. looks like the memory would be stranded if lib_vsprintf()
   * returned an error.  Does that ever happen?
   */

  DEBUGASSERT(nbytes < 0 || nbytes == nulloutstream.nput);
  *ptr = buf;
  return nbytes;
}
