/****************************************************************************
 * libs/libc/string/lib_stpncpy.c
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
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stpncpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.  strncpy() will not
 *   copy more than 'n' bytes from 'src' to 'dest' array (including the
 *   NUL terminator).
 *
 *   If the array pointed to by 'src' is a string that is shorter than 'n'
 *   bytes, NUL characters will be appended to the copy in the array
 *   pointed to by 'dest', until 'n' bytes in all are written.
 *
 *   If copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   If a NUL character is written to the destination, the stpncpy()
 *   function will return the address of the first such NUL character.
 *   Otherwise, it will return &dest[n]
 *
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_STPNCPY
FAR char *stpncpy(FAR char *dest, FAR const char *src, size_t n)
{
  FAR char *end = dest + n; /* End of dest buffer + 1 byte */
  FAR char *ret;            /* Value to be returned */

  /* Copy up n bytes, breaking out of the loop early if a NUL terminator is
   * encountered.
   */

  while ((dest != end) && (*dest = *src++) != '\0')
    {
      /* Increment the 'dest' pointer only if it does not refer to the
       * NUL terminator.
       */

      dest++;
    }

  /* Return the pointer to the NUL terminator (or to the end of the buffer
   * + 1).
   */

  ret = dest;

  /* Pad the remainder of the array pointer to 'dest' with NULs.  This
   * overwrites any previously copied NUL terminator.
   */

  while (dest != end)
    {
      *dest++ = '\0';
    }

  return ret;
}
#endif
