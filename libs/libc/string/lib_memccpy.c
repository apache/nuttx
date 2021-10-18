/****************************************************************************
 * libs/libc/string/lib_memccpy.c
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
 * Name: memccpy
 *
 * Description:
 *   The memccpy() function copies bytes from memory area s2 into s1,
 *   stopping after the first occurrence of byte c (converted to an unsigned
 *   char) is copied, or after n bytes are copied, whichever comes first. If
 *   copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The memccpy() function returns a pointer to the byte after the copy of c
 *   in s1, or a null pointer if c was not found in the first n bytes of s2.
 *
 ****************************************************************************/

#undef memccpy /* See mm/README.txt */
FAR void *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n)
{
  FAR unsigned char *pout = (FAR unsigned char *)s1;
  FAR unsigned char *pin  = (FAR unsigned char *)s2;

  /* Copy at most n bytes */

  while (n-- > 0)
    {
      /* Copy one byte */

      *pout = *pin++;

      /* Did we just copy the terminating byte c? */

      if (*pout++ == (unsigned char)c)
        {
          /* Yes return a pointer to the byte after the copy of c into s1 */

          return (FAR void *)pout;
        }
    }

  /* C was not found in the first n bytes of s2 */

  return NULL;
}
