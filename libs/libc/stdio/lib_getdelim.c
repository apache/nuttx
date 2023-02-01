/****************************************************************************
 * libs/libc/stdio/lib_getdelim.c
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

#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some environments may return CR as end-of-line, others LF, and others
 * both.  Because of the definition of the getline() function, it can handle
 * only single character line terminators.
 */

#undef HAVE_GETLINE
#if defined(CONFIG_EOL_IS_CR)
#  define HAVE_GETLINE 1
#  define EOLCH        '\r'
#elif defined(CONFIG_EOL_IS_LF)
#  define HAVE_GETLINE 1
#  define EOLCH        '\n'
#endif

#define BUFSIZE_INIT   64
#define BUFSIZE_INCR   32

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getdelim()
 *
 * Description:
 *   The getdelim() function will read from stream until it encounters a
 *   character matching the delimiter character.  The delimiter argument
 *   is an int, the value of which the application will ensure is a
 *   character representable as an unsigned char of equal value that
 *   terminates the read process. If the delimiter argument has any other
 *   value, the behavior is undefined.
 *
 *   The application will ensure that *lineptr is a valid argument that
 *   could be passed to the free() function.  If *n is non-zero, the
 *   application will ensure that *lineptr either points to an object of
 *   size at least *n bytes, or is a null pointer.
 *
 *   If *lineptr is a null pointer or if the object pointed to by
 *   *lineptr is of insufficient size, an object will be allocated as if
 *   by malloc() or the object will be reallocated as if by realloc(),
 *   respectively, such that the object is large enough to hold the
 *   characters to be written to it, including the terminating NUL, and
 *   *n will be set to the new size.  If the object was allocated, or if
 *   the reallocation operation moved the object, *lineptr will be
 *   updated to point to the new object or new location.  The characters
 *   read, including any delimiter, will be stored in the object, and a
 *   terminating NUL added when the delimiter or end-of-file is encountered.
 *
 * Returned Value:
 *  Upon successful completion, the getline() and getdelim() functions will
 *  return the number of bytes written into the buffer, including the
 *  delimiter character if one was encountered before EOF, but excluding
 *  the terminating NUL character.  If the end-of-file indicator for the
 *  stream is set, or if no characters were read and the stream is at
 *  end-of-file, the end-of-file indicator for the stream will be set and
 *  the function will return -1. If an error occurs, the error indicator for
 *  the stream will be set, and the function will return -1 and set errno to
 *  indicate the error.
 *
 ****************************************************************************/

ssize_t getdelim(FAR char **lineptr, size_t *n, int delimiter,
                 FAR FILE *stream)
{
  FAR char *dest;
  size_t bufsize;
  size_t maxcopy;
  ssize_t ncopied;
  int ch;
  int ret;

  /* Verify pointers */

  if (lineptr == NULL || n == NULL || stream == NULL)
    {
      ret = EINVAL;
      goto errout;
    }

  /* Verify the buffer size */

  bufsize = *n;
  if (bufsize == 0)
    {
      /* Pick an initial buffer size */

      bufsize = BUFSIZE_INIT;
      *n      = BUFSIZE_INIT;

      /* Free any mystery buffer. It will be reallocated below. */

      if (*lineptr != NULL)
        {
          lib_free(*lineptr);
          *lineptr = NULL;
        }
    }

  /* If *lineptr is a NULL pointer, then allocate *lineptr with size *n */

  dest = *lineptr;
  if (dest == NULL)
    {
      dest = (FAR char *)lib_malloc(bufsize);
      if (dest == NULL)
        {
          ret = ENOMEM;
          goto errout;
        }

      *lineptr = dest;
    }

  /* Transfer characters until either bufsize characters have been transfer
   * or until the delimiter is encountered.
   */

  ncopied  = 0;             /* No bytes have been transferred yet */
  maxcopy  = bufsize - 1;   /* Reserve a byte for the NUL terminator */

  do
    {
      /* If the object pointed to by *lineptr is of insufficient size, the
       * object will be reallocated such that the object is large enough to
       * hold the characters to be written to it, including the terminating
       * NUL, and *n will be set to the new size.
       */

      if (ncopied >= maxcopy)
        {
          FAR char *newbuffer;

          /* This function should fail with EOVERFLOW if ncopied exceeds
           * SSIZE_MAX.  However, I think we will have failed a memory
           * allocation or crashed long before that could occur.
           */

          bufsize  += BUFSIZE_INCR;
          newbuffer = (FAR char *)lib_realloc(*lineptr, bufsize);
          if (newbuffer == NULL)
            {
              ret = ENOMEM;
              goto errout;
            }

          *lineptr = newbuffer;
          *n       = bufsize;
          dest     = &newbuffer[ncopied];
          maxcopy  = bufsize - 1;
        }

      /* Get the next character and test for EOF */

      ch = fgetc(stream);
      if (ch == EOF)
        {
          /* errno is not set in this case */

          return -1;
        }

      /* Save the character in the user buffer and increment the number of
       * bytes transferred.
       */

      *dest++ = ch;
      ncopied++;

      /* Terminate the loop when the delimiter is found or we have hit the
       * end of the buffer (reserving a byte for the NUL terminator)
       */
    }
  while (ch != delimiter);

  /* Add a NUL terminator character (but don't report this in the number of
   * bytes transferred).
   */

  *dest = '\0';
  return ncopied;

errout:
  set_errno(ret);
  return -1;
}

/****************************************************************************
 * Name: getline()
 *
 * Description:
 *   The getline() function will be equivalent to the getdelim() function
 *   with the delimiter character equal to the <newline> character.
 *
 *   NOTE: Because of this functional definition, getline() will not work
 *   with on systems where multiple characters are used to denote the end
 *   of line such a CR-LF sequences.  In that case, the CR would be
 *   transferred into the user buffer.
 *
 ****************************************************************************/

#ifdef HAVE_GETLINE
ssize_t getline(FAR char **lineptr, size_t *n, FAR FILE *stream)
{
  return getdelim(lineptr, n, EOLCH, stream);
}
#endif
