/****************************************************************************
 * libs/libc/string/lib_strsep.c
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecon.net>
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

#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strsep
 *
 * Description:
 *    If *strp is NULL, the strsep() function returns NULL and does
 *    nothing else.  Otherwise, this function finds the first token in the
 *    string *strp, that is delimited by one of the bytes in the string
 *    delim.  This token is terminated by overwriting the delimiter with a
 *    null byte ('\0'), and *strp is updated to point past the token.
 *    In case no delimiter was found, the token is taken to be the entire
 *    string *strp, and *strp is made NULL.
 *
 * Returned Value:
 *    The strsep() function returns a pointer to the token, that is, it
 *    returns the original value of *strp.
 *
 ****************************************************************************/

FAR char *strsep(FAR char **strp, FAR const char *delim)
{
  FAR char *sbegin = *strp;
  FAR char *end;

  if (sbegin == NULL)
    {
      return NULL;
    }

  end = strpbrk(sbegin, delim);
  if (end != NULL)
    {
      *end++ = '\0';
    }

  *strp = end;
  return sbegin;
}
