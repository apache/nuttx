/****************************************************************************
 * libs/libc/string/lib_strsep.c
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
