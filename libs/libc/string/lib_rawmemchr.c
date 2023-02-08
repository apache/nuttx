/****************************************************************************
 * libs/libc/string/lib_rawmemchr.c
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

#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawmemchr
 *
 * Description:
 *   The rawmemchr() function is similar to memchr(), it assumes (i.e., the
 *   programmer knows for certain) that an instance of c lies somewhere in
 *   the memory area starting at the location pointed to by s,and so performs
 *   an optimized search for c (i.e., no use of a count argument to limit the
 *   range of the search). If an instance of c is not found, the results are
 *   unpredictable.The following call is a fast means of locating a string's
 *   terminating null byte.
 *
 * Returned Value:
 *   The rawmemchr() function returns a pointer to the located byte, or a
 *   null pointer if the byte does not occur in the object.
 *
 ****************************************************************************/

FAR void *rawmemchr(FAR const void *s, int c)
{
  if (c != '\0')
    {
      return memchr(s, c, SSIZE_MAX);
    }

  return (FAR char *)s + strlen(s);
}
