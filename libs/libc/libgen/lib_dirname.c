/****************************************************************************
 * libs/libc/libgen/lib_dirname.c
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
#include <libgen.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dirname
 *
 * Description:
 *   dirname() extracts the directory component from a null-terminated
 *   pathname string. In the usual case, dirname() returns the string up
 *   to, but not including, the final '/'. Trailing '/' characters are not
 *   counted as part of the pathname.
 *
 *   If path does not contain a slash, dirname() returns the string ".". If
 *   path is the string "/", then dirname() returns the string "/". If path
 *   is a NULL pointer or points to an empty string, then dirname() returns
 *   the string ".".
 *
 *   dirname() may modify the contents of path, so copies should be passed.
 *   dirname() may return pointers to statically allocated memory which may
 *   be overwritten by subsequent calls.
 *
 * Input Parameters:
 *   path The null-terminated string referring to the path to be decomposed
 *
 * Returned Value:
 *   On success the directory component of the path is returned.
 *
 ****************************************************************************/

FAR char *dirname(FAR char *path)
{
  FAR char *p;
  int       len;

  /* Handle some corner cases */

  if (!path || *path == '\0')
    {
      return ".";
    }

  /* Check for trailing slash characters */

  len = strlen(path);
  while (path[len - 1] == '/')
    {
      /* Remove trailing '/' UNLESS this would make a zero length string */

      if (len > 1)
        {
          path[len - 1] = '\0';
          len--;
        }
      else
        {
          return "/";
        }
    }

  /* Get the address of the last '/' which is not at the end of the path and,
   * therefore, must be the end of the directory component.
   */

  p = strrchr(path, '/');
  if (p)
    {
      do
        {
          /* Handle the case where the only '/' in the string is the at the
           * beginning of the path.
           */

          if (p == path)
            {
              return "/";
            }

          /* No, the directory component is the substring before the '/'. */

          *p-- = '\0';
        }
      while (*p == '/');

      return path;
    }

  /* There is no '/' in the path */

  return ".";
}
