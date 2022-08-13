/****************************************************************************
 * libs/libc/libgen/lib_basename.c
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
 * Name: basename
 *
 * Description:
 *   basename() extracts the filename component from a null-terminated
 *   pathname string. In the usual case, basename() returns the component
 *   following the final '/'. Trailing '/' characters are not counted as
 *   part of the pathname.
 *
 *   If path does not contain a slash, basename() returns a copy of path.
 *   If path is the string "/", then basename() returns the string "/". If
 *   path is a NULL pointer or points to an empty string, then basename()
 *   return the string ".".
 *
 *   basename() may modify the contents of path, so copies should be passed.
 *   basename() may return pointers to statically allocated memory which may
 *   be overwritten by subsequent calls.
 *
 * Input Parameters:
 *   path The null-terminated string referring to the path to be decomposed
 *
 * Returned Value:
 *   On success the filename component of the path is returned.
 *
 ****************************************************************************/

FAR char *basename(FAR char *path)
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
   * therefore, must be just before the beginning of the filename component.
   */

  p = strrchr(path, '/');
  if (p)
    {
      return p + 1;
    }

  /* There is no '/' in the path */

  return path;
}
