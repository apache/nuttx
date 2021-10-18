/****************************************************************************
 * libs/libc/string/lib_strrchr.c
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

/* The strrchr() function returns a pointer to the last
 * occurrence of the character c in the string s.
 */

#undef strrchr /* See mm/README.txt */
FAR char *strrchr(FAR const char *s, int c)
{
  if (s)
    {
      const char *p = &s[strlen(s)];
      for (; p >= s; p--)
        {
          if (*p == c)
            {
              return (FAR char *)p;
            }
        }
    }

  return NULL;
}
