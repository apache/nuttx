/****************************************************************************
 * libs/libc/stdio/lib_tempnam.c
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
#include <stdlib.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tempnam
 *
 * Description:
 *   The tempnam() function generates a pathname that may be used for a
 *   temporary file.
 *
 *   The tempnam() function allows the user to control the choice of a
 *   directory. The dir argument points to the name of the directory in which
 *   the file is to be created. If dir is a null pointer or points to a
 *   string which is not a name for an appropriate directory, the path prefix
 *   defined as P_tmpdir in the <stdio.h> header will be used. If that
 *   directory is not accessible, an implementation-defined directory may be
 *   used.
 *
 *   Many applications prefer their temporary files to have certain initial
 *   letter sequences in their names. The pfx argument should be used for
 *   this. This argument may be a null pointer or point to a string of up
 *   to five bytes to be used as the beginning of the filename.
 *
 * Returned Value:
 *   Upon successful completion, tempnam() will allocate space for a string
 *   put the generated pathname in that space, and return a pointer to it.
 *   The pointer will be suitable for use in a subsequent call to free().
 *   Otherwise, it will return a null pointer and set errno to indicate the
 *   error.
 *
 *   The tempnam() function will fail if:
 *     ENOMEM - Insufficient storage space is available.
 *
 ****************************************************************************/

FAR char *tempnam(FAR const char *dir, FAR const char *pfx)
{
  FAR char *template;
  FAR char *path;

  asprintf(&template, "%s/%s-XXXXXX", dir, pfx);
  if (template)
    {
      path = mktemp(template);
      if (path != NULL)
        {
          return path;
        }

      lib_free(template);
    }
  else
    {
      set_errno(ENOMEM);
    }

  return NULL;
}
