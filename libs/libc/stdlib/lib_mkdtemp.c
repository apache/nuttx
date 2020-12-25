/****************************************************************************
 * libs/libc/stdlib/lib_mkdtemp.c
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
#include <nuttx/compiler.h>

#include <unistd.h>
#include <stdlib.h>

#include <sys/stat.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkdtemp
 *
 * Description:
 *  The mkdtemp() function shall create a directory with a unique name
 *  derived from template. The application shall ensure that the string
 *  provided in template is a pathname ending with at least six trailing
 *  'X' characters. The mkdtemp() function shall modify the contents of
 *  template by replacing six or more 'X' characters at the end of the
 *  pathname with the same number of characters from the portable filename
 *  character set. The characters shall be chosen such that the resulting
 *  pathname does not duplicate the name of an existing file at the time
 *  of the call to mkdtemp(). The mkdtemp() function shall use the
 *  resulting pathname to create the new directory as if by a call to:
 *
 * Input Parameters:
 *   template - The base directory name that will be modified to produce
 *     the unique name. This must be a full path beginning with /tmp.
 *     This function will modify only the first XXXXXX characters within
 *     that full path.
 *
 * Returned Value:
 *   Upon successful completion, the mkdtemp() function shall return the
 *   value of template. Otherwise, it shall return a null pointer and
 *   shall set errno to indicate the error.
 *
 ****************************************************************************/

FAR char *mkdtemp(FAR char *path_template)
{
  FAR char *path = mktemp(path_template);

  if (path)
    {
      if (mkdir(path, S_IRWXU) < 0)
        {
          path = NULL;
        }
    }

  return path;
}
