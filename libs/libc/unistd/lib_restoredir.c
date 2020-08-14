/****************************************************************************
 * libs/libc/unistd/lib_restoredir.c
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
#include <unistd.h>
#include <errno.h>

#include "libc.h"

#ifndef CONFIG_DISABLE_ENVIRON

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_restoredir
 ****************************************************************************/

int lib_restoredir(void)
{
  char *oldpwd;
  int ret = OK;

  oldpwd = getenv("OLDPWD");
  if (oldpwd)
    {
      oldpwd = strdup(oldpwd);  /* kludge needed because environment is realloc'ed */
      ret = setenv("PWD", oldpwd, TRUE);
      lib_free(oldpwd);
    }

  return ret;
}

#endif /* !CONFIG_DISABLE_ENVIRON */
