/****************************************************************************
 * sched/environ/env_putenv.c
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <stdlib.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: putenv
 *
 * Description:
 *   The putenv() function adds or changes the value of environment
 *   variables.
 *   The argument string is of the form name=value. If name does not already
 *   exist in  the  environment, then string is added to the environment. If
 *   name does exist, then the value of name in the environment is changed to
 *   value.
 *
 * Input Parameters:
 *   name=value string describing the environment setting to add/modify
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int putenv(FAR const char *string)
{
  char *pname;
  char *pequal;
  int ret = OK;

  /* Verify that a string was passed */

  if (!string)
    {
      ret = EINVAL;
      goto errout;
    }

  /* Parse the name=value string */

  pname = strdup(string);
  if (!pname)
    {
      ret = ENOMEM;
      goto errout;
    }

  pequal = strchr(pname, '=');
  if (pequal)
    {
      /* Then let setenv do all of the work */

      *pequal = '\0';
      ret = setenv(pname, pequal + 1, TRUE);
    }

  kmm_free(pname);
  return ret;

errout:
  set_errno(ret);
  return ERROR;
}

#endif /* CONFIG_DISABLE_ENVIRON */
