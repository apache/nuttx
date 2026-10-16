/****************************************************************************
 * libs/libc/stdlib/lib_environ.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/mutex.h>

#if defined(CONFIG_BUILD_KERNEL) && !defined(__KERNEL__) && \
    !defined(CONFIG_DISABLE_ENVIRON)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_envlock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static size_t env_namelen(FAR const char *name)
{
  if (name == NULL || *name == '\0' || strchr(name, '=') != NULL)
    {
      return 0;
    }

  return strlen(name);
}

static FAR char **env_find(FAR char **envp, FAR const char *name,
                           size_t len)
{
  for (; envp != NULL && *envp != NULL; envp++)
    {
      if (strncmp(*envp, name, len) == 0 && (*envp)[len] == '=')
        {
          return envp;
        }
    }

  return NULL;
}

static int env_put(FAR char *var, size_t len, bool overwrite)
{
  FAR char **envp = get_environ_ptr();
  FAR char **found = env_find(envp, var, len);
  size_t n = 0;

  if (found != NULL)
    {
      if (overwrite)
        {
          free(*found);
          *found = var;
        }
      else
        {
          free(var);
        }

      return OK;
    }

  while (envp != NULL && envp[n] != NULL)
    {
      n++;
    }

  envp = realloc(envp, (n + 2) * sizeof(*envp));
  if (envp == NULL)
    {
      free(var);
      return -ENOMEM;
    }

  envp[n]     = var;
  envp[n + 1] = NULL;
  set_environ_ptr(envp);
  return OK;
}

static int env_ret(int ret)
{
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *getenv(FAR const char *name)
{
  size_t len = env_namelen(name);
  FAR char **found;

  if (len == 0)
    {
      return NULL;
    }

  nxmutex_lock(&g_envlock);
  found = env_find(get_environ_ptr(), name, len);
  nxmutex_unlock(&g_envlock);

  return found != NULL ? *found + len + 1 : NULL;
}

int setenv(FAR const char *name, FAR const char *value, int overwrite)
{
  size_t len = env_namelen(name);
  FAR char *var;
  int ret;

  if (len == 0)
    {
      return env_ret(-EINVAL);
    }

  if (value == NULL)
    {
      value = "";
    }

  var = malloc(len + strlen(value) + 2);
  if (var == NULL)
    {
      return env_ret(-ENOMEM);
    }

  memcpy(var, name, len);
  var[len] = '=';
  strcpy(var + len + 1, value);

  nxmutex_lock(&g_envlock);
  ret = env_put(var, len, overwrite != 0);
  nxmutex_unlock(&g_envlock);

  return env_ret(ret);
}

int putenv(FAR const char *string)
{
  FAR const char *eq = string != NULL ? strchr(string, '=') : NULL;
  FAR char *var;
  int ret;

  if (eq == NULL || eq == string)
    {
      return env_ret(-EINVAL);
    }

  var = strdup(string);
  if (var == NULL)
    {
      return env_ret(-ENOMEM);
    }

  nxmutex_lock(&g_envlock);
  ret = env_put(var, eq - string, true);
  nxmutex_unlock(&g_envlock);

  return env_ret(ret);
}

int unsetenv(FAR const char *name)
{
  size_t len = env_namelen(name);
  FAR char **found;

  if (len == 0)
    {
      return env_ret(-EINVAL);
    }

  nxmutex_lock(&g_envlock);
  found = env_find(get_environ_ptr(), name, len);
  if (found != NULL)
    {
      free(*found);
      do
        {
          found[0] = found[1];
        }
      while (*found++ != NULL);
    }

  nxmutex_unlock(&g_envlock);
  return OK;
}

int clearenv(void)
{
  FAR char **envp;
  FAR char **var;

  nxmutex_lock(&g_envlock);
  envp = get_environ_ptr();
  for (var = envp; var != NULL && *var != NULL; var++)
    {
      free(*var);
    }

  free(envp);
  set_environ_ptr(NULL);
  nxmutex_unlock(&g_envlock);
  return OK;
}

#endif
