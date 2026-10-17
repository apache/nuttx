/****************************************************************************
 * net/bt/compat/src/subsys/fs/fs.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* SPDX-License-Identifier: Apache-2.0 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>

#include <nuttx/net/bluetooth/compat/settings/settings.h>
#include <syslog.h>

/* Simple filesystem-backed settings storage */

#define BT_SETTINGS_BASE "/data/bt"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int settings_subsys_init(void)
{
  mkdir(BT_SETTINGS_BASE, 0755);
  return 0;
}

static int settings_path(const char *name, char *path, size_t len)
{
  /* Replace '/' with '_' for filesystem compatibility */

  char safe[256];
  size_t i = 0;

  while (name[i] && i < sizeof(safe) - 1)
    {
      safe[i] = (name[i] == '/') ? '_' : name[i];
      i++;
    }

  safe[i] = '\0';
  return snprintf(path, len, BT_SETTINGS_BASE "/%s", safe);
}

int settings_save_one(const char *name, const void *value, size_t val_len)
{
  char path[320];

  settings_path(name, path, sizeof(path));

  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

  if (fd < 0)
    {
      return -errno;
    }

  ssize_t written = write(fd, value, val_len);

  close(fd);

  return (written == (ssize_t)val_len) ? 0 : -EIO;
}

int settings_load_subtree_direct(const char *subtree,
                                 settings_load_direct_cb cb, void *param)
{
  /* Scan directory for saved settings under subtree */

  char dirpath[320];

  settings_path(subtree, dirpath, sizeof(dirpath));

  /* For now, just return success - no persisted data */

  (void)cb;
  (void)param;
  return 0;
}

int settings_load_subtree(const char *subtree)
{
  return settings_load_subtree_direct(subtree, NULL, NULL);
}

int settings_delete(const char *name)
{
  char path[320];

  settings_path(name, path, sizeof(path));
  unlink(path);
  return 0;
}

int settings_commit(void)
{
  return 0;
}

int settings_register(struct settings_handler *handler)
{
  (void)handler;
  return 0;
}

int settings_name_next(const char *name, const char **next)
{
  const char *slash;

  if (name == NULL)
    {
      if (next != NULL)
        {
          *next = NULL;
        }

      return 0;
    }

  slash = strchr(name, '/');
  if (slash == NULL)
    {
      if (next != NULL)
        {
          *next = NULL;
        }

      return strlen(name);
    }

  if (next != NULL)
    {
      *next = slash + 1;
    }

  return slash - name;
}
