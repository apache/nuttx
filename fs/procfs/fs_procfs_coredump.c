/****************************************************************************
 * fs/procfs/fs_procfs_coredump.c
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

#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/coredump.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>

#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COREDUMP_LINELEN 128

#define COREDUMP_MAXARGS 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct coredump_file_s
{
  struct procfs_file_s base;
  struct coredump_config_s config;
  char line[COREDUMP_LINELEN];
};

struct coredump_mode_map_s
{
  FAR const char *name;
  enum coredump_mode_e mode;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int coredump_procfs_open(FAR struct file *filep,
                                FAR const char *relpath, int oflags,
                                mode_t mode);
static int coredump_procfs_close(FAR struct file *filep);
static ssize_t coredump_procfs_read(FAR struct file *filep, FAR char *buffer,
                                    size_t buflen);
static ssize_t coredump_procfs_write(FAR struct file *filep,
                                     FAR const char *buffer,
                                     size_t buflen);
static int coredump_procfs_dup(FAR const struct file *oldp,
                               FAR struct file *newp);
static int coredump_procfs_stat(FAR const char *relpath,
                                FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct procfs_operations g_coredump_operations =
{
  coredump_procfs_open,  /* open */
  coredump_procfs_close, /* close */
  coredump_procfs_read,  /* read */
  coredump_procfs_write, /* write */
  NULL,                  /* poll */
  coredump_procfs_dup,   /* dup */
  NULL,                  /* opendir */
  NULL,                  /* closedir */
  NULL,                  /* readdir */
  NULL,                  /* rewinddir */
  coredump_procfs_stat   /* stat */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coredump_mode_map_s g_coredump_modes[] =
{
  { "off",     COREDUMP_MODE_DISABLED     },
  { "current", COREDUMP_MODE_CURRENT_TASK },
  { "all",     COREDUMP_MODE_ALL_TASKS    },
};

static FAR const char *g_coredump_procfs_help[] =
{
  "commands:\n",
  "  off                    disable coredump\n",
  "  current                dump current task only\n",
  "  all                    dump all tasks\n",
  "  range clear            clear memory ranges\n",
  "  range add {start,end,flags}\n",
  "  range set {start,end,flags}[,{start,end,flags}...]\n",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR const char *coredump_procfs_mode_name(enum coredump_mode_e mode)
{
  int i;

  for (i = 0; i < nitems(g_coredump_modes); i++)
    {
      if (g_coredump_modes[i].mode == mode)
        {
          return g_coredump_modes[i].name;
        }
    }

  return "unknown";
}

static int coredump_parse_args(FAR char *cmd, FAR char **argv,
                               int maxargs)
{
  FAR char *saveptr;
  FAR char *arg;
  int argc = 0;

  for (arg = strtok_r(cmd, " \f\n\r\t\v", &saveptr);
       arg != NULL;
       arg = strtok_r(NULL, " \f\n\r\t\v", &saveptr))
    {
      if (argc >= maxargs)
        {
          return argc;
        }

      argv[argc++] = arg;
    }

  return argc;
}

static FAR char *coredump_next_arg(FAR char *arg)
{
  arg += strlen(arg) + 1;
  while (*arg == ' ' || *arg == '\f' || *arg == '\n' || *arg == '\r' ||
         *arg == '\t' || *arg == '\v')
    {
      arg++;
    }

  return *arg == '\0' ? NULL : arg;
}

static int coredump_parse_ulong(FAR const char *str,
                                FAR unsigned long *value)
{
  FAR char *endptr;

  errno = 0;
  *value = strtoul(str, &endptr, 0);
  if (errno != 0 || endptr == str || *endptr != '\0')
    {
      return -EINVAL;
    }

  return OK;
}

static int coredump_parse_region_list(FAR char *str,
                                      FAR struct memory_region_s *regions,
                                      int maxregions,
                                      FAR int *nregions)
{
  FAR char *saveptr;
  FAR char *arg;
  int argc = 0;
  int ret;

  if (strchr(str, '{') == NULL || strchr(str, '}') == NULL)
    {
      return -EINVAL;
    }

  *nregions = 0;

  for (arg = strtok_r(str, "{}, \f\n\r\t\v", &saveptr);
       arg != NULL;
       arg = strtok_r(NULL, "{}, \f\n\r\t\v", &saveptr))
    {
      FAR struct memory_region_s *region;
      unsigned long value;

      if (*nregions >= maxregions)
        {
          return -E2BIG;
        }

      ret = coredump_parse_ulong(arg, &value);
      if (ret < 0)
        {
          return ret;
        }

      region = &regions[*nregions];
      switch (argc % 3)
        {
          case 0:
            region->start = (uintptr_t)value;
            break;

          case 1:
            region->end = (uintptr_t)value;
            break;

          default:
            if (value > UINT32_MAX)
              {
                return -EINVAL;
              }

            region->flags = (uint32_t)value;
            if (region->start == 0 && region->end == 0 &&
                region->flags == 0)
              {
                return OK;
              }

            if (region->start >= region->end)
              {
                return -EINVAL;
              }

            (*nregions)++;
            break;
        }

      argc++;
    }

  return (argc > 0 && (argc % 3) == 0) ? OK : -EINVAL;
}

static int coredump_procfs_set_ranges(FAR char *arg)
{
  struct coredump_config_s config;
  int nregions;
  int ret;

  memset(&config, 0, sizeof(config));

  ret = coredump_parse_region_list(arg, config.regions,
                                   CONFIG_COREDUMP_MEMORY_REGION_MAX,
                                   &nregions);
  if (ret < 0)
    {
      return ret;
    }

  return coredump_set_memory_regions(config.regions, nregions);
}

static int coredump_procfs_add_range(FAR char *arg)
{
  struct memory_region_s region;
  int nregions;
  int ret;

  ret = coredump_parse_region_list(arg, &region, 1, &nregions);
  if (ret < 0)
    {
      return ret;
    }

  if (nregions != 1)
    {
      return -EINVAL;
    }

  return coredump_add_memory_region((FAR const void *)region.start,
                                    region.end - region.start,
                                    region.flags);
}

static int coredump_procfs_command(FAR char *cmd)
{
  FAR char *argv[COREDUMP_MAXARGS];
  int argc;

  argc = coredump_parse_args(cmd, argv, nitems(argv));
  if (argc < 0)
    {
      return argc;
    }

  if (argc == 0)
    {
      return -EINVAL;
    }

  if (argc == 1)
    {
      int i;

      for (i = 0; i < nitems(g_coredump_modes); i++)
        {
          if (strcmp(argv[0], g_coredump_modes[i].name) == 0)
            {
              return coredump_set_mode(g_coredump_modes[i].mode);
            }
        }
    }

  if (argc >= 2 && strcmp(argv[0], "range") == 0)
    {
      if (strcmp(argv[1], "clear") == 0)
        {
          if (coredump_next_arg(argv[1]) != NULL)
            {
              return -EINVAL;
            }

          coredump_clear_memory_regions();
          return OK;
        }

      if (strcmp(argv[1], "add") == 0)
        {
          FAR char *arg = coredump_next_arg(argv[1]);

          return arg == NULL ? -EINVAL : coredump_procfs_add_range(arg);
        }

      if (strcmp(argv[1], "set") == 0)
        {
          FAR char *arg = coredump_next_arg(argv[1]);

          return arg == NULL ? -EINVAL : coredump_procfs_set_ranges(arg);
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: coredump_procfs_open
 *
 * Description:
 *   Open the coredump procfs node and snapshot the current runtime
 *   coredump configuration for stable reads on this file instance.
 *
 * Input Parameters:
 *   filep   - Open file structure.
 *   relpath - Relative procfs path.
 *   oflags  - Open flags.
 *   mode    - Requested file mode.
 *
 * Returned Value:
 *   On success, OK; a negated errno value on failure.
 *
 ****************************************************************************/

static int coredump_procfs_open(FAR struct file *filep,
                                FAR const char *relpath, int oflags,
                                mode_t mode)
{
  FAR struct coredump_file_s *priv;

  if (strcmp(relpath, "coredump") != 0)
    {
      return -ENOENT;
    }

  priv = fs_heap_zalloc(sizeof(struct coredump_file_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  coredump_get_config(&priv->config);
  filep->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: coredump_procfs_close
 ****************************************************************************/

static int coredump_procfs_close(FAR struct file *filep)
{
  FAR struct coredump_file_s *priv = filep->f_priv;

  DEBUGASSERT(priv != NULL);
  fs_heap_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: coredump_procfs_read
 *
 * Description:
 *   Read the snapshotted coredump configuration using bounded logical lines
 *   so each read returns only copied bytes and reaches EOF stably.
 *
 * Input Parameters:
 *   filep  - Open file structure.
 *   buffer - User buffer that receives procfs data.
 *   buflen - Size of the user buffer in bytes.
 *
 * Returned Value:
 *   The number of bytes copied to the user buffer.
 *
 ****************************************************************************/

static ssize_t coredump_procfs_read(FAR struct file *filep, FAR char *buffer,
                                    size_t buflen)
{
  FAR struct coredump_file_s *priv = filep->f_priv;
  FAR const struct memory_region_s *region;
  size_t linesize;
  size_t copysize;
  size_t totalsize;
  off_t offset;
  int i;

  if (buflen == 0)
    {
      return 0;
    }

  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(priv != NULL);

  offset = filep->f_pos;
  totalsize = 0;

  linesize = procfs_snprintf(priv->line, COREDUMP_LINELEN, "mode: %s\n",
                             coredump_procfs_mode_name(priv->config.mode));
  copysize = procfs_memcpy(priv->line, linesize, buffer, buflen, &offset);
  buffer += copysize;
  buflen -= copysize;
  totalsize += copysize;

  if (buflen > 0)
    {
      linesize = procfs_snprintf(priv->line, COREDUMP_LINELEN, "range:\n");
      copysize = procfs_memcpy(priv->line, linesize, buffer, buflen,
                               &offset);
      buffer += copysize;
      buflen -= copysize;
      totalsize += copysize;
    }

  if (buflen > 0)
    {
      if (priv->config.regions[0].start >= priv->config.regions[0].end)
        {
          linesize = procfs_snprintf(priv->line, COREDUMP_LINELEN,
                                     "  none\n");
          copysize = procfs_memcpy(priv->line, linesize, buffer, buflen,
                                   &offset);
          buffer += copysize;
          buflen -= copysize;
          totalsize += copysize;
        }
      else
        {
          for (i = 0, region = priv->config.regions;
               i < CONFIG_COREDUMP_MEMORY_REGION_MAX && buflen > 0 &&
               region->start < region->end;
               i++, region++)
            {
              linesize = procfs_snprintf(priv->line, COREDUMP_LINELEN,
                                         "  {0x%" PRIxPTR ",0x%" PRIxPTR
                                         ",0x%" PRIx32 "}\n",
                                         region->start, region->end,
                                         region->flags);
              copysize = procfs_memcpy(priv->line, linesize, buffer, buflen,
                                       &offset);
              buffer += copysize;
              buflen -= copysize;
              totalsize += copysize;
            }
        }
    }

  for (i = 0; i < nitems(g_coredump_procfs_help) && buflen > 0; i++)
    {
      linesize = procfs_snprintf(priv->line, COREDUMP_LINELEN, "%s",
                                 g_coredump_procfs_help[i]);
      copysize = procfs_memcpy(priv->line, linesize, buffer, buflen,
                               &offset);
      buffer += copysize;
      buflen -= copysize;
      totalsize += copysize;
    }

  filep->f_pos += totalsize;
  return totalsize;
}

/****************************************************************************
 * Name: coredump_procfs_write
 *
 * Description:
 *   Parse a coredump runtime command written through procfs.
 *
 * Input Parameters:
 *   filep  - Open file structure.
 *   buffer - User buffer containing the command text.
 *   buflen - Size of the command text in bytes.
 *
 * Returned Value:
 *   On success, the number of consumed bytes; a negated errno value on
 *   failure.
 *
 ****************************************************************************/

static ssize_t coredump_procfs_write(FAR struct file *filep,
                                     FAR const char *buffer,
                                     size_t buflen)
{
  char line[COREDUMP_LINELEN];
  FAR const char *end = buffer + buflen;
  FAR const char *p = buffer;
  int ret;

  DEBUGASSERT(filep->f_priv != NULL && buffer != NULL && buflen > 0);

  /* A single write may carry several newline-separated commands. */

  while (p < end)
    {
      FAR const char *nl = memchr(p, '\n', end - p);
      size_t len = (nl != NULL ? nl : end) - p;

      if (len >= COREDUMP_LINELEN)
        {
          fwarn("WARNING: Coredump command too long; use range add\n");
          return -E2BIG;
        }

      if (len > 0)
        {
          memcpy(line, p, len);
          line[len] = '\0';

          ret = coredump_procfs_command(line);
          if (ret < 0)
            {
              return p - buffer > 0 ? p - buffer : ret;
            }
        }

      p = nl != NULL ? nl + 1 : end;
    }

  return buflen;
}

/****************************************************************************
 * Name: coredump_procfs_dup
 ****************************************************************************/

static int coredump_procfs_dup(FAR const struct file *oldp,
                               FAR struct file *newp)
{
  FAR struct coredump_file_s *oldpriv = oldp->f_priv;
  FAR struct coredump_file_s *newpriv;

  DEBUGASSERT(oldpriv != NULL);

  newpriv = fs_heap_zalloc(sizeof(struct coredump_file_s));
  if (newpriv == NULL)
    {
      return -ENOMEM;
    }

  memcpy(newpriv, oldpriv, sizeof(struct coredump_file_s));
  newp->f_priv = newpriv;
  return OK;
}

/****************************************************************************
 * Name: coredump_procfs_stat
 ****************************************************************************/

static int coredump_procfs_stat(FAR const char *relpath,
                                FAR struct stat *buf)
{
  if (strcmp(relpath, "coredump") != 0)
    {
      return -ENOENT;
    }

  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR;
  buf->st_size = 0;
  buf->st_blksize = 0;
  buf->st_blocks = 0;
  return OK;
}
