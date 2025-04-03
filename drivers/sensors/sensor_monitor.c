/****************************************************************************
 * drivers/sensors/sensor_monitor.c
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

#include <ctype.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <search.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sensor_monitor_buffer_s
{
  FAR char *buffer;
  size_t   totalsize;
  size_t   buflen;
  off_t    offset;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int sensor_monitor_open(FAR struct file *filep,
                               FAR const char *relpath,
                               int oflags, mode_t mode);
static int sensor_monitor_close(FAR struct file *filep);
static ssize_t sensor_monitor_read(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen);
static ssize_t sensor_monitor_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct procfs_operations g_sensor_monitor_operations =
{
  sensor_monitor_open,    /* open */
  sensor_monitor_close,   /* close */
  sensor_monitor_read,    /* read */
  sensor_monitor_write,   /* write */
};

static const struct procfs_entry_s g_sensor_monitor_entry =
{
  "sensor_monitor", &g_sensor_monitor_operations
};

static struct hsearch_data *g_sensor_monitor_table;

FAR static const char *g_sensor_monitor_header =
  "Sensor procfs - Dynamic sensor debugging tool\n"
  "\n"
  "Usage:\n"
  "  cat /proc/sensor_monitor - Show currently monitored topics\n"
  "  echo <level> <topic> > /proc/sensor_monitor - Add topic(s)\n"
  "  echo rm <topic> > /proc/sensor_monitor - Remove topic(s)\n"
  "  echo add <topic> <topic> > /proc/sensor_monitor"
  " - add/remove topics\n"
  "  echo clean > /proc/sensor_monitor      - Remove all topics\n"
  "\n"
  "Examples:\n"
  "  echo sensor_accel > /proc/sensor_monitor\n"
  "  echo \"sensor_accel sensor_compass\" > /proc/sensor_monitor\n"
  "  echo \"1 sensor_accel sensor_compass\" > /proc/sensor_monitor\n"
  "  echo \"2 sensor_accel sensor_compass\" > /proc/sensor_monitor\n"
  "  echo \"rm sensor_accel\" > /proc/sensor_monitor\n"
  "  echo \"rm sensor_accel sensor_compass\" > /proc/sensor_monitor\n"
  "  echo clean > /proc/sensor_monitor\n"
  "  echo \"add 1 sensor_a rm sensor_b\" > /proc/sensor_monitor\n"
  "\n"
  "Note:\n"
  "  If <level> is not specified, it defaults to 1.\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sensor_monitor_clean(void)
{
  hdestroy_r(g_sensor_monitor_table);
  hcreate_r(CONFIG_SENSORS_MONITOR_BUCKET, g_sensor_monitor_table);
}

/****************************************************************************
 * Name: sensor_monitor_open
 *
 * Description:
 *   Open a file in the procfs.
 *
 ****************************************************************************/

static int sensor_monitor_open(FAR struct file *filep,
                               FAR const char *relpath,
                               int oflags, mode_t mode)
{
  FAR struct procfs_file_s *procfile;

  /* Allocate a container to hold the file attributes */

  procfile = kmm_malloc(sizeof(struct procfs_file_s));
  if (procfile == NULL)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = procfile;
  return OK;
}

/****************************************************************************
 * Name: sensor_monitor_close
 *
 * Description:
 *   Close a file in the procfs.
 *
 ****************************************************************************/

static int sensor_monitor_close(FAR struct file *filep)
{
  FAR struct procfs_file_s *procfile;

  /* Recover our private data from the struct file instance */

  procfile = filep->f_priv;
  DEBUGASSERT(procfile);

  /* Release the file attributes structure */

  kmm_free(procfile);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: sensor_monitor_print
 *
 * Description:
 *   Print the sensor currently being monitored
 *
 ****************************************************************************/

static void sensor_monitor_print(FAR ENTRY *item, FAR void *args)
{
  FAR struct sensor_monitor_buffer_s *sminfo =
    (FAR struct sensor_monitor_buffer_s *)args;
  char buffer[NAME_MAX];
  size_t copysize;
  size_t linesize;

  if (sminfo->buflen > 0)
    {
      linesize = procfs_snprintf(buffer, NAME_MAX,
                                 "%-20s\t%-20d\n", item->key,
                                 (int)(intptr_t)item->data);
      copysize = procfs_memcpy(buffer, linesize, sminfo->buffer,
                               sminfo->buflen, &sminfo->offset);
      sminfo->totalsize += copysize;
      sminfo->buffer += copysize;
      sminfo->buflen -= copysize;
    }
}

/****************************************************************************
 * Name: sensor_monitor_read
 *
 * Description:
 *   Print the topic currently being monitored
 *
 ****************************************************************************/

static ssize_t sensor_monitor_read(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen)
{
  struct sensor_monitor_buffer_s sminfo;
  size_t copysize;

  sminfo.offset = filep->f_pos;
  sminfo.totalsize = 0;

  copysize = procfs_memcpy(g_sensor_monitor_header,
                           strlen(g_sensor_monitor_header),
                           buffer, buflen,
                           &sminfo.offset);
  sminfo.buffer = buffer + copysize;
  sminfo.buflen = buflen - copysize;
  sminfo.totalsize += copysize;

  hforeach_r(sensor_monitor_print, &sminfo, g_sensor_monitor_table);
  filep->f_pos += sminfo.totalsize;
  return sminfo.totalsize;
}

/****************************************************************************
 * Name: sensor_monitor_remove
 *
 * Description:
 *   remove monitored topics.
 *
 ****************************************************************************/

static int sensor_monitor_remove(FAR const char *token)
{
  ENTRY item;

  item.key = (char *)token;

  if (!hsearch_r(item, DELETE, NULL, g_sensor_monitor_table))
    {
      snerr("failed to delete topic %s\n", token);
      return -ENOENT;
    }

  return OK;
}

/****************************************************************************
 * Name: sensor_monitor_add
 *
 * Description:
 *   add monitored topics.
 *
 ****************************************************************************/

static int sensor_monitor_add(FAR const char *name, int level)
{
  FAR ENTRY *entry;
  ENTRY item;

  item.key = (FAR char *)name;
  item.data = NULL;

  if (hsearch_r(item, FIND, &entry, g_sensor_monitor_table))
    {
      entry->data = (FAR void *)(intptr_t)level;
      return OK;
    }

  item.key = strdup(name);
  item.data = (FAR void *)(intptr_t)level;

  if (!hsearch_r(item, ENTER, &entry, g_sensor_monitor_table))
    {
      kmm_free(item.key);
      snerr("failed to add topic %s\n", name);
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: sensor_monitor_write
 *
 * Description:
 *   Add/delete monitored topics.
 *
 ****************************************************************************/

static ssize_t sensor_monitor_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen)
{
  int ret;
  bool add = true;
  int level = 1;
  size_t remaining = buflen;
  char token[NAME_MAX];

  while (remaining > 0 && buffer[0] != '\n')
    {
      size_t len;
      FAR const char *end;

      end = memchr(buffer, ' ', remaining);
      if (end)
        {
          len = end - buffer;
        }
      else
        {
          len = remaining;
        }

      memcpy(token, buffer, len);
      token[len] = '\0';
      buffer += len;
      remaining -= len;

      if (remaining > 0 && *buffer != '\0')
        {
          buffer++;
          remaining--;
        }

      if (!strcmp(token, "rm"))
        {
          add = false;
        }
      else if (!strcmp(token, "add"))
        {
          add = true;
        }
      else if (!strcmp(token, "clean"))
        {
          sensor_monitor_clean();
        }
      else if (isdigit(token[0]))
        {
          level = atoi(token);
        }
      else if (token[0])
        {
          if (add)
            {
              ret = sensor_monitor_add(token, level);
              if (ret < 0)
                {
                  return ret;
                }
            }
          else
            {
              ret = sensor_monitor_remove(token);
              if (ret < 0)
                {
                  return ret;
                }
            }
        }
    }

  return buflen;
}

static void sensor_monitor_free_entry(FAR ENTRY *entry)
{
  kmm_free(entry->key);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_monitor_initialize
 *
 * Description:
 *   Register sensor procfs.
 *
 * Return Value:
 *   0 on success, or negative error code on failure.
 *
 ****************************************************************************/

int sensor_monitor_initialize(void)
{
  size_t len;

  g_sensor_monitor_table = kmm_zalloc(sizeof(struct hsearch_data));
  if (!g_sensor_monitor_table)
    {
      return -ENOMEM;
    }

  g_sensor_monitor_table->free_entry = sensor_monitor_free_entry;

  if (!hcreate_r(CONFIG_SENSORS_MONITOR_BUCKET, g_sensor_monitor_table))
    {
      kmm_free(g_sensor_monitor_table);
      return -ENOMEM;
    }

  len = strlen(CONFIG_SENSORS_MONITOR_LIST);
  sensor_monitor_write(NULL, CONFIG_SENSORS_MONITOR_LIST, len);
  return procfs_register(&g_sensor_monitor_entry);
}

/****************************************************************************
 * Name: sensor_monitor_level
 *
 * Description:
 *   get sensor monitor log level
 *
 * Return Value:
 *   syslog level
 *
 ****************************************************************************/

int sensor_monitor_level(FAR const char *name)
{
  FAR ENTRY *entry;
  ENTRY item;

  item.key = (FAR char *)name;
  item.data = NULL;

  if (!hsearch_r(item, FIND, &entry, g_sensor_monitor_table))
    {
      return LOG_EMERG;
    }

  return (int)(intptr_t)entry->data;
}
