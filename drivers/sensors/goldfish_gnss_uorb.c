/****************************************************************************
 * drivers/sensors/goldfish_gnss_uorb.c
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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/goldfish_gnss.h>
#include <nuttx/sensors/gnss.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct goldfish_gnss_s
{
  struct gnss_lowerhalf_s gnss;
  struct file pipe;
  volatile bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_gnss_activate(FAR struct gnss_lowerhalf_s *lower,
                                  FAR struct file *filep, bool enabled);
static int goldfish_gnss_thread(int argc, FAR char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gnss_ops_s g_goldfish_gnss_ops =
{
  .activate = goldfish_gnss_activate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int goldfish_gnss_write_pipe(FAR struct file *pipe,
                                           FAR const void *buffer,
                                           size_t size)
{
  FAR const char *p = (FAR const char *)buffer;

  while (size > 0)
    {
      int n = file_write(pipe, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

static inline int goldfish_gnss_open_pipe(FAR struct file *filep,
                                          FAR const char *ns,
                                          FAR const char *pipe_name,
                                          int flags)
{
  char buf[256];
  int buf_len;
  int ret;

  ret = file_open(filep, "/dev/goldfish_pipe", flags);
  if (ret < 0)
    {
      snerr("Could not open '%s': %s",
            "/dev/goldfish_pipe", strerror(-ret));
      return ret;
    }

  if (ns)
    {
      buf_len = snprintf(buf, sizeof(buf), "pipe:%s:%s", ns, pipe_name);
    }
  else
    {
      buf_len = snprintf(buf, sizeof(buf), "pipe:%s", pipe_name);
    }

  ret = goldfish_gnss_write_pipe(filep, buf, buf_len + 1);
  if (ret < 0)
    {
      snerr("Could not connect to the '%s' service: %s",
            buf, strerror(-ret));
      file_close(filep);
      return ret;
    }

  return OK;
}

static int goldfish_gnss_activate(FAR struct gnss_lowerhalf_s *gnss,
                                  FAR struct file *filep,
                                  bool enabled)
{
  FAR struct goldfish_gnss_s *priv =
    container_of(gnss, struct goldfish_gnss_s, gnss);
  priv->running = enabled;

  return OK;
}

static int goldfish_gnss_thread(int argc, FAR char** argv)
{
  FAR struct goldfish_gnss_s *priv = (FAR struct goldfish_gnss_s *)
                                ((uintptr_t)strtoul(argv[1], NULL, 16));
  ssize_t len;
  char buf[256];

  while (true)
    {
      len = file_read(&priv->pipe, buf, sizeof(buf));
      if (priv->running && len > 0)
        {
          priv->gnss.push_data(priv->gnss.priv, buf, len, true);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_gnss_init
 *
 * Description:
 *   Goldfish GNSS driver entrypoint.
 *
 * Input Parameters:
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int goldfish_gnss_init(int devno, uint32_t batch_number)
{
  FAR struct goldfish_gnss_s *gnss;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Alloc memory for sensor */

  gnss = kmm_zalloc(sizeof(struct goldfish_gnss_s));
  if (!gnss)
    {
      return -ENOMEM;
    }

  ret = goldfish_gnss_open_pipe(&gnss->pipe, "qemud", "gps",
                                O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      kmm_free(gnss);
      return ret;
    }

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", gnss);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("goldfish_gnss_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       goldfish_gnss_thread, argv);
  if (ret < 0)
    {
      file_close(&gnss->pipe);
      kmm_free(gnss);
      return ret;
    }

  /*  Register sensor */

  gnss->gnss.ops = &g_goldfish_gnss_ops;

  return gnss_register(&gnss->gnss, devno, batch_number);
}
