/****************************************************************************
 * drivers/sensors/goldfish_gps_uorb.c
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
#include <nuttx/sensors/goldfish_gps.h>
#include <nuttx/sensors/gps.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct goldfish_gps_s
{
  struct gps_lowerhalf_s gps;
  struct file pipe;
  volatile bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_gps_activate(FAR struct gps_lowerhalf_s *lower,
                                 FAR struct file *filep, bool enabled);
static int goldfish_gps_thread(int argc, FAR char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gps_ops_s g_goldfish_gps_ops =
{
  .activate = goldfish_gps_activate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int goldfish_gps_write_pipe(FAR struct file *pipe,
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

static inline int goldfish_gps_open_pipe(FAR struct file *filep,
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

  ret = goldfish_gps_write_pipe(filep, buf, buf_len + 1);
  if (ret < 0)
    {
      snerr("Could not connect to the '%s' service: %s",
            buf, strerror(-ret));
      file_close(filep);
      return ret;
    }

  return OK;
}

static int goldfish_gps_activate(FAR struct gps_lowerhalf_s *gps,
                                 FAR struct file *filep,
                                 bool enabled)
{
  FAR struct goldfish_gps_s *priv =
    container_of(gps, struct goldfish_gps_s, gps);
  priv->running = enabled;

  return OK;
}

static int goldfish_gps_thread(int argc, FAR char** argv)
{
  FAR struct goldfish_gps_s *priv = (FAR struct goldfish_gps_s *)
                                ((uintptr_t)strtoul(argv[1], NULL, 0));
  ssize_t len;
  char buf[256];

  while (true)
    {
      len = file_read(&priv->pipe, buf, sizeof(buf));
      if (priv->running && len > 0)
        {
          priv->gps.push_data(priv->gps.priv, buf, len, true);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_gps_init
 *
 * Description:
 *   Goldfish GPS driver entrypoint.
 *
 * Input Parameters:
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int goldfish_gps_init(int devno, uint32_t batch_number)
{
  FAR struct goldfish_gps_s *gps;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Alloc memory for sensor */

  gps = kmm_zalloc(sizeof(struct goldfish_gps_s));
  if (!gps)
    {
      return -ENOMEM;
    }

  ret = goldfish_gps_open_pipe(&gps->pipe, "qemud", "gps",
                               O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      kmm_free(gps);
      return ret;
    }

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", gps);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("goldfish_gps_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       goldfish_gps_thread, argv);
  if (ret < 0)
    {
      file_close(&gps->pipe);
      kmm_free(gps);
      return ret;
    }

  /*  Register sensor */

  gps->gps.ops = &g_goldfish_gps_ops;

  return gps_register(&gps->gps, devno, batch_number);
}
