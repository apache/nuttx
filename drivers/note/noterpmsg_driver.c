/****************************************************************************
 * drivers/note/noterpmsg_driver.c
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

#include <nuttx/note/note_driver.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/sched_note.h>
#include <nuttx/wqueue.h>

#include "noterpmsg.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define NOTE_RPMSG_WORK_DELAY MSEC2TICK(CONFIG_DRIVERS_NOTERPMSG_WORK_DELAY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noterpmsg_driver_s
{
  struct note_driver_s  driver;
  volatile size_t       head;
  volatile size_t       tail;
  uint8_t               buffer[CONFIG_DRIVERS_NOTERPMSG_BUFSIZE];
  struct work_s         work;
  struct rpmsg_endpoint ept;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void noterpmsg_add(FAR struct note_driver_s *driver,
                          FAR const void *note, size_t notelen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_noterpmsg_ops =
{
  noterpmsg_add
};

struct noterpmsg_driver_s g_noterpmsg_driver =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "rpmsg",
    {
      {
        CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
      },
    },
#endif
    &g_noterpmsg_ops
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline size_t noterpmsg_next(FAR struct noterpmsg_driver_s *drv,
                                    size_t pos, size_t offset)
{
  pos += offset;
  if (pos >= CONFIG_DRIVERS_NOTERPMSG_BUFSIZE)
    {
      pos -= CONFIG_DRIVERS_NOTERPMSG_BUFSIZE;
    }

  return pos;
}

static inline size_t noterpmsg_length(FAR struct noterpmsg_driver_s *drv)
{
  size_t head = drv->head;
  size_t tail = drv->tail;

  if (tail > head)
    {
      head += CONFIG_DRIVERS_NOTERPMSG_BUFSIZE;
    }

  return head - tail;
}

static inline void noterpmsg_remove(FAR struct noterpmsg_driver_s *drv)
{
  size_t tail = drv->tail;
  uint8_t notelen = drv->buffer[tail];

  DEBUGASSERT(notelen <= noterpmsg_length(drv));
  drv->tail = noterpmsg_next(drv, tail, notelen);
}

static bool noterpmsg_transfer(FAR struct noterpmsg_driver_s *drv,
                               bool wait)
{
  for (; ; )
    {
      FAR uint8_t *buffer;
      uint32_t space;
      size_t len;

      len = noterpmsg_length(drv);
      if (len == 0)
        {
          return true;
        }

      buffer = rpmsg_get_tx_payload_buffer(&drv->ept, &space, wait);
      if (buffer == NULL)
        {
          return false;
        }

      if (space < len)
        {
          /* Find the len of large entire note data */

          size_t pos = drv->tail;
          uint8_t notelen = drv->buffer[pos];

          len = 0;
          while (len + notelen <= space)
            {
              pos = noterpmsg_next(drv, pos, notelen);
              len += notelen;
              notelen = drv->buffer[pos];
            }
        }

      space = CONFIG_DRIVERS_NOTERPMSG_BUFSIZE - drv->tail;
      space = space < len ? space : len;

      memcpy(buffer, drv->buffer + drv->tail, space);
      memcpy(buffer + space, drv->buffer, len - space);

      if (rpmsg_send_nocopy(&drv->ept, buffer, len) < 0)
        {
          rpmsg_release_tx_buffer(&drv->ept, buffer);
        }

      drv->tail = noterpmsg_next(drv, drv->tail, len);
    }
}

static void noterpmsg_work(FAR void *priv)
{
  FAR struct noterpmsg_driver_s *drv = priv;
  irqstate_t flags = enter_critical_section();

  if (!noterpmsg_transfer(drv, false))
    {
      work_queue(HPWORK, &drv->work, noterpmsg_work, drv,
                 NOTE_RPMSG_WORK_DELAY);
    }

  leave_critical_section(flags);
}

static void noterpmsg_add(FAR struct note_driver_s *driver,
                          FAR const void *note, size_t notelen)
{
  FAR struct noterpmsg_driver_s *drv =
    (FAR struct noterpmsg_driver_s *)driver;
  irqstate_t flags;
  size_t space;

  flags = enter_critical_section();

  space = CONFIG_DRIVERS_NOTERPMSG_BUFSIZE - noterpmsg_length(drv);
  if (space < notelen)
    {
      if (!up_interrupt_context() && !sched_idletask())
        {
          noterpmsg_transfer(drv, true);
        }
      else
        {
          /* Overwrite */

          do
            {
              noterpmsg_remove(drv);
              space = CONFIG_DRIVERS_NOTERPMSG_BUFSIZE -
                      noterpmsg_length(drv);
            }
          while (space < notelen);
        }
    }

  space = CONFIG_DRIVERS_NOTERPMSG_BUFSIZE - drv->head;
  space = space < notelen ? space : notelen;

  memcpy(drv->buffer + drv->head, note, space);
  memcpy(drv->buffer, note + space, notelen - space);

  drv->head = noterpmsg_next(drv, drv->head, notelen);

  if (work_available(&drv->work))
    {
      work_queue(HPWORK, &drv->work, noterpmsg_work, drv,
                 NOTE_RPMSG_WORK_DELAY);
    }

  leave_critical_section(flags);
}

static int noterpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len, uint32_t src,
                            FAR void *priv)
{
  return 0;
}

static void noterpmsg_device_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct noterpmsg_driver_s *drv = priv;
  int ret;

  if (strcmp(CONFIG_DRIVERS_NOTERPMSG_SERVER_NAME,
             rpmsg_get_cpuname(rdev)) == 0)
    {
      drv->ept.priv = drv;

      ret = rpmsg_create_ept(&drv->ept, rdev, NOTERPMSG_EPT_NAME,
                             RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                             noterpmsg_ept_cb, NULL);
      if (ret >= 0)
        {
          work_queue(HPWORK, &drv->work, noterpmsg_work, drv, 0);
        }
    }
}

static void noterpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct noterpmsg_driver_s *drv = priv;

  if (strcmp(CONFIG_DRIVERS_NOTERPMSG_SERVER_NAME,
             rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&drv->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noterpmsg_init
 *
 * Description:
 *   Register a rmpsg channel to note.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int noterpmsg_init(void)
{
  return rpmsg_register_callback(&g_noterpmsg_driver,
                                 noterpmsg_device_created,
                                 noterpmsg_device_destroy,
                                 NULL,
                                 NULL);
}
