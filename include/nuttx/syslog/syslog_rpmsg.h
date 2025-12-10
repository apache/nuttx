/****************************************************************************
 * include/nuttx/syslog/syslog_rpmsg.h
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

#ifndef __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H
#define __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define SYSLOG_RPMSG_MAGIC_NUMBER   0x53595250

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct syslog_rpmsg_buffer_header_s
{
  uint32_t              magic;
  volatile uint32_t     head;       /* The head index (where data is added) */
  volatile uint32_t     tail;       /* The tail index (where data is removed) */
  uint32_t              size;       /* Size of the RAM buffer */
  char                  buffer[];   /* Circular RAM buffer */
};

#ifdef CONFIG_SYSLOG_RPMSG
struct syslog_rpmsg_s
{
  FAR struct syslog_rpmsg_buffer_header_s *header; /* The trace header */
  struct work_s                            work;   /* Used for deferred callback work */

  struct rpmsg_endpoint                    ept;
  spinlock_t                               lock;
  bool                                     suspend;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern struct syslog_rpmsg_s g_syslog_rpmsg;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_RPMSG
int syslog_rpmsg_init(void);

int syslog_rpmsg_putc(FAR syslog_channel_t *channel, int ch);
ssize_t syslog_rpmsg_write(FAR syslog_channel_t *channel,
                           FAR const char *buffer, size_t buflen);
#endif

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
int syslog_rpmsg_server_init(void);
#endif

static inline_function void syslog_rpmsg_dump(FAR const void *buffer)
{
  FAR const struct syslog_rpmsg_buffer_header_s *header =
    (FAR const struct syslog_rpmsg_buffer_header_s *)buffer;
  uint32_t head = header->head;
  uint32_t tail = header->tail;
  uint32_t size = header->size;

  if (head > tail)
    {
      _alert("%.*s\n", (int)(head - tail), &header->buffer[tail]);
    }
  else if (head < tail)
    {
      _alert("%.*s%.*s\n", (int)(size - tail), &header->buffer[tail],
             (int)head, &header->buffer[0]);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SYSLOG_SYSLOG_RPMSG_H */
