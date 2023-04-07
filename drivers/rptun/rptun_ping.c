/****************************************************************************
 * drivers/rptun/rptun_ping.c
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
#include <nuttx/arch.h>

#include <inttypes.h>
#include <string.h>
#include <sys/param.h>

#include "rptun.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPTUN_PING_EPT_NAME         "rpmsg-ping"
#define RPTUN_PING_SEND             1
#define RPTUN_PING_SEND_NOACK       2
#define RPTUN_PING_ACK              3

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rptun_ping_msg_s
{
  uint32_t cmd;
  uint32_t len;
  uint64_t cookie;
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rptun_ping_ept_cb(FAR struct rpmsg_endpoint *ept,
                             FAR void *data, size_t len, uint32_t src,
                             FAR void *priv)
{
  FAR struct rptun_ping_msg_s *msg = data;
  FAR sem_t *sem = (FAR sem_t *)(uintptr_t)msg->cookie;

  if (msg->cmd == RPTUN_PING_SEND)
    {
      msg->cmd = RPTUN_PING_ACK;
      rpmsg_send(ept, msg, len);
    }
  else if (msg->cmd == RPTUN_PING_ACK)
    {
      nxsem_post(sem);
    }

  return 0;
}

static int rptun_ping_once(FAR struct rpmsg_endpoint *ept,
                           int len, bool ack)
{
  FAR struct rptun_ping_msg_s *msg;
  uint32_t space;
  int ret;

  msg = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  len = MAX(len, sizeof(struct rptun_ping_msg_s));
  len = MIN(len, space);

  memset(msg, 0, len);

  if (ack)
    {
      sem_t sem;

      msg->cmd    = RPTUN_PING_SEND;
      msg->len    = len;
      msg->cookie = (uintptr_t)&sem;

      nxsem_init(&sem, 0, 0);

      ret = rpmsg_send_nocopy(ept, msg, len);
      if (ret >= 0)
        {
          nxsem_wait_uninterruptible(&sem);
        }

      nxsem_destroy(&sem);
    }
  else
    {
      msg->cmd = RPTUN_PING_SEND_NOACK;
      msg->len = len;
      ret = rpmsg_send_nocopy(ept, msg, len);
    }

  return ret;
}

static void rptun_ping_logout(FAR const char *s, unsigned long value)
{
  struct timespec ts;

  up_perf_convert(value, &ts);

#ifdef CONFIG_SYSTEM_TIME64
  syslog(LOG_INFO, "%s: s %" PRIu64 ", ns %ld\n", s, ts.tv_sec, ts.tv_nsec);
#else
  syslog(LOG_INFO, "%s: s %" PRIu32 ", ns %ld\n", s, ts.tv_sec, ts.tv_nsec);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rptun_ping(FAR struct rpmsg_endpoint *ept,
               FAR const struct rptun_ping_s *ping)
{
  unsigned long min = ULONG_MAX;
  unsigned long max = 0;
  uint64_t total = 0;
  int i;

  if (!ept || !ping || ping->times <= 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < ping->times; i++)
    {
      unsigned long tm = up_perf_gettime();

      int ret = rptun_ping_once(ept, ping->len, ping->ack);
      if (ret < 0)
        {
          return ret;
        }

      tm     = up_perf_gettime() - tm;
      min    = MIN(min, tm);
      max    = MAX(max, tm);
      total += tm;

      usleep(ping->sleep * USEC_PER_MSEC);
    }

  syslog(LOG_INFO, "current CPU freq: %lu, ping times: %d\n",
                    up_perf_getfreq(), ping->times);

  rptun_ping_logout("avg", total / ping->times);
  rptun_ping_logout("min", min);
  rptun_ping_logout("max", max);

  return 0;
}

int rptun_ping_init(FAR struct rpmsg_virtio_device *rvdev,
                    FAR struct rpmsg_endpoint *ept)
{
  return rpmsg_create_ept(ept, &rvdev->rdev, RPTUN_PING_EPT_NAME,
                          RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                          rptun_ping_ept_cb, NULL);
}

void rptun_ping_deinit(FAR struct rpmsg_endpoint *ept)
{
  rpmsg_destroy_ept(ept);
}
