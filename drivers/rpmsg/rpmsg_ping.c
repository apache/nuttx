/****************************************************************************
 * drivers/rpmsg/rpmsg_ping.c
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
#include <nuttx/signal.h>

#include <inttypes.h>
#include <string.h>
#include <sys/param.h>
#include <time.h>
#include <syslog.h>

#include "rpmsg_ping.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_PING_EPT_NAME         "rpmsg-ping"
#define RPMSG_PING_SEND             1
#define RPMSG_PING_SEND_CHECK       2
#define RPMSG_PING_SEND_NOACK       3
#define RPMSG_PING_ACK              4
#define RPMSG_PING_CHECK_DATA       0xee

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_ping_msg_s
{
  uint32_t cmd;
  uint32_t len;
  uint64_t cookie;
  uint8_t  data[1];
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsg_ping_ept_cb(FAR struct rpmsg_endpoint *ept,
                             FAR void *data, size_t len, uint32_t src,
                             FAR void *priv)
{
  FAR struct rpmsg_ping_msg_s *msg = data;
  FAR sem_t *sem = (FAR sem_t *)(uintptr_t)msg->cookie;

  if (msg->cmd == RPMSG_PING_SEND)
    {
      msg->cmd = RPMSG_PING_ACK;
      rpmsg_send(ept, msg, len);
    }
  else if (msg->cmd == RPMSG_PING_SEND_CHECK)
    {
      size_t data_len;
      size_t i;

      data_len = msg->len - sizeof(struct rpmsg_ping_msg_s) + 1;
      for (i = 0; i < data_len; i++)
        {
          if (msg->data[i] != RPMSG_PING_CHECK_DATA)
            {
              syslog(LOG_ERR, "rptun ping remote receive data error!\n");
              break;
            }

          msg->data[i] = 0;
        }

      msg->cmd = RPMSG_PING_ACK;
      rpmsg_send(ept, msg, len);
    }
  else if (msg->cmd == RPMSG_PING_ACK)
    {
      nxsem_post(sem);
    }

  return 0;
}

static int rpmsg_ping_once(FAR struct rpmsg_endpoint *ept,
                           int len, int ack, uint32_t *buf_len)
{
  FAR struct rpmsg_ping_msg_s *msg;
  int ret;

  msg = rpmsg_get_tx_payload_buffer(ept, buf_len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  len = MAX(len, sizeof(struct rpmsg_ping_msg_s));
  len = MIN(len, *buf_len);

  memset(msg, 0, len);

  if (ack)
    {
      sem_t sem;

      msg->cmd = (ack == 1)? RPMSG_PING_SEND : RPMSG_PING_SEND_CHECK;
      msg->len    = len;
      msg->cookie = (uintptr_t)&sem;

      if (msg->cmd == RPMSG_PING_SEND_CHECK)
        {
          memset(msg->data, RPMSG_PING_CHECK_DATA,
                 len - sizeof(struct rpmsg_ping_msg_s) + 1);
        }

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
      msg->cmd = RPMSG_PING_SEND_NOACK;
      msg->len = len;
      ret = rpmsg_send_nocopy(ept, msg, len);
    }

  return ret;
}

static void rpmsg_ping_logout(FAR const char *s, clock_t value)
{
  struct timespec ts;

  perf_convert(value, &ts);

#ifdef CONFIG_SYSTEM_TIME64
  syslog(LOG_INFO, "%s: %" PRIu64 " s, %ld ns\n", s, ts.tv_sec, ts.tv_nsec);
#else
  syslog(LOG_INFO, "%s: %" PRIu32 " s, %ld ns\n", s, ts.tv_sec, ts.tv_nsec);
#endif
}

static void rpmsg_ping_logout_rate(uint64_t len, clock_t avg)
{
  struct timespec ts;
  size_t ratebits;
  size_t rateint;
  size_t ratedec;

  perf_convert(avg, &ts);

  ratebits = len * 8 * 1000000000 / (ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec);
  rateint = ratebits / 1000000;
  ratedec = ratebits - rateint * 1000000;

  syslog(LOG_INFO, "rate: %zu.%06zu Mbits/sec\n", rateint, ratedec);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rpmsg_ping(FAR struct rpmsg_endpoint *ept,
               FAR const struct rpmsg_ping_s *ping)
{
  clock_t min = CLOCK_MAX;
  clock_t max = 0;
  uint64_t total = 0;
  uint32_t buf_len = 0;
  int send_len = 0;
  int i;

  if (!ept || !ping || ping->times <= 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < ping->times; i++)
    {
      clock_t tm = perf_gettime();

      send_len = rpmsg_ping_once(ept, ping->len, ping->ack, &buf_len);
      if (send_len < 0)
        {
          return send_len;
        }

      tm     = perf_gettime() - tm;
      min    = MIN(min, tm);
      max    = MAX(max, tm);
      total += tm;

      nxsig_usleep(ping->sleep * USEC_PER_MSEC);
    }

  syslog(LOG_INFO, "ping times: %d\n", ping->times);

  rpmsg_ping_logout("avg", total / ping->times);
  rpmsg_ping_logout("min", min);
  rpmsg_ping_logout("max", max);
  rpmsg_ping_logout_rate(send_len, total / ping->times);

  return 0;
}

int rpmsg_ping_init(FAR struct rpmsg_device *rdev,
                    FAR struct rpmsg_endpoint *ept)
{
  return rpmsg_create_ept(ept, rdev, RPMSG_PING_EPT_NAME,
                          RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                          rpmsg_ping_ept_cb, NULL);
}

void rpmsg_ping_deinit(FAR struct rpmsg_endpoint *ept)
{
  rpmsg_destroy_ept(ept);
}
