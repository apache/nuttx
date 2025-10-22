/****************************************************************************
 * drivers/timers/rpmsg_boottime.c
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

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/timers/rpmsg_boottime.h>

#include <errno.h>
#include <string.h>
#include <debug.h>
#include <sys/time.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_BOOTTIME_EPT_NAME             "rpmsg-boottime"

#define RPMSG_BOOTTIME_SYNC                 0
#define RPMSG_BOOTTIME_FOLLOW_UP            1
#define RPMSG_BOOTTIME_DELAYREQ             2
#define RPMSG_BOOTTIME_DELAYRESP            3

/* BOOTTIME client path delay averaging count, Measured path delay is
 * averaged over this many samples.
 */

#define RPMSG_BOOTTIME_DELAYREQ_AVGCOUNT    100

/* BOOTTIME client maximum path delay (ms),Measured path delay longer than
 * this is ignored.
 */

#define RPMSG_BOOTTIME_MAX_PATH_DELAY_MS    10

/* BOOTTIME client clock drift rate averaging time (ms) */

#define RPMSG_BOOTTIME_DRIFT_AVERAGE_MS     600000

/* BOOTTIME client timeout for changing clock source (ms), must received sync
 * from server before timeout.
 */

#define RPMSG_BOOTTIME_SYNC_TIMEOUT_MS      300000

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_boottime_msg_s
{
  uint32_t command;
  int64_t nsec;
} end_packed_struct;

struct rpmsg_boottime_client_s
{
  struct rpmsg_endpoint ept;
  struct work_s work;

  /* The localtime and remotetime when client received sync packet */

  int64_t received_localtime_ns;
  int64_t received_remotetime_ns;

  int64_t received_delayresp_ns;
  int64_t send_delayreq_ns;
  int64_t path_delay_ns;
  int32_t path_delay_avgcount;

  /* Previous measurement and estimated clock drift rate */

  int64_t last_delta_timestamp_ns;
  int64_t last_delta_ns;
  int64_t last_adjtime_ns;
  long drift_avg_total_ms;
  long drift_ppb;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int64_t rpmsg_boottime_get_time(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 1000000000ull * ts.tv_sec + ts.tv_nsec;
}

static void rpmsg_boottime_send_delay_req_work(FAR void *priv)
{
  FAR struct rpmsg_boottime_client_s *client = priv;
  struct rpmsg_boottime_msg_s msg;

  if (client->send_delayreq_ns != 0)
    {
      return;
    }

  msg.command = RPMSG_BOOTTIME_DELAYREQ;
  msg.nsec = rpmsg_boottime_get_time();
  rpmsg_send(&client->ept, &msg, sizeof(msg));

  /* Update timestamp after send completes */

  client->send_delayreq_ns = rpmsg_boottime_get_time();

  _info("rpmsg boottime send delay req:%"PRIi64" \n",
        client->send_delayreq_ns);
}

static
void rpmsg_boottime_update_local(FAR struct rpmsg_boottime_client_s *client)
{
  int64_t delta_ns;
  int64_t absdelta_ns;

  delta_ns = client->received_remotetime_ns - client->received_localtime_ns;
  delta_ns += client->path_delay_ns;
  absdelta_ns = (delta_ns < 0) ? -delta_ns : delta_ns;

  _info("delta_ns:%"PRIi64", path_delay_ns:%"PRIi64", sync:%"PRIi64"\n",
        delta_ns, client->path_delay_ns, client->received_remotetime_ns);

  if (absdelta_ns > CONFIG_RPMSG_BOOTTIME_SETTIME_THRESHOLD_MS *
                    (int64_t)NSEC_PER_MSEC)
    {
      struct timespec new_time;
      int64_t new_time_ns;

      /* Large difference, move by jumping. */

      new_time_ns = rpmsg_boottime_get_time() -
                    client->received_localtime_ns;
      new_time_ns += client->received_remotetime_ns;
      new_time.tv_sec = new_time_ns / NSEC_PER_SEC;
      new_time.tv_nsec = new_time_ns - new_time.tv_sec * NSEC_PER_SEC;
      clock_synchronize_monotonic(&new_time);

      /* Reinitialize drift adjustment parameters */

      client->last_delta_timestamp_ns = new_time_ns;
      client->last_delta_ns = 0;
      client->last_adjtime_ns = 0;
      client->drift_avg_total_ms = 0;
      client->drift_ppb = 0;

      _info("Jumped to timestamp %lld.%09ld s g_montonic_basetime:"
            "%lld.%09ld s\n", new_time.tv_sec, new_time.tv_nsec,
            g_monotonic_basetime.tv_sec, g_monotonic_basetime.tv_nsec);
    }
  else
    {
#ifdef CONFIG_CLOCK_ADJTIME
      struct timeval delta;
      int64_t adjustment_ns;
      int64_t drift_ppb = 0;
      int max_avg_period_ms;
      int interval_ms = 1;
      int ret = OK;

      interval_ms = (client->received_localtime_ns -
                     client->last_delta_timestamp_ns) / NSEC_PER_MSEC;
      if (interval_ms > 0 && interval_ms < RPMSG_BOOTTIME_SYNC_TIMEOUT_MS)
        {
          drift_ppb = (delta_ns - client->last_delta_ns) * MSEC_PER_SEC /
                      interval_ms;
        }

      /* Account for the adjustment previously made */

      drift_ppb += client->last_adjtime_ns * MSEC_PER_SEC /
                   CONFIG_CLOCK_ADJTIME_PERIOD_MS;

      if (drift_ppb > CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM * 1000 ||
          drift_ppb < -CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM * 1000)
        {
          drift_ppb = client->drift_ppb;
        }

      /* Take direct average of drift estimate for first measurements,
       * after that update the exponential sliding average.
       * Measurements are weighted according to the interval, because
       * drift estimate is more accurate over longer timespan.
       */

      client->drift_avg_total_ms += interval_ms;
      max_avg_period_ms = RPMSG_BOOTTIME_DRIFT_AVERAGE_MS;
      if (client->drift_avg_total_ms > max_avg_period_ms)
        {
          client->drift_avg_total_ms = max_avg_period_ms;
        }

      client->drift_ppb += (drift_ppb - client->drift_ppb) * interval_ms /
                           client->drift_avg_total_ms;

      /* Compute the value we need to give to adjtime() to match the
       * drift rate.
       */

      adjustment_ns = client->drift_ppb * CONFIG_CLOCK_ADJTIME_PERIOD_MS /
                      MSEC_PER_SEC;

      /* Drift estimation ensures local clock runs at same rate as remote.
       *
       * Adding the current clock offset to adjustment brings the clocks
       * to match. To avoid individual outliers from causing jitter, we
       * take the larger signed value of two previous deltas. This is based
       * on the logic that packets can get delayed in transit, but do not
       * travel backwards in time.
       *
       * Clock offset is applied over ADJTIME_PERIOD. If there is significant
       * noise in measurements, increasing ADJTIME_PERIOD will reduce its
       * effect on the local clock run rate.
       */

      if (client->last_delta_ns > delta_ns)
        {
          adjustment_ns += client->last_delta_ns;
        }
      else
        {
          adjustment_ns += delta_ns;
        }

      /* Apply adjustment and store information for next time */

      client->last_delta_ns = delta_ns;
      client->last_delta_timestamp_ns = client->received_localtime_ns;
      client->last_adjtime_ns = adjustment_ns;

      delta.tv_sec = adjustment_ns / NSEC_PER_SEC;
      adjustment_ns -= (int64_t)delta.tv_sec * NSEC_PER_SEC;
      delta.tv_usec = adjustment_ns / NSEC_PER_USEC;
      ret = adjtime(&delta, NULL);
      if (ret != OK)
        {
          tmrerr("adjtime() failed: %d\n", errno);
        }
      else
        {
          _info("adjtime local:%"PRIi64", remote:%"PRIi64", "
                "delta:%lld.%ld\n",
                client->received_localtime_ns,
                client->received_remotetime_ns,
                delta.tv_sec, delta.tv_usec);
        }
#endif
    }
}

static void
rpmsg_boottime_calc_path_delay(FAR struct rpmsg_boottime_client_s *client)
{
  int64_t path_delay_ns;
  int64_t sync_delay_ns;

  path_delay_ns = client->received_delayresp_ns - client->send_delayreq_ns;
  sync_delay_ns = client->path_delay_ns - client->last_delta_ns;
  path_delay_ns = (path_delay_ns + sync_delay_ns) / 2;
  client->send_delayreq_ns = 0;

  if (path_delay_ns >= 0 &&
      path_delay_ns < RPMSG_BOOTTIME_MAX_PATH_DELAY_MS * NSEC_PER_MSEC)
    {
      if (client->path_delay_avgcount < RPMSG_BOOTTIME_DELAYREQ_AVGCOUNT)
        {
          client->path_delay_avgcount++;
        }

      client->path_delay_ns += (path_delay_ns - client->path_delay_ns) /
                               client->path_delay_avgcount;

      _info("Path delay:%"PRIi64", path_delay_ns:%"PRIi64", "
            "last_delta_ns:%"PRIi64", avgcount:%"PRIi32"\n", path_delay_ns,
            client->path_delay_ns, client->last_delta_ns,
            client->path_delay_avgcount);
    }
  else
    {
      _info("Path delay out of range: %"PRIu64" ns\n", path_delay_ns);
    }
}

static int rpmsg_boottime_ept_cb(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_boottime_client_s *client = priv;
  FAR struct rpmsg_boottime_msg_s *msg = data;

  switch (msg->command)
    {
      case RPMSG_BOOTTIME_SYNC:
        {
          /* waiting for follow-up */

          client->received_localtime_ns = rpmsg_boottime_get_time();
        }
        break;

      case RPMSG_BOOTTIME_FOLLOW_UP:
        {
          client->received_remotetime_ns = msg->nsec;
          rpmsg_boottime_update_local(client);
        }
        break;

      case RPMSG_BOOTTIME_DELAYRESP:
        {
          client->received_delayresp_ns = msg->nsec;

          /* doing update localtime */

          rpmsg_boottime_calc_path_delay(client);
        }
        break;

      default:
        _err("rpmsg boottime invalid command\n");
        break;
    }

  return 0;
}

static void rpmsg_boottime_client_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_boottime_client_s *client;
  clock_t period = SEC2TICK(CONFIG_RPMSG_BOOTTIME_DELAY_REQ_INTERVAL);

  client = container_of(ept, struct rpmsg_boottime_client_s, ept);

  work_queue_next(HPWORK, &client->work,
                  rpmsg_boottime_send_delay_req_work,
                  client, period);
}

static void rpmsg_boottime_client_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_boottime_client_s *client = container_of(ept,
                           struct rpmsg_boottime_client_s, ept);

  work_cancel_sync(HPWORK, &client->work);
  memset(client, 0, sizeof(*client));
}

static void rpmsg_boottime_client_created(FAR struct rpmsg_device *rdev,
                                          FAR void *priv)
{
  FAR struct rpmsg_boottime_client_s *client = priv;

  if (strcmp(CONFIG_RPMSG_BOOTTIME_SERVER_NAME,
             rpmsg_get_cpuname(rdev)) == 0)
    {
      client->ept.priv = client;
      client->ept.release_cb = rpmsg_boottime_client_ept_release;
      client->ept.ns_bound_cb = rpmsg_boottime_client_ns_bound;
      rpmsg_create_ept(&client->ept, rdev, RPMSG_BOOTTIME_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_boottime_ept_cb, NULL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_boottime_initialize
 *
 * Description:
 *
 *   Using remote boottime to sync local boottime.
 *
 ****************************************************************************/

void rpmsg_boottime_initialize(void)
{
  FAR struct rpmsg_boottime_client_s *client;

  client = kmm_zalloc(sizeof(*client));
  if (client)
    {
      rpmsg_register_callback(client, rpmsg_boottime_client_created,
                              NULL, NULL, NULL);
    }
}
