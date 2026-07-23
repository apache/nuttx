/****************************************************************************
 * arch/sim/src/sim/posix/sim_bsimtime.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "bs_pc_base.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSEC_PER_BSIM_USEC 1000ull

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pb_dev_state_t g_bsim_dev;
static uint64_t g_now_nsec;
static bool g_connected;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void bsimtime_disconnect_atexit(void)
{
  host_bsimtime_disconnect();
}

static void bsimtime_fail(void)
{
  host_abort(1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_bsimtime_init(const char *sim_id, const char *phy_id,
                       unsigned int dev_nbr)
{
  int ret;

  if (sim_id == NULL || phy_id == NULL)
    {
      return -EINVAL;
    }

  ret = pb_dev_init_com(&g_bsim_dev, dev_nbr, sim_id, phy_id);
  if (ret != 0)
    {
      return ret;
    }

  g_connected = true;
  atexit(bsimtime_disconnect_atexit);

  return 0;
}

uint64_t host_bsimtime_gettime(void)
{
  return g_now_nsec;
}

bool host_bsimtime_is_enabled(void)
{
  return g_connected;
}

void host_bsimtime_sleepuntil(uint64_t nsec)
{
  pb_wait_t wait;

  if (nsec <= g_now_nsec)
    {
      return;
    }

  if (!g_connected)
    {
      bsimtime_fail();
    }

  wait.end = (nsec + NSEC_PER_BSIM_USEC - 1) / NSEC_PER_BSIM_USEC;

  if (pb_dev_request_wait_block(&g_bsim_dev, &wait) < 0)
    {
      bsimtime_fail();
    }

  g_now_nsec = wait.end * NSEC_PER_BSIM_USEC;
}

void host_bsimtime_disconnect(void)
{
  if (!g_connected)
    {
      return;
    }

  pb_dev_disconnect(&g_bsim_dev);
  g_connected = false;
}
