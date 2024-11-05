/****************************************************************************
 * drivers/coresight/coresight_common.c
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

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <nuttx/spinlock.h>
#include <nuttx/bits.h>
#include <nuttx/arch.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CORESIGHT_UNLOCK               0xc5acce55
#define CORESIGHT_LOCK                 0x0

#define CORESIGHT_SOURCE_BITMAP_SIZE   32

#define CORESIGHT_ETM_PMU_SEED         0x10

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DECLARE_BITMAP(g_coresight_trace_id_bitmap,
                      CORESIGHT_SOURCE_BITMAP_SIZE);
static spinlock_t g_coresight_trace_id_lock = SP_UNLOCKED;

static const uint32_t g_coresight_barrier_pkt[4] =
{
  0x7fffffff,
  0x7fffffff,
  0x7fffffff,
  0x7fffffff
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: coresight_lock
 *
 * Description:
 *   To ensure that the software being debugged can never access an unlocked
 *   CoreSight component, a software monitor that accesses debug registers
 *   must unlock the component before accessing any registers, and lock the
 *   component again before exiting the monitor.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 ****************************************************************************/

void coresight_lock(uintptr_t addr)
{
  coresight_put32(CORESIGHT_LOCK, addr + CORESIGHT_LAR);
}

/****************************************************************************
 * Name: coresight_unlock
 ****************************************************************************/

void coresight_unlock(uintptr_t addr)
{
  coresight_put32(CORESIGHT_UNLOCK, addr + CORESIGHT_LAR);
}

/****************************************************************************
 * Name: coresight_claim_device
 *
 * Description:
 *   Claim the device for self-hosted usage to prevent an external tool from
 *   touching this device.Use Protocol 3: Set private bit and check for race.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_claim_device(uintptr_t addr)
{
  int ret = -EBUSY;

  coresight_unlock(addr);
  if (coresight_get32(addr + CORESIGHT_CLAIMCLR) != 0)
    {
      goto out;
    }

  coresight_put32(CORESIGHT_CLAIM_SELF_HOSTED, addr + CORESIGHT_CLAIMSET);
  if (coresight_get32(addr + CORESIGHT_CLAIMCLR) ==
        CORESIGHT_CLAIM_SELF_HOSTED)
    {
      ret = 0;
    }
  else
    {
      /* There was a race setting the tags, clean up and fail */

      coresight_put32(CORESIGHT_CLAIM_SELF_HOSTED,
                      addr + CORESIGHT_CLAIMCLR);
    }

out:
  coresight_lock(addr);
  return ret;
}

/****************************************************************************
 * Name: coresight_disclaim_device
 *
 * Description:
 *   Disclaim the device, then an external tool can touch the device.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 ****************************************************************************/

void coresight_disclaim_device(uintptr_t addr)
{
  coresight_unlock(addr);
  if (coresight_get32(addr + CORESIGHT_CLAIMCLR) ==
        CORESIGHT_CLAIM_SELF_HOSTED)
    {
      coresight_put32(CORESIGHT_CLAIM_SELF_HOSTED,
                      addr + CORESIGHT_CLAIMCLR);
    }
  else
    {
      cserr("current device is not claimed or something wrong happend\n");
    }

  coresight_lock(addr);
}

/****************************************************************************
 * Name: coresight_get_cpu_trace_id
 *
 * Description:
 *   Used to get an unique trace id associated with cpu id of an ETM
 *   coresight device.
 *
 * Input Parameters:
 *   cpu  - CPU index to generate an unique trace id.
 *
 * Returned Value:
 *   Unique trace id on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_get_cpu_trace_id(int cpu)
{
  int traceid = CORESIGHT_ETM_PMU_SEED + cpu * 2;

  if (traceid >= CORESIGHT_SOURCE_BITMAP_SIZE)
    {
      cserr("trace id is out of bounds\n");
      return -EINVAL;
    }

  return traceid;
}

/****************************************************************************
 * Name: coresight_get_system_trace_id
 *
 * Description:
 *   Used to get an unique trace id of a STM coresight device. The trace ID
 *   value for *ETM* tracers start at CPU_ID * 2 + 0x10, and Trace ID 0x00
 *   and anything equal to or higher than 0x70 is reserved.
 *
 * Returned Value:
 *   Unique trace id on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_get_system_trace_id(void)
{
  int max = CORESIGHT_SOURCE_BITMAP_SIZE / 2;
  irqstate_t flags;
  int traceid = 0;
  int i;

  flags = spin_lock_irqsave(&g_coresight_trace_id_lock);
  for (i = 0; i < max - 1; i++)
    {
      if (test_bit(1 + 2 * i, g_coresight_trace_id_bitmap) == 0)
        {
          traceid = 1 + 2 * i;
          break;
        }
    }

  if (traceid == 0)
    {
      cserr("get system trace id failed\n");
      spin_unlock_irqrestore(&g_coresight_trace_id_lock, flags);
      return -EINVAL;
    }

  set_bit(traceid, g_coresight_trace_id_bitmap);
  spin_unlock_irqrestore(&g_coresight_trace_id_lock, flags);

  return traceid;
}

/****************************************************************************
 * Name: coresight_put_system_trace_id
 *
 * Description:
 *    Release an allocated system trace ID.
 *
 * Input Parameters:
 *   traceid  - Traceid to be released.
 *
 ****************************************************************************/

void coresight_put_system_trace_id(int traceid)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_coresight_trace_id_lock);
  clear_bit(traceid, g_coresight_trace_id_bitmap);
  spin_unlock_irqrestore(&g_coresight_trace_id_lock, flags);
}

/****************************************************************************
 * Name: coresight_timeout
 *
 * Description:
 *   Loop until a bitmask of register has changed to a specific value.
 *
 * Input Parameters:
 *   addr    - Base addr of the coresight device.
 *   off     - Register offset of the coresight device.
 *   bitmask - Bitmask to be checked.
 *   val     - Value to be matched.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_timeout(uint32_t val, uint32_t mask, uintptr_t addr)
{
  int i;

  for (i = CONFIG_CORESIGHT_TIMEOUT; i > 0; i--)
    {
      uint32_t value = coresight_get32(addr);
      if ((value & mask) == val)
        {
          return 0;
        }

      up_udelay(1);
    }

  return -EAGAIN;
}

/****************************************************************************
 * Name: coresight_insert_barrier_packet
 *
 * Description:
 *   When losing synchronisation a new barrier packet needs to be inserted at
 *   the beginning of the data collected in a buffer.  That way the decoder
 *   knows that it needs to look for another sync sequence.
 *
 * Input Parameters:
 *   buf  - buffer that a new barrier packet inserts to.
 *
 ****************************************************************************/

void coresight_insert_barrier_packet(FAR void *buf)
{
  if (buf != NULL)
    {
      memcpy(buf, g_coresight_barrier_pkt, sizeof(g_coresight_barrier_pkt));
    }
}
