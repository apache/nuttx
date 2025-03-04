/****************************************************************************
 * drivers/coresight/coresight_stm.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/lib/math32.h>
#include <nuttx/bits.h>

#include <nuttx/coresight/coresight_stm.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM registers */

#define STM_DMASTARTR           0xc04
#define STM_DMASTOPR            0xc08
#define STM_DMASTATR            0xc0c
#define STM_DMACTLR             0xc10
#define STM_DMAIDR              0xcfc
#define STM_HEER                0xd00
#define STM_HETER               0xd20
#define STM_HEBSR               0xd60
#define STM_HEMCR               0xd64
#define STM_HEMASTR             0xdf4
#define STM_HEFEAT1R            0xdf8
#define STM_HEIDR               0xdfc
#define STM_SPER                0xe00
#define STM_SPTER               0xe20
#define STM_PRIVMASKR           0xe40
#define STM_SPSCR               0xe60
#define STM_SPMSCR              0xe64
#define STM_SPOVERRIDER         0xe68
#define STM_SPMOVERRIDER        0xe6c
#define STM_SPTRIGCSR           0xe70
#define STM_TCSR                0xe80
#define STM_TSSTIMR             0xe84
#define STM_TSFREQR             0xe8c
#define STM_SYNCR               0xe90
#define STM_AUXCR               0xe94
#define STM_SPFEAT1R            0xea0
#define STM_SPFEAT2R            0xea4
#define STM_SPFEAT3R            0xea8
#define STM_ITTRIGGER           0xee8
#define STM_ITATBDATA0          0xeec
#define STM_ITATBCTR2           0xef0
#define STM_ITATBID             0xef4
#define STM_ITATBCTR0           0xef8

#define STM_DEFAULT_CHANNELS    32
#define STM_BYTES_PER_CHANNEL   256

/* Register bit definition */

#define STM_EN                  BIT(0)
#define STM_TIMESTAMPED_EN      BIT(1)
#define STM_TCSR_BUSY           BIT(23)

#define STM_ATBTRIGEN_DIR       BIT(4)

#define STM_HE_EN               BIT(0)
#define STM_HE_ERRDETECT_EN     BIT(2)

/* Channel offset bit definition */

#define STM_NO_TIMESTAMPED      BIT(3)
#define STM_NO_MARKED           BIT(4)
#define STM_NO_GUARANTEED       BIT(7)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Address offset of different data type. */

enum stm_pkt_type_e
{
  STM_PKT_TYPE_DATA = 0x00,
  STM_PKT_TYPE_FLAG = 0x60,
  STM_PKT_TYPE_TRIG = 0x70,
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int stm_enable(FAR struct coresight_dev_s *csdev);
static void stm_disable(FAR struct coresight_dev_s *csdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_source_ops_s g_stm_source_ops =
{
  .enable  = stm_enable,
  .disable = stm_disable,
};

static const struct coresight_ops_s g_stm_ops =
{
  .source_ops = &g_stm_source_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm_hw_disable
 ****************************************************************************/

static void stm_hw_disable(FAR struct coresight_stm_dev_s *stmdev)
{
  coresight_unlock(stmdev->csdev.addr);
  coresight_modify32(0x0, STM_EN, stmdev->csdev.addr + STM_TCSR);
  coresight_put32(0x0, stmdev->csdev.addr + STM_SPER);
  coresight_put32(0x0, stmdev->csdev.addr + STM_SPTRIGCSR);

  coresight_put32(0x0, stmdev->csdev.addr + STM_HEMCR);
  coresight_put32(0x0, stmdev->csdev.addr + STM_HEER);
  coresight_put32(0x0, stmdev->csdev.addr + STM_HETER);

  if (coresight_timeout(0x0, STM_TCSR_BUSY,
                        stmdev->csdev.addr + STM_TCSR) < 0)
    {
      cserr("timeout waiting for STM stopped\n");
    }

  coresight_lock(stmdev->csdev.addr);
}

/****************************************************************************
 * Name: stm_hw_enable
 ****************************************************************************/

static void stm_hw_enable(FAR struct coresight_stm_dev_s *stmdev)
{
  coresight_unlock(stmdev->csdev.addr);
  if (stmdev->stmheer != 0)
    {
      coresight_put32(stmdev->stmhebsr, stmdev->csdev.addr + STM_HEBSR);
      coresight_put32(stmdev->stmheter, stmdev->csdev.addr + STM_HETER);
      coresight_put32(stmdev->stmheer, stmdev->csdev.addr + STM_HEER);
      coresight_put32(STM_HE_EN | STM_HE_ERRDETECT_EN,
                      stmdev->csdev.addr + STM_HEMCR);
    }

  coresight_put32(STM_ATBTRIGEN_DIR, stmdev->csdev.addr + STM_SPTRIGCSR);
  coresight_put32(stmdev->stmspscr, stmdev->csdev.addr + STM_SPSCR);
  coresight_put32(stmdev->stmsper, stmdev->csdev.addr + STM_SPER);

  /* 4096 byte between synchronisation packets */

  coresight_put32(0xfff, stmdev->csdev.addr + STM_SYNCR);
  coresight_put32((stmdev->traceid << 16) | STM_TIMESTAMPED_EN | STM_EN,
                  stmdev->csdev.addr + STM_TCSR);

  coresight_lock(stmdev->csdev.addr);
}

/****************************************************************************
 * Name: stm_hw_disable
 ****************************************************************************/

static void stm_disable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_stm_dev_s *stmdev =
    (FAR struct coresight_stm_dev_s *)csdev;

  stm_hw_disable(stmdev);
  coresight_disclaim_device(stmdev->csdev.addr);
}

/****************************************************************************
 * Name: stm_enable
 ****************************************************************************/

static int stm_enable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_stm_dev_s *stmdev =
    (FAR struct coresight_stm_dev_s *)csdev;
  int ret;

  ret = coresight_claim_device(stmdev->csdev.addr);
  if (ret < 0)
    {
      return ret;
    }

  stm_hw_enable(stmdev);
  return ret;
}

/****************************************************************************
 * Name: stm_get_stimulus_port_num
 ****************************************************************************/

static uint32_t stm_get_stimulus_port_num(uintptr_t addr)
{
  uint32_t numsp;

  coresight_unlock(addr);
  numsp = coresight_get32(addr + CORESIGHT_DEVID);
  if ((numsp & 0x1ffff) == 0)
    {
      numsp = STM_DEFAULT_CHANNELS;
    }

  coresight_lock(addr);
  return numsp;
}

/****************************************************************************
 * Name: stm_get_fundamental_data_size
 ****************************************************************************/

static uint8_t stm_get_fundamental_data_size(uintptr_t addr)
{
  uint32_t stmspfeat2r;

  if (sizeof(uintptr_t) == 4)
    {
      return 4;
    }

  coresight_unlock(addr);
  stmspfeat2r = coresight_get32(addr + STM_SPFEAT2R);
  coresight_lock(addr);

  return BMVAL(stmspfeat2r, 12, 15) ? 8 : 4;
}

/****************************************************************************
 * Name: stm_init_default_data
 ****************************************************************************/

static void stm_init_default_data(FAR struct coresight_stm_dev_s *stmdev)
{
  /* Do not use port selection, and enable all channels. */

  stmdev->stmspscr = 0x0;
  stmdev->stmsper = ~0x0;

  /* Disable hardware event tracing. */

  stmdev->stmheer = 0x0;

  /* Set invariant transaction timing on all channels. */

  memset(stmdev->guaranteed, 0,
         sizeof(unsigned long) * BITS_TO_LONGS(stmdev->numsp));
}

/****************************************************************************
 * Name: stm_send
 ****************************************************************************/

static void stm_send(uintptr_t addr, const void *data,
                     uint32_t size, uint8_t write_bytes)
{
  uint64_t payload[1];

  /* Check data address whether write_bytes aligned. */

  if ((unsigned long)data & (write_bytes - 1))
    {
      memcpy(payload, data, size);
      data = payload;
    }

  switch (size)
    {
      case 8:
        DEBUGASSERT(sizeof(uintptr_t) == 8);

        coresight_put64(*(uint64_t *)data, addr);
        break;

      case 4:
        coresight_put32(*(uint32_t *)data, addr);
        break;

      case 2:
        coresight_put16(*(uint16_t *)data, addr);
        break;

      case 1:
        coresight_put8(*(uint8_t *)data, addr);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm_set_channel_options
 *
 * Description:
 *   Set a channel's trace mode.
 *
 * Input Parameters:
 *   stmdev  - Pointer to STM device.
 *   channel - Channels to configure.
 *   options - If this channel's trace mode is guaranteed(blocking)
 *             or invariant(noblocking).
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int stm_set_channel_options(FAR struct coresight_stm_dev_s *stmdev,
                            uint32_t channel, uint32_t options)
{
  if (channel >= stmdev->numsp)
    {
      return -EINVAL;
    }

  switch (options)
    {
      case STM_OPTION_GUARANTEED:
        set_bit(channel, stmdev->guaranteed);
        break;

      case STM_OPTION_INVARIANT:
        clear_bit(channel, stmdev->guaranteed);
        break;

      default:
       return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: stm_sendpacket
 *
 * Description:
 *   Write data to STM device.
 *
 * Input Parameters:
 *   stmdev  - Pointer to STM device.
 *   type    - Data type.
 *   flag    - Data flags (or attributes).
 *   channel - Channels this data from.
 *   data    - Pointer to the data buffer.
 *   size    - Data size.
 *
 * Returned Value:
 *   Size of data written to STM device; a negative value on failure.
 *
 ****************************************************************************/

ssize_t stm_sendpacket(FAR struct coresight_stm_dev_s *stmdev,
                       enum stp_packet_type_e type,
                       enum stp_packet_flags_e flag, uint32_t channel,
                       FAR const void *data, size_t size)
{
  uintptr_t chaddr;
  uint32_t offset;

  if (channel >= stmdev->numsp)
    {
      return -EINVAL;
    }

  /* Aligned to STM fundamental data size. */

  if (size > stmdev->write_bytes)
    {
      size = stmdev->write_bytes;
    }
  else
    {
      size = rounddown_pow_of_two(size);
    }

  /* Write data to corresponding channel offset according to data flags. */

  chaddr = stmdev->stimulus_port_addr + channel * STM_BYTES_PER_CHANNEL;
  offset = (flag & STP_PACKET_TIMESTAMPED) ? 0 : STM_NO_TIMESTAMPED;
  offset |= test_bit(channel, stmdev->guaranteed) ? 0 : STM_NO_GUARANTEED;

  switch (type)
    {
      case STP_PACKET_FLAG:
        chaddr += offset | STM_PKT_TYPE_FLAG;
        stm_send(chaddr, data, 1, stmdev->write_bytes);
        size = 1;
        break;

      case STP_PACKET_DATA:
        offset |= (flag & STP_PACKET_MARKED) ? 0 : STM_NO_MARKED;
        chaddr += offset | STM_PKT_TYPE_DATA;
        stm_send(chaddr, data, size, stmdev->write_bytes);
        break;

      default:
        return -EOPNOTSUPP;
    }

  return size;
}

/****************************************************************************
 * Name: stm_register
 *
 * Description:
 *   Register a STM devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a STM device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_stm_dev_s *
stm_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_stm_dev_s *stmdev;
  FAR struct coresight_dev_s *csdev;
  uint32_t numsp = stm_get_stimulus_port_num(desc->addr);
  int ret;

  stmdev = kmm_zalloc(sizeof(struct coresight_stm_dev_s) +
                      sizeof(unsigned long) * BITS_TO_LONGS(numsp));
  if (stmdev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  stmdev->numsp = numsp;
  stmdev->write_bytes = stm_get_fundamental_data_size(desc->addr);
  stmdev->stimulus_port_addr = desc->stimulus_port_addr;

  stmdev->traceid = coresight_get_system_trace_id();
  if (stmdev->traceid < 0)
    {
      kmm_free(stmdev);
      cserr("%s:get unique traceid failed!\n", desc->name);
      return NULL;
    }

  stm_init_default_data(stmdev);

  csdev = &stmdev->csdev;
  csdev->ops = &g_stm_ops;
  ret = coresight_register(csdev, desc);
  if (ret < 0)
    {
      coresight_put_system_trace_id(stmdev->traceid);
      kmm_free(stmdev);
      cserr("%s:register failed\n", desc->name);
      return NULL;
    }

  return stmdev;
}

/****************************************************************************
 * Name: stm_unregister
 *
 * Description:
 *   Unregister a STM devices.
 *
 * Input Parameters:
 *   stmdev  - Pointer to the STM device.
 *
 ****************************************************************************/

void stm_unregister(FAR struct coresight_stm_dev_s *stmdev)
{
  coresight_unregister(&stmdev->csdev);
  coresight_put_system_trace_id(stmdev->traceid);
  kmm_free(stmdev);
}
