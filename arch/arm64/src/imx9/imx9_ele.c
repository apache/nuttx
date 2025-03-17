/****************************************************************************
 * arch/arm64/src/imx9/imx9_ele.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "imx9_ele.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mmio_read_32(c)                       getreg32(c)
#define mmio_write_32(c, v)                   putreg32(v, c)
#define mmio_clrbits_32(addr, clear)          modifyreg32(addr, clear, 0)
#define mmio_setbits_32(addr, set)            modifyreg32(addr, 0, set)
#define mmio_clrsetbits_32(addr, clear, set)  modifyreg32(addr, clear, set)

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)(n))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ele_msg msg;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ele_sendmsg
 *
 * Description:
 *   This function communicates with the Advanced High Assurance Boot (AHAB)
 *   image that should reside in the particular address. This function
 *   sends a message to AHAB.
 *
 * Input Parameters:
 *   msg_ptr         -  Message to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ele_sendmsg(struct ele_msg *msg_ptr)
{
  /* Check that ele is ready to receive */

  while (!((1) & getreg32(ELE_MU_TSR)));

  /* write header to slog 0 */

  putreg32(msg_ptr->header.data, ELE_MU_TR(0));

  /* write data */

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      int tx_channel;

      tx_channel = i % ELE_TR_NUM;
      while (!((1 << tx_channel) & getreg32(ELE_MU_TSR)));

      /* Write data */

      putreg32(msg_ptr->data[i - 1], ELE_MU_TR(tx_channel));
    }
}

/****************************************************************************
 * Name: imx9_ele_receivemsg
 *
 * Description:
 *   This function communicates with the Advanced High Assurance Boot (AHAB)
 *   image that should reside in the particular address. This function
 *   receives message from AHAB.
 *
 * Input Parameters:
 *   msg_ptr         -  receive message buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ele_receivemsg(struct ele_msg *msg_ptr)
{
  /* Check if data ready */

  while (!((1) & getreg32(ELE_MU_RSR)));

  /* Read Header from slot 0 */

  msg_ptr->header.data = getreg32(ELE_MU_RR(0));

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      /* Check if empty */

      int rx_channel = (i) % ELE_RR_NUM;
      while (!((1 << rx_channel) & getreg32(ELE_MU_RSR)));

      /* Read data */

      msg_ptr->data[i - 1] = getreg32(ELE_MU_RR(rx_channel));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imx9_ele_init(void)
{
  putreg32(0, ELE_MU_TCR);
  putreg32(0, ELE_MU_RCR);
}

int imx9_ele_release_rdc(uint32_t rdc_id)
{
  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_RELEASE_RDC_REQ;
  msg.data[0] = rdc_id;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

uint32_t imx9_ele_read_common_fuse(uint32_t fuse_id)
{
  uint32_t value = 0;

  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_READ_FUSE_REQ;
  msg.data[0] = fuse_id;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      value = msg.data[1];
    }

  return value;
}

int imx9_ele_get_key(uint8_t *key, size_t key_size,
                     uint8_t *ctx, size_t ctx_size)
{
  if (!key)
    {
      _err("Invalid key parameter\n");
      return -EINVAL;
    }

  if (!ctx)
    {
      _err("Invalid context parameter\n");
      return -EINVAL;
    }

  if ((key_size != 16) && (key_size != 32))
    {
      _err("Invalid key size\n");
      return -EINVAL;
    }

  if (!IS_ALIGNED((uintptr_t)key, ARMV8A_DCACHE_LINESIZE))
    {
      _err("Invalid key alignment\n");
      return -EINVAL;
    }

  if (!IS_ALIGNED((uintptr_t)ctx, ARMV8A_DCACHE_LINESIZE))
    {
      _err("Invalid context alignment\n");
      return -EINVAL;
    }

  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 7;
  msg.header.command = ELE_DERIVE_KEY_REQ;
  msg.data[0] = upper_32_bits((ulong)key);
  msg.data[1] = lower_32_bits((ulong)key);
  msg.data[2] = upper_32_bits((ulong)ctx);
  msg.data[3] = lower_32_bits((ulong)ctx);
  msg.data[4] = ((ctx_size << 16) | key_size);

  uint32_t crc = msg.header.data;
  for (uint32_t i = 0; i < msg.header.size - 2; i++)
    {
      crc ^= msg.data[i];
    }

  msg.data[5] = crc;

  up_flush_dcache((uintptr_t)ctx, (uintptr_t)(ctx + ctx_size));
  up_invalidate_dcache((uintptr_t)key, (uintptr_t)(key + key_size));

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  up_invalidate_dcache((uintptr_t)key, (uintptr_t)(key + key_size));

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imx9_ele_get_events(uint32_t *buffer, size_t buffer_size)
{
  size_t events_num;
  size_t i;

  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_GET_EVENTS;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      events_num = msg.data[1] & 0xffff;
      if (buffer)
        {
          for (i = 0; (i < buffer_size) && (i < events_num); i++)
            {
              buffer[i] =  msg.data[i + 2];
            }

          return (int)i;
        }
      else
        {
          return (int)events_num;
        }
    }

  return -EIO;
}

int imx9_ele_close_device(void)
{
  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_FWD_LIFECYCLE_UP;
  msg.data[0] = 0x08;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

uint32_t imx9_ele_get_lifecycle(void)
{
  return (getreg32(FSB_LC_REG) & 0x3ff);
}

