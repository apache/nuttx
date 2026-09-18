/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_ele.c
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
#include <nuttx/nuttx.h>

#include <sys/types.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/rt118x/imxrt118x_memorymap.h"
#include "imxrt118x_ele.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ELE_LOAD_FW
#  ifndef IMXRT_ELE_FW_ABS_PATH
#    error "IMXRT_ELE_FW_ABS_PATH must be set via CFLAGS"
#  endif

#  define STR2(m) #m
#  define STR(m) STR2(m)
#endif

/* The M7 core is Armv7-M and the M33 core is Armv8-M; pick whichever
 * D-Cache line size macro chip.h provided for the core we're building
 * for.
 */

#if defined(ARMV8M_DCACHE_LINESIZE)
#  define DCACHE_LINESIZE ARMV8M_DCACHE_LINESIZE
#else
#  define DCACHE_LINESIZE ARMV7M_DCACHE_LINESIZE
#endif

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)(n))

#define ELE_RNG_TIMEOUT_US    5000
#define ELE_RNG_SLEEP_US      100
#define ELE_TRNG_STATUS_READY 0x3
#define ELE_CSAL_STATUS_READY 0x2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ele_trng_state
{
  uint8_t  trng_state;
  uint8_t  csal_state;
  uint16_t reserved;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ele_msg g_msg;

#ifdef CONFIG_IMXRT_ELE_LOAD_FW
/* Embeds the NXP EdgeLock Enclave firmware AHAB container (path from
 * CONFIG_IMXRT_ELE_FW_PATH) directly into the driver image.
 *
 * This is legacy/fallback support: the RT118x ROM can also load the ELE
 * FW automatically from a properly packed AHAB container, in which case
 * CONFIG_IMXRT_ELE_LOAD_FW should be disabled.
 */

__asm__ (
    ".section .rodata.imxrt118x_ele_fw, \"a\"\n"
    ".balign  8\n"
    ".globl   imxrt118x_ele_fw\n"
"imxrt118x_ele_fw:\n"
    ".incbin " STR(IMXRT_ELE_FW_ABS_PATH) "\n"
    ".balign  8\n"
    ".globl   imxrt118x_ele_fw_end\n"
"imxrt118x_ele_fw_end:\n"
);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_ele_sendmsg
 *
 * Description:
 *   Send a message to the EdgeLock Enclave over the System 3 Message Unit A.
 *
 ****************************************************************************/

static void imxrt118x_ele_sendmsg(struct ele_msg *msg_ptr)
{
  /* Check that ele is ready to receive */

  while (!((1) & getreg32(ELE_MU_TSR)));

  /* write header to slot 0 */

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
 * Name: imxrt118x_ele_receivemsg
 *
 * Description:
 *   Receive a response message from the EdgeLock Enclave.
 *
 ****************************************************************************/

static void imxrt118x_ele_receivemsg(struct ele_msg *msg_ptr)
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

void imxrt118x_ele_init(void)
{
  putreg32(0, ELE_MU_TCR);
  putreg32(0, ELE_MU_RCR);

#ifdef CONFIG_IMXRT_ELE_LOAD_FW
  /* Load the ELE firmware via mailbox */

  imxrt118x_ele_load_fw((uint32_t)(uintptr_t)imxrt118x_ele_fw);
#endif

  imxrt118x_ele_check_fw_version();
}

int imxrt118x_ele_load_fw(uint32_t fw_addr)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 4;
  g_msg.header.command = ELE_LOAD_FW_REQ;
  g_msg.data[0] = fw_addr;
  g_msg.data[1] = 0;
  g_msg.data[2] = fw_addr;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_release_rdc(uint32_t rdc_id)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_RELEASE_RDC_REQ;
  g_msg.data[0] = rdc_id;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

uint32_t imxrt118x_ele_read_common_fuse(uint32_t fuse_id)
{
  uint32_t value = 0;

  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_READ_FUSE_REQ;
  g_msg.data[0] = fuse_id;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      value = g_msg.data[1];
    }

  return value;
}

int imxrt118x_ele_get_key(uint8_t *key, size_t key_size,
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

  if (!IS_ALIGNED((uintptr_t)key, DCACHE_LINESIZE))
    {
      _err("Invalid key alignment\n");
      return -EINVAL;
    }

  if (!IS_ALIGNED((uintptr_t)ctx, DCACHE_LINESIZE))
    {
      _err("Invalid context alignment\n");
      return -EINVAL;
    }

  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 7;
  g_msg.header.command = ELE_DERIVE_KEY_REQ;
  g_msg.data[0] = upper_32_bits((uintptr_t)key);
  g_msg.data[1] = lower_32_bits((uintptr_t)key);
  g_msg.data[2] = upper_32_bits((uintptr_t)ctx);
  g_msg.data[3] = lower_32_bits((uintptr_t)ctx);
  g_msg.data[4] = ((ctx_size << 16) | key_size);

  uint32_t crc = g_msg.header.data;

  for (uint32_t i = 0; i < g_msg.header.size - 2; i++)
    {
      crc ^= g_msg.data[i];
    }

  g_msg.data[5] = crc;

  up_flush_dcache((uintptr_t)ctx, (uintptr_t)(ctx + ctx_size));
  up_invalidate_dcache((uintptr_t)key, (uintptr_t)(key + key_size));

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  up_invalidate_dcache((uintptr_t)key, (uintptr_t)(key + key_size));

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_get_events(uint32_t *buffer, size_t buffer_size)
{
  size_t events_num;
  size_t i;

  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_GET_EVENTS_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      events_num = g_msg.data[1] & 0xffff;
      if (buffer)
        {
          for (i = 0; (i < buffer_size) && (i < events_num); i++)
            {
              buffer[i] =  g_msg.data[i + 2];
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

int imxrt118x_ele_close_device(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_FWD_LIFECYCLE_UP_REQ;
  g_msg.data[0] = 0x08;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

uint32_t imxrt118x_ele_get_lifecycle(void)
{
  static uint8_t info[ELE_GET_INFO_BYTES]
    aligned_data(DCACHE_LINESIZE);

  uint32_t addr = (uint32_t)(uintptr_t)info;

  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 4;
  g_msg.header.command = ELE_GET_INFO_REQ;
  g_msg.data[0] = upper_32_bits(addr);
  g_msg.data[1] = lower_32_bits(addr);
  g_msg.data[2] = sizeof(info);

  up_invalidate_dcache(addr, addr + sizeof(info));

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) != ELE_OK)
    {
      return 0;
    }

  up_invalidate_dcache(addr, addr + sizeof(info));

  return ((uint32_t *)info)[ELE_GET_INFO_LC_WORD] & ELE_GET_INFO_LC_MASK;
}

int imxrt118x_ele_auth_oem_ctnr(unsigned long ctnr_addr, uint32_t *response)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 3;
  g_msg.header.command = ELE_OEM_CNTN_AUTH_REQ;
  g_msg.data[0] = upper_32_bits(ctnr_addr);
  g_msg.data[1] = lower_32_bits(ctnr_addr);

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);
  if (response)
    {
      *response = g_msg.data[0];
    }

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_release_container(uint32_t *response)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_RELEASE_CONTAINER_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if (response)
    {
      *response = g_msg.data[0];
    }

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_verify_image(uint32_t img_id, uint32_t *response)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_VERIFY_IMAGE_REQ;
  g_msg.data[0] = 1 << img_id;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if (response)
    {
      *response = g_msg.data[0];
    }

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_start_rng(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_START_RNG_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_get_trng_state(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_GET_TRNG_STATE_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      struct ele_trng_state *ele_trng =
        (struct ele_trng_state *)(g_msg.data + 1);

      if (ele_trng->trng_state != ELE_TRNG_STATUS_READY ||
          ele_trng->csal_state != ELE_CSAL_STATUS_READY)
        {
          /* Ensure imxrt118x_ele_start_rng() was called earlier or we
           * will end up here.
           */

          return -EBUSY;
        }
      else
        {
          return 0;
        }
    }

  return -EIO;
}

int imxrt118x_ele_get_random(uint32_t paddr, size_t len)
{
  uint16_t counter = 0;
  uint16_t max_tries = ELE_RNG_TIMEOUT_US / ELE_RNG_SLEEP_US;

  if (paddr == 0 || len == 0)
    {
      _err("Wrong input parameters!\n");
      return -EINVAL;
    }

  while ((imxrt118x_ele_get_trng_state() != 0))
    {
      if (counter > max_tries)
        {
          _err("Timed out after %hu iterations!\n", counter);
          return -EBUSY;
        }

      usleep(ELE_RNG_SLEEP_US);
      counter++;
    }

  /* Flush the cache before sending the request to ELE. */

  up_flush_dcache((uintptr_t)paddr, (uintptr_t)(paddr + len));

  g_msg.header.version = ELE_VERSION_FW;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 4;
  g_msg.header.command = ELE_GET_RNG_REQ;
  g_msg.data[0] = 0;
  g_msg.data[1] = paddr;
  g_msg.data[2] = len;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      /* Invalidate the cache so we can read the result from RAM. */

      up_invalidate_dcache((uintptr_t)paddr,
                           (uintptr_t)(paddr + len));
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_commit(uint32_t info, uint32_t *response)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_COMMIT_REQ;
  g_msg.data[0] = info;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if (response)
    {
      *response = g_msg.data[0];
    }

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_enable_apc(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_ENABLE_APC_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_voltage_change_start(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_VOLTAGE_CHANGE_START_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_voltage_change_finish(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_VOLTAGE_CHANGE_FINISH_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_check_fw_version(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_GET_FW_VERSION_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  return 0;
}
