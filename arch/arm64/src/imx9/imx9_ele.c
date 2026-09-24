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

#include <nuttx/arch.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

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

#define ELE_RNG_TIMEOUT_US    5000
#define ELE_POLL_SLEEP_US     100

/* A key generation is the slowest call, a few hundred milliseconds. */

#define ELE_REPLY_TIMEOUT_US  2000000

/* The enclave reports how much it means to write before writing it, so this
 * only has to be larger than a key store, not exactly its size.
 */

/* A key store is a master blob plus a chunk per key group. */

#define ELE_BLOB_SLOTS        8
#define ELE_BLOB_SIZE         2048
#define ELE_BLOB_MASTER_ID    0xffffffff
#define ELE_CHUNK_GET_SUCCESS 0xca3bb3acu

#define ELE_STORAGE_FAILURE   0x29
#define ELE_EXPORT_STATUS_SUCCESS 0xba2cc2abu
#define ELE_RNG_SLEEP_US      100
#define ELE_TRNG_STATUS_READY 0x3
#define ELE_CSAL_STATUS_READY 0x2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ele_msg msg;

/* Arrives mid-command, so it cannot be allocated at the point of need. */

static uint8_t g_ele_blob_data[ELE_BLOB_SLOTS][ELE_BLOB_SIZE]
  aligned_data(ARMV8A_DCACHE_LINESIZE);

struct ele_blob_s
{
  uint32_t id;
  uint32_t id_ext;
  uint32_t len;
  bool     valid;
  bool     pending;
};

static struct ele_blob_s g_ele_blob[ELE_BLOB_SLOTS];

/* The command payloads, as the enclave's message interface lays them out. */

begin_packed_struct struct ele_session_open_s
{
  uint8_t  rsvd1;
  uint8_t  interrupt_num;
  uint16_t rsvd2;
  uint8_t  priority;
  uint8_t  op_mode;
  uint16_t rsvd3;
} end_packed_struct;

begin_packed_struct struct ele_key_store_open_s
{
  uint32_t session_handle;
  uint32_t key_store_id;
  uint32_t auth_nonce;
  uint16_t rsvd1;
  uint8_t  flags;
  uint8_t  rsvd2;
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_key_mgmt_open_s
{
  uint32_t key_store_handle;
  uint32_t msbi;
  uint32_t msbo;
  uint8_t  flags;
  uint8_t  reserved[3];
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_generate_key_s
{
  uint32_t key_mgmt_handle;
  uint32_t key_id;
  uint16_t public_key_size;
  uint16_t key_group;
  uint16_t key_type;
  uint16_t key_size;
  uint32_t key_lifetime;
  uint32_t key_usage;
  uint32_t permitted_algo;
  uint32_t key_lifecycle;
  uint8_t  flags;
  uint8_t  reserved[3];
  uint32_t public_key_addr;
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_sig_gen_open_s
{
  uint32_t key_store_handle;
  uint32_t msbi;
  uint32_t msbo;
  uint8_t  flags;
  uint8_t  reserved[3];
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_sign_s
{
  uint32_t sig_gen_handle;
  uint32_t key_identifier;
  uint32_t message_addr;
  uint32_t signature_addr;
  uint32_t message_size;
  uint16_t signature_size;
  uint8_t  flags;
  uint8_t  reserved;
  uint32_t scheme_id;
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_storage_open_s
{
  uint32_t session_handle;
  uint32_t msbi;
  uint32_t msbo;
  uint8_t  flags;
  uint8_t  reserved[3];
  uint32_t crc;
} end_packed_struct;

begin_packed_struct struct ele_master_import_s
{
  uint32_t storage_handle;
  uint32_t key_store_addr;
  uint32_t key_store_size;
} end_packed_struct;

struct ele_trng_state
{
  uint8_t  trng_state;
  uint8_t  csal_state;
  uint16_t reserved;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void imx9_ele_service_request(struct ele_msg *req);

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

static int imx9_ele_wait_tx(int channel)
{
  uint32_t waited;

  for (waited = 0; !(getreg32(ELE_MU_TSR) & (1 << channel));
       waited += ELE_POLL_SLEEP_US)
    {
      if (waited >= ELE_REPLY_TIMEOUT_US)
        {
          return -ETIMEDOUT;
        }

      up_udelay(ELE_POLL_SLEEP_US);
    }

  return 0;
}

static int imx9_ele_sendmsg(struct ele_msg *msg_ptr)
{
  if (imx9_ele_wait_tx(0) < 0)
    {
      return -ETIMEDOUT;
    }

  putreg32(msg_ptr->header.data, ELE_MU_TR(0));

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      int tx_channel = i % ELE_TR_NUM;

      if (imx9_ele_wait_tx(tx_channel) < 0)
        {
          return -ETIMEDOUT;
        }

      putreg32(msg_ptr->data[i - 1], ELE_MU_TR(tx_channel));
    }

  return 0;
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

static int imx9_ele_wait_rx(int channel)
{
  uint32_t waited;

  for (waited = 0; !(getreg32(ELE_MU_RSR) & (1 << channel));
       waited += ELE_POLL_SLEEP_US)
    {
      if (waited >= ELE_REPLY_TIMEOUT_US)
        {
          return -ETIMEDOUT;
        }

      up_udelay(ELE_POLL_SLEEP_US);
    }

  return 0;
}

static int imx9_ele_receivemsg_raw(struct ele_msg *msg_ptr)
{
  /* An enclave that never answers must not take the caller with it. */

  if (imx9_ele_wait_rx(0) < 0)
    {
      return -ETIMEDOUT;
    }

  msg_ptr->header.data = getreg32(ELE_MU_RR(0));

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      int rx_channel = (i) % ELE_RR_NUM;

      if (imx9_ele_wait_rx(rx_channel) < 0)
        {
          return -ETIMEDOUT;
        }

      msg_ptr->data[i - 1] = getreg32(ELE_MU_RR(rx_channel));
    }

  return 0;
}

/****************************************************************************
 * Name: imx9_ele_receivemsg
 *
 * Description:
 *   Wait for the reply to the command this side sent. ELE_RESP_TAG is a
 *   reply, ELE_CMD_TAG is the enclave asking something of its own, which is
 *   answered here because the reply does not come until it is.
 *
 ****************************************************************************/

static void imx9_ele_receivemsg(struct ele_msg *msg_ptr)
{
  for (; ; )
    {
      if (imx9_ele_receivemsg_raw(msg_ptr) < 0)
        {
          /* Callers read the response word, so it must not be stale. */

          _err("ELE did not answer command 0x%02x\n",
               msg_ptr->header.command);
          msg_ptr->data[0] = ELE_STORAGE_FAILURE;
          return;
        }

      if (msg_ptr->header.tag == ELE_RESP_TAG)
        {
          return;
        }

      imx9_ele_service_request(msg_ptr);
    }
}

/****************************************************************************
 * Name: imx9_ele_buffer_pa
 *
 * Description:
 *   Physical address of a buffer the ELE will read or write.  The enclave
 *   addresses memory physically while cache maintenance takes the virtual
 *   address, so a caller holding one of them cannot supply the other.
 *
 * Returned Value:
 *   The physical address, or zero if the buffer is not mapped.
 *
 ****************************************************************************/

static uintptr_t imx9_ele_buffer_pa(void *va)
{
#ifdef CONFIG_ARCH_USE_MMU
  return up_addrenv_va_to_pa(va);
#else
  /* Without translation the virtual address is the physical one. */

  return (uintptr_t)va;
#endif
}

/****************************************************************************
 * Name: imx9_ele_service_request
 *
 * Description:
 *   Answer a request the enclave sent while a command of this side's was in
 *   flight. Having been asked to sync a key store it asks back where to put
 *   the blob, then whether it was kept, and the command that provoked those
 *   does not reply until they are answered. An unrecognised request is
 *   refused, so it fails rather than hangs.
 *
 ****************************************************************************/

static int imx9_ele_blob_slot(uint32_t id, uint32_t id_ext, bool allocate)
{
  int free_slot = -1;
  int i;

  for (i = 0; i < ELE_BLOB_SLOTS; i++)
    {
      if (g_ele_blob[i].valid &&
          g_ele_blob[i].id == id && g_ele_blob[i].id_ext == id_ext)
        {
          return i;
        }

      if (!g_ele_blob[i].valid && free_slot < 0)
        {
          free_slot = i;
        }
    }

  return allocate ? free_slot : -1;
}

static void imx9_ele_service_request(struct ele_msg *req)
{
  /* A kilobyte does not belong on the caller's stack. */

  static struct ele_msg rsp;
  uint32_t handle = req->data[0];
  uintptr_t paddr;
  int slot;

  memset(&rsp, 0, sizeof(rsp));
  rsp.header.version = req->header.version;
  rsp.header.tag = ELE_RESP_TAG;
  rsp.header.command = req->header.command;

  rsp.header.size = 2;
  rsp.data[0] = ELE_STORAGE_FAILURE;

  switch (req->header.command)
    {
      case ELE_STORAGE_EXPORT_START:

      /* data[1] is the size it will write; a smaller buffer overruns. */

        slot = imx9_ele_blob_slot(ELE_BLOB_MASTER_ID, 0, true);

        if (slot < 0 || req->data[1] > ELE_BLOB_SIZE)
          {
            break;
          }

        paddr = imx9_ele_buffer_pa(g_ele_blob_data[slot]);
        if (paddr == 0)
          {
            break;
          }

        g_ele_blob[slot].id = ELE_BLOB_MASTER_ID;
        g_ele_blob[slot].id_ext = 0;
        g_ele_blob[slot].len = req->data[1];
        g_ele_blob[slot].pending = true;

      /* The enclave writes behind the cache. */

        up_flush_dcache((uintptr_t)g_ele_blob_data[slot],
                        (uintptr_t)g_ele_blob_data[slot] + ELE_BLOB_SIZE);

        rsp.header.size = 4;
        rsp.data[0] = handle;
        rsp.data[1] = ELE_OK;
        rsp.data[2] = (uint32_t)paddr;
        break;

      case ELE_STORAGE_CHUNK_EXPORT:

        /* One key group. data[1] is its size, data[2] and data[3] name it. */

        slot = imx9_ele_blob_slot(req->data[2], req->data[3], true);

        if (slot < 0 || req->data[1] > ELE_BLOB_SIZE)
          {
            break;
          }

        paddr = imx9_ele_buffer_pa(g_ele_blob_data[slot]);
        if (paddr == 0)
          {
            break;
          }

        g_ele_blob[slot].id = req->data[2];
        g_ele_blob[slot].id_ext = req->data[3];
        g_ele_blob[slot].len = req->data[1];
        g_ele_blob[slot].pending = true;

        up_flush_dcache((uintptr_t)g_ele_blob_data[slot],
                        (uintptr_t)g_ele_blob_data[slot] + ELE_BLOB_SIZE);

        rsp.header.size = 3;
        rsp.data[0] = ELE_OK;
        rsp.data[1] = (uint32_t)paddr;
        break;

      case ELE_STORAGE_EXPORT_FINISH:

      /* Only the pieces this export wrote are settled by it. */

        for (slot = 0; slot < ELE_BLOB_SLOTS; slot++)
          {
            if (!g_ele_blob[slot].pending)
              {
                continue;
              }

            g_ele_blob[slot].pending = false;

            if (req->data[1] != ELE_EXPORT_STATUS_SUCCESS)
              {
                g_ele_blob[slot].len = 0;
                continue;
              }

            up_invalidate_dcache((uintptr_t)g_ele_blob_data[slot],
                                 (uintptr_t)g_ele_blob_data[slot] +
                                 ELE_BLOB_SIZE);

            g_ele_blob[slot].valid = true;
          }

        rsp.header.size = 3;
        rsp.data[0] = handle;
        rsp.data[1] = ELE_OK;
        break;

      case ELE_STORAGE_CHUNK_GET:

      /* Not having it is the ordinary first boot, not a failure. */

        slot = imx9_ele_blob_slot(req->data[1], req->data[2], false);

        if (slot < 0)
          {
            break;
          }

        paddr = imx9_ele_buffer_pa(g_ele_blob_data[slot]);
        if (paddr == 0)
          {
            break;
          }

        up_flush_dcache((uintptr_t)g_ele_blob_data[slot],
                        (uintptr_t)g_ele_blob_data[slot] + ELE_BLOB_SIZE);

        rsp.header.size = 4;
        rsp.data[0] = g_ele_blob[slot].len;
        rsp.data[1] = (uint32_t)paddr;
        rsp.data[2] = ELE_OK;
        break;

      case ELE_STORAGE_CHUNK_GET_DONE:
        rsp.header.size = 2;
        rsp.data[0] = ELE_OK;
        break;

      default:
        break;
    }

  imx9_ele_sendmsg(&rsp);
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
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
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

  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
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
  uintptr_t key_pa;
  uintptr_t ctx_pa;

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

  key_pa = imx9_ele_buffer_pa(key);
  ctx_pa = imx9_ele_buffer_pa(ctx);

  if (key_pa == 0 || ctx_pa == 0)
    {
      _err("Buffer is not mapped\n");
      return -EFAULT;
    }

  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 7;
  msg.header.command = ELE_DERIVE_KEY_REQ;
  msg.data[0] = upper_32_bits((ulong)key_pa);
  msg.data[1] = lower_32_bits((ulong)key_pa);
  msg.data[2] = upper_32_bits((ulong)ctx_pa);
  msg.data[3] = lower_32_bits((ulong)ctx_pa);
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

  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_GET_EVENTS_REQ;

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
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_FWD_LIFECYCLE_UP_REQ;
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

int imx9_ele_auth_oem_ctnr(unsigned long ctnr_addr, uint32_t *response)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 3;
  msg.header.command = ELE_OEM_CNTN_AUTH_REQ;
  msg.data[0] = upper_32_bits(ctnr_addr);
  msg.data[1] = lower_32_bits(ctnr_addr);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);
  if (response)
    {
      *response = msg.data[0];
    }

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imx9_ele_release_container(uint32_t *response)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_RELEASE_CONTAINER_REQ;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (response)
    {
      *response = msg.data[0];
    }

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imx9_ele_verify_image(uint32_t img_id, uint32_t *response)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_VERIFY_IMAGE_REQ;
  msg.data[0] = 1 << img_id;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (response)
    {
      *response = msg.data[0];
    }

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imx9_ele_start_rng(void)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_START_RNG_REQ;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imx9_ele_get_trng_state(void)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_GET_TRNG_STATE_REQ;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      struct ele_trng_state *ele_trng =
        (struct ele_trng_state *)(msg.data + 1);

      if (ele_trng->trng_state != ELE_TRNG_STATUS_READY ||
          ele_trng->csal_state != ELE_CSAL_STATUS_READY)
        {
          /* Ensure imx9_ele_start_rng() was called earlier or we will
           * end up here.
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

int imx9_ele_get_random(void *buf, size_t len)
{
  uint16_t counter = 0;
  uint16_t max_tries = ELE_RNG_TIMEOUT_US / ELE_RNG_SLEEP_US;
  uintptr_t paddr;

  if (buf == NULL || len == 0)
    {
      _err("Wrong input parameters!\n");
      return -EINVAL;
    }

  /* A neighbour sharing an end cache line would lose its contents. */

  if (!IS_ALIGNED((uintptr_t)buf, ARMV8A_DCACHE_LINESIZE) ||
      !IS_ALIGNED(len, ARMV8A_DCACHE_LINESIZE))
    {
      _err("Buffer is not a whole number of cache lines\n");
      return -EINVAL;
    }

  paddr = imx9_ele_buffer_pa(buf);

  /* The address travels in a single 32-bit message word. */

  if (paddr == 0 || paddr > UINT32_MAX - len)
    {
      _err("Buffer is not mapped, or is beyond the ELE address range\n");
      return -EFAULT;
    }

  while ((imx9_ele_get_trng_state() != 0))
    {
      if (counter > max_tries)
        {
          _err("Timed out after %hu iterations!\n", counter);
          return -EBUSY;
        }

      usleep(ELE_RNG_SLEEP_US);
      counter++;
    }

  /* Cache maintenance takes the virtual address; the ELE takes the
   * physical one.
   */

  up_flush_dcache((uintptr_t)buf, (uintptr_t)buf + len);

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 4;
  msg.header.command = ELE_GET_RNG_REQ;
  msg.data[0] = 0;
  msg.data[1] = (uint32_t)paddr;
  msg.data[2] = len;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      /* Invalidate the cache so we can read the result from RAM. */

      up_invalidate_dcache((uintptr_t)buf, (uintptr_t)buf + len);
      return 0;
    }

  return -EIO;
}

int imx9_ele_commit(uint32_t info, uint32_t *response)
{
  msg.header.version = ELE_VERSION;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_COMMIT_REQ;
  msg.data[0] = info;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (response)
    {
      *response = msg.data[0];
    }

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

/****************************************************************************
 * Name: imx9_ele_session_open
 *
 * Description:
 *   Open an ELE session. Every key store service hangs off one of these, and
 *   the enclave holds it until it is closed.
 *
 * Output Parameters:
 *   session - handle for the opened session
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ele_sab_init
 *
 * Description:
 *   Start the enclave's security services. Every key store command answers
 *   "not ready" until this has been done once.
 *
 * Output Parameters:
 *   rsp - the raw ELE response word, or NULL
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ele_update_crc
 *
 * Description:
 *   Fill the trailing crc word of a key store command. The enclave refuses
 *   these commands with rating 0xb9 without it. It is the exclusive or of
 *   every word of the message, the header included, except the crc word
 *   itself, which is the last one.
 *
 ****************************************************************************/

static void imx9_ele_update_crc(struct ele_msg *msg_ptr)
{
  uint32_t *words = (uint32_t *)msg_ptr;
  uint32_t crc = 0;
  unsigned int i;

  for (i = 0; i < msg_ptr->header.size - 1; i++)
    {
      crc ^= words[i];
    }

  msg_ptr->data[msg_ptr->header.size - 2] = crc;
}

int imx9_ele_sab_init(uint32_t *rsp)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1;
  msg.header.command = ELE_SAB_INIT_REQ;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

int imx9_ele_session_open_rsp(uint32_t *session, uint32_t *rsp)
{
  struct ele_session_open_s cmd;

  if (session == NULL)
    {
      return -EINVAL;
    }

  memset(&cmd, 0, sizeof(cmd));

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_SESSION_OPEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  *session = msg.data[1];
  return 0;
}

int imx9_ele_session_open(uint32_t *session)
{
  return imx9_ele_session_open_rsp(session, NULL);
}

/****************************************************************************
 * Name: imx9_ele_session_close
 *
 * Description:
 *   Close a session opened by imx9_ele_session_open().
 *
 * Input Parameters:
 *   session - the session handle
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_session_close(uint32_t session)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_SESSION_CLOSE_REQ;
  msg.data[0] = session;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

/****************************************************************************
 * Name: imx9_ele_key_store_open
 *
 * Description:
 *   Open a key store, creating it if asked. A key store is where a generated
 *   key lives, and the private half has no command that returns it.
 *
 * Input Parameters:
 *   session  - an open session
 *   id       - caller-chosen key store identifier
 *   nonce    - authentication nonce for the store
 *   flags    - ELE_KEY_STORE_FLAG_*, none of them to load an existing store
 *
 * Output Parameters:
 *   store - handle for the opened key store
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_key_store_open_rsp(uint32_t session, uint32_t id,
                                uint32_t nonce, uint8_t flags,
                                uint32_t *store, uint32_t *rsp)
{
  struct ele_key_store_open_s cmd;

  if (store == NULL)
    {
      return -EINVAL;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.session_handle = session;
  cmd.key_store_id = id;
  cmd.auth_nonce = nonce;

  /* Asking for SYNC is asking the enclave to hand the store back. */

  cmd.flags = flags;

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_KEY_STORE_OPEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  *store = msg.data[1];
  return 0;
}

int imx9_ele_key_store_open(uint32_t session, uint32_t id, uint32_t nonce,
                            uint8_t flags, uint32_t *store)
{
  return imx9_ele_key_store_open_rsp(session, id, nonce, flags, store,
                                     NULL);
}

/****************************************************************************
 * Name: imx9_ele_key_store_close
 *
 * Description:
 *   Close a key store opened by imx9_ele_key_store_open().
 *
 ****************************************************************************/

int imx9_ele_key_store_close(uint32_t store)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_KEY_STORE_CLOSE_REQ;
  msg.data[0] = store;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

/****************************************************************************
 * Name: imx9_ele_key_mgmt_open / imx9_ele_key_mgmt_close
 *
 * Description:
 *   Open a key management service on a key store. Generating a key needs one
 *   of these, and it is another handle to close.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_key_mgmt_open(uint32_t store, uint32_t *mgmt, uint32_t *rsp)
{
  struct ele_key_mgmt_open_s cmd;

  if (mgmt == NULL)
    {
      return -EINVAL;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.key_store_handle = store;

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_KEY_MGMT_OPEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  *mgmt = msg.data[1];
  return 0;
}

int imx9_ele_key_mgmt_close(uint32_t mgmt)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_KEY_MGMT_CLOSE_REQ;
  msg.data[0] = mgmt;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

/****************************************************************************
 * Name: imx9_ele_generate_key
 *
 * Description:
 *   Generate a key pair inside the enclave. The public half is written to
 *   the caller's buffer; the private half stays in the key store and there
 *   is no command that returns it. Withholding the export usage is what
 *   makes that true rather than merely unimplemented.
 *
 * Input Parameters:
 *   mgmt      - an open key management handle
 *   key_type  - ELE_KEY_TYPE_ECC_PAIR_SECP_R1 and friends
 *   key_bits  - key size in bits
 *   algo      - the one algorithm this key is permitted to perform
 *   lifecycle - the device lifecycle the key may be used in
 *   pubkey    - buffer for the public half, cache line aligned and sized
 *   pubkey_len- its length
 *
 * Output Parameters:
 *   key_id - identifier of the generated key
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_generate_key(uint32_t mgmt, uint16_t key_type,
                          uint16_t key_bits, uint32_t algo,
                          uint32_t lifecycle,
                          void *pubkey, size_t pubkey_len,
                          uint32_t *key_id, uint32_t *rsp)
{
  struct ele_generate_key_s cmd;

  uintptr_t paddr;

  if (pubkey == NULL || key_id == NULL)
    {
      return -EINVAL;
    }

  /* A neighbour sharing an end cache line would lose its contents. */

  if (!IS_ALIGNED((uintptr_t)pubkey, ARMV8A_DCACHE_LINESIZE) ||
      !IS_ALIGNED(pubkey_len, ARMV8A_DCACHE_LINESIZE))
    {
      return -EINVAL;
    }

  paddr = imx9_ele_buffer_pa(pubkey);
  if (paddr == 0 || paddr > UINT32_MAX - pubkey_len)
    {
      return -EFAULT;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.key_mgmt_handle = mgmt;
  cmd.public_key_size = (uint16_t)pubkey_len;
  cmd.key_group = ELE_KEY_GROUP_PERSISTENT;
  cmd.key_type = key_type;
  cmd.key_size = key_bits;
  cmd.key_lifetime = ELE_KEY_LIFETIME_PERSISTENT;

  /* Sign only, and no export: the private half has no way out. */

  cmd.key_usage = ELE_KEY_USAGE_SIGN_HASH;
  cmd.permitted_algo = algo;
  cmd.key_lifecycle = lifecycle;

  /* The lifetime is intent; this is what writes the key to the store. */

  cmd.flags = ELE_KEY_FLAG_STRICT;
  cmd.public_key_addr = (uint32_t)paddr;

  up_flush_dcache((uintptr_t)pubkey, (uintptr_t)pubkey + pubkey_len);

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_GENERATE_KEY_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  up_invalidate_dcache((uintptr_t)pubkey, (uintptr_t)pubkey + pubkey_len);

  *key_id = msg.data[1];
  return 0;
}

/****************************************************************************
 * Name: imx9_ele_sig_gen_open / imx9_ele_sig_gen_close
 *
 * Description:
 *   Open a signature generation service on a key store. Signing needs one of
 *   these, and it is another handle to close.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_sig_gen_open(uint32_t store, uint32_t *svc, uint32_t *rsp)
{
  struct ele_sig_gen_open_s cmd;

  if (svc == NULL)
    {
      return -EINVAL;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.key_store_handle = store;

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_SIG_GEN_OPEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  *svc = msg.data[1];
  return 0;
}

int imx9_ele_sig_gen_close(uint32_t svc)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_SIG_GEN_CLOSE_REQ;
  msg.data[0] = svc;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

/****************************************************************************
 * Name: imx9_ele_sign
 *
 * Description:
 *   Sign with a key held in the key store. The key is named by identifier,
 *   never handed over, so this is the only way to use it.
 *
 * Input Parameters:
 *   svc     - an open signature generation handle
 *   key_id  - identifier returned by imx9_ele_generate_key()
 *   algo    - the algorithm, which must be the one the key permits
 *   digest  - true if input is already hashed, false to let the enclave hash
 *   in      - message or digest, cache line aligned
 *   inlen   - its length
 *   out     - buffer for the signature, cache line aligned
 *   outlen  - its length, 2 * key bytes + 1 for ECDSA
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_sign(uint32_t svc, uint32_t key_id, uint32_t algo, bool digest,
                  void *in, size_t inlen, void *out, size_t outlen,
                  uint32_t *rsp)
{
  struct ele_sign_s cmd;

  uintptr_t in_pa;
  uintptr_t out_pa;
  size_t in_span;
  size_t out_span;

  if (in == NULL || out == NULL || inlen == 0 || outlen == 0)
    {
      return -EINVAL;
    }

  /* A signature is never a whole number of cache lines. */

  if (!IS_ALIGNED((uintptr_t)in, ARMV8A_DCACHE_LINESIZE) ||
      !IS_ALIGNED((uintptr_t)out, ARMV8A_DCACHE_LINESIZE))
    {
      return -EINVAL;
    }

  in_span = ALIGN_UP(inlen, ARMV8A_DCACHE_LINESIZE);
  out_span = ALIGN_UP(outlen, ARMV8A_DCACHE_LINESIZE);

  in_pa = imx9_ele_buffer_pa(in);
  out_pa = imx9_ele_buffer_pa(out);
  if (in_pa == 0 || out_pa == 0 ||
      in_pa > UINT32_MAX - inlen || out_pa > UINT32_MAX - outlen)
    {
      return -EFAULT;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.sig_gen_handle = svc;
  cmd.key_identifier = key_id;
  cmd.message_addr = (uint32_t)in_pa;
  cmd.signature_addr = (uint32_t)out_pa;
  cmd.message_size = (uint32_t)inlen;
  cmd.signature_size = (uint16_t)outlen;
  cmd.flags = digest ? ELE_SIG_FLAG_INPUT_DIGEST
                     : ELE_SIG_FLAG_INPUT_MESSAGE;
  cmd.scheme_id = algo;

  up_flush_dcache((uintptr_t)in, (uintptr_t)in + in_span);
  up_flush_dcache((uintptr_t)out, (uintptr_t)out + out_span);

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_SIGNATURE_GEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  up_invalidate_dcache((uintptr_t)out, (uintptr_t)out + out_span);

  return 0;
}

/****************************************************************************
 * Name: imx9_ele_poll_msg
 *
 * Description:
 *   Receive a message the enclave sent on its own initiative, rather than a
 *   reply to something this side asked for. Persisting a key store works
 *   that way round: the enclave asks the host to store the blob. Unlike
 *   imx9_ele_receivemsg() this gives up instead of spinning forever, because
 *   a wrong guess about whether a message is coming would otherwise hang the
 *   caller.
 *
 * Input Parameters:
 *   timeout_us - how long to wait for the header
 *
 * Output Parameters:
 *   msg_ptr - the received message
 *
 * Returned Value:
 *   Zero (OK) on success, -ETIMEDOUT if nothing arrived.
 *
 ****************************************************************************/

int imx9_ele_poll_msg(struct ele_msg *msg_ptr, uint32_t timeout_us)
{
  uint32_t waited;
  int i;

  if (msg_ptr == NULL)
    {
      return -EINVAL;
    }

  for (waited = 0; !(getreg32(ELE_MU_RSR) & 1); waited += ELE_POLL_SLEEP_US)
    {
      if (waited >= timeout_us)
        {
          return -ETIMEDOUT;
        }

      up_udelay(ELE_POLL_SLEEP_US);
    }

  msg_ptr->header.data = getreg32(ELE_MU_RR(0));

  for (i = 1; i < msg_ptr->header.size; i++)
    {
      int rx_channel = i % ELE_RR_NUM;

      for (waited = 0; !(getreg32(ELE_MU_RSR) & (1 << rx_channel));
           waited += ELE_POLL_SLEEP_US)
        {
          if (waited >= timeout_us)
            {
              return -ETIMEDOUT;
            }

          up_udelay(ELE_POLL_SLEEP_US);
        }

      msg_ptr->data[i - 1] = getreg32(ELE_MU_RR(rx_channel));
    }

  return 0;
}

/****************************************************************************
 * Name: imx9_ele_storage_open / imx9_ele_storage_close
 *
 * Description:
 *   Open a storage session, without which the enclave will not sync a key
 *   store. It is the only part of that exchange this side initiates: the
 *   import and export that follow are requests the enclave sends.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ele_storage_open(uint32_t session, uint32_t *storage, uint32_t *rsp)
{
  struct ele_storage_open_s cmd;

  if (storage == NULL)
    {
      return -EINVAL;
    }

  memset(&cmd, 0, sizeof(cmd));
  cmd.session_handle = session;

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_STORAGE_OPEN_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));
  imx9_ele_update_crc(&msg);

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  if ((msg.data[0] & 0xff) != ELE_OK)
    {
      return -EIO;
    }

  *storage = msg.data[1];
  return 0;
}

int imx9_ele_storage_close(uint32_t storage)
{
  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_STORAGE_CLOSE_REQ;
  msg.data[0] = storage;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}

/****************************************************************************
 * Name: imx9_ele_blob_get / imx9_ele_blob_put
 *
 * Description:
 *   The key store the enclave asked to have persisted, and the one to hand
 *   back on the next boot. Where it is kept is not this driver's business.
 *   A store is a master blob plus a chunk per key group, so it is a slot at
 *   a time.
 *
 ****************************************************************************/

uint32_t imx9_ele_blob_get(unsigned slot, uint32_t *id, uint32_t *id_ext,
                           const void **blob)
{
  if (slot >= ELE_BLOB_SLOTS || !g_ele_blob[slot].valid)
    {
      return 0;
    }

  if (id != NULL)
    {
      *id = g_ele_blob[slot].id;
    }

  if (id_ext != NULL)
    {
      *id_ext = g_ele_blob[slot].id_ext;
    }

  if (blob != NULL)
    {
      *blob = g_ele_blob_data[slot];
    }

  return g_ele_blob[slot].len;
}

int imx9_ele_blob_put(uint32_t id, uint32_t id_ext, const void *blob,
                      uint32_t len)
{
  int slot;

  if (blob == NULL || len == 0 || len > ELE_BLOB_SIZE)
    {
      return -EINVAL;
    }

  slot = imx9_ele_blob_slot(id, id_ext, true);
  if (slot < 0)
    {
      return -ENOSPC;
    }

  memcpy(g_ele_blob_data[slot], blob, len);
  g_ele_blob[slot].id = id;
  g_ele_blob[slot].id_ext = id_ext;
  g_ele_blob[slot].len = len;
  g_ele_blob[slot].valid = true;
  g_ele_blob[slot].pending = false;
  return 0;
}

/****************************************************************************
 * Name: imx9_ele_storage_master_import
 *
 * Description:
 *   Give the enclave back the master blob of a key store it exported before.
 *   Chunks it asks for by id; the master is the one piece this side pushes,
 *   and without it the open that follows answers UNKNOWN_ID.
 *
 * Input Parameters:
 *   storage - an open storage session
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure, -ENOENT if this side is holding no master blob.
 *
 ****************************************************************************/

int imx9_ele_storage_master_import(uint32_t storage, uint32_t *rsp)
{
  struct ele_master_import_s cmd;

  uintptr_t paddr;
  int slot = imx9_ele_blob_slot(ELE_BLOB_MASTER_ID, 0, false);

  if (slot < 0)
    {
      return -ENOENT;
    }

  paddr = imx9_ele_buffer_pa(g_ele_blob_data[slot]);
  if (paddr == 0)
    {
      return -EFAULT;
    }

  up_flush_dcache((uintptr_t)g_ele_blob_data[slot],
                  (uintptr_t)g_ele_blob_data[slot] + ELE_BLOB_SIZE);

  memset(&cmd, 0, sizeof(cmd));
  cmd.storage_handle = storage;
  cmd.key_store_addr = (uint32_t)paddr;
  cmd.key_store_size = g_ele_blob[slot].len;

  msg.header.version = ELE_VERSION_FW;
  msg.header.tag = ELE_CMD_TAG;
  msg.header.size = 1 + (sizeof(cmd) / sizeof(uint32_t));
  msg.header.command = ELE_STORAGE_IMPORT_REQ;
  memcpy(msg.data, &cmd, sizeof(cmd));

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if (rsp != NULL)
    {
      *rsp = msg.data[0];
    }

  return ((msg.data[0] & 0xff) == ELE_OK) ? 0 : -EIO;
}
