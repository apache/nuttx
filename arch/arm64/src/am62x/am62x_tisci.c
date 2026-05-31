/****************************************************************************
 * arch/arm64/src/am62x/am62x_tisci.c
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

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/am62x_memorymap.h"
#include "hardware/am62x_secure_proxy.h"
#include "hardware/am62x_tisci_proto.h"
#include "am62x_tisci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Host id and secure-proxy thread assignment (Kconfig-tunable; defaults
 * match the A53 HLOS host on a standard AM62x DM/TIFS boardcfg).
 */

#define AM62X_TISCI_HOST_ID    CONFIG_AM62X_TISCI_HOST_ID
#define AM62X_TISCI_RX_THREAD  CONFIG_AM62X_TISCI_RX_THREAD
#define AM62X_TISCI_TX_THREAD  CONFIG_AM62X_TISCI_TX_THREAD

/* Bounded poll: up to ~1 s waiting on a secure-proxy thread credit */

#define AM62X_TISCI_POLL_US    1
#define AM62X_TISCI_POLL_MAX   1000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serialises the single outstanding request/response transaction */

static mutex_t g_tisci_lock = NXMUTEX_INITIALIZER;

/* Rolling sequence number used to match responses to requests */

static uint8_t g_tisci_seq;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am62x_sproxy_wait
 *
 * Description:
 *   Wait until the given secure-proxy thread has a non-zero credit count
 *   (a free slot for TX, or a pending message for RX), or report a thread
 *   error / timeout.
 *
 ****************************************************************************/

static int am62x_sproxy_wait(uintptr_t status_reg)
{
  int retries = AM62X_TISCI_POLL_MAX;
  uint32_t status;

  do
    {
      status = getreg32(status_reg);
      if ((status & AM62X_SEC_PROXY_RT_ERR) != 0)
        {
          return -EIO;
        }

      if ((status & AM62X_SEC_PROXY_RT_CNT_MASK) != 0)
        {
          return OK;
        }

      up_udelay(AM62X_TISCI_POLL_US);
    }
  while (--retries > 0);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: am62x_sproxy_send
 *
 * Description:
 *   Push a message (up to 60 bytes) onto the TX thread.  The message is
 *   zero-padded to the full 15-word window; the write to the final word
 *   (offset 0x3c) commits the transfer to the firmware.
 *
 ****************************************************************************/

static int am62x_sproxy_send(const void *msg, size_t len)
{
  uintptr_t data = AM62X_SEC_PROXY_DATA(AM62X_TISCI_TX_THREAD);
  uintptr_t status = AM62X_SEC_PROXY_RT(AM62X_TISCI_TX_THREAD) +
                     AM62X_SEC_PROXY_RT_STATUS;
  uint32_t buf[AM62X_SEC_PROXY_MSG_WORDS];
  int ret;
  int i;

  if (len > AM62X_SEC_PROXY_MAX_MSG_SIZE)
    {
      return -EINVAL;
    }

  ret = am62x_sproxy_wait(status);
  if (ret < 0)
    {
      return ret;
    }

  memset(buf, 0, sizeof(buf));
  memcpy(buf, msg, len);

  /* Write all 15 words in ascending order; the last (offset 0x3c) commits. */

  for (i = 0; i < AM62X_SEC_PROXY_MSG_WORDS; i++)
    {
      putreg32(buf[i], data + AM62X_SEC_PROXY_DATA_START + i * 4);
    }

  return OK;
}

/****************************************************************************
 * Name: am62x_sproxy_recv
 *
 * Description:
 *   Read a response from the RX thread.  All 15 words are read so the
 *   final-word read marks the message consumed.
 *
 ****************************************************************************/

static int am62x_sproxy_recv(void *msg, size_t len)
{
  uintptr_t data = AM62X_SEC_PROXY_DATA(AM62X_TISCI_RX_THREAD);
  uintptr_t status = AM62X_SEC_PROXY_RT(AM62X_TISCI_RX_THREAD) +
                     AM62X_SEC_PROXY_RT_STATUS;
  uint32_t buf[AM62X_SEC_PROXY_MSG_WORDS];
  int ret;
  int i;

  if (len > AM62X_SEC_PROXY_MAX_MSG_SIZE)
    {
      return -EINVAL;
    }

  ret = am62x_sproxy_wait(status);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < AM62X_SEC_PROXY_MSG_WORDS; i++)
    {
      buf[i] = getreg32(data + AM62X_SEC_PROXY_DATA_START + i * 4);
    }

  memcpy(msg, buf, len);
  return OK;
}

/****************************************************************************
 * Name: am62x_sproxy_drain
 *
 * Description:
 *   Discard any stale message pending on the RX thread before a new
 *   transaction, so an orphaned response cannot desynchronise sequencing.
 *
 ****************************************************************************/

static void am62x_sproxy_drain(void)
{
  uintptr_t data = AM62X_SEC_PROXY_DATA(AM62X_TISCI_RX_THREAD);
  uintptr_t status = AM62X_SEC_PROXY_RT(AM62X_TISCI_RX_THREAD) +
                     AM62X_SEC_PROXY_RT_STATUS;
  int guard = AM62X_SEC_PROXY_MSG_WORDS + 1;
  uint32_t s;
  int i;

  while (guard-- > 0)
    {
      s = getreg32(status);
      if ((s & AM62X_SEC_PROXY_RT_ERR) != 0 ||
          (s & AM62X_SEC_PROXY_RT_CNT_MASK) == 0)
        {
          break;
        }

      for (i = 0; i < AM62X_SEC_PROXY_MSG_WORDS; i++)
        {
          getreg32(data + AM62X_SEC_PROXY_DATA_START + i * 4);
        }
    }
}

/****************************************************************************
 * Name: am62x_tisci_xfer
 *
 * Description:
 *   Run one synchronous TISCI request/response.  Fills the header host, seq,
 *   and ACK-on-processed flag, sends the request, reads the response, and
 *   validates the ACK and matching sequence number.
 *
 ****************************************************************************/

static int am62x_tisci_xfer(struct ti_sci_msg_hdr *req, size_t req_sz,
                            struct ti_sci_msg_hdr *resp, size_t resp_sz)
{
  uint8_t seq;
  int ret;

  ret = nxmutex_lock(&g_tisci_lock);
  if (ret < 0)
    {
      return ret;
    }

  seq = ++g_tisci_seq;
  req->host  = AM62X_TISCI_HOST_ID;
  req->seq   = seq;
  req->flags |= TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

  am62x_sproxy_drain();

  ret = am62x_sproxy_send(req, req_sz);
  if (ret == OK)
    {
      ret = am62x_sproxy_recv(resp, resp_sz);
    }

  nxmutex_unlock(&g_tisci_lock);

  if (ret < 0)
    {
      return ret;
    }

  if ((resp->flags & TI_SCI_FLAG_RESP_GENERIC_ACK) == 0)
    {
      return -EIO;   /* firmware NAK */
    }

  if (resp->seq != seq)
    {
      return -EPROTO;
    }

  return OK;
}

/****************************************************************************
 * Name: am62x_tisci_clkid
 *
 * Description:
 *   Encode a clock id into the 8-bit clk_id / 32-bit clk_id_32 pair used by
 *   the clock messages.
 *
 ****************************************************************************/

static void am62x_tisci_clkid(uint32_t clkid, uint8_t *clk_id,
                              uint32_t *clk_id_32)
{
  if (clkid < TI_SCI_CLOCK_ID_INDIRECT)
    {
      *clk_id    = (uint8_t)clkid;
      *clk_id_32 = 0;
    }
  else
    {
      *clk_id    = TI_SCI_CLOCK_ID_INDIRECT;
      *clk_id_32 = clkid;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int am62x_tisci_get_version(struct am62x_tisci_version_s *ver)
{
  struct ti_sci_msg_hdr req;
  struct ti_sci_msg_resp_version resp;
  int ret;

  memset(&req, 0, sizeof(req));
  req.type = TI_SCI_MSG_VERSION;

  ret = am62x_tisci_xfer(&req, sizeof(req), &resp.hdr, sizeof(resp));
  if (ret < 0)
    {
      return ret;
    }

  if (ver != NULL)
    {
      ver->firmware_revision = resp.firmware_revision;
      ver->abi_major = resp.abi_major;
      ver->abi_minor = resp.abi_minor;
      memcpy(ver->firmware_description, resp.firmware_description,
             sizeof(resp.firmware_description));
      ver->firmware_description[sizeof(resp.firmware_description) - 1] =
        '\0';
    }

  return OK;
}

int am62x_tisci_set_device_state(uint32_t devid, uint8_t state)
{
  struct ti_sci_msg_req_set_device_state req;
  struct ti_sci_msg_hdr resp;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_SET_DEVICE_STATE;
  req.id = devid;
  req.state = state;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_get_device_state(uint32_t devid, uint8_t *current_state)
{
  struct ti_sci_msg_req_get_device_state req;
  struct ti_sci_msg_resp_get_device_state resp;
  int ret;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_GET_DEVICE_STATE;
  req.id = devid;

  ret = am62x_tisci_xfer(&req.hdr, sizeof(req), &resp.hdr, sizeof(resp));
  if (ret == OK && current_state != NULL)
    {
      *current_state = resp.current_state;
    }

  return ret;
}

int am62x_tisci_module_enable(uint32_t devid, uint32_t clkid)
{
  int ret;

  ret = am62x_tisci_set_device_state(devid, MSG_DEVICE_SW_STATE_ON);
  if (ret < 0)
    {
      return ret;
    }

  ret = am62x_tisci_set_device_resets(devid, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = am62x_tisci_clk_enable(devid, clkid);
  return ret;
}

int am62x_tisci_set_device_resets(uint32_t devid, uint32_t resets)
{
  struct ti_sci_msg_req_set_device_resets req;
  struct ti_sci_msg_hdr resp;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_SET_DEVICE_RESETS;
  req.id = devid;
  req.resets = resets;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_set_clock_state(uint32_t devid, uint32_t clkid,
                                uint8_t state)
{
  struct ti_sci_msg_req_set_clock_state req;
  struct ti_sci_msg_hdr resp;
  uint8_t clk_id;
  uint32_t clk_id_32;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_SET_CLOCK_STATE;
  req.dev_id = devid;
  req.request_state = state;
  am62x_tisci_clkid(clkid, &clk_id, &clk_id_32);
  req.clk_id = clk_id;
  req.clk_id_32 = clk_id_32;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_clk_enable(uint32_t devid, uint32_t clkid)
{
  return am62x_tisci_set_clock_state(devid, clkid, MSG_CLOCK_SW_STATE_REQ);
}

int am62x_tisci_set_clock_freq(uint32_t devid, uint32_t clkid,
                               uint64_t freq_hz)
{
  struct ti_sci_msg_req_set_clock_freq req;
  struct ti_sci_msg_hdr resp;
  uint8_t clk_id;
  uint32_t clk_id_32;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_SET_CLOCK_FREQ;
  req.dev_id = devid;
  req.min_freq_hz = freq_hz;
  req.target_freq_hz = freq_hz;
  req.max_freq_hz = freq_hz;
  am62x_tisci_clkid(clkid, &clk_id, &clk_id_32);
  req.clk_id = clk_id;
  req.clk_id_32 = clk_id_32;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_get_clock_freq(uint32_t devid, uint32_t clkid,
                               uint64_t *freq_hz)
{
  struct ti_sci_msg_req_get_clock_freq req;
  struct ti_sci_msg_resp_get_clock_freq resp;
  uint8_t clk_id;
  uint32_t clk_id_32;
  int ret;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_GET_CLOCK_FREQ;
  req.dev_id = devid;
  am62x_tisci_clkid(clkid, &clk_id, &clk_id_32);
  req.clk_id = clk_id;
  req.clk_id_32 = clk_id_32;

  ret = am62x_tisci_xfer(&req.hdr, sizeof(req), &resp.hdr, sizeof(resp));
  if (ret == OK && freq_hz != NULL)
    {
      *freq_hz = resp.freq_hz;
    }

  return ret;
}

int am62x_tisci_irq_set(uint16_t src_id, uint16_t src_index,
                        uint16_t dst_id, uint16_t dst_host_irq)
{
  struct ti_sci_msg_req_manage_irq req;
  struct ti_sci_msg_hdr resp;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_RM_IRQ_SET;
  req.valid_params = MSG_FLAG_DST_ID_VALID | MSG_FLAG_DST_HOST_IRQ_VALID;
  req.src_id = src_id;
  req.src_index = src_index;
  req.dst_id = dst_id;
  req.dst_host_irq = dst_host_irq;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_irq_release(uint16_t src_id, uint16_t src_index,
                            uint16_t dst_id, uint16_t dst_host_irq)
{
  struct ti_sci_msg_req_manage_irq req;
  struct ti_sci_msg_hdr resp;

  memset(&req, 0, sizeof(req));
  req.hdr.type = TI_SCI_MSG_RM_IRQ_RELEASE;
  req.valid_params = MSG_FLAG_DST_ID_VALID | MSG_FLAG_DST_HOST_IRQ_VALID;
  req.src_id = src_id;
  req.src_index = src_index;
  req.dst_id = dst_id;
  req.dst_host_irq = dst_host_irq;

  return am62x_tisci_xfer(&req.hdr, sizeof(req), &resp, sizeof(resp));
}

int am62x_tisci_initialize(void)
{
  struct am62x_tisci_version_s ver;
  int ret;

  ret = am62x_tisci_get_version(&ver);

  if (ret < 0)
    {
      _err("ERROR: TISCI version handshake failed: %d\n", ret);
      return ret;
    }

  _info("AM62x TISCI: firmware \"%s\" rev %u ABI %u.%u\n",
        ver.firmware_description, ver.firmware_revision,
        ver.abi_major, ver.abi_minor);

  return OK;
}

