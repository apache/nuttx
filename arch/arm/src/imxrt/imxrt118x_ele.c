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

#include <stdint.h>
#include <inttypes.h>
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
#    error "IMXRT_ELE_FW_ABS_PATH must be set via CFLAGS (see arch/arm/src/imxrt/Make.defs)"
#  endif

#  define STR2(m) #m
#  define STR(m) STR2(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ele_msg g_msg;

#ifdef CONFIG_IMXRT_ELE_LOAD_FW
/* Embeds the NXP EdgeLock Enclave firmware AHAB container (path from
 * CONFIG_IMXRT_ELE_FW_PATH) directly into the driver image so that
 * imxrt118x_ele_init() can hand its address to the ELE via LOAD_FW at
 * boot time, without depending on where the container sits in the outer
 * AHAB flash layout.
 *
 * The container is proprietary NXP object code; see
 * tools/imxrt1180/fetch_ele_fw.sh for the license note.
 *
 * This is legacy/fallback support: the RT118x ROM can also load the ELE
 * FW automatically from a properly packed AHAB container, in which case
 * CONFIG_IMXRT_ELE_LOAD_FW should be left disabled (the default).
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
  /* Load the ELE firmware ourselves.  The RT118x ROM only ships a minimal
   * ELE stub that understands LOAD_FW, so every other ELE command -
   * RELEASE_RDC, ENABLE_APC - depends on this.  The blob is embedded in
   * this file (from CONFIG_IMXRT_ELE_FW_PATH).
   *
   * This is disabled by default: the RT118x ROM can instead load a
   * properly packed AHAB container containing the ELE FW automatically,
   * before this code ever runs (see tools/imxrt1180/build_flash_image.sh).
   */

  int ret;

  ret = imxrt118x_ele_load_fw((uint32_t)(uintptr_t)imxrt118x_ele_fw);
  if (ret < 0)
    {
      _err("ELE: LOAD_FW failed (%d) - aborting handshake\n", ret);
    }
#endif

  /* Verify that some ELE firmware - whichever way it got there - is
   * actually running, by reading back its version.
   */

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

int imxrt118x_ele_check_fw_version(void)
{
  uint32_t raw;
  uint32_t sha1;

  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_GET_FW_VERSION_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) != ELE_OK)
    {
      _err("ELE: GET_FW_VERSION failed (status 0x%" PRIx32 ") - "
           "ELE firmware may not be loaded\n", g_msg.data[0]);
      return -EIO;
    }

  /* data[1] = FW version word: bits 23:16 major, bits 15:4 minor/patch,
   *           bits 3:0 variant, bit 31 "dirty build" flag.
   * data[2] = first 4 bytes of the FW build's commit SHA1.
   */

  raw = g_msg.data[1];
  sha1 = g_msg.data[2];

  _info("ELE: firmware version %" PRIu32 ".%" PRIu32 ".%" PRIu32
        "%s, sha1 %08" PRIx32 "\n",
        (raw >> 16) & 0xff, (raw >> 4) & 0xfff, raw & 0xf,
        (raw & 0x80000000) ? " (dirty)" : "", sha1);

  return 0;
}
