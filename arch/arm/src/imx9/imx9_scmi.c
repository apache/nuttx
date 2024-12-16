/****************************************************************************
 * arch/arm/src/imx9/imx9_scmi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/crc32.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "hardware/imx95/imx95_memorymap.h"
#include "imx9_mu.h"
#include "imx9_scmi.h"

#ifdef CONFIG_IMX9_SCMI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SM_PLATFORM_MU_INST  0
#define SM_PLATFORM_SMA_ADDR 0
#define SMT_MAX_CHN          3

#define SMT_FREE     (1UL << 0u)
#define SMT_ERROR    (1UL << 1u)
#define SMT_COMP_INT (1UL << 0u)

#define SMT_BUFFER_SIZE    128u /* SMT buffer size */
#define SMT_BUFFER_HEADER  24u  /* SMT buffer header size */
#define SMT_BUFFER_PAYLOAD (SMT_BUFFER_SIZE - SMT_BUFFER_HEADER - 4u)
#define SMT_CRC_NONE       0u /* No CRC */
#define SMT_CRC_XOR        1u /* Simple and fast 32-bit exclusive-OR sum */
#define SMT_CRC_J1850      2u /* J1850 standard CRC */
#define SMT_CRC_CRC32      3u /* CRC32 standard CRC */

/* SCMI protocol message IDs */

#define SCMI_PROTOCOL_BASE    0x10u /* Base protocol */
#define SCMI_PROTOCOL_POWER   0x11u /* Power domain management protocol */
#define SCMI_PROTOCOL_SYS     0x12u /* System power management protocol */
#define SCMI_PROTOCOL_PERF    0x13u /* Performance domain management protocol */
#define SCMI_PROTOCOL_CLOCK   0x14u /* Clock management protocol */
#define SCMI_PROTOCOL_SENSOR  0x15u /* Sensor management protocol */
#define SCMI_PROTOCOL_RESET   0x16u /* Reset domain management protocol */
#define SCMI_PROTOCOL_VOLTAGE 0x17u /* Voltage domain management protocol */
#define SCMI_PROTOCOL_PINCTRL 0x19u /* Pin control protocol */
#define SCMI_PROTOCOL_LMM     0x80u /* LM management protocol */
#define SCMI_PROTOCOL_BBM     0x81u /* BBM management protocol */
#define SCMI_PROTOCOL_CPU     0x82u /* CPU management protocol */
#define SCMI_PROTOCOL_FUSA    0x83u /* FuSa protocol */
#define SCMI_PROTOCOL_MISC    0x84u /* Misc protocol */

/* SCMI clock protocol message IDs */

#define SCMI_MSG_CLOCK_ATTRIBUTES           0x3u /* Get clock attributes */
#define SCMI_MSG_CLOCK_DESCRIBE_RATES       0x4u /* Get clock rate description */
#define SCMI_MSG_CLOCK_RATE_SET             0x5u /* Set clock rate */
#define SCMI_MSG_CLOCK_RATE_GET             0x6u /* Get clock rate */
#define SCMI_MSG_CLOCK_CONFIG_SET           0x7u /* Set clock configuration */
#define SCMI_MSG_CLOCK_CONFIG_GET           0xbu /* Get clock configuration */
#define SCMI_MSG_CLOCK_POSSIBLE_PARENTS_GET 0xcu /* Get all possible parents*/
#define SCMI_MSG_CLOCK_PARENT_SET           0xdu /* Set clock parent */
#define SCMI_MSG_CLOCK_PARENT_GET           0xeu /* Get clock parent */
#define SCMI_MSG_CLOCK_GET_PERMISSIONS      0xfu /* Get clock permissions */

/* SCMI pinctrl protocol message IDs  */

#define SCMI_MSG_PINCTRL_ATTRIBUTES      0x3u /* Get pin attributes */
#define SCMI_MSG_PINCTRL_CONFIG_GET      0x5u /* Get pin configuration */
#define SCMI_MSG_PINCTRL_CONFIG_SET      0x6u /* Set pin configuration */
#define SCMI_MSG_PINCTRL_FUNCTION_SELECT 0x7u /* Select a function for a pin   \
                                               */
#define SCMI_MSG_PINCTRL_REQUEST 0x8u         /* Request a pin */
#define SCMI_MSG_PINCTRL_RELEASE 0x9u         /* Release a pin */

/* SCMI header creation */

#define SCMI_HEADER_MSG(x)      (((x) & 0xffu) << 0u)
#define SCMI_HEADER_TYPE(x)     (((x) & 0x3u) << 8u)
#define SCMI_HEADER_PROTOCOL(x) (((x) & 0xffu) << 10u)
#define SCMI_HEADER_TOKEN(x)    (((x) & 0x3ffu) << 18u)

/* SCMI header extraction */

#define SCMI_HEADER_MSG_EX(x)      (((x) & 0xffu) >> 0u)
#define SCMI_HEADER_TYPE_EX(x)     (((x) & 0x300u) >> 8u)
#define SCMI_HEADER_PROTOCOL_EX(x) (((x) & 0x3fc00u) >> 10u)
#define SCMI_HEADER_TOKEN_EX(x)    (((x) & 0x0ffc0000u) >> 18u)

/* Max payload length */

#define SCMI_PAYLOAD_LEN 100u

/* Calc number of array elements */

#define SCMI_ARRAY(X, Y) ((SCMI_PAYLOAD_LEN - (X)) / sizeof(Y))

#define SCMI_PINCTRL_MAX_NAME      16u
#define SCMI_PINCTRL_MAX_CONFIGS   SCMI_ARRAY(8u, scmi_pin_config_t)
#define SCMI_PINCTRL_MAX_CONFIGS_T SCMI_ARRAY(8u, scmi_pin_config_t)

/* Critical section lock callout */

#define SCMI_A2P_LOCK(lock) if (getpid()) {nxmutex_lock(&mutex);}

/* Critical section unlock callout */

#define SCMI_A2P_UNLOCK(lock) if (getpid()) {nxmutex_unlock(&mutex);}

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  bool valid;
  uint8_t mb_inst;
  uint8_t mb_doorbell;
  uint32_t shared_memory_address;
} smt_chn_config_t;

typedef struct
{
  uint32_t header;
  int32_t status;
} msg_status_t;

typedef struct
{
  uint32_t resv;
  volatile uint32_t channel_status;
  uint32_t impStatus;
  uint32_t imp_crc;
  uint32_t channel_flags;
  uint32_t length;
  uint32_t header;
  uint32_t payload[SMT_BUFFER_PAYLOAD / 4u];
} smt_buf_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static smt_chn_config_t s_smtconfig[SMT_MAX_CHN];
static struct imx9_mudev_s *g_mudev;
static mutex_t mutex = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t imx9_crcxor(const uint32_t *addr, uint32_t size)
{
  const uint32_t *a = addr;
  uint32_t sz       = size;
  uint32_t crc      = 0u;

  /* Loop over data */

  while (sz > 0u)
    {
      /* Update CRC */

      crc ^= *a;
      a++;
      sz--;
    }

  /* Return CRC */

  return crc;
}

static uint32_t imx9_crc32(const uint8_t *addr, uint32_t size)
{
  const uint8_t *a = addr;
  uint32_t sz      = size;
  uint32_t crc     = 0u;

  /* Poly table */

  static uint32_t const crc_table[] = /* clang-format off */

        {
          0x4dbdf21cu, 0x500ae278u, 0x76d3d2d4u, 0x6b64c2b0u,
          0x3b61b38cu, 0x26d6a3e8u, 0x000f9344u, 0x1db88320u,
          0xa005713cu, 0xbdb26158u, 0x9b6b51f4u, 0x86dc4190u,
          0xd6d930acu, 0xcb6e20c8u, 0xedb71064u, 0xf0000000u
        }; /* clang-format on */

  /* Loop over data */

  while (sz > 0u)
    {
      crc = (crc >> 4u) ^
              crc_table[(crc ^ (((uint32_t)(*a)) >> 0u)) & 0x0fu];
      crc = (crc >> 4u) ^
              crc_table[(crc ^ (((uint32_t)(*a)) >> 4u)) & 0x0fu];
      a++;
      sz--;
    }

  /* Return CRC */

  return crc;
}

static smt_buf_t *imx9_smt_smaget(uint32_t smtchannel)
{
  smt_buf_t *rtn = NULL;

  /* Check channel */

  if ((smtchannel < SMT_MAX_CHN) && (s_smtconfig[smtchannel].valid))
    {
      uint8_t db = s_smtconfig[smtchannel].mb_doorbell;

      uint32_t shared_memory_address
          = s_smtconfig[smtchannel].shared_memory_address;

      /* Allow use of internal MU SRAM */

      if (shared_memory_address == 0u)
        {
          shared_memory_address = IMX9_MU5_MUA_BASE + 0x1000u;
        }

      /* Apply channel spacing */

      shared_memory_address += ((uint32_t)db) * SMT_BUFFER_SIZE;

      /* Set return */

      rtn = (smt_buf_t *)shared_memory_address;
    }

  return rtn;
}

static bool imx9_smt_channelfree(uint32_t smtchannel)
{
  const smt_buf_t *buf = (const smt_buf_t *)imx9_smt_smaget(smtchannel);
  bool free_state      = true;

  /* Check for valid buffer */

  if (buf != NULL)
    {
      free_state = ((buf->channel_status & SMT_FREE) != 0u);
    }

  /* Return state */

  return free_state;
}

static int32_t imx9_smt_channelconfig(uint32_t smtchannel, uint8_t mb_inst)
{
  int32_t status = OK;

  if (smtchannel < SMT_MAX_CHN)
    {
      s_smtconfig[smtchannel].mb_inst               = mb_inst;
      s_smtconfig[smtchannel].mb_doorbell           = smtchannel;
      s_smtconfig[smtchannel].shared_memory_address = 0;
      s_smtconfig[smtchannel].valid                 = true;
    }

  else
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

static void *imx9_smt_hdraddrget(uint32_t channel)
{
  void *rtn      = NULL;
  smt_buf_t *buf = imx9_smt_smaget(channel);

  /* Get address of header */

  if (buf != NULL)
    {
      rtn = (void *)&buf->header;
    }

  /* Return address */

  return rtn;
}

static int32_t imx9_smt_tx(uint32_t smtchannel, uint32_t len, bool callee,
                           bool comp_int)
{
  int32_t status = OK;
  uint8_t db     = s_smtconfig[smtchannel].mb_doorbell;
  smt_buf_t *buf = (smt_buf_t *)imx9_smt_smaget(smtchannel);

  /* Check length */

  if (len > (SMT_BUFFER_SIZE - SMT_BUFFER_HEADER))
    {
      status = -EPROTO;
    }

  else
    {
      uint32_t impStatus = buf->impStatus;

      if (callee)
        {
          /* Wait until channel is busy */

          while (imx9_smt_channelfree(smtchannel))
            {
              ; /* Intentional empty while */
            }
        }

      else
        {
          /* Wait until channel is free */

          while (!imx9_smt_channelfree(smtchannel))
            {
              ; /* Intentional empty while */
            }
        }

      /* Fill in reserved */

      buf->resv = 0u;

      /* Completion interrupt if caller wants */

      if (!callee)
        {
          if (comp_int)
            {
              buf->channel_flags = SMT_COMP_INT;
            }

          else
            {
              buf->channel_flags = 0u;
            }
        }

      /* Fill in length */

      buf->length = len;

      /* Calculate CRC */

      switch (impStatus)
        {
        case SMT_CRC_XOR:
          buf->imp_crc =
                imx9_crcxor((const uint32_t *)&buf->header, len / 4u);
          break;
        case SMT_CRC_CRC32:
          buf->imp_crc = imx9_crc32((const uint8_t *)&buf->header, len);
          break;
        default:; /* Intentional empty while */

          break;
        }

      if (callee)
        {
          /* Mark as free */

          buf->channel_status |= SMT_FREE;
        }

      else
        {
          /* Mark as busy */

          buf->channel_status &= ~SMT_FREE;
        }

      /* Trigger GI interrupt */

      imx95_mu_trigger_interrupts(g_mudev, 1 << db);
    }

  /* Return status */

  return status;
}

static int32_t imx9_smt_rx(uint32_t smtchannel, uint32_t *len, bool callee)
{
  int32_t status       = OK;
  const smt_buf_t *buf = (const smt_buf_t *)imx9_smt_smaget(smtchannel);

  /* Check buffer */

  if (buf == NULL)
    {
      status = -EINVAL;
    }

  else
    {
      uint32_t impStatus = buf->impStatus;
      const void *msgrx  = imx9_smt_hdraddrget(smtchannel);

      if (callee)
        {
          /* Wait until channel is busy */

          while (imx9_smt_channelfree(smtchannel))
            {
              ; /* Intentional empty while */
            }
        }

      else
        {
          /* Wait until channel is free */

          while (!imx9_smt_channelfree(smtchannel))
            {
              ; /* Intentional empty while */
            }
        }

      /* Record the length */

      *len = buf->length;

      /* Check the CRC */

      switch (impStatus)
        {
        case SMT_CRC_XOR:
          if (buf->imp_crc
                != imx9_crcxor((const uint32_t *)msgrx, *len / 4u))
            {
              status = -EIO;
            }

          break;
        case SMT_CRC_CRC32:
          if (buf->imp_crc != imx9_crc32((const uint8_t *)msgrx, *len))
            {
              status = -EIO;
            }

          break;
        default:; /* Intentional empty while */

          break;
        }
    }

  /* Return status */

  return status;
}

/* SCMI generics */

static int32_t imx9_scmi_bufinit(uint32_t channel, void **msg)
{
  int32_t status = OK;

  /* Wait until channel is free */

  while (!imx9_smt_channelfree(channel))
    {
      ; /* Intentional empty while */
    }

  /* Get header address */

  *msg = imx9_smt_hdraddrget(channel);
  if (*msg == NULL)
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

static int32_t imx9_scmi_a2ptx(uint32_t channel, uint32_t protocol_id,
                               uint32_t message_id, uint32_t len,
                               uint32_t *header)
{
  int32_t status;
  msg_status_t *msg;

  /* Get transport buffer address */

  msg = (msg_status_t *)imx9_smt_hdraddrget(channel);

  /* Valid channel and buffer? */

  if (msg != NULL)
    {
      static uint32_t s_token = 0u;

      /* Generate header */

      *header = SCMI_HEADER_MSG(message_id)
                | SCMI_HEADER_PROTOCOL(protocol_id)
                | SCMI_HEADER_TYPE(0UL) | SCMI_HEADER_TOKEN(s_token);
      msg->header = *header;

      /* Increment token */

      s_token++;

      /* Send message via transport */

      status = imx9_smt_tx(channel, len, false, false);
    }

  else
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

static int32_t imx9_scmi_a2prx(uint32_t channel, uint32_t min_len,
                               uint32_t header)
{
  int32_t status;
  const msg_status_t *msg;
  uint32_t len;

  /* Get transport buffer address */

  msg = (msg_status_t *)imx9_smt_hdraddrget(channel);

  /* Valid channel and buffer? */

  if (msg != NULL)
    {
      /* Receive message via transport */

      status = imx9_smt_rx(channel, &len, false);

      /* Check header */

      if ((status == OK) && (header != msg->header))
        {
          status = -EPROTO;
        }

      /* Check status */

      if ((status == OK) && (len >= 8u))
        {
          status = msg->status;
        }

      /* Check size */

      if ((status == OK) && (len < min_len))
        {
          status = -EPROTO;
        }
    }

  else
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t imx9_scmi_clockparentget(uint32_t channel, uint32_t clockid,
                                 uint32_t *parentid)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Response message structure */

  typedef struct
  {
    uint32_t header;
    int32_t status;
    uint32_t parentid;
  } msg_rclockd14_t;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t clockid;
      } msg_tclockd14_t;

      msg_tclockd14_t *msgtx = (msg_tclockd14_t *)msg;

      /* Fill in parameters */

      msgtx->clockid = clockid;

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_PARENT_GET,
                               sizeof(msg_tclockd14_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_rclockd14_t), header);
    }

  /* Copy out if no error */

  if (status == OK)
    {
      const msg_rclockd14_t *msgrx = (const msg_rclockd14_t *)msg;

      /* Extract parentid */

      if (parentid != NULL)
        {
          *parentid = msgrx->parentid;
        }
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

int32_t imx9_scmi_clockparentset(uint32_t channel, uint32_t clockid,
                                 uint32_t parentid)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t clockid;
        uint32_t parentid;
      } msg_tclockd13_t;

      msg_tclockd13_t *msgtx = (msg_tclockd13_t *)msg;

      /* Fill in parameters */

      msgtx->clockid  = clockid;
      msgtx->parentid = parentid;

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_PARENT_SET,
                               sizeof(msg_tclockd13_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

int32_t imx9_scmi_clockconfigset(uint32_t channel, uint32_t clockid,
                                 uint32_t attributes,
                                 uint32_t oem_config_val)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t clockid;
        uint32_t attributes;
        uint32_t oem_config_val;
      } msg_tclockd7_t;

      msg_tclockd7_t *msgtx = (msg_tclockd7_t *)msg;

      /* Fill in parameters */

      msgtx->clockid        = clockid;
      msgtx->attributes     = attributes;
      msgtx->oem_config_val = oem_config_val;

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_CONFIG_SET,
                               sizeof(msg_tclockd7_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

int32_t imx9_scmi_clockrateget(uint32_t channel, uint32_t clockid,
                               scmi_clock_rate_t *rate)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Response message structure */

  typedef struct
  {
    uint32_t header;
    int32_t status;
    scmi_clock_rate_t rate;
  } msg_rclockd6_t;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t clockid;
      } msg_tclockd6_t;

      msg_tclockd6_t *msgtx = (msg_tclockd6_t *)msg;

      /* Fill in parameters */

      msgtx->clockid = clockid;

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_RATE_GET,
                               sizeof(msg_tclockd6_t),
                               &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_rclockd6_t), header);
    }

  /* Copy out if no error */

  if (status == OK)
    {
      const msg_rclockd6_t *msgrx = (const msg_rclockd6_t *)msg;

      /* Extract rate */

      if (rate != NULL)
        {
          *rate = msgrx->rate;
        }
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

int32_t imx9_scmi_clockrateset(uint32_t channel, uint32_t clockid,
                               uint32_t flags, scmi_clock_rate_t rate)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t flags;
        uint32_t clockid;
        scmi_clock_rate_t rate;
      } msg_tclockd5_t;

      msg_tclockd5_t *msgtx = (msg_tclockd5_t *)msg;

      /* Fill in parameters */

      msgtx->flags   = flags;
      msgtx->clockid = clockid;
      msgtx->rate    = rate;

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_RATE_SET,
                               sizeof(msg_tclockd5_t),
                               &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

int32_t imx9_scmi_pinctrlconfigset(uint32_t channel, uint32_t identifier,
                                   uint32_t attributes,
                                   const scmi_pin_config_t *configs)
{
  int32_t status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  SCMI_A2P_LOCK(channel);

  /* Init buffer */

  status = imx9_scmi_bufinit(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t identifier;
        uint32_t attributes;
        scmi_pin_config_t configs[SCMI_PINCTRL_MAX_CONFIGS_T];
      } msg_tpinctrld6_t;

      msg_tpinctrld6_t *msgtx = (msg_tpinctrld6_t *)msg;

      /* Fill in parameters */

      msgtx->identifier = identifier;
      msgtx->attributes = attributes;

      memcpy((uint8_t *)&msgtx->configs, (const uint8_t *)configs,
             ((attributes >> 2) * sizeof(scmi_pin_config_t)));

      /* Send message */

      status = imx9_scmi_a2ptx(channel, SCMI_PROTOCOL_PINCTRL,
                               SCMI_MSG_PINCTRL_CONFIG_SET,
                               sizeof(msg_tpinctrld6_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_a2prx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  SCMI_A2P_UNLOCK(channel);

  /* Return status */

  return status;
}

void imx9_scmi_initialize()
{
  /* Configure SMT */

  imx9_smt_channelconfig(SM_PLATFORM_A2P, SM_PLATFORM_MU_INST);

  /* Configure MU */

  g_mudev = imx95_mu_init(5);
}

#endif /* CONFIG_IMX9_SCMI */
