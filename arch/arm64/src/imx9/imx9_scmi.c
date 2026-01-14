/****************************************************************************
 * arch/arm64/src/imx9/imx9_scmi.c
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
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "arm64_internal.h"
#include "hardware/imx95/imx95_memorymap.h"
#include "imx9_mu.h"
#include "imx9_scmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCMI_PLATFORM_MU_INST 0
#define SCMI_TFORM_SMA_ADDR   0
#define SCMI_MAX_CHN          3

#define SCMI_FREE             (1UL << 0u)
#define SCMI_ERROR            (1UL << 1u)
#define SCMI_COMP_INT         (1UL << 0u)

#define SCMI_BUFFER_SIZE      128u /* SCMI buffer size */
#define SCMI_BUFFER_HEADER    24u  /* SCMI buffer header size */
#define SCMI_BUFFER_PAYLOAD   (SCMI_BUFFER_SIZE - SCMI_BUFFER_HEADER - 4u)

/* SCMI protocol message IDs */

#define SCMI_PROTOCOL_BASE    0x10u /* Base protocol */
#define SCMI_PROTOCOL_POWER   0x11u /* Power domain management protocol */
#define SCMI_PROTOCOL_SYS     0x12u /* System power management protocol */
#define SCMI_PROTOCOL_PERF    0x13u /* Performance domain protocol */

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

#define SCMI_MSG_CLOCK_ATTRIBUTES        0x3u /* Get clock attributes */
#define SCMI_MSG_CLOCK_DESCRIBE_RATES    0x4u /* Get clock rate description */
#define SCMI_MSG_CLOCK_RATE_SET          0x5u /* Set clock rate */
#define SCMI_MSG_CLOCK_RATE_GET          0x6u /* Get clock rate */
#define SCMI_MSG_CLOCK_CONFIG_SET        0x7u /* Set clock configuration */
#define SCMI_MSG_CLOCK_CONFIG_GET        0xbu /* Get clock configuration */
#define SCMI_MSG_CLOCK_POSB_PARENTS_GET  0xcu /* Get all possible parents*/
#define SCMI_MSG_CLOCK_PARENT_SET        0xdu /* Set clock parent */
#define SCMI_MSG_CLOCK_PARENT_GET        0xeu /* Get clock parent */
#define SCMI_MSG_CLOCK_GET_PERMISSIONS   0xfu /* Get clock permissions */

/* SCMI pinctrl protocol message IDs  */

#define SCMI_MSG_PINCTRL_ATTRIBUTES      0x3u /* Get pin attributes */
#define SCMI_MSG_PINCTRL_CONFIG_GET      0x5u /* Get pin configuration */
#define SCMI_MSG_PINCTRL_CONFIG_SET      0x6u /* Set pin configuration */
#define SCMI_MSG_PINCTRL_FUNCTION_SELECT 0x7u /* Select a function for a pin */
#define SCMI_MSG_PINCTRL_REQUEST         0x8u /* Request a pin */
#define SCMI_MSG_PINCTRL_RELEASE         0x9u /* Release a pin */

/* SCMI power protocol message IDs and masks */

#define SCMI_MSG_POWER_STATE_SET         0x4U
#define SCMI_MSG_POWER_STATE_GET         0x5U

/* SCMI power domain states */

#define SCMI_POWER_DOMAIN_STATE_ON       0x00000000U
#define SCMI_POWER_DOMAIN_STATE_OFF      0x40000000U

/* SCMI power domain attributes */

#define SCMI_POWER_ATTR_CHANGE(x)        (((x) & 0x1U) << 31U)
#define SCMI_POWER_ATTR_ASYNC(x)         (((x) & 0x1U) << 30U)
#define SCMI_POWER_ATTR_SYNC(x)          (((x) & 0x1U) << 29U)
#define SCMI_POWER_ATTR_CHANGE_REQ(x)    (((x) & 0x1U) << 28U)
#define SCMI_POWER_ATTR_EXT_NAME(x)      (((x) & 0x1U) << 27U)

/* SCMI power state set flags */

#define SCMI_POWER_FLAGS_ASYNC(x)        (((x) & 0x1U) >> 0U)

/* SCMI header creation */

#define SCMI_HEADER_MSG(x)               (((x) & 0xffu) << 0u)
#define SCMI_HEADER_TYPE(x)              (((x) & 0x3u) << 8u)
#define SCMI_HEADER_PROTOCOL(x)          (((x) & 0xffu) << 10u)
#define SCMI_HEADER_TOKEN(x)             (((x) & 0x3ffu) << 18u)

/* SCMI header extraction */

#define SCMI_HEADER_MSG_EX(x)            (((x) & 0xffu) >> 0u)
#define SCMI_HEADER_TYPE_EX(x)           (((x) & 0x300u) >> 8u)
#define SCMI_HEADER_PROTOCOL_EX(x)       (((x) & 0x3fc00u) >> 10u)
#define SCMI_HEADER_TOKEN_EX(x)          (((x) & 0x0ffc0000u) >> 18u)

/* Max payload length */

#define SCMI_PAYLOAD_LEN                 100u

/* Calc number of array elements */

#define SCMI_ARRAY(X, Y) ((SCMI_PAYLOAD_LEN - (X)) / sizeof(Y))

#define SCMI_PINCTRL_MAX_NAME            16u
#define SCMI_PINCTRL_MAX_CONFIGS         SCMI_ARRAY(8u, scmi_pin_config_t)
#define SCMI_PINCTRL_MAX_CONFIGS_T       SCMI_ARRAY(8u, scmi_pin_config_t)

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
  uint32_t payload[SCMI_BUFFER_PAYLOAD / 4u];
} smt_buf_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static smt_chn_config_t g_smt_config[SCMI_MAX_CHN];
static struct imx9_mu_dev_s *g_mudev;
static spinlock_t g_lock = SP_UNLOCKED;
static atomic_t g_token;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static smt_buf_t *imx9_smt_get_buf(uint32_t channel)
{
  smt_buf_t *buf = NULL;

  /* Check channel */

  if ((channel < SCMI_MAX_CHN) && g_smt_config[channel].valid)
    {
      uint8_t db = g_smt_config[channel].mb_doorbell;

      uintptr_t shared_memory_address
          = g_smt_config[channel].shared_memory_address;

      /* Allow use of internal MU SRAM */

      if (shared_memory_address == 0u)
        {
          shared_memory_address = IMX9_MU2_MUA_BASE + 0x1000u;
        }

      /* Apply channel spacing */

      shared_memory_address += db * SCMI_BUFFER_SIZE;

      /* Set return */

      buf = (smt_buf_t *)shared_memory_address;
    }

  return buf;
}

static bool imx9_smt_free_channel(uint32_t channel)
{
  const smt_buf_t *buf = imx9_smt_get_buf(channel);
  bool free_state      = true;

  /* Check for valid buffer */

  if (buf != NULL)
    {
      free_state = ((buf->channel_status & SCMI_FREE) != 0u);
    }

  /* Return state */

  return free_state;
}

static int imx9_smt_config_channel(uint32_t channel, uint8_t mb_inst)
{
  int status = OK;

  if (channel < SCMI_MAX_CHN)
    {
      g_smt_config[channel].mb_inst               = mb_inst;
      g_smt_config[channel].mb_doorbell           = channel;
      g_smt_config[channel].shared_memory_address = 0;
      g_smt_config[channel].valid                 = true;
    }

  else
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

static void *imx9_smt_get_hdraddr(uint32_t channel)
{
  smt_buf_t *smt_buf = imx9_smt_get_buf(channel);

  /* Get address of header */

  if (smt_buf != NULL)
    {
      return &smt_buf->header;
    }

  /* Return address */

  return NULL;
}

static int imx9_smt_tx(uint32_t channel, uint32_t len, bool callee,
                       bool comp_int)
{
  int status = OK;
  uint8_t db     = g_smt_config[channel].mb_doorbell;
  smt_buf_t *buf = imx9_smt_get_buf(channel);

  /* Check length */

  if (len > (SCMI_BUFFER_SIZE - SCMI_BUFFER_HEADER))
    {
      status = -EPROTO;
    }

  else
    {
      if (callee)
        {
          /* Wait until channel is busy */

          while (imx9_smt_free_channel(channel))
            {
              ; /* Intentional empty while */
            }
        }

      else
        {
          /* Wait until channel is free */

          while (!imx9_smt_free_channel(channel))
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
              buf->channel_flags = SCMI_COMP_INT;
            }

          else
            {
              buf->channel_flags = 0u;
            }
        }

      /* Fill in length */

      buf->length = len;

      if (callee)
        {
          /* Mark as free */

          buf->channel_status |= SCMI_FREE;
        }

      else
        {
          /* Mark as busy */

          buf->channel_status &= ~SCMI_FREE;
        }

      /* Trigger GI interrupt */

      imx95_mu_trigger_interrupts(g_mudev, 1 << db);
    }

  /* Return status */

  return status;
}

static int imx9_smt_rx(uint32_t channel, uint32_t *len, bool callee)
{
  int status           = OK;
  const smt_buf_t *buf = imx9_smt_get_buf(channel);

  /* Check buffer */

  if (buf == NULL)
    {
      status = -EINVAL;
    }

  else
    {
      if (callee)
        {
          /* Wait until channel is busy */

          while (imx9_smt_free_channel(channel))
            {
              ; /* Intentional empty while */
            }
        }

      else
        {
          /* Wait until channel is free */

          while (!imx9_smt_free_channel(channel))
            {
              ; /* Intentional empty while */
            }
        }

      /* Record the length */

      *len = buf->length;
    }

  /* Return status */

  return status;
}

/* SCMI generics */

static int imx9_scmi_init_buf(uint32_t channel, void **msg)
{
  int status = OK;

  /* Wait until channel is free */

  while (!imx9_smt_free_channel(channel))
    {
      ; /* Intentional empty while */
    }

  /* Get header address */

  *msg = imx9_smt_get_hdraddr(channel);
  if (*msg == NULL)
    {
      status = -EINVAL;
    }

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_tx
 *
 * Description:
 *   Use scmi a2p channel send date
 *
 * Input Parameters:
 *   channel        - Channel id
 *   protocol_id    - The protocol id will be send
 *   message_id     - The message id will be send
 *   len            - The dare date len
 *   header         - The message header returned will be send
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_scmi_tx(uint32_t channel, uint32_t protocol_id,
                        uint32_t message_id, uint32_t len, uint32_t *header)
{
  int status;
  msg_status_t *msg;

  /* Get transport buffer address */

  msg = imx9_smt_get_hdraddr(channel);

  /* Valid channel and buffer? */

  if (msg != NULL)
    {
      /* Generate header */

      *header = SCMI_HEADER_MSG(message_id)
                | SCMI_HEADER_PROTOCOL(protocol_id)
                | SCMI_HEADER_TYPE(0UL)
                | SCMI_HEADER_TOKEN(atomic_fetch_add(&g_token, 1));
      msg->header = *header;

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

/****************************************************************************
 * Name: imx9_scmi_rx
 *
 * Description:
 *   Use scmi a2p channel receive date
 *
 * Input Parameters:
 *   channel        - Channel id
 *   min_len        - The receive date len
 *   header         - The header returned imx9_scmi_tx
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_scmi_rx(uint32_t channel, uint32_t min_len, uint32_t header)
{
  int status;
  const msg_status_t *msg;
  uint32_t len;

  /* Get transport buffer address */

  msg = imx9_smt_get_hdraddr(channel);

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

/****************************************************************************
 * Name: imx9_scmi_get_clock_parent
 *
 * Description:
 *   Use scmi get clockid's parentid
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   parentid       - Identifier for the parent clock id
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_clock_parent(uint32_t channel, uint32_t clockid,
                               uint32_t *parentid)
{
  irqstate_t flags;
  int status;
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

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

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

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_PARENT_GET,
                               sizeof(msg_tclockd14_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_rclockd14_t), header);
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

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_set_clock_parent
 *
 * Description:
 *   Use scmi set clockid's parentid
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   parentid       - Identifier for the parent clock id
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_parent(uint32_t channel, uint32_t clockid,
                               uint32_t parentid)
{
  irqstate_t flags;
  int status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

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

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_PARENT_SET,
                               sizeof(msg_tclockd13_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_set_clock_config
 *
 * Description:
 *   Use scmi set clock config
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   attributes     - The attributes to be set
 *   oem_config_val - The oem config val
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_config(uint32_t channel, uint32_t clockid,
                               uint32_t attributes, uint32_t oem_config_val)
{
  irqstate_t flags;
  int status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

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

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_CONFIG_SET,
                               sizeof(msg_tclockd7_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_get_clock_rate
 *
 * Description:
 *   Use scmi get clock rate.
 *
 * Input Parameters:
 *   channel - Channel id
 *   clockid - Identifier for the clock
 *   rate    - Returned rate
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_clock_rate(uint32_t channel, uint32_t clockid,
                             scmi_clock_rate_t *rate)
{
  irqstate_t flags;
  int status;
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

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

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

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_RATE_GET,
                               sizeof(msg_tclockd6_t),
                               &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_rclockd6_t), header);
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

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_set_clock_rate
 *
 * Description:
 *   Use scmi set clock rate.
 *
 * Input Parameters:
 *   channel - Channel id
 *   clockid - Identifier for the clock
 *   flags   - Rate flags
 *   rate    - The rate to be set
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_rate(uint32_t channel, uint32_t clockid,
                             uint32_t flags, scmi_clock_rate_t rate)
{
  irqstate_t flag;
  int status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  flag = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

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

      volatile msg_tclockd5_t *msgtx = (msg_tclockd5_t *)msg;

      /* Fill in parameters */

      msgtx->flags   = flags;
      msgtx->clockid = clockid;
      msgtx->rate.lower    = rate.lower;
      msgtx->rate.upper    = rate.upper;

      /* Send message */

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_CLOCK,
                               SCMI_MSG_CLOCK_RATE_SET,
                               sizeof(msg_tclockd5_t),
                               &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flag);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_set_pinctrl_config
 *
 * Description:
 *   Use scmi set pinctrl config.
 *
 * Input Parameters:
 *   channel    - Channel id
 *   identifier - Identifier for the pin or group
 *   attributes - Pin control attributes
 *   configs    - Array of configurations: sorted in numerically
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_pinctrl_config(uint32_t channel, uint32_t identifier,
                                 uint32_t attributes,
                                 const scmi_pin_config_t *configs)
{
  irqstate_t flags;
  int status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t identifier;
        uint32_t function_id;
        uint32_t attributes;
        scmi_pin_config_t configs[SCMI_PINCTRL_MAX_CONFIGS_T];
      } msg_tpinctrld6_t;

      volatile msg_tpinctrld6_t *msgtx = (msg_tpinctrld6_t *)msg;

      /* Fill in parameters */

      msgtx->identifier = identifier;
      msgtx->attributes = attributes;

      memcpy((uint8_t *)&msgtx->configs, (const uint8_t *)configs,
             ((attributes >> 2) * sizeof(scmi_pin_config_t)));

      /* Send message */

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_PINCTRL,
                               SCMI_MSG_PINCTRL_CONFIG_SET,
                               sizeof(msg_tpinctrld6_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_status_t), header);
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_get_power_state
 *
 * Description:
 *   Use scmi set powerdomain state.
 *
 * Input Parameters:
 *   channel   - Channel id
 *   domain    - The power domain id
 *   state     - The returnrd power state
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_power_state(uint32_t channel, uint32_t domain,
                              uint32_t *state)
{
  irqstate_t flags;
  int status;
  uint32_t header;
  void *msg;

  /* Response message structure */

  typedef struct
  {
    uint32_t header;
    int32_t status;
    uint32_t powerstate;
  } msg_tpower5_t;

  /* Acquire lock */

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t domainid;
      } msg_rpower5_t;

       volatile msg_rpower5_t *msgtx = (msg_rpower5_t *)msg;

      /* Fill in parameters */

      msgtx->domainid = domain;

      /* Send message */

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_POWER,
                               SCMI_MSG_POWER_STATE_GET,
                               sizeof(msg_rpower5_t), &header);
    }

  /* Receive response */

  if (status == OK)
    {
      status = imx9_scmi_rx(channel, sizeof(msg_tpower5_t), header);
    }

  /* Copy out if no error */

  if (status == OK)
    {
      const msg_tpower5_t *msgrx = (const msg_tpower5_t *)msg;

      /* Extract parentid */

      if (state != NULL)
        {
          *state = msgrx->powerstate;
        }
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_set_power_state
 *
 * Description:
 *   Use scmi set powerdomain state.
 *
 * Input Parameters:
 *   channel   - Channel id
 *   domain    - The power domain id
 *   state     - The power state
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_power_state(uint32_t channel, uint32_t domain,
                              uint32_t state)
{
  irqstate_t flags;
  int status;
  uint32_t header;
  void *msg;

  /* Acquire lock */

  flags = spin_lock_irqsave(&g_lock);

  /* Init buffer */

  status = imx9_scmi_init_buf(channel, &msg);

  /* Send request */

  if (status == OK)
    {
      /* Request message structure */

      typedef struct
      {
        uint32_t header;
        uint32_t flags;
        uint32_t domainid;
        uint32_t powerstate;
      } msg_rpower4_t;

       volatile msg_rpower4_t *msgtx = (msg_rpower4_t *)msg;

      /* Fill in parameters */

      msgtx->domainid = domain;
      msgtx->powerstate = state;

      /* Send message */

      status = imx9_scmi_tx(channel, SCMI_PROTOCOL_POWER,
                               SCMI_MSG_POWER_STATE_SET,
                               sizeof(msg_rpower4_t), &header);
    }

  /* Release lock */

  spin_unlock_irqrestore(&g_lock, flags);

  /* Return status */

  return status;
}

/****************************************************************************
 * Name: imx9_scmi_initialize
 *
 * Description:
 *   Initialize the scmi
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_scmi_initialize(void)
{
  atomic_set(&g_token , 0);

  /* Configure SCMI */

  imx9_smt_config_channel(SCMI_PLATFORM_A2P, SCMI_PLATFORM_MU_INST);

  /* Configure MU */

  g_mudev = imx95_mu_init(2);
}
