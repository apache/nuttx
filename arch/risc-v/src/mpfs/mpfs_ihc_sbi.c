/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_ihc_sbi.c
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
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>
#include "hardware/mpfs_sysreg.h"
#include "hardware/mpfs_ihc.h"
#include "hardware/mpfs_ihc_sbi.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_connected_hart_ints_b;
static uint32_t g_connected_harts_b;
static uint32_t g_connected_hart_ints_c;
static uint32_t g_connected_harts_c;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_ihc_sbi_parse_incoming_hartid
 *
 * Description:
 *   This function determines the remote hart id and whether the remote is
 *   acking to a message or not.
 *
 * Input Parameters:
 *   mhartid - My hart id.  If remote owns multiple harts, this is the
 *             mhartid base on the context, not necessarily the actual
 *             mhartid.
 *   is_ack  - Boolean that is set true if an ack has been found
 *
 * Returned Value:
 *   Remote hart id
 *
 ****************************************************************************/

static uint32_t mpfs_ihc_sbi_parse_incoming_hartid(uint32_t mhartid,
                                                   bool *is_ack)
{
  uint32_t hart_id             = 0;
  uint32_t return_hart_id      = UNDEFINED_HART_ID;
  uint32_t msg_avail           = getreg32(MPFS_IHC_MSG_AVAIL(mhartid));
  uint32_t connected_harts     = g_connected_harts_b;
  uint32_t connected_hart_ints = g_connected_hart_ints_b;

  if (mhartid == (CONTEXTA_HARTID + 1))
    {
      connected_harts     = g_connected_harts_c;
      connected_hart_ints = g_connected_hart_ints_c;
    }

  while (hart_id < MPFS_NUM_HARTS)
    {
      if (connected_harts & (1 << hart_id))
        {
          uint32_t test_int = (1 << ((hart_id * 2) + 1));

          if (msg_avail & test_int)
            {
              if (connected_hart_ints & test_int)
                {
                  return_hart_id = hart_id;
                  *is_ack = true;
                  break;
                }
            }

          test_int = (1 << (hart_id * 2));

          if (msg_avail & test_int)
            {
              if (((connected_hart_ints & test_int) == test_int))
                {
                  return_hart_id = hart_id;
                  *is_ack = false;
                  break;
                }
            }
        }

      hart_id++;
    }

  return return_hart_id;
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_context_to_remote_hart_id
 *
 * Description:
 *   This function determines the remote hart id with the provided context
 *   handle.
 *
 * Input Parameters:
 *   channel  - Enum that describes the channel used.
 *
 * Returned Value:
 *   Remote hart id
 *
 ****************************************************************************/

static uint32_t mpfs_ihc_sbi_context_to_remote_hart_id(ihc_channel_t channel)
{
  uint32_t harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
  uint32_t hart             = UNDEFINED_HART_ID;
  uint32_t hart_idx         = 0;

  hart_idx = 0;

  while (hart_idx < MPFS_NUM_HARTS)
    {
      if (harts_in_context & (1 << hart_idx))
        {
          if (channel != IHC_CHANNEL_TO_CONTEXTC)
            {
              hart = hart_idx;
              break;
            }
          else
            {
              hart = hart_idx + 1;
              break;
            }
        }

      hart_idx++;
    }

  DEBUGASSERT(hart != UNDEFINED_HART_ID);

  return hart;
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_context_to_local_hart_id
 *
 * Description:
 *   Maps the context to a local hart id.
 *
 * Input Parameters:
 *   channel   - Enum that describes the channel used.
 *
 * Returned Value:
 *   Local hart id
 *
 ****************************************************************************/

static uint32_t mpfs_ihc_sbi_context_to_local_hart_id(ihc_channel_t channel)
{
  uint32_t hart             = UNDEFINED_HART_ID;
  uint32_t hart_idx         = 0;
  uint32_t harts_in_context = LIBERO_SETTING_CONTEXT_A_HART_EN;
  uint32_t hart_next        = 0;

  DEBUGASSERT(channel > IHC_CHANNEL_TO_HART4);

  hart_idx = 0;
  while (hart_idx < MPFS_NUM_HARTS)
    {
      if (harts_in_context & (1 << hart_idx))
        {
          hart = hart_idx;

          if (channel != IHC_CHANNEL_TO_CONTEXTC)
            {
              break;
            }

          /* Pick the 2nd channel for IHC_CHANNEL_TO_CONTEXTC */

          if (hart_next == 1)
            {
              break;
            }

          hart_next = 1;
        }

      hart_idx++;
    }

  DEBUGASSERT(hart < MPFS_NUM_HARTS);

  return hart;
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_message_present_handler
 *
 * Description:
 *   This function fills up a structure that gets into the remote end, such
 *   as Linux kernel.  The structure may contain data from the
 *   MPFS_IHC_MSG_IN -register, or in case of an ack, just the irq_type.
 *
 * Input Parameters:
 *   message  - Pointer for storing data, must not be NULL
 *   mhartid  - The primary hart id of a set of hartids.  Not necessarily
 *              the absolute mhartid if multiple harts are incorporated
 *              within the remote (eg. Linux kernel used on 2 harts).
 *   rhartid  - Remote hart id
 *   is_ack   - Boolean indicating an ack message
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_sbi_message_present_handler(uint32_t *message,
                                                 uint32_t mhartid,
                                                 uint32_t rhartid,
                                                 bool is_ack)
{
  struct ihc_sbi_rx_msg_s *msg;
  uintptr_t message_ihc     = (uintptr_t)MPFS_IHC_MSG_IN(mhartid, rhartid);
  uint32_t message_size_ihc = getreg32(MPFS_IHC_MSG_SIZE(mhartid, rhartid));

  msg = (struct ihc_sbi_rx_msg_s *)message;

  if (is_ack)
    {
      msg->irq_type = ACK_IRQ;

      /* msg->ihc_msg content doesn't matter here */
    }
  else
    {
      msg->irq_type = MP_IRQ;
      msg->ihc_msg = *(struct mpfs_ihc_msg_s *)message_ihc;
    }

    DEBUGASSERT(sizeof(msg->ihc_msg) >= message_size_ihc);
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_rx_message
 *
 * Description:
 *   This function determines the remote hart id with the provided context
 *   handle.
 *
 * Input Parameters:
 *   rhartid  - Remote hart id
 *   mhartid  - Context hart id, not necessarily the absolute mhartid but
 *              rather, the primary hartid of the set of harts.
 *   is_ack   - Boolean indicating an ack message
 *   msg      - For storing data, could be NULL
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_sbi_rx_message(uint32_t rhartid, uint32_t mhartid,
                                    bool is_ack, uint32_t *msg)
{
  uint32_t ctrl_reg = getreg32(MPFS_IHC_CTRL(mhartid, rhartid));

  if (is_ack)
    {
      if (mhartid == CONTEXTB_HARTID)
        {
          DEBUGPANIC();
        }
      else
        {
          /* This path is meant for the OpenSBI vendor extension only */

          DEBUGASSERT(msg != NULL);
          mpfs_ihc_sbi_message_present_handler(msg, mhartid, rhartid,
                                               is_ack);
        }
    }
  else if (MP_MESSAGE_PRESENT == (ctrl_reg & MP_MASK))
    {
      /* Check if we have a message */

      if (mhartid == CONTEXTB_HARTID)
        {
          DEBUGPANIC();
        }
      else
        {
          /* This path is meant for the OpenSBI vendor extension only */

          DEBUGASSERT(msg != NULL);
          mpfs_ihc_sbi_message_present_handler(msg, mhartid, rhartid,
                                               is_ack);
        }

      /* Set MP to 0. Note this generates an interrupt on the other hart
       * if it has RMPIE bit set in the control register
       */

      volatile uint32_t temp = getreg32(MPFS_IHC_CTRL(mhartid, rhartid)) &
                                        ~MP_MASK;

      /* Check if ACKIE_EN is set */

      if (temp & ACKIE_EN)
        {
          temp |= ACK_INT;
        }

      putreg32(temp, MPFS_IHC_CTRL(mhartid, rhartid));
    }
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_message_present_indirect_isr
 *
 * Description:
 *   This is used by OpenSBI.  This is handled as an OpenSBI extension.
 *   The S-mode kernel uses this in its extended OpenSBI vendor call.
 *
 * Input Parameters:
 *   channel   - Enum that describes the channel used.
 *   msg       - The msg pointer from sbi_trap_regs->a1 register for data
 *               exchange.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_ihc_sbi_message_present_indirect_isr(ihc_channel_t channel,
                                               uint32_t *msg)
{
  bool is_ack;
  uint32_t mhartid     = mpfs_ihc_sbi_context_to_local_hart_id(channel);
  uint32_t origin_hart = mpfs_ihc_sbi_parse_incoming_hartid(mhartid,
                                                            &is_ack);

  if (origin_hart != UNDEFINED_HART_ID)
    {
      /* Process incoming packet */

      mpfs_ihc_sbi_rx_message(origin_hart, mhartid, is_ack, msg);

      if (is_ack)
        {
          /* Clear the ack */

          modifyreg32(MPFS_IHC_CTRL(mhartid, origin_hart), ACK_CLR, 0);
        }
    }
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_local_context_init
 *
 * Description:
 *   This initializes the local context by zeroing the CTRL register and
 *   applying proper values for the g_connected_harts_b|c and
 *   g_connected_hart_ints_b|c globals. These globals are used to map the
 *   harts and contexts properly.
 *
 * Input Parameters:
 *   hart_to_configure   - Hart to be configured
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_sbi_local_context_init(uint32_t hart_to_configure)
{
  uint32_t rhartid = 0;

  DEBUGASSERT(hart_to_configure < MPFS_NUM_HARTS);

  while (rhartid < MPFS_NUM_HARTS)
    {
      putreg32(0, MPFS_IHC_CTRL(hart_to_configure, rhartid));
#ifdef CONFIG_MPFS_IHC_TWO_RPMSG_CHANNELS
      /* This is the Context C */

      if ((hart_to_configure + 1) < MPFS_NUM_HARTS)
        {
          putreg32(0, MPFS_IHC_CTRL(hart_to_configure + 1, rhartid));
        }

#endif
      rhartid++;
    }

  g_connected_harts_b     = (HSS_HART_MASK | (1 << CONTEXTB_HARTID));
  g_connected_hart_ints_b = IHCIA_CONTEXTA_INTS;
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_local_remote_config
 *
 * Description:
 *   This enables the required interrupts via two registers.
 *
 * Input Parameters:
 *   hart_to_configure   - Hart to be configured
 *   rhartid             - The associated remote hart id
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_sbi_local_remote_config(uint32_t hart_to_configure,
                                             uint32_t rhartid)
{
  /* Set-up enables in concentrator */

  putreg32(IHCIA_CONTEXTA_INTS, MPFS_IHC_INT_EN(hart_to_configure));

  /* OpenSBI extension may configure 2x consecutive harts */

#ifdef CONFIG_MPFS_IHC_TWO_RPMSG_CHANNELS
  if ((hart_to_configure + 1) < MPFS_NUM_HARTS)
    {
      putreg32(IHCIA_CONTEXTA2_INTS,
               MPFS_IHC_INT_EN(hart_to_configure + 1));
    }
#endif

  modifyreg32(MPFS_IHC_CTRL(hart_to_configure, rhartid), 0, MPIE_EN |
                            ACKIE_EN);

  /* OpenSBI extension may configure 2x consecutive harts */

#ifdef CONFIG_MPFS_IHC_TWO_RPMSG_CHANNELS
  if ((hart_to_configure + 1) < MPFS_NUM_HARTS)
    {
      modifyreg32(MPFS_IHC_CTRL(hart_to_configure + 1, rhartid + 1), 0,
                                MPIE_EN | ACKIE_EN);
    }
#endif
}

/****************************************************************************
 * Name: mpfs_ihc_sbi_tx_message
 *
 * Description:
 *   Sends and notifies the remote hart of an incoming message.
 *
 * Input Parameters:
 *   channel   - Enum that describes the channel used.
 *   message   - Pointer to the message to be sent
 *
 * Returned Value:
 *   OK on success, busy in case of error
 *
 ****************************************************************************/

static int mpfs_ihc_sbi_tx_message(ihc_channel_t channel, uint32_t *message)
{
  uint32_t i;
  uint32_t mhartid      = mpfs_ihc_sbi_context_to_local_hart_id(channel);
  uint32_t rhartid      = mpfs_ihc_sbi_context_to_remote_hart_id(channel);
  uint32_t message_size = getreg32(MPFS_IHC_MSG_SIZE(mhartid, rhartid));
  uint32_t ctrl_reg;
  uint32_t retries      = 10000;

  DEBUGASSERT(message_size <= IHC_MAX_MESSAGE_SIZE);

  /* Check if the system is busy.  All we can try is wait. */

  do
    {
      ctrl_reg = getreg32(MPFS_IHC_CTRL(mhartid, rhartid));
    }
  while ((ctrl_reg & (RMP_MESSAGE_PRESENT | ACK_INT)) && --retries);

  /* Return if RMP bit 1 indicating busy */

  if (RMP_MESSAGE_PRESENT == (ctrl_reg & RMP_MASK))
    {
      return -EBUSY;
    }
  else if (ACK_INT == (ctrl_reg & ACK_INT_MASK))
    {
      return -EBUSY;
    }
  else
    {
      /* Fill the buffer */

      for (i = 0; i < message_size; i++)
        {
          putreg32(message[i], MPFS_IHC_MSG_OUT(mhartid, rhartid) + i * 4);
        }

      /* Set the MP bit. This will notify other of incoming hart message */

      modifyreg32(MPFS_IHC_CTRL(mhartid, rhartid), 0, RMP_MESSAGE_PRESENT);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_ihc_sbi_ecall_handler
 *
 * Description:
 *   This is sbi_platform_operations / vendor_ext_provider ecall handler.
 *   Related Linux ecalls end up here.
 *
 * Input Parameters:
 *   funcid          - SBI_EXT_IHC_CTX_INIT, SBI_EXT_IHC_SEND or
 *                     SBI_EXT_IHC_RECEIVE.  Others are invalid.
 *   remote_channel  - The remote we're communicating with
 *   message_ptr     - Local storage for data exchange
 *
 * Returned Value:
 *   OK on success, a negated error code otherwise
 *
 ****************************************************************************/

int mpfs_ihc_sbi_ecall_handler(unsigned long funcid, uint32_t remote_channel,
                               uint32_t *message_ptr)
{
  uint32_t mhartid;
  uint32_t rhartid;
  int result = OK;

  /* Check the channel is bound to a valid context.  Context C indicates
   * 2x rpmsg channels.
   */

#ifdef CONFIG_MPFS_IHC_TWO_RPMSG_CHANNELS
  if ((remote_channel < IHC_CHANNEL_TO_CONTEXTA) ||
      (remote_channel > IHC_CHANNEL_TO_CONTEXTC))
    {
      return -EINVAL;
    }
#else
  if ((remote_channel < IHC_CHANNEL_TO_CONTEXTA) ||
      (remote_channel > IHC_CHANNEL_TO_CONTEXTB))
    {
      return -EINVAL;
    }
#endif

  switch (funcid)
    {
      case SBI_EXT_IHC_CTX_INIT:

        /* mhartid = Linux hart id, rhartid = NuttX hart id */

        mhartid = mpfs_ihc_sbi_context_to_local_hart_id(remote_channel);
        rhartid = mpfs_ihc_sbi_context_to_remote_hart_id(remote_channel);
        if (remote_channel == IHC_CHANNEL_TO_CONTEXTB)
          {
            mpfs_ihc_sbi_local_context_init(mhartid);
          }

        /* If we have 2x rpmsg channels, the 2nd channel needs its ints */

#ifdef CONFIG_MPFS_IHC_TWO_RPMSG_CHANNELS
        if (remote_channel == IHC_CHANNEL_TO_CONTEXTC)
          {
           g_connected_hart_ints_c = IHCIA_CONTEXTA2_INTS;
           g_connected_harts_c = (HSS_HART_MASK | (1 << CONTEXTC_HARTID));
          }
#endif

        /* Remote means the NuttX here */

        if (remote_channel == IHC_CHANNEL_TO_CONTEXTB)
          {
            mpfs_ihc_sbi_local_remote_config(mhartid, rhartid);
          }
        break;

      case SBI_EXT_IHC_SEND:
        result = mpfs_ihc_sbi_tx_message(remote_channel, message_ptr);
        break;

      case SBI_EXT_IHC_RECEIVE:
        mpfs_ihc_sbi_message_present_indirect_isr(remote_channel,
                                                  message_ptr);
        break;

      default:
        result = -ENOTSUP;
  }

  return result;
}
