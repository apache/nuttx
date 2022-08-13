/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_ihc.c
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
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <nuttx/rptun/openamp.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/list.h>

#include <arch/board/board.h>
#include "hardware/mpfs_sysreg.h"
#include "hardware/mpfs_ihc.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MPFS_IHC_DEBUG
#  define ihcerr  _err
#  define ihcwarn _warn
#  define ihcinfo _info
#else
#  define ihcerr(x...)
#  define ihcwarn(x...)
#  define ihcinfo(x...)
#endif

/* Slave, IHC_CHANNEL_SIDE_A is master */

#define IHC_CHANNEL_SIDE_B

/* This name is picked by the master */

#define MPFS_RPTUN_PING_EPT_NAME "rpmsg-amp-demo-channel"

/* rptun initialization names */

#define MPFS_RPTUN_CPU_NAME      "mpfs-ihc"
#define MPFS_RPTUN_SHMEM_NAME    "mpfs-shmem"

/* Vring configuration parameters */

#define VRING_SHMEM              0xa2410000  /* Vring shared memory start */
#define VRING0_DESCRIPTORS       0xa2400000  /* Vring0 descriptor area    */
#define VRING1_DESCRIPTORS       0xa2408000  /* Vring1 descriptor area    */
#define VRINGS                   0x02        /* Number of vrings          */
#define VRING_ALIGN              0x1000      /* Vring alignment           */
#define VRING_NR                 256         /* Number of descriptors     */
#define VRING_SIZE               512         /* Size of one descriptor    */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpfs_rptun_shmem_s
{
  volatile uintptr_t         base;
  volatile unsigned int      seqs;
  volatile unsigned int      seqm;
  struct rptun_rsc_s         rsc;
  bool                       master_up;
};

struct mpfs_rptun_dev_s
{
  struct list_node           node;
  struct rptun_dev_s         rptun;
  rptun_callback_t           callback;
  void                      *arg;
  bool                       master;
  unsigned int               seq;
  struct mpfs_rptun_shmem_s *shmem;
  struct simple_addrenv_s    addrenv[VRINGS];
  char                       cpuname[RPMSG_NAME_SIZE + 1];
  char                       shmemname[RPMSG_NAME_SIZE + 1];
};

struct mpfs_queue_table_s
{
  void *data;
};

enum mpfs_irq_type_e
{
  MP_IRQ  = 0x0,
  ACK_IRQ = 0x1,
};

struct mpfs_ihc_msg_s
{
  uint32_t msg[IHC_MAX_MESSAGE_SIZE];
};

/* Used to store information for the remote via ecall (eg. Linux) */

struct ihc_sbi_rx_msg_s
{
  uint8_t irq_type;
  struct mpfs_ihc_msg_s ihc_msg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *mpfs_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *mpfs_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s
*mpfs_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct rptun_rsc_s *mpfs_rptun_get_resource(struct rptun_dev_s *dev);
static bool mpfs_rptun_is_autostart(struct rptun_dev_s *dev);
static bool mpfs_rptun_is_master(struct rptun_dev_s *dev);
static int mpfs_rptun_start(struct rptun_dev_s *dev);
static int mpfs_rptun_stop(struct rptun_dev_s *dev);
static int mpfs_rptun_notify(struct rptun_dev_s *dev, uint32_t notifyid);
static int mpfs_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a bug workaround.  This increases the image size in purpose.  The
 * problem currently is, that the vendor based bootloader (HSS) in
 * combination with u-boot / Linux kernel will not boot if this NuttX image
 * is small.  In the linker script we KEEP the filler_area section so that no
 * compiler will optimize it away.  This will be removed once the root cause
 * has been found out.  This is not needed if NuttX is the bootloader, not
 * HSS.
 */

uint8_t unused_filler[0x80000] __attribute__((section(".filler_area")));

static struct rpmsg_endpoint        g_mpgs_echo_ping_ept;
static struct mpfs_queue_table_s    g_mpfs_virtqueue_table[VRINGS];
static struct mpfs_rptun_shmem_s    g_shmem;
static struct rpmsg_device         *g_mpfs_rpmsg_device;
static struct rpmsg_virtio_device  *g_mpfs_virtio_device;

static sem_t  g_mpfs_ack_sig       = SEM_INITIALIZER(0);
static sem_t  g_mpfs_rx_sig        = SEM_INITIALIZER(0);
static struct list_node g_dev_list = LIST_INITIAL_VALUE(g_dev_list);

static uint32_t g_connected_hart_ints;
static uint32_t g_connected_harts;
static int      g_vq_idx;
static int      g_plic_irq;
static bool     g_rptun_initialized;

const uint32_t ihcia_remote_harts[MPFS_NUM_HARTS] =
{
  IHCIA_H0_REMOTE_HARTS,
  IHCIA_H1_REMOTE_HARTS,
  IHCIA_H2_REMOTE_HARTS,
  IHCIA_H3_REMOTE_HARTS,
  IHCIA_H4_REMOTE_HARTS
};

const uint32_t ihcia_remote_hart_ints[MPFS_NUM_HARTS] =
{
  IHCIA_H0_REMOTE_HARTS_INTS,
  IHCIA_H1_REMOTE_HARTS_INTS,
  IHCIA_H2_REMOTE_HARTS_INTS,
  IHCIA_H3_REMOTE_HARTS_INTS,
  IHCIA_H4_REMOTE_HARTS_INTS
};

static const struct rptun_ops_s g_mpfs_rptun_ops =
{
  .get_cpuname       = mpfs_rptun_get_cpuname,
  .get_firmware      = mpfs_rptun_get_firmware,
  .get_addrenv       = mpfs_rptun_get_addrenv,
  .get_resource      = mpfs_rptun_get_resource,
  .is_autostart      = mpfs_rptun_is_autostart,
  .is_master         = mpfs_rptun_is_master,
  .start             = mpfs_rptun_start,
  .stop              = mpfs_rptun_stop,
  .notify            = mpfs_rptun_notify,
  .register_callback = mpfs_rptun_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_ihc_parse_incoming_hartid
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

static uint32_t mpfs_ihc_parse_incoming_hartid(uint32_t mhartid,
                                               bool *is_ack)
{
  uint32_t hart_id        = 0;
  uint32_t return_hart_id = UNDEFINED_HART_ID;
  uint32_t msg_avail      = getreg32(MPFS_IHC_MSG_AVAIL(mhartid));

  while (hart_id < MPFS_NUM_HARTS)
    {
      if (g_connected_harts & (1 << hart_id))
        {
          uint32_t test_int = (1 << ((hart_id * 2) + 1));

          if (msg_avail & test_int)
            {
              if (g_connected_hart_ints & test_int)
                {
                  return_hart_id = hart_id;
                  *is_ack = true;
                  break;
                }
            }

          test_int = (1 << (hart_id * 2));

          if (msg_avail & test_int)
            {
              if (((g_connected_hart_ints & test_int) == test_int))
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
 * Name: mpfs_ihc_context_to_remote_hart_id
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

static uint32_t mpfs_ihc_context_to_remote_hart_id(ihc_channel_t channel)
{
  uint32_t harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
  uint32_t hart             = UNDEFINED_HART_ID;
  uint32_t hart_idx         = 0;

  if (channel <= IHC_CHANNEL_TO_HART4)
    {
      hart = channel;
    }
  else
    {
      DEBUGASSERT(LIBERO_SETTING_CONTEXT_A_HART_EN > 0);
      DEBUGASSERT(LIBERO_SETTING_CONTEXT_B_HART_EN > 0);

      /* Determine context we are in */

      if (channel == IHC_CHANNEL_TO_CONTEXTA)
        {
          harts_in_context = LIBERO_SETTING_CONTEXT_A_HART_EN;
        }
      else
        {
          harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
        }

      hart_idx = 0;

      while (hart_idx < MPFS_NUM_HARTS)
        {
          if (harts_in_context & (1 << hart_idx))
            {
              hart = hart_idx;
              break;
            }

          hart_idx++;
        }
    }

  DEBUGASSERT(hart != UNDEFINED_HART_ID);

  return hart;
}

/****************************************************************************
 * Name: mpfs_ihc_context_to_local_hart_id
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

static uint32_t mpfs_ihc_context_to_local_hart_id(ihc_channel_t channel)
{
  uint32_t hart             = UNDEFINED_HART_ID;
  uint32_t hart_idx         = 0;
  uint32_t harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
  uint64_t mhartid          = riscv_mhartid();

  /* If we are sending to a Context, assume we are a Context.
   * i.e. HSS bootloader will not send directly to a context.
   */

  if (channel <= IHC_CHANNEL_TO_HART4)
    {
      hart = (uint32_t)mhartid;
    }
  else
    {
      if (channel == IHC_CHANNEL_TO_CONTEXTA)
        {
          /* We are context B */

          harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
        }
      else
        {
          /* We are context A */

          harts_in_context = LIBERO_SETTING_CONTEXT_A_HART_EN;
        }

      hart_idx = 0;
      while (hart_idx < MPFS_NUM_HARTS)
        {
          if (harts_in_context & (1 << hart_idx))
            {
              hart = hart_idx;
              break;
            }

          hart_idx++;
        }
    }

  DEBUGASSERT(hart < MPFS_NUM_HARTS);

  return hart;
}

/****************************************************************************
 * Name: mpfs_ihc_rx_handler
 *
 * Description:
 *   This handles the received information and either lets the vq to proceed
 *   via posting g_mpfs_ack_sig, or lets the mpfs_rptun_thread() run as it
 *   waits for the g_mpfs_rx_sig.  virtqueue_notification() cannot be called
 *   from the interrupt context, thus the thread that will perform it.
 *
 * Input Parameters:
 *   message   - Pointer to the incoming message
 *   is_ack    - Boolean indicating whether an ack is received
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_rx_handler(uint32_t *message, bool is_ack)
{
  if (is_ack)
    {
      /* Received the ack */

      nxsem_post(&g_mpfs_ack_sig);
    }
  else
    {
      g_vq_idx = (message[0] >> 16);

      DEBUGASSERT(g_vq_idx < VRINGS);

      nxsem_post(&g_mpfs_rx_sig);
    }
}

/****************************************************************************
 * Name: mpfs_ihc_message_present_handler
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

static void mpfs_ihc_message_present_handler(uint32_t *message,
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
 * Name: mpfs_ihc_rx_message
 *
 * Description:
 *   This function determines the remote hart id with the provided context
 *   handle.
 *
 * Input Parameters:
 *   channel  - Enum that describes the channel used.
 *   mhartid  - Context hart id, not necessarily the absolute mhartid but
 *              rather, the primary hartid of the set of harts.
 *   is_ack   - Boolean indicating an ack message
 *   msg      - For storing data, could be NULL
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_rx_message(ihc_channel_t channel, uint32_t mhartid,
                                bool is_ack, uint32_t *msg)
{
  uint32_t rhartid  = mpfs_ihc_context_to_remote_hart_id(channel);
  uint32_t ctrl_reg = getreg32(MPFS_IHC_CTRL(mhartid, rhartid));

  if (is_ack)
    {
      if (mhartid == CONTEXTB_HARTID)
        {
          uintptr_t msg_in = MPFS_IHC_MSG_IN(mhartid, rhartid);
          DEBUGASSERT(msg == NULL);
          mpfs_ihc_rx_handler((uint32_t *)msg_in, is_ack);
        }
      else
        {
          /* This path is meant for the OpenSBI vendor extension only */

          DEBUGASSERT(msg != NULL);
          mpfs_ihc_message_present_handler(msg, mhartid, rhartid, is_ack);
        }
    }
  else if (MP_MESSAGE_PRESENT == (ctrl_reg & MP_MASK))
    {
      /* Check if we have a message */

      if (mhartid == CONTEXTB_HARTID)
        {
          uintptr_t msg_in = MPFS_IHC_MSG_IN(mhartid, rhartid);
          DEBUGASSERT(msg == NULL);
          mpfs_ihc_rx_handler((uint32_t *)msg_in, is_ack);
        }
      else
        {
          /* This path is meant for the OpenSBI vendor extension only */

          DEBUGASSERT(msg != NULL);
          mpfs_ihc_message_present_handler(msg, mhartid, rhartid, is_ack);
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
 * Name: mpfs_ihc_message_present_isr
 *
 * Description:
 *   This is called from the interrupt handler. This figures out the actions
 *   based on the information retieved from the subsequent functions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_message_present_isr(void)
{
  uint64_t mhartid = riscv_mhartid();
  bool is_ack;

  /* Check all our channels */

  uint32_t origin_hart = mpfs_ihc_parse_incoming_hartid(mhartid, &is_ack);

  if (origin_hart != UNDEFINED_HART_ID)
    {
      /* If autostart is false, we delay the ack until rptun is up.  Note
       * that the master will keep sending the irq if no ack is received.
       */

      if (!g_rptun_initialized)
        {
          return;
        }

      /* This is used to declare the master is up and running */

      g_shmem.master_up = true;

      /* Process incoming packet */

      mpfs_ihc_rx_message(origin_hart, mhartid, is_ack, NULL);

      if (is_ack)
        {
          /* Clear the ack */

          modifyreg32(MPFS_IHC_CTRL(mhartid, origin_hart),
                      ACK_CLR, 0);
        }
    }
}

/****************************************************************************
 * Name: mpfs_ihc_interrupt
 *
 * Description:
 *   This is the interrupt handler.
 *
 * Input Parameters:
 *   irq      - unused
 *   context  - context, unused
 *   arg      - private data pointer, unused
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int mpfs_ihc_interrupt(int irq, void *context, void *arg)
{
  /* The 1st proper interrupt indicates the master is up */

  if (!g_shmem.master_up)
    {
      g_shmem.rsc.rpmsg_vdev.status |= VIRTIO_CONFIG_STATUS_DRIVER_OK;
    }

  mpfs_ihc_message_present_isr();

  return OK;
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

#ifdef CONFIG_MPFS_OPENSBI
void mpfs_ihc_sbi_message_present_indirect_isr(ihc_channel_t channel,
                                               uint32_t *msg)
{
  bool is_ack;
  uint32_t mhartid     = mpfs_ihc_context_to_local_hart_id(channel);
  uint32_t origin_hart = mpfs_ihc_parse_incoming_hartid(mhartid, &is_ack);

  if (origin_hart != UNDEFINED_HART_ID)
    {
      /* Process incoming packet */

      mpfs_ihc_rx_message(origin_hart, mhartid, is_ack, msg);

      if (is_ack)
        {
          /* Clear the ack */

          modifyreg32(MPFS_IHC_CTRL(mhartid, origin_hart), ACK_CLR, 0);
        }
    }
}
#endif

/****************************************************************************
 * Name: mpfs_ihc_local_context_init
 *
 * Description:
 *   This initializes the local context by zeroing the CTRL register and
 *   applying proper values for the g_connected_harts and
 *   g_connected_hart_ints globals. These globals are used to map the harts
 *   and contexts properly.
 *
 * Input Parameters:
 *   hart_to_configure   - Hart to be configured
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_ihc_local_context_init(uint32_t hart_to_configure)
{
  uint32_t rhartid = 0;

  DEBUGASSERT(hart_to_configure < MPFS_NUM_HARTS);

  while (rhartid < MPFS_NUM_HARTS)
    {
      putreg32(0, MPFS_IHC_CTRL(hart_to_configure, rhartid));
      rhartid++;
    }

#ifdef CONFIG_MPFS_OPENSBI
  /* With OpenSBI we act the opposite direction */

  g_connected_harts     = (HSS_HART_MASK | (1 << CONTEXTB_HARTID));
  g_connected_hart_ints = IHCIA_CONTEXTA_INTS;
#else
  g_connected_harts     = ihcia_remote_harts[hart_to_configure];
  g_connected_hart_ints = ihcia_remote_hart_ints[hart_to_configure];
#endif
}

/****************************************************************************
 * Name: mpfs_ihc_local_remote_config
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

static void mpfs_ihc_local_remote_config(uint32_t hart_to_configure,
                                         uint32_t rhartid)
{
  /* Set-up enables in concentrator */

  putreg32(ihcia_remote_hart_ints[hart_to_configure],
           MPFS_IHC_INT_EN(hart_to_configure));

  modifyreg32(MPFS_IHC_CTRL(hart_to_configure, rhartid), 0, MPIE_EN |
                            ACKIE_EN);
}

/****************************************************************************
 * Name: mpfs_ihc_tx_message
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

static int mpfs_ihc_tx_message(ihc_channel_t channel, uint32_t *message)
{
  uint32_t i;
  uint32_t mhartid      = mpfs_ihc_context_to_local_hart_id(channel);
  uint32_t rhartid      = mpfs_ihc_context_to_remote_hart_id(channel);
  uint32_t message_size = getreg32(MPFS_IHC_MSG_SIZE(mhartid, rhartid));
  uint32_t ctrl_reg     = getreg32(MPFS_IHC_CTRL(mhartid, rhartid));

  DEBUGASSERT(message_size <= IHC_MAX_MESSAGE_SIZE);

  /* Check if the system is busy.  All we can try is wait. */

  if ((RMP_MESSAGE_PRESENT | ACK_INT) & ctrl_reg)
    {
      nxsig_usleep(100);

      /* Give it a one more try */

      ctrl_reg = getreg32(MPFS_IHC_CTRL(mhartid, rhartid));
    }

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

      /* Wait for the ACK to arrive to maintain the logic */

      if (mhartid == CONTEXTB_HARTID)
        {
          /* Only applicable for the CONTEXTB_HART */

          nxsem_wait_uninterruptible(&g_mpfs_ack_sig);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_rptun_get_cpuname
 *
 * Description:
 *   Gets the mpfs rptun cpuname.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Pointer to the cpu name string.
 *
 ****************************************************************************/

static const char *mpfs_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct mpfs_rptun_dev_s *priv = container_of(dev,
                                               struct mpfs_rptun_dev_s,
                                               rptun);
  return priv->cpuname;
}

/****************************************************************************
 * Name: mpfs_rptun_get_firmware
 *
 * Description:
 *   Gets the mpfs rptun firmware.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Always null, no associated firmware present
 *
 ****************************************************************************/

static const char *mpfs_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: mpfs_rptun_get_addrenv
 *
 * Description:
 *   Gets the mpfs rptun addrenv.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Always null, no associated addrenv present.
 *
 ****************************************************************************/

static const struct rptun_addrenv_s *
mpfs_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: mpfs_rptun_get_resource
 *
 * Description:
 *   Gets the mpfs rptun resource.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   The resource
 *
 ****************************************************************************/

static struct rptun_rsc_s *
mpfs_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct mpfs_rptun_dev_s *priv = container_of(dev,
                                               struct mpfs_rptun_dev_s,
                                               rptun);
  struct rptun_rsc_s *rsc;

  /* Only slave supported so far */

  DEBUGASSERT(!priv->master);

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }
  else
    {
      /* Perform initial setup */

      priv->shmem = &g_shmem;
      rsc = &priv->shmem->rsc;

      g_shmem.base = VRING_SHMEM;
      g_shmem.seqm = 0;
      g_shmem.seqs = 0;

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                               rpmsg_vdev);
      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS  |
                                      1 << VIRTIO_RPMSG_F_ACK |
                                      VIRTIO_RING_F_EVENT_IDX;

      rsc->rpmsg_vdev.gfeatures     = 1 << VIRTIO_RPMSG_F_NS  |
                                      1 << VIRTIO_RPMSG_F_ACK |
                                      VIRTIO_RING_F_EVENT_IDX;

      /* Set to VIRTIO_CONFIG_STATUS_DRIVER_OK when master is up */

      rsc->rpmsg_vdev.status        = 0;

      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = VRINGS;
      rsc->rpmsg_vring0.align       = VRING_ALIGN;
      rsc->rpmsg_vring0.num         = VRING_NR;
      rsc->rpmsg_vring0.da          = VRING0_DESCRIPTORS;
      rsc->rpmsg_vring0.notifyid    = 0;
      rsc->rpmsg_vring1.align       = VRING_ALIGN;
      rsc->rpmsg_vring1.num         = VRING_NR;
      rsc->rpmsg_vring1.da          = VRING1_DESCRIPTORS;
      rsc->rpmsg_vring0.notifyid    = 1;
      rsc->config.r2h_buf_size      = VRING_SIZE;
      rsc->config.h2r_buf_size      = VRING_SIZE;
    }

  /* It might be tempting to set this at mpfs_rptun_start(), but it's only
   * called if we're the bus master.
   */

  g_rptun_initialized = true;

  return &priv->shmem->rsc;
}

/****************************************************************************
 * Name: mpfs_rptun_is_autostart
 *
 * Description:
 *   Checks whether the rptun needs to autostart without explicit user
 *   start command.  Currently autostart is false, which means that the user
 *   must start the service explicitly, eg.  rptun start /dev/rptun/mpfs-ihc
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Always false
 *
 ****************************************************************************/

static bool mpfs_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return false;
}

/****************************************************************************
 * Name: mpfs_rptun_is_master
 *
 * Description:
 *   Checks whether we are master. Currently only slave mode is supported.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   True, if master, false if slave
 *
 ****************************************************************************/

static bool mpfs_rptun_is_master(struct rptun_dev_s *dev)
{
  struct mpfs_rptun_dev_s *priv = container_of(dev,
                                               struct mpfs_rptun_dev_s,
                                               rptun);
  return priv->master;
}

/****************************************************************************
 * Name: mpfs_rptun_start
 *
 * Description:
 *   Rptun start notifier. This is unused at the moment.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Always zero.
 *
 ****************************************************************************/

static int mpfs_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: mpfs_rptun_stop
 *
 * Description:
 *   Rptun stop notifier. This is currently unused.
 *
 * Input Parameters:
 *   dev   - Rptun device.
 *
 * Returned Value:
 *   Always zero.
 *
 ****************************************************************************/

static int mpfs_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: mpfs_rptun_notify
 *
 * Description:
 *   This rptun notifier is used to indicate the remote hart that it has a
 *   message present.
 *
 * Input Parameters:
 *   dev       - Rptun device.
 *   notifyid  - rpmsg_vringX.notifyid, as declared in the resource.
 *
 * Returned Value:
 *   OK on success, busy on error
 *
 ****************************************************************************/

static int mpfs_rptun_notify(struct rptun_dev_s *dev, uint32_t notifyid)
{
  uint32_t tx_msg[IHC_MAX_MESSAGE_SIZE];

  /* We're looking for the id, but currently it's just RPTUN_NOTIFY_ALL. It's
   * OK, the remote end doesn't really care about the id, but that might
   * change in the future.
   */

  if (notifyid == RPTUN_NOTIFY_ALL)
    {
      tx_msg[0] = 0;
      tx_msg[1] = 0;
    }
  else
    {
      tx_msg[0] = (notifyid << 16);
      tx_msg[1] = 0;
    }

  return mpfs_ihc_tx_message(IHC_CHANNEL_TO_CONTEXTA, tx_msg);
}

/****************************************************************************
 * Name: mpfs_rptun_register_callback
 *
 * Description:
 *   This registers a callback for the rptun.
 *
 * Input Parameters:
 *   dev       - Rptun device.
 *   callback  - The callback rptun calls on receiving data
 *
 * Returned Value:
 *   Zero always
 *
 ****************************************************************************/

static int mpfs_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg)
{
  struct mpfs_rptun_dev_s *priv = container_of(dev,
                                               struct mpfs_rptun_dev_s,
                                               rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

/****************************************************************************
 * Name: mpfs_rptun_init
 *
 * Description:
 *   Initializes the rptun device.
 *
 * Input Parameters:
 *   shmemname  - Shared mempory name
 *   cpuname    - Local CPU name
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 ****************************************************************************/

static int mpfs_rptun_init(const char *shmemname, const char *cpuname)
{
  struct mpfs_rptun_dev_s *dev;
  int ret;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

#ifdef IHC_CHANNEL_SIDE_B
  dev->master = false;
#else
  dev->master = true;
#endif

  dev->rptun.ops = &g_mpfs_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);
  list_add_tail(&g_dev_list, &dev->node);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      list_remove_tail(&g_dev_list);
      kmm_free(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_echo_ping_ept_cb
 *
 * Description:
 *   Callback for the echo ping endpoint. This simply echoes all data it
 *   receives.
 *
 * Input Parameters:
 *   ept   - Endpoint data
 *   data  - Pointer to the data
 *   len   - Length of the received data
 *   src   - Source address
 *   priv  - Handle t o private data
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int mpfs_echo_ping_ept_cb(struct rpmsg_endpoint *ept, void *data,
                                 size_t len, uint32_t src, void *priv)
{
  /* This is simply echoes the data back */

  rpmsg_send(ept, data, len);

  return OK;
}

/****************************************************************************
 * Name: mpfs_echo_ping_init
 *
 * Description:
 *   Initializes the rpmsg echo ping endpoint.
 *
 * Input Parameters:
 *   rdev  - Rpmsg device
 *   ept   - Endpoint data

 * Returned Value:
 *   OK on success, a negated errno otherwise
 *
 ****************************************************************************/

static int mpfs_echo_ping_init(struct rpmsg_device *rdev,
                               struct rpmsg_endpoint *ept)
{
  return rpmsg_create_ept(ept, rdev, MPFS_RPTUN_PING_EPT_NAME,
                          RPMSG_ADDR_ANY, 0,
                          mpfs_echo_ping_ept_cb, NULL);
}

/****************************************************************************
 * Name: mpfs_rpmsg_device_created
 *
 * Description:
 *   Callback that is called when the underlying rpmsg device has been
 *   created. This is used to initialize the ping enpoint at the proper
 *   time.
 *
 * Input Parameters:
 *   rdev   - Rpmsg device
 *   priv_  - Private data

 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_rpmsg_device_created(struct rpmsg_device *rdev, void *priv_)
{
  struct rpmsg_virtio_device *vdev = container_of(rdev,
                                                  struct rpmsg_virtio_device,
                                                  rdev);

  g_mpfs_virtio_device = vdev;
  g_mpfs_rpmsg_device  = rdev;

  g_mpfs_virtqueue_table[0].data = g_mpfs_virtio_device->svq;
  g_mpfs_virtqueue_table[1].data = g_mpfs_virtio_device->rvq;

  mpfs_echo_ping_init(rdev, &g_mpgs_echo_ping_ept);
}

/****************************************************************************
 * Name: mpfs_rptun_thread
 *
 * Description:
 *   This is used to listen to the g_mpfs_rx_sig semaphore and then
 *   notifying the associated virtqueue.  The virtqueue_notification()
 *   cannot be called from irq context.
 *
 * Input Parameters:
 *   argc   - Argument count
 *   argv   - Argument variables

 * Returned Value:
 *   0 on exit
 *
 ****************************************************************************/

static int mpfs_rptun_thread(int argc, char *argv[])
{
  struct mpfs_queue_table_s *info;

  while (1)
    {
      info = &g_mpfs_virtqueue_table[g_vq_idx];
      virtqueue_notification((struct virtqueue *)info->data);

      nxsem_wait(&g_mpfs_rx_sig);
    }

  return 0;
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

#ifdef CONFIG_MPFS_OPENSBI
int mpfs_ihc_sbi_ecall_handler(unsigned long funcid, uint32_t remote_channel,
                               uint32_t *message_ptr)
{
  uint32_t mhartid;
  uint32_t rhartid;
  int result = OK;

  /* Check the channel is bound to a valid context */

  if ((remote_channel < IHC_CHANNEL_TO_CONTEXTA) ||
      (remote_channel > IHC_CHANNEL_TO_CONTEXTB))
    {
      return -EINVAL;
    }

  switch (funcid)
    {
      case SBI_EXT_IHC_CTX_INIT:
        mhartid = mpfs_ihc_context_to_local_hart_id(remote_channel);
        rhartid = mpfs_ihc_context_to_remote_hart_id(remote_channel);
        mpfs_ihc_local_context_init(mhartid);
        mpfs_ihc_local_remote_config(mhartid, rhartid);
        break;

      case SBI_EXT_IHC_SEND:
        result = mpfs_ihc_tx_message(remote_channel, message_ptr);
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
#endif

/****************************************************************************
 * Name: mpfs_ihc_init
 *
 * Description:
 *   This initializes the Inter-Hart Communication (IHC) module.  Rptun is
 *   used to simplify the integration of rpmsg and virtio.  This function
 *   installs the proper interrupt handlers, installs a thread, and performs
 *   all the required initialization tasks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a nagated errno on error
 *
 ****************************************************************************/

int mpfs_ihc_init(void)
{
  uint32_t  mhartid = (uint32_t)riscv_mhartid();
  char     *argv[3];
  char      arg1[19];
  uint32_t  rhartid;
  int       ret;

  /* We're IHC channel side B (slave) */

#ifdef IHC_CHANNEL_SIDE_A
  rhartid = CONTEXTB_HARTID;
#else
  rhartid = CONTEXTA_HARTID;
#endif

#if CONTEXTB_HARTID == 1
  g_plic_irq = MPFS_IRQ_FABRIC_F2H_62;
#elif CONTEXTB_HARTID == 2
  g_plic_irq = MPFS_IRQ_FABRIC_F2H_61;
#elif CONTEXTB_HARTID == 3
  g_plic_irq = MPFS_IRQ_FABRIC_F2H_60;
#elif CONTEXTB_HARTID == 4
  g_plic_irq = MPFS_IRQ_FABRIC_F2H_59;
#else
#  error Misconfiguration
#endif

  /* Initialize IHC FPGA module registers to a known state */

  mpfs_ihc_local_context_init(mhartid);
  mpfs_ihc_local_remote_config(mhartid, rhartid);

  /* Attach and enable the applicable irq */

  ret = irq_attach(g_plic_irq, mpfs_ihc_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(g_plic_irq);
    }
  else
    {
      ihcerr("ERROR: Not able to attach irq\n");
      return ret;
    }

  /* Initialize and wait for the master. This will block until. */

  ihcinfo("Waiting for the master online...\n");
  ret = mpfs_rptun_init(MPFS_RPTUN_SHMEM_NAME, MPFS_RPTUN_CPU_NAME);
  ihcinfo("..master is online\n");

  /* Register callback to notify when rpmsg device is ready */

  ret = rpmsg_register_callback(NULL, mpfs_rpmsg_device_created,
                                NULL, NULL, NULL);
  if (ret < 0)
    {
      ihcerr("ERROR: Not able to register rpmsg callback\n");
      goto init_error;
    }

  /* Thread initialization */

  snprintf(arg1, sizeof(arg1), "0x%" PRIxPTR,
          (uintptr_t)g_mpfs_virtqueue_table);
  argv[0] = "mpfs_ihc_thread";
  argv[1] = arg1;
  argv[2] = NULL;

  ret = kthread_create("mpfs-rptun", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, mpfs_rptun_thread, argv);
  if (ret < 0)
    {
      ihcerr("ERROR: Not able to create a thread!\n");
      rpmsg_unregister_callback(NULL, mpfs_rpmsg_device_created,
                                NULL, NULL, NULL);
      goto init_error;
    }

  return OK;

init_error:

  up_disable_irq(g_plic_irq);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   This is needed by openamp/libmetal/lib/system/nuttx/io.c:78. The
 *   physical memory is mapped as virtual.
 *
 * Input Parameters:
 *   va_
 *
 * Returned Value:
 *   va
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(void *va)
{
  return (uintptr_t)va;
}

/****************************************************************************
 * Name: up_addrenv_pa_to_va
 *
 * Description:
 *   This is needed by openamp/libmetal/lib/system/nuttx/io.c. The
 *   physical memory is mapped as virtual.
 *
 * Input Parameters:
 *   pa
 *
 * Returned Value:
 *   pa
 *
 ****************************************************************************/

void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (void *)pa;
}
