/****************************************************************************
 * drivers/can/sja1000.c
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

#include <assert.h>
#include <debug.h>
#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/can/can.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include <nuttx/can/sja1000.h>
#include "sja1000.h"

#include <nuttx/can.h>

#ifdef CONFIG_CAN_SJA1000

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration
 * ************************************************************/

#if defined(CONFIG_CAN_SJA1000_DEBUG)
#define cantrace _info
#else
#define cantrace _none
#endif /* CONFIG_CAN_SJA1000_DEBUG */

/* Default values written to various registers on initialization */

#define SJA1000_INIT_TEC        0
#define SJA1000_INIT_REC        0
#define SJA1000_INIT_EWL        96

#define SJA1000_ACCEPTANCE_CODE 0x0        /* 32-bit address to match */
#define SJA1000_ACCEPTANCE_MASK 0xffffffff /* 32-bit address mask */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SJA1000 Register access */

#ifdef CONFIG_CANBUS_REGDEBUG
static void sja1000_printreg(uint32_t addr, uint32_t value);
#endif

/* SJA1000 methods */

static void sja1000_reset(struct can_dev_s *dev);
static int sja1000_setup(struct can_dev_s *dev);
static void sja1000_shutdown(struct can_dev_s *dev);
static void sja1000_rxint(struct can_dev_s *dev, bool enable);
static void sja1000_txint(struct can_dev_s *dev, bool enable);
static int sja1000_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg);
static int sja1000_remoterequest(struct can_dev_s *dev, uint16_t id);
static int sja1000_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool sja1000_txready(struct can_dev_s *dev);
static bool sja1000_txempty(struct can_dev_s *dev);

/* SJA1000 interrupts */

static int sja1000_interrupt(FAR struct sja1000_config_s *config,
                             void *arg);

/* SJA1000 acceptance filter */

static void sja1000_set_acc_filter(struct sja1000_dev_s *priv,
                                   uint32_t code, uint32_t mask,
                                   bool single_filter);

/* SJA1000 bit-timing initialization */

static int sja1000_baud_rate(struct sja1000_dev_s *priv, int rate,
                             int clock, int sjw, int sampl_pt, int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_sja1000ops =
{
  .co_reset         = sja1000_reset,
  .co_setup         = sja1000_setup,
  .co_shutdown      = sja1000_shutdown,
  .co_rxint         = sja1000_rxint,
  .co_txint         = sja1000_txint,
  .co_ioctl         = sja1000_ioctl,
  .co_remoterequest = sja1000_remoterequest,
  .co_send          = sja1000_send,
  .co_txready       = sja1000_txready,
  .co_txempty       = sja1000_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sja1000_printreg
 *
 * Description:
 *   Print the value read from a register.
 *
 * Input Parameters:
 *   addr - The register address
 *   value - The register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CANBUS_REGDEBUG
static void sja1000_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr;
  static uint32_t preval;
  static uint32_t count;

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              caninfo("...\n");
            }

          return;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          caninfo("[repeats %" PRId32 " more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = value;
      count    = 1;
    }

  /* Show the register value read */

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, value);
}
#endif /* CONFIG_CANBUS_REGDEBUG */

/****************************************************************************
 * Name: sja1000_reset
 *
 * Description:
 *   Reset the SJA1000 device.  Called early to initialize the hardware.
 *This function is called, before litex_sja1000_setup() and on error
 *conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void sja1000_reset(struct can_dev_s *dev)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  irqstate_t flags;
  int ret;

  caninfo("SJA1000 Device %" PRIu8 "\n", port);

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  flags = spin_lock_irqsave(&priv->lock);
#else
  flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  /* Disable the SJA1000 and stop ongoing transmissions */

  uint32_t mode_value = SJA1000_RESET_MODE_M | SJA1000_LISTEN_ONLY_MODE_M;
  sja1000_putreg(priv,
      SJA1000_MODE_REG, mode_value); /* Enter Reset Mode */

  sja1000_modifyreg32(priv,
      SJA1000_CLOCK_DIVIDER_REG, 0, SJA1000_EXT_MODE_M);

  sja1000_putreg(priv, SJA1000_INT_ENA_REG, 0); /* Disable interrupts */
  sja1000_getreg(priv, SJA1000_STATUS_REG);     /* Clear status bits */

  sja1000_putreg(priv,
      SJA1000_TX_ERR_CNT_REG, SJA1000_INIT_TEC); /* TEC */
  sja1000_putreg(priv,
      SJA1000_RX_ERR_CNT_REG, SJA1000_INIT_REC); /* REC */
  sja1000_putreg(priv,
      SJA1000_ERR_WARNING_LIMIT_REG, SJA1000_INIT_EWL); /* EWL */

  sja1000_set_acc_filter(
      priv, SJA1000_ACCEPTANCE_CODE, SJA1000_ACCEPTANCE_MASK, true);

  /* Set bit timing */

  ret = sja1000_baud_rate(priv, config->bitrate, config->clk_freq,
                          config->sjw, config->samplep, 0);

  if (ret != OK)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
    }

  /* Restart the SJA1000 */

  if (config->loopback)
    {
      /* Leave Reset Mode, enter Test Mode */

      sja1000_putreg(priv, SJA1000_MODE_REG, SJA1000_SELF_TEST_MODE_M);
    }
  else
    {
      /* Leave Reset Mode */

      sja1000_putreg(priv, SJA1000_MODE_REG, 0);
    }

  /* Abort transmission and clear overrun.
   * Command register can only be modified when in Operation Mode.
   */

  sja1000_putreg(priv,
      SJA1000_CMD_REG, SJA1000_ABORT_TX_M | SJA1000_CLR_OVERRUN_M);

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */
}

/****************************************************************************
 * Name: sja1000_setup
 *
 * Description:
 *   Configure the SJA1000. This method is called the first time that the
 *SJA1000 device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching SJA1000 interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int sja1000_setup(struct can_dev_s *dev)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  irqstate_t flags;
  int ret = OK;

  caninfo("SJA1000 (%" PRIu8 ")\n", port);

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  flags = spin_lock_irqsave(&priv->lock);
#else
  flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  sja1000_putreg(priv, SJA1000_INT_ENA_REG, SJA1000_DEFAULT_INTERRUPTS);

  /* clear latched interrupts */

  sja1000_getreg(priv, SJA1000_INT_RAW_REG);

  /* Attach the SJA1000 interrupts and handler. */

  ret = config->attach(
      config, (sja1000_handler_t)sja1000_interrupt, (FAR void *)dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach to IRQ Handler!\n");
      return ret;
    }

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  return ret;
}

/****************************************************************************
 * Name: sja1000_shutdown
 *
 * Description:
 *   Disable the SJA1000.  This method is called when the SJA1000 device is
 *closed. This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sja1000_shutdown(struct can_dev_s *dev)
{
  int ret;
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;

  cantrace("shutdown SJA1000 (%" PRIu8 ")\n", port);

  /* Detach the SJA1000 interrupts and handler. */

  ret = config->detach(config);
  if (ret < 0)
    {
      canerr("ERROR: Failed to detach from IRQ Handler!\n");
    }
}

/****************************************************************************
 * Name: sja1000_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable receive interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sja1000_rxint(struct can_dev_s *dev, bool enable)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  uint32_t regval;
  irqstate_t flags;

  cantrace("SJA1000 (%" PRIu8 ") enable: %d\n", port, enable);

  /* The INT_ENA register is also modified from the interrupt handler,
   * so we have to protect this code section.
   */

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  flags = spin_lock_irqsave(&priv->lock);
#else
  flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  regval = sja1000_getreg(priv, SJA1000_INT_ENA_REG);
  if (enable)
    {
      regval |= SJA1000_RX_INT_ENA_M;
    }
  else
    {
      regval &= ~SJA1000_RX_INT_ENA_M;
    }

  sja1000_putreg(priv, SJA1000_INT_ENA_REG, regval);
#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */
}

/****************************************************************************
 * Name: sja1000_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable transmit interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sja1000_txint(struct can_dev_s *dev, bool enable)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  uint32_t regval;
  irqstate_t flags;

  cantrace("SJA1000 (%" PRIu8 ") enable: %d\n", port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to
   * avoid lost TX interrupts.
   */

  if (!enable)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

#ifdef CONFIG_ARCH_HAVE_MULTICPU
      flags = spin_lock_irqsave(&priv->lock);
#else
      flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

      /* Disable all TX interrupts */

      regval = sja1000_getreg(priv, SJA1000_INT_ENA_REG);
      regval &= ~(SJA1000_TX_INT_ENA_M);
      sja1000_putreg(priv, SJA1000_INT_ENA_REG, regval);
#ifdef CONFIG_ARCH_HAVE_MULTICPU
      spin_unlock_irqrestore(&priv->lock, flags);
#else
      leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */
    }

  cantrace("Exiting.\n");
}

/****************************************************************************
 * Name: sja1000_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   cmd - A ioctl command.
 *   arg - A ioctl argument.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int sja1000_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  int ret                         = -ENOTTY;
  uint8_t port                    = config->port;

  cantrace("SJA1000 (%" PRIu8 ") cmd=%04x arg=%lu\n", port, cmd, arg);

  /* Handle the command */

  switch (cmd)
    {
        /* CANIOC_GET_BITTIMING:
         *   Description:    Return the current bit timing settings
         *   Argument:       A pointer to a write-able instance of struct
         *                   canioc_bittiming_s in which current bit timing
         *                   values will be returned.
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR) is returned with the errno variable set
         *                   to indicate the nature of the error.
         *   Dependencies:   None
         */

      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt = (struct canioc_bittiming_s *)arg;
          uint32_t timing0;
          uint32_t timing1;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

          timing0 = sja1000_getreg(priv, SJA1000_BUS_TIMING_0_REG);
          timing1 = sja1000_getreg(priv, SJA1000_BUS_TIMING_1_REG);

          brp        = ((timing0 & SJA1000_BAUD_PRESC_M) + 1) * 2;
          bt->bt_sjw = ((timing0 & SJA1000_SYNC_JUMP_WIDTH_M)
                        >> SJA1000_SYNC_JUMP_WIDTH_S)
                       + 1;

          bt->bt_tseg1
              = ((timing1 & SJA1000_TIME_SEG1_M) >> SJA1000_TIME_SEG1_S)
                + 1;
          bt->bt_tseg2
              = ((timing1 & SJA1000_TIME_SEG2_M) >> SJA1000_TIME_SEG2_S)
                + 1;
          bt->bt_baud = config->clk_freq
                        / (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));

          cantrace("Retrieved baud rate. TS1: %" PRId8 " TS2: %" PRId8
                   " BRP: %" PRId32 "\n",
                   bt->bt_tseg1, bt->bt_tseg2, brp);
          cantrace("timing0: 0x%" PRIx32 ", timing1: 0x%" PRIx32 " Baud: "
                                                                 "%" PRId32
                   "\n",
                   timing0, timing1, bt->bt_baud);

          ret = OK;
        }
        break;

        /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
}

static int sja1000_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canwarn("Remote request not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: sja1000_send
 *
 * Description:
 *    Send one SJA1000 message.
 *
 *    One SJA1000-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit SJA1000 identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit SJA1000 identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: SJA1000 data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   msg - A message to send.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int sja1000_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct sja1000_dev_s *priv      = (struct sja1000_dev_s *)dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint32_t regval;
  uint32_t i;
  uint32_t len;
  uint32_t id;
  uint32_t frame_info;
  irqstate_t flags;
  uint8_t port       = config->port;
  int ret            = OK;

  cantrace("SJA1000 (%" PRIu8 ") ID: %" PRIu32 " DLC: %" PRIu8 "\n", port,
           (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
  if (len > CAN_MAXDATALEN)
    len = CAN_MAXDATALEN;

  frame_info = len;

  if (msg->cm_hdr.ch_rtr)
    {
      frame_info |= (1 << 6);
    }

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  flags = spin_lock_irqsave(&priv->lock);
#else
  flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  /* Make sure that TX interrupts are enabled BEFORE sending the
   * message.
   *
   * NOTE: The INT_ENA is also modified from the interrupt handler, but the
   * following is safe because interrupts are disabled here.
   */

  regval = sja1000_getreg(priv, SJA1000_INT_ENA_REG);
  regval |= SJA1000_TX_INT_ENA_M;
  sja1000_putreg(priv, SJA1000_INT_ENA_REG, regval);

  /* Set up the transfer */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_EXTMSGID) == 0);
      frame_info |= (1 << 7);
      sja1000_putreg(priv, SJA1000_DATA_0_REG, frame_info);

      id <<= 3;
      sja1000_putreg(priv, SJA1000_DATA_4_REG, id & 0xff);
      id >>= 8;
      sja1000_putreg(priv, SJA1000_DATA_3_REG, id & 0xff);
      id >>= 8;
      sja1000_putreg(priv, SJA1000_DATA_2_REG, id & 0xff);
      id >>= 8;
      sja1000_putreg(priv, SJA1000_DATA_1_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          sja1000_putreg(priv,
              (SJA1000_DATA_5_REG + i), msg->cm_data[i]);
        }
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_STDMSGID) == 0);
      sja1000_putreg(priv, SJA1000_DATA_0_REG, frame_info);
      id <<= 5;
      sja1000_putreg(priv, SJA1000_DATA_1_REG, (id >> 8) & 0xff);
      sja1000_putreg(priv, SJA1000_DATA_2_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          sja1000_putreg(priv,
              (SJA1000_DATA_3_REG + i), msg->cm_data[i]);
        }
    }

  /* Send the message */

  if (config->loopback)
    {
      sja1000_putreg(priv, SJA1000_CMD_REG,
                   SJA1000_SELF_RX_REQ_M | SJA1000_ABORT_TX_M);
    }
  else
    {
      sja1000_putreg(priv, SJA1000_CMD_REG, SJA1000_TX_REQ_M);
    }

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  return ret;
}

/****************************************************************************
 * Name: sja1000_txready
 *
 * Description:
 *   Return true if the SJA1000 hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if the SJA1000 hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool sja1000_txready(struct can_dev_s *dev)
{
  struct sja1000_dev_s *priv      = dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  uint32_t regval = sja1000_getreg(priv, SJA1000_STATUS_REG);

  caninfo("SJA1000 (%" PRIu8 ") txready: %d\n", port,
          ((regval & SJA1000_TX_BUF_ST_M) != 0));
  return ((regval & SJA1000_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: sja1000_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the SJA1000
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the SJA1000 hardware.
 *
 ****************************************************************************/

static bool sja1000_txempty(struct can_dev_s *dev)
{
  struct sja1000_dev_s *priv      = dev->cd_priv;
  struct sja1000_config_s *config = priv->config;
  uint8_t port                    = config->port;
  uint32_t regval = sja1000_getreg(priv, SJA1000_STATUS_REG);

  caninfo("SJA1000 (%" PRIu8 ") txempty: %d\n", port,
          ((regval & SJA1000_TX_BUF_ST_M) != 0));
  return ((regval & SJA1000_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: sja1000_interrupt
 *
 * Description:
 *   SJA1000 RX/TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - The pointer to driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int sja1000_interrupt(FAR struct sja1000_config_s *config, void *arg)
{
#ifdef CONFIG_CAN_SJA1000
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct sja1000_dev_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[8];
  uint32_t frame_info;
  uint32_t len;
  uint32_t datastart;
  uint32_t regval;
  uint32_t i;

  /* Read the interrupt register results in clearing bits  */

  regval = sja1000_getreg(priv, SJA1000_INT_RAW_REG);

  cantrace("Entered. Regval = 0x%" PRIx32 "\n", regval);

  /* Check for a receive interrupt */

  if ((regval & SJA1000_RX_INT_ST_M) != 0)
    {
      memset(&hdr, 0, sizeof(hdr));
      memset(data, 0, sizeof(data));

      frame_info = sja1000_getreg(priv, SJA1000_DATA_0_REG);

      /* Construct the SJA1000 header */

      if (frame_info & (1 << 6))
        {
          hdr.ch_rtr = 1;
        }

#ifdef CONFIG_CAN_EXTID
      if (frame_info & (1 << 7))
        {
          /* The provided ID should be 29 bits */

          hdr.ch_extid = 1;
          hdr.ch_id    = (sja1000_getreg(priv, SJA1000_DATA_1_REG) << 21)
                      + (sja1000_getreg(priv, SJA1000_DATA_2_REG) << 13)
                      + (sja1000_getreg(priv, SJA1000_DATA_3_REG) << 5)
                      + (sja1000_getreg(priv, SJA1000_DATA_4_REG) >> 3);
          datastart = SJA1000_DATA_5_REG;
        }
      else
#endif /* CONFIG_CAN_EXTID */
        {
          /* The provided ID should be 11 bits */

          hdr.ch_id = (sja1000_getreg(priv, SJA1000_DATA_1_REG) << 3)
                      + (sja1000_getreg(priv, SJA1000_DATA_2_REG) >> 5);
          datastart = SJA1000_DATA_3_REG;
        }

      len = frame_info & 0xf;
      if (len > CAN_MAXDATALEN)
        {
          len = CAN_MAXDATALEN;
        }

      hdr.ch_dlc = len;

      for (i = 0; i < len; i++)
        {
          data[i] = sja1000_getreg(priv, (datastart + i));
        }

      /* Release the receive buffer */

      sja1000_putreg(priv, SJA1000_CMD_REG, SJA1000_RELEASE_BUF_M);

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error = 0; /* Error reporting not supported */
#endif                  /* CONFIG_CAN_ERRORS */
      can_receive(dev, &hdr, data);
    }

  /* Check for TX buffer complete */

  if ((regval & SJA1000_TX_INT_ST_M) != 0)
    {
      /* Disable all further TX buffer interrupts */

      regval = sja1000_getreg(priv, SJA1000_INT_ENA_REG);
      regval &= ~SJA1000_TX_INT_ENA_M;
      sja1000_putreg(priv, SJA1000_INT_ENA_REG, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

#endif /* CONFIG_CAN_SJA1000 */
  return OK;
}

/****************************************************************************
 * Name: sja1000_set_acc_filter
 *
 * Description:
 *   Call to set acceptance filter.
 *   Must be called in reset mode.
 *
 * Input Parameters:
 *   priv - Private SJA1000 context
 *   code - Acceptance Code.
 *   mask - Acceptance Mask.
 *   single_filter - Whether to enable single filter mode.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sja1000_set_acc_filter(struct sja1000_dev_s *priv,
                                   uint32_t code, uint32_t mask,
                                   bool single_filter)
{
  uint32_t regval;

  regval = sja1000_getreg(priv, SJA1000_MODE_REG);
  if (single_filter)
    {
      regval |= SJA1000_RX_FILTER_MODE_M;
    }
  else
    {
      regval &= ~(SJA1000_RX_FILTER_MODE_M);
    }

  sja1000_putreg(priv, SJA1000_MODE_REG, regval);

  for (int i = 0; i < 4; i++)
    {
      sja1000_putreg(priv, (SJA1000_DATA_0_REG + i),
                     ((code >> ((3 - i) * 8)) & 0xff));
      sja1000_putreg(priv, (SJA1000_DATA_4_REG + i),
                     ((mask >> ((3 - i) * 8)) & 0xff));
    }
}

/****************************************************************************
 * Name: sja1000_baud_rate
 *
 * Description:
 *   Set the CAN bus timing registers based on the configured bit-rate and
 *   sample point position.
 *
 * The bit timing logic monitors the serial bus-line and performs sampling
 * and adjustment of the sample point by synchronizing on the start-bit edge
 * and resynchronizing on the following edges.
 *
 * Its operation may be explained simply by splitting nominal bit time into
 * three segments as follows:
 *
 * 1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *    within this time segment. It has a fixed length of one time quantum
 *    (1 x tCAN).
 * 2. Bit segment 1 (BS1): defines the location of the sample point. It
 *    includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *    is programmable between 1 and 16 time quanta but may be automatically
 *    lengthened to compensate for positive phase drifts due to differences
 *    in the frequency of the various nodes of the network.
 * 3. Bit segment 2 (BS2): defines the location of the transmit point. It
 *    represents the PHASE_SEG2 of the CAN standard. Its duration is
 *    programmable between 1 and 8 time quanta but may also be automatically
 *    shortened to compensate for negative phase drifts.
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tcan
 *
 * Where:
 *   Tcan is the period of the APB clock
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int sja1000_baud_rate(struct sja1000_dev_s *priv, int rate,
                             int clock, int sjw, int sampl_pt, int flags)
{
  struct sja1000_config_s *config = priv->config;
  const struct can_bittiming_const_s *timing = config->bittiming_const;
  int best_error = 1000000000;
  int error;
  int best_tseg = 0;
  int best_brp  = 0;
  int best_rate = 0;
  int brp       = 0;
  int tseg      = 0;
  int tseg1     = 0;
  int tseg2     = 0;
  uint32_t timing0;
  uint32_t timing1;

  /* tseg even = round down, odd = round up */

  for (tseg = (0 + 0 + 2) * 2;
       tseg <= (timing->tseg2_max + timing->tseg1_max + 2) * 2 + 1; tseg++)
    {
      brp = clock / ((1 + tseg / 2) * rate) + tseg % 2;
      if (brp == 0 || brp > 64)
        {
          continue;
        }

      error = rate - clock / (brp * (1 + tseg / 2));
      if (error < 0)
        {
          error = -error;
        }

      if (error <= best_error)
        {
          best_error = error;
          best_tseg  = tseg / 2;
          best_brp   = brp;
          best_rate  = clock / (brp * (1 + tseg / 2));
        }
    }

  if (best_error && (rate / best_error < 10))
    {
      canerr(
          "baud rate %d is not possible with %d Hz clock\n", rate, clock);
      canerr("%d bps. brp=%d, best_tseg=%d, tseg1=%d, tseg2=%d\n",
             best_rate, best_brp, best_tseg, tseg1, tseg2);
      return -EINVAL;
    }

  tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100 + 1;
  if (tseg2 < 0)
    {
      tseg2 = 0;
    }

  if (tseg2 > timing->tseg2_max)
    {
      tseg2 = timing->tseg2_max;
    }

  tseg1 = best_tseg - tseg2;
  if (tseg1 > timing->tseg1_max)
    {
      tseg1 = timing->tseg1_max;
      tseg2 = best_tseg - tseg1;
    }

  caninfo("Setting baud rate. TS1: %d TS2: %d BRP: %d\n", tseg1, tseg2,
          best_brp);

  /* Configure bit timing */

  timing0 = ((best_brp - 1) / 2) & SJA1000_BAUD_PRESC_M;
  timing0 |= ((sjw - 1) << SJA1000_SYNC_JUMP_WIDTH_S)
             & SJA1000_SYNC_JUMP_WIDTH_M;
  timing1 = (tseg1 - 1) & SJA1000_TIME_SEG1_M;
  timing1 |= ((tseg2 - 1) << SJA1000_TIME_SEG2_S) & SJA1000_TIME_SEG2_M;

  if (config->triple_sample)
    {
      /* The bus is sampled 3 times (recommended for low to medium speed
       * buses to spikes on the bus-line).
       */

      timing1 |= (config->triple_sample << SJA1000_TIME_SAMP_S)
                 & SJA1000_TIME_SAMP_M;
    }

  cantrace("Writing to BTR0, BTR1: timing0: 0x%" PRIx32 " timing1: "
                                                        "0x%" PRIx32 "\n",
           timing0, timing1);

  sja1000_putreg(priv, SJA1000_BUS_TIMING_1_REG, timing1);
  sja1000_putreg(priv, SJA1000_BUS_TIMING_0_REG, timing0);

#ifdef CONFIG_CANBUS_REGDEBUG
  timing1 = sja1000_getreg(priv, SJA1000_BUS_TIMING_1_REG);
  timing0 = sja1000_getreg(priv, SJA1000_BUS_TIMING_0_REG);
  caninfo("Read-verify: timing0: 0x%" PRIx32 " timing1: 0x%" PRIx32 "\n",
          timing0, timing1);
#endif /* CONFIG_CANBUS_REGDEBUG */
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sja1000_instantiate
 *
 * Description:
 *   Initialize the selected SJA1000 CAN Bus Controller
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" CAN driver state structure.
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *sja1000_instantiate(FAR struct sja1000_dev_s *priv)
{
  struct sja1000_config_s *config = priv->config;
  FAR struct can_dev_s *dev;
  irqstate_t flags;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(config);

  cantrace("Starting sja1000_instantiate()!\n");

  /* Allocate a CAN Device structure */

  dev = kmm_zalloc(sizeof(struct can_dev_s));
  if (dev == NULL)
    {
      canerr("ERROR: Failed to allocate instance of can_dev_s!\n");
      return NULL;
    }

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  flags = spin_lock_irqsave(&priv->lock);
#else
  flags = enter_critical_section();
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  priv->lock = SP_UNLOCKED;
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  dev->cd_ops = &g_sja1000ops;
  dev->cd_priv = (FAR void *)priv;

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  leave_critical_section(flags);
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

  /* Reset chip */

  sja1000_reset(dev);

  return dev;
}

#endif /* CONFIG_CAN_SJA1000 */
