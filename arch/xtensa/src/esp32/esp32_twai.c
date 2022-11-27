/****************************************************************************
 * arch/xtensa/src/esp32/esp32_twai.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"

#include "esp32_gpio.h"
#include "esp32_twai.h"
#include "esp32_irq.h"
#include "esp32_clockconfig.h"

#include "hardware/esp32_dport.h"
#include "hardware/esp32_gpio_sigmap.h"

#if defined(CONFIG_ESP32_TWAI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Default values written to various registers on initialization */

#define TWAI_INIT_TEC             0
#define TWAI_INIT_REC             0
#define TWAI_INIT_EWL             96

/* Exclude data overrun (bit[3]) and brp_div (bit[4]) */

#define TWAI_DEFAULT_INTERRUPTS   0xe7

#define TWAI_ACCEPTANCE_CODE      0x0           /* 32-bit address to match */
#define TWAI_ACCEPTANCE_MASK      0xffffffff    /* 32-bit address mask */

#ifdef CONFIG_ESP32_TWAI0

/* A TWAI bit rate must be provided */

#  ifndef CONFIG_ESP32_TWAI0_BITRATE
#    error "CONFIG_ESP32_TWAI0_BITRATE is not defined"
#  endif

/* If no sample point is provided, use a sample point of 80 */

#  ifndef CONFIG_ESP32_TWAI0_SAMPLEP
#    define CONFIG_ESP32_TWAI0_SAMPLEP 80
#  endif
#endif

/* If no Synchronization Jump Width is provided, use a SJW of 3 */

#ifndef CONFIG_ESP32_TWAI0_SJW
#  define CONFIG_ESP32_TWAI0_SJW 3
#endif

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing TWAI */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_ESP32_TWAI_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN hardware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */

struct can_bittiming_const
{
  uint32_t tseg1_min;  /* Time segment 1 */
  uint32_t tseg1_max;
  uint32_t tseg2_min;  /* Time segment 2 */
  uint32_t tseg2_max;
  uint32_t sjw_min;    /* Synchronization jump width */
  uint32_t sjw_max;
  uint32_t brp_min;    /* Bit-rate prescaler */
  uint32_t brp_max;
  uint32_t brp_inc;
};

struct twai_dev_s
{
  /* Device configuration */

  const struct can_bittiming_const *bittiming_const;
  uint8_t    port;       /* TWAI port number */
  uint8_t    periph;     /* Peripheral ID */
  uint8_t    irq;        /* IRQ associated with this TWAI */
  uint8_t    cpu;        /* CPU ID */
  uint8_t    cpuint;     /* CPU interrupt assigned to this TWAI */
  uint32_t   bitrate;    /* Configured bit rate */
  uint32_t   samplep;    /* Configured sample point */
  uint32_t   sjw;        /* Synchronization jump width */
  uint32_t   base;       /* TWAI register base address */
  spinlock_t lock;       /* Device specific lock */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TWAI Register access */

#ifdef CONFIG_ESP32_TWAI_REGDEBUG
static void twai_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t twai_getreg(uint32_t addr);
static void twai_putreg(uint32_t addr, uint32_t value);

/* TWAI methods */

static void esp32twai_reset(struct can_dev_s *dev);
static int  esp32twai_setup(struct can_dev_s *dev);
static void esp32twai_shutdown(struct can_dev_s *dev);
static void esp32twai_rxint(struct can_dev_s *dev, bool enable);
static void esp32twai_txint(struct can_dev_s *dev, bool enable);
static int  esp32twai_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  esp32twai_remoterequest(struct can_dev_s *dev,
                                      uint16_t id);
static int  esp32twai_send(struct can_dev_s *dev,
                           struct can_msg_s *msg);
static bool esp32twai_txready(struct can_dev_s *dev);
static bool esp32twai_txempty(struct can_dev_s *dev);

/* TWAI interrupts */

static int esp32twai_interrupt(int irq, void *context, void *arg);

/* TWAI acceptance filter */

static void esp32twai_set_acc_filter(uint32_t code, uint32_t mask,
                                     bool single_filter);

/* TWAI bit-timing initialization */

static int twai_baud_rate(struct twai_dev_s *priv, int rate, int clock,
    int sjw, int sampl_pt, int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_bittiming_const esp32_twai_bittiming_const =
{
  .tseg1_min        = 1,
  .tseg1_max        = 16,
  .tseg2_min        = 1,
  .tseg2_max        = 8,
  .sjw_min          = 1,
  .sjw_max          = 3,
  .brp_min          = 1,
  .brp_max          = 64,
  .brp_inc          = 1,
};

static const struct can_ops_s g_twaiops =
{
  .co_reset         = esp32twai_reset,
  .co_setup         = esp32twai_setup,
  .co_shutdown      = esp32twai_shutdown,
  .co_rxint         = esp32twai_rxint,
  .co_txint         = esp32twai_txint,
  .co_ioctl         = esp32twai_ioctl,
  .co_remoterequest = esp32twai_remoterequest,
  .co_send          = esp32twai_send,
  .co_txready       = esp32twai_txready,
  .co_txempty       = esp32twai_txempty,
};

#ifdef CONFIG_ESP32_TWAI0
static struct twai_dev_s g_twai0priv =
{
  .bittiming_const  = &esp32_twai_bittiming_const,
  .port             = 0,
  .periph           = ESP32_PERIPH_TWAI,
  .irq              = ESP32_IRQ_TWAI,
  .cpuint           = -ENOMEM,
  .bitrate          = CONFIG_ESP32_TWAI0_BITRATE,
  .samplep          = CONFIG_ESP32_TWAI0_SAMPLEP,
  .sjw              = CONFIG_ESP32_TWAI0_SJW,
  .base             = DR_REG_TWAI_BASE,
  .lock             = SP_UNLOCKED,
};

static struct can_dev_s g_twai0dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: twai_printreg
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

#ifdef CONFIG_ESP32_TWAI_REGDEBUG
static void twai_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

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

          caninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = value;
      count    = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", addr, value);
}
#endif

/****************************************************************************
 * Name: twai_getreg
 *
 * Description:
 *   Read the value of an TWAI register.
 *
 * Input Parameters:
 *   addr - The address to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_TWAI_REGDEBUG
static uint32_t twai_getreg(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  twai_printreg(addr, value);
  return value;
}
#else
static uint32_t twai_getreg(uint32_t addr)
{
  return getreg32(addr);
}
#endif

/****************************************************************************
 * Name: twai_putreg
 *
 * Description:
 *   Set the value of an TWAI register.
 *
 * Input Parameters:
 *   addr - The address to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_TWAI_REGDEBUG
static void twai_putreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void twai_putreg(uint32_t addr, uint32_t value)
{
  putreg32(value, addr);
}
#endif

/****************************************************************************
 * Name: esp32twai_reset
 *
 * Description:
 *   Reset the TWAI device.  Called early to initialize the hardware. This
 *   function is called, before esp32_twai_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void esp32twai_reset(struct can_dev_s *dev)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  flags = spin_lock_irqsave(&priv->lock);

  /* Disable the TWAI and stop ongoing transmissions */

  uint32_t mode_value = TWAI_RESET_MODE_M | TWAI_LISTEN_ONLY_MODE_M;
  twai_putreg(TWAI_MODE_REG, mode_value);                 /* Enter Reset Mode */

  modifyreg32(TWAI_CLOCK_DIVIDER_REG, 0, TWAI_EXT_MODE_M);

  twai_putreg(TWAI_INT_ENA_REG, 0);                       /* Disable interrupts */
  twai_getreg(TWAI_STATUS_REG);                           /* Clear status bits */

  twai_putreg(TWAI_TX_ERR_CNT_REG, TWAI_INIT_TEC);        /* TEC */
  twai_putreg(TWAI_RX_ERR_CNT_REG, TWAI_INIT_REC);        /* REC */
  twai_putreg(TWAI_ERR_WARNING_LIMIT_REG, TWAI_INIT_EWL); /* EWL */

  esp32twai_set_acc_filter(TWAI_ACCEPTANCE_CODE,
                           TWAI_ACCEPTANCE_MASK, true);

  /* Set bit timing */

  ret = twai_baud_rate(priv, priv->bitrate, esp_clk_apb_freq(),
                       priv->sjw, priv->samplep, 0);

  if (ret != OK)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
    }

  /* Restart the TWAI */

#ifdef CONFIG_CAN_LOOPBACK
  twai_putreg(TWAI_MODE_REG, TWAI_SELF_TEST_MODE_M); /* Leave Reset Mode, enter Test Mode */
#else
  twai_putreg(TWAI_MODE_REG, 0);                     /* Leave Reset Mode */
#endif

  /* Abort transmission, release RX buffer and clear overrun.
   * Command register can only be modified when in Operation Mode.
   */

  twai_putreg(TWAI_CMD_REG, TWAI_ABORT_TX_M | TWAI_RELEASE_BUF_M |
              TWAI_CLR_OVERRUN_M);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32twai_setup
 *
 * Description:
 *   Configure the TWAI. This method is called the first time that the TWAI
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching TWAI interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32twai_setup(struct can_dev_s *dev)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret = OK;

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  flags = spin_lock_irqsave(&priv->lock);

  twai_putreg(TWAI_INT_ENA_REG, TWAI_DEFAULT_INTERRUPTS);

  twai_getreg(TWAI_INT_RAW_REG);          /* clear latched interrupts */

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->irq);
    }

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32_setup_irq(priv->cpu, priv->periph,
                                 1, ESP32_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = priv->cpuint;
      spin_unlock_irqrestore(&priv->lock, flags);

      return ret;
    }

  ret = irq_attach(priv->irq, esp32twai_interrupt, dev);
  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32_teardown_irq(priv->cpu, priv->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      spin_unlock_irqrestore(&priv->lock, flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the TWAI device. */

  up_enable_irq(priv->irq);

  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: esp32twai_shutdown
 *
 * Description:
 *   Disable the TWAI.  This method is called when the TWAI device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32twai_shutdown(struct can_dev_s *dev)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;

#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("shutdown TWAI%" PRIu8 "\n", priv->port);
#endif

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable cpu interrupt */

      up_disable_irq(priv->irq);

      /* Dissociate the IRQ from the ISR */

      irq_detach(priv->irq);

      /* Free cpu interrupt that is attached to this peripheral */

      esp32_teardown_irq(priv->cpu, priv->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
    }
}

/****************************************************************************
 * Name: esp32twai_rxint
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

static void esp32twai_rxint(struct can_dev_s *dev, bool enable)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  /* The INT_ENA register is also modified from the interrupt handler,
   * so we have to protect this code section.
   */

  flags = spin_lock_irqsave(&priv->lock);

  regval = twai_getreg(TWAI_INT_ENA_REG);
  if (enable)
    {
      regval |= TWAI_RX_INT_ENA_M;
    }
  else
    {
      regval &= ~TWAI_RX_INT_ENA_M;
    }

  twai_putreg(TWAI_INT_ENA_REG, regval);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32twai_txint
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

static void esp32twai_txint(struct can_dev_s *dev, bool enable)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (!enable)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

      flags = spin_lock_irqsave(&priv->lock);

      /* Disable all TX interrupts */

      regval = twai_getreg(TWAI_INT_ENA_REG);
      regval &= ~(TWAI_TX_INT_ENA_M);
      twai_putreg(TWAI_INT_ENA_REG, regval);
      spin_unlock_irqrestore(&priv->lock, flags);
    }
}

/****************************************************************************
 * Name: esp32twai_ioctl
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

static int esp32twai_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  int ret = -ENOTTY;

  caninfo("TWAI%" PRIu8 " cmd=%04x arg=%lu\n", priv->port, cmd, arg);

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
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;
          uint32_t timing0;
          uint32_t timing1;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

          timing0 = twai_getreg(TWAI_BUS_TIMING_0_REG);
          timing1 = twai_getreg(TWAI_BUS_TIMING_1_REG);

          brp = ((timing0 & TWAI_BAUD_PRESC_M) + 1) * 2;
          bt->bt_sjw = ((timing0 & TWAI_SYNC_JUMP_WIDTH_M) >>
                       TWAI_SYNC_JUMP_WIDTH_S) + 1;

          bt->bt_tseg1 = ((timing1 & TWAI_TIME_SEG1_M) >>
                         TWAI_TIME_SEG1_S) + 1;
          bt->bt_tseg2 = ((timing1 & TWAI_TIME_SEG2_M) >>
                         TWAI_TIME_SEG2_S)+ 1;
          bt->bt_baud = esp_clk_apb_freq() /
              (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));

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

static int esp32twai_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canwarn("Remote request not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp32twai_send
 *
 * Description:
 *    Send one TWAI message.
 *
 *    One TWAI-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit TWAI identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit TWAI identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: TWAI data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   msg - A message to send.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32twai_send(struct can_dev_s *dev,
                          struct can_msg_s *msg)
{
  struct twai_dev_s *priv = (struct twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  uint32_t i;
  uint32_t len;
  uint32_t id;
  uint32_t frame_info;
  irqstate_t flags;
  int ret = OK;

  caninfo("TWAI%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
  if (len > CAN_MAXDATALEN) len = CAN_MAXDATALEN;

  frame_info = len;

  if (msg->cm_hdr.ch_rtr)
    {
      frame_info |= (1 << 6);
    }

  flags = spin_lock_irqsave(&priv->lock);

  /* Make sure that TX interrupts are enabled BEFORE sending the
   * message.
   *
   * NOTE: The INT_ENA is also modified from the interrupt handler, but the
   * following is safe because interrupts are disabled here.
   */

  regval  = twai_getreg(TWAI_INT_ENA_REG);
  regval |= TWAI_TX_INT_ENA_M;
  twai_putreg(TWAI_INT_ENA_REG, regval);

  /* Set up the transfer */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_EXTMSGID) == 0);
      frame_info |= (1 << 7);
      twai_putreg(TWAI_DATA_0_REG, frame_info);

      id <<= 3;
      twai_putreg(TWAI_DATA_4_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_3_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_2_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_1_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          twai_putreg(TWAI_DATA_5_REG + (i * 4), msg->cm_data[i]);
        }
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_STDMSGID) == 0);
      twai_putreg(TWAI_DATA_0_REG, frame_info);
      id <<= 5;
      twai_putreg(TWAI_DATA_1_REG, (id >> 8) & 0xff);
      twai_putreg(TWAI_DATA_2_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          twai_putreg(TWAI_DATA_3_REG + (i * 4), msg->cm_data[i]);
        }
    }

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
    twai_putreg(TWAI_CMD_REG, TWAI_SELF_RX_REQ_M | TWAI_ABORT_TX_M);
#else
    twai_putreg(TWAI_CMD_REG, TWAI_TX_REQ_M);
#endif
  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: esp32twai_txready
 *
 * Description:
 *   Return true if the TWAI hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if the TWAI hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool esp32twai_txready(struct can_dev_s *dev)
{
  struct twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_getreg(TWAI_STATUS_REG);

  caninfo("TWAI%" PRIu8 " txready: %d\n", priv->port,
         ((regval & TWAI_TX_BUF_ST_M) != 0));
  return ((regval & TWAI_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: esp32twai_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the TWAI
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the TWAI hardware.
 *
 ****************************************************************************/

static bool esp32twai_txempty(struct can_dev_s *dev)
{
  struct twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_getreg(TWAI_STATUS_REG);

  caninfo("TWAI%" PRIu8 " txempty: %d\n", priv->port,
         ((regval & TWAI_TX_BUF_ST_M) != 0));
  return ((regval & TWAI_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: esp32twai_interrupt
 *
 * Description:
 *   TWAI0 RX/TX interrupt handler
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

static int esp32twai_interrupt(int irq, void *context, void *arg)
{
#ifdef CONFIG_ESP32_TWAI0
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct can_hdr_s hdr;
  uint8_t data[8];
  uint32_t frame_info;
  uint32_t len;
  uint32_t datastart;
  uint32_t regval;
  uint32_t i;

  /* Read the interrupt register results in clearing bits  */

  regval = twai_getreg(TWAI_INT_RAW_REG);

  /* Check for a receive interrupt */

  if ((regval & TWAI_RX_INT_ST_M) != 0)
    {
      memset(&hdr, 0, sizeof(hdr));
      memset(data, 0, sizeof(data));

      frame_info = twai_getreg(TWAI_DATA_0_REG);

      /* Construct the TWAI header */

      if (frame_info & (1 << 6))
        {
          hdr.ch_rtr    = 1;
        }

#ifdef CONFIG_CAN_EXTID
      if (frame_info & (1 << 7))
        {
          /* The provided ID should be 29 bits */

          hdr.ch_extid = 1;
          hdr.ch_id =
          (twai_getreg(TWAI_DATA_1_REG) << 21) +
          (twai_getreg(TWAI_DATA_2_REG) << 13) +
          (twai_getreg(TWAI_DATA_3_REG) << 5) +
          (twai_getreg(TWAI_DATA_4_REG) >> 3);
          datastart = TWAI_DATA_5_REG;
        }
      else
#endif
        {
          /* The provided ID should be 11 bits */

          hdr.ch_id =
          (twai_getreg(TWAI_DATA_1_REG) << 3) +
          (twai_getreg(TWAI_DATA_2_REG) >> 5);
          datastart = TWAI_DATA_3_REG;
        }

      len = frame_info & 0xf;
      if (len > CAN_MAXDATALEN) len = CAN_MAXDATALEN;
      hdr.ch_dlc = len;

      for (i = 0; i < len; i++)
        {
          data[i] = twai_getreg(datastart + (i * 4));
        }

      /* Release the receive buffer */

      twai_putreg(TWAI_CMD_REG, TWAI_RELEASE_BUF_M);

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error  = 0; /* Error reporting not supported */
#endif
      can_receive(dev, &hdr, data);
    }

  /* Check for TX buffer complete */

  if ((regval & TWAI_TX_INT_ST_M) != 0)
    {
      /* Disable all further TX buffer interrupts */

      regval  = twai_getreg(TWAI_INT_ENA_REG);
      regval &= ~TWAI_TX_INT_ENA_M;
      twai_putreg(TWAI_INT_ENA_REG, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

#endif
  return OK;
}

/****************************************************************************
 * Name: esp32twai_set_acc_filter
 *
 * Description:
 *   Call to set acceptance filter.
 *   Must be called in reset mode.
 *
 * Input Parameters:
 *   code - Acceptance Code.
 *   mask - Acceptance Mask.
 *   single_filter - Whether to enable single filter mode.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32twai_set_acc_filter(uint32_t code, uint32_t mask,
                                     bool single_filter)
{
  uint32_t regval;
  uint32_t code_swapped = __builtin_bswap32(code);
  uint32_t mask_swapped = __builtin_bswap32(mask);

  regval = twai_getreg(TWAI_MODE_REG);
  if (single_filter)
    {
      regval |= TWAI_RX_FILTER_MODE_M;
    }
  else
    {
      regval &= ~(TWAI_RX_FILTER_MODE_M);
    }

  twai_putreg(TWAI_MODE_REG, regval);

  for (int i = 0; i < 4; i++)
    {
      twai_putreg(TWAI_DATA_0_REG + (i * 4),
                  ((code_swapped >> (i * 8)) & 0xff));
      twai_putreg(TWAI_DATA_4_REG + (i * 4),
                  ((mask_swapped >> (i * 8)) & 0xff));
    }
}

/****************************************************************************
 * Name: twai_baud_rate
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

static int twai_baud_rate(struct twai_dev_s *priv, int rate, int clock,
                          int sjw, int sampl_pt, int flags)
{
  const struct can_bittiming_const *timing =
      &esp32_twai_bittiming_const;
  int best_error = 1000000000;
  int error;
  int best_tseg = 0;
  int best_brp = 0;
  int best_rate = 0;
  int brp = 0;
  int tseg = 0;
  int tseg1 = 0;
  int tseg2 = 0;
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
          best_tseg = tseg / 2;
          best_brp = brp;
          best_rate = clock / (brp * (1 + tseg / 2));
        }
    }

  if (best_error && (rate / best_error < 10))
    {
      canerr("baud rate %d is not possible with %d Hz clock\n",
              rate, clock);
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

  caninfo("TS1: %d TS2: %d BRP: %d\n", tseg1, tseg2, best_brp);

  /* Configure bit timing */

  timing0 = (best_brp / 2) - 1;
  timing0 |= (sjw - 1) << TWAI_SYNC_JUMP_WIDTH_S;

  timing1 = tseg1 - 1;
  timing1 |= (tseg2 - 1) << TWAI_TIME_SEG2_S;

#ifdef CONFIG_ESP32_TWAI0_SAM
  /* The bus is sampled 3 times (recommended for low to medium speed buses
   * to spikes on the bus-line).
   */

  timing1 |= CONFIG_ESP32_TWAI0_SAM << TWAI_TIME_SAMP_S;
#endif

  twai_putreg(TWAI_BUS_TIMING_0_REG, timing0);
  twai_putreg(TWAI_BUS_TIMING_1_REG, timing1);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_twaiinitialize
 *
 * Description:
 *   Initialize the selected TWAI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Valid TWAI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *esp32_twaiinitialize(int port)
{
  struct can_dev_s *twaidev;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 "\n",  port);

#ifdef CONFIG_ESP32_TWAI0
  if (port == 0)
    {
      twaidev = &g_twai0dev;

      flags = spin_lock_irqsave(&g_twai0priv.lock);

      /* Enable power to the TWAI module and
       * Enable clocking to the TWAI module
       */

      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_TWAI_CLK_EN);
      modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_TWAI_RST, 0);

      /* Configure CAN GPIO pins */

      esp32_gpio_matrix_out(CONFIG_ESP32_TWAI0_TXPIN, TWAI_TX_IDX, 0, 0);
      esp32_configgpio(CONFIG_ESP32_TWAI0_TXPIN, OUTPUT_FUNCTION_1);

      esp32_configgpio(CONFIG_ESP32_TWAI0_RXPIN, INPUT_FUNCTION_1);
      esp32_gpio_matrix_in(CONFIG_ESP32_TWAI0_RXPIN, TWAI_RX_IDX, 0);

      spin_unlock_irqrestore(&g_twai0priv.lock, flags);
    }
  else
#endif

    {
      canerr("ERROR: Unsupported port: %d\n", port);

      return NULL;
    }

  /* Then just perform a TWAI reset operation */

  esp32twai_reset(twaidev);

  return twaidev;
}
#endif
