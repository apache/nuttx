/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_twai.c
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
#include <nuttx/spinlock.h>
#include <nuttx/can/can.h>
#include <nuttx/signal.h>

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_twai.h"
#include "esp_irq.h"
#include "esp_clk.h"

#include "periph_ctrl.h"
#include "hal/twai_hal.h"
#include "hal/twai_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/reg_base.h"

#if defined(CONFIG_ESPRESSIF_TWAI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#  if defined(CONFIG_CAN_LOOPBACK) && defined(CONFIG_ESPRESSIF_TWAI_TEST_MODE)
#   define TX_PIN_ATTR (OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1)
#   define RX_PIN_ATTR (OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1)
#  else
#   define TX_PIN_ATTR OUTPUT_FUNCTION_1
#   define RX_PIN_ATTR INPUT_FUNCTION_1
#  endif

#  ifdef CONFIG_ESPRESSIF_TWAI0
#   ifdef CONFIG_TWAI0_TIMING_100KBITS
      #define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_100KBITS()
#   elif CONFIG_TWAI0_TIMING_125KBITS
      #define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#   elif CONFIG_TWAI0_TIMING_250KBITS
      #define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_250KBITS()
#   elif CONFIG_TWAI0_TIMING_500KBITS
      #define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_500KBITS()
#   else
      #define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_800KBITS()
#   endif
#  endif

#  ifdef CONFIG_ESPRESSIF_TWAI1
#   ifdef CONFIG_TWAI1_TIMING_100KBITS
      #define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_100KBITS()
#   elif CONFIG_TWAI1_TIMING_125KBITS
      #define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#   elif CONFIG_TWAI1_TIMING_250KBITS
      #define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_250KBITS()
#   elif CONFIG_TWAI1_TIMING_500KBITS
      #define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_500KBITS()
#   else
      #define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_800KBITS()
#   endif
#  endif

#  ifdef CONFIG_ESPRESSIF_ESP32C3
#    define INT_ENA_REG(hw)       hw->interrupt_enable_reg.val
#    define PERIPH_TWAI0_MODULE   PERIPH_TWAI_MODULE
#    define TWAI0_TX_IDX          TWAI_TX_IDX
#    define TWAI0_RX_IDX          TWAI_RX_IDX
#    define ETS_TWAI0_INTR_SOURCE ETS_TWAI_INTR_SOURCE
#    define ESP_IRQ_TWAI0         ESP_IRQ_TWAI
#  else
#    define INT_ENA_REG(hw)       hw->interrupt_enable.val
#  endif /* CONFIG_ESPRESSIF_ESP32C3 */

#  ifdef CONFIG_ESPRESSIF_ESP32H2
#    define TWAI0_TX_IDX          TWAI_TX_IDX
#    define TWAI0_RX_IDX          TWAI_RX_IDX
#  endif /* CONFIG_ESPRESSIF_ESP32H2 */

/* Configuration ************************************************************/

#  ifndef CONFIG_CAN_EXTID
#   define EXTID 0
#  else
#   define EXTID 1
#  endif

#  ifndef CONFIG_CAN_LOOPBACK
#   define LOOPBACK 0
#  else
#   define LOOPBACK 1
#  endif

/* Default values written to various registers on initialization */

#  define TWAI_DEFAULT_INTERRUPTS   0xe7  /* Exclude data overrun (bit[3]) and brp_div (bit[4]) */

struct esp_twai_dev_s
{
  /* Device configuration */

  uint8_t port;                   /* TWAI port number */
  uint8_t periph;                 /* Peripheral ID */
  uint8_t irq;                    /* IRQ associated with this TWAI */
  int8_t cpuint;                  /* CPU interrupt assigned to this TWAI */
  twai_hal_context_t ctx;         /* Context struct of common layer */
  twai_timing_config_t t_config;  /* Timing struct of common layer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TWAI methods */

static void esp_twai_reset(struct can_dev_s *dev);
static int  esp_twai_setup(struct can_dev_s *dev);
static void esp_twai_shutdown(struct can_dev_s *dev);
static void esp_twai_rxint(struct can_dev_s *dev, bool enable);
static void esp_twai_txint(struct can_dev_s *dev, bool enable);
static int  esp_twai_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  esp_twai_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  esp_twai_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool esp_twai_txready(struct can_dev_s *dev);
static bool esp_twai_txempty(struct can_dev_s *dev);

/* TWAI interrupts */

static int esp_twai_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_twaiops =
{
  .co_reset         = esp_twai_reset,
  .co_setup         = esp_twai_setup,
  .co_shutdown      = esp_twai_shutdown,
  .co_rxint         = esp_twai_rxint,
  .co_txint         = esp_twai_txint,
  .co_ioctl         = esp_twai_ioctl,
  .co_remoterequest = esp_twai_remoterequest,
  .co_send          = esp_twai_send,
  .co_txready       = esp_twai_txready,
  .co_txempty       = esp_twai_txempty,
};

#ifdef CONFIG_ESPRESSIF_TWAI0
static struct esp_twai_dev_s g_twai0priv =
{
  .port             = 0,
  .periph           = ETS_TWAI0_INTR_SOURCE,
  .irq              = ESP_IRQ_TWAI0,
  .cpuint           = -ENOMEM,
  .t_config         = TWAI0_TIMING_CONFIG,
};

static struct can_dev_s g_twai0dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai0priv,
};
#endif /* CONFIG_ESPRESSIF_TWAI0 */

#ifdef CONFIG_ESPRESSIF_TWAI1
static struct esp_twai_dev_s g_twai1priv =
{
  .port             = 1,
  .periph           = ETS_TWAI1_INTR_SOURCE,
  .irq              = ESP_IRQ_TWAI1,
  .cpuint           = -ENOMEM,
  .t_config         = TWAI1_TIMING_CONFIG,
};

static struct can_dev_s g_twai1dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai1priv,
};
#endif /* CONFIG_ESPRESSIF_TWAI1 */

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_twai_reset
 *
 * Description:
 *   Reset the TWAI device. Called early to initialize the hardware. This
 *   function is called, before esp_twai_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void esp_twai_reset(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;
  twai_hal_config_t hal_config =
    {
      .clock_source_hz = esp_clk_apb_freq(),
      .controller_id = priv->port,
    };

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  flags = enter_critical_section();

  ret = twai_hal_init(&priv->ctx, &hal_config);
  assert(ret);
  twai_hal_configure(&priv->ctx, &priv->t_config, &f_config,
                     TWAI_DEFAULT_INTERRUPTS, 0);

  /* Restart the TWAI */

#ifdef CONFIG_CAN_LOOPBACK
  twai_hal_start(&priv->ctx, TWAI_MODE_NO_ACK); /* Leave Reset Mode, enter Test Mode */
#else
  twai_hal_start(&priv->ctx, TWAI_MODE_NORMAL); /* Leave Reset Mode */
#endif

  /* Abort transmission, release RX buffer and clear overrun.
   * Command register can only be modified when in Operation Mode.
   */

  twai_ll_set_cmd_release_rx_buffer(priv->ctx.dev);
  twai_ll_set_cmd_abort_tx(priv->ctx.dev);
  twai_ll_set_cmd_clear_data_overrun(priv->ctx.dev);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_twai_setup
 *
 * Description:
 *   Configure the TWAI. This method is called the first time that the TWAI
 *   the device is opened and it configures and attaches the TWAI interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp_twai_setup(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret = OK;

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  flags = enter_critical_section();

  twai_ll_set_enabled_intrs(priv->ctx.dev, TWAI_DEFAULT_INTERRUPTS);

  twai_ll_get_and_clear_intrs(priv->ctx.dev); /* clear latched interrupts */

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->irq);
    }

  priv->cpuint = esp_setup_irq(priv->periph,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = priv->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  ret = irq_attach(priv->irq, esp_twai_interrupt, dev);
  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp_teardown_irq(priv->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the TWAI device. */

  up_enable_irq(priv->irq);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp_twai_shutdown
 *
 * Description:
 *   Disable the TWAI. This method is called when the TWAI device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_twai_shutdown(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;

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

      esp_teardown_irq(priv->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
    }
}

/****************************************************************************
 * Name: esp_twai_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev    - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable receive interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_twai_rxint(struct can_dev_s *dev, bool enable)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  /* The INT_ENA register is also modified from the interrupt handler,
   * so we have to protect this code section.
   */

  flags = enter_critical_section();
  regval = twai_ll_get_and_clear_intrs(priv->ctx.dev);
  if (enable == true)
    {
      regval |= TWAI_LL_INTR_RI;
    }
  else
    {
      regval &= ~TWAI_LL_INTR_RI;
    }

  twai_ll_set_enabled_intrs(priv->ctx.dev, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_twai_txint
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

static void esp_twai_txint(struct can_dev_s *dev, bool enable)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (enable == false)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

      flags = enter_critical_section();

      /* Disable all TX interrupts */

      regval = INT_ENA_REG(priv->ctx.dev);
      regval &= ~TWAI_LL_INTR_TI;
      twai_ll_set_enabled_intrs(priv->ctx.dev, regval);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: esp_twai_ioctl
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

static int esp_twai_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
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

          brp = (priv->t_config.brp + 1) * 2;
          bt->bt_sjw = priv->t_config.sjw + 1;
          bt->bt_tseg1 = priv->t_config.tseg_1 + 1;
          bt->bt_tseg2 = priv->t_config.tseg_2 + 1;

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

/****************************************************************************
 * Name: esp_twai_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *   id  - Requested 11-bit data frame identifier
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp_twai_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canwarn("Remote request not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_twai_send
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

static int esp_twai_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  uint32_t i;
  uint32_t len;
  uint32_t id;
  uint32_t twai_flags;
  irqstate_t flags;
  int ret = OK;

  caninfo("TWAI%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
  if (len > CAN_MAXDATALEN)
    {
      len = CAN_MAXDATALEN;
    }

  flags = enter_critical_section();

  /* Make sure that TX interrupts are enabled BEFORE sending the
   * message.
   *
   * NOTE: The INT_ENA is also modified from the interrupt handler, but the
   * following is safe because interrupts are disabled here.
   */

  regval = INT_ENA_REG(priv->ctx.dev);
  regval |= TWAI_LL_INTR_TI;
  twai_ll_set_enabled_intrs(priv->ctx.dev, regval);

  twai_hal_frame_t tx_frame;

  /* Adjustments from NuttX TWAI message struct to common layer TWAI struct */

  twai_flags = ((EXTID << TWAI_MSG_FLAG_EXTD) |
                (msg->cm_hdr.ch_rtr << TWAI_MSG_FLAG_RTR) |
                (LOOPBACK << TWAI_MSG_FLAG_SELF));

  /* Set up the transfer */

  twai_ll_format_frame_buffer(msg->cm_hdr.ch_id, len,
                              msg->cm_data, twai_flags, &tx_frame);
  twai_ll_set_tx_buffer(priv->ctx.dev, &tx_frame);

  /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
  twai_ll_set_cmd_self_rx_request(priv->ctx.dev);
#else
  twai_ll_set_cmd_tx(priv->ctx.dev);
#endif

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp_twai_txready
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

static bool esp_twai_txready(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_ll_get_status(priv->ctx.dev);
  caninfo("TWAI%" PRIu8 " txready: %d\n", priv->port,
         ((regval & TWAI_LL_STATUS_TBS) != 0));
  return ((regval & TWAI_LL_STATUS_TBS) != 0);
}

/****************************************************************************
 * Name: esp_twai_txempty
 *
 * Description:
 *   Return true if all message have been sent. If for example, the TWAI
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

static bool esp_twai_txempty(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_ll_get_status(priv->ctx.dev);

  caninfo("TWAI%" PRIu8 " txempty: %d\n", priv->port,
         ((regval & TWAI_LL_STATUS_TBS) != 0));
  return ((regval & TWAI_LL_STATUS_TBS) != 0);
}

/****************************************************************************
 * Name: esp_twai_interrupt
 *
 * Description:
 *   TWAI RX/TX interrupt handler
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

static int esp_twai_interrupt(int irq, void *context, void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct esp_twai_dev_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[8];
  uint32_t regval;
  twai_hal_frame_t rx_frame;
  uint32_t flags = 0;
  uint32_t id;
  uint8_t dlc;

  /* Read the interrupt register results in clearing bits */

  regval = twai_ll_get_and_clear_intrs(priv->ctx.dev);

  /* Check for a receive interrupt */

  if ((regval & TWAI_LL_INTR_RI) != 0)
    {
      memset(&hdr, 0, sizeof(hdr));
      memset(data, 0, sizeof(data));

      twai_ll_get_rx_buffer(priv->ctx.dev, &rx_frame);

      /* Release the receive buffer */

      twai_ll_set_cmd_release_rx_buffer(priv->ctx.dev);
      twai_ll_parse_frame_buffer(&rx_frame, &id, &dlc, data, &flags);
      hdr.ch_id = id;
      hdr.ch_dlc = dlc;
      hdr.ch_rtr = (flags && TWAI_MSG_FLAG_RTR) ? 1 : 0;

      can_receive(dev, &hdr, data);
    }

  /* Check for TX buffer complete */

  if ((regval & TWAI_LL_INTR_TI) != 0)
    {
      /* Disable all further TX buffer interrupts */

      esp_twai_txint(dev, false);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_twaiinitialize
 *
 * Description:
 *   Initialize TWAI peripheral
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Valid TWAI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *esp_twaiinitialize(int port)
{
  struct can_dev_s *dev;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 "\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_ESPRESSIF_TWAI0
  if (port == 0)
    {
      /* Enable power to the TWAI module and
       * Enable clocking to the TWAI module
       */

      periph_module_reset(PERIPH_TWAI0_MODULE);
      periph_module_enable(PERIPH_TWAI0_MODULE);

      /* Configure CAN GPIO pins */

      esp_gpio_matrix_out(CONFIG_ESPRESSIF_TWAI0_TXPIN, TWAI0_TX_IDX, 0, 0);
      esp_configgpio(CONFIG_ESPRESSIF_TWAI0_TXPIN, TX_PIN_ATTR);

      esp_configgpio(CONFIG_ESPRESSIF_TWAI0_RXPIN, RX_PIN_ATTR);
      esp_gpio_matrix_in(CONFIG_ESPRESSIF_TWAI0_RXPIN, TWAI0_RX_IDX, 0);

      dev = &g_twai0dev;
    }
  else
#endif

#ifdef CONFIG_ESPRESSIF_TWAI1
  if (port == 1)
    {
      /* Enable power to the TWAI module and
       * Enable clocking to the TWAI module
       */

      periph_module_reset(PERIPH_TWAI1_MODULE);
      periph_module_enable(PERIPH_TWAI1_MODULE);

      /* Configure CAN GPIO pins */

      esp_gpio_matrix_out(CONFIG_ESPRESSIF_TWAI1_TXPIN, TWAI1_TX_IDX, 0, 0);
      esp_configgpio(CONFIG_ESPRESSIF_TWAI1_TXPIN, TX_PIN_ATTR);

      esp_configgpio(CONFIG_ESPRESSIF_TWAI1_RXPIN, RX_PIN_ATTR);
      esp_gpio_matrix_in(CONFIG_ESPRESSIF_TWAI1_RXPIN, TWAI1_RX_IDX, 0);

      dev = &g_twai1dev;
    }
  else
#endif

    {
      canerr("ERROR: Unsupported port: %d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

  /* Then just perform a TWAI reset operation */

  esp_twai_reset(dev);

  leave_critical_section(flags);

  return dev;
}
#endif /* CONFIG_ESPRESSIF_TWAI */
