/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hpm6750.h"
#include "hpm6750_config.h"
#include "hpm6750_iomux.h"
#include "hpm6750_pmic_iomux.h"
#include "hpm6750_lowputc.h"
#include "hpm6750_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */
#ifdef USE_SERIALDRIVER
/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  if defined(CONFIG_HPM6750_UART0)
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#    undef  TTYS1_DEV
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */
#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t  uartbase;          /* Base address of UART registers */
  uint32_t  irq;               /* IRQ associated with this UART */
  uint32_t  im;                /* Interrupt mask state */
  uart_config_t  config;       /* Uart config */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_HPM6750_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct up_dev_s g_uart0priv =
{
  .uartbase  = HPM6750_UART0_BASE,
  .irq       = HPM6750_IRQ_UART0,
  .im        = 0,
  .config =
    {
      .src_freq_in_hz = 24000000,
      .baudrate = 115200,
      .num_of_stop_bits = stop_bits_1,
      .word_length = word_length_8_bits,
      .parity = parity_none,
      .rx_fifo_level = uart_rx_fifo_trg_not_empty,
      .tx_fifo_level = uart_tx_fifo_trg_not_full,
      .fifo_enable = true,
      .dma_enable = false,
      .modem_config =
        {
          .auto_flow_ctrl_en = false,
          .loop_back_en = false,
          .set_rts_high = false,
        },
    },
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialmodify
 ****************************************************************************/

static void up_serialmodfiy(struct up_dev_s *priv, int offset,
                            uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->uartbase + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t im)
{
  irqstate_t flags = enter_critical_section();

  priv->im = im;
  up_serialout(priv, UART_IER_OFFSET, im);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint8_t *im)
{
  irqstate_t flags = enter_critical_section();

  /* Return the current interrupt mask value */

  if (im)
    {
     *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
  up_serialout(priv, UART_IER_OFFSET, 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm6750_init_uart_pins
 ****************************************************************************/

static void hpm6750_init_uart_pins(struct up_dev_s *priv)
{
  if (priv->uartbase == HPM6750_UART0_BASE)
    {
      putreg32(IOC_PY06_FUNC_CTL_UART0_TXD, IOC_PAD_PY06_FUNC_CTL);
      putreg32(IOC_PY07_FUNC_CTL_UART0_RXD, IOC_PAD_PY07_FUNC_CTL);
      putreg32(IOC_PY06_FUNC_CTL_SOC_PY_06, PIOC_PAD_PY06_FUNC_CTL);
      putreg32(IOC_PY07_FUNC_CTL_SOC_PY_07, PIOC_PAD_PY07_FUNC_CTL);
    }
}

/****************************************************************************
 * Name: hpm6750_uart_calculate_baudrate
 ****************************************************************************/

#define HPM_UART_MINIMUM_BAUDRATE (200U)
#define HPM_UART_BAUDRATE_TOLERANCE (3)
#define HPM_UART_OSC_MAX (32U)
#define HPM_UART_OSC_MIN (8U)
#define HPM_UART_BAUDRATE_DIV_MAX (0xFFFFU)
#define HPM_UART_BAUDRATE_DIV_MIN (1U)

static bool hpm6750_uart_calculate_baudrate(uint32_t freq, uint32_t baudrate,
                                         uint16_t *div_out, uint8_t *osc_out)
{
  uint16_t div;
  uint16_t osc;
  uint16_t delta;
  float tmp;

  if ((div_out == NULL) || (!freq) || (!baudrate)
    || (baudrate < HPM_UART_MINIMUM_BAUDRATE)
    || (freq / HPM_UART_BAUDRATE_DIV_MIN < baudrate * HPM_UART_OSC_MIN)
    || (freq / HPM_UART_BAUDRATE_DIV_MAX > (baudrate * HPM_UART_OSC_MAX)))
    {
      return 0;
    }

  tmp = (float) freq / baudrate;
  for (uint8_t i = 0; i < HPM_UART_OSC_MAX; i += 2)
    {
      /* osc range: 0 - 32, even number */

      if (i == 0)
        {
          /* osc == 0 in bitfield, oversample rate is 32 */

          osc = HPM_UART_OSC_MAX;
        }
      else if (i <= 8)
        {
          /* osc <= 8 in bitfield, oversample rate is 8 */

          osc = HPM_UART_OSC_MIN;
        }
      else
        {
          /* osc > 8 && osc < 32 in bitfield, oversample rate is osc */

          osc = i;
        }

      delta = 0;
      div = (uint16_t)(tmp / osc);
      if (div < HPM_UART_BAUDRATE_DIV_MIN)
        {
          /* invalid div */

          continue;
        }

      if (div * osc > tmp)
        {
          delta = div * osc - tmp;
        }
      else if (div * osc < tmp)
        {
          delta = tmp - div * osc;
        }
      else
        {
          /* Do Nothing */
        }

      if (delta && ((delta * 100 / tmp) > HPM_UART_BAUDRATE_TOLERANCE))
        {
          continue;
        }
      else
        {
          *div_out = div;
          *osc_out = (i <= 8 && i) ? osc : i;
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: hpm6750_uart_modem_config
 ****************************************************************************/

static void hpm6750_uart_modem_config(struct up_dev_s *priv,
                                     uart_modem_config_t *config)
{
  uint32_t value;

  value = UART_MCR_AFE_SET(config->auto_flow_ctrl_en)
        | UART_MCR_LOOP_SET(config->loop_back_en)
        | UART_MCR_RTS_SET(!config->set_rts_high);

  up_serialout(priv, UART_MCR_OFFSET, value);
}

/****************************************************************************
 * Name: hpm6750_uart_init
 ****************************************************************************/

static void hpm6750_uart_init(struct up_dev_s *priv)
{
  uint32_t tmp;
  uint8_t osc;
  uint16_t div;
  uart_config_t *config;

  config = &priv->config;

  /* disable all interrupts */

  up_serialout(priv, UART_IER_OFFSET, 0);

  /* Set DLAB to 1 */

  up_serialmodfiy(priv, UART_LCR_OFFSET, 0, UART_LCR_DLAB_MASK);

  (void)hpm6750_uart_calculate_baudrate(config->src_freq_in_hz,
                                        config->baudrate, &div, &osc);

  up_serialmodfiy(priv, UART_OSCR_OFFSET,
                  UART_OSCR_OSC_MASK, UART_OSCR_OSC_SET(osc));
  up_serialout(priv, UART_DLL_OFFSET, UART_DLL_DLL_SET(div >> 0));
  up_serialout(priv, UART_DLM_OFFSET, UART_DLM_DLM_SET(div >> 8));

  /* DLAB bit needs to be cleared once baudrate is configured */

  up_serialmodfiy(priv, UART_LCR_OFFSET, UART_LCR_DLAB_MASK, 0);

  tmp = up_serialin(priv, UART_LCR_OFFSET);
  tmp &= ~(UART_LCR_SPS_MASK | UART_LCR_EPS_MASK | UART_LCR_PEN_MASK);
  switch (config->parity)
    {
      case parity_none:
        break;
      case parity_odd:
        tmp |= UART_LCR_PEN_MASK;
        break;
      case parity_even:
        tmp |= UART_LCR_PEN_MASK | UART_LCR_EPS_MASK;
        break;
      case parity_always_1:
        tmp |= UART_LCR_PEN_MASK | UART_LCR_SPS_MASK;
        break;
      case parity_always_0:
        tmp |= UART_LCR_EPS_MASK | UART_LCR_PEN_MASK | UART_LCR_SPS_MASK;
        break;
      default:
        break;
    }

  tmp &= ~UART_LCR_STB_MASK;
  switch (config->num_of_stop_bits)
    {
      case stop_bits_1:
        break;
      case stop_bits_1_5:
      case stop_bits_2:
        tmp |= UART_LCR_STB_MASK;
        break;
      default:
        break;
    }

  tmp &= ~UART_LCR_WLS_MASK;
  tmp |= UART_LCR_WLS_SET(config->word_length);
  up_serialout(priv, UART_LCR_OFFSET, tmp);
  up_serialout(priv, UART_FCR_OFFSET,
               UART_FCR_TFIFORST_MASK | UART_FCR_RFIFORST_MASK);
  if (config->fifo_enable)
    {
      /* Enable FIFO, reset TX and RX. */

      tmp = UART_FCR_TFIFORST_MASK
          | UART_FCR_RFIFORST_MASK | UART_FCR_FIFOE_MASK
          | UART_FCR_TFIFOT_SET(config->tx_fifo_level)
          | UART_FCR_RFIFOT_SET(config->rx_fifo_level)
          | UART_FCR_DMAE_SET(config->dma_enable);
      up_serialout(priv, UART_FCR_OFFSET, tmp);
    }

  hpm6750_uart_modem_config(priv, &config->modem_config);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  hpm6750_init_uart_pins(priv);

  hpm6750_uart_init(priv);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, up_interrupt, dev);

  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t irq_id;

  irq_id = up_serialin(priv, UART_IIR_OFFSET) & UART_IIR_INTRID_MASK;

  /* Length of uart rx data transfer arrived interrupt */

  if (irq_id == uart_intr_id_rx_data_avail)
    {
      /* Receive Data ready */

      uart_recvchars(dev);
    }

  /* Tx fifo ready interrupt,auto-cleared when data is pushed */

  if (irq_id == uart_intr_id_tx_slot_avail)
    {
      /* Transmit data request interrupt */

      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int rxdata;

  if ((up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_DR_MASK) != 0)
    {
      rxdata = (int)(up_serialin(priv, UART_RBR_OFFSET) & UART_RBR_RBR_MASK);
      *status = 0;
    }
  else
    {
      *status = -1;
    }

  return rxdata;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ERBI_MASK;
#endif
    }
  else
    {
      priv->im &= ~UART_IER_ERBI_MASK;
    }

  up_serialout(priv, UART_IER_OFFSET, priv->im);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_DR_MASK) != 0)
         ? true : false;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ETHEI_MASK;
      up_serialout(priv, UART_IER_OFFSET, priv->im);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_IER_ETHEI_MASK;
      up_serialout(priv, UART_IER_OFFSET, priv->im);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX FIFO is not full */

  return (up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE_MASK)
         ? true : false;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX wartermak is pending */

  return (up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_TEMT_MASK)
         ? true : false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint8_t imr;

  up_disableuartint(priv, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  up_restoreuartint(priv, imr);
#endif
  return ch;
}

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
