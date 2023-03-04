/***************************************************************************
 * arch/arm64/src/qemu/qemu_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "qemu_serial.h"
#include "arm64_arch_timer.h"
#include "qemu_boot.h"
#include "arm64_gic.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-5 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port         /* UART1 is console */
#  define TTYS0_DEV       g_uart1port         /* UART1 is ttyS0 */
#  define UART1_ASSIGNED  1
#endif

#define PL011_BIT_MASK(x, y)  (((2 << (x)) - 1) << (y))

/* PL011 Uart Flags Register */
#define PL011_FR_CTS                    BIT(0)  /* clear to send - inverted */
#define PL011_FR_DSR                    BIT(1)  /* data set ready - inverted
                                                 */
#define PL011_FR_DCD                    BIT(2)  /* data carrier detect -
                                                 * inverted */
#define PL011_FR_BUSY                   BIT(3)  /* busy transmitting data */
#define PL011_FR_RXFE                   BIT(4)  /* receive FIFO empty */
#define PL011_FR_TXFF                   BIT(5)  /* transmit FIFO full */
#define PL011_FR_RXFF                   BIT(6)  /* receive FIFO full */
#define PL011_FR_TXFE                   BIT(7)  /* transmit FIFO empty */
#define PL011_FR_RI                     BIT(8)  /* ring indicator - inverted */

/* PL011 Integer baud rate register */
#define PL011_IBRD_BAUD_DIVINT_MASK     0xff /* 16 bits of divider */

/* PL011 Fractional baud rate register */
#define PL011_FBRD_BAUD_DIVFRAC         0x3f
#define PL011_FBRD_WIDTH                6u

/* PL011 Receive status register / error clear register */
#define PL011_RSR_ECR_FE                BIT(0)  /* framing error */
#define PL011_RSR_ECR_PE                BIT(1)  /* parity error */
#define PL011_RSR_ECR_BE                BIT(2)  /* break error */
#define PL011_RSR_ECR_OE                BIT(3)  /* overrun error */

#define PL011_RSR_ERROR_MASK            (PL011_RSR_ECR_FE | PL011_RSR_ECR_PE | \
                                         PL011_RSR_ECR_BE | PL011_RSR_ECR_OE)

/* PL011 Line Control Register  */
#define PL011_LCRH_BRK                  BIT(0)  /* send break */
#define PL011_LCRH_PEN                  BIT(1)  /* enable parity */
#define PL011_LCRH_EPS                  BIT(2)  /* select even parity */
#define PL011_LCRH_STP2                 BIT(3)  /* select two stop bits */
#define PL011_LCRH_FEN                  BIT(4)  /* enable FIFOs */
#define PL011_LCRH_WLEN_SHIFT           5       /* word length */
#define PL011_LCRH_WLEN_WIDTH           2
#define PL011_LCRH_SPS                  BIT(7)  /* stick parity bit */

#define PL011_LCRH_WLEN_SIZE(x)         ((x) - 5)

#define PL011_LCRH_FORMAT_MASK          (PL011_LCRH_PEN | PL011_LCRH_EPS |     \
                                         PL011_LCRH_SPS |                      \
                                         PL011_BIT_MASK(PL011_LCRH_WLEN_WIDTH, \
                                                        PL011_LCRH_WLEN_SHIFT))

#define PL011_LCRH_PARTIY_EVEN          (PL011_LCRH_PEN | PL011_LCRH_EPS)
#define PL011_LCRH_PARITY_ODD           (PL011_LCRH_PEN)
#define PL011_LCRH_PARITY_NONE          (0)

/* PL011 Control Register */
#define PL011_CR_UARTEN                 BIT(0)  /* enable uart operations */
#define PL011_CR_SIREN                  BIT(1)  /* enable IrDA SIR */
#define PL011_CR_SIRLP                  BIT(2)  /* IrDA SIR low power mode */
#define PL011_CR_LBE                    BIT(7)  /* loop back enable */
#define PL011_CR_TXE                    BIT(8)  /* transmit enable */
#define PL011_CR_RXE                    BIT(9)  /* receive enable */
#define PL011_CR_DTR                    BIT(10) /* data transmit ready */
#define PL011_CR_RTS                    BIT(11) /* request to send */
#define PL011_CR_Out1                   BIT(12)
#define PL011_CR_Out2                   BIT(13)
#define PL011_CR_RTSEn                  BIT(14) /* RTS hw flow control enable
                                                 */
#define PL011_CR_CTSEn                  BIT(15) /* CTS hw flow control enable
                                                 */

/* PL011 Interrupt Fifo Level Select Register */
#define PL011_IFLS_TXIFLSEL_SHIFT       0   /* bits 2:0 */
#define PL011_IFLS_TXIFLSEL_WIDTH       3
#define PL011_IFLS_RXIFLSEL_SHIFT       3   /* bits 5:3 */
#define PL011_IFLS_RXIFLSEL_WIDTH       3

/* PL011 Interrupt Mask Set/Clear Register */
#define PL011_IMSC_RIMIM                BIT(0)  /* RTR modem interrupt mask */
#define PL011_IMSC_CTSMIM               BIT(1)  /* CTS modem interrupt mask */
#define PL011_IMSC_DCDMIM               BIT(2)  /* DCD modem interrupt mask */
#define PL011_IMSC_DSRMIM               BIT(3)  /* DSR modem interrupt mask */
#define PL011_IMSC_RXIM                 BIT(4)  /* receive interrupt mask */
#define PL011_IMSC_TXIM                 BIT(5)  /* transmit interrupt mask */
#define PL011_IMSC_RTIM                 BIT(6)  /* receive timeout interrupt
                                                 * mask */
#define PL011_IMSC_FEIM                 BIT(7)  /* framing error interrupt
                                                 * mask */
#define PL011_IMSC_PEIM                 BIT(8)  /* parity error interrupt mask
                                                 */
#define PL011_IMSC_BEIM                 BIT(9)  /* break error interrupt mask
                                                 */
#define PL011_IMSC_OEIM                 BIT(10) /* overrun error interrupt
                                                 * mask */

#define PL011_IMSC_ERROR_MASK           (PL011_IMSC_FEIM |                   \
                                         PL011_IMSC_PEIM | PL011_IMSC_BEIM | \
                                         PL011_IMSC_OEIM)

#define PL011_IMSC_MASK_ALL             (PL011_IMSC_OEIM | PL011_IMSC_BEIM | \
                                         PL011_IMSC_PEIM | PL011_IMSC_FEIM | \
                                         PL011_IMSC_RIMIM |                  \
                                         PL011_IMSC_CTSMIM |                 \
                                         PL011_IMSC_DCDMIM |                 \
                                         PL011_IMSC_DSRMIM |                 \
                                         PL011_IMSC_RXIM | PL011_IMSC_TXIM | \
                                         PL011_IMSC_RTIM)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* UART PL011 register map structure */

struct pl011_regs
{
  uint32_t dr;   /* data register */
  union
  {
    uint32_t rsr;
    uint32_t ecr;
  };

  uint32_t reserved_0[4];
  uint32_t fr;   /* flags register */
  uint32_t reserved_1;
  uint32_t ilpr;
  uint32_t ibrd;
  uint32_t fbrd;
  uint32_t lcr_h;
  uint32_t cr;
  uint32_t ifls;
  uint32_t imsc;
  uint32_t ris;
  uint32_t mis;
  uint32_t icr;
  uint32_t dmacr;
};

struct pl011_config
{
  volatile struct pl011_regs *uart;
  uint32_t sys_clk_freq;
};

/* Device data structure */

struct pl011_data
{
  uint32_t baud_rate;
  bool sbsa;
};

struct pl011_uart_port_s
{
  struct pl011_data data;
  struct pl011_config config;
  unsigned int irq_num;
  bool is_console;
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static void pl011_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->cr |= PL011_CR_UARTEN;
}

static void pl011_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->cr &= ~PL011_CR_UARTEN;
}

static void pl011_enable_fifo(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->lcr_h |= PL011_LCRH_FEN;
}

static void pl011_disable_fifo(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->lcr_h &= ~PL011_LCRH_FEN;
}

static int pl011_set_baudrate(const struct pl011_uart_port_s *sport,
                              uint32_t clk, uint32_t baudrate)
{
  const struct pl011_config *config = &sport->config;

  /* Avoiding float calculations, bauddiv is left shifted by 6 */

  uint64_t bauddiv =
      (((uint64_t)clk) << PL011_FBRD_WIDTH) / (baudrate * 16U);

  /* Valid bauddiv value
   * uart_clk (min) >= 16 x baud_rate (max)
   * uart_clk (max) <= 16 x 65535 x baud_rate (min)
   */

  if ((bauddiv < (1U << PL011_FBRD_WIDTH)) ||
      (bauddiv > (65535U << PL011_FBRD_WIDTH)))
    {
      return -EINVAL;
    }

  config->uart->ibrd    = bauddiv >> PL011_FBRD_WIDTH;
  config->uart->fbrd    = bauddiv & ((1U << PL011_FBRD_WIDTH) - 1U);

  ARM64_DMB();

  /* In order to internally update the contents of ibrd or fbrd, a
   * lcr_h write must always be performed at the end
   * ARM DDI 0183F, Pg 3-13
   */

  config->uart->lcr_h = config->uart->lcr_h;

  return 0;
}

static void pl011_irq_tx_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_TXIM;
}

static void pl011_irq_tx_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~PL011_IMSC_TXIM;
}

static void pl011_irq_rx_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_RXIM | PL011_IMSC_RTIM;
}

static void pl011_irq_rx_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~(PL011_IMSC_RXIM | PL011_IMSC_RTIM);
}

static int pl011_irq_tx_complete(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  /* check for TX FIFO empty */

  return config->uart->fr & PL011_FR_TXFE;
}

static int pl011_irq_rx_ready(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;
  const struct pl011_data   *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_RXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_RXIM) &&
         (!(config->uart->fr & PL011_FR_RXFE));
}

/***************************************************************************
 * Name: qemu_pl011_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ***************************************************************************/

static bool qemu_pl011_txready(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_TXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_TXIM) &&
         pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: qemu_pl011_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ***************************************************************************/

static bool qemu_pl011_txempty(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  return pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: qemu_pl011_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ***************************************************************************/

static void qemu_pl011_send(struct uart_dev_s *dev, int ch)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;

  config->uart->dr = ch;
}

/***************************************************************************
 * Name: qemu_pl011_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ***************************************************************************/

static bool qemu_pl011_rxavailable(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;

  if (!data->sbsa &&
      (!(config->uart->cr & PL011_CR_UARTEN) ||
       !(config->uart->cr & PL011_CR_RXE)))
    {
      return false;
    }

  return (config->uart->fr & PL011_FR_RXFE) == 0U;
}

/***************************************************************************
 * Name: qemu_pl011_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ***************************************************************************/

static void qemu_pl011_rxint(struct uart_dev_s *dev, bool enable)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  if (enable)
    {
      pl011_irq_rx_enable(sport);
    }
  else
    {
      pl011_irq_rx_disable(sport);
    }
}

/***************************************************************************
 * Name: qemu_pl011_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ***************************************************************************/

static void qemu_pl011_txint(struct uart_dev_s *dev, bool enable)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      pl011_irq_tx_enable(sport);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
    }
  else
    {
      pl011_irq_tx_disable(sport);
    }

  leave_critical_section(flags);
}

/***************************************************************************
 * Name: qemu_pl011_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ***************************************************************************/

static int qemu_pl011_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  unsigned int              rx;

  rx = config->uart->dr;

  *status = 0;

  return rx;
}

/***************************************************************************
 * Name: qemu_pl011_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *   for current qemu configure,
 *
 ***************************************************************************/

static int qemu_pl011_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;
  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 * Name: qemu_pl011_irq_handler (and front-ends)
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ***************************************************************************/

static int qemu_pl011_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s         *dev = (struct uart_dev_s *)arg;
  struct pl011_uart_port_s  *sport;
  UNUSED(irq);
  UNUSED(context);

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  sport = (struct pl011_uart_port_s *)dev->priv;

  if (pl011_irq_rx_ready(sport))
    {
      uart_recvchars(dev);
    }

  if (qemu_pl011_txready(dev))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/***************************************************************************
 * Name: qemu_pl011_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ***************************************************************************/

static void qemu_pl011_detach(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  up_disable_irq(sport->irq_num);
  irq_detach(sport->irq_num);
}

/***************************************************************************
 * Name: qemu_pl011_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 ***************************************************************************/

static int qemu_pl011_attach(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport;
  struct pl011_data         *data;
  int                       ret;

  sport = (struct pl011_uart_port_s *)dev->priv;
  data  = &sport->data;

  ret = irq_attach(sport->irq_num, qemu_pl011_irq_handler, dev);

  if (ret == OK)
    {
      up_enable_irq(sport->irq_num);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }

  if (!data->sbsa)
    {
      pl011_enable(sport);
    }

  return ret;
}

/***************************************************************************
 * Name: qemu_pl011_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ***************************************************************************/

static void qemu_pl011_shutdown(struct uart_dev_s *dev)
{
  UNUSED(dev);
  sinfo("%s: call unexpected\n", __func__);
}

static int qemu_pl011_setup(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;
  int                       ret;
  uint32_t                  lcrh;
  irqstate_t                i_flags;

  i_flags = up_irq_save();

  /* If working in SBSA mode, we assume that UART is already configured,
   * or does not require configuration at all (if UART is emulated by
   * virtualization software).
   */

  if (!data->sbsa)
    {
      /* disable the uart */

      pl011_disable(sport);
      pl011_disable_fifo(sport);

      /* Set baud rate */

      ret = pl011_set_baudrate(sport, config->sys_clk_freq,
                               data->baud_rate);
      if (ret != 0)
        {
          up_irq_restore(i_flags);
          return ret;
        }

      /* Setting the default character format */

      lcrh  = config->uart->lcr_h & ~(PL011_LCRH_FORMAT_MASK);
      lcrh  &= ~(BIT(0) | BIT(7));
      lcrh  |= PL011_LCRH_WLEN_SIZE(8) << PL011_LCRH_WLEN_SHIFT;
      config->uart->lcr_h = lcrh;

      /* Enabling the FIFOs */

      pl011_enable_fifo(sport);
    }

  /* initialize all IRQs as masked */

  config->uart->imsc    = 0U;
  config->uart->icr     = PL011_IMSC_MASK_ALL;

  if (!data->sbsa)
    {
      config->uart->dmacr = 0U;
      ARM64_ISB();
      config->uart->cr  &= ~(BIT(14) | BIT(15) | BIT(1));
      config->uart->cr  |= PL011_CR_RXE | PL011_CR_TXE;
      ARM64_ISB();
    }

  up_irq_restore(i_flags);

  return 0;
}

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = qemu_pl011_setup,
  .shutdown = qemu_pl011_shutdown,
  .attach   = qemu_pl011_attach,
  .detach   = qemu_pl011_detach,
  .ioctl    = qemu_pl011_ioctl,
  .receive  = qemu_pl011_receive,
  .rxint    = qemu_pl011_rxint,
  .rxavailable = qemu_pl011_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = qemu_pl011_send,
  .txint    = qemu_pl011_txint,
  .txready  = qemu_pl011_txready,
  .txempty  = qemu_pl011_txempty,
};

/* This describes the state of the uart1 port. */

static struct pl011_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .sbsa       = false,
    },

  .config =
    {
      .uart           = (volatile struct pl011_regs *)CONFIG_QEMU_UART_BASE,
      .sys_clk_freq   = 24000000,
    },

    .irq_num       = CONFIG_QEMU_UART_IRQ,
    .is_console   = 1,
};

/* I/O buffers */

#ifdef CONFIG_QEMU_UART_PL011

static char                 g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char                 g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct uart_dev_s    g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};

#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  qemu_pl011_setup(&CONSOLE_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm64_lowputc('\r');
    }

  arm64_lowputc((uint8_t)ch);
  return ch;
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      sinfo("error at register dev/console, ret =%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      sinfo("error at register dev/ttyS0, ret =%d\n", ret);
    }
}

#else /* USE_SERIALDRIVER */

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm64_lowputc('\r');
    }

  arm64_lowputc((uint8_t)ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
