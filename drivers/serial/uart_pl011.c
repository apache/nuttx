/***************************************************************************
 * drivers/serial/uart_pl011.c
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
#include <nuttx/bits.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_pl011.h>

#ifdef CONFIG_UART_PL011

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-5 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_UART0_PL011)
#  define HAVE_PL011_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_UART1_PL011)
#  define HAVE_PL011_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_UART2_PL011)
#  define HAVE_PL011_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_UART3_PL011)
#  define HAVE_PL011_CONSOLE 1
#else
#  undef HAVE_PL011_CONSOLE 1
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
  FAR volatile struct pl011_regs *uart;
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
};

static int pl011_setup(FAR struct uart_dev_s *dev);
static void pl011_shutdown(FAR struct uart_dev_s *dev);
static int pl011_attach(FAR struct uart_dev_s *dev);
static void pl011_detach(FAR struct uart_dev_s *dev);
static int pl011_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int pl011_receive(FAR struct uart_dev_s *dev,
                         FAR unsigned int *status);
static void pl011_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool pl011_rxavailable(FAR struct uart_dev_s *dev);
static void pl011_send(FAR struct uart_dev_s *dev, int ch);
static void pl011_txint(FAR struct uart_dev_s *dev, bool enable);
static bool pl011_txready(FAR struct uart_dev_s *dev);
static bool pl011_txempty(FAR struct uart_dev_s *dev);

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = pl011_setup,
  .shutdown = pl011_shutdown,
  .attach   = pl011_attach,
  .detach   = pl011_detach,
  .ioctl    = pl011_ioctl,
  .receive  = pl011_receive,
  .rxint    = pl011_rxint,
  .rxavailable = pl011_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = pl011_send,
  .txint    = pl011_txint,
  .txready  = pl011_txready,
  .txempty  = pl011_txempty,
};

/* I/O buffers */

#ifdef CONFIG_UART0_PL011
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_UART1_PL011
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_UART2_PL011
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_UART3_PL011
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

/* This describes the state of the uart0 port. */

#ifdef CONFIG_UART0_PL011

static struct pl011_uart_port_s g_uart0priv =
{
  .data =
    {
      .baud_rate = CONFIG_UART0_BAUD,
      .sbsa      = false,
    },

  .config =
    {
      .uart         = (FAR volatile struct pl011_regs *)CONFIG_UART0_BASE,
      .sys_clk_freq = CONFIG_UART0_CLK_FREQ,
    },

    .irq_num    = CONFIG_UART0_IRQ,
};

/* I/O buffers */

static struct uart_dev_s g_uart0port =
{
  .recv =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },

  .xmit =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },

  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};

#endif /* CONFIG_UART0_PL011 */

/* This describes the state of the uart1 port. */

#ifdef CONFIG_UART1_PL011

static struct pl011_uart_port_s g_uart1priv =
{
  .data =
    {
      .baud_rate = CONFIG_UART1_BAUD,
      .sbsa      = false,
    },

  .config =
    {
      .uart         = (FAR volatile struct pl011_regs *)CONFIG_UART1_BASE,
      .sys_clk_freq = CONFIG_UART1_CLK_FREQ,
    },

    .irq_num    = CONFIG_UART1_IRQ,
};

/* I/O buffers */

static struct uart_dev_s g_uart1port =
{
  .recv =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops  = &g_uart_ops,
  .priv = &g_uart1priv,
};

#endif /* CONFIG_UART1_PL011 */

/* This describes the state of the uart2 port. */

#ifdef CONFIG_UART2_PL011

static struct pl011_uart_port_s g_uart2priv =
{
  .data =
    {
      .baud_rate = CONFIG_UART2_BAUD,
      .sbsa      = false,
    },

  .config =
    {
      .uart         = (FAR volatile struct pl011_regs *)CONFIG_UART2_BASE,
      .sys_clk_freq = CONFIG_UART2_CLK_FREQ,
    },

    .irq_num    = CONFIG_UART2_IRQ,
};

/* I/O buffers */

static struct uart_dev_s g_uart2port =
{
  .recv =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },

  .xmit =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },

  .ops  = &g_uart_ops,
  .priv = &g_uart2priv,
};

#endif /* CONFIG_UART2_PL011 */

/* This describes the state of the uart3 port. */

#ifdef CONFIG_UART3_PL011

static struct pl011_uart_port_s g_uart3priv =
{
  .data =
    {
      .baud_rate = CONFIG_UART3_BAUD,
      .sbsa      = false,
    },

  .config =
    {
      .uart         = (FAR volatile struct pl011_regs *)CONFIG_UART3_BASE,
      .sys_clk_freq = CONFIG_UART3_CLK_FREQ,
    },

    .irq_num    = CONFIG_UART3_IRQ,
};

/* I/O buffers */

static struct uart_dev_s g_uart3port =
{
  .recv =
    {
      .size   = CONFIG_UART3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },

  .xmit =
    {
      .size   = CONFIG_UART3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },

  .ops  = &g_uart_ops,
  .priv = &g_uart3priv,
};

#endif /* CONFIG_UART3_PL011 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port         /* UART0 is console */
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port         /* UART1 is console */
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port         /* UART2 is console */
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart3port         /* UART3 is console */
#endif

#ifdef CONFIG_UART0_PL011
#  define TTYS0_DEV       g_uart0port
#endif

#ifdef CONFIG_UART1_PL011
#  define TTYS1_DEV       g_uart1port
#endif

#ifdef CONFIG_UART2_PL011
#  define TTYS2_DEV       g_uart2port
#endif

#ifdef CONFIG_UART3_PL011
#  define TTYS3_DEV       g_uart3port
#endif

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static void pl011_enable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->cr |= PL011_CR_UARTEN;
}

static void pl011_disable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->cr &= ~PL011_CR_UARTEN;
}

static void pl011_enable_fifo(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->lcr_h |= PL011_LCRH_FEN;
}

static void pl011_disable_fifo(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->lcr_h &= ~PL011_LCRH_FEN;
}

static int pl011_set_baudrate(FAR const struct pl011_uart_port_s *sport,
                              uint32_t clk, uint32_t baudrate)
{
  FAR const struct pl011_config *config = &sport->config;

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

  /* In order to internally update the contents of ibrd or fbrd, a
   * lcr_h write must always be performed at the end
   * ARM DDI 0183F, Pg 3-13
   */

  config->uart->lcr_h = config->uart->lcr_h;

  return 0;
}

static void pl011_irq_tx_enable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_TXIM;
}

static void pl011_irq_tx_disable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~PL011_IMSC_TXIM;
}

static void pl011_irq_rx_enable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_RXIM | PL011_IMSC_RTIM;
}

static void pl011_irq_rx_disable(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~(PL011_IMSC_RXIM | PL011_IMSC_RTIM);
}

static int pl011_irq_tx_complete(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;

  /* check for TX FIFO empty */

  return config->uart->fr & PL011_FR_TXFE;
}

static int pl011_irq_rx_ready(FAR const struct pl011_uart_port_s *sport)
{
  FAR const struct pl011_config *config = &sport->config;
  FAR const struct pl011_data   *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_RXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_RXIM) &&
         (!(config->uart->fr & PL011_FR_RXFE));
}

/***************************************************************************
 * Name: pl011_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ***************************************************************************/

static bool pl011_txready(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s  *sport  = dev->priv;
  FAR const struct pl011_config *config = &sport->config;
  FAR struct pl011_data         *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_TXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_TXIM) &&
         pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: pl011_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ***************************************************************************/

static bool pl011_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s *sport = dev->priv;

  return pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: pl011_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ***************************************************************************/

static void pl011_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct pl011_uart_port_s  *sport  = dev->priv;
  FAR const struct pl011_config *config = &sport->config;

  config->uart->dr = ch;
}

/***************************************************************************
 * Name: pl011_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ***************************************************************************/

static bool pl011_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s  *sport  = dev->priv;
  FAR const struct pl011_config *config = &sport->config;
  FAR struct pl011_data         *data   = &sport->data;

  if (!data->sbsa &&
      (!(config->uart->cr & PL011_CR_UARTEN) ||
       !(config->uart->cr & PL011_CR_RXE)))
    {
      return false;
    }

  return (config->uart->fr & PL011_FR_RXFE) == 0U;
}

/***************************************************************************
 * Name: pl011_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ***************************************************************************/

static void pl011_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct pl011_uart_port_s *sport = dev->priv;

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
 * Name: pl011_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ***************************************************************************/

static void pl011_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct pl011_uart_port_s *sport = dev->priv;
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
 * Name: pl011_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ***************************************************************************/

static int pl011_receive(FAR struct uart_dev_s *dev,
                         FAR unsigned int *status)
{
  FAR struct pl011_uart_port_s  *sport  = dev->priv;
  FAR const struct pl011_config *config = &sport->config;
  unsigned int              rx;

  rx = config->uart->dr;

  *status = 0;

  return rx;
}

/***************************************************************************
 * Name: pl011_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *   for current qemu configure,
 *
 ***************************************************************************/

static int pl011_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
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
 * Name: pl011_irq_handler (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ***************************************************************************/

static int pl011_irq_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s        *dev = arg;
  FAR struct pl011_uart_port_s *sport;
  UNUSED(irq);
  UNUSED(context);

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  sport = dev->priv;

  if (pl011_irq_rx_ready(sport))
    {
      uart_recvchars(dev);
    }

  if (pl011_txready(dev))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/***************************************************************************
 * Name: pl011_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ***************************************************************************/

static void pl011_detach(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s *sport = dev->priv;

  up_disable_irq(sport->irq_num);
  irq_detach(sport->irq_num);
}

/***************************************************************************
 * Name: pl011_attach
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

static int pl011_attach(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s  *sport;
  FAR struct pl011_data         *data;
  int                       ret;

  sport = dev->priv;
  data  = &sport->data;

  ret = irq_attach(sport->irq_num, pl011_irq_handler, dev);

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
 * Name: pl011_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ***************************************************************************/

static void pl011_shutdown(FAR struct uart_dev_s *dev)
{
  UNUSED(dev);
  sinfo("%s: call unexpected\n", __func__);
}

static int pl011_setup(FAR struct uart_dev_s *dev)
{
  FAR struct pl011_uart_port_s  *sport  = dev->priv;
  FAR const struct pl011_config *config = &sport->config;
  FAR struct pl011_data         *data   = &sport->data;
  int                            ret;
  uint32_t                       lcrh;
  irqstate_t                     i_flags;

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
      config->uart->cr  &= ~(BIT(14) | BIT(15) | BIT(1));
      config->uart->cr  |= PL011_CR_RXE | PL011_CR_TXE;
    }

  up_irq_restore(i_flags);

  return 0;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: pl011_earlyserialinit
 *
 * Description:
 *   see nuttx/serial/uart_pl011.h
 *
 ***************************************************************************/

void pl011_earlyserialinit(void)
{
  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  pl011_setup(&CONSOLE_DEV);
#endif
}

/***************************************************************************
 * Name: pl011_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   pl011_earlyserialinit was called previously.
 *
 ***************************************************************************/

void pl011_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ***************************************************************************/

#ifdef HAVE_PL011_CONSOLE
int up_putc(int ch)
{
  FAR struct uart_dev_s *dev = &CONSOLE_DEV;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      pl011_send(dev, '\r');
    }

  pl011_send(dev, ch);

  return ch;
}
#endif

#endif /* CONFIG_UART_PL011 */
