/****************************************************************************
 * arch/risc-v/src/bl602/bl602_serial.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "bl602_lowputc.h"

#include "hardware/bl602_gpio.h"
#include "hardware/bl602_uart.h"
#include "hardware/bl602_glb.h"

#include "riscv_arch.h"
#include "riscv_internal.h"

#include "bl602_config.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    0
#define BL602_CONSOLE_BAUD   CONFIG_UART0_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART0_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART0_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART0_2STOP
#define BL602_CONSOLE_TX     GPIO_UART0_TX
#define BL602_CONSOLE_RX     GPIO_UART0_RX
#define HAVE_UART
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    1
#define BL602_CONSOLE_BAUD   CONFIG_UART1_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART1_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART1_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART1_2STOP
#define BL602_CONSOLE_TX     GPIO_UART1_TX
#define BL602_CONSOLE_RX     GPIO_UART1_RX
#define HAVE_UART
#endif
#endif /* HAVE_CONSOLE */
/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port /* UART0 is console */
#define TTYS0_DEV   g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV                /* No ttyS1 */
#define SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart1port /* UART0 is console */
#error "I'm confused... Do we have a serial console or not?"
#endif
#else
#undef CONSOLE_DEV /* No console */
#undef CONFIG_UART0_SERIAL_CONSOLE
#if defined(CONFIG_BL602_UART0)
#define TTYS0_DEV g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV              /* No ttyS1 */
#define SERIAL_CONSOLE 1
#else
#undef TTYS0_DEV
#undef TTYS1_DEV
#endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of up_earlyserialinit(), up_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const uint32_t       uartbase; /* Base address of UART registers */
  uint8_t              irq;      /* IRQ associated with this UART */
  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
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
  .setup       = up_setup,
  .shutdown    = up_shutdown,
  .attach      = up_attach,
  .detach      = up_detach,
  .ioctl       = up_ioctl,
  .receive     = up_receive,
  .rxint       = up_rxint,
  .rxavailable = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send    = up_send,
  .txint   = up_txint,
  .txready = up_txready,
  .txempty = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_BL602_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct up_dev_s g_uart0priv =
{
  .uartbase = UART0_BASE,
  .irq      = BL602_IRQ_UART0,

  .config =
    {
      .idx       = 0,
      .baud      = CONFIG_UART0_BAUD,
      .parity    = CONFIG_UART0_PARITY,
      .data_bits = CONFIG_UART0_BITS,
      .stop_bits = CONFIG_UART0_2STOP,
      .tx_pin    = CONFIG_BL602_UART0_TX_PIN,
      .rx_pin    = CONFIG_BL602_UART0_RX_PIN,
      .rts_pin   = CONFIG_BL602_UART0_RTS_PIN,
      .cts_pin   = CONFIG_BL602_UART0_CTS_PIN,

#ifdef CONFIG_UART0_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART0_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART0_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART0_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart0port =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  .isconsole = 1,
#endif
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
  .priv = (void *)&g_uart0priv,
};
#endif

#ifdef CONFIG_BL602_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct up_dev_s g_uart1priv =
{
  .uartbase = UART1_BASE,
  .irq      = BL602_IRQ_UART1,

  .config =
    {
      .idx       = 1,
      .baud      = CONFIG_UART1_BAUD,
      .parity    = CONFIG_UART1_PARITY,
      .data_bits = CONFIG_UART1_BITS,
      .stop_bits = CONFIG_UART1_2STOP,
      .tx_pin    = CONFIG_BL602_UART1_TX_PIN,
      .rx_pin    = CONFIG_BL602_UART1_RX_PIN,
      .rts_pin   = CONFIG_BL602_UART1_RTS_PIN,
      .cts_pin   = CONFIG_BL602_UART1_CTS_PIN,

#ifdef CONFIG_UART1_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART1_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART1_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART1_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart1port =
{
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  .isconsole = 1,
#endif
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
  .priv = (void *)&g_uart1priv,
};
#endif

static struct uart_dev_s *const g_uart_devs[] =
{
#ifdef CONFIG_BL602_UART0
  [0] = &g_uart0port,
#endif
#ifdef CONFIG_BL602_UART1
  [1] = &g_uart1port
#endif
};

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int __uart_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uart_dev_t *     dev       = (uart_dev_t *)arg;
  struct up_dev_s *uart_priv = dev->priv;
  uint32_t         tmp_val   = 0;
  uint32_t         mask_val  = 0;

  tmp_val  = bl602_up_serialin(uart_priv->uartbase, UART_INT_STS_OFFSET);
  mask_val = bl602_up_serialin(uart_priv->uartbase, UART_INT_MASK_OFFSET);

  /* Length of uart rx data transfer arrived interrupt */

  if ((tmp_val & (1 << UART_URX_END_INT_POS)) &&
      !(mask_val & (1 << UART_CR_URX_END_MASK_POS)))
    {
      bl602_up_serialout(uart_priv->uartbase, UART_INT_CLEAR_OFFSET, 0x2);

      /* Receive Data ready */

      uart_recvchars(dev);
    }

  /* Tx fifo ready interrupt,auto-cleared when data is pushed */

  if ((tmp_val & (1 << UART_UTX_FIFO_INT_POS)) &&
      !(mask_val & (1 << UART_CR_UTX_FIFO_MASK_POS)))
    {
      /* Transmit data request interrupt */

      uart_xmitchars(dev);
    }

  /* Rx fifo ready interrupt,auto-cleared when data is popped */

  if ((tmp_val & (1 << UART_URX_FIFO_INT_POS)) &&
      !(mask_val & (1 << UART_CR_URX_FIFO_MASK_POS)))
    {
      /* Receive Data ready */

      uart_recvchars(dev);
    }

  return OK;
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
  struct up_dev_s *uart_priv = (struct up_dev_s *)dev->priv;

  bl602_uart_configure(uart_priv->uartbase, &uart_priv->config);
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
  struct up_dev_s *uart_priv = (struct up_dev_s *)dev->priv;

  /* Disable uart before config */

  bl602_up_serialmodify(uart_priv->uartbase, UART_UTX_CONFIG_OFFSET, 1, 0);
  bl602_up_serialmodify(uart_priv->uartbase, UART_URX_CONFIG_OFFSET, 1, 0);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  int              ret;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  ret = irq_attach(priv->irq, __uart_interrupt, (void *)dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
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
 * Name: bl602_data_getbits
 ****************************************************************************/

static uint32_t
bl602_data_getbits(uint32_t data, uint32_t start, uint32_t len)
{
  return (((data) >> (start)) & (~((~0) << (len))));
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
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode *     inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      do
        {
          struct termios * termiosp = (struct termios *)arg;
          struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }
          termiosp->c_cflag = 0;

          /* Return parity */

          termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                              ((priv->parity == 1) ? PARODD : 0);

          /* Return stop bits */

          termiosp->c_cflag |= (priv->stop_bits) ? CSTOPB : 0;

          /* Return flow control */

          termiosp->c_cflag |= (priv->iflow_ctl) ? CRTS_IFLOW : 0;
          termiosp->c_cflag |= (priv->oflow_ctl) ? CCTS_OFLOW : 0;

          /* Return baud */

          cfsetispeed(termiosp, priv->baud);

          /* Return number of bits */

          switch (priv->data_bits)
            {
            case 5:
              termiosp->c_cflag |= CS5;
              break;

            case 6:
              termiosp->c_cflag |= CS6;
              break;

            case 7:
              termiosp->c_cflag |= CS7;
              break;

            default:
            case 8:
              termiosp->c_cflag |= CS8;
              break;
            }
        }
      while (0);
      break;

    case TCSETS:
      do
        {
          struct termios *     termiosp = (struct termios *)arg;
          struct up_dev_s *    priv     = (struct up_dev_s *)dev->priv;
          struct uart_config_s config;
          uint32_t             tmp_val;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Decode baud. */

          ret         = OK;
          config.baud = cfgetispeed(termiosp);

          /* Decode number of bits */

          switch (termiosp->c_cflag & CSIZE)
            {
            case CS5:
              config.data_bits = 5;
              break;

            case CS6:
              config.data_bits = 6;
              break;

            case CS7:
              config.data_bits = 7;
              break;

            case CS8:
              config.data_bits = 8;
              break;

            default:
              ret = -EINVAL;
              break;
            }

          /* Decode parity */

          if ((termiosp->c_cflag & PARENB) != 0)
            {
              config.parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              config.parity = 0;
            }

          /* Decode stop bits */

          config.stop_bits = (termiosp->c_cflag & CSTOPB) != 0;

          /* Decode flow control */

          if (priv->idx == 0)
            {
#if CONFIG_UART0_IFLOWCONTROL
              config.iflow_ctl = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#if CONFIG_UART0_OFLOWCONTROL
              config.oflow_ctl = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
            }
          else
            {
#if CONFIG_UART1_IFLOWCONTROL
              config.iflow_ctl = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#if CONFIG_UART1_OFLOWCONTROL
              config.oflow_ctl = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
            }

          /* Verify that all settings are valid before committing */

          if (ret == OK)
            {
              /* Commit */

              memcpy(&priv->config, &config, sizeof(config));

              /* effect the changes immediately - note that we do not
               * implement TCSADRAIN / TCSAFLUSH
               */

              tmp_val =
                bl602_up_serialin(priv->uartbase, UART_INT_MASK_OFFSET);
              bl602_uart_configure(priv->uartbase, &config);
              bl602_up_serialout(
                priv->uartbase, UART_INT_MASK_OFFSET, tmp_val);
            }
        }
      while (0);
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
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
  int              rxdata;

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* if uart fifo cnts > 0 */

  if (bl602_data_getbits(
        bl602_up_serialin(priv->uartbase, UART_FIFO_CONFIG_1_OFFSET),
        UART_RX_FIFO_CNT_POS,
        6) > 0)
    {
      rxdata = bl602_up_serialin(priv->uartbase, UART_FIFO_RDATA_OFFSET);
    }
  else
    {
      rxdata = -1;
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
  uint32_t         tmp_val;
  struct up_dev_s *priv  = (struct up_dev_s *)dev->priv;
  irqstate_t       flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      tmp_val = bl602_up_serialin(priv->uartbase, UART_INT_MASK_OFFSET);
      tmp_val &= ~(1 << UART_INT_RX_FIFO_REQ);
      tmp_val &= ~(1 << UART_INT_RX_END);
      bl602_up_serialout(priv->uartbase, UART_INT_MASK_OFFSET, tmp_val);
#endif
    }
  else
    {
      tmp_val = bl602_up_serialin(priv->uartbase, UART_INT_MASK_OFFSET);
      tmp_val |= (1 << UART_INT_RX_FIFO_REQ);
      tmp_val |= (1 << UART_INT_RX_END);
      bl602_up_serialout(priv->uartbase, UART_INT_MASK_OFFSET, tmp_val);
    }

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

  /* Return true is data is available in the receive data buffer */

  uint32_t rxcnt = bl602_data_getbits(
    bl602_up_serialin(priv->uartbase, UART_FIFO_CONFIG_1_OFFSET),
    UART_RX_FIFO_CNT_POS,
    6);

  return rxcnt != 0;
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

  /* Wait for FIFO */

  while (bl602_data_getbits(
           bl602_up_serialin(priv->uartbase, UART_FIFO_CONFIG_1_OFFSET),
           UART_TX_FIFO_CNT_POS,
           6) == 0)
    ;

  bl602_up_serialout(priv->uartbase, UART_FIFO_WDATA_OFFSET, ch);
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
  irqstate_t       flags;
  uint32_t         tmp_val;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      tmp_val = bl602_up_serialin(priv->uartbase, UART_INT_MASK_OFFSET);
      tmp_val &= ~(1 << UART_INT_TX_FIFO_REQ);
      bl602_up_serialout(priv->uartbase, UART_INT_MASK_OFFSET, tmp_val);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      tmp_val = bl602_up_serialin(priv->uartbase, UART_INT_MASK_OFFSET);
      tmp_val |= (1 << UART_INT_TX_FIFO_REQ);
      bl602_up_serialout(priv->uartbase, UART_INT_MASK_OFFSET, tmp_val);
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

  uint32_t txcnt =
    bl602_up_serialin(priv->uartbase, UART_FIFO_CONFIG_1_OFFSET);
  txcnt = (txcnt & UART_TX_FIFO_CNT_MSK) >> UART_TX_FIFO_CNT_POS;

  return (txcnt != 0);
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

  /* Return TRUE if the TX is pending */

  uint32_t txcnt = bl602_data_getbits(
    bl602_up_serialin(priv->uartbase, UART_FIFO_CONFIG_1_OFFSET),
    UART_TX_FIFO_CNT_POS,
    6);

  return (txcnt == 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  int  i;
  char devname[16];

#ifdef HAVE_SERIAL_CONSOLE
  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  strcpy(devname, "/dev/ttySx");
  for (i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      if (g_uart_devs[i] == 0)
        {
          continue;
        }

      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + i;
      uart_register(devname, g_uart_devs[i]);
    }
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
  (void)priv;

  irqstate_t flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  leave_critical_section(flags);
#endif
  return ch;
}

/****************************************************************************
 * Name: up_earlyserialinit, up_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void up_earlyserialinit(void)
{
}

void up_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else  /* USE_SERIALDRIVER */

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

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
