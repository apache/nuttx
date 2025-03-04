/****************************************************************************
 * arch/risc-v/src/bl808/bl808_serial.c
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/tioctl.h>

#include "hardware/bl808_uart.h"
#include "hardware/bl808_glb.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port /* UART0 is console */
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart1port /* UART1 is console */
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart2port /* UART2 is console */
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart3port /* UART3 is console */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_config_s
{
  uint8_t  idx;       /* Uart idx */
  uint32_t baud;      /* Configured baud */
  uint8_t  iflow_ctl; /* Input flow control supported */
  uint8_t  oflow_ctl; /* Output flow control supported. */
  uint8_t  data_bits; /* Number of bits per word */
  bool     stop_bits; /* true=2 stop bits; false=1 stop bit */
  uint8_t  parity;    /* Parity selection:  0=none, 1=odd, 2=even */
};

struct bl808_uart_s
{
  uint8_t              irq;      /* IRQ associated with this UART */
  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  bl808_setup(struct uart_dev_s *dev);
static void bl808_shutdown(struct uart_dev_s *dev);
static int  bl808_attach(struct uart_dev_s *dev);
static void bl808_detach(struct uart_dev_s *dev);
static int  bl808_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  bl808_receive(struct uart_dev_s *dev, unsigned int *status);
static void bl808_rxint(struct uart_dev_s *dev, bool enable);
static bool bl808_rxavailable(struct uart_dev_s *dev);
static void bl808_send(struct uart_dev_s *dev, int ch);
static void bl808_txint(struct uart_dev_s *dev, bool enable);
static bool bl808_txready(struct uart_dev_s *dev);
static bool bl808_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = bl808_setup,
  .shutdown    = bl808_shutdown,
  .attach      = bl808_attach,
  .detach      = bl808_detach,
  .ioctl       = bl808_ioctl,
  .receive     = bl808_receive,
  .rxint       = bl808_rxint,
  .rxavailable = bl808_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send    = bl808_send,
  .txint   = bl808_txint,
  .txready = bl808_txready,
  .txempty = bl808_txempty,
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];

/* UART port structs */

static struct bl808_uart_s g_uart0priv =
{
  .irq = BL808_IRQ_UART0,
  .config =
    {
      .idx       = 0,
      .baud      = CONFIG_UART0_BAUD,
      .parity    = CONFIG_UART0_PARITY,
      .data_bits = CONFIG_UART0_BITS,
      .stop_bits = CONFIG_UART0_2STOP,

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
#else
  .isconsole = 0,
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

static struct bl808_uart_s g_uart1priv =
{
  .irq = BL808_IRQ_UART1,
  .config =
    {
      .idx       = 1,
      .baud      = CONFIG_UART1_BAUD,
      .parity    = CONFIG_UART1_PARITY,
      .data_bits = CONFIG_UART1_BITS,
      .stop_bits = CONFIG_UART1_2STOP,

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
#else
  .isconsole = 0,
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

static struct bl808_uart_s g_uart2priv =
{
  .irq = BL808_IRQ_UART2,
  .config =
    {
      .idx       = 2,
      .baud      = CONFIG_UART2_BAUD,
      .parity    = CONFIG_UART2_PARITY,
      .data_bits = CONFIG_UART2_BITS,
      .stop_bits = CONFIG_UART2_2STOP,

#ifdef CONFIG_UART2_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART2_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART2_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART2_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart2port =
{
#ifdef CONFIG_UART2_SERIAL_CONSOLE
  .isconsole = 1,
#else
  .isconsole = 0,
#endif

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
  .priv = (void *)&g_uart2priv,
};

static struct bl808_uart_s g_uart3priv =
{
  .irq = BL808_IRQ_UART3,
  .config =
    {
      .idx       = 3,
      .baud      = CONFIG_UART3_BAUD,
      .parity    = CONFIG_UART3_PARITY,
      .data_bits = CONFIG_UART3_BITS,
      .stop_bits = CONFIG_UART3_2STOP,

#ifdef CONFIG_UART3_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART3_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART3_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART3_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart3port =
{
#ifdef CONFIG_UART3_SERIAL_CONSOLE
  .isconsole = 1,
#else
  .isconsole = 0,
#endif

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
  .priv = (void *)&g_uart3priv,
};

static struct uart_dev_s *const g_uart_devs[] =
{
  [0] = &g_uart0port,
  [1] = &g_uart1port,
  [2] = &g_uart2port,
  [3] = &g_uart3port
};

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int __uart_interrupt(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  struct bl808_uart_s *priv = dev->priv;
  uint8_t uart_idx = priv->config.idx;
  uint32_t int_status;
  uint32_t int_mask;

  int_status = getreg32(BL808_UART_INT_STS(uart_idx));
  int_mask = getreg32(BL808_UART_INT_MASK(uart_idx));

  /* Length of uart rx data transfer arrived interrupt */

  if ((int_status & UART_INT_STS_URX_END_INT) &&
      !(int_mask & UART_INT_MASK_CR_URX_END_MASK))
    {
      putreg32(UART_INT_CLEAR_CR_URX_END_CLR,
               BL808_UART_INT_CLEAR(uart_idx));

      /* Receive Data ready */

      uart_recvchars(dev);
    }

  /* Tx fifo ready interrupt,auto-cleared when data is pushed */

  if ((int_status & UART_INT_STS_UTX_FIFO_INT) &&
      !(int_mask & UART_INT_MASK_CR_UTX_FIFO_MASK))
    {
      /* Transmit data request interrupt */

      uart_xmitchars(dev);
    }

  /* Rx fifo ready interrupt,auto-cleared when data is popped */

  if ((int_status & UART_INT_STS_URX_FIFO_INT) &&
      !(int_mask & UART_INT_MASK_CR_URX_FIFO_MASK))
    {
      /* Receive Data ready */

      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: bl808_uart_configure
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc.
 *
 ****************************************************************************/

static void bl808_uart_configure(const struct uart_config_s *config)
{
  uint8_t uart_idx = config->idx;

  /* UTX_CONFIG */

  uint32_t tmp_val = getreg32(BL808_UART_UTX_CONFIG(uart_idx));

  tmp_val &= ~UART_UTX_CONFIG_CR_BIT_CNT_P_MASK;
  if (config->stop_bits)
    {
      tmp_val |= 3 << UART_UTX_CONFIG_CR_BIT_CNT_P_SHIFT;
    }
  else
    {
      tmp_val |= 1 << UART_UTX_CONFIG_CR_BIT_CNT_P_SHIFT;
    }

  tmp_val &= ~UART_UTX_CONFIG_CR_BIT_CNT_D_MASK;
  tmp_val |= config->data_bits << UART_UTX_CONFIG_CR_BIT_CNT_D_SHIFT;

  switch (config->parity)
    {
    case 0:
      tmp_val &= ~UART_UTX_CONFIG_CR_PRT_EN;
      break;

    case 1:
      tmp_val |= UART_UTX_CONFIG_CR_PRT_EN;
      tmp_val |= UART_UTX_CONFIG_CR_PRT_SEL;
      break;

    case 2:
      tmp_val |= UART_UTX_CONFIG_CR_PRT_EN;
      tmp_val &= ~UART_UTX_CONFIG_CR_PRT_SEL;
      break;
    }

  tmp_val |= UART_UTX_CONFIG_CR_FRM_EN;

  if (config->oflow_ctl)
    {
      tmp_val |= UART_UTX_CONFIG_CR_CTS_EN;
    }
  else
    {
      tmp_val &= ~UART_UTX_CONFIG_CR_CTS_EN;
    }

  tmp_val |= UART_UTX_CONFIG_CR_EN;
  putreg32(tmp_val, BL808_UART_UTX_CONFIG(uart_idx));

  /* URX CONFIG */

  tmp_val = getreg32(BL808_UART_URX_CONFIG(uart_idx));
  tmp_val |= UART_URX_CONFIG_CR_EN;
  putreg32(tmp_val, BL808_UART_URX_CONFIG(uart_idx));

  /* BIT PRD (baud rate) */

  uint16_t div = BL808_UART_CLK / config->baud - 1;
  tmp_val = getreg32(BL808_UART_BIT_PRD(uart_idx));
  tmp_val = div | (div << 16);
  putreg32(tmp_val, BL808_UART_BIT_PRD(uart_idx));

  /* FIFO CONFIG 1 */

  tmp_val = getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx));
  tmp_val |= 1 << UART_FIFO_CONFIG_1_TX_TH_SHIFT;
  putreg32(tmp_val, BL808_UART_FIFO_CONFIG_1(uart_idx));
}

/****************************************************************************
 * Name: bl808_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int bl808_setup(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;

  bl808_uart_configure(&priv->config);
  return OK;
}

/****************************************************************************
 * Name: bl808_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void bl808_shutdown(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  /* Disable uart before config */

  modifyreg32(BL808_UART_UTX_CONFIG(uart_idx), UART_UTX_CONFIG_CR_EN, 0);
  modifyreg32(BL808_UART_URX_CONFIG(uart_idx), UART_URX_CONFIG_CR_EN, 0);
}

/****************************************************************************
 * Name: bl808_attach
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

static int bl808_attach(struct uart_dev_s *dev)
{
  int                  ret;
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;

  ret = irq_attach(priv->irq, __uart_interrupt, (void *)dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: bl808_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void bl808_detach(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: bl808_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int bl808_ioctl(struct file *filep, int cmd, unsigned long arg)
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
          struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }
          termiosp->c_cflag = 0;

          /* Return parity */

          termiosp->c_cflag = ((priv->config.parity != 0) ? PARENB : 0) |
                              ((priv->config.parity == 1) ? PARODD : 0);

          /* Return stop bits */

          termiosp->c_cflag |= (priv->config.stop_bits) ? CSTOPB : 0;

          /* Return flow control */

          termiosp->c_cflag |= (priv->config.iflow_ctl) ? CRTS_IFLOW : 0;
          termiosp->c_cflag |= (priv->config.oflow_ctl) ? CCTS_OFLOW : 0;

          /* Return baud */

          cfsetispeed(termiosp, priv->config.baud);

          /* Return number of bits */

          switch (priv->config.data_bits)
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
          struct bl808_uart_s *    priv = (struct bl808_uart_s *)dev->priv;
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

          if (priv->config.idx == 3)
            {
#ifdef CONFIG_UART3_IFLOWCONTROL
              config.iflow_ctl = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_UART3_OFLOWCONTROL
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

              tmp_val = getreg32(BL808_UART_INT_MASK(config.idx));
              bl808_uart_configure(&config);
              putreg32(tmp_val, BL808_UART_INT_MASK(config.idx));
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
 * Name: bl808_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int bl808_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;
  int rxdata;

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* if uart fifo cnts > 0 */

  if (getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
      UART_FIFO_CONFIG_1_RX_CNT_MASK)
    {
      rxdata = getreg32(BL808_UART_FIFO_RDATA(uart_idx)) &
               UART_FIFO_RDATA_MASK;
    }
  else
    {
      rxdata = -1;
    }
  return rxdata;
}

/****************************************************************************
 * Name: bl808_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void bl808_rxint(struct uart_dev_s *dev, bool enable)
{
  uint32_t int_mask;
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      int_mask = getreg32(BL808_UART_INT_MASK(uart_idx));
      int_mask &= ~(UART_INT_MASK_CR_URX_FIFO_MASK);
      int_mask &= ~(UART_INT_MASK_CR_URX_END_MASK);
      putreg32(int_mask, BL808_UART_INT_MASK(uart_idx));
#endif
    }
  else
    {
      int_mask = getreg32(BL808_UART_INT_MASK(uart_idx));
      int_mask |= UART_INT_MASK_CR_URX_FIFO_MASK;
      int_mask |= UART_INT_MASK_CR_URX_END_MASK;
      putreg32(int_mask, BL808_UART_INT_MASK(uart_idx));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bl808_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool bl808_rxavailable(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  /* Return true is data is available in the receive data buffer */

  return (getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
          UART_FIFO_CONFIG_1_RX_CNT_MASK) != 0;
}

/****************************************************************************
 * Name: bl808_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void bl808_send(struct uart_dev_s *dev, int ch)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  /* Wait for FIFO to be empty */

  while ((getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
         UART_FIFO_CONFIG_1_TX_CNT_MASK) == 0);

  putreg32(ch, BL808_UART_FIFO_WDATA(uart_idx));
}

/****************************************************************************
 * Name: bl808_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void bl808_txint(struct uart_dev_s *dev, bool enable)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;
  irqstate_t flags;
  uint32_t int_mask;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      int_mask = getreg32(BL808_UART_INT_MASK(uart_idx));
      int_mask &= ~(UART_INT_MASK_CR_UTX_FIFO_MASK);
      putreg32(int_mask, BL808_UART_INT_MASK(uart_idx));

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      int_mask = getreg32(BL808_UART_INT_MASK(uart_idx));
      int_mask |= UART_INT_MASK_CR_UTX_FIFO_MASK;
      putreg32(int_mask, BL808_UART_INT_MASK(uart_idx));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bl808_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/

static bool bl808_txready(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  /* Return TRUE if the TX FIFO is not full */

  return (getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
          UART_FIFO_CONFIG_1_TX_CNT_MASK) != 0;
}

/****************************************************************************
 * Name: bl808_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool bl808_txempty(struct uart_dev_s *dev)
{
  struct bl808_uart_s *priv = (struct bl808_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  return (getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
          UART_FIFO_CONFIG_1_TX_CNT_MASK) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void bl808_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  bl808_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: bl808_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void bl808_serialinit(void)
{
  int  i;
  char devname[16];

  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);

  /* Register all UARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));
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

void up_putc(int ch)
{
  struct bl808_uart_s *priv = CONSOLE_DEV.priv;
  uint8_t uart_idx = priv->config.idx;
  irqstate_t flags = enter_critical_section();

  while ((getreg32(BL808_UART_FIFO_CONFIG_1(uart_idx)) &
            UART_FIFO_CONFIG_1_TX_CNT_MASK) == 0);

  putreg32(ch, BL808_UART_FIFO_WDATA(uart_idx));

  leave_critical_section(flags);
}
