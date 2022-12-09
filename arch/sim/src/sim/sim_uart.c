/****************************************************************************
 * arch/sim/src/sim/sim_uart.c
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
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>

#include "sim_internal.h"

#if defined(USE_DEVCONSOLE) || CONFIG_SIM_UART_NUMBER > 0
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SIM_UART_BUFFER_SIZE
  #define CONFIG_SIM_UART_BUFFER_SIZE 256
#endif

#define TTY_RECVSEND(port) \
  if (g_tty##port##_priv.fd > 0) \
    { \
      uart_recvchars(&g_tty##port##_dev); \
      uart_xmitchars(&g_tty##port##_dev); \
    } \

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tty_priv_s
{
  /* tty-port path name */

  const char *path;

  /* The file descriptor. It is returned by open */

  int             fd;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tty_setup(struct uart_dev_s *dev);
static void tty_shutdown(struct uart_dev_s *dev);
static int  tty_attach(struct uart_dev_s *dev);
static void tty_detach(struct uart_dev_s *dev);
static int  tty_ioctl(struct file *filep, int cmd,
                      unsigned long arg);
static int  tty_receive(struct uart_dev_s *dev, uint32_t *status);
static void tty_rxint(struct uart_dev_s *dev, bool enable);
static bool tty_rxavailable(struct uart_dev_s *dev);
static bool tty_rxflowcontrol(struct uart_dev_s *dev,
                              unsigned int nbuffered, bool upper);
static void tty_send(struct uart_dev_s *dev, int ch);
static void tty_txint(struct uart_dev_s *dev, bool enable);
static bool tty_txready(struct uart_dev_s *dev);
static bool tty_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_tty_ops =
{
  .setup          = tty_setup,
  .shutdown       = tty_shutdown,
  .attach         = tty_attach,
  .detach         = tty_detach,
  .ioctl          = tty_ioctl,
  .receive        = tty_receive,
  .rxint          = tty_rxint,
  .rxavailable    = tty_rxavailable,
  .rxflowcontrol  = tty_rxflowcontrol,
  .send           = tty_send,
  .txint          = tty_txint,
  .txready        = tty_txready,
  .txempty        = tty_txempty,
};
#endif

#ifdef USE_DEVCONSOLE
static char g_console_rxbuf[CONFIG_SIM_UART_BUFFER_SIZE];
static char g_console_txbuf[CONFIG_SIM_UART_BUFFER_SIZE];
#endif

#ifdef CONFIG_SIM_UART0_NAME
static char g_tty0_rxbuf[CONFIG_SIM_UART_BUFFER_SIZE];
static char g_tty0_txbuf[CONFIG_SIM_UART_BUFFER_SIZE];
#endif

#ifdef CONFIG_SIM_UART1_NAME
static char g_tty1_rxbuf[CONFIG_SIM_UART_BUFFER_SIZE];
static char g_tty1_txbuf[CONFIG_SIM_UART_BUFFER_SIZE];
#endif

#ifdef CONFIG_SIM_UART2_NAME
static char g_tty2_rxbuf[CONFIG_SIM_UART_BUFFER_SIZE];
static char g_tty2_txbuf[CONFIG_SIM_UART_BUFFER_SIZE];
#endif

#ifdef CONFIG_SIM_UART3_NAME
static char g_tty3_rxbuf[CONFIG_SIM_UART_BUFFER_SIZE];
static char g_tty3_txbuf[CONFIG_SIM_UART_BUFFER_SIZE];
#endif

#ifdef USE_DEVCONSOLE
static struct uart_dev_s g_console_dev =
{
  .isconsole      = true,
  .ops            = &g_tty_ops,
  .xmit =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_console_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_console_rxbuf,
  },
};
#endif

#ifdef CONFIG_SIM_UART0_NAME
static struct tty_priv_s g_tty0_priv =
{
  .path           = CONFIG_SIM_UART0_NAME,
  .fd             = -1,
};

static struct uart_dev_s g_tty0_dev =
{
  .ops            = &g_tty_ops,
  .priv           = &g_tty0_priv,
  .xmit =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty0_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty0_rxbuf,
  },
};
#endif

#ifdef CONFIG_SIM_UART1_NAME
static struct tty_priv_s g_tty1_priv =
{
  .path           = CONFIG_SIM_UART1_NAME,
  .fd             = -1,
};

static struct uart_dev_s g_tty1_dev =
{
  .ops            = &g_tty_ops,
  .priv           = &g_tty1_priv,
  .xmit =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty1_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty1_rxbuf,
  },
};
#endif

#ifdef CONFIG_SIM_UART2_NAME
static struct tty_priv_s g_tty2_priv =
{
  .path           = CONFIG_SIM_UART2_NAME,
  .fd             = -1,
};

static struct uart_dev_s g_tty2_dev =
{
  .ops            = &g_tty_ops,
  .priv           = &g_tty2_priv,
  .xmit =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty2_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty2_rxbuf,
  },
};
#endif

#ifdef CONFIG_SIM_UART3_NAME
static struct tty_priv_s g_tty3_priv =
{
  .path           = CONFIG_SIM_UART3_NAME,
  .fd             = -1,
};

static struct uart_dev_s g_tty3_dev =
{
  .ops            = &g_tty_ops,
  .priv           = &g_tty3_priv,
  .xmit =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty3_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_SIM_UART_BUFFER_SIZE,
    .buffer       = g_tty3_rxbuf,
  },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(USE_DEVCONSOLE) || CONFIG_SIM_UART_NUMBER > 0
/****************************************************************************
 * Name: tty_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int tty_setup(struct uart_dev_s *dev)
{
  struct tty_priv_s *priv = dev->priv;

  if (!dev->isconsole && priv->fd < 0)
    {
      priv->fd = host_uart_open(priv->path);
      if (priv->fd < 0)
        {
          return priv->fd;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tty_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void tty_shutdown(struct uart_dev_s *dev)
{
  struct tty_priv_s *priv = dev->priv;

  if (!dev->isconsole && priv->fd >= 0)
    {
      /* close file Description and reset fd */

      host_uart_close(priv->fd);
      priv->fd = -1;
    }
}

/****************************************************************************
 * Name: tty_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int tty_attach(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: tty_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void tty_detach(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: tty_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int tty_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct tty_priv_s *priv = dev->priv;
  struct termios *termiosp = (struct termios *)(uintptr_t)arg;

  if (!termiosp)
    {
      return -EINVAL;
    }

  switch (cmd)
    {
      case TCGETS:
        return host_uart_getcflag(dev->isconsole ? 0 : priv->fd,
                                  &termiosp->c_cflag);

      case TCSETS:
        return host_uart_setcflag(dev->isconsole ? 0 : priv->fd,
                                  termiosp->c_cflag);
    }
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Name: tty_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int tty_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct tty_priv_s *priv = dev->priv;

  *status = 0;
  return host_uart_getc(dev->isconsole ? 0 : priv->fd);
}

/****************************************************************************
 * Name: tty_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void tty_rxint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: tty_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool tty_rxavailable(struct uart_dev_s *dev)
{
  struct tty_priv_s *priv = dev->priv;

  return host_uart_checkin(dev->isconsole ? 0 : priv->fd);
}

/****************************************************************************
 * Name: tty_rxflowcontrol
 *
 * Description:
 *   Return true if UART activated RX flow control to block more incoming
 *   data.
 *
 ****************************************************************************/

static bool tty_rxflowcontrol(struct uart_dev_s *dev,
                              unsigned int nbuffered, bool upper)
{
  struct uart_buffer_s *rxbuf = &dev->recv;

  if (nbuffered == rxbuf->size)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: tty_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void tty_send(struct uart_dev_s *dev, int ch)
{
  struct tty_priv_s *priv = dev->priv;

  /* For console device */

  if (dev->isconsole && ch == '\n')
    {
      host_uart_putc(1, '\r');
    }

  host_uart_putc(dev->isconsole ? 1 : priv->fd, ch);
}

/****************************************************************************
 * Name: tty_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void tty_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      uart_xmitchars(dev);
    }
}

/****************************************************************************
 * Name: uart_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool tty_txready(struct uart_dev_s *dev)
{
  struct tty_priv_s *priv = dev->priv;

  return host_uart_checkout(dev->isconsole ? 0 : priv->fd);
}

/****************************************************************************
 * Name: tty_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool tty_txempty(struct uart_dev_s *dev)
{
  return true;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_uartinit
 ****************************************************************************/

void sim_uartinit(void)
{
#ifdef USE_DEVCONSOLE
  /* Start the simulated UART device */

  host_uart_start();

  /* Register console device */

  uart_register("/dev/console", &g_console_dev);
#endif

#ifdef CONFIG_SIM_UART0_NAME
  uart_register(CONFIG_SIM_UART0_NAME, &g_tty0_dev);
#endif

#ifdef CONFIG_SIM_UART1_NAME
  uart_register(CONFIG_SIM_UART1_NAME, &g_tty1_dev);
#endif

#ifdef CONFIG_SIM_UART2_NAME
  uart_register(CONFIG_SIM_UART2_NAME, &g_tty2_dev);
#endif

#ifdef CONFIG_SIM_UART3_NAME
  uart_register(CONFIG_SIM_UART3_NAME, &g_tty3_dev);
#endif
}

/****************************************************************************
 * Name: sim_uartloop
 ****************************************************************************/

void sim_uartloop(void)
{
#ifdef USE_DEVCONSOLE
  uart_recvchars(&g_console_dev);
  uart_xmitchars(&g_console_dev);
#endif

#ifdef CONFIG_SIM_UART0_NAME
  TTY_RECVSEND(0)
#endif

#ifdef CONFIG_SIM_UART1_NAME
  TTY_RECVSEND(1)
#endif

#ifdef CONFIG_SIM_UART2_NAME
  TTY_RECVSEND(2)
#endif

#ifdef CONFIG_SIM_UART3_NAME
  TTY_RECVSEND(3)
#endif
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef USE_DEVCONSOLE
  tty_send(&g_console_dev, ch);
#endif
  return 0;
}
