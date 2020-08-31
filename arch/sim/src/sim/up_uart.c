/****************************************************************************
 * arch/sim/src/sim/up_uart.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "up_internal.h"

#if defined(USE_DEVCONSOLE) || CONFIG_SIM_UART_NUMBER > 0
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFSIZE 256

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tty_priv_s
{
  /* tty-port path name */

  FAR const char *path;

  /* The file descriptor. It is returned by open */

  int             fd;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tty_setup(FAR struct uart_dev_s *dev);
static void tty_shutdown(FAR struct uart_dev_s *dev);
static int  tty_attach(FAR struct uart_dev_s *dev);
static void tty_detach(FAR struct uart_dev_s *dev);
static int  tty_ioctl(FAR struct file *filep, int cmd,
                      unsigned long arg);
static int  tty_receive(FAR struct uart_dev_s *dev, uint32_t *status);
static void tty_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool tty_rxavailable(FAR struct uart_dev_s *dev);
static void tty_send(FAR struct uart_dev_s *dev, int ch);
static void tty_txint(FAR struct uart_dev_s *dev, bool enable);
static bool tty_txready(FAR struct uart_dev_s *dev);
static bool tty_txempty(FAR struct uart_dev_s *dev);

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
  .send           = tty_send,
  .txint          = tty_txint,
  .txready        = tty_txready,
  .txempty        = tty_txempty,
};
#endif

#ifdef USE_DEVCONSOLE
static char g_console_rxbuf[BUFSIZE];
static char g_console_txbuf[BUFSIZE];
#endif

#ifdef CONFIG_SIM_UART0_NAME
static char g_tty0_rxbuf[BUFSIZE];
static char g_tty0_txbuf[BUFSIZE];
#endif

#ifdef CONFIG_SIM_UART1_NAME
static char g_tty1_rxbuf[BUFSIZE];
static char g_tty1_txbuf[BUFSIZE];
#endif

#ifdef CONFIG_SIM_UART2_NAME
static char g_tty2_rxbuf[BUFSIZE];
static char g_tty2_txbuf[BUFSIZE];
#endif

#ifdef CONFIG_SIM_UART3_NAME
static char g_tty3_rxbuf[BUFSIZE];
static char g_tty3_txbuf[BUFSIZE];
#endif

#ifdef USE_DEVCONSOLE
static struct uart_dev_s g_console_dev =
{
  .isconsole      = true,
  .ops            = &g_tty_ops,
  .xmit =
  {
    .size         = BUFSIZE,
    .buffer       = g_console_txbuf,
  },
  .recv =
  {
    .size         = BUFSIZE,
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
    .size         = BUFSIZE,
    .buffer       = g_tty0_txbuf,
  },
  .recv =
  {
    .size         = BUFSIZE,
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
    .size         = BUFSIZE,
    .buffer       = g_tty1_txbuf,
  },
  .recv =
  {
    .size         = BUFSIZE,
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
    .size         = BUFSIZE,
    .buffer       = g_tty2_txbuf,
  },
  .recv =
  {
    .size         = BUFSIZE,
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
    .size         = BUFSIZE,
    .buffer       = g_tty3_txbuf,
  },
  .recv =
  {
    .size         = BUFSIZE,
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

static int tty_setup(FAR struct uart_dev_s *dev)
{
  FAR struct tty_priv_s *priv = dev->priv;

  if (!dev->isconsole && priv->fd < 0)
    {
      priv->fd = simuart_open(priv->path);
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
  FAR struct tty_priv_s *priv = dev->priv;

  if (!dev->isconsole && priv->fd >= 0)
    {
      /* close file Description and reset fd */

      simuart_close(priv->fd);
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

static void tty_detach(FAR struct uart_dev_s *dev)
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
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_dev_s *dev = inode->i_private;
  FAR struct tty_priv_s *priv = dev->priv;
  FAR struct termios *termiosp = (FAR struct termios *)(uintptr_t)arg;

  if (!termiosp)
    {
      return -EINVAL;
    }

  switch (cmd)
    {
      case TCGETS:
        return simuart_getcflag(dev->isconsole ? 0 : priv->fd,
                                &termiosp->c_cflag);

      case TCSETS:
        return simuart_setcflag(dev->isconsole ? 0 : priv->fd,
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
  FAR struct tty_priv_s *priv = dev->priv;

  *status = 0;
  return simuart_getc(dev->isconsole ? 0 : priv->fd);
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
  FAR struct tty_priv_s *priv = dev->priv;

  return simuart_checkc(dev->isconsole ? 0 : priv->fd);
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
  FAR struct tty_priv_s *priv = dev->priv;

  /* For console device */

  if (dev->isconsole && ch == '\n')
    {
      simuart_putc(1, '\r');
    }

  simuart_putc(dev->isconsole ? 1 : priv->fd, ch);
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
  return true;
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
 * Name: up_uart
 ****************************************************************************/

void up_uartinit(void)
{
#ifdef USE_DEVCONSOLE
  /* Start the simulated UART device */

  simuart_start();

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
 * Name: up_uartloop
 ****************************************************************************/

void up_uartloop(void)
{
#ifdef USE_DEVCONSOLE
  if (simuart_checkc(0))
    {
      uart_recvchars(&g_console_dev);
    }
#endif

#ifdef CONFIG_SIM_UART0_NAME
  if (g_tty0_priv.fd > 0 && simuart_checkc(g_tty0_priv.fd))
    {
      uart_recvchars(&g_tty0_dev);
    }
#endif

#ifdef CONFIG_SIM_UART1_NAME
  if (g_tty1_priv.fd > 0 && simuart_checkc(g_tty1_priv.fd))
    {
      uart_recvchars(&g_tty1_dev);
    }
#endif

#ifdef CONFIG_SIM_UART2_NAME
  if (g_tty2_priv.fd > 0 && simuart_checkc(g_tty2_priv.fd))
    {
      uart_recvchars(&g_tty2_dev);
    }
#endif

#ifdef CONFIG_SIM_UART3_NAME
  if (g_tty3_priv.fd > 0 && simuart_checkc(g_tty3_priv.fd))
    {
      uart_recvchars(&g_tty3_dev);
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

#ifdef USE_DEVCONSOLE
int up_putc(int ch)
{
  tty_send(&g_console_dev, ch);
  return 0;
}
#endif
