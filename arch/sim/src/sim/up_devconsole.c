/****************************************************************************
 * arch/sim/src/sim/up_devconsole.c
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

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVCONSOLE_BUFSIZE 256

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  devconsole_setup(FAR struct uart_dev_s *dev);
static void devconsole_shutdown(FAR struct uart_dev_s *dev);
static int  devconsole_attach(FAR struct uart_dev_s *dev);
static void devconsole_detach(FAR struct uart_dev_s *dev);
static int  devconsole_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int  devconsole_receive(FAR struct uart_dev_s *dev, uint32_t *status);
static void devconsole_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool devconsole_rxavailable(FAR struct uart_dev_s *dev);
static void devconsole_send(FAR struct uart_dev_s *dev, int ch);
static void devconsole_txint(FAR struct uart_dev_s *dev, bool enable);
static bool devconsole_txready(FAR struct uart_dev_s *dev);
static bool devconsole_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_devconsole_ops =
{
  .setup          = devconsole_setup,
  .shutdown       = devconsole_shutdown,
  .attach         = devconsole_attach,
  .detach         = devconsole_detach,
  .ioctl          = devconsole_ioctl,
  .receive        = devconsole_receive,
  .rxint          = devconsole_rxint,
  .rxavailable    = devconsole_rxavailable,
  .send           = devconsole_send,
  .txint          = devconsole_txint,
  .txready        = devconsole_txready,
  .txempty        = devconsole_txempty,
};

static char g_devconsole_rxbuf[DEVCONSOLE_BUFSIZE];
static char g_devconsole_txbuf[DEVCONSOLE_BUFSIZE];

static struct uart_dev_s g_devconsole_dev =
{
  .isconsole      = true,
  .ops            = &g_devconsole_ops,
  .xmit =
  {
    .size         = DEVCONSOLE_BUFSIZE,
    .buffer       = g_devconsole_txbuf,
  },
  .recv =
  {
    .size         = DEVCONSOLE_BUFSIZE,
    .buffer       = g_devconsole_rxbuf,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devconsole_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int devconsole_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: devconsole_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void devconsole_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: devconsole_attach
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

static int devconsole_attach(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: devconsole_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void devconsole_detach(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: devconsole_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int devconsole_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: devconsole_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int devconsole_receive(struct uart_dev_s *dev, uint32_t *status)
{
  *status = 0;
  return simuart_getc();
}

/****************************************************************************
 * Name: devconsole_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void devconsole_rxint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: devconsole_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool devconsole_rxavailable(struct uart_dev_s *dev)
{
  return simuart_checkc();
}

/****************************************************************************
 * Name: devconsole_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void devconsole_send(struct uart_dev_s *dev, int ch)
{
  simuart_putc(ch);
}

/****************************************************************************
 * Name: devconsole_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void devconsole_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      uart_xmitchars(&g_devconsole_dev);
    }
}

/****************************************************************************
 * Name: devconsole_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool devconsole_txready(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: devconsole_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool devconsole_txempty(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_devconsole
 ****************************************************************************/

void up_devconsole(void)
{
  uart_register("/dev/console", &g_devconsole_dev);
  uart_register("/dev/ttyS0", &g_devconsole_dev);
}

/****************************************************************************
 * Name: up_devconloop
 ****************************************************************************/

void up_devconloop(void)
{
  if (simuart_checkc())
    {
      uart_recvchars(&g_devconsole_dev);
    }
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  /* Just map to the host simuart_putc routine */

  return simuart_putc(ch);
}
