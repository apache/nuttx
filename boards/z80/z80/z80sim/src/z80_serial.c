/****************************************************************************
 * boards/z80/z80/z80sim/src/z80_serial.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include "z80_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(FAR struct uart_dev_s *dev);
static void up_shutdown(FAR struct uart_dev_s *dev);
static int  up_attach(FAR struct uart_dev_s *dev);
static void up_detach(FAR struct uart_dev_s *dev);
static int  up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  up_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(FAR struct uart_dev_s *dev);
static void up_send(FAR struct uart_dev_s *dev, int ch);
static void up_txint(FAR struct uart_dev_s *dev, bool enable);
static bool up_txready(FAR struct uart_dev_s *dev);
static bool up_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  up_setup,                 /* setup */
  up_shutdown,              /* shutdown */
  up_attach,                /* attach */
  up_detach,                /* detach */
  up_ioctl,                 /* ioctl */
  up_receive,               /* receive */
  up_rxint,                 /* rxint */
  up_rxavailable,           /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,                     /* rxflowcontrol */
#endif
  up_send,                  /* send */
  up_txint,                 /* txint */
  up_txready,               /* txready */
  up_txempty,               /* txempty */
};

/* I/O buffers */

static char g_uartrxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_uarttxbuffer[CONFIG_UART_TXBUFSIZE];

/* This describes the state of the fake UART port. */

static uart_dev_t g_uartport =
{
  0,                        /* open_count */
  false,                    /* xmitwaiting */
  false,                    /* recvwaiting */
  true,                     /* isconsole */
  { 1 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  { 0 },                    /* pollsem */
  {                         /* xmit */
    { 1 },                  /*   sem */
    0,                      /*   head */
    0,                      /*   tail */
    CONFIG_UART_TXBUFSIZE,  /*   size */
    g_uarttxbuffer          /*   buffer */
  },
  {                         /* recv */
    { 1 },                  /*   sem */
    0,                      /*   head */
    0,                      /*   tail */
    CONFIG_UART_RXBUFSIZE,  /*   size */
    g_uartrxbuffer          /*   buffer */
  },
  &g_uart_ops,              /* ops */
  NULL,                     /* priv */
  {                         /* pollfds */
      NULL
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev)
{
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

static void up_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int up_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
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

static int up_receive(FAR struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t ch = z80_lowgetc();
  *status = 0;
  return ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(FAR struct uart_dev_s *dev, int ch)
{
  z80_lowputc(ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_serial_initialize
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

void z80_serial_initialize(void)
{
  uart_register("/dev/console", &g_uartport);
  uart_register("/dev/ttyS0", &g_uartport);
}

#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  z80_lowputc(ch);
  return 0;
}
