/****************************************************************************
 * drivers/segger/serial_rtt.c
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

#include <assert.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/segger/rtt.h>
#include <nuttx/serial/serial.h>
#include <nuttx/wdog.h>

#include <SEGGER_RTT.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef SEGGER_RTT_BUFFER_SECTION
#  define SERIAL_RTT_BUFFER_SECTION locate_data(SEGGER_RTT_BUFFER_SECTION)
#else
#  define SERIAL_RTT_BUFFER_SECTION
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct serial_rtt_s
{
  struct uart_dev_s uart;
  struct wdog_s wdog;
  int channel;
  FAR char *up_buffer;
  FAR char *down_buffer;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int serial_rtt_setup(FAR struct uart_dev_s *dev);
static void serial_rtt_shutdown(FAR struct uart_dev_s *dev);
static int serial_rtt_attach(FAR struct uart_dev_s *dev);
static void serial_rtt_detach(FAR struct uart_dev_s *dev);
static int serial_rtt_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int serial_rtt_receive(FAR struct uart_dev_s *dev,
                              FAR unsigned int *status);
static void serial_rtt_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool serial_rtt_rxavailable(FAR struct uart_dev_s *dev);
static void serial_rtt_dmasend(FAR struct uart_dev_s *dev);
static void serial_rtt_dmareceive(FAR struct uart_dev_s *dev);
static void serial_rtt_dmarxfree(FAR struct uart_dev_s *dev);
static void serial_rtt_dmatxavail(FAR struct uart_dev_s *dev);
static void serial_rtt_send(FAR struct uart_dev_s *dev, int ch);
static void serial_rtt_txint(FAR struct uart_dev_s *dev, bool enable);
static bool serial_rtt_txready(FAR struct uart_dev_s *dev);
static bool serial_rtt_txempty(FAR struct uart_dev_s *dev);

static void serial_rtt_timeout(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_serial_rtt_ops =
{
  serial_rtt_setup,
  serial_rtt_shutdown,
  serial_rtt_attach,
  serial_rtt_detach,
  serial_rtt_ioctl,
  serial_rtt_receive,
  serial_rtt_rxint,
  serial_rtt_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,
#endif
  serial_rtt_dmasend,
  serial_rtt_dmareceive,
  serial_rtt_dmarxfree,
  serial_rtt_dmatxavail,
  serial_rtt_send,
  serial_rtt_txint,
  serial_rtt_txready,
  serial_rtt_txempty,
};

#ifdef CONFIG_SERIAL_RTT0
static char g_rtt0_xmit_buffer[CONFIG_SEGGER_RTT_BUFFER_SIZE_UP];
static char g_rtt0_recv_buffer[CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN];

static struct serial_rtt_s g_serial_rtt0 =
{
  .uart =
  {
#ifdef CONFIG_SERIAL_RTT_CONSOLE
    .isconsole = CONFIG_SERIAL_RTT_CONSOLE_CHANNEL == 0,
#endif
    .recv =
    {
      .buffer = g_rtt0_recv_buffer,
      .size = CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN,
    },
    .xmit =
    {
      .buffer = g_rtt0_xmit_buffer,
      .size = CONFIG_SEGGER_RTT_BUFFER_SIZE_UP,
    },
    .ops = &g_serial_rtt_ops,
    .priv = &g_serial_rtt0,
  },
  .channel = 0,
  .up_buffer = NULL,
  .down_buffer = NULL,
};
#endif

#ifdef CONFIG_SERIAL_RTT1
static char g_rtt1_xmit_buffer[CONFIG_SEGGER_RTT1_BUFFER_SIZE_UP];
static char g_rtt1_recv_buffer[CONFIG_SEGGER_RTT1_BUFFER_SIZE_DOWN];

static char SERIAL_RTT_BUFFER_SECTION
g_rtt1_up_buffer[CONFIG_SEGGER_RTT1_BUFFER_SIZE_UP];
static char SERIAL_RTT_BUFFER_SECTION
g_rtt1_down_buffer[CONFIG_SEGGER_RTT1_BUFFER_SIZE_DOWN];

static struct serial_rtt_s g_serial_rtt1 =
{
  .uart =
  {
#ifdef CONFIG_SERIAL_RTT_CONSOLE
    .isconsole = CONFIG_SERIAL_RTT_CONSOLE_CHANNEL == 1,
#endif
    .recv =
    {
      .buffer = g_rtt1_recv_buffer,
      .size = CONFIG_SEGGER_RTT1_BUFFER_SIZE_DOWN,
    },
    .xmit =
    {
      .buffer = g_rtt1_xmit_buffer,
      .size = CONFIG_SEGGER_RTT1_BUFFER_SIZE_UP,
    },
    .ops = &g_serial_rtt_ops,
    .priv = &g_serial_rtt1,
  },
  .channel = 1,
  .up_buffer = g_rtt1_up_buffer,
  .down_buffer = g_rtt1_down_buffer,
};
#endif

#ifdef CONFIG_SERIAL_RTT2
static char g_rtt2_xmit_buffer[CONFIG_SEGGER_RTT2_BUFFER_SIZE_UP];
static char g_rtt2_recv_buffer[CONFIG_SEGGER_RTT2_BUFFER_SIZE_DOWN];

static char SERIAL_RTT_BUFFER_SECTION
g_rtt2_up_buffer[CONFIG_SEGGER_RTT2_BUFFER_SIZE_UP];
static char SERIAL_RTT_BUFFER_SECTION
g_rtt2_down_buffer[CONFIG_SEGGER_RTT2_BUFFER_SIZE_DOWN];

static struct serial_rtt_s g_serial_rtt2 =
{
  .uart =
  {
#ifdef CONFIG_SERIAL_RTT_CONSOLE
    .isconsole = CONFIG_SERIAL_RTT_CONSOLE_CHANNEL == 2,
#endif
    .recv =
    {
      .buffer = g_rtt2_recv_buffer,
      .size = CONFIG_SEGGER_RTT2_BUFFER_SIZE_DOWN,
    },
    .xmit =
    {
      .buffer = g_rtt2_xmit_buffer,
      .size = CONFIG_SEGGER_RTT2_BUFFER_SIZE_UP,
    },
    .ops = &g_serial_rtt_ops,
    .priv = &g_serial_rtt2,
  },
  .channel = 2,
  .up_buffer = g_rtt2_up_buffer,
  .down_buffer = g_rtt2_down_buffer,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: serial_rtt_setup
 ****************************************************************************/

static int serial_rtt_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: serial_rtt_shutdown
 ****************************************************************************/

static void serial_rtt_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: serial_rtt_attach
 ****************************************************************************/

static int serial_rtt_attach(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  wd_start(&rtt->wdog, USEC2TICK(CONFIG_SERIAL_RTT_POLLING_INTERVAL),
           serial_rtt_timeout, (wdparm_t)dev);
  return OK;
}

/****************************************************************************
 * Name: serial_rtt_detach
 ****************************************************************************/

static void serial_rtt_detach(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  wd_cancel(&rtt->wdog);
}

/****************************************************************************
 * Name: serial_rtt_ioctl
 ****************************************************************************/

static int serial_rtt_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: serial_rtt_receive
 ****************************************************************************/

static int serial_rtt_receive(FAR struct uart_dev_s *dev,
                              FAR unsigned int *status)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  int ret;
  int ch;

  ret = SEGGER_RTT_ReadNoLock(rtt->channel, &ch, 1);
  *status = ret == 1 ? 0 : -EAGAIN;
  return ch;
}

/****************************************************************************
 * Name: serial_rtt_rxint
 ****************************************************************************/

static void serial_rtt_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: serial_rtt_rxavailable
 ****************************************************************************/

static bool serial_rtt_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  return SEGGER_RTT_HasData(rtt->channel) != 0;
}

/****************************************************************************
 * Name: serial_rtt_dmasend
 ****************************************************************************/

static void serial_rtt_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  size_t len;

  SEGGER_RTT_BLOCK_IF_FIFO_FULL(rtt->channel);
  len = SEGGER_RTT_WriteNoLock(rtt->channel, xfer->buffer, xfer->length);
  if (len == xfer->length && xfer->nlength)
    {
      len += SEGGER_RTT_WriteNoLock(rtt->channel, xfer->nbuffer,
                                    xfer->nlength);
    }

  xfer->nbytes = len;
  uart_xmitchars_done(dev);
}

/****************************************************************************
 * Name: serial_rtt_dmareceive
 ****************************************************************************/

static void serial_rtt_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  size_t len;

  len = SEGGER_RTT_ReadNoLock(rtt->channel, xfer->buffer, xfer->length);
  if (len == xfer->length && xfer->nbuffer &&
      SEGGER_RTT_HasData(rtt->channel))
    {
      len += SEGGER_RTT_ReadNoLock(rtt->channel, xfer->nbuffer,
                                   xfer->nlength);
    }

  xfer->nbytes = len;
  uart_recvchars_done(dev);
}

/****************************************************************************
 * Name: serial_rtt_dmarxfree
 ****************************************************************************/

static void serial_rtt_dmarxfree(FAR struct uart_dev_s *dev)
{
  /* When the DMA buffer is empty, check whether there is data to read */

  if (serial_rtt_rxavailable(dev))
    {
      uart_recvchars_dma(dev);
    }
}

/****************************************************************************
 * Name: serial_rtt_dmatxavail
 ****************************************************************************/

static void serial_rtt_dmatxavail(FAR struct uart_dev_s *dev)
{
  if (serial_rtt_txready(dev))
    {
      uart_xmitchars_dma(dev);
    }
}

/****************************************************************************
 * Name: serial_rtt_send
 ****************************************************************************/

static void serial_rtt_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct serial_rtt_s *rtt = dev->priv;

  SEGGER_RTT_BLOCK_IF_FIFO_FULL(rtt->channel);
  SEGGER_RTT_PutChar(rtt->channel, ch);
}

/****************************************************************************
 * Name: serial_rtt_txint
 ****************************************************************************/

static void serial_rtt_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: serial_rtt_txready
 ****************************************************************************/

static bool serial_rtt_txready(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  return SEGGER_RTT_GetAvailWriteSpace(rtt->channel) != 0;
}

/****************************************************************************
 * Name: serial_rtt_txempty
 ****************************************************************************/

static bool serial_rtt_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_s *rtt = dev->priv;
  return SEGGER_RTT_GetBytesInBuffer(rtt->channel) == 0;
}

/****************************************************************************
 * Name: serial_rtt_timeout
 ****************************************************************************/

static void serial_rtt_timeout(wdparm_t arg)
{
  FAR struct serial_rtt_s *rtt = (FAR struct serial_rtt_s *)arg;

  serial_rtt_dmarxfree(&rtt->uart);
  serial_rtt_dmatxavail(&rtt->uart);
  wd_start(&rtt->wdog, USEC2TICK(CONFIG_SERIAL_RTT_POLLING_INTERVAL),
           serial_rtt_timeout, arg);
}

/****************************************************************************
 * Name: serial_rtt_register
 ****************************************************************************/

static void serial_rtt_register(FAR const char *name,
                                FAR struct serial_rtt_s *rtt)
{
  SEGGER_RTT_ConfigUpBuffer(rtt->channel, name, rtt->up_buffer,
                            rtt->uart.xmit.size,
                            SEGGER_RTT_MODE_NO_BLOCK_TRIM);
  SEGGER_RTT_ConfigDownBuffer(rtt->channel, name, rtt->down_buffer,
                              rtt->uart.recv.size,
                              SEGGER_RTT_MODE_NO_BLOCK_TRIM);

#ifdef CONFIG_SERIAL_RTT_CONSOLE
  if (rtt->uart.isconsole)
    {
      uart_register("/dev/console", &rtt->uart);
    }
#endif

  uart_register(name, &rtt->uart);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * serial_rtt_initialize
 ****************************************************************************/

void serial_rtt_initialize(void)
{
#ifdef CONFIG_SERIAL_RTT0
  serial_rtt_register("/dev/ttyR0", &g_serial_rtt0);
#endif

#ifdef CONFIG_SERIAL_RTT1
  serial_rtt_register("/dev/ttyR1", &g_serial_rtt1);
#endif

#ifdef CONFIG_SERIAL_RTT2
  serial_rtt_register("/dev/ttyR2", &g_serial_rtt2);
#endif
}
