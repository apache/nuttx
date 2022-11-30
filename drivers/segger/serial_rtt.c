/****************************************************************************
 * drivers/segger/serial_rtt.c
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
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/list.h>
#include <nuttx/serial/serial.h>
#include <nuttx/signal.h>

#include <SEGGER_RTT.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct serial_rtt_dev_s
{
  struct list_node node;
  struct uart_dev_s uart;
  int channel;
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct list_node g_serial_rtt_list = LIST_INITIAL_VALUE(g_serial_rtt_list);

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
  return OK;
}

/****************************************************************************
 * Name: serial_rtt_detach
 ****************************************************************************/

static void serial_rtt_detach(FAR struct uart_dev_s *dev)
{
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
  return -ENOSYS;
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
  FAR struct serial_rtt_dev_s *rtt =
             (FAR struct serial_rtt_dev_s *)dev->priv;
  return SEGGER_RTT_HasData(rtt->channel) != 0;
}

/****************************************************************************
 * Name: serial_rtt_dmasend
 ****************************************************************************/

static void serial_rtt_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_dev_s *rtt =
      (FAR struct serial_rtt_dev_s *)dev->priv;
  FAR SEGGER_RTT_BUFFER_UP *ring = (SEGGER_RTT_BUFFER_UP *)((FAR char *)
      &_SEGGER_RTT.aUp[rtt->channel] + SEGGER_RTT_UNCACHED_OFF);
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  size_t len = xfer->length + xfer->nlength;
  size_t slen = 0;

  if (rtt->channel)
    {
      ring->WrOff = (ring->WrOff + len) % dev->xmit.size;
    }
  else
    {
      slen += SEGGER_RTT_Write(rtt->channel, xfer->buffer, xfer->length);
      slen += SEGGER_RTT_Write(rtt->channel, xfer->nbuffer, xfer->nlength);
    }

  xfer->nbytes = slen;
  uart_xmitchars_done(dev);
}

/****************************************************************************
 * Name: serial_rtt_dmareceive
 ****************************************************************************/

static void serial_rtt_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_dev_s *rtt =
             (FAR struct serial_rtt_dev_s *)dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR SEGGER_RTT_BUFFER_DOWN *ring = (SEGGER_RTT_BUFFER_DOWN *)((FAR char *)
      &_SEGGER_RTT.aDown[rtt->channel] + SEGGER_RTT_UNCACHED_OFF);
  size_t space = xfer->length + xfer->nlength;
  size_t recvlen = 0;

  if (rtt->channel)
    {
      if (ring->RdOff <= ring->WrOff)
        {
          recvlen = ring->WrOff - ring->RdOff;
        }
      else
        {
          recvlen = ring->SizeOfBuffer - (ring->WrOff - ring->RdOff);
        }

      recvlen = (recvlen > space) ? space : recvlen;
      ring->RdOff = recvlen;
    }
  else
    {
      recvlen += SEGGER_RTT_Read(rtt->channel, xfer->buffer, xfer->length);
      if ((recvlen == xfer->length) && SEGGER_RTT_HasData(rtt->channel))
        {
          recvlen += SEGGER_RTT_Read(rtt->channel, xfer->nbuffer,
                                     xfer->nlength);
        }
    }

  xfer->nbytes = recvlen;
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
  uart_xmitchars_dma(dev);
}

/****************************************************************************
 * Name: serial_rtt_send
 ****************************************************************************/

static void serial_rtt_send(FAR struct uart_dev_s *dev, int ch)
{
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
    FAR struct serial_rtt_dev_s *rtt =
             (FAR struct serial_rtt_dev_s *)dev->priv;
  return SEGGER_RTT_GetAvailWriteSpace(rtt->channel) != 0;
}

/****************************************************************************
 * Name: serial_rtt_txempty
 ****************************************************************************/

static bool serial_rtt_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct serial_rtt_dev_s *rtt =
             (FAR struct serial_rtt_dev_s *)dev->priv;
  return SEGGER_RTT_GetBytesInBuffer(rtt->channel) == 0;
}

/****************************************************************************
 * Name: serial_rtt_loop
 ****************************************************************************/

static int serial_rtt_loop(int argc, FAR char *argv[])
{
  while (1)
    {
      FAR struct serial_rtt_dev_s *rtt;
      irqstate_t flags;

      nxsig_usleep(CONFIG_SERIAL_RTT_POLLING_INTERVAL);
      flags = spin_lock_irqsave(NULL);
      list_for_every_entry(&g_serial_rtt_list,
                           rtt, struct serial_rtt_dev_s, node)
        {
          spin_unlock_irqrestore(NULL, flags);

          /* Check if there is data update */

          serial_rtt_dmarxfree(&rtt->uart);
          flags = spin_lock_irqsave(NULL);
        }

      spin_unlock_irqrestore(NULL, flags);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void serial_rtt_register(FAR const char *name, int channel,
                         bool isconsole, size_t txsize, size_t rxsize)
{
  irqstate_t flags;
  FAR struct serial_rtt_dev_s *rtt =
             kmm_zalloc(sizeof(struct serial_rtt_dev_s));
  rtt->channel = channel;
  rtt->uart.isconsole = isconsole;
  rtt->uart.ops = &g_serial_rtt_ops;
  rtt->uart.priv = rtt;
  rtt->uart.xmit.size = txsize;
  rtt->uart.xmit.buffer = kmm_zalloc(txsize);
  DEBUGASSERT(rtt->uart.xmit.buffer);
  rtt->uart.recv.size = rxsize;
  rtt->uart.recv.buffer = kmm_zalloc(rxsize);
  DEBUGASSERT(rtt->uart.recv.buffer);

  if (channel != 0)
    {
      SEGGER_RTT_ConfigUpBuffer(channel, name, rtt->uart.xmit.buffer,
                                txsize, SEGGER_RTT_MODE_DEFAULT);
      SEGGER_RTT_ConfigDownBuffer(channel, name, rtt->uart.recv.buffer,
                                  rxsize, SEGGER_RTT_MODE_DEFAULT);
    }

  if (list_is_empty(&g_serial_rtt_list))
    {
      kthread_create("serial_rtt_daemon", SCHED_PRIORITY_MAX,
                     CONFIG_DEFAULT_TASK_STACKSIZE, serial_rtt_loop, NULL);
    }

  flags = spin_lock_irqsave(NULL);
  list_add_tail(&g_serial_rtt_list, &rtt->node);
  spin_unlock_irqrestore(NULL, flags);
  uart_register(name, &rtt->uart);
}
