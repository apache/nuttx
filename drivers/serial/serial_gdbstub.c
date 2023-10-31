/****************************************************************************
 * drivers/serial/serial_gdbstub.c
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

#include <nuttx/serial/serial.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/kmalloc.h>
#include <nuttx/gdbstub.h>
#include <nuttx/nuttx.h>

#include <string.h>
#include <debug.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct uart_gdbstub_s
{
  FAR struct uart_dev_s *dev;
  FAR struct gdb_state_s *state;
  FAR const struct uart_ops_s *org_ops;
  struct uart_ops_s ops;
  struct notifier_block nb;
};

static FAR struct uart_gdbstub_s *g_uart_gdbstub;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_gdbstub_panic_callback
 *
 * Description:
 *   This is panic callback for gdbstub, If a crash occurs,
 *   you can debug it through gdb
 *
 ****************************************************************************/

static int uart_gdbstub_panic_callback(FAR struct notifier_block *nb,
                                       unsigned long action, FAR void *data)
{
  FAR struct uart_gdbstub_s *uart_gdbstub =
    container_of(nb, struct uart_gdbstub_s, nb);

  if (action != PANIC_KERNEL_FINAL)
    {
      return 0;
    }

  _alert("Enter panic gdbstub mode, plase use gdb connect to debug\n");
  _alert("Please use gdb of the corresponding architecture to "
         "connect to nuttx");
  _alert("such as: arm-none-eabi-gdb nuttx -ex \"set "
         "target-charset ASCII\" -ex \"target remote /dev/ttyUSB0\"\n");

  gdb_console_message(uart_gdbstub->state, "Enter panic gdbstub mode!\n");
  gdb_process(uart_gdbstub->state, GDBSTUB_STOPREASON_CTRLC, NULL);
  return 0;
}

/****************************************************************************
 * Name: uart_gdbstub_ctrlc
 *
 * Description:
 *   This is uart receive callback in interruption.
 *   The function is to accept the initial connection of Ctrl c and gdb.
 *
 ****************************************************************************/

static int uart_gdbstub_ctrlc(FAR struct uart_dev_s *dev,
                              FAR unsigned int *status)
{
  uart_disablerxint(dev);
  gdb_process(g_uart_gdbstub->state, GDBSTUB_STOPREASON_CTRLC, NULL);
  uart_enablerxint(dev);
  return 0;
}

/****************************************************************************
 * Name: uart_gdbstub_receive
 *
 * Description:
 *   This is gdbstub receive char function.
 *
 ****************************************************************************/

static ssize_t uart_gdbstub_receive(FAR void *priv, FAR void *buf,
                                    size_t len)
{
  FAR struct uart_gdbstub_s *uart_gdbstub = priv;
  FAR uart_dev_t *dev = uart_gdbstub->dev;
  FAR char *ptr = buf;
  unsigned int state;
  size_t i = 0;

  while (i < len)
    {
      if (uart_gdbstub->org_ops->rxavailable(dev))
        {
          ptr[i++] = g_uart_gdbstub->org_ops->receive(dev, &state);
        }
    }

  return len;
}

/****************************************************************************
 * Name: uart_gdbstub_send
 *
 * Description:
 *   This is gdbstub send char function.
 *
 ****************************************************************************/

static ssize_t uart_gdbstub_send(FAR void *priv, FAR void *buf, size_t len)
{
  FAR struct uart_gdbstub_s *uart_gdbstub = priv;
  FAR uart_dev_t *dev = uart_gdbstub->dev;
  size_t i = 0;

  while (i < len)
    {
      if (uart_gdbstub->org_ops->txready(dev))
        {
          uart_gdbstub->org_ops->send(dev, ((FAR char *)buf)[i++]);
        }
    }

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_gdbstub_register
 *
 * Description:
 *   Use the uart device to register gdbstub.
 *   gdbstub run with serial interrupt.
 *
 ****************************************************************************/

int uart_gdbstub_register(FAR uart_dev_t *dev)
{
  FAR struct uart_gdbstub_s *uart_gdbstub;

  uart_gdbstub = kmm_malloc(sizeof(struct uart_gdbstub_s));
  if (uart_gdbstub == NULL)
    {
      return -ENOMEM;
    }

  uart_gdbstub->state = gdb_state_init(uart_gdbstub_send,
                                       uart_gdbstub_receive,
                                       NULL, uart_gdbstub);
  if (uart_gdbstub->state == NULL)
    {
      kmm_free(uart_gdbstub);
      return -ENOMEM;
    }

  g_uart_gdbstub = uart_gdbstub;

  memcpy(&uart_gdbstub->ops, dev->ops, sizeof(struct uart_ops_s));
  uart_gdbstub->dev = dev;
  uart_gdbstub->org_ops = dev->ops;
  uart_gdbstub->ops.receive = uart_gdbstub_ctrlc;
  dev->ops = &uart_gdbstub->ops;
  uart_setup(dev);
  uart_attach(dev);
  uart_enablerxint(dev);

  uart_gdbstub->nb.notifier_call = uart_gdbstub_panic_callback;
  panic_notifier_chain_register(&uart_gdbstub->nb);

  return 0;
}
