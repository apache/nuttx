/****************************************************************************
 * drivers/serial/serial_gdbstub.c
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

#include <nuttx/serial/serial.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/gdbstub.h>
#include <nuttx/nuttx.h>

#include <string.h>
#include <debug.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_gdbstub_s
{
  FAR struct uart_dev_s *dev;
  FAR struct uart_dev_s *console;
  FAR struct gdb_state_s *state;
  FAR const struct uart_ops_s *org_ops;
  struct uart_ops_s ops;
  struct notifier_block nb;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct uart_gdbstub_s *g_uart_gdbstub;

/****************************************************************************
 * Private Functions prototypes
 ****************************************************************************/

static int uart_gdbstub_ctrlc(FAR struct uart_dev_s *dev,
                              FAR unsigned int *status);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void uart_gdbstub_attach(FAR struct uart_gdbstub_s *uart_gdbstub,
                                bool replace)
{
  FAR uart_dev_t *dev = uart_gdbstub->dev;

  if (replace && uart_gdbstub->org_ops == NULL)
    {
      memcpy(&uart_gdbstub->ops, dev->ops, sizeof(struct uart_ops_s));
      uart_gdbstub->org_ops = dev->ops;
      uart_gdbstub->ops.receive = uart_gdbstub_ctrlc;
      dev->ops = &uart_gdbstub->ops;
    }

  uart_setup(dev);
  uart_attach(dev);
  uart_disablerxint(dev);
}

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
#if CONFIG_SERIAL_GDBSTUB_PANIC_TIMEOUT != 0
  unsigned int base;
  unsigned int status;
  char ch;
#endif

  if (action != PANIC_KERNEL_FINAL)
    {
      return 0;
    }

#if CONFIG_SERIAL_GDBSTUB_PANIC_TIMEOUT == 0
  gdb_console_message(uart_gdbstub->state,
                      "Enter panic gdbstub mode!\n");
#else
  _alert("Press Y/y key in %d seconds to enter gdb debug mode\n",
         CONFIG_SERIAL_GDBSTUB_PANIC_TIMEOUT);
  syslog_flush();

  if (uart_gdbstub->console == NULL)
    {
#ifndef CONFIG_SERIAL_GDBSTUB_AUTO_ATTACH
      uart_gdbstub_attach(uart_gdbstub, false);
#endif
      uart_gdbstub->console = uart_gdbstub->dev;
    }

  base = clock_systime_ticks();
  while (true)
    {
      if (uart_gdbstub->console == uart_gdbstub->dev &&
          uart_gdbstub->org_ops != NULL)
        {
          if (uart_gdbstub->org_ops->recvbuf)
            {
              if (uart_gdbstub->org_ops->rxavailable(uart_gdbstub->console))
                {
                  uart_gdbstub->org_ops->recvbuf(uart_gdbstub->console,
                                                 &ch, 1);
                }
            }
          else
            {
              ch = uart_gdbstub->org_ops->receive(uart_gdbstub->console,
                                                  &status);
            }
        }
      else
        {
          if (uart_gdbstub->console->ops->recvbuf)
            {
              if (uart_gdbstub->console->ops->rxavailable(
                                              uart_gdbstub->console))
                {
                  uart_gdbstub->console->ops->recvbuf(uart_gdbstub->console,
                                                      &ch, 1);
                }
            }
          else
            {
              ch = uart_gdbstub->console->ops->receive(uart_gdbstub->console,
                                                       &status);
            }
        }

      if (ch == 'Y' || ch == 'y')
        {
          break;
        }

      if ((clock_systime_ticks()) - base >=
           SEC2TICK(CONFIG_SERIAL_GDBSTUB_PANIC_TIMEOUT))
        {
          _alert("%d seconds passed, exit now\n",
                 CONFIG_SERIAL_GDBSTUB_PANIC_TIMEOUT);
          return 0;
        }
    }
#endif

#ifndef CONFIG_SERIAL_GDBSTUB_AUTO_ATTACH
  uart_gdbstub_attach(uart_gdbstub, true);
#endif

  _alert("Enter panic gdbstub mode, plase use gdb connect to debug\n");
  _alert("Please use gdb of the corresponding architecture to "
         "connect to nuttx");
  _alert("such as: arm-none-eabi-gdb nuttx -ex \"set "
         "target-charset ASCII\" -ex \"target remote /dev/ttyUSB0\"\n");

  syslog_flush();
  gdb_process(uart_gdbstub->state, GDB_STOPREASON_NONE, NULL);
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
  gdb_process(g_uart_gdbstub->state, GDB_STOPREASON_CTRLC, NULL);
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
          if (uart_gdbstub->org_ops->recvbuf)
            {
              i += uart_gdbstub->org_ops->recvbuf(dev, ptr + i, len - i);
            }
          else
            {
              ptr[i++] = uart_gdbstub->org_ops->receive(dev, &state);
            }
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

static ssize_t uart_gdbstub_send(FAR void *priv, FAR const char *buf,
                                 size_t len)
{
  FAR struct uart_gdbstub_s *uart_gdbstub = priv;
  FAR uart_dev_t *dev = uart_gdbstub->dev;
  size_t i = 0;

  while (i < len)
    {
      if (uart_gdbstub->org_ops->txready(dev))
        {
          if (uart_gdbstub->org_ops->sendbuf)
            {
              i += uart_gdbstub->org_ops->sendbuf(dev, buf + i, len - i);
            }
          else
            {
              uart_gdbstub->org_ops->send(dev, buf[i++]);
            }
        }
    }

  while (!uart_gdbstub->org_ops->txempty(dev));

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

int uart_gdbstub_register(FAR uart_dev_t *dev, FAR const char *path)
{
  FAR struct uart_gdbstub_s *uart_gdbstub;

  if (g_uart_gdbstub == NULL)
    {
      uart_gdbstub = kmm_zalloc(sizeof(struct uart_gdbstub_s));
      if (uart_gdbstub == NULL)
        {
          return -ENOMEM;
        }

      g_uart_gdbstub = uart_gdbstub;
    }
  else
    {
      uart_gdbstub = g_uart_gdbstub;
    }

  if (dev->isconsole && uart_gdbstub->console == NULL)
    {
      uart_gdbstub->console = dev;
    }

  if (strcmp(path, CONFIG_SERIAL_GDBSTUB_PATH) != 0)
    {
      return -EINVAL;
    }

  uart_gdbstub->state = gdb_state_init(uart_gdbstub_send,
                                       uart_gdbstub_receive,
                                       NULL, uart_gdbstub);
  if (uart_gdbstub->state == NULL)
    {
      kmm_free(uart_gdbstub);
      return -ENOMEM;
    }

  uart_gdbstub->dev = dev;
  uart_gdbstub->nb.notifier_call = uart_gdbstub_panic_callback;
  panic_notifier_chain_register(&uart_gdbstub->nb);

#ifdef CONFIG_SERIAL_GDBSTUB_AUTO_ATTACH
  uart_gdbstub_attach(uart_gdbstub, true);
  return 0;
#else
  return 1;
#endif
}
