/****************************************************************************
 * drivers/rpmsg/rpmsg_port_uart.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/crc16.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>

#include "rpmsg_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_PORT_UART_START              0x7f
#define RPMSG_PORT_UART_END                0x7e
#define RPMSG_PORT_UART_CONNREQ            0x7d
#define RPMSG_PORT_UART_CONNACK            0x7c
#define RPMSG_PORT_UART_ESCAPE             0x7b
#define RPMSG_PORT_UART_ESCAPE_MASK        0x20

#define RPMSG_PORT_UART_BUFLEN             256

#define RPMSG_PORT_UART_RX_WAIT_START      1
#define RPMSG_PORT_UART_RX_RECV_NORMAL     2
#define RPMSG_PORT_UART_RX_RECV_ESCAPE     3

#ifdef CONFIG_RPMSG_PORT_UART_CRC
#  define rpmsg_port_uart_crc16(hdr)       \
  crc16((FAR uint8_t *)&(hdr)->cmd, (hdr)->len - sizeof((hdr)->crc))
#else
#  define rpmsg_port_uart_crc16(hdr)       0
#endif

#ifdef CONFIG_RPMSG_PORT_UART_DEBUG
#  define rpmsgdump                        lib_dumpbuffer
#  define rpmsgdbg                         rpmsginfo
#else
#  define rpmsgdump(m,b,s)
#  define rpmsgdbg(fmt,...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_port_uart_s
{
  struct rpmsg_port_s    port;     /* Rpmsg port device */
  struct file            file;     /* Indicate uart device */
  char                   localcpu[RPMSG_NAME_SIZE];
  rpmsg_port_rx_cb_t     rx_cb;
  bool                   connected;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_port_uart_send_data(FAR struct rpmsg_port_uart_s *rpuart,
                                      FAR struct rpmsg_port_header_s *hdr);

static void rpmsg_port_uart_register_callback(FAR struct rpmsg_port_s *port,
                                              rpmsg_port_rx_cb_t callback);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_port_ops_s g_rpmsg_port_uart_ops =
{
  .notify_tx_ready   = NULL,
  .notify_rx_free    = NULL,
  .register_callback = rpmsg_port_uart_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_uart_send_packet
 *
 * Description:
 *   Send a data packet.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_packet(FAR struct rpmsg_port_uart_s *rpuart,
                                        FAR const void *data, size_t datalen)
{
  rpmsgdump("TX Packed Data", data, datalen);

  while (datalen > 0)
    {
      ssize_t ret = file_write(&rpuart->file, data, datalen);
      DEBUGASSERT(ret >= 0);

      data = (FAR uint8_t *)data + ret;
      datalen -= ret;
    }

  rpmsgdbg("Sent %zu Data\n", datalen);
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_frame
 *
 * Description:
 *   Send a frame.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_frame(FAR struct rpmsg_port_uart_s *rpuart,
                                       FAR const void *data, size_t datalen)
{
  uint8_t buf[RPMSG_PORT_UART_BUFLEN];
  uint8_t ch;
  size_t next = 0;

  rpmsgdump("Send Data", data, datalen);

  /* Pack start frame char first */

  buf[next++] = RPMSG_PORT_UART_START;

  /* Pack the data */

  for (; datalen-- > 0; data = (FAR uint8_t *)data + 1)
    {
      ch = *(FAR uint8_t *)data;
      if (ch >= RPMSG_PORT_UART_ESCAPE && ch <= RPMSG_PORT_UART_START)
        {
          buf[next++] = RPMSG_PORT_UART_ESCAPE;
          buf[next++] = ch ^ RPMSG_PORT_UART_ESCAPE_MASK;
        }
      else
        {
          buf[next++] = ch;
        }

      if (next > RPMSG_PORT_UART_BUFLEN - 2)
        {
          rpmsg_port_uart_send_packet(rpuart, buf, next);
          next = 0;
        }
    }

  /* Pack end frame char */

  buf[next++] = RPMSG_PORT_UART_END;

  /* Send this frame */

  rpmsg_port_uart_send_packet(rpuart, buf, next);
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_data
 *
 * Description:
 *   Send a data frame.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_data(FAR struct rpmsg_port_uart_s *rpuart,
                                      FAR struct rpmsg_port_header_s *hdr)
{
  rpmsgdbg("Send data len: %" PRIu16 "\n", hdr->len);

  hdr->cmd = 0;
  hdr->avail = 0;
  hdr->crc = rpmsg_port_uart_crc16(hdr);

  rpmsg_port_uart_send_frame(rpuart, hdr, hdr->len);
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_connect_req
 ****************************************************************************/

static void
rpmsg_port_uart_send_connect_req(FAR struct rpmsg_port_uart_s *rpuart)
{
  uint8_t ch = RPMSG_PORT_UART_CONNREQ;
  ssize_t ret = file_write(&rpuart->file, &ch, 1);
  if (ret != 1)
    {
      rpmsgerr("Send connect request failed, ret=%d\n", ret);
      PANIC();
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_connect_ack
 ****************************************************************************/

static void
rpmsg_port_uart_send_connect_ack(FAR struct rpmsg_port_uart_s *rpuart)
{
  uint8_t ch = RPMSG_PORT_UART_CONNACK;
  ssize_t ret = file_write(&rpuart->file, &ch, 1);
  if (ret != 1)
    {
      rpmsgerr("Send connect ack failed, ret=%d\n", ret);
      PANIC();
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_register_callback
 ****************************************************************************/

static void rpmsg_port_uart_register_callback(FAR struct rpmsg_port_s *port,
                                              rpmsg_port_rx_cb_t callback)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)port;

  rpuart->rx_cb = callback;
}

/****************************************************************************
 * Name: rpmsg_port_uart_process_rx_conn
 ****************************************************************************/

static void
rpmsg_port_uart_process_rx_conn(FAR struct rpmsg_port_uart_s *rpuart,
                                uint8_t ch)
{
  if (ch == RPMSG_PORT_UART_CONNREQ)
    {
      rpmsgdbg("Connect Request Command %d\n", rpuart->connected);
      if (rpuart->connected)
        {
          rpmsg_port_unregister(&rpuart->port);
        }
      else
        {
          rpuart->connected = true;
        }

      rpmsg_port_uart_send_connect_ack(rpuart);
      rpmsg_port_register(&rpuart->port, rpuart->localcpu);
    }
  else if (ch == RPMSG_PORT_UART_CONNACK)
    {
      rpmsgdbg("Connect Ack Command %d\n", rpuart->connected);
      if (!rpuart->connected)
        {
          rpuart->connected = true;
          rpmsg_port_register(&rpuart->port, rpuart->localcpu);
        }
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_rx_thread
 ****************************************************************************/

static int rpmsg_port_uart_rx_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)(uintptr_t)strtoul(argv[2], NULL, 16);
  FAR struct rpmsg_port_queue_s *rxq = &rpuart->port.rxq;
  FAR struct rpmsg_port_header_s *hdr = NULL;
  uint8_t state = RPMSG_PORT_UART_RX_WAIT_START;
  uint8_t buf[RPMSG_PORT_UART_BUFLEN];
  uint16_t next;

  for (; ; )
    {
      ssize_t ret;
      ssize_t i;

      ret = file_read(&rpuart->file, buf, sizeof(buf));
      if (ret < 0)
        {
          rpmsgerr("file_read failed, ret=%zd\n", ret);
          PANIC();
        }

      rpmsgdump("Received data:", buf, ret);

      for (i = 0; i < ret; i++)
        {
          if (buf[i] == RPMSG_PORT_UART_CONNREQ ||
              buf[i] == RPMSG_PORT_UART_CONNACK)
            {
              if (hdr != NULL)
                {
                  rpmsg_port_queue_return_buffer(rxq, hdr);
                  hdr = NULL;
                }

              rpmsg_port_uart_process_rx_conn(rpuart, buf[i]);
              state = RPMSG_PORT_UART_RX_WAIT_START;
              continue;
            }

          if (rpuart->connected == false)
            {
              continue;
            }

          switch (state)
            {
              case RPMSG_PORT_UART_RX_WAIT_START:
                if (buf[i] == RPMSG_PORT_UART_START)
                  {
                    if (hdr == NULL)
                      {
                        hdr = rpmsg_port_queue_get_available_buffer(rxq,
                                                                    true);
                        next = 0;
                      }

                    state = RPMSG_PORT_UART_RX_RECV_NORMAL;
                  }
                break;

              case RPMSG_PORT_UART_RX_RECV_NORMAL:
                if (buf[i] == RPMSG_PORT_UART_START)
                  {
                    rpmsgerr("Recv dup start char, i=%zd\n", i);
                    state = RPMSG_PORT_UART_RX_WAIT_START;
                  }
                else if (buf[i] == RPMSG_PORT_UART_END)
                  {
                    DEBUGASSERT(hdr->len == next);
                    DEBUGASSERT(hdr->crc == 0 ||
                                hdr->crc == rpmsg_port_uart_crc16(hdr));

                    if (rpuart->rx_cb != NULL)
                      {
                        rpuart->rx_cb(&rpuart->port, hdr);
                      }

                    state = RPMSG_PORT_UART_RX_WAIT_START;
                    hdr = NULL;
                  }
                else if (buf[i] == RPMSG_PORT_UART_ESCAPE)
                  {
                    state = RPMSG_PORT_UART_RX_RECV_ESCAPE;
                  }
                else
                  {
                    *((FAR char *)hdr + next++) = buf[i];
                  }
                break;

              case RPMSG_PORT_UART_RX_RECV_ESCAPE:
                *((FAR char *)hdr + next++) =
                  buf[i] ^ RPMSG_PORT_UART_ESCAPE_MASK;
                state = RPMSG_PORT_UART_RX_RECV_NORMAL;
                break;

              default:
                PANIC();
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_tx_thread
 ****************************************************************************/

static int rpmsg_port_uart_tx_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)(uintptr_t)strtoul(argv[2], NULL, 16);
  FAR struct rpmsg_port_queue_s *txq = &rpuart->port.txq;
  FAR struct rpmsg_port_header_s *hdr;

  rpmsg_port_uart_send_connect_req(rpuart);

  for (; ; )
    {
      while (!rpuart->connected)
        {
          nxsig_usleep(100000);
        }

      while ((hdr = rpmsg_port_queue_get_buffer(txq, true)) != NULL)
        {
          rpmsg_port_uart_send_data(rpuart, hdr);
          rpmsg_port_queue_return_buffer(txq, hdr);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_uart_initialize
 *
 * Description:
 *   Initialze a rpmsg_port_uart device to communicate between two chips.
 *
 * Input Parameters:
 *   cfg      - Configuration of buffers needed for communication.
 *   uartpath - Uart device path.
 *   localcpu - Local cpu name
 *
 ****************************************************************************/

int rpmsg_port_uart_initialize(FAR const struct rpmsg_port_config_s *cfg,
                               FAR const char *uartpath,
                               FAR const char *localcpu)
{
  FAR struct rpmsg_port_uart_s *rpuart;
  FAR char *argv[3];
  char arg1[32];
  int ret;
  int rx;

  rpuart = kmm_zalloc(sizeof(*rpuart));
  if (rpuart == NULL)
    {
      rpmsgerr("Malloc rptun uart failed\n");
      return -ENOMEM;
    }

  /* Hardware initialize */

  ret = file_open(&rpuart->file, uartpath, O_RDWR);
  if (ret < 0)
    {
      rpmsgerr("Open uart device failed, ret=%d\n", ret);
      goto err_file;
    }

  if (localcpu != NULL)
    {
      strlcpy(rpuart->localcpu, localcpu, sizeof(rpuart->localcpu));
    }

  ret = rpmsg_port_initialize(&rpuart->port, cfg, &g_rpmsg_port_uart_ops);
  if (ret < 0)
    {
      rpmsgerr("Port initialize failed, ret=%d\n", ret);
      goto err_rpmsg_port;
    }

  snprintf(arg1, sizeof(arg1), "%p", rpuart);

  argv[0] = (FAR char *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-uart-rx",
                       CONFIG_RPMSG_PORT_UART_RX_THREAD_PRIORITY,
                       CONFIG_RPMSG_PORT_UART_RX_THREAD_STACKSIZE,
                       rpmsg_port_uart_rx_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("Rx thread create failed, ret=%d\n", ret);
      goto err_rx_thread;
    }

  rx = ret;
  argv[0] = (FAR char *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-uart-tx",
                       CONFIG_RPMSG_PORT_UART_TX_THREAD_PRIORITY,
                       CONFIG_RPMSG_PORT_UART_TX_THREAD_STACKSIZE,
                       rpmsg_port_uart_tx_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("Tx thread create failed, ret=%d\n", ret);
      goto err_tx_thread;
    }

  return ret;

err_tx_thread:
  kthread_delete(rx);
err_rx_thread:
  rpmsg_port_uninitialize(&rpuart->port);
err_rpmsg_port:
  file_close(&rpuart->file);
err_file:
  kmm_free(rpuart);
  return ret;
}
