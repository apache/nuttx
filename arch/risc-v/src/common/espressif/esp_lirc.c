/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_lirc.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/circbuf.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/lirc.h>
#include <nuttx/rc/lirc_dev.h>
#include <nuttx/semaphore.h>

#include "hal/rmt_types.h"

#include "esp_rmt.h"
#include "esp_lirc.h"

#if defined(CONFIG_ESP_RMT) && defined(CONFIG_DRIVERS_RC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RMT channel clock: 10 MHz → 1 tick = 0.1 µs */

#define RMT_TICK_TO_US(t)  ((t) / 10)
#define RMT_US_TO_TICK(u)  ((u) * 10)

/* DMA receive scratch buffer size (in bytes, must be multiple of 4) */

#define LIRC_RX_BUF_BYTES  CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE

/* Stack size for the RX worker thread */

#define LIRC_RX_THREAD_STACK  2048

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_lirc_dev_s
{
  struct lirc_lowerhalf_s lower;   /* Must be first — lirc upper-half iface */
  FAR struct rmt_dev_s   *rmt;     /* ESP RMT lower-half */
  volatile bool           running; /* RX thread keep-alive flag */
  pid_t                   rxpid;   /* RX worker thread PID */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  esp_lirc_open(FAR struct lirc_lowerhalf_s *lower);
static void esp_lirc_close(FAR struct lirc_lowerhalf_s *lower);
static int  esp_lirc_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                            FAR unsigned int *txbuf, unsigned int n);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct lirc_ops_s g_esp_lirc_rx_ops =
{
  .driver_type = LIRC_DRIVER_IR_RAW,
  .open        = esp_lirc_open,
  .close       = esp_lirc_close,
};

static const struct lirc_ops_s g_esp_lirc_tx_ops =
{
  .driver_type = LIRC_DRIVER_IR_RAW_TX,
  .open        = esp_lirc_open,
  .close       = esp_lirc_close,
  .tx_ir       = esp_lirc_tx_ir,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lirc_rx_thread
 ****************************************************************************/

static int esp_lirc_rx_thread(int argc, FAR char *argv[])
{
  FAR struct esp_lirc_dev_s *priv;
  FAR const char *arg;
  FAR char *rxbuf;
  FAR rmt_symbol_word_t *sym;
  unsigned int sample;
  int nbytes;
  int nsyms;
  int i;

  DEBUGASSERT(argc >= 2 && argv[argc - 1] != NULL);

  arg = argv[argc - 1];
  priv = (FAR struct esp_lirc_dev_s *)(uintptr_t)strtoul(arg, NULL, 16);

  rxbuf = kmm_malloc(LIRC_RX_BUF_BYTES);
  if (rxbuf == NULL)
    {
      rcerr("ERROR: out of memory for RX buffer\n");
      return -ENOMEM;
    }

  while (priv->running)
    {
      if (priv->rmt->ops->read)
        {
          priv->rmt->ops->read(priv->rmt, rxbuf, LIRC_RX_BUF_BYTES);
        }

      nxsem_wait_uninterruptible(priv->rmt->recvsem);

      if (!priv->running)
        {
          break;
        }

      while ((nbytes = circbuf_read(priv->rmt->circbuf,
                                    rxbuf, LIRC_RX_BUF_BYTES)) > 0)
        {
          nsyms = nbytes / sizeof(rmt_symbol_word_t);
          sym   = (FAR rmt_symbol_word_t *)rxbuf;

          for (i = 0; i < nsyms; i++)
            {
              uint16_t dur0 = sym[i].duration0;
              uint8_t  lvl0 = sym[i].level0;
              uint16_t dur1 = sym[i].duration1;
              uint8_t  lvl1 = sym[i].level1;

              if (dur0 > 0)
                {
                  sample = lvl0 ?
                    LIRC_PULSE(RMT_TICK_TO_US(dur0)) :
                    LIRC_SPACE(RMT_TICK_TO_US(dur0));
                  lirc_sample_event(&priv->lower, sample);
                }

              if (dur1 > 0)
                {
                  sample = lvl1 ?
                    LIRC_PULSE(RMT_TICK_TO_US(dur1)) :
                    LIRC_SPACE(RMT_TICK_TO_US(dur1));
                  lirc_sample_event(&priv->lower, sample);
                }
            }
        }
    }

  kmm_free(rxbuf);
  return OK;
}

/****************************************************************************
 * Name: esp_lirc_open
 ****************************************************************************/

static int esp_lirc_open(FAR struct lirc_lowerhalf_s *lower)
{
  FAR struct esp_lirc_dev_s *priv =
    (FAR struct esp_lirc_dev_s *)lower;
  FAR char *argv[3];
  char addrstr[20];
  int ret = OK;

  if (priv->rxpid > 0)
    {
      return OK;
    }

  if (priv->rmt->recvsem != NULL)
    {
      priv->running = true;

      snprintf(addrstr, sizeof(addrstr), "%lx",
               (unsigned long)(uintptr_t)priv);
      argv[0] = "esp_lirc_rx";
      argv[1] = addrstr;
      argv[2] = NULL;

      ret = kthread_create("esp_lirc_rx", SCHED_PRIORITY_DEFAULT,
                           LIRC_RX_THREAD_STACK,
                           esp_lirc_rx_thread, argv);
      if (ret < 0)
        {
          priv->running = false;
          rcerr("ERROR: kthread_create failed: %d\n", ret);
          return ret;
        }

      priv->rxpid = ret;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_lirc_close
 ****************************************************************************/

static void esp_lirc_close(FAR struct lirc_lowerhalf_s *lower)
{
  FAR struct esp_lirc_dev_s *priv =
    (FAR struct esp_lirc_dev_s *)lower;

  if (priv->rxpid > 0)
    {
      priv->running = false;

      if (priv->rmt->recvsem != NULL)
        {
          nxsem_post(priv->rmt->recvsem);
        }

      priv->rxpid = -1;
    }
}

/****************************************************************************
 * Name: esp_lirc_tx_ir
 ****************************************************************************/

static int esp_lirc_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                          FAR unsigned int *txbuf, unsigned int n)
{
  FAR struct esp_lirc_dev_s *priv =
    (FAR struct esp_lirc_dev_s *)lower;
  FAR rmt_symbol_word_t *items;
  unsigned int nwords;
  unsigned int i;
  int ret;

  if (!priv->rmt->ops->write)
    {
      return -ENOSYS;
    }

  nwords = (n + 1) / 2;
  items  = kmm_zalloc(nwords * sizeof(rmt_symbol_word_t));
  if (items == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < n; i++)
    {
      uint32_t us  = LIRC_VALUE(txbuf[i]);
      uint8_t  lvl = LIRC_IS_PULSE(txbuf[i]) ? 1 : 0;
      uint16_t tks = (uint16_t)RMT_US_TO_TICK(us);

      if ((i & 1) == 0)
        {
          items[i / 2].duration0 = tks;
          items[i / 2].level0    = lvl;
        }
      else
        {
          items[i / 2].duration1 = tks;
          items[i / 2].level1    = lvl;
        }
    }

  ret = priv->rmt->ops->write(priv->rmt,
                              (FAR const char *)items,
                              nwords * sizeof(rmt_symbol_word_t));

  kmm_free(items);
  return (ret < 0) ? ret : (int)n;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lirc_rx_initialize
 *
 * Description:
 *   Register a pre-initialized RMT RX channel as a LIRC raw IR device.
 *
 * Input Parameters:
 *   devno - The LIRC device number (e.g. 0 for /dev/lirc0)
 *   rmt   - An already-initialized RMT RX lower-half handle
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_lirc_rx_initialize(int devno, FAR struct rmt_dev_s *rmt)
{
  FAR struct esp_lirc_dev_s *priv;
  int ret;

  DEBUGASSERT(rmt != NULL);

  priv = kmm_zalloc(sizeof(struct esp_lirc_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.ops           = &g_esp_lirc_rx_ops;
  priv->lower.buffer_bytes  = LIRC_RX_BUF_BYTES;
  priv->lower.rx_resolution = 100; /* 0.1 µs = 100 ns */
  priv->rmt                 = rmt;
  priv->rxpid               = -1;

  ret = lirc_register(&priv->lower, devno);
  if (ret < 0)
    {
      rcerr("ERROR: lirc_register failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_lirc_tx_initialize
 *
 * Description:
 *   Register a pre-initialized RMT TX channel as a LIRC raw TX device.
 *
 * Input Parameters:
 *   devno - The LIRC device number (e.g. 0 for /dev/lirc0)
 *   rmt   - An already-initialized RMT TX lower-half handle
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_lirc_tx_initialize(int devno, FAR struct rmt_dev_s *rmt)
{
  FAR struct esp_lirc_dev_s *priv;
  int ret;

  DEBUGASSERT(rmt != NULL);

  priv = kmm_zalloc(sizeof(struct esp_lirc_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.ops           = &g_esp_lirc_tx_ops;
  priv->lower.tx_resolution = 100; /* 0.1 µs = 100 ns */
  priv->rmt                 = rmt;
  priv->rxpid               = -1;

  ret = lirc_register(&priv->lower, devno);
  if (ret < 0)
    {
      rcerr("ERROR: lirc_register failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_ESP_RMT && CONFIG_DRIVERS_RC */
