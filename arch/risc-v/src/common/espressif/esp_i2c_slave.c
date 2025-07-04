/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_i2c_slave.c
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

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_slave.h>
#include <nuttx/kthread.h>
#include <arch/board/board.h>

#include "esp_i2c_slave.h"
#include "esp_irq.h"
#include "esp_gpio.h"
#include "riscv_internal.h"

#include "periph_ctrl.h"
#include "hal/i2c_hal.h"
#include "hal/i2c_types.h"
#include "hal/i2c_ll.h"
#include "soc/system_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2c_periph.h"
#if defined(CONFIG_ESPRESSIF_ESP32H2) || defined(CONFIG_ESPRESSIF_ESP32C6)
#  include "soc/pcr_reg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_FIFO_FULL_THRESH_VAL      28
#define I2C_FIFO_EMPTY_THRESH_VAL     5
#define I2C_SLAVE_TIMEOUT_DEFAULT     32000                /* I2C slave timeout value, APB clock cycle number */
#define I2C_SLAVE_SDA_SAMPLE_DEFAULT  10                   /* I2C slave sample time after scl positive edge default value */
#define I2C_SLAVE_SDA_HOLD_DEFAULT    10                   /* I2C slave hold time after scl negative edge default value */
#define I2C_SLAVE_BUFF_SIZE           1024
#ifdef CONFIG_I2C_POLLED
#define I2C_SLAVE_POLL_RATE           10
#endif

#if !SOC_RCC_IS_INDEPENDENT
#  define I2C_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define I2C_RCC_ATOMIC()
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Device hardware configuration */

struct esp_i2c_config_s
{
  uint8_t scl_pin;      /* GPIO configuration for SCL as SCL */
  uint8_t sda_pin;      /* GPIO configuration for SDA as SDA */

#ifndef CONFIG_I2C_POLLED
  uint8_t periph;      /* Peripheral ID */
  uint8_t irq;         /* Interrupt ID */
#endif

  uint32_t scl_insig;  /* I2C SCL input signal index */

  uint32_t sda_insig;  /* I2C SDA input signal index */
  uint32_t sda_outsig; /* I2C SDA output signal index */
};

/* I2C Device Private Data */

struct esp_i2c_priv_s
{
  const struct i2c_slaveops_s *ops; /* Standard I2C operations */
  uint32_t id;                      /* I2C instance */

  /* Port configuration */

  const struct esp_i2c_config_s *config;
  int refs;                         /* Reference count */
  mutex_t lock;                     /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  int cpuint;                       /* CPU interrupt assigned to this I2C */
#endif

#ifdef CONFIG_I2C_POLLED
  bool enabled;
#endif

  uint32_t error;                         /* I2C transform error */
  i2c_hal_context_t *ctx;                 /* Common layer struct */
  int addr;                               /* Slave device address */
  int nbits;                              /* Slave device address bit count */
  i2c_slave_callback_t *cb;               /* Callback function when interrupt happens */
  void *cb_arg;                           /* Argument of callback function */
  uint32_t tx_length;                     /* Location of next TX value */
  uint8_t tx_buffer[I2C_SLAVE_BUFF_SIZE]; /* I2C Slave TX queue buffer */
  uint32_t rx_length;                     /* Location of next RX value */
  uint8_t rx_buffer[I2C_SLAVE_BUFF_SIZE]; /* I2C Slave RX queue buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_i2c_slave_setownaddress(struct i2c_slave_s *dev, int addr,
                                       int nbits);
static int esp_i2c_slave_write(struct i2c_slave_s *dev,
                               const uint8_t *buffer, int buflen);
static int esp_i2c_slave_read(struct i2c_slave_s *dev,
                              uint8_t *buffer, int buflen);
static int esp_i2c_slave_registercallback(struct i2c_slave_s *dev,
                                          i2c_slave_callback_t *callback,
                                          void *arg);
static void esp_i2c_slave_intr_disable(struct esp_i2c_priv_s *priv);
static void esp_i2c_slave_init(struct esp_i2c_priv_s *priv);
static void esp_i2c_slave_deinit(struct esp_i2c_priv_s *priv);
#ifndef CONFIG_I2C_POLLED
static int esp_i2c_slave_irq(int cpuint, void *context, void *arg);
#endif
#ifdef CONFIG_I2C_POLLED
static int esp_i2c_slave_polling_waitdone(struct esp_i2c_priv_s *priv);
#endif /* CONFIG_I2C_POLLED */
static inline void esp_i2c_process(struct esp_i2c_priv_s *priv,
                                   uint32_t irq_status);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_slaveops_s esp_i2c_ops =
{
  .setownaddress    = esp_i2c_slave_setownaddress,
  .write            = esp_i2c_slave_write,
  .read             = esp_i2c_slave_read,
  .registercallback = esp_i2c_slave_registercallback,
};

#ifdef CONFIG_ESPRESSIF_I2C0_SLAVE_MODE

i2c_hal_context_t i2c0_ctx =
{
  0
};

/* I2C device structure */

static const struct esp_i2c_config_s esp_i2c0_config =
{
  .scl_pin    = CONFIG_ESPRESSIF_I2C0_SCLPIN,
  .sda_pin    = CONFIG_ESPRESSIF_I2C0_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ETS_I2C_EXT0_INTR_SOURCE,
  .irq        = ESP_IRQ_I2C_EXT0,
#endif
  .scl_insig  = I2CEXT0_SCL_IN_IDX,
  .sda_insig  = I2CEXT0_SDA_IN_IDX,
  .sda_outsig = I2CEXT0_SDA_OUT_IDX
};

static struct esp_i2c_priv_s esp_i2c0_priv =
{
  .ops        = &esp_i2c_ops,
  .id         = 0,
  .config     = &esp_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .cpuint     = -ENOMEM,
#endif

#ifdef CONFIG_I2C_POLLED
  .enabled    = false,
#endif

  .error      = 0,
  .ctx        = &i2c0_ctx,
  .addr       = 0,
  .nbits      = 0,
  .cb         = NULL,
  .cb_arg     = NULL,
  .tx_length  = 0,
  .tx_buffer  =
  {
    0
  },
  .rx_length  = 0,
  .rx_buffer  =
  {
    0
  },
};
#endif

#ifdef CONFIG_ESPRESSIF_I2C1_SLAVE_MODE

i2c_hal_context_t i2c1_ctx =
{
  0
};

/* I2C device structure */

static const struct esp_i2c_config_s esp_i2c1_config =
{
  .scl_pin    = CONFIG_ESPRESSIF_I2C1_SCLPIN,
  .sda_pin    = CONFIG_ESPRESSIF_I2C1_SDAPIN,
#ifndef CONFIG_I2C_POLLED
  .periph     = ETS_I2C_EXT1_INTR_SOURCE,
  .irq        = ESP_IRQ_I2C_EXT1,
#endif
  .scl_insig  = I2CEXT1_SCL_IN_IDX,
  .sda_insig  = I2CEXT1_SDA_IN_IDX,
  .sda_outsig = I2CEXT1_SDA_OUT_IDX
};

static struct esp_i2c_priv_s esp_i2c1_priv =
{
  .ops        = &esp_i2c_ops,
  .id         = 1,
  .config     = &esp_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .cpuint     = -ENOMEM,
#endif

#ifdef CONFIG_I2C_POLLED
  .enabled    = false,
#endif

  .error      = 0,
  .ctx        = &i2c1_ctx,
  .addr       = 0,
  .nbits      = 0,
  .cb         = NULL,
  .cb_arg     = NULL,
  .tx_length  = 0,
  .tx_buffer  =
  {
    0
  },
  .rx_length  = 0,
  .rx_buffer  =
  {
    0
  },
};
#endif /* CONFIG_ESPRESSIF_I2C1 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2c_slave_setownaddress
 *
 * Description:
 *   Set our own I2C address.
 *
 *   One may register a callback to be notified about reception. During the
 *   slave mode reception, the methods READ and WRITE must be used to
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - I2C slave device-specific state data
 *   address - Our own slave address
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int esp_i2c_slave_setownaddress(struct i2c_slave_s *dev, int addr,
                                       int nbits)
{
  struct esp_i2c_priv_s *priv;
  irqstate_t flags;
  bool extended_addr = false;

  DEBUGASSERT(dev);
  priv = (struct esp_i2c_priv_s *)dev;
  flags = enter_critical_section();

  if (priv->addr == addr && priv->nbits == nbits)
    {
      leave_critical_section(flags);
      return OK;
    }

  switch (nbits)
    {
      case 7:
        {
          priv->addr = addr & 0x7f;
          priv->nbits = 7;
        }
        break;

      case 10:
        {
          priv->addr = addr & 0x3ff;
          priv->nbits = 10;
          extended_addr = true;
        }
        break;

      default:
        {
          leave_critical_section(flags);
          return ERROR;
        }
        break;
    }

  i2c_ll_set_slave_addr(priv->ctx->dev, priv->addr, extended_addr);
  i2c_ll_update(priv->ctx->dev);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_i2c_slave_write
 *
 * Description:
 *   Send a block of data on I2C when a bus master wants to read data from
 *   this particular device.
 *
 * Input Parameters:
 *   dev    - I2C slave device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to the
 *            device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   Returns the number of items written to the I2C memory.
 *
 ****************************************************************************/

static int esp_i2c_slave_write(struct i2c_slave_s *dev,
                               const uint8_t *buffer, int buflen)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;
  irqstate_t flags = enter_critical_section();
  int cnt = buflen;
#ifdef CONFIG_I2C_POLLED
  int ret;
#endif

  /* Update the registered buffer and length */

  cnt = MIN(I2C_SLAVE_BUFF_SIZE - priv->tx_length, buflen);
  memcpy(priv->tx_buffer + priv->tx_length, buffer, cnt);
  priv->tx_length += cnt;

  if (priv->tx_length > 0)
    {
      i2c_ll_slave_enable_tx_it(priv->ctx->dev);
    }

#ifdef CONFIG_I2C_POLLED
  ret = esp_i2c_slave_polling_waitdone(priv);
  if (ret < 0)
    {
      cnt = ret;
    }
#endif

  leave_critical_section(flags);
  return cnt;
}

/****************************************************************************
 * Name: esp_i2c_slave_read
 *
 * Description:
 *   Receive a block of data from I2C when a bus master writes data addressed
 *   to this particular device.
 *
 * Input Parameters:
 *   dev    - I2C slave device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The maximum size of the buffer
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int esp_i2c_slave_read(struct i2c_slave_s *dev,
                              uint8_t *buffer, int buflen)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;
  irqstate_t flags = enter_critical_section();
  int read_len = ERROR;

  /* Update the registered buffer and length */

  i2c_ll_slave_enable_rx_it(priv->ctx->dev);

  read_len = MIN(priv->rx_length, buflen);
  memcpy(buffer, priv->rx_buffer, read_len);
  memmove(priv->rx_buffer, priv->rx_buffer + read_len, read_len);
  priv->rx_length -= read_len;

  leave_critical_section(flags);

  return read_len;
}

/****************************************************************************
 * Name: esp_i2c_slave_registercallback
 *
 * Description:
 *   Register a callback function that will be invoked when something is
 *   received on I2C.
 *
 * Input Parameters:
 *   dev      - I2C slave device-specific state data
 *   callback - The function to be called when something has been received.
 *   arg      - User provided argument to be used with the callback
 *
 * Returned Value:
 *   OK when successful, or a negated errno when there is an error.
 *
 ****************************************************************************/

static int esp_i2c_slave_registercallback(struct i2c_slave_s *dev,
                                          i2c_slave_callback_t *callback,
                                          void *arg)
{
  struct esp_i2c_priv_s *priv;
  irqstate_t flags;

  DEBUGASSERT(dev);
  DEBUGASSERT(callback);

  priv = (struct esp_i2c_priv_s *)dev;
  flags = enter_critical_section();

  /* Set callback and callback argument */

  priv->cb = callback;
  priv->cb_arg = arg;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_i2c_slave_intr_disable
 *
 * Description:
 *   Disable I2C interrupts.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_slave_intr_disable(struct esp_i2c_priv_s *priv)
{
  i2c_ll_disable_intr_mask(priv->ctx->dev, I2C_LL_INTR_MASK);
  i2c_ll_clear_intr_mask(priv->ctx->dev, I2C_LL_INTR_MASK);
}

/****************************************************************************
 * Name: esp_i2c_slave_init
 *
 * Description:
 *   Initialize I2C hardware.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_slave_init(struct esp_i2c_priv_s *priv)
{
  const struct esp_i2c_config_s *config = priv->config;

  /* Configure GPIO signals for I2C SCL and SDA pins */

  esp_gpiowrite(config->scl_pin, 1);
  esp_gpiowrite(config->sda_pin, 1);

  esp_configgpio(config->scl_pin, INPUT_PULLUP);
  esp_gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

  esp_configgpio(config->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  esp_gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
  esp_gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

  /* Enable I2C hardware */

  I2C_RCC_ATOMIC()
    {
      i2c_ll_enable_bus_clock(priv->id, true);
      i2c_ll_reset_register(priv->id);
    }

  i2c_hal_init(priv->ctx, priv->id);

  /* Disable I2C interrupts */

  esp_i2c_slave_intr_disable(priv);

  /* Initialize I2C Slave */

  i2c_hal_slave_init(priv->ctx);
  i2c_ll_slave_enable_auto_start(priv->ctx->dev, true);
  i2c_ll_set_source_clk(priv->ctx->dev, I2C_CLK_SRC_DEFAULT);
  i2c_ll_set_slave_addr(priv->ctx->dev, priv->addr, false);
  i2c_ll_set_rxfifo_full_thr(priv->ctx->dev, I2C_FIFO_FULL_THRESH_VAL);
  i2c_ll_set_txfifo_empty_thr(priv->ctx->dev, I2C_FIFO_EMPTY_THRESH_VAL);

  /* set timing for data */

  i2c_ll_set_sda_timing(priv->ctx->dev, I2C_SLAVE_SDA_SAMPLE_DEFAULT,
                        I2C_SLAVE_SDA_HOLD_DEFAULT);
  i2c_ll_set_tout(priv->ctx->dev, I2C_SLAVE_TIMEOUT_DEFAULT);
  i2c_ll_slave_enable_rx_it(priv->ctx->dev);

  i2c_ll_update(priv->ctx->dev);
}

/****************************************************************************
 * Name: esp_i2c_slave_deinit
 *
 * Description:
 *   Disable I2C hardware.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 ****************************************************************************/

static void esp_i2c_slave_deinit(struct esp_i2c_priv_s *priv)
{
  const struct esp_i2c_config_s *config = priv->config;

  i2c_hal_deinit(priv->ctx);
  I2C_RCC_ATOMIC()
    {
      i2c_ll_enable_bus_clock(priv->id, false);
    }
}

/****************************************************************************
 * Name: esp_i2c_irq
 *
 * Description:
 *   This is the common I2C interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Parameters:
 *   cpuint  - CPU interrupt index
 *   context - Context data from the ISR
 *   arg     - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/
#ifndef CONFIG_I2C_POLLED
static int esp_i2c_slave_irq(int cpuint, void *context, void *arg)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)arg;
  uint32_t irq_status = 0;

  i2c_ll_get_intr_mask(priv->ctx->dev, &irq_status);
  if (irq_status == 0)
    {
      return OK;
    }

  esp_i2c_process(priv , irq_status);
  i2c_ll_clear_intr_mask(priv->ctx->dev, irq_status);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_i2c_slave_polling_waitdone
 *
 * Description:
 *   Wait for a transfer to complete by polling status interrupt registers,
 *   which indicates the status of the I2C operations. This function is only
 *   used in polling driven mode.
 *
 * Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Values:
 *   Zero (OK) is returned on successful transfer. -ETIMEDOUT is returned
 *   in case a transfer didn't finish within the timeout interval.
 *
 ****************************************************************************/
#ifdef CONFIG_I2C_POLLED
static int esp_i2c_slave_polling_waitdone(struct esp_i2c_priv_s *priv)
{
  int ret;
  clock_t current;
  clock_t timeout;
  uint32_t status = 0;
  i2c_intr_event_t event = 0;

  current = clock_systime_ticks();
  timeout = current + SEC2TICK(I2C_SLAVE_POLL_RATE);

  while ((sclock_t)(current - timeout) < 0)
    {
      /* Check if any interrupt triggered, clear them
       * process the operation.
       */

      i2c_ll_get_intr_mask(priv->ctx->dev, &status);
      if (status != 0)
        {
          i2c_ll_slave_get_event(priv->ctx->dev, &event);

          esp_i2c_process(priv, status);
          i2c_ll_clear_intr_mask(priv->ctx->dev, status);
        }

      /* Update current time */

      current = clock_systime_ticks();
    }

  if (current >= timeout)
    {
      ret = -ETIMEDOUT;
    }
  else
    {
      ret = OK;
    }

  /* Disable all interrupts */

  esp_i2c_slave_intr_disable(priv);

  return ret;
}

/****************************************************************************
 * Name: esp_i2c_slave_thread
 *
 * Description:
 *   Thread for i2c slave driver in polling mode.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 * Returned Values:
 *   Zero (OK)
 *
 ****************************************************************************/

static int esp_i2c_slave_thread(int argc, char **argv)
{
  struct esp_i2c_priv_s *priv =
      (struct esp_i2c_priv_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int ret;

  nxsig_usleep(1000);
  while (true)
    {
      esp_i2c_slave_polling_waitdone(priv);

      /* Sleeping thread before checking i2c peripheral */

      nxsig_usleep(100);
    }

  return OK;
}

#endif

/****************************************************************************
 * Name: esp_i2c_process
 *
 * Description:
 *   This routine manages the transfer. It's called after some specific
 *   commands from the I2C controller are executed or in case of errors.
 *   It's responsible for writing/reading operations and transferring data
 *   from/to FIFO.
 *   It's called in the interrupt and polled driven mode.
 *
 * Parameters:
 *   priv   - Pointer to the internal driver state structure.
 *   status - The current interrupt status register.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void esp_i2c_process(struct esp_i2c_priv_s *priv,
                                   uint32_t irq_status)
{
  i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
  uint32_t rx_fifo_cnt;
  uint32_t tx_fifo_rem;

  i2c_ll_slave_get_event(priv->ctx->dev, &evt_type);
  if (evt_type == I2C_INTR_EVENT_TRANS_DONE ||
      evt_type == I2C_INTR_EVENT_RXFIFO_FULL)
    {
      i2c_ll_get_rxfifo_cnt(priv->ctx->dev, &rx_fifo_cnt);
      rx_fifo_cnt = MIN(I2C_SLAVE_BUFF_SIZE - priv->rx_length, rx_fifo_cnt);
      i2c_ll_read_rxfifo(priv->ctx->dev, priv->rx_buffer + priv->rx_length,
                         rx_fifo_cnt);
      priv->rx_length += rx_fifo_cnt;

      /* Code closed for temporary time due to upper layer function issues */

#if 0
      if (priv->cb != NULL)
        {
          priv->cb(priv->cb_arg, I2CS_RX_COMPLETE, rx_fifo_cnt);
        }
#endif
    }
  else if (evt_type == I2C_INTR_EVENT_TXFIFO_EMPTY)
    {
      i2c_ll_get_txfifo_len(priv->ctx->dev, &tx_fifo_rem);
      tx_fifo_rem = MIN(priv->tx_length, tx_fifo_rem);
      if (tx_fifo_rem != 0)
        {
          i2c_ll_write_txfifo(priv->ctx->dev, priv->tx_buffer, tx_fifo_rem);
          memmove(priv->tx_buffer, priv->tx_buffer + tx_fifo_rem,
                  tx_fifo_rem);
        }
      else
        {
          i2c_ll_slave_disable_tx_it(priv->ctx->dev);
        }

      priv->tx_length -= tx_fifo_rem;

      /* Code closed for temporary time due to upper layer function issues */

#if 0
      if (priv->cb != NULL)
        {
          priv->cb(priv->cb_arg, I2CS_TX_COMPLETE, tx_fifo_rem);
        }
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2cbus_slave_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a pointer to an unique
 *   instance of struct i2c_slave_s. This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency.
 *
 * Parameters:
 *   port - Port number of the I2C interface to be initialized.
 *   addr - Address of the slave device
 *
 * Returned Value:
 *   Pointer to valid I2C device structure is returned on success.
 *   A NULL pointer is returned on failure.
 *
 ****************************************************************************/

struct i2c_slave_s *esp_i2cbus_slave_initialize(int port, int addr)
{
  struct esp_i2c_priv_s *priv;
  int ret;
#ifndef CONFIG_I2C_POLLED
  const struct esp_i2c_config_s *config;
#else
  char *argv[2];
  char arg1[32];
#endif

  switch (port)
    {
#ifdef CONFIG_ESPRESSIF_I2C0_SLAVE_MODE
    case ESPRESSIF_I2C0_SLAVE:
      priv = &esp_i2c0_priv;
      break;
#endif
#ifdef CONFIG_ESPRESSIF_I2C1_SLAVE_MODE
    case ESPRESSIF_I2C1_SLAVE:
      priv = &esp_i2c1_priv;
      break;
#endif
    default:
      return NULL;
    }

  nxmutex_lock(&priv->lock);

  if (priv->refs++ != 0)
    {
      nxmutex_unlock(&priv->lock);

      i2cinfo("Returning previously initialized I2C bus. "
              "Handler: %p\n", priv);

      return (struct i2c_slave_s *)priv;
    }

#ifndef CONFIG_I2C_POLLED
  config = priv->config;
  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the previous IRQ */

      up_disable_irq(config->irq);
      esp_teardown_irq(config->periph, priv->cpuint);
    }

  priv->cpuint = esp_setup_irq(config->periph,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  ret = irq_attach(config->irq, esp_i2c_slave_irq, priv);
  if (ret != OK)
    {
      /* Failed to attach IRQ, free the allocated CPU interrupt */

      esp_teardown_irq(config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      priv->refs--;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the I2C device. */

  up_enable_irq(config->irq);
#else
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("esp_i2c_slave_thread",
                       SCHED_PRIORITY_DEFAULT,
                       128,
                       esp_i2c_slave_thread,
                       argv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

#endif

  priv->addr = addr;
  esp_i2c_slave_init(priv);
  nxmutex_unlock(&priv->lock);

#ifdef CONFIG_I2C_POLLED
  priv->enabled = true;
#endif

  i2cinfo("I2C bus initialized! Handler: %p\n", priv);

  return (struct i2c_slave_s *)priv;
}

/****************************************************************************
 * Name: esp_i2cbus_slave_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device.
 *
 * Parameters:
 *   dev - Device structure as returned by
 *         esp_i2cbus_slave_uninitialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int esp_i2cbus_slave_uninitialize(struct i2c_slave_s *dev)
{
  struct esp_i2c_priv_s *priv = (struct esp_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  esp_teardown_irq(priv->config->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;
#endif

#ifdef CONFIG_I2C_POLLED
  priv->enabled = false;
#endif

  esp_i2c_slave_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE */
