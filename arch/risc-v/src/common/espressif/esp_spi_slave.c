/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spi_slave.c
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

#if defined(CONFIG_ESPRESSIF_SPI) && defined(CONFIG_SPI_SLAVE)

#include <assert.h>
#include <debug.h>
#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#include <arch/board/board.h>

#include "esp_spi.h"
#include "esp_irq.h"
#include "esp_gpio.h"
#include "hal/spi_ll.h"
#include "hal/spi_hal.h"
#include "hal/spi_slave_hal.h"
#include "periph_ctrl.h"

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
#include "esp_dma.h"
#endif

#include "riscv_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define SPI_SLAVE_BUFSIZE (CONFIG_ESPRESSIF_SPI2_SLAVE_BUFSIZE)

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
/* SPI DMA RX/TX number of descriptors */

#if (SPI_SLAVE_BUFSIZE % ESPRESSIF_DMA_DATALEN_MAX) > 0
#  define SPI_DMA_DESC_NUM (SPI_SLAVE_BUFSIZE / ESPRESSIF_DMA_DATALEN_MAX + 1)
#else
#  define SPI_DMA_DESC_NUM (SPI_SLAVE_BUFSIZE / ESPRESSIF_DMA_DATALEN_MAX)
#endif

#endif /* CONFIG_ESPRESSIF_SPI2_DMA */

#if defined(CONFIG_ARCH_CHIP_ESP32C6) || defined(CONFIG_ARCH_CHIP_ESP32H2)
#  define SPI2_INTR_SOURCE GSPI2_INTR_SOURCE
#  define ESP_IRQ_SPI2     ESP_IRQ_GSPI2
#endif

/* Verify whether SPI has been assigned IOMUX pins.
 * Otherwise, SPI signals will be routed via GPIO Matrix.
 */

#define SPI_IS_CS_IOMUX   (CONFIG_ESPRESSIF_SPI2_CSPIN == SPI2_IOMUX_CSPIN)
#define SPI_IS_CLK_IOMUX  (CONFIG_ESPRESSIF_SPI2_CLKPIN == SPI2_IOMUX_CLKPIN)
#define SPI_IS_MOSI_IOMUX (CONFIG_ESPRESSIF_SPI2_MOSIPIN == SPI2_IOMUX_MOSIPIN)
#define SPI_IS_MISO_IOMUX (CONFIG_ESPRESSIF_SPI2_MISOPIN == SPI2_IOMUX_MISOPIN)

#define SPI_VIA_IOMUX     (SPI_IS_CS_IOMUX) && (SPI_IS_CLK_IOMUX) && \
                          (SPI_IS_MOSI_IOMUX) && (SPI_IS_MISO_IOMUX)

/* SPI Slave default width */

#define SPI_SLAVE_DEFAULT_WIDTH (8)

/* SPI Slave default mode */

#define SPI_SLAVE_DEFAULT_MODE  (SPISLAVE_MODE0)

#define WORDS2BYTES(_priv, _wn)   ((_wn) * ((_priv)->nbits / 8))
#define BYTES2WORDS(_priv, _bn)   ((_bn) / ((_priv)->nbits / 8))

/* SPI Slave controller hardware configuration */

struct spislave_config_s
{
  int32_t width;              /* SPI Slave default width */
  enum spi_slave_mode_e mode; /* SPI Slave default mode */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */
  uint8_t periph;             /* Peripheral ID */
  uint8_t irq;                /* Interrupt ID */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* SPI reset bit */
  uint32_t cs_insig;          /* SPI CS input signal index */
  uint32_t cs_outsig;         /* SPI CS output signal index */
  uint32_t mosi_insig;        /* SPI MOSI input signal index */
  uint32_t mosi_outsig;       /* SPI MOSI output signal index */
  uint32_t miso_insig;        /* SPI MISO input signal index */
  uint32_t miso_outsig;       /* SPI MISO output signal index */
  uint32_t clk_insig;         /* SPI CLK input signal index */
  uint32_t clk_outsig;        /* SPI CLK output signal index */
};

struct spislave_priv_s
{
  /* Externally visible part of the SPI Slave controller interface */

  struct spi_slave_ctrlr_s ctrlr;

  /* Reference to SPI Slave device interface */

  struct spi_slave_dev_s *dev;

  /* Port configuration */

  const struct spislave_config_s *config;
  int refs;                     /* Reference count */
  int cpuint;                   /* SPI interrupt ID */
  enum spi_slave_mode_e mode;   /* Current SPI Slave hardware mode */
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  int32_t dma_channel;        /* Channel assigned by the GDMA driver */
#endif
  uint8_t nbits;                /* Current configured bit width */
  uint32_t tx_length;           /* Location of next TX value */

  /* SPI Slave TX queue buffer */

  uint8_t tx_buffer[SPI_SLAVE_BUFSIZE];
  uint32_t rx_length;           /* Location of next RX value */

  /* SPI Slave RX queue buffer */

  uint8_t rx_buffer[SPI_SLAVE_BUFSIZE];

  /* Flag that indicates whether SPI Slave is currently processing */

  bool is_processing;

  /* Flag that indicates whether SPI Slave TX is currently enabled */

  bool is_tx_enabled;
  int module;                   /* Peripheral module */
  spi_slave_hal_context_t ctx;  /* Context struct of the common layer */
  spi_slave_hal_config_t cfg;   /* Configuration struct of the common layer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Slave controller buffer operations */

#ifndef CONFIG_ESPRESSIF_SPI2_DMA
static inline void spislave_cpu_tx_fifo_reset(spi_dev_t *hw);
#else
static inline void spislave_dma_tx_fifo_reset(spi_dev_t *hw);
static inline void spislave_dma_rx_fifo_reset(spi_dev_t *hw);
#endif

/* SPI Slave controller interrupt handlers */

static int spislave_cs_interrupt(int irq, void *context, void *arg);
static int spislave_periph_interrupt(int irq, void *context, void *arg);

/* SPI Slave controller internal functions */

static void spislave_evict_sent_data(struct spislave_priv_s *priv,
                                     uint32_t sent_bytes);
static inline void spislave_hal_store_result(spi_slave_hal_context_t *hal);
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void spislave_setup_rx_dma(struct spislave_priv_s *priv);
static void spislave_setup_tx_dma(struct spislave_priv_s *priv);
static void spislave_prepare_next_tx(struct spislave_priv_s *priv);
static void spislave_dma_init(struct spislave_priv_s *priv);
#endif
static void spislave_initialize(struct spi_slave_ctrlr_s *ctrlr);

/* SPI Slave controller operations */

static void spislave_bind(struct spi_slave_ctrlr_s *ctrlr,
                          struct spi_slave_dev_s *dev,
                          enum spi_slave_mode_e mode,
                          int nbits);
static void spislave_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int spislave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            const void *data,
                            size_t nwords);
static bool spislave_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void spislave_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t spislave_qpoll(struct spi_slave_ctrlr_s *ctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2
static const struct spislave_config_s esp_spi2slave_config =
{
  .width        = SPI_SLAVE_DEFAULT_WIDTH,
  .mode         = SPI_SLAVE_DEFAULT_MODE,
  .cs_pin       = CONFIG_ESPRESSIF_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESPRESSIF_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESPRESSIF_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESPRESSIF_SPI2_CLKPIN,
  .periph       = SPI2_INTR_SOURCE,
  .irq          = ESP_IRQ_SPI2,
  .cs_insig     = FSPICS0_IN_IDX,
  .cs_outsig    = FSPICS0_OUT_IDX,
  .mosi_insig   = FSPID_IN_IDX,
  .mosi_outsig  = FSPID_OUT_IDX,
  .miso_insig   = FSPIQ_IN_IDX,
  .miso_outsig  = FSPIQ_OUT_IDX,
  .clk_insig    = FSPICLK_IN_IDX,
  .clk_outsig   = FSPICLK_OUT_IDX
};

static const struct spi_slave_ctrlrops_s esp_spi2slave_ops =
{
  .bind     = spislave_bind,
  .unbind   = spislave_unbind,
  .enqueue  = spislave_enqueue,
  .qfull    = spislave_qfull,
  .qflush   = spislave_qflush,
  .qpoll    = spislave_qpoll
};

static struct spislave_priv_s esp_spi2slave_priv =
{
  .ctrlr         =
                  {
                    .ops = &esp_spi2slave_ops
                  },
  .dev           = NULL,
  .config        = &esp_spi2slave_config,
  .refs          = 0,
  .cpuint        = -ENOMEM,
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  .dma_channel   = -ENOMEM,
#endif
  .mode          = SPISLAVE_MODE0,
  .nbits         = 0,
  .tx_length     = 0,
  .tx_buffer     =
                  {
                    0
                  },
  .rx_length     = 0,
  .rx_buffer     =
                  {
                    0
                  },
  .is_processing = false,
  .is_tx_enabled = false,
  .module = PERIPH_SPI2_MODULE,
  .ctx =
          {
            0
          },
  .cfg =
          {
            .host_id = SPI2_HOST,
          }
};
#endif /* CONFIG_ESPRESSIF_SPI2 */

#ifdef CONFIG_ESPRESSIF_SPI2_DMA

/* SPI DMA RX/TX description */

static struct esp_dmadesc_s dma_rxdesc[SPI_DMA_DESC_NUM];
static struct esp_dmadesc_s dma_txdesc[SPI_DMA_DESC_NUM];

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spislave_hal_store_result
 *
 * Description:
 *   Get data from SPI peripheral driver and update local buffer. A similar
 *   function is present in the ESP HAL, but it seems there is an issue in
 *   the spi_ll_read_buffer(hal->hw, hal->rx_buffer, hal->rcv_bitlen) call.
 *   Therefore, we are developing our own function to handle this issue.
 *
 * NOTE: We have a similar function in the ESP HAL, but the
 *   spi_ll_read_buffer seems to be receiving the wrong parameter.
 *   This function will address the issue until the Espressif HAL is fixed.
 *
 * Input Parameters:
 *   hw - Beginning address of the HAL context register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void spislave_hal_store_result(spi_slave_hal_context_t *hal)
{
  hal->rcv_bitlen = spi_ll_slave_get_rcv_bitlen(hal->hw);
  if (hal->rcv_bitlen == hal->bitlen - 1)
    {
      hal->rcv_bitlen++;
    }

  if (!hal->use_dma && hal->rx_buffer)
    {
      spi_ll_read_buffer(hal->hw, hal->rx_buffer, hal->rcv_bitlen);
    }
}

/****************************************************************************
 * Name: spislave_cpu_tx_fifo_reset
 *
 * Description:
 *   Reset the BUF TX AFIFO, which is used to send data out in SPI Slave
 *   CPU-controlled mode transfer.
 *
 * Input Parameters:
 *   hw - Beginning address of the peripheral register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_ESPRESSIF_SPI2_DMA
static inline void spislave_cpu_tx_fifo_reset(spi_dev_t *hw)
{
  spi_ll_cpu_tx_fifo_reset(hw);
}
#endif

/****************************************************************************
 * Name: spislave_dma_tx_fifo_reset
 *
 * Description:
 *   Reset the DMA TX AFIFO, which is used to send data out in SPI Slave
 *   DMA-controlled mode transfer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static inline void spislave_dma_tx_fifo_reset(spi_dev_t *hw)
{
  spi_ll_dma_tx_fifo_reset(hw);
}
#endif

/****************************************************************************
 * Name: spislave_dma_rx_fifo_reset
 *
 * Description:
 *   Reset the RX AFIFO, which is used to receive data in SPI Slave mode
 *   transfer.
 *
 * Input Parameters:
 *   hw - Beginning address of the peripheral register
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static inline void spislave_dma_rx_fifo_reset(spi_dev_t *hw)
{
  spi_ll_dma_rx_fifo_reset(hw);
}
#endif

/****************************************************************************
 * Name: spislave_cs_interrupt
 *
 * Description:
 *   Handler for the GPIO interrupt which is triggered when the chip select
 *   has toggled to inactive state (active high).
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - SPI Slave controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int spislave_cs_interrupt(int irq, void *context, void *arg)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)arg;

  if (priv->is_processing)
    {
      priv->is_processing = false;
      SPIS_DEV_SELECT(priv->dev, false);
    }

  return 0;
}

/****************************************************************************
 * Name: spislave_evict_sent_data
 *
 * Description:
 *   Evict from the TX buffer data sent on the latest transaction and update
 *   the length. This is a post transaction operation.
 *
 * Input Parameters:
 *   priv       - Private SPI Slave controller structure
 *   sent_bytes - Number of transmitted bytes
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_evict_sent_data(struct spislave_priv_s *priv,
                                     uint32_t sent_bytes)
{
  if (sent_bytes < priv->tx_length)
    {
      priv->tx_length -= sent_bytes;

      memmove(priv->tx_buffer, priv->tx_buffer + sent_bytes,
              priv->tx_length);

      memset(priv->tx_buffer + priv->tx_length, 0, sent_bytes);
    }
  else
    {
      priv->tx_length = 0;
    }
}

/****************************************************************************
 * Name: spislave_setup_rx_dma
 *
 * Description:
 *   Configure the SPI Slave peripheral to perform the next RX data transfer
 *   via DMA.
 *
 * Input Parameters:
 *   priv - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void spislave_setup_rx_dma(struct spislave_priv_s *priv)
{
  uint32_t length = SPI_SLAVE_BUFSIZE - priv->rx_length;

  esp_dma_setup(priv->dma_channel, false, dma_rxdesc, SPI_DMA_DESC_NUM,
                priv->rx_buffer + priv->rx_length, length);

  spi_ll_slave_reset(priv->ctx.hw);

  /* Clear input FIFO full error */

  spi_ll_infifo_full_clr(priv->ctx.hw);

  /* Enable SPI DMA RX */

  spi_ll_dma_rx_enable(priv->ctx.hw, true);

  esp_dma_enable(priv->dma_channel, false);
}
#endif

/****************************************************************************
 * Name: spislave_setup_tx_dma
 *
 * Description:
 *   Configure the SPI Slave peripheral to perform the next TX data transfer
 *   via DMA.
 *
 * Input Parameters:
 *   priv   - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void spislave_setup_tx_dma(struct spislave_priv_s *priv)
{
  esp_dma_setup(priv->dma_channel, true, dma_txdesc, SPI_DMA_DESC_NUM,
                priv->tx_buffer, SPI_SLAVE_BUFSIZE);

  spislave_dma_tx_fifo_reset(priv->ctx.hw);

  spi_ll_slave_reset(priv->ctx.hw);

  /* Clear output FIFO full error */

  spi_ll_outfifo_empty_clr(priv->ctx.hw);

  /* Enable SPI DMA TX */

  spi_ll_dma_tx_enable(priv->ctx.hw, true);

  esp_dma_enable(priv->dma_channel, true);
}
#endif

/****************************************************************************
 * Name: spi_slave_prepare_data
 *
 * Description:
 *   Prepare the SPI Slave controller for transmitting data in CPU-controlled
 *   mode. This function resets the SPI Slave hardware, writes the data to
 *   the TX buffer, and resets the TX FIFO.
 *
 * Input Parameters:
 *   priv           - Private SPI Slave controller structure
 *   nbits_to_send  - Number of bits to send in the next transaction
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void spi_slave_prepare_data(struct spislave_priv_s *priv,
                      ssize_t nbits_to_send)
{
  spi_ll_slave_reset(priv->ctx.hw);
  spi_ll_write_buffer(priv->ctx.hw, priv->tx_buffer, nbits_to_send);
  spislave_cpu_tx_fifo_reset(priv->ctx.hw);
}

/****************************************************************************
 * Name: spislave_prepare_next_tx
 *
 * Description:
 *   Prepare the SPI Slave controller for transmitting data on the next
 *   transaction.
 *
 * Input Parameters:
 *   priv - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_prepare_next_tx(struct spislave_priv_s *priv)
{
  uint32_t nbits_to_send;

  if (priv->tx_length != 0)
    {
#ifdef CONFIG_ESPRESSIF_SPI2_DMA
      spislave_setup_tx_dma(priv);
#else
      nbits_to_send = priv->nbits * priv->tx_length;
      spi_slave_prepare_data(priv, nbits_to_send);
#endif
      priv->is_tx_enabled = true;
    }
  else
    {
      spiwarn("TX buffer empty! Disabling TX for next transaction\n");

#ifndef CONFIG_ESPRESSIF_SPI2_DMA
      memset(priv->tx_buffer, 0, sizeof(priv->tx_buffer));
      spi_slave_prepare_data(priv, priv->ctx.rcv_bitlen);
#endif

      priv->is_tx_enabled = false;
    }

    spi_ll_slave_set_rx_bitlen(priv->ctx.hw, priv->ctx.rcv_bitlen);
    spi_ll_slave_set_tx_bitlen(priv->ctx.hw, priv->ctx.rcv_bitlen);
}

/****************************************************************************
 * Name: spislave_periph_interrupt
 *
 * Description:
 *   Handler for the SPI Slave controller interrupt which is triggered when a
 *   transfer is finished.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - SPI Slave controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int spislave_periph_interrupt(int irq, void *context, void *arg)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)arg;
  if (!priv->is_processing)
    {
      SPIS_DEV_SELECT(priv->dev, true);
      priv->is_processing = true;
    }

  /* RX process */

  /* Point to the next free position in the buffer */

  priv->ctx.rx_buffer += priv->rx_length;
  spislave_hal_store_result(&priv->ctx);
  priv->rx_length += priv->ctx.rcv_bitlen / priv->nbits;
  SPIS_DEV_NOTIFY(priv->dev, SPISLAVE_RX_COMPLETE);

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  if (priv->rx_length < SPI_SLAVE_BUFSIZE)
    {
      spislave_setup_rx_dma(priv);
    }
#endif

  /* TX process */

  if (priv->ctx.rcv_bitlen > 0 && priv->is_tx_enabled)
    {
      spislave_evict_sent_data(priv, priv->ctx.rcv_bitlen / priv->nbits);
      SPIS_DEV_NOTIFY(priv->dev, SPISLAVE_TX_COMPLETE);
    }

  priv->ctx.bitlen = priv->tx_length;
  priv->ctx.rx_buffer = priv->rx_buffer;
  priv->ctx.tx_buffer = priv->tx_buffer;
  spislave_prepare_next_tx(priv);

  if (priv->is_processing && esp_gpioread(priv->config->cs_pin))
    {
      priv->is_processing = false;
      SPIS_DEV_SELECT(priv->dev, false);
    }

  /* Clear the trans_done interrupt flag */

  /* Trigger the start of user-defined transaction */

  spi_slave_hal_user_start(&priv->ctx);

  return 0;
}

/****************************************************************************
 * Name: spislave_dma_init
 *
 * Description:
 *   Initialize SPI Slave connection to GDMA engine.
 *
 * Input Parameters:
 *   priv - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
static void spislave_dma_init(struct spislave_priv_s *priv)
{
  /* Initialize GDMA controller */

  esp_dma_init();

  /* Request a GDMA channel for SPI peripheral */

  priv->dma_channel = esp_dma_request(ESPRESSIF_DMA_PERIPH_M2M, 1, 1,
                                      true);
  if (priv->dma_channel < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      DEBUGPANIC();
    }
}
#endif

/****************************************************************************
 * Name: spislave_initialize
 *
 * Description:
 *   Initialize SPI Slave hardware interface.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_initialize(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  const struct spislave_config_s *config = priv->config;

  spiinfo("ctrlr=%p\n", ctrlr);

  esp_gpiowrite(config->cs_pin, 1);
  esp_gpiowrite(config->mosi_pin, 1);
  esp_gpiowrite(config->miso_pin, 1);
  esp_gpiowrite(config->clk_pin, 1);

#if SPI_VIA_IOMUX
  esp_configgpio(config->cs_pin, INPUT_FUNCTION_3 | PULLUP);
  esp_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp_configgpio(config->mosi_pin, INPUT_FUNCTION_3 | PULLUP);
  esp_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp_configgpio(config->miso_pin, OUTPUT_FUNCTION_3);
  esp_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp_configgpio(config->clk_pin, INPUT_FUNCTION_3 | PULLUP);
  esp_gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);
#else
  esp_configgpio(config->cs_pin, INPUT_FUNCTION_2 | PULLUP);
  esp_gpio_matrix_in(config->cs_pin, config->cs_insig, 0);

  esp_configgpio(config->mosi_pin, INPUT_FUNCTION_2 | PULLUP);
  esp_gpio_matrix_in(config->mosi_pin, config->mosi_insig, 0);

  esp_configgpio(config->miso_pin, OUTPUT_FUNCTION_2);
  esp_gpio_matrix_out(config->miso_pin, config->miso_outsig, 0, 0);

  esp_configgpio(config->clk_pin, INPUT_FUNCTION_2 | PULLUP);
  esp_gpio_matrix_in(config->clk_pin, config->clk_insig, 0);
#endif

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  spislave_dma_init(priv);
#endif

  esp_gpioirqenable(ESP_PIN2IRQ(config->cs_pin), RISING);

  priv->ctx.rx_lsbfirst = 0;
  priv->ctx.tx_lsbfirst = 0;

#ifdef CONFIG_ESPRESSIF_SPI2_DMA
  priv->ctx.dmadesc_n = priv->dma_channel;
  priv->ctx.use_dma = 1;

  priv->ctx.dmadesc_rx = (lldesc_t *)dma_rxdesc;
  priv->ctx.dmadesc_tx = (lldesc_t *)dma_txdesc;
#else
  priv->ctx.dmadesc_n = 0;
  priv->ctx.use_dma = 0;
#endif
}

/****************************************************************************
 * Name: spislave_bind
 *
 * Description:
 *   Bind the SPI Slave device interface to the SPI Slave controller
 *   interface and configure the SPI interface. Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   dev   - SPI Slave device interface instance
 *   mode  - The SPI mode requested
 *   nbits - The number of bits requests.
 *            If value is greater than 0, then it implies MSB first
 *            If value is less than 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This implementation currently supports only positive "nbits" values,
 *   i.e., it always configures the SPI Slave controller driver as MSB first.
 *
 ****************************************************************************/

static void spislave_bind(struct spi_slave_ctrlr_s *ctrlr,
                          struct spi_slave_dev_s *dev,
                          enum spi_slave_mode_e mode,
                          int nbits)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  const void *data = NULL;
  irqstate_t flags;
  size_t num_words;

  spiinfo("ctrlr=%p dev=%p mode=%d nbits=%d\n", ctrlr, dev, mode, nbits);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev == NULL);
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(nbits > 0);

  flags = enter_critical_section();

  priv->dev = dev;

  SPIS_DEV_SELECT(dev, false);

  SPIS_DEV_CMDDATA(dev, false);

  priv->rx_length = 0;
  priv->tx_length = 0;
  priv->is_tx_enabled = false;
  priv->nbits = nbits;
  priv->ctx.mode = mode;

  /* Configure SPI Slave peripheral */

  spislave_initialize(ctrlr);
  periph_module_enable(priv->module);
  spi_slave_hal_init(&priv->ctx, &priv->cfg);
  spi_slave_hal_setup_device(&priv->ctx);

  num_words = SPIS_DEV_GETDATA(dev, &data);

  if (data != NULL && num_words > 0)
    {
      size_t num_bytes = WORDS2BYTES(priv, num_words);
      memcpy(priv->tx_buffer, data, num_bytes);
      priv->tx_length += num_bytes;
    }

  /* Enable the CPU interrupt that is linked to the SPI Slave controller */

  up_enable_irq(priv->config->irq);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_unbind
 *
 * Description:
 *   Un-bind the SPI Slave device interface from the SPI Slave controller
 *   interface. Reset the SPI interface and restore the SPI Slave
 *   controller driver to its initial state.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("Unbinding %p\n", priv->dev);

  flags = enter_critical_section();

  up_disable_irq(priv->config->irq);

  esp_gpioirqdisable(ESP_PIN2IRQ(priv->config->cs_pin));

  /* Disable the trans_done interrupt */

  spi_ll_disable_intr(priv->ctx.hw, SPI_LL_INTR_TRANS_DONE);

  periph_module_disable(priv->module);

  priv->dev = NULL;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface. This adds
 *   the word to the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   data  - Pointer to the command/data mode data to be shifted out.
 *           The data width must be aligned to the nbits parameter which was
 *           previously provided to the bind() method.
 *   len   - Number of units of "nbits" wide to enqueue,
 *           "nbits" being the data width previously provided to the bind()
 *           method.
 *
 * Returned Value:
 *   Number of data items successfully queued, or a negated errno:
 *         - "len" if all the data was successfully queued
 *         - "0..len-1" if queue is full
 *         - "-errno" in any other error
 *
 ****************************************************************************/

static int spislave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            const void *data,
                            size_t len)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  size_t num_bytes = WORDS2BYTES(priv, len);
  size_t bufsize;
  irqstate_t flags;
  int enqueued_words;

  spiinfo("ctrlr=%p, data=%p, num_bytes=%zu\n", ctrlr, data, num_bytes);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  flags = enter_critical_section();

  bufsize = SPI_SLAVE_BUFSIZE - priv->tx_length;
  if (bufsize == 0)
    {
      leave_critical_section(flags);
      return -ENOSPC;
    }

  num_bytes = MIN(num_bytes, bufsize);
  memcpy(priv->tx_buffer + priv->tx_length, data, num_bytes);
  priv->tx_length += num_bytes;

  enqueued_words = BYTES2WORDS(priv, num_bytes);

  if (!priv->is_processing)
    {
      spislave_prepare_next_tx(priv);
    }

  leave_critical_section(flags);

  return enqueued_words;
}

/****************************************************************************
 * Name: spislave_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

static bool spislave_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;
  bool is_full = false;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();
  is_full = priv->tx_length == SPI_SLAVE_BUFSIZE;
  leave_critical_section(flags);

  return is_full;
}

/****************************************************************************
 * Name: spislave_qflush
 *
 * Description:
 *   Discard all saved values in the output queue. On return from this
 *   function the output queue will be empty. Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();
  priv->tx_length = 0;
  priv->is_tx_enabled = false;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_qpoll
 *
 * Description:
 *   Tell the controller to output all the receive queue data.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Number of units of width "nbits" left in the RX queue. If the device
 *   accepted all the data, the return value will be 0.
 *
 ****************************************************************************/

static size_t spislave_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;
  uint32_t tmp;
  uint32_t recv_n;
  size_t remaining_words;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();

  tmp = SPIS_DEV_RECEIVE(priv->dev, priv->rx_buffer,
                         BYTES2WORDS(priv, priv->rx_length));
  recv_n = WORDS2BYTES(priv, tmp);
  if (recv_n < priv->rx_length)
    {
      /* If the upper layer does not receive all of the data from the receive
       * buffer, move the remaining data to the head of the buffer.
       */

      priv->rx_length -= recv_n;
      memmove(priv->rx_buffer, priv->rx_buffer + recv_n, priv->rx_length);
    }
  else
    {
      priv->rx_length = 0;
    }

  remaining_words = BYTES2WORDS(priv, priv->rx_length);

  leave_critical_section(flags);

  return remaining_words;
}

/****************************************************************************
 * Name: esp_spislave_ctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI Slave bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple SPI Slave interfaces)
 *
 * Returned Value:
 *   Valid SPI Slave controller structure reference on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct spi_slave_ctrlr_s *esp_spislave_ctrlr_initialize(int port)
{
  struct spi_slave_ctrlr_s *spislave_dev;
  struct spislave_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_ESPRESSIF_SPI2
      case ESPRESSIF_SPI2:
        priv = &esp_spi2slave_priv;
        break;
#endif
      default:
        return NULL;
    }

  spislave_dev = (struct spi_slave_ctrlr_s *)priv;

  flags = enter_critical_section();

  if ((volatile int)priv->refs != 0)
    {
      leave_critical_section(flags);

      return spislave_dev;
    }

  /* Attach IRQ for CS pin interrupt */

  DEBUGVERIFY(irq_attach(ESP_PIN2IRQ(priv->config->cs_pin),
                         spislave_cs_interrupt,
                         priv));

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->config->irq);
    }

  priv->cpuint = esp_setup_irq(priv->config->periph,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      leave_critical_section(flags);

      return NULL;
    }

  if (irq_attach(priv->config->irq, spislave_periph_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp_teardown_irq(priv->config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return NULL;
    }

  priv->refs++;

  leave_critical_section(flags);

  return spislave_dev;
}

/****************************************************************************
 * Name: esp_spislave_ctrlr_uninitialize
 *
 * Description:
 *   Uninitialize an SPI Slave bus.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp_spislave_ctrlr_uninitialize(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(ctrlr != NULL);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  up_disable_irq(priv->config->irq);
  esp_teardown_irq(priv->config->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;

  esp_gpioirqdisable(ESP_PIN2IRQ(priv->config->cs_pin));

  /* Disable the trans_done interrupt */

  spi_ll_disable_intr(priv->ctx.hw, SPI_LL_INTR_TRANS_DONE);

  periph_module_disable(priv->module);

  priv->cpuint = -ENOMEM;
  priv->mode = SPISLAVE_MODE0;
  priv->nbits = 0;
  priv->tx_length = 0;
  priv->rx_length = 0;
  priv->is_processing = false;
  priv->is_tx_enabled = false;

  leave_critical_section(flags);

  return OK;
}

#endif /* defined(CONFIG_ESPRESSIF_SPI) && defined (CONFIG_SPI_SLAVE) */
