/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_spi_slave.c
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

#if defined(CONFIG_ESP32C3_SPI) && defined(CONFIG_SPI_SLAVE)

#include <assert.h>
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

#include "esp32c3.h"
#include "esp32c3_spi.h"
#include "esp32c3_irq.h"
#include "esp32c3_gpio.h"

#include "riscv_arch.h"
#include "hardware/esp32c3_gpio_sigmap.h"
#include "hardware/esp32c3_pinmap.h"
#include "hardware/esp32c3_spi.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_system.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define SPI_SLAVE_BUFSIZE (CONFIG_ESP32C3_SPI2_SLAVE_BUFSIZE)

/* Verify whether SPI has been assigned IOMUX pins.
 * Otherwise, SPI signals will be routed via GPIO Matrix.
 */

#define SPI_IS_CS_IOMUX   (CONFIG_ESP32C3_SPI2_CSPIN == SPI2_IOMUX_CSPIN)
#define SPI_IS_CLK_IOMUX  (CONFIG_ESP32C3_SPI2_CLKPIN == SPI2_IOMUX_CLKPIN)
#define SPI_IS_MOSI_IOMUX (CONFIG_ESP32C3_SPI2_MOSIPIN == SPI2_IOMUX_MOSIPIN)
#define SPI_IS_MISO_IOMUX (CONFIG_ESP32C3_SPI2_MISOPIN == SPI2_IOMUX_MISOPIN)

#define SPI_VIA_IOMUX     (SPI_IS_CS_IOMUX) && (SPI_IS_CLK_IOMUX) && \
                          (SPI_IS_MOSI_IOMUX) && (SPI_IS_MISO_IOMUX)

/* SPI Slave interrupt mask */

#define SPI_INT_MASK      (SPI_TRANS_DONE_INT_ENA_M |      \
                           SPI_SLV_WR_DMA_DONE_INT_ENA_M | \
                           SPI_SLV_RD_DMA_DONE_INT_ENA_M | \
                           SPI_SLV_WR_BUF_DONE_INT_ENA_M | \
                           SPI_SLV_RD_BUF_DONE_INT_ENA_M)

/* SPI Slave default width */

#define SPI_SLAVE_DEFAULT_WIDTH (8)

/* SPI Slave default mode */

#define SPI_SLAVE_DEFAULT_MODE  (SPISLAVE_MODE0)

/* SPI Slave maximum buffer size in bytes */

#define SPI_SLAVE_HW_BUF_SIZE   (64)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define WORDS2BYTES(_priv, _wn)   ((_wn) * ((_priv)->nbits / 8))
#define BYTES2WORDS(_priv, _bn)   ((_bn) / ((_priv)->nbits / 8))

/* SPI Slave controller hardware configuration */

struct spislave_config_s
{
  int32_t width;              /* SPI Slave default width */
  enum spi_smode_e mode;      /* SPI Slave default mode */

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

  struct spi_sctrlr_s sctrlr;

  /* Reference to SPI Slave device interface */

  struct spi_sdev_s *sdev;

  /* Port configuration */

  const struct spislave_config_s *config;
  int refs;              /* Reference count */
  int cpuint;            /* SPI interrupt ID */
  enum spi_smode_e mode; /* Current SPI Slave hardware mode */
  uint8_t nbits;         /* Current SPI send/receive bits once transmission */
  uint32_t tx_length;    /* Location of next TX value */

  /* SPI Slave TX queue buffer */

  uint8_t tx_buffer[SPI_SLAVE_BUFSIZE];
  uint32_t rx_length;    /* Location of next RX value */

  /* SPI Slave RX queue buffer */

  uint8_t rx_buffer[SPI_SLAVE_BUFSIZE];

  /* Flag that indicates whether SPI Slave is currently processing */

  bool is_processing;

  /* Flag that indicates whether SPI Slave TX is currently enabled */

  bool is_tx_enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Slave controller interrupt handlers */

static int spislave_cs_interrupt(int irq, void *context, FAR void *arg);
static int spislave_periph_interrupt(int irq, void *context, FAR void *arg);

/* SPI Slave controller internal functions */

static void spislave_setmode(FAR struct spi_sctrlr_s *dev,
                             enum spi_smode_e mode);
static void spislave_setbits(FAR struct spi_sctrlr_s *dev, int nbits);
static void spislave_store_result(FAR struct spislave_priv_s *priv,
                                  uint32_t recv_bytes);
static void spislave_prepare_next_rx(FAR struct spislave_priv_s *priv);
static void spislave_evict_sent_data(FAR struct spislave_priv_s *priv,
                                     uint32_t sent_bytes);
static void spislave_write_tx_buffer(FAR struct spislave_priv_s *priv);
static void spislave_initialize(FAR struct spi_sctrlr_s *sctrlr);
static void spislave_deinitialize(FAR struct spi_sctrlr_s *sctrlr);

/* SPI Slave controller operations */

static void spislave_bind(FAR struct spi_sctrlr_s *sctrlr,
                          FAR struct spi_sdev_s *sdev,
                          enum spi_smode_e mode,
                          int nbits);
static void spislave_unbind(FAR struct spi_sctrlr_s *sctrlr);
static int spislave_enqueue(FAR struct spi_sctrlr_s *sctrlr,
                            FAR const void *data,
                            size_t nwords);
static bool spislave_qfull(FAR struct spi_sctrlr_s *sctrlr);
static void spislave_qflush(FAR struct spi_sctrlr_s *sctrlr);
static size_t spislave_qpoll(FAR struct spi_sctrlr_s *sctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SPI2
static const struct spislave_config_s esp32c3_spi2slave_config =
{
  .width        = SPI_SLAVE_DEFAULT_WIDTH,
  .mode         = SPI_SLAVE_DEFAULT_MODE,
  .cs_pin       = CONFIG_ESP32C3_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32C3_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32C3_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32C3_SPI2_CLKPIN,
  .periph       = ESP32C3_PERIPH_SPI2,
  .irq          = ESP32C3_IRQ_SPI2,
  .clk_bit      = SYSTEM_SPI2_CLK_EN,
  .rst_bit      = SYSTEM_SPI2_RST,
  .cs_insig     = FSPICS0_IN_IDX,
  .cs_outsig    = FSPICS0_OUT_IDX,
  .mosi_insig   = FSPID_IN_IDX,
  .mosi_outsig  = FSPID_OUT_IDX,
  .miso_insig   = FSPIQ_IN_IDX,
  .miso_outsig  = FSPIQ_OUT_IDX,
  .clk_insig    = FSPICLK_IN_IDX,
  .clk_outsig   = FSPICLK_OUT_IDX
};

static const struct spi_sctrlrops_s esp32c3_spi2slave_ops =
{
  .bind     = spislave_bind,
  .unbind   = spislave_unbind,
  .enqueue  = spislave_enqueue,
  .qfull    = spislave_qfull,
  .qflush   = spislave_qflush,
  .qpoll    = spislave_qpoll
};

static struct spislave_priv_s esp32c3_spi2slave_priv =
{
  .sctrlr        =
                  {
                    .ops = &esp32c3_spi2slave_ops
                  },
  .sdev          = NULL,
  .config        = &esp32c3_spi2slave_config,
  .refs          = 0,
  .cpuint        = -ENOMEM,
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
  .is_tx_enabled = false
};
#endif /* CONFIG_ESP32C3_SPI2 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spislave_peripheral_reset
 *
 * Description:
 *   Reset the SPI Slave peripheral before next transaction.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void spislave_peripheral_reset(void)
{
  setbits(SPI_SOFT_RESET_M, SPI_SLAVE_REG);
  resetbits(SPI_SOFT_RESET_M, SPI_SLAVE_REG);
}

/****************************************************************************
 * Name: spislave_cpu_tx_fifo_reset
 *
 * Description:
 *   Reset the BUF TX AFIFO, which is used to send data out in SPI Slave
 *   CPU-controlled mode transfer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void spislave_cpu_tx_fifo_reset(void)
{
  setbits(SPI_BUF_AFIFO_RST_M, SPI_DMA_CONF_REG);
  resetbits(SPI_BUF_AFIFO_RST_M, SPI_DMA_CONF_REG);
}

/****************************************************************************
 * Name: spislave_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   sctrlr - SPI Slave controller interface instance
 *   mode   - Requested SPI Slave mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_setmode(FAR struct spi_sctrlr_s *sctrlr,
                             enum spi_smode_e mode)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      uint32_t ck_idle_edge;
      uint32_t rsck_i_edge;
      uint32_t tsck_i_edge;
      uint32_t clk_mode_13;

      switch (mode)
        {
          case SPISLAVE_MODE0: /* CPOL=0; CPHA=0 */
            ck_idle_edge = 0;
            rsck_i_edge = 0;
            tsck_i_edge = 0;
            clk_mode_13 = 0;
            break;

          case SPISLAVE_MODE1: /* CPOL=0; CPHA=1 */
            ck_idle_edge = 0;
            rsck_i_edge = 1;
            tsck_i_edge = 1;
            clk_mode_13 = 1;
            break;

          case SPISLAVE_MODE2: /* CPOL=1; CPHA=0 */
            ck_idle_edge = 1;
            rsck_i_edge = 1;
            tsck_i_edge = 1;
            clk_mode_13 = 0;
            break;

          case SPISLAVE_MODE3: /* CPOL=1; CPHA=1 */
            ck_idle_edge = 1;
            rsck_i_edge = 0;
            tsck_i_edge = 0;
            clk_mode_13 = 1;
            break;

          default:
            spierr("Invalid mode: %d\n", mode);
            DEBUGASSERT(false);
            return;
        }

      modifyreg32(SPI_MISC_REG,
                  SPI_CK_IDLE_EDGE_M,
                  VALUE_TO_FIELD(ck_idle_edge, SPI_CK_IDLE_EDGE));

      modifyreg32(SPI_USER_REG,
                  SPI_RSCK_I_EDGE_M | SPI_TSCK_I_EDGE_M,
                  VALUE_TO_FIELD(rsck_i_edge, SPI_RSCK_I_EDGE) |
                  VALUE_TO_FIELD(tsck_i_edge, SPI_TSCK_I_EDGE));

      modifyreg32(SPI_SLAVE_REG,
                  SPI_CLK_MODE_13_M | SPI_RSCK_DATA_OUT_M,
                  VALUE_TO_FIELD(clk_mode_13, SPI_CLK_MODE_13));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spislave_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   sctrlr - SPI Slave controller interface instance
 *   nbits  - The number of bits in an SPI word
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_setbits(FAR struct spi_sctrlr_s *sctrlr, int nbits)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;

  spiinfo("nbits=%d\n", nbits);

  priv->nbits = nbits;
}

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

static int spislave_cs_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)arg;

  if (priv->is_processing)
    {
      priv->is_processing = false;
      SPI_SDEV_SELECT(priv->sdev, false);
    }

  return 0;
}

/****************************************************************************
 * Name: spislave_store_result
 *
 * Description:
 *   Fetch data from the data buffer registers and record the length.
 *   This is a post transaction operation.
 *
 * Input Parameters:
 *   priv       - Private SPI Slave controller structure
 *   recv_bytes - Number of received bytes
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_store_result(FAR struct spislave_priv_s *priv,
                                  uint32_t recv_bytes)
{
  uint32_t remaining_space = SPI_SLAVE_BUFSIZE - priv->rx_length;
  uint32_t bytes_to_copy = recv_bytes;

  if (bytes_to_copy > remaining_space)
    {
      spiwarn("RX buffer full! Discarded %" PRIu32 " received bytes\n",
              bytes_to_copy - remaining_space);

      bytes_to_copy = remaining_space;
    }

  /* If DMA is not enabled, software should copy incoming data from data
   * buffer registers to receive buffer.
   */

    {
      /* Set data_buf_reg with the address of the first data buffer
       * register (W0).
       */

      uintptr_t data_buf_reg = SPI_W0_REG;

      /* Read received data words from SPI data buffer registers. */

      for (int i = 0; i < bytes_to_copy; i += sizeof(uint32_t))
        {
          uint32_t r_wd = getreg32(data_buf_reg);

          memcpy(priv->rx_buffer + priv->rx_length + i, &r_wd,
                 sizeof(uint32_t));

          /* Update data_buf_reg to point to the next data buffer register. */

          data_buf_reg += sizeof(uintptr_t);
        }

      /* Clear data buffer registers to avoid echoing on the next transfer. */

      data_buf_reg = SPI_W0_REG;

      for (int i = 0; i < recv_bytes; i += sizeof(uint32_t))
        {
          putreg32(0, data_buf_reg);

          data_buf_reg += sizeof(uintptr_t);
        }
    }

  priv->rx_length += bytes_to_copy;
}

/****************************************************************************
 * Name: spislave_prepare_next_rx
 *
 * Description:
 *   Prepare the SPI Slave controller for receiving data on the next
 *   transaction.
 *
 * Input Parameters:
 *   priv   - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_prepare_next_rx(FAR struct spislave_priv_s *priv)
{
  if (priv->rx_length < SPI_SLAVE_BUFSIZE)
    {
      setbits(SPI_USR_MOSI_M, SPI_USER_REG);
    }
  else
    {
      spiwarn("RX buffer full! Disabling RX for next transaction\n");

      /* Disable SPI Slave input data phase. */

      resetbits(SPI_USR_MOSI_M, SPI_USER_REG);
    }
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

static void spislave_evict_sent_data(FAR struct spislave_priv_s *priv,
                                     uint32_t sent_bytes)
{
  if (sent_bytes < priv->tx_length)
    {
      priv->tx_length -= sent_bytes;
      memmove(priv->tx_buffer, priv->tx_buffer + sent_bytes,
              priv->tx_length);
    }
  else
    {
      priv->tx_length = 0;
    }
}

/****************************************************************************
 * Name: spislave_write_tx_buffer
 *
 * Description:
 *   Write to SPI Slave peripheral data buffer registers.
 *
 * Input Parameters:
 *   priv   - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_write_tx_buffer(FAR struct spislave_priv_s *priv)
{
  /* Initialize data_buf_reg with the address of the first data buffer
   * register (W0).
   */

  uintptr_t data_buf_reg = SPI_W0_REG;

  uint32_t transfer_size = MIN(SPI_SLAVE_HW_BUF_SIZE, priv->tx_length);

  /* Write data words to data buffer registers.
   * SPI peripheral contains 16 registers (W0 - W15).
   */

  for (int i = 0; i < transfer_size; i += sizeof(uint32_t))
    {
      uint32_t w_wd = UINT32_MAX;

      memcpy(&w_wd, priv->tx_buffer + i, sizeof(uint32_t));

      putreg32(w_wd, data_buf_reg);

      /* Update data_buf_reg to point to the next data buffer register. */

      data_buf_reg += sizeof(uintptr_t);
    }
}

/****************************************************************************
 * Name: spislave_prepare_next_tx
 *
 * Description:
 *   Prepare the SPI Slave controller for transmitting data on the next
 *   transaction.
 *
 * Input Parameters:
 *   priv   - Private SPI Slave controller structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_prepare_next_tx(FAR struct spislave_priv_s *priv)
{
  if (priv->tx_length != 0)
    {
      spislave_peripheral_reset();

      spislave_write_tx_buffer(priv);

      spislave_cpu_tx_fifo_reset();

      priv->is_tx_enabled = true;

      /* Enable SPI Slave output data phase. */

      setbits(SPI_USR_MISO_M, SPI_USER_REG);
    }
  else
    {
      spiwarn("TX buffer empty! Disabling TX for next transaction\n");

      spislave_cpu_tx_fifo_reset();

      priv->is_tx_enabled = false;

      /* Disable SPI Slave output data phase. */

      resetbits(SPI_USR_MISO_M, SPI_USER_REG);
    }
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

static int spislave_periph_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)arg;

  uint32_t regval = getreg32(SPI_SLAVE1_REG);
  uint32_t transfer_size = REG_MASK(regval, SPI_SLV_DATA_BITLEN) / 8;

  if (!priv->is_processing)
    {
      SPI_SDEV_SELECT(priv->sdev, true);
      priv->is_processing = true;
    }

  /* RX process */

  spislave_store_result(priv, transfer_size);

  spislave_prepare_next_rx(priv);

  /* TX process */

  if (priv->is_tx_enabled)
    {
      spislave_evict_sent_data(priv, transfer_size);
    }

  spislave_prepare_next_tx(priv);

  if (priv->is_processing && esp32c3_gpioread(priv->config->cs_pin))
    {
      priv->is_processing = false;
      SPI_SDEV_SELECT(priv->sdev, false);
    }

  /* Clear the trans_done interrupt flag */

  setbits(SPI_TRANS_DONE_INT_CLR_M, SPI_DMA_INT_CLR_REG);

  /* Trigger the start of user-defined transaction */

  setbits(SPI_USR_M, SPI_CMD_REG);

  return 0;
}

/****************************************************************************
 * Name: spislave_initialize
 *
 * Description:
 *   Initialize ESP32-C3 SPI Slave hardware interface.
 *
 * Input Parameters:
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_initialize(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  const struct spislave_config_s *config = priv->config;

  spiinfo("sctrlr=%p\n", sctrlr);

  esp32c3_gpiowrite(config->cs_pin, 1);
  esp32c3_gpiowrite(config->mosi_pin, 1);
  esp32c3_gpiowrite(config->miso_pin, 1);
  esp32c3_gpiowrite(config->clk_pin, 1);

#if SPI_VIA_IOMUX
  esp32c3_configgpio(config->cs_pin, INPUT_FUNCTION_2 | PULLUP);
  esp32c3_gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp32c3_configgpio(config->mosi_pin, INPUT_FUNCTION_2 | PULLUP);
  esp32c3_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp32c3_configgpio(config->miso_pin, OUTPUT_FUNCTION_2);
  esp32c3_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);

  esp32c3_configgpio(config->clk_pin, INPUT_FUNCTION_2 | PULLUP);
  esp32c3_gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);
#else
  esp32c3_configgpio(config->cs_pin, INPUT_FUNCTION_1 | PULLUP);
  esp32c3_gpio_matrix_in(config->cs_pin, config->cs_insig, 0);

  esp32c3_configgpio(config->mosi_pin, INPUT_FUNCTION_1 | PULLUP);
  esp32c3_gpio_matrix_in(config->mosi_pin, config->mosi_insig, 0);

  esp32c3_configgpio(config->miso_pin, OUTPUT_FUNCTION_1);
  esp32c3_gpio_matrix_out(config->miso_pin, config->miso_outsig, 0, 0);

  esp32c3_configgpio(config->clk_pin, INPUT_FUNCTION_1 | PULLUP);
  esp32c3_gpio_matrix_in(config->clk_pin, config->clk_insig, 0);
#endif

  setbits(config->clk_bit, SYSTEM_PERIP_CLK_EN0_REG);
  resetbits(config->rst_bit, SYSTEM_PERIP_RST_EN0_REG);

  /* Configure SPI Slave peripheral */

  putreg32(0, SPI_CLOCK_REG);

  putreg32(SPI_DOUTDIN_M, SPI_USER_REG);

  putreg32(0, SPI_CTRL_REG);

  putreg32(SPI_SLAVE_MODE_M, SPI_SLAVE_REG);

  spislave_peripheral_reset();

  /* Use all 64 bytes of the data buffer */

  resetbits(SPI_USR_MISO_HIGHPART_M | SPI_USR_MOSI_HIGHPART_M, SPI_USER_REG);

  /* Disable interrupts */

  resetbits(SPI_INT_MASK, SPI_DMA_INT_ENA_REG);

  /* Set the maximum data length to the size of the HW buffer in bits */

  putreg32((SPI_SLAVE_HW_BUF_SIZE * 8) - 1, SPI_MS_DLEN_REG);

  esp32c3_gpioirqenable(ESP32C3_PIN2IRQ(config->cs_pin), RISING);

  /* Force a transaction done interrupt.
   * This interrupt won't fire yet because we initialized the SPI interrupt
   * as disabled. This way, we can just enable the SPI interrupt and the
   * interrupt handler will kick in, handling any transactions that are
   * queued.
   */

  setbits(SPI_TRANS_DONE_INT_RAW_M, SPI_DMA_INT_RAW_REG);
  setbits(SPI_TRANS_DONE_INT_ENA_M, SPI_DMA_INT_ENA_REG);
}

/****************************************************************************
 * Name: spislave_deinitialize
 *
 * Description:
 *   Deinitialize ESP32-C3 SPI Slave hardware interface.
 *
 * Input Parameters:
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_deinitialize(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;

  esp32c3_gpioirqdisable(ESP32C3_PIN2IRQ(priv->config->cs_pin));

  /* Disable the trans_done interrupt */

  resetbits(SPI_TRANS_DONE_INT_ENA_M, SPI_DMA_INT_ENA_REG);

  setbits(priv->config->clk_bit, SYSTEM_PERIP_RST_EN0_REG);
  resetbits(priv->config->clk_bit, SYSTEM_PERIP_CLK_EN0_REG);

  priv->mode = SPISLAVE_MODE0;
  priv->nbits = 0;
  priv->tx_length = 0;
  priv->rx_length = 0;
  priv->is_processing = false;
  priv->is_tx_enabled = false;
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
 *   sctrlr - SPI Slave controller interface instance
 *   sdev   - SPI Slave device interface instance
 *   mode   - The SPI mode requested
 *   nbits  - The number of bits requests.
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

static void spislave_bind(FAR struct spi_sctrlr_s *sctrlr,
                          FAR struct spi_sdev_s *sdev,
                          enum spi_smode_e mode,
                          int nbits)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  FAR const void *data = NULL;
  irqstate_t flags;
  size_t num_words;

  spiinfo("sctrlr=%p sdev=%p mode=%d nbits=%d\n", sctrlr, sdev, mode, nbits);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev == NULL);
  DEBUGASSERT(sdev != NULL);
  DEBUGASSERT(nbits > 0);

  flags = enter_critical_section();

  priv->sdev = sdev;

  SPI_SDEV_SELECT(sdev, false);

  SPI_SDEV_CMDDATA(sdev, false);

  priv->rx_length = 0;
  priv->tx_length = 0;
  priv->is_tx_enabled = false;

  spislave_initialize(sctrlr);

  spislave_setmode(sctrlr, mode);
  spislave_setbits(sctrlr, nbits);

  num_words = SPI_SDEV_GETDATA(sdev, &data);

  if (data != NULL && num_words > 0)
    {
      size_t num_bytes = WORDS2BYTES(priv, num_words);
      memcpy(priv->tx_buffer, data, num_bytes);
      priv->tx_length += num_bytes;
    }

  /* Enable the CPU interrupt that is linked to the SPI Slave controller */

  up_enable_irq(priv->cpuint);

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
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_unbind(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev != NULL);

  spiinfo("Unbinding %p\n", priv->sdev);

  flags = enter_critical_section();

  up_disable_irq(priv->cpuint);

  esp32c3_gpioirqdisable(ESP32C3_PIN2IRQ(priv->config->cs_pin));

  /* Disable the trans_done interrupt */

  resetbits(SPI_TRANS_DONE_INT_ENA_M, SPI_DMA_INT_ENA_REG);

  resetbits(priv->config->clk_bit, SYSTEM_PERIP_CLK_EN0_REG);

  priv->sdev = NULL;

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
 *   sctrlr - SPI Slave controller interface instance
 *   data   - Pointer to the command/data mode data to be shifted out.
 *            The data width must be aligned to the nbits parameter which was
 *            previously provided to the bind() method.
 *   len    - Number of units of "nbits" wide to enqueue,
 *            "nbits" being the data width previously provided to the bind()
 *            method.
 *
 * Returned Value:
 *   Number of data items successfully queued, or a negated errno:
 *          - "len" if all the data was successfully queued
 *          - "0..len-1" if queue is full
 *          - "-errno" in any other error
 *
 ****************************************************************************/

static int spislave_enqueue(FAR struct spi_sctrlr_s *sctrlr,
                            FAR const void *data,
                            size_t len)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  size_t num_bytes = WORDS2BYTES(priv, len);
  size_t bufsize;
  irqstate_t flags;
  int enqueued_words;

  spiinfo("sctrlr=%p, data=%p, num_bytes=%zu\n", sctrlr, data, num_bytes);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev != NULL);

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
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

static bool spislave_qfull(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  irqstate_t flags;
  bool is_full = false;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev != NULL);

  spiinfo("sctrlr=%p\n", sctrlr);

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
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_qflush(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev != NULL);

  spiinfo("sctrlr=%p\n", sctrlr);

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
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Number of units of width "nbits" left in the RX queue. If the device
 *   accepted all the data, the return value will be 0.
 *
 ****************************************************************************/

static size_t spislave_qpoll(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  irqstate_t flags;
  uint32_t tmp;
  uint32_t recv_n;
  size_t remaining_words;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->sdev != NULL);

  spiinfo("sctrlr=%p\n", sctrlr);

  flags = enter_critical_section();

  tmp = SPI_SDEV_RECEIVE(priv->sdev, priv->rx_buffer,
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
 * Name: esp32c3_spislave_sctrlr_initialize
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

FAR struct spi_sctrlr_s *esp32c3_spislave_sctrlr_initialize(int port)
{
  FAR struct spi_sctrlr_s *spislave_dev;
  FAR struct spislave_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_ESP32C3_SPI2
      case ESP32C3_SPI2:
        priv = &esp32c3_spi2slave_priv;
        break;
#endif
      default:
        return NULL;
    }

  spislave_dev = (FAR struct spi_sctrlr_s *)priv;

  flags = enter_critical_section();

  if ((volatile int)priv->refs != 0)
    {
      leave_critical_section(flags);

      return spislave_dev;
    }

  /* Attach IRQ for CS pin interrupt */

  DEBUGVERIFY(irq_attach(ESP32C3_PIN2IRQ(priv->config->cs_pin),
                         spislave_cs_interrupt,
                         priv));

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(priv->cpuint);
    }

  priv->cpuint = esp32c3_request_irq(priv->config->periph,
                                     ESP32C3_INT_PRIO_DEF,
                                     ESP32C3_INT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      leave_critical_section(flags);

      return NULL;
    }

  if (irq_attach(priv->config->irq, spislave_periph_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32c3_free_cpuint(priv->config->periph);
      priv->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return NULL;
    }

  priv->refs++;

  leave_critical_section(flags);

  return spislave_dev;
}

/****************************************************************************
 * Name: esp32c3_spislave_sctrlr_uninitialize
 *
 * Description:
 *   Uninitialize an SPI Slave bus.
 *
 * Input Parameters:
 *   sctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_spislave_sctrlr_uninitialize(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct spislave_priv_s *priv = (FAR struct spislave_priv_s *)sctrlr;
  irqstate_t flags;

  DEBUGASSERT(sctrlr != NULL);

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

  up_disable_irq(priv->cpuint);
  esp32c3_free_cpuint(priv->config->periph);
  priv->cpuint = -ENOMEM;

  spislave_deinitialize(sctrlr);

  leave_critical_section(flags);

  return OK;
}

#endif /* defined(CONFIG_ESP32C3_SPI) && defined (CONFIG_SPI_SLAVE) */
