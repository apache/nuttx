/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spi_slave.c
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

#ifdef CONFIG_ESP32_SPI

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#include <arch/board/board.h>

#include "esp32_spi.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"
#include "esp32_dma.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_pinmap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define SPI_SLAVE_BUFSIZE     (CONFIG_SPI_SLAVE_BUFSIZE)

/* SPI DMA channel number */

#define SPI_DMA_CHANNEL_MAX   (2)

/* SPI DMA RX/TX description number */

#if SPI_SLAVE_BUFSIZE % ESP32_DMA_BUFLEN_MAX
#  define SPI_DMADESC_NUM (SPI_SLAVE_BUFSIZE / ESP32_DMA_BUFLEN_MAX + 1)
#else
#  define SPI_DMADESC_NUM (SPI_SLAVE_BUFSIZE / ESP32_DMA_BUFLEN_MAX)
#endif

/* SPI DMA reset before exchange */

#define SPI_DMA_RESET_MASK (SPI_AHBM_RST_M | SPI_AHBM_FIFO_RST_M | \
                            SPI_OUT_RST_M | SPI_IN_RST_M)

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define WORDS2BYTES(_priv, _wn)   (_wn * ((_priv)->nbits / 8))
#define BYTES2WORDS(_priv, _bn)   (_bn / ((_priv)->nbits / 8))

/* SPI Device hardware configuration */

struct esp32_spislv_config_s
{
  uint32_t reg_base;          /* SPI register base address */

  enum spi_mode_e mode;       /* SPI default mode */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */

  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* I2C reset bit */

  bool use_dma;               /* Use DMA */
  uint8_t dma_chan_s;         /* DMA channel register shift */
  uint8_t dma_chan;           /* DMA channel */
  uint32_t dma_clk_bit;       /* DMA clock enable bit */
  uint32_t dma_rst_bit;       /* DMA reset bit */

  uint32_t cs_insig;          /* SPI CS input signal index */
  uint32_t cs_outsig;         /* SPI CS output signal index */
  uint32_t mosi_insig;        /* SPI MOSI input signal index */
  uint32_t mosi_outsig;       /* SPI MOSI output signal index */
  uint32_t miso_insig;        /* SPI MISO input signal index */
  uint32_t miso_outsig;       /* SPI MISO output signal index */
  uint32_t clk_insig;         /* SPI CLK input signal index */
  uint32_t clk_outsig;        /* SPI CLK output signal index */
};

struct esp32_spislv_priv_s
{
  /* Externally visible part of the SPI slave controller interface */

  struct spi_slave_ctrlr_s ctrlr;

  /* Externally visible part of the SPI interface */

  struct spi_slave_dev_s   *dev;

  const struct esp32_spislv_config_s *config; /* Port configuration */

  uint8_t          cpu;         /* CPU ID */
  int              cpuint;      /* SPI interrupt ID */

  enum spi_mode_e  mode;        /* Actual SPI hardware mode */
  uint8_t          nbits;       /* Actual SPI send/receive bits once transmission */
  int              refs;        /* Check if it is initialized */

  uint32_t         txlen;       /* Location of next RX value */

  /* SPI slave TX queue buffer */

  uint8_t          txbuffer[SPI_SLAVE_BUFSIZE];

  uint32_t         rxlen;       /* Location of next RX value */

  /* SPI slave RX queue buffer */

  uint8_t          rxbuffer[SPI_SLAVE_BUFSIZE];

  uint32_t         outval;      /* Default shift-out value */

  bool             process;     /* If SPI Slave process */

  bool             txen;        /* Enable TX */

  /* Copy from config to speed up checking */

  bool dma_chan;

  spinlock_t lock;              /* Device specific lock. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_spislv_setmode(struct spi_slave_ctrlr_s *ctrlr,
                                 enum spi_mode_e mode);
static void esp32_spislv_setbits(struct spi_slave_ctrlr_s *ctrlr,
                                 int nbits);
static int esp32_spislv_interrupt(int irq, void *context, void *arg);
static void esp32_spislv_initialize(struct spi_slave_ctrlr_s *ctrlr);
static void esp32_spislv_bind(struct spi_slave_ctrlr_s *ctrlr,
                              struct spi_slave_dev_s *dev,
                              enum spi_slave_mode_e mode,
                              int nbits);
static void esp32_spislv_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int esp32_spislv_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                const void *data,
                                size_t nwords);
static bool esp32_spislv_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void esp32_spislv_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t esp32_spislv_qpoll(struct spi_slave_ctrlr_s *ctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2
static const struct esp32_spislv_config_s esp32_spi2_config =
{
  .reg_base     = REG_SPI_BASE(2),
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI2_CLKPIN,
  .periph       = ESP32_PERIPH_SPI2,
  .irq          = ESP32_IRQ_SPI2,
  .clk_bit      = DPORT_SPI_CLK_EN_2,
  .rst_bit      = DPORT_SPI_RST_2,
#ifdef CONFIG_ESP32_SPI2_DMA
  .use_dma      = true,
#else
  .use_dma      = false,
#endif
  .dma_chan_s   = 2,
  .dma_chan     = 1,
  .dma_clk_bit  = DPORT_SPI_DMA_CLK_EN,
  .dma_rst_bit  = DPORT_SPI_DMA_RST,
  .cs_insig     = HSPICS0_IN_IDX,
  .cs_outsig    = HSPICS0_OUT_IDX,
  .mosi_insig   = HSPID_IN_IDX,
  .mosi_outsig  = HSPID_OUT_IDX,
  .miso_insig   = HSPIQ_IN_IDX,
  .miso_outsig  = HSPIQ_OUT_IDX,
  .clk_insig    = HSPICLK_IN_IDX,
  .clk_outsig   = HSPICLK_OUT_IDX
};

static const struct spi_slave_ctrlrops_s esp32_spi2slv_ops =
{
  .bind     = esp32_spislv_bind,
  .unbind   = esp32_spislv_unbind,
  .enqueue  = esp32_spislv_enqueue,
  .qfull    = esp32_spislv_qfull,
  .qflush   = esp32_spislv_qflush,
  .qpoll    = esp32_spislv_qpoll
};

static struct esp32_spislv_priv_s esp32_spi2slv_priv =
{
  .ctrlr =
              {
                .ops = &esp32_spi2slv_ops
              },
  .config = &esp32_spi2_config,
  .mode = SPIDEV_MODE3
};
#endif /* CONFIG_ESP32_SPI2 */

#ifdef CONFIG_ESP32_SPI3
static const struct esp32_spislv_config_s esp32_spi3_config =
{
  .reg_base     = REG_SPI_BASE(3),
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI3_CLKPIN,
  .periph       = ESP32_PERIPH_SPI3,
  .irq          = ESP32_IRQ_SPI3,
  .clk_bit      = DPORT_SPI_CLK_EN,
  .rst_bit      = DPORT_SPI_RST,
#ifdef CONFIG_ESP32_SPI3_DMA
  .use_dma      = true,
#else
  .use_dma      = false,
#endif
  .dma_chan_s   = 4,
  .dma_chan     = 2,
  .dma_clk_bit  = DPORT_SPI_DMA_CLK_EN,
  .dma_rst_bit  = DPORT_SPI_DMA_RST,
  .cs_insig     = VSPICS0_IN_IDX,
  .cs_outsig    = VSPICS0_OUT_IDX,
  .mosi_insig   = VSPID_IN_IDX,
  .mosi_outsig  = VSPID_OUT_IDX,
  .miso_insig   = VSPIQ_IN_IDX,
  .miso_outsig  = VSPIQ_OUT_IDX,
  .clk_insig    = VSPICLK_IN_IDX,
  .clk_outsig   = VSPICLK_OUT_MUX_IDX
};

static const struct spi_slave_ctrlrops_s esp32_spi3slv_ops =
{
  .bind     = esp32_spislv_bind,
  .unbind   = esp32_spislv_unbind,
  .enqueue  = esp32_spislv_enqueue,
  .qfull    = esp32_spislv_qfull,
  .qflush   = esp32_spislv_qflush,
  .qpoll    = esp32_spislv_qpoll
};

static struct esp32_spislv_priv_s esp32_spi3slv_priv =
{
  .ctrlr =
              {
                .ops = &esp32_spi3slv_ops
              },
  .config = &esp32_spi3_config,
  .mode = SPIDEV_MODE3
};
#endif /* CONFIG_ESP32_SPI3 */

/* SPI DMA RX/TX description */

struct esp32_dmadesc_s s_rx_desc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];
struct esp32_dmadesc_s s_tx_desc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spi_set_reg
 *
 * Description:
 *   Set the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   value  - Value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_set_reg(struct esp32_spislv_priv_s *priv,
                                     int offset,
                                     uint32_t value)
{
  putreg32(value, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_get_reg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *
 * Returned Value:
 *   The contents of the register
 *
 ****************************************************************************/

static inline uint32_t esp32_spi_get_reg(struct esp32_spislv_priv_s *priv,
                                         int offset)
{
  return getreg32(priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_set_regbits
 *
 * Description:
 *   Set the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_set_regbits(struct esp32_spislv_priv_s *priv,
                                         int offset,
                                         uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp | bits, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_reset_regbits
 *
 * Description:
 *   Clear the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32_spi_reset_regbits(struct esp32_spislv_priv_s *priv,
                                           int offset,
                                           uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp & (~bits), priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spi_iomux
 *
 * Description:
 *   Check if the option SPI GPIO pins can use IOMUX directly
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *
 * Returned Value:
 *   True if can use IOMUX or false if can't.
 *
 ****************************************************************************/

static inline bool esp32_spi_iomux(struct esp32_spislv_priv_s *priv)
{
  bool mapped = false;
  const struct esp32_spislv_config_s *cfg = priv->config;

  if (REG_SPI_BASE(2) == cfg->reg_base)
    {
      if (cfg->mosi_pin == SPI2_IOMUX_MOSIPIN &&
          cfg->cs_pin == SPI2_IOMUX_CSPIN &&
          cfg->miso_pin == SPI2_IOMUX_MISOPIN &&
          cfg->clk_pin == SPI2_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }
  else if (REG_SPI_BASE(3) == cfg->reg_base)
    {
      if (cfg->mosi_pin == SPI3_IOMUX_MOSIPIN &&
          cfg->cs_pin == SPI3_IOMUX_CSPIN &&
          cfg->miso_pin == SPI3_IOMUX_MISOPIN &&
          cfg->clk_pin == SPI3_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }

  return mapped;
}

/****************************************************************************
 * Name: esp32_spislv_setmode
 *
 * Description:
 *   Set the SPI Slave mode.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   mode  - Requested SPI Slave mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32_spislv_setmode(struct spi_slave_ctrlr_s *ctrlr,
                                 enum spi_mode_e mode)
{
  uint32_t ck_idle_edge;
  uint32_t ck_in_edge;
  uint32_t miso_delay_mode;
  uint32_t miso_delay_num;
  uint32_t mosi_delay_mode;
  uint32_t mosi_delay_num;
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPISLAVE_MODE0: /* CPOL=0; CPHA=0 */
          if (priv->dma_chan)
            {
              ck_idle_edge = 0;
              ck_in_edge = 1;
              miso_delay_mode = 0;
              miso_delay_num = 1;
              mosi_delay_mode = 0;
              mosi_delay_num = 1;
            }
          else
            {
              ck_idle_edge = 1;
              ck_in_edge = 0;
              miso_delay_mode = 0;
              miso_delay_num = 0;
              mosi_delay_mode = 2;
              mosi_delay_num = 2;
            }
          break;

        case SPISLAVE_MODE1: /* CPOL=0; CPHA=1 */
          ck_idle_edge = 1;
          ck_in_edge = 1;
          miso_delay_mode = 2;
          miso_delay_num = 0;
          mosi_delay_mode = 0;
          mosi_delay_num = 0;
          break;

        case SPISLAVE_MODE2: /* CPOL=1; CPHA=0 */
          if (priv->dma_chan)
            {
              ck_idle_edge = 1;
              ck_in_edge = 0;
              miso_delay_mode = 0;
              miso_delay_num = 1;
              mosi_delay_mode = 0;
              mosi_delay_num = 1;
            }
          else
            {
              ck_idle_edge = 0;
              ck_in_edge = 1;
              miso_delay_mode = 0;
              miso_delay_num = 0;
              mosi_delay_mode = 1;
              mosi_delay_num = 2;
            }
          break;

        case SPISLAVE_MODE3: /* CPOL=1; CPHA=1 */
          ck_idle_edge = 0;
          ck_in_edge = 0;
          miso_delay_mode = 1;
          miso_delay_num = 0;
          mosi_delay_mode = 0;
          mosi_delay_num = 0;
          break;

        default:
          return;
        }

      esp32_spi_reset_regbits(priv,
                              SPI_PIN_OFFSET,
                              SPI_CK_IDLE_EDGE_M);
      esp32_spi_set_regbits(priv,
                            SPI_PIN_OFFSET,
                            (ck_idle_edge << SPI_CK_IDLE_EDGE_S));

      esp32_spi_reset_regbits(priv,
                              SPI_USER_OFFSET,
                              SPI_CK_I_EDGE_M);
      esp32_spi_set_regbits(priv,
                            SPI_USER_OFFSET,
                            (ck_in_edge << SPI_CK_I_EDGE_S));

      esp32_spi_reset_regbits(priv,
                              SPI_CTRL2_OFFSET,
                              SPI_MISO_DELAY_MODE_M |
                              SPI_MISO_DELAY_NUM_M |
                              SPI_MOSI_DELAY_NUM_M |
                              SPI_MOSI_DELAY_MODE_M);
      esp32_spi_set_regbits(priv,
                            SPI_CTRL2_OFFSET,
                            (miso_delay_mode << SPI_MISO_DELAY_MODE_S) |
                            (miso_delay_num  << SPI_MISO_DELAY_NUM_S) |
                            (mosi_delay_mode << SPI_MOSI_DELAY_MODE_S) |
                            (mosi_delay_num  << SPI_MOSI_DELAY_NUM_S));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32_spislv_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   nbits - The number of bits in an SPI word
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32_spislv_setbits(struct spi_slave_ctrlr_s *ctrlr,
                                 int nbits)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;

  spiinfo("nbits=%d\n", nbits);

  if (nbits != priv->nbits)
    {
      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: esp32_io_interrupt
 *
 * Description:
 *   Common I/O interrupt handler
 *
 * Input Parameters:
 *   arg - I/O controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_io_interrupt(int irq, void *context, void *arg)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)arg;

  if (priv->process == true)
    {
      priv->process = false;
      SPIS_DEV_SELECT(priv->dev, false);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32_spislv_tx
 *
 * Description:
 *   Process SPI slave TX.
 *
 *   DMA mode    : Initialize register to prepare for TX
 *   Non-DMA mode: Fill data to TX register
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_tx(struct esp32_spislv_priv_s *priv)
{
  int i;
  uint32_t regval;

  if (priv->dma_chan)
    {
      esp32_dma_init(s_tx_desc[priv->dma_chan - 1], SPI_DMADESC_NUM,
                     priv->txbuffer, priv->txlen);

      regval = (uint32_t)s_tx_desc[priv->dma_chan - 1] & SPI_OUTLINK_ADDR_V;
      esp32_spi_set_reg(priv, SPI_DMA_OUT_LINK_OFFSET,
                        regval | SPI_OUTLINK_START_M);
      esp32_spi_set_reg(priv, SPI_SLV_WRBUF_DLEN_OFFSET,
                        priv->txlen * 8 - 1);

      esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MISO_M);
    }
  else
    {
      for (i = 0; i < priv->txlen; i += 4)
        {
          esp32_spi_set_reg(priv, SPI_W8_OFFSET + i,
                            *(uint32_t *)(&priv->txbuffer[i]));
        }

      esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MISO_M);
    }
}

/****************************************************************************
 * Name: esp32_spislv_rx
 *
 * Description:
 *   Process SPI slave RX. Process SPI slave device receive callback by
 *   calling SPIS_DEV_RECEIVE and prepare for next RX.
 *
 *   DMA mode : Initialize register to prepare for RX
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_rx(struct esp32_spislv_priv_s *priv)
{
  uint32_t tmp;
  uint32_t recv_n;
  uint32_t regval;

  tmp = SPIS_DEV_RECEIVE(priv->dev, priv->rxbuffer,
                         BYTES2WORDS(priv, priv->rxlen));
  recv_n = WORDS2BYTES(priv, tmp);

  if (priv->dma_chan)
    {
      DEBUGASSERT((recv_n % 4) == 0);
    }

  if (recv_n < priv->rxlen)
    {
      /** If upper layer does not receive all data of receive
       *  buffer, move the rest data to head of the buffer
       */

      priv->rxlen -= recv_n;
      memmove(priv->rxbuffer, priv->rxbuffer + recv_n, priv->rxlen);
    }
  else
    {
      priv->rxlen = 0;
    }

  if (priv->dma_chan)
    {
      tmp = SPI_SLAVE_BUFSIZE - priv->rxlen;
      if (tmp)
        {
          /* Start to receive next block of data */

          esp32_dma_init(s_rx_desc[priv->dma_chan - 1], SPI_DMADESC_NUM,
                        priv->rxbuffer + priv->rxlen, tmp);

          regval = (uint32_t)s_rx_desc[priv->dma_chan - 1] &
                   SPI_INLINK_ADDR_V;
          esp32_spi_set_reg(priv, SPI_DMA_IN_LINK_OFFSET,
                            regval | SPI_INLINK_START_M);
          esp32_spi_set_reg(priv, SPI_SLV_RDBUF_DLEN_OFFSET, tmp * 8 - 1);

          esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MOSI_M);
        }
    }
}

/****************************************************************************
 * Name: esp32_spislv_interrupt
 *
 * Description:
 *   Common SPI interrupt handler
 *
 * Input Parameters:
 *   arg - SPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_spislv_interrupt(int irq, void *context, void *arg)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)arg;
  uint32_t n;
  uint32_t tmp;
  int i;

  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_TRANS_DONE_M);

  if (priv->dma_chan)
    {
      esp32_spi_set_regbits(priv, SPI_DMA_CONF_OFFSET, SPI_DMA_RESET_MASK);
      esp32_spi_reset_regbits(priv, SPI_DMA_CONF_OFFSET, SPI_DMA_RESET_MASK);
    }

  if (priv->process == false)
    {
      SPIS_DEV_SELECT(priv->dev, true);
      priv->process = true;
    }

  /* Read and calculate read bytes */

  n = (esp32_spi_get_reg(priv, SPI_SLV_RD_BIT_OFFSET) + 1) / 8;

  esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MOSI_M);

  /* RX process */

  if (!priv->dma_chan)
    {
      /** With DMA, software should copy data from register
       *  to receive buffer
       */

      for (i = 0; i < n; i += 4)
        {
          tmp = esp32_spi_get_reg(priv, SPI_W0_OFFSET + i);
          memcpy(priv->rxbuffer + priv->rxlen + i, &tmp, n);
        }
    }

  priv->rxlen += n;

  esp32_spislv_rx(priv);

  /* TX process */

  if (priv->txen)
    {
      if (n < priv->txlen)
        {
          priv->txlen -= n;
          memmove(priv->txbuffer, priv->txbuffer + n, priv->txlen);
        }
      else
        {
          priv->txlen = 0;
          priv->txen = false;
        }
    }

  if (priv->txlen)
    {
      esp32_spislv_tx(priv);
      priv->txen = true;
    }

  if (priv->process == true && esp32_gpioread(priv->config->cs_pin))
    {
      priv->process = false;
      SPIS_DEV_SELECT(priv->dev, false);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32_spislv_initialize
 *
 * Description:
 *   Initialize ESP32 SPI Slave hardware interface
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_initialize(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  const struct esp32_spislv_config_s *config = priv->config;
  uint32_t regval;

  esp32_gpiowrite(config->cs_pin, 1);
  esp32_gpiowrite(config->mosi_pin, 1);
  esp32_gpiowrite(config->miso_pin, 1);
  esp32_gpiowrite(config->clk_pin, 1);

  if (esp32_spi_iomux(priv))
    {
      esp32_configgpio(config->cs_pin, INPUT_FUNCTION_2 | PULLUP);
      esp32_configgpio(config->mosi_pin, INPUT_FUNCTION_2 | PULLUP);
      esp32_configgpio(config->miso_pin, OUTPUT_FUNCTION_2);
      esp32_configgpio(config->clk_pin, INPUT_FUNCTION_2 | PULLUP);
    }
  else
    {
      esp32_configgpio(config->cs_pin, INPUT_FUNCTION_3 | PULLUP);
      esp32_gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
      esp32_gpio_matrix_in(config->cs_pin, config->cs_insig, 0);

      esp32_configgpio(config->mosi_pin, INPUT_FUNCTION_3 | PULLUP);
      esp32_gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);
      esp32_gpio_matrix_in(config->mosi_pin, config->mosi_insig, 0);

      esp32_configgpio(config->miso_pin, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(config->miso_pin, config->miso_outsig, 0, 0);
      esp32_gpio_matrix_in(config->miso_pin, config->miso_insig, 0);

      esp32_configgpio(config->clk_pin, INPUT_FUNCTION_3 | PULLUP);
      esp32_gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);
      esp32_gpio_matrix_in(config->clk_pin, config->clk_insig, 0);
    }

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
  modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);

  esp32_spi_set_reg(priv, SPI_USER_OFFSET, SPI_DOUTDIN_M |
                                           SPI_USR_MOSI_M |
                                           SPI_USR_MISO_M |
                                           SPI_USR_MISO_HIGHPART_M);
  esp32_spi_set_reg(priv, SPI_USER1_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_PIN_OFFSET, SPI_CS1_DIS_M | SPI_CS2_DIS_M);
  esp32_spi_set_reg(priv, SPI_CTRL_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_CTRL2_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_USER2_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_CLOCK_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_SLAVE_OFFSET, SPI_SLAVE_MODE_M |
                                            SPI_SLV_WR_RD_BUF_EN_M |
                                            SPI_INT_EN_M);

  if (priv->dma_chan)
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->dma_clk_bit);
      modifyreg32(DPORT_PERIP_RST_EN_REG, config->dma_rst_bit, 0);

      modifyreg32(DPORT_SPI_DMA_CHAN_SEL_REG, 0,
                  (config->dma_chan << config->dma_chan_s));

      esp32_spi_set_reg(priv, SPI_DMA_CONF_OFFSET, SPI_OUT_DATA_BURST_EN_M |
                                                   SPI_INDSCR_BURST_EN_M |
                                                   SPI_OUTDSCR_BURST_EN_M);

      esp32_dma_init(s_rx_desc[priv->dma_chan - 1], SPI_DMADESC_NUM,
                     priv->rxbuffer, SPI_SLAVE_BUFSIZE);

      regval = (uint32_t)s_rx_desc[priv->dma_chan - 1] & SPI_INLINK_ADDR_V;
      esp32_spi_set_reg(priv, SPI_DMA_IN_LINK_OFFSET,
                        regval | SPI_INLINK_START_M);
      esp32_spi_set_reg(priv, SPI_SLV_RDBUF_DLEN_OFFSET,
                        SPI_SLAVE_BUFSIZE * 8 - 1);
      esp32_spi_set_reg(priv, SPI_SLV_WRBUF_DLEN_OFFSET,
                        SPI_SLAVE_BUFSIZE * 8 - 1);

      esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MOSI_M);
    }
  else
    {
      /* TX/RX hardware fill can cache 4 words = 32 bytes = 256 bits */

      esp32_spi_set_reg(priv, SPI_SLV_WRBUF_DLEN_OFFSET, 256 - 1);
      esp32_spi_set_reg(priv, SPI_SLV_RDBUF_DLEN_OFFSET, 256 - 1);
    }

  esp32_spislv_setmode(ctrlr, config->mode);
  esp32_spislv_setbits(ctrlr, 8);

  esp32_spi_set_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);

  esp32_gpioirqenable(ESP32_PIN2IRQ(config->cs_pin), RISING);

  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_TRANS_DONE_M);
}

/****************************************************************************
 * Name: esp32_spislv_deinit
 *
 * Description:
 *   Deinitialize ESP32 SPI Slave hardware interface
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_deinit(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;

  esp32_gpioirqdisable(ESP32_PIN2IRQ(priv->config->cs_pin));
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_INT_EN_M);

  modifyreg32(DPORT_PERIP_RST_EN_REG, 0, priv->config->clk_bit);
  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);

  priv->mode = SPIDEV_MODE0;
  priv->nbits = 0;
  priv->txlen = 0;
  priv->rxlen = 0;
  priv->process = false;
  priv->txen = false;
  priv->dma_chan = false;
}

/****************************************************************************
 * Name: esp32_spislv_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface.  Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *   dev   - SPI slave device interface instance
 *   mode  - The SPI Slave mode requested
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_bind(struct spi_slave_ctrlr_s *ctrlr,
                              struct spi_slave_dev_s *dev,
                              enum spi_slave_mode_e mode,
                              int nbits)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  irqstate_t flags;

  spiinfo("dev=%p mode=%d nbits=%d\n", dev, mode, nbits);

  DEBUGASSERT(priv != NULL && priv->dev == NULL && dev != NULL);

  flags = spin_lock_irqsave(&priv->lock);

  priv->dev = dev;

  SPIS_DEV_SELECT(dev, false);

  SPIS_DEV_CMDDATA(dev, false);

  priv->rxlen = 0;

  priv->txlen = 0;
  priv->txen  = false;

  esp32_spislv_initialize(ctrlr);

  esp32_spislv_setmode(ctrlr, mode);
  esp32_spislv_setbits(ctrlr, nbits);

  up_enable_irq(priv->config->irq);

  esp32_spi_set_regbits(priv, SPI_CMD_OFFSET, SPI_USR_M);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32_spislv_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface.  Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state,
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spislv_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  spiinfo("Unbinding %p\n", priv->dev);

  DEBUGASSERT(priv->dev != NULL);

  flags = spin_lock_irqsave(&priv->lock);

  up_disable_irq(priv->config->irq);

  esp32_gpioirqdisable(ESP32_PIN2IRQ(priv->config->cs_pin));
  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_INT_EN_M);
  if (priv->dma_chan)
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->dma_clk_bit, 0);
    }

  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);

  priv->dev = NULL;

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32_spislv_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface.  This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on anyin-process or currently "committed" transfers
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   Zero if the word was successfully queue; A negated errno valid is
 *   returned on any failure to enqueue the word (such as if the queue is
 *   full).
 *
 ****************************************************************************/

static int esp32_spislv_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                const void *data,
                                size_t nwords)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  size_t n = WORDS2BYTES(priv, nwords);
  size_t bufsize;
  irqstate_t flags;
  int ret;

  spiinfo("spi_enqueue(ctrlr=%p, data=%p, nwords=%d)\n",
          ctrlr, data, nwords);
  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  flags = spin_lock_irqsave(&priv->lock);

  bufsize = SPI_SLAVE_BUFSIZE - priv->txlen;
  if (!bufsize)
    {
      ret = -ENOSPC;
    }
  else
    {
      n = MIN(n, bufsize);
      memcpy(priv->txbuffer + priv->txlen, data, n);
      priv->txlen += n;
      ret = OK;

      if (priv->process == false)
        {
          esp32_spislv_tx(priv);
          priv->txen = true;
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: esp32_spislv_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   true if the output wueue is full
 *
 ****************************************************************************/

static bool esp32_spislv_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  irqstate_t flags;
  bool ret = 0;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  spiinfo("spi_qfull(ctrlr=%p)\n", ctrlr);

  flags = spin_lock_irqsave(&priv->lock);
  ret = priv->txlen == SPI_SLAVE_BUFSIZE;
  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: esp32_spislv_qflush
 *
 * Description:
 *   Discard all saved values in the output queue.  On return from this
 *   function the output queue will be empty.  Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spislv_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  flags = spin_lock_irqsave(&priv->lock);
  priv->rxlen = 0;
  priv->txlen = 0;
  priv->txen  = false;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32_spislv_qpoll
 *
 * Description:
 *   Tell the controller to output all the receive queue data.
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   Number of units of width "nbits" left in the rx queue. If the device
 *   accepted all the data, the return value will be 0
 *
 ****************************************************************************/

static size_t esp32_spislv_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;
  irqstate_t flags;
  uint32_t n;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

  flags = spin_lock_irqsave(&priv->lock);

  esp32_spislv_rx(priv);
  n = priv->rxlen;

  spin_unlock_irqrestore(&priv->lock, flags);

  return n;
}

/****************************************************************************
 * Name: esp32_spislv_ctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI slave bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI slave interfaces)
 *
 * Returned Value:
 *   Valid SPI slave device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_slave_ctrlr_s *esp32_spislv_ctrlr_initialize(int port)
{
  int ret;
  struct spi_slave_ctrlr_s *spislv_dev;
  struct esp32_spislv_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_ESP32_SPI2
      case 2:
        priv = &esp32_spi2slv_priv;
        break;
#endif
#ifdef CONFIG_ESP32_SPI3
      case 3:
        priv = &esp32_spi3slv_priv;
        break;
#endif
      default:
        return NULL;
    }

  spislv_dev = (struct spi_slave_ctrlr_s *)priv;

  flags = spin_lock_irqsave(&priv->lock);

  if ((volatile int)priv->refs != 0)
    {
      spin_unlock_irqrestore(&priv->lock, flags);

      return spislv_dev;
    }

  if (priv->config->use_dma)
    {
      priv->dma_chan = priv->config->dma_chan;
    }
  else
    {
      priv->dma_chan = 0;
    }

  DEBUGVERIFY(irq_attach(ESP32_PIN2IRQ(priv->config->cs_pin),
                         esp32_io_interrupt,
                         priv));

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32_setup_irq(priv->cpu, priv->config->periph,
                                 1, ESP32_CPUINT_LEVEL);

  ret = irq_attach(priv->config->irq, esp32_spislv_interrupt, priv);
  if (ret != OK)
    {
      esp32_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);

      spin_unlock_irqrestore(&priv->lock, flags);

      return NULL;
    }

  priv->refs++;

  spin_unlock_irqrestore(&priv->lock, flags);

  return spislv_dev;
}

/****************************************************************************
 * Name: esp32_spislv_ctrlr_uninitialize
 *
 * Description:
 *   Uninitialize an SPI slave bus
 *
 * Input Parameters:
 *   ctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   OK if success or fail
 *
 ****************************************************************************/

int esp32_spislv_ctrlr_uninitialize(struct spi_slave_ctrlr_s *ctrlr)
{
  irqstate_t flags;
  struct esp32_spislv_priv_s *priv = (struct esp32_spislv_priv_s *)ctrlr;

  DEBUGASSERT(ctrlr);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->lock);

  if (--priv->refs)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return OK;
    }

  up_disable_irq(priv->config->irq);
  esp32_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
  esp32_spislv_deinit(ctrlr);

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

#endif /* CONFIG_ESP32_SPI */
