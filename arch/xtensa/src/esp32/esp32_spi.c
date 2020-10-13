/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spi.c
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
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "esp32_spi.h"
#include "esp32_gpio.h"
#include "esp32_cpuint.h"
#include "esp32_dma.h"

#include "xtensa.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_pinmap.h"
#include "rom/esp32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI DMA RX/TX description number */

#define SPI_DMADESC_NUM     (CONFIG_SPI_DMADESC_NUM)

/* SPI DMA channel number */

#define SPI_DMA_CHANNEL_MAX (2)

/* SPI DMA reset before exchange */

#define SPI_DMA_RESET_MASK  (SPI_AHBM_RST_M | SPI_AHBM_FIFO_RST_M | \
                             SPI_OUT_RST_M | SPI_IN_RST_M)

/* SPI Default speed (limited by clock divider) */

#define SPI_FREQ_DEFAULT  400000

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */

struct esp32_spi_config_s
{
  uint32_t reg_base;          /* SPI register base address */

  uint32_t clk_freq;          /* SPI clock frequency */
  enum spi_mode_e mode;       /* SPI default mode */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */

  uint8_t cpu;                /* CPU ID */
  uint8_t periph;             /* peripher ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* SPI reset bit */

  bool use_dma;               /* Use DMA */
  uint8_t dma_chan_s;         /* DMA channel regitser shift */
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

struct esp32_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* Port configuration */

  const struct esp32_spi_config_s *config;

  int refs;                    /* Referernce count */

  /* Held while chip is selected for mutual exclusion */

  sem_t            exclsem;

  /* Interrupt wait semaphore */

  sem_t            sem_isr;

  int              cpuint;      /* SPI interrupt ID */

  uint32_t         frequency;   /* Requested clock frequency */
  uint32_t         actual;      /* Actual clock frequency */

  enum spi_mode_e  mode;        /* Actual SPI hardware mode */

  /* Actual SPI send/receive bits once transmission */

  uint8_t          nbits;

  /* Copy from config to speed up checking */

  uint8_t dma_chan;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32_spi_lock(FAR struct spi_dev_s *dev, bool lock);
#ifndef CONFIG_ESP32_SPI_UDCS
static void esp32_spi_select(FAR struct spi_dev_s *dev,
                             uint32_t devid, bool selected);
#endif
static uint32_t esp32_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
static void esp32_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void esp32_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int esp32_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t esp32_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void esp32_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void esp32_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               size_t nwords);
static void esp32_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *rxbuffer,
                                size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int esp32_spi_trigger(FAR struct spi_dev_s *dev);
#endif
static void esp32_spi_init(FAR struct spi_dev_s *dev);
static void esp32_spi_deinit(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2
static const struct esp32_spi_config_s esp32_spi2_config =
{
  .reg_base     = REG_SPI_BASE(2),
  .clk_freq     = SPI_FREQ_DEFAULT,
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI2_CLKPIN,
  .cpu          = 0,
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

static const struct spi_ops_s esp32_spi2_ops =
{
  .lock              = esp32_spi_lock,
#ifdef CONFIG_ESP32_SPI_UDCS
  .select            = esp32_spi2_select,
#else
  .select            = esp32_spi_select,
#endif
  .setfrequency      = esp32_spi_setfrequency,
  .setmode           = esp32_spi_setmode,
  .setbits           = esp32_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32_spi_hwfeatures,
#endif
  .status            = esp32_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32_spi2_cmddata,
#endif
  .send              = esp32_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32_spi_exchange,
#else
  .sndblock          = esp32_spi_sndblock,
  .recvblock         = esp32_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32_spi_priv_s esp32_spi2_priv =
{
  .spi_dev =
              {
                .ops = &esp32_spi2_ops
              },
  .config = &esp32_spi2_config
};
#endif /* CONFIG_ESP32_SPI2 */

#ifdef CONFIG_ESP32_SPI3
static const struct esp32_spi_config_s esp32_spi3_config =
{
  .reg_base     = REG_SPI_BASE(3),
  .clk_freq     = SPI_FREQ_DEFAULT,
  .mode         = SPIDEV_MODE0,
  .cs_pin       = CONFIG_ESP32_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32_SPI3_CLKPIN,
  .cpu          = 0,
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

static const struct spi_ops_s esp32_spi3_ops =
{
  .lock              = esp32_spi_lock,
#ifdef CONFIG_ESP32_SPI_UDCS
  .select            = esp32_spi3_select,
#else
  .select            = esp32_spi_select,
#endif
  .setfrequency      = esp32_spi_setfrequency,
  .setmode           = esp32_spi_setmode,
  .setbits           = esp32_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = esp32_spi_hwfeatures,
#endif
  .status            = esp32_spi3_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = esp32_spi3_cmddata,
#endif
  .send              = esp32_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = esp32_spi_exchange,
#else
  .sndblock          = esp32_spi_sndblock,
  .recvblock         = esp32_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = esp32_spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct esp32_spi_priv_s esp32_spi3_priv =
{
  .spi_dev =
              {
                .ops = &esp32_spi3_ops
              },
  .config = &esp32_spi3_config
};
#endif /* CONFIG_ESP32_SPI3 */

/* SPI DMA RX/TX description */

struct esp32_dmadesc_s s_dma_rxdesc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];
struct esp32_dmadesc_s s_dma_txdesc[SPI_DMA_CHANNEL_MAX][SPI_DMADESC_NUM];

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

static inline void esp32_spi_set_reg(struct esp32_spi_priv_s *priv,
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

static inline uint32_t esp32_spi_get_reg(struct esp32_spi_priv_s *priv,
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

static inline void esp32_spi_set_regbits(struct esp32_spi_priv_s *priv,
                                         int offset, uint32_t bits)
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

static inline void esp32_spi_reset_regbits(struct esp32_spi_priv_s *priv,
                                           int offset, uint32_t bits)
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

static inline bool esp32_spi_iomux(struct esp32_spi_priv_s *priv)
{
  bool mapped = false;
  const struct esp32_spi_config_s *cfg = priv->config;

  if (REG_SPI_BASE(2) == cfg->reg_base)
    {
      if (cfg->mosi_pin == SPI2_IOMUX_MOSIPIN &&
#ifndef CONFIG_ESP32_SPI_SWCS
          cfg->cs_pin == SPI2_IOMUX_CSPIN &&
#endif
          cfg->miso_pin == SPI2_IOMUX_MISOPIN &&
          cfg->clk_pin == SPI2_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }
  else if (REG_SPI_BASE(3) == cfg->reg_base)
    {
      if (cfg->mosi_pin == SPI3_IOMUX_MOSIPIN &&
#ifndef CONFIG_ESP32_SPI_SWCS
          cfg->cs_pin == SPI3_IOMUX_CSPIN &&
#endif
          cfg->miso_pin == SPI3_IOMUX_MISOPIN &&
          cfg->clk_pin == SPI3_IOMUX_CLKPIN)
        {
          mapped = true;
        }
    }

  return mapped;
}

/****************************************************************************
 * Name: esp32_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   lock   - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device
 *
 ****************************************************************************/

static int esp32_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  int ret;
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_spi_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

static int esp32_spi_sem_waitdone(FAR struct esp32_spi_priv_s *priv)
{
  int ret;
  struct timespec abstime;

  clock_gettime(CLOCK_REALTIME, &abstime);

  abstime.tv_sec += 10;
  abstime.tv_nsec += 0;

  ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);

  return ret;
}

/****************************************************************************
 * Name: esp32_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If disable ESP32_SPI_SWCS, driver will use hardware CS so that when
 *   once transmission is started, hardware select the device and when this
 *   transmission is done, hardware deselect the device automatically. And
 *   the function will do nothing.
 *
 * Input Parameters:
 *   priv     - Private SPI device structure
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ESP32_SPI_UDCS
static void esp32_spi_select(FAR struct spi_dev_s *dev,
                             uint32_t devid, bool selected)
{
#ifdef CONFIG_ESP32_SPI_SWCS
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;
  bool value = selected ? false : true;

  esp32_gpiowrite(priv->config->cs_pin, value);
#endif

  spiinfo("devid: %08lx CS: %s\n", devid, selected ? "select" : "free");
}
#endif

/****************************************************************************
 * Name: esp32_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t esp32_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  uint32_t reg_val;
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;
  const uint32_t duty_cycle = 128;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */

      return priv->actual;
    }

  if (frequency > ((APB_CLK_FREQ / 4) * 3))
    {
      reg_val = SPI_CLK_EQU_SYSCLK_M;
      priv->actual = APB_CLK_FREQ;
    }
  else
    {
      int pre;
      int n;
      int h;
      int l;
      int bestn = -1;
      int bestpre = -1;
      int besterr = 0;
      int errval;

      for (n = 2; n <= 64; n++)
        {
          pre = ((APB_CLK_FREQ / n) + (frequency / 2)) / frequency;

          if (pre <= 0)
            {
              pre = 1;
            }

          if (pre > 8192)
            {
              pre = 8192;
            }

          errval = abs(APB_CLK_FREQ / (pre * n) - frequency);
          if (bestn == -1 || errval <= besterr)
            {
              besterr = errval;
              bestn = n;
              bestpre = pre;
            }
        }

      n = bestn;
      pre = bestpre;
      l = n;
      h = (duty_cycle * n + 127) / 256;
      if (h <= 0)
        {
          h = 1;
        }

      reg_val = ((l - 1) << SPI_CLKCNT_L_S) |
                ((h - 1) << SPI_CLKCNT_H_S) |
                ((n - 1) << SPI_CLKCNT_N_S) |
                ((pre - 1) << SPI_CLKDIV_PRE_S);

      priv->actual = APB_CLK_FREQ / (n * pre);
    }

  priv->frequency = frequency;

  esp32_spi_set_reg(priv, SPI_CLOCK_OFFSET, reg_val);

  spiinfo("frequency=%d, actual=%d\n", priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: esp32_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  uint32_t ck_idle_edge;
  uint32_t ck_out_edge;
  uint32_t delay_mode;
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          ck_idle_edge = 0;
          ck_out_edge = 0;
          delay_mode = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          ck_idle_edge = 0;
          ck_out_edge = 1;
          delay_mode = 2;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          ck_idle_edge = 1;
          ck_out_edge = 1;
          delay_mode = 2;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          ck_idle_edge = 1;
          ck_out_edge = 0;
          delay_mode = 0;
          break;

        default:
          return;
        }

      esp32_spi_reset_regbits(priv, SPI_PIN_OFFSET, SPI_CK_IDLE_EDGE_M);
      esp32_spi_set_regbits(priv, SPI_PIN_OFFSET,
                            (ck_idle_edge << SPI_CK_IDLE_EDGE_S));

      esp32_spi_reset_regbits(priv, SPI_USER_OFFSET,
                              SPI_CK_OUT_EDGE_M);
      esp32_spi_set_regbits(priv, SPI_USER_OFFSET,
                            (ck_out_edge << SPI_CK_OUT_EDGE_S));

      esp32_spi_reset_regbits(priv, SPI_CTRL2_OFFSET,
                              SPI_MISO_DELAY_MODE_M |
                              SPI_MISO_DELAY_NUM_M |
                              SPI_MOSI_DELAY_NUM_M |
                              SPI_MOSI_DELAY_MODE_M);

      esp32_spi_set_regbits(priv, SPI_CTRL2_OFFSET,
                            (delay_mode << SPI_MISO_DELAY_MODE_S) |
                            (delay_mode << SPI_MOSI_DELAY_MODE_S));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /**
       * Save the selection so the subsequence re-configurations
       * will be faster.
       */

      priv->nbits = nbits;

      /**
       * Each DMA transmission will set these value according to
       * calculate buffer length.
       */

      if (!priv->dma_chan)
        {
          esp32_spi_set_reg(priv, SPI_MISO_DLEN_OFFSET,
                            (priv->nbits - 1) << SPI_USR_MISO_DBITLEN_S);
          esp32_spi_set_reg(priv, SPI_MOSI_DLEN_OFFSET,
                            (priv->nbits - 1) << SPI_USR_MOSI_DBITLEN_S);
        }
    }
}

/****************************************************************************
 * Name: esp32_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int esp32_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32_spi_dma_exchange
 *
 * Description:
 *   Exchange a block of data from SPI by DMA.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_dma_exchange(FAR struct esp32_spi_priv_s *priv,
                                   FAR const void *txbuffer,
                                   FAR void *rxbuffer,
                                   uint32_t nwords)
{
  uint32_t bytes = nwords * (priv->nbits / 8);
  uint8_t *tp;
  uint8_t *rp;
  uint32_t n;
  uint32_t regval;
#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  uint8_t *alloctp;
  uint8_t *allocrp;
#endif

  /* If the buffer comes from PSRAM, allocate a new one from DRAM */

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(txbuffer))
    {
      alloctp = xtensa_imm_malloc(bytes);
      DEBUGASSERT(alloctp != NULL);
      memcpy(alloctp, txbuffer, bytes);
      tp = alloctp;
    }
  else
#endif
    {
      tp = (uint8_t *)txbuffer;
    }

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(rxbuffer))
    {
      allocrp = xtensa_imm_malloc(bytes);
      DEBUGASSERT(allocrp != NULL);
      rp = allocrp;
    }
  else
#endif
    {
      rp = (uint8_t *)rxbuffer;
    }

  if (!tp)
    {
      tp = rp;
    }

  while (bytes)
    {
      esp32_spi_set_reg(priv, SPI_DMA_IN_LINK_OFFSET, 0);
      esp32_spi_set_reg(priv, SPI_DMA_OUT_LINK_OFFSET, 0);

      esp32_spi_set_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);
      esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_SYNC_RESET_M);

      esp32_spi_set_reg(priv, SPI_DMA_CONF_OFFSET, SPI_DMA_RESET_MASK);
      esp32_spi_reset_regbits(priv, SPI_DMA_CONF_OFFSET, SPI_DMA_RESET_MASK);

      n = esp32_dma_init(s_dma_txdesc[priv->dma_chan - 1],
                         SPI_DMADESC_NUM, tp, bytes, 0);

      regval = (uint32_t)s_dma_txdesc[priv->dma_chan - 1] &
               SPI_OUTLINK_ADDR_V;
      esp32_spi_set_reg(priv, SPI_DMA_OUT_LINK_OFFSET,
                        regval | SPI_OUTLINK_START_M);
      esp32_spi_set_reg(priv, SPI_MOSI_DLEN_OFFSET, bytes * 8 - 1);
      if (tp)
        {
          esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MOSI_M);
        }
      else
        {
          esp32_spi_reset_regbits(priv, SPI_USER_OFFSET, SPI_USR_MOSI_M);
        }

      if (rp)
        {
          esp32_dma_init(s_dma_rxdesc[priv->dma_chan - 1],
                         SPI_DMADESC_NUM, rp, bytes, 1);

          regval = (uint32_t)s_dma_rxdesc[priv->dma_chan - 1] &
                   SPI_INLINK_ADDR_V;
          esp32_spi_set_reg(priv, SPI_DMA_IN_LINK_OFFSET,
                            regval | SPI_INLINK_START_M);
          esp32_spi_set_reg(priv, SPI_MISO_DLEN_OFFSET, bytes * 8 - 1);
          esp32_spi_set_regbits(priv, SPI_USER_OFFSET, SPI_USR_MISO_M);
        }
      else
        {
          esp32_spi_reset_regbits(priv, SPI_USER_OFFSET, SPI_USR_MISO_M);
        }

      esp32_spi_set_regbits(priv, SPI_CMD_OFFSET, SPI_USR_M);

      esp32_spi_sem_waitdone(priv);

      bytes -= n;
      tp += n;
      rp += n;
    }

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(rxbuffer))
    {
      memcpy(rxbuffer, allocrp, bytes);
      xtensa_imm_free(allocrp);
    }
#endif

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(txbuffer))
    {
      xtensa_imm_free(alloctp);
    }
#endif
}

/****************************************************************************
 * Name: esp32_spi_poll_send
 *
 * Description:
 *   Exchange one word on SPI by polling mode.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t esp32_spi_poll_send(FAR struct esp32_spi_priv_s *priv,
                                    uint32_t wd)
{
  uint32_t val;

  esp32_spi_set_reg(priv, SPI_W0_OFFSET, wd);

  esp32_spi_set_regbits(priv, SPI_CMD_OFFSET, SPI_USR_M);

  while (esp32_spi_get_reg(priv, SPI_CMD_OFFSET) & SPI_USR_M)
    {
      ;
    }

  val = esp32_spi_get_reg(priv, SPI_W0_OFFSET);

  spiinfo("send=%x and recv=%x\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: esp32_spi_dma_send
 *
 * Description:
 *   Exchange one word on SPI by SPI DMA mode.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t esp32_spi_dma_send(FAR struct esp32_spi_priv_s *priv,
                                   uint32_t wd)
{
  uint32_t rd;

  esp32_spi_dma_exchange(priv, &wd, &rd, 1);

  return rd;
}

/****************************************************************************
 * Name: esp32_spi_send
 *
 * Description:
 *   Exchange one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t esp32_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;
  uint32_t rd;

  if (priv->dma_chan)
    {
      rd = esp32_spi_dma_send(priv, wd);
    }
  else
    {
      rd = esp32_spi_poll_send(priv, wd);
    }

  return rd;
}

/****************************************************************************
 * Name: esp32_spi_poll_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_poll_exchange(FAR struct esp32_spi_priv_s *priv,
                                    FAR const void *txbuffer,
                                    FAR void *rxbuffer,
                                    size_t nwords)
{
  int i;

  for (i = 0 ; i < nwords; i++)
    {
      uint32_t w_wd = 0xffff;
      uint32_t r_wd;

      if (txbuffer)
        {
          if (priv->nbits == 8)
            {
              w_wd = ((uint8_t *)txbuffer)[i];
            }
          else
            {
              w_wd = ((uint16_t *)txbuffer)[i];
            }
        }

      r_wd = esp32_spi_poll_send(priv, w_wd);

      if (rxbuffer)
        {
          if (priv->nbits == 8)
            {
              ((uint8_t *)rxbuffer)[i] = r_wd;
            }
          else
            {
              ((uint16_t *)rxbuffer)[i] = r_wd;
            }
        }
    }
}

/****************************************************************************
 * Name: esp32_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer,
                               size_t nwords)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  if (priv->dma_chan)
    {
      esp32_spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
    {
      esp32_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: esp32_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  esp32_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: esp32_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *rxbuffer,
                                size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  esp32_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: esp32_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int esp32_spi_trigger(FAR struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp32_spi_init
 *
 * Description:
 *   Initialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_init(FAR struct spi_dev_s *dev)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;
  const struct esp32_spi_config_s *config = priv->config;

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  esp32_gpiowrite(config->cs_pin, 1);
  esp32_gpiowrite(config->mosi_pin, 1);
  esp32_gpiowrite(config->miso_pin, 1);
  esp32_gpiowrite(config->clk_pin, 1);

#ifdef CONFIG_ESP32_SPI_SWCS
  esp32_configgpio(config->cs_pin, OUTPUT);
  gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif

  if (esp32_spi_iomux(priv))
    {
#ifndef CONFIG_ESP32_SPI_SWCS
      esp32_configgpio(config->cs_pin, OUTPUT_FUNCTION_2);
      gpio_matrix_out(config->cs_pin, SIG_GPIO_OUT_IDX, 0, 0);
#endif
      esp32_configgpio(config->mosi_pin, OUTPUT_FUNCTION_2);
      gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);

      esp32_configgpio(config->miso_pin, INPUT_FUNCTION_2 | PULLUP);
      gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);

      esp32_configgpio(config->clk_pin, OUTPUT_FUNCTION_2);
      gpio_matrix_out(config->clk_pin, SIG_GPIO_OUT_IDX, 0, 0);
    }
  else
    {
#ifndef CONFIG_ESP32_SPI_SWCS
      esp32_configgpio(config->cs_pin, OUTPUT_FUNCTION_3);
      gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);
#endif

      esp32_configgpio(config->mosi_pin, OUTPUT_FUNCTION_3);
      gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);

      esp32_configgpio(config->miso_pin, INPUT_FUNCTION_3 | PULLUP);
      gpio_matrix_in(config->miso_pin, config->miso_insig, 0);

      esp32_configgpio(config->clk_pin, OUTPUT_FUNCTION_3);
      gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);
    }

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
  modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);

  esp32_spi_set_reg(priv, SPI_USER_OFFSET, SPI_DOUTDIN_M |
                                           SPI_USR_MISO_M |
                                           SPI_USR_MOSI_M |
                                           SPI_CS_HOLD_M);
  esp32_spi_set_reg(priv, SPI_USER1_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_SLAVE_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_PIN_OFFSET, SPI_CS1_DIS_M | SPI_CS2_DIS_M);
#ifdef CONFIG_ESP32_SPI_SWCS
  esp32_spi_set_regbits(priv, SPI_PIN_OFFSET, SPI_CS0_DIS_M);
#endif
  esp32_spi_set_reg(priv, SPI_CTRL_OFFSET, 0);
  esp32_spi_set_reg(priv, SPI_CTRL2_OFFSET, (0 << SPI_HOLD_TIME_S));

  if (priv->dma_chan)
    {
      nxsem_init(&priv->sem_isr, 0, 0);
      nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);

      modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->dma_clk_bit);
      modifyreg32(DPORT_PERIP_RST_EN_REG, config->dma_rst_bit, 0);

      modifyreg32(DPORT_SPI_DMA_CHAN_SEL_REG, 0,
                  (config->dma_chan << config->dma_chan_s));

      esp32_spi_set_reg(priv, SPI_DMA_CONF_OFFSET, SPI_OUT_DATA_BURST_EN_M |
                                                   SPI_INDSCR_BURST_EN_M |
                                                   SPI_OUTDSCR_BURST_EN_M);

      esp32_spi_set_regbits(priv, SPI_SLAVE_OFFSET, SPI_INT_EN_M);
    }

  esp32_spi_setfrequency(dev, config->clk_freq);
  esp32_spi_setbits(dev, 8);
  esp32_spi_setmode(dev, config->mode);
}

/****************************************************************************
 * Name: esp32_spi_deinit
 *
 * Description:
 *   Deinitialize ESP32 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_spi_deinit(FAR struct spi_dev_s *dev)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  if (priv->dma_chan)
    {
      modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->dma_clk_bit, 0);
    }

  modifyreg32(DPORT_PERIP_CLK_EN_REG, priv->config->clk_bit, 0);
}

/****************************************************************************
 * Name: esp32_spi_interrupt
 *
 * Description:
 *   Common SPI DMA interrupt handler
 *
 * Input Parameters:
 *   arg - SPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int esp32_spi_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)arg;

  esp32_spi_reset_regbits(priv, SPI_SLAVE_OFFSET, SPI_TRANS_DONE_M);
  nxsem_post(&priv->sem_isr);

  return 0;
}

/****************************************************************************
 * Name: esp32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *esp32_spibus_initialize(int port)
{
  int ret;
  FAR struct spi_dev_s *spi_dev;
  FAR struct esp32_spi_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_ESP32_SPI2
      case 2:
        priv = &esp32_spi2_priv;
        break;
#endif
#ifdef CONFIG_ESP32_SPI3
      case 3:
        priv = &esp32_spi3_priv;
        break;
#endif
      default:
        return NULL;
    }

  spi_dev = (FAR struct spi_dev_s *)priv;

  flags = enter_critical_section();

  if ((volatile int)priv->refs != 0)
    {
      leave_critical_section(flags);

      return spi_dev;
    }

  if (priv->config->use_dma)
    {
      priv->dma_chan = priv->config->dma_chan;
    }
  else
    {
      priv->dma_chan = 0;
    }

  if (priv->dma_chan)
    {
      priv->cpuint = esp32_alloc_levelint(1);
      if (priv->cpuint < 0)
        {
          leave_critical_section(flags);
          return NULL;
        }

      up_disable_irq(priv->cpuint);
      esp32_attach_peripheral(priv->config->cpu,
                              priv->config->periph,
                              priv->cpuint);
      ret = irq_attach(priv->config->irq, esp32_spi_interrupt, priv);
      if (ret != OK)
        {
          esp32_detach_peripheral(priv->config->cpu,
                                  priv->config->periph,
                                  priv->cpuint);
          esp32_free_cpuint(priv->cpuint);

          leave_critical_section(flags);
          return NULL;
        }

      up_enable_irq(priv->cpuint);
    }

  esp32_spi_init(spi_dev);

  priv->refs++;

  leave_critical_section(flags);

  return spi_dev;
}

/****************************************************************************
 * Name: esp32_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int esp32_spibus_uninitialize(FAR struct spi_dev_s *dev)
{
  irqstate_t flags;
  FAR struct esp32_spi_priv_s *priv = (FAR struct esp32_spi_priv_s *)dev;

  DEBUGASSERT(dev);

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

  leave_critical_section(flags);

  if (priv->dma_chan)
    {
      up_disable_irq(priv->cpuint);
      esp32_detach_peripheral(priv->config->cpu,
                              priv->config->periph,
                              priv->cpuint);
      esp32_free_cpuint(priv->cpuint);

      nxsem_destroy(&priv->sem_isr);
    }

  esp32_spi_deinit(dev);

  nxsem_destroy(&priv->exclsem);

  return OK;
}

#endif /* CONFIG_ESP32_SPI */
