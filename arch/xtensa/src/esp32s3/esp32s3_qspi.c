/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_qspi.c
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

#ifdef CONFIG_ESP32S3_SPI

#include <assert.h>
#include <debug.h>
#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/qspi.h>

#include <arch/board/board.h>

#include "xtensa.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_spi.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"

#include "esp32s3_irq.h"
#include "esp32s3_gpio.h"
#include "esp32s3_qspi.h"

#ifdef CONFIG_ESP32S3_SPI_DMA
#include "esp32s3_dma.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_DMA

/* QSPI maximum DMA buffer size in bytes */

#define QSPI_DMA_BUFSIZE CONFIG_ESP32S3_SPI_DMA_BUFSIZE

/* QSPI DMA RX/TX number of descriptors */

#  if (QSPI_DMA_BUFSIZE % ESP32S3_DMA_BUFLEN_MAX) > 0
#    define QSPI_DMA_DESC_NUM (QSPI_DMA_BUFSIZE / ESP32S3_DMA_BUFLEN_MAX + 1)
#  else
#    define QSPI_DMA_DESC_NUM (QSPI_DMA_BUFSIZE / ESP32S3_DMA_BUFLEN_MAX)
#  endif

/* QSPI DMA reset before exchange */

#  define QSPI_DMA_RESET_MASK (SPI_DMA_AFIFO_RST_M | SPI_RX_AFIFO_RST_M)

#endif /* CONFIG_ESP32S3_SPI_DMA */

/* QSPI default frequency (limited by clock divider) */

#define QSPI_DEFAULT_FREQ   (400000)

/* QSPI default width */

#define QSPI_DEFAULT_WIDTH  (8)

/* QSPI default mode */

#define QSPI_DEFAULT_MODE   (QSPIDEV_MODE0)

/* QSPI maximum non-DMA buffer size in bytes */

#define QSPI_CMD_BUFSIZE    (64)

/* Verify whether QSPI has been assigned IOMUX pins.
 * Otherwise, QSPI signals will be routed via GPIO Matrix.
 */

#ifdef CONFIG_ESP32S3_SPI2

/* In quad QSPI mode, data IO map is:
 *    MOSI -> IO0
 *    MISO -> IO1
 *    WP   -> IO2
 *    Hold -> IO3
 */

#  define QSPI_IS_CS_IOMUX    (CONFIG_ESP32S3_SPI2_CSPIN == SPI2_IOMUX_CSPIN)
#  define QSPI_IS_CLK_IOMUX   (CONFIG_ESP32S3_SPI2_CLKPIN == SPI2_IOMUX_CLKPIN)
#  define QSPI_IS_MOSI_IOMUX  (CONFIG_ESP32S3_SPI2_MOSIPIN == SPI2_IOMUX_MOSIPIN)
#  define QSPI_IS_MISO_IOMUX  (CONFIG_ESP32S3_SPI2_MISOPIN == SPI2_IOMUX_MISOPIN)
#  define QSPI_IS_IO2_IOMUX   (CONFIG_ESP32S3_SPI2_IO2PIN == SPI2_IOMUX_WPPIN)
#  define QSPI_IS_IO3_IOMUX   (CONFIG_ESP32S3_SPI2_IO3PIN == SPI2_IOMUX_HDPIN)

#  define QSPI_VIA_IOMUX      ((QSPI_IS_CS_IOMUX) && \
                               (QSPI_IS_CLK_IOMUX) && \
                               (QSPI_IS_MOSI_IOMUX) && \
                               (QSPI_IS_MISO_IOMUX) && \
                               (QSPI_IS_IO2_IOMUX) && \
                               (QSPI_IS_IO3_IOMUX))
#else
#  define QSPI_VIA_IOMUX      0
#endif

/* Check if 16-bit command or 8 bit command and return its bits */

#define QSPI_CMD_BITS(cmd)    ((cmd) & 0xff00 ? 16 : 8)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* QSPI Device hardware configuration */

struct esp32s3_qspi_config_s
{
  uint8_t id;                 /* ESP32-S3 QSPI device ID: SPIx {2,3} */

  uint8_t cs_pin;             /* GPIO configuration for CS */
  uint8_t mosi_pin;           /* GPIO configuration for MOSI */
  uint8_t miso_pin;           /* GPIO configuration for MISO */
  uint8_t clk_pin;            /* GPIO configuration for CLK */
  uint8_t io2_pin;            /* GPIO configuration for IO2 */
  uint8_t io3_pin;            /* GPIO configuration for IO3 */

  int8_t periph;              /* Peripheral ID */
  uint8_t irq;                /* Interrupt ID */

  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t rst_bit;           /* QSPI reset bit */

#ifdef CONFIG_ESP32S3_SPI_DMA

  /* Peripheral for which the DMA channel request */

  enum esp32s3_dma_periph_e dma_periph;
  uint32_t dma_clk_bit;       /* DMA clock enable bit */
  uint32_t dma_rst_bit;       /* DMA reset bit */
#endif

  uint32_t cs_insig;          /* QSPI CS input signal index */
  uint32_t cs_outsig;         /* QSPI CS output signal index */
  uint32_t mosi_insig;        /* QSPI MOSI input signal index */
  uint32_t mosi_outsig;       /* QSPI MOSI output signal index */
  uint32_t miso_insig;        /* QSPI MISO input signal index */
  uint32_t miso_outsig;       /* QSPI MISO output signal index */
  uint32_t clk_insig;         /* QSPI CLK input signal index */
  uint32_t clk_outsig;        /* QSPI CLK output signal index */
  uint32_t io2_insig;         /* QSPI IO2 input signal index */
  uint32_t io2_outsig;        /* QSPI IO2 output signal index */
  uint32_t io3_insig;         /* QSPI IO3 input signal index */
  uint32_t io3_outsig;        /* QSPI IO3 output signal index */
};

struct esp32s3_qspi_priv_s
{
  /* Externally visible part of the (Q)QSPI interface */

  struct qspi_dev_s spi_dev;

  /* Port configuration */

  const struct esp32s3_qspi_config_s *config;

  int refs;               /* Reference count */
  mutex_t lock;           /* Held while chip is selected for mutual exclusion */

#ifdef CONFIG_ESP32S3_SPI_DMA
  sem_t sem_isr;          /* Interrupt wait semaphore */

  int cpu;                /* CPU ID */
  int cpuint;             /* QSPI interrupt ID */

  int32_t dma_channel;    /* Channel assigned by the GDMA driver */

  /* DMA description */

  struct esp32s3_dmadesc_s *dma_desc;
#endif

  uint32_t frequency;     /* Requested clock frequency */
  uint32_t actual;        /* Actual clock frequency */
  enum qspi_mode_e mode;  /* Actual QSPI hardware mode */
  uint8_t nbits;          /* Actual QSPI send/receive bits once transmission */

  uint8_t dummies;        /* Number of dummy cycles of command transfer */
  uint8_t addr_lines;     /* Number of address transmiting I/O pins */
  uint8_t data_lines;     /* Number of data transmiting I/O pins */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t esp32s3_qspi_setfrequency(struct qspi_dev_s *dev,
                                          uint32_t frequency);
static void esp32s3_qspi_setmode(struct qspi_dev_s *dev,
                                 enum qspi_mode_e mode);
static void esp32s3_qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int esp32s3_qspi_command(struct qspi_dev_s *dev,
                                struct qspi_cmdinfo_s *cmdinfo);
static int esp32s3_qspi_memory(struct qspi_dev_s *dev,
                               struct qspi_meminfo_s *meminfo);
static void *esp32s3_qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void esp32s3_qspi_free(struct qspi_dev_s *dev, void *buffer);

#ifdef CONFIG_ESP32S3_SPI_DMA
static int esp32s3_qspi_interrupt(int irq, void *context, void *arg);
static int esp32s3_qspi_wait_sem(struct esp32s3_qspi_priv_s *priv);
static void esp32s3_qspi_init_dma(struct esp32s3_qspi_priv_s *priv);
#endif
static void esp32s3_qspi_init(struct esp32s3_qspi_priv_s *priv);
static void esp32s3_qspi_deinit(struct esp32s3_qspi_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2
static const struct esp32s3_qspi_config_s esp32s3_spi2_config =
{
  .id           = 2,

  .cs_pin       = CONFIG_ESP32S3_SPI2_CSPIN,
  .mosi_pin     = CONFIG_ESP32S3_SPI2_MOSIPIN,
  .miso_pin     = CONFIG_ESP32S3_SPI2_MISOPIN,
  .clk_pin      = CONFIG_ESP32S3_SPI2_CLKPIN,
  .io2_pin      = CONFIG_ESP32S3_SPI2_IO2PIN,
  .io3_pin      = CONFIG_ESP32S3_SPI2_IO3PIN,

  .periph       = ESP32S3_PERIPH_SPI2,
  .irq          = ESP32S3_IRQ_SPI2,

  .clk_bit      = SYSTEM_SPI2_CLK_EN,
  .rst_bit      = SYSTEM_SPI2_RST,

#ifdef CONFIG_ESP32S3_SPI_DMA
  .dma_periph   = ESP32S3_DMA_PERIPH_SPI2,
  .dma_clk_bit  = SYSTEM_SPI2_DMA_CLK_EN,
  .dma_rst_bit  = SYSTEM_SPI2_DMA_RST,
#endif

  .cs_insig     = FSPICS0_IN_IDX,
  .cs_outsig    = FSPICS0_OUT_IDX,
  .mosi_insig   = FSPID_IN_IDX,
  .mosi_outsig  = FSPID_OUT_IDX,
  .miso_insig   = FSPIQ_IN_IDX,
  .miso_outsig  = FSPIQ_OUT_IDX,
  .clk_insig    = FSPICLK_IN_IDX,
  .clk_outsig   = FSPICLK_OUT_IDX,
  .io2_insig    = FSPIWP_IN_IDX,
  .io2_outsig   = FSPIWP_OUT_IDX,
  .io3_insig    = FSPIHD_IN_IDX,
  .io3_outsig   = FSPIHD_OUT_IDX,
};

static const struct qspi_ops_s esp32s3_spi2_ops =
{
  .lock         = esp32s3_qspi_lock,
  .setfrequency = esp32s3_qspi_setfrequency,
  .setmode      = esp32s3_qspi_setmode,
  .setbits      = esp32s3_qspi_setbits,
  .command      = esp32s3_qspi_command,
  .memory       = esp32s3_qspi_memory,
  .alloc        = esp32s3_qspi_alloc,
  .free         = esp32s3_qspi_free,
};

#  ifdef CONFIG_ESP32S3_SPI_DMA

/* QSPI 2 DMA RX/TX description */

static struct esp32s3_dmadesc_s esp32s3_spi2_dma_desc[QSPI_DMA_DESC_NUM];
#  endif

static struct esp32s3_qspi_priv_s esp32s3_spi2_priv =
{
  .spi_dev     =
    {
      .ops     = &esp32s3_spi2_ops
    },

  .config      = &esp32s3_spi2_config,

  .refs        = 0,
  .lock        = NXMUTEX_INITIALIZER,

#ifdef CONFIG_ESP32S3_SPI_DMA
  .sem_isr     = SEM_INITIALIZER(0),

  .cpu         = 0,
  .cpuint      = -ENOMEM,

  .dma_channel = -1,
  .dma_desc    = esp32s3_spi2_dma_desc,
#endif

  .frequency   = 0,
  .actual      = 0,
  .mode        = QSPIDEV_MODE0,
  .nbits       = 0,

  .dummies     = 0,
  .addr_lines  = 4,
  .data_lines  = 4,
};
#endif /* CONFIG_ESP32S3_SPI2 */

#ifdef CONFIG_ESP32S3_SPI3
static const struct esp32s3_qspi_config_s esp32s3_spi3_config =
{
  .id           = 3,

  .cs_pin       = CONFIG_ESP32S3_SPI3_CSPIN,
  .mosi_pin     = CONFIG_ESP32S3_SPI3_MOSIPIN,
  .miso_pin     = CONFIG_ESP32S3_SPI3_MISOPIN,
  .clk_pin      = CONFIG_ESP32S3_SPI3_CLKPIN,
  .io2_pin      = CONFIG_ESP32S3_SPI3_IO2PIN,
  .io3_pin      = CONFIG_ESP32S3_SPI3_IO3PIN,

  .periph       = ESP32S3_PERIPH_SPI3,
  .irq          = ESP32S3_IRQ_SPI3,

  .clk_bit      = SYSTEM_SPI3_CLK_EN,
  .rst_bit      = SYSTEM_SPI3_RST,

#ifdef CONFIG_ESP32S3_SPI_DMA
  .dma_periph   = ESP32S3_DMA_PERIPH_SPI3,
  .dma_clk_bit  = SYSTEM_SPI3_DMA_CLK_EN,
  .dma_rst_bit  = SYSTEM_SPI3_DMA_RST,
#endif

  .cs_insig     = SPI3_CS0_IN_IDX,
  .cs_outsig    = SPI3_CS0_OUT_IDX,
  .mosi_insig   = SPI3_D_IN_IDX,
  .mosi_outsig  = SPI3_D_OUT_IDX,
  .miso_insig   = SPI3_Q_IN_IDX,
  .miso_outsig  = SPI3_Q_OUT_IDX,
  .clk_insig    = SPI3_CLK_IN_IDX,
  .clk_outsig   = SPI3_CLK_OUT_IDX,
  .io2_insig    = SPI3_WP_IN_IDX,
  .io2_outsig   = SPI3_WP_OUT_IDX,
  .io3_insig    = SPI3_HD_IN_IDX,
  .io3_outsig   = SPI3_HD_OUT_IDX,
};

static const struct qspi_ops_s esp32s3_spi3_ops =
{
  .lock         = esp32s3_qspi_lock,
  .setfrequency = esp32s3_qspi_setfrequency,
  .setmode      = esp32s3_qspi_setmode,
  .setbits      = esp32s3_qspi_setbits,
  .command      = esp32s3_qspi_command,
  .memory       = esp32s3_qspi_memory,
  .alloc        = esp32s3_qspi_alloc,
  .free         = esp32s3_qspi_free,
};

#  ifdef CONFIG_ESP32S3_SPI_DMA

/* QSPI 3 DMA description */

static struct esp32s3_dmadesc_s esp32s3_spi3_dma_desc[QSPI_DMA_DESC_NUM];
#  endif

static struct esp32s3_qspi_priv_s esp32s3_spi3_priv =
{
  .spi_dev     =
    {
      .ops     = &esp32s3_spi3_ops
    },

  .config      = &esp32s3_spi3_config,

  .refs        = 0,
  .lock        = NXMUTEX_INITIALIZER,

#ifdef CONFIG_ESP32S3_SPI_DMA
  .sem_isr     = SEM_INITIALIZER(0),

  .cpu         = 0,
  .cpuint      = -ENOMEM,

  .dma_channel = -1,
  .dma_desc    = esp32s3_spi3_dma_desc,
#endif

  .frequency   = 0,
  .actual      = 0,
  .mode        = QSPIDEV_MODE0,
  .nbits       = 0,

  .dummies     = 0,
  .addr_lines  = 4,
  .data_lines  = 4,
};
#endif /* CONFIG_ESP32S3_SPI3 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_qspi_lock
 *
 * Description:
 *   Lock or unlock the QSPI device.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock QSPI bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the QSPI device.
 *
 ****************************************************************************/

static int esp32s3_qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  int ret;
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_qspi_setfrequency
 *
 * Description:
 *   Set the QSPI frequency.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The requested QSPI frequency
 *
 * Returned Value:
 *   Returns the current selected frequency.
 *
 ****************************************************************************/

static uint32_t esp32s3_qspi_setfrequency(struct qspi_dev_s *dev,
                                          uint32_t frequency)
{
  uint32_t regval;
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;
  const uint32_t duty_cycle = 128;

  if (priv->frequency == frequency)
    {
      /* Requested frequency is the same as the current frequency. */

      return priv->actual;
    }

  /* In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
   * The value written to register is one lower than the used value.
   */

  if (frequency > ((APB_CLK_FREQ / 4) * 3))
    {
      /* Using APB frequency directly will give us the best result here. */

      regval = SPI_CLK_EQU_SYSCLK_M;
      priv->actual = APB_CLK_FREQ;
    }
  else
    {
      /* For best duty cycle resolution, we want n to be as close to 32 as
       * possible, but we also need a pre/n combo that gets us as close as
       * possible to the intended frequency. To do this, we bruteforce n and
       * calculate the best pre to go along with that. If there's a choice
       * between pre/n combos that give the same result, use the one with the
       * higher n.
       */

      int32_t pre;
      int32_t n;
      int32_t h;
      int32_t l;
      int32_t bestn = -1;
      int32_t bestpre = -1;
      int32_t besterr = 0;
      int32_t errval;

      /* Start at n = 2. We need to be able to set h/l so we have at least
       * one high and one low pulse.
       */

      for (n = 2; n <= 64; n++)
        {
          /* Effectively, this does:
           *   pre = round((APB_CLK_FREQ / n) / frequency)
           */

          pre = ((APB_CLK_FREQ / n) + (frequency / 2)) / frequency;

          if (pre <= 0)
            {
              pre = 1;
            }

          if (pre > 16)
            {
              pre = 16;
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

      /* Effectively, this does:
       *   h = round((duty_cycle * n) / 256)
       */

      h = (duty_cycle * n + 127) / 256;
      if (h <= 0)
        {
          h = 1;
        }

      regval = ((l - 1) << SPI_CLKCNT_L_S) |
               ((h - 1) << SPI_CLKCNT_H_S) |
               ((n - 1) << SPI_CLKCNT_N_S) |
               ((pre - 1) << SPI_CLKDIV_PRE_S);

      priv->actual = APB_CLK_FREQ / (n * pre);
    }

  priv->frequency = frequency;

  putreg32(regval, SPI_CLOCK_REG(priv->config->id));

  spiinfo("frequency=%" PRIu32 ", actual=%" PRIu32 "\n",
          priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name:  esp32s3_qspi_setmode
 *
 * Description:
 *   Set the QSPI mode.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The requested QSPI mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_setmode(struct qspi_dev_s *dev,
                                 enum qspi_mode_e mode)
{
  uint32_t regval;
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;
  uint8_t id = priv->config->id;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      uint32_t ck_idle_edge;
      uint32_t ck_out_edge;

      switch (mode)
        {
          case QSPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            ck_idle_edge = 0;
            ck_out_edge = 0;
            break;

          case QSPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            ck_idle_edge = 0;
            ck_out_edge = 1;
            break;

          case QSPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            ck_idle_edge = 1;
            ck_out_edge = 1;
            break;

          case QSPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            ck_idle_edge = 1;
            ck_out_edge = 0;
            break;

          default:
            spierr("Invalid mode: %d\n", mode);
            DEBUGPANIC();
            return;
        }

      regval  = getreg32(SPI_MISC_REG(id));
      regval &= SPI_CK_IDLE_EDGE_M;
      regval |= ck_idle_edge << SPI_CK_IDLE_EDGE_S;
      putreg32(regval, SPI_MISC_REG(id));

      regval  = getreg32(SPI_USER_REG(id));
      regval &= SPI_CK_OUT_EDGE_M;
      regval |= ck_out_edge << SPI_CK_OUT_EDGE_S;
      putreg32(regval, SPI_USER_REG(id));

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: esp32s3_qspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits in an QSPI word.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  priv->nbits = nbits;
}

/****************************************************************************
 * Name: esp32s3_qspi_command
 *
 * Description:
 *   Perform one QSPI data transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int esp32s3_qspi_command(struct qspi_dev_s *dev,
                                struct qspi_cmdinfo_s *cmdinfo)
{
  uint32_t regval;
  uint8_t *data;
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;
  uint8_t id = priv->config->id;
  uint32_t user1_reg = getreg32(SPI_USER1_REG(id));
  uint32_t user_reg = getreg32(SPI_USER_REG(id));

  if (QSPICMD_ISWRITE(cmdinfo->flags) || QSPICMD_ISREAD(cmdinfo->flags))
    {
      if (cmdinfo->buflen > QSPI_CMD_BUFSIZE)
        {
          return -EINVAL;
        }
      else
        {
          putreg32(cmdinfo->buflen * 8 - 1, SPI_MS_DLEN_REG(id));
        }
    }

  /* Initiliaze QSPI user register */

  user_reg &= ~(SPI_USR_ADDR_M |
                SPI_USR_MOSI_M |
                SPI_USR_MISO_M |
                SPI_USR_DUMMY_M |
                SPI_FWRITE_DUAL_M |
                SPI_FWRITE_OCT_M |
                SPI_FWRITE_QUAD_M);
  user_reg |= SPI_USR_COMMAND_M;

  /* Set command bits and value, and command is always needed */

  regval  = getreg32(SPI_USER2_REG(id));
  regval &= ~(SPI_USR_COMMAND_BITLEN_M |
              SPI_USR_COMMAND_VALUE_M |
              SPI_MST_REMPTY_ERR_END_EN_M);
  regval |= ((QSPI_CMD_BITS(cmdinfo->cmd) - 1) << SPI_USR_COMMAND_BITLEN_S) |
            (cmdinfo->cmd << SPI_USR_COMMAND_VALUE_S);
  putreg32(regval, SPI_USER2_REG(id));

  /* Set address bits and value */

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      user1_reg &= ~SPI_USR_ADDR_BITLEN_M;
      user1_reg |= (cmdinfo->addrlen * 8 - 1) << SPI_USR_ADDR_BITLEN_S;

      user_reg |= SPI_USR_ADDR_M;

      putreg32(cmdinfo->addr, SPI_ADDR_REG(id));
    }

  /* Set dummy */

  if (priv->dummies)
    {
      user1_reg &= ~SPI_USR_DUMMY_CYCLELEN_M;
      user1_reg |= (priv->dummies - 1) << SPI_USR_DUMMY_CYCLELEN_S;

      user_reg |= SPI_USR_DUMMY_M;
    }

  /* Set TX data */

  if (QSPICMD_ISWRITE(cmdinfo->flags))
    {
      data = (uint8_t *)cmdinfo->buffer;

      for (int i = 0; i < cmdinfo->buflen; i += 4)
        {
          memcpy(&regval, data + i, 4);
          putreg32(regval, SPI_W0_REG(id) + i);
        }

      user_reg |= SPI_USR_MOSI_M;

      if (priv->data_lines == 2)
        {
          user_reg |= SPI_FWRITE_DUAL_M;
        }
      else if (priv->data_lines == 4)
        {
          user_reg |= SPI_FWRITE_QUAD_M;
        }
    }
  else if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      user_reg |= SPI_USR_MISO_M;
    }

  putreg32(user_reg,  SPI_USER_REG(id));
  putreg32(user1_reg, SPI_USER1_REG(id));

  /* Set command and address I/O mode */

  regval  = getreg32(SPI_CTRL_REG(id));
  regval &= ~(SPI_FCMD_OCT_M | SPI_FADDR_OCT_M | SPI_FREAD_OCT_M |
              SPI_FCMD_DUAL_M | SPI_FADDR_DUAL_M | SPI_FREAD_DUAL_M |
              SPI_FCMD_QUAD_M | SPI_FADDR_QUAD_M | SPI_FREAD_QUAD_M);
  if (QSPICMD_ISIDUAL(cmdinfo->flags))
    {
      regval |= SPI_FCMD_DUAL_M;
    }
  else if (QSPICMD_ISIQUAD(cmdinfo->flags))
    {
      regval |= SPI_FCMD_QUAD_M;
    }

  if (priv->addr_lines == 2)
    {
      regval |= SPI_FADDR_DUAL_M;
    }
  else if (priv->addr_lines == 4)
    {
      regval |= SPI_FADDR_QUAD_M;
    }

  if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      if (priv->data_lines == 2)
        {
          regval |= SPI_FREAD_DUAL_M;
        }
      else if (priv->data_lines == 4)
        {
          regval |= SPI_FREAD_QUAD_M;
        }
    }

  putreg32(regval, SPI_CTRL_REG(id));

  /* Update QSPI master registers value */

  regval = getreg32(SPI_CMD_REG(id));
  regval |= SPI_UPDATE_M;
  putreg32(regval, SPI_CMD_REG(id));

  while ((getreg32(SPI_CMD_REG(id)) & SPI_UPDATE_M) != 0)
    {
      ;
    }

  /* Start transmision */

  regval = getreg32(SPI_CMD_REG(id));
  regval |= SPI_USR_M;
  putreg32(regval, SPI_CMD_REG(id));

  /* Wait until transmission is done */

  while ((getreg32(SPI_CMD_REG(id)) & SPI_USR_M) != 0)
    {
      ;
    }

  if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      data = (uint8_t *)cmdinfo->buffer;

      for (int i = 0; i < cmdinfo->buflen; i += 4)
        {
          regval = getreg32(SPI_W0_REG(id) + i);
          memcpy(data + i, &regval, MIN(4, cmdinfo->buflen - i));
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_qspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int esp32s3_qspi_memory(struct qspi_dev_s *dev,
                               struct qspi_meminfo_s *meminfo)
{
#ifdef CONFIG_ESP32S3_SPI_DMA
  uint32_t regval;
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;
  uint8_t id = priv->config->id;
  uint32_t user1_reg = getreg32(SPI_USER1_REG(id));
  uint32_t user_reg = getreg32(SPI_USER_REG(id));

  if (QSPIMEM_ISWRITE(meminfo->flags) || QSPIMEM_ISREAD(meminfo->flags))
    {
      if (meminfo->buflen > QSPI_DMA_BUFSIZE)
        {
          return -EINVAL;
        }
      else
        {
          putreg32(meminfo->buflen * 8 - 1, SPI_MS_DLEN_REG(id));
        }
    }

  /* Reset SPI DMA FIFO */

  regval  = getreg32(SPI_DMA_CONF_REG(id));
  regval |= QSPI_DMA_RESET_MASK;
  putreg32(regval, SPI_DMA_CONF_REG(id));

  regval &= ~QSPI_DMA_RESET_MASK;
  putreg32(regval, SPI_DMA_CONF_REG(id));

  /* Initiliaze QSPI user register */

  user_reg &= ~(SPI_USR_MOSI_M |
                SPI_USR_MISO_M |
                SPI_USR_DUMMY_M |
                SPI_FWRITE_DUAL_M |
                SPI_FWRITE_OCT_M |
                SPI_FWRITE_QUAD_M);
  user_reg |= SPI_USR_COMMAND_M | SPI_USR_ADDR_M;

  /* Set command bits and value, and command is always needed */

  regval  = getreg32(SPI_USER2_REG(id));
  regval &= ~(SPI_USR_COMMAND_BITLEN_M |
              SPI_USR_COMMAND_VALUE_M |
              SPI_MST_REMPTY_ERR_END_EN_M);
  regval |= ((QSPI_CMD_BITS(meminfo->cmd) - 1) << SPI_USR_COMMAND_BITLEN_S) |
            (meminfo->cmd << SPI_USR_COMMAND_VALUE_S);
  putreg32(regval, SPI_USER2_REG(id));

  /* Set address bits and value */

  user1_reg &= ~SPI_USR_ADDR_BITLEN_M;
  user1_reg |= (meminfo->addrlen * 8 - 1) << SPI_USR_ADDR_BITLEN_S;

  putreg32(meminfo->addr, SPI_ADDR_REG(id));

  /* Set dummy */

  if (meminfo->dummies)
    {
      user1_reg &= ~SPI_USR_DUMMY_CYCLELEN_M;
      user1_reg |= (meminfo->dummies - 1) << SPI_USR_DUMMY_CYCLELEN_S;

      user_reg |= SPI_USR_DUMMY_M;
    }

  /* Enable SPI DMA TX */

  if (QSPIMEM_ISWRITE(meminfo->flags))
    {
      regval  = getreg32(SPI_DMA_CONF_REG(id));
      regval |= SPI_DMA_TX_ENA_M;
      putreg32(regval, SPI_DMA_CONF_REG(id));

      user_reg |= SPI_USR_MOSI_M;

      if (priv->data_lines == 2)
        {
          user_reg |= SPI_FWRITE_DUAL_M;
        }
      else if (priv->data_lines == 4)
        {
          user_reg |= SPI_FWRITE_QUAD_M;
        }

      esp32s3_dma_setup(priv->dma_channel,
                        true,
                        priv->dma_desc,
                        QSPI_DMA_DESC_NUM,
                        (uint8_t *)meminfo->buffer,
                        meminfo->buflen);
      esp32s3_dma_enable(priv->dma_channel, true);
    }
  else if (QSPIMEM_ISREAD(meminfo->flags))
    {
      regval  = getreg32(SPI_DMA_CONF_REG(id));
      regval |= SPI_DMA_RX_ENA_M;
      putreg32(regval, SPI_DMA_CONF_REG(id));

      user_reg |= SPI_USR_MISO_M;

      esp32s3_dma_setup(priv->dma_channel,
                        false,
                        priv->dma_desc,
                        QSPI_DMA_DESC_NUM,
                        (uint8_t *)meminfo->buffer,
                        meminfo->buflen);
      esp32s3_dma_enable(priv->dma_channel, false);
    }

  putreg32(user_reg,  SPI_USER_REG(id));
  putreg32(user1_reg, SPI_USER1_REG(id));

  /* Set command and address I/O mode */

  regval  = getreg32(SPI_CTRL_REG(id));
  regval &= ~(SPI_FCMD_OCT_M | SPI_FADDR_OCT_M | SPI_FREAD_OCT_M |
              SPI_FCMD_DUAL_M | SPI_FADDR_DUAL_M | SPI_FREAD_DUAL_M |
              SPI_FCMD_QUAD_M | SPI_FADDR_QUAD_M | SPI_FREAD_QUAD_M);
  if (QSPIMEM_ISIDUAL(meminfo->flags))
    {
      regval |= SPI_FCMD_DUAL_M;
    }
  else if (QSPIMEM_ISIQUAD(meminfo->flags))
    {
      regval |= SPI_FCMD_QUAD_M;
    }

  if (priv->addr_lines == 2)
    {
      regval |= SPI_FADDR_DUAL_M;
    }
  else if (priv->addr_lines == 4)
    {
      regval |= SPI_FADDR_QUAD_M;
    }

  if (QSPIMEM_ISREAD(meminfo->flags))
    {
      if (priv->data_lines == 2)
        {
          regval |= SPI_FREAD_DUAL_M;
        }
      else if (priv->data_lines == 4)
        {
          regval |= SPI_FREAD_QUAD_M;
        }
    }

  putreg32(regval, SPI_CTRL_REG(id));

  /* Set interrupt */

  putreg32(SPI_TRANS_DONE_INT_CLR_M, SPI_DMA_INT_CLR_REG(id));
  putreg32(SPI_TRANS_DONE_INT_ENA_M, SPI_DMA_INT_ENA_REG(id));

  /* Update QSPI master registers value */

  regval = getreg32(SPI_CMD_REG(id));
  regval |= SPI_UPDATE_M;
  putreg32(regval, SPI_CMD_REG(id));

  while ((getreg32(SPI_CMD_REG(id)) & SPI_UPDATE_M) != 0)
    {
      ;
    }

  /* Start transmision */

  regval = getreg32(SPI_CMD_REG(id));
  regval |= SPI_USR_M;
  putreg32(regval, SPI_CMD_REG(id));

  /* Wait for transmision done */

  esp32s3_qspi_wait_sem(priv);

  /* Reset interrupt */

  putreg32(0, SPI_DMA_INT_ENA_REG(id));

  return 0;
#else
  return -1;
#endif
}

/****************************************************************************
 * Name: esp32s3_qspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of the allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

static void *esp32s3_qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
#ifdef CONFIG_ESP32S3_SPI_DMA
  return kmm_malloc(ALIGN_UP(buflen, 4));
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: esp32s3_qspi_free
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_free(struct qspi_dev_s *dev, void *buffer)
{
#ifdef CONFIG_ESP32S3_SPI_DMA
  if (buffer)
    {
      kmm_free(buffer);
    }
#endif
}

/****************************************************************************
 * Name:  esp32s3_qspi_wait_sem
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_DMA
static int esp32s3_qspi_wait_sem(struct esp32s3_qspi_priv_s *priv)
{
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, SEC2TICK(10));
}
#endif

/****************************************************************************
 * Name: esp32s3_qspi_init_dma
 *
 * Description:
 *   Initialize ESP32-S3 QSPI connection to GDMA engine.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_DMA
void esp32s3_qspi_init_dma(struct esp32s3_qspi_priv_s *priv)
{
  const struct esp32s3_qspi_config_s *config = priv->config;

  /* Enable GDMA clock for the QSPI peripheral */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, config->dma_clk_bit);

  /* Reset GDMA for the QSPI peripheral */

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, config->dma_rst_bit, 0);

  /* Initialize GDMA controller */

  esp32s3_dma_init();

  /* Request a GDMA channel for QSPI peripheral */

  priv->dma_channel = esp32s3_dma_request(config->dma_periph, 1, 1, true);
  if (priv->dma_channel < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      DEBUGPANIC();
    }

  /* Disable segment transaction mode for QSPI Master */

  putreg32((SPI_SLV_RX_SEG_TRANS_CLR_EN_M | SPI_SLV_TX_SEG_TRANS_CLR_EN_M),
           SPI_DMA_CONF_REG(config->id));
}
#endif

/****************************************************************************
 * Name: esp32s3_qspi_init_iomux
 *
 * Description:
 *   Initialize ESP32-S3 QSPI GPIO by IO MUX.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if QSPI_VIA_IOMUX != 0
static void esp32s3_qspi_init_iomux(struct esp32s3_qspi_priv_s *priv)
{
  uint32_t attr = OUTPUT_FUNCTION_5;
  const struct esp32s3_qspi_config_s *config = priv->config;

  esp32s3_configgpio(config->cs_pin,  attr);
  esp32s3_configgpio(config->clk_pin, attr);

  attr |= INPUT;

  esp32s3_configgpio(config->mosi_pin, attr);
  esp32s3_configgpio(config->miso_pin, attr);
  esp32s3_configgpio(config->io2_pin,  attr);
  esp32s3_configgpio(config->io3_pin,  attr);

  esp32s3_gpio_matrix_out(config->cs_pin,   SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(config->clk_pin,  SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(config->mosi_pin, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(config->miso_pin, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(config->io2_pin,  SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(config->io3_pin,  SIG_GPIO_OUT_IDX, 0, 0);
}
#endif

/****************************************************************************
 * Name: esp32s3_qspi_init_iomatrix
 *
 * Description:
 *   Initialize ESP32-S3 QSPI GPIO by IO matrix.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if QSPI_VIA_IOMUX == 0 || defined(CONFIG_ESP32S3_SPI3)
static void esp32s3_qspi_init_iomatrix(struct esp32s3_qspi_priv_s *priv)
{
  uint32_t attr = OUTPUT;
  const struct esp32s3_qspi_config_s *config = priv->config;

  esp32s3_configgpio(config->cs_pin, OUTPUT);
  esp32s3_gpio_matrix_out(config->cs_pin, config->cs_outsig, 0, 0);

  esp32s3_configgpio(config->clk_pin, OUTPUT);
  esp32s3_gpio_matrix_out(config->clk_pin, config->clk_outsig, 0, 0);

  attr |= INPUT;

  esp32s3_configgpio(config->mosi_pin, attr);
  esp32s3_gpio_matrix_in(config->mosi_pin, config->mosi_insig, 0);
  esp32s3_gpio_matrix_out(config->mosi_pin, config->mosi_outsig, 0, 0);

  esp32s3_configgpio(config->miso_pin, attr);
  esp32s3_gpio_matrix_in(config->miso_pin, config->miso_insig, 0);
  esp32s3_gpio_matrix_out(config->miso_pin, config->miso_outsig, 0, 0);

  esp32s3_configgpio(config->io2_pin, attr);
  esp32s3_gpio_matrix_in(config->io2_pin, config->io2_insig, 0);
  esp32s3_gpio_matrix_out(config->io2_pin, config->io2_outsig, 0, 0);

  esp32s3_configgpio(config->io3_pin, attr);
  esp32s3_gpio_matrix_in(config->io3_pin, config->io3_insig, 0);
  esp32s3_gpio_matrix_out(config->io3_pin, config->io3_outsig, 0, 0);
}
#endif

/****************************************************************************
 * Name: esp32s3_qspi_init_gpio
 *
 * Description:
 *   Initialize ESP32-S3 QSPI GPIO
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_init_gpio(struct esp32s3_qspi_priv_s *priv)
{
  /* SPI3 doesn't have IOMUX  */

#if QSPI_VIA_IOMUX != 0
  if (priv->config->id == 2)
    {
      esp32s3_qspi_init_iomux(priv);
    }
#ifdef CONFIG_ESP32S3_SPI3
  else
    {
      esp32s3_qspi_init_iomatrix(priv);
    }
#endif
#else
  esp32s3_qspi_init_iomatrix(priv);
#endif
}

/****************************************************************************
 * Name: esp32s3_qspi_init
 *
 * Description:
 *   Initialize ESP32-S3 QSPI hardware interface.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_init(struct esp32s3_qspi_priv_s *priv)
{
  const struct esp32s3_qspi_config_s *config = priv->config;
  uint8_t id = config->id;

  esp32s3_qspi_init_gpio(priv);

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, config->clk_bit);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, config->rst_bit, 0);

  putreg32(SPI_USR_MOSI_M | SPI_CS_HOLD_M, SPI_USER_REG(id));

  putreg32(0, SPI_USER1_REG(id));
  putreg32(0, SPI_SLAVE_REG(id));
  putreg32(SPI_CS1_DIS_M | SPI_CS2_DIS_M, SPI_MISC_REG(id));

  putreg32(SPI_CLK_EN_M | SPI_MST_CLK_ACTIVE_M | SPI_MST_CLK_SEL_M,
           SPI_CLK_GATE_REG(id));

  putreg32(0, SPI_CTRL_REG(id));

#ifdef CONFIG_ESP32S3_SPI_DMA
  esp32s3_qspi_init_dma(priv);
#endif

  esp32s3_qspi_setfrequency(&priv->spi_dev, QSPI_DEFAULT_FREQ);
  esp32s3_qspi_setbits(&priv->spi_dev, QSPI_DEFAULT_WIDTH);
  esp32s3_qspi_setmode(&priv->spi_dev, QSPI_DEFAULT_MODE);
}

/****************************************************************************
 * Name: esp32s3_qspi_deinit
 *
 * Description:
 *   Deinitialize ESP32-S3 QSPI hardware interface.
 *
 * Input Parameters:
 *   priv - QSPI private state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s3_qspi_deinit(struct esp32s3_qspi_priv_s *priv)
{
  const struct esp32s3_qspi_config_s *config = priv->config;

#ifdef CONFIG_ESP32S3_SPI_DMA
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, config->dma_rst_bit);
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, config->dma_clk_bit, 0);
#endif

  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, config->rst_bit);
  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, config->clk_bit, 0);

  priv->frequency  = 0;
  priv->actual     = 0;
  priv->mode       = QSPIDEV_MODE0;
  priv->nbits      = 0;
  priv->dummies    = 0;
  priv->addr_lines = 4;
  priv->data_lines = 4;
}

/****************************************************************************
 * Name: esp32s3_qspi_interrupt
 *
 * Description:
 *   Common QSPI DMA interrupt handler.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - QSPI controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_DMA
static int esp32s3_qspi_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)arg;

  /* Write 1 to clear interrupt bit */

  putreg32(SPI_TRANS_DONE_INT_CLR_M, SPI_DMA_INT_CLR_REG(priv->config->id));

  nxsem_post(&priv->sem_isr);

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_qspibus_set_attr
 *
 * Description:
 *   Set attribution of QSPI bus transfer.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   dummies    - Number of dummy cycles, this only works in command
 *                transfer, not works in memory transfer
 *   addr_lines - Number of address transmiting I/O pins
 *   data_lines - Number of data transmiting I/O pins
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_qspibus_set_attr(struct qspi_dev_s *dev,
                             uint8_t dummies,
                             uint8_t addr_lines,
                             uint8_t data_lines)
{
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  if ((addr_lines != 1) &&
      (addr_lines != 2) &&
      (addr_lines != 4))
    {
      return ERROR;
    }

  if ((data_lines != 1) &&
      (data_lines != 2) &&
      (data_lines != 4))
    {
      return ERROR;
    }

  priv->dummies = dummies;
  priv->addr_lines = addr_lines;
  priv->data_lines = data_lines;

  return OK;
}

/****************************************************************************
 * Name: esp32s3_qspibus_initialize
 *
 * Description:
 *   Initialize the selected QSPI bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple QSPI interfaces)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct qspi_dev_s *esp32s3_qspibus_initialize(int port)
{
  struct qspi_dev_s *spi_dev;
  struct esp32s3_qspi_priv_s *priv;

  switch (port)
    {
#ifdef CONFIG_ESP32S3_SPI2
      case ESP32S3_SPI2:
        priv = &esp32s3_spi2_priv;
        break;
#endif
#ifdef CONFIG_ESP32S3_SPI3
      case ESP32S3_SPI3:
        priv = &esp32s3_spi3_priv;
        break;
#endif
      default:
        return NULL;
    }

  spi_dev = (struct qspi_dev_s *)priv;

  nxmutex_lock(&priv->lock);
  if (priv->refs != 0)
    {
      priv->refs++;
      nxmutex_unlock(&priv->lock);
      return spi_dev;
    }

#ifdef CONFIG_ESP32S3_SPI_DMA

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->config->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  /* Attach and enable the IRQ */

  if (irq_attach(priv->config->irq, esp32s3_qspi_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
      nxmutex_unlock(&priv->lock);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the QSPI device. */

  up_enable_irq(priv->config->irq);
#endif

  esp32s3_qspi_init(priv);
  priv->refs++;
  nxmutex_unlock(&priv->lock);
  return spi_dev;
}

/****************************************************************************
 * Name: esp32s3_qspibus_uninitialize
 *
 * Description:
 *   Uninitialize an QSPI bus.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_qspibus_uninitialize(struct qspi_dev_s *dev)
{
  struct esp32s3_qspi_priv_s *priv = (struct esp32s3_qspi_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs != 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

#ifdef CONFIG_ESP32S3_SPI_DMA
  up_disable_irq(priv->config->irq);
  esp32s3_teardown_irq(priv->cpu, priv->config->periph, priv->cpuint);
  irq_detach(priv->config->irq);

  priv->cpuint = -ENOMEM;
#endif

  esp32s3_qspi_deinit(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif /* CONFIG_ESP32S3_SPI */
