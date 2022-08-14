/****************************************************************************
 * arch/risc-v/src/bl602/bl602_spi.c
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "bl602_glb.h"
#include "bl602_gpio.h"
#include "bl602_romapi.h"
#include "bl602_spi.h"
#include "bl602_dma.h"
#include "hardware/bl602_dma.h"
#include "hardware/bl602_glb.h"
#include "hardware/bl602_spi.h"
#include "hardware/bl602_hbn.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_FREQ_DEFAULT 400000
#define LLI_BUFF_SIZE 2048  /* Maximum transaction count per LLI entry. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI frequency cal configuration */

struct prescale_and_count_cal_ctx_s
{
  uint64_t desired_ticks;
  uint64_t count_max;
  uint64_t error;
  uint64_t count;
  uint32_t prescale;
  uint8_t update;
};

/* SPI clock configuration */

struct spi_clock_cfg_s
{
  uint8_t start_len;       /* Length of start condition */
  uint8_t stop_len;        /* Length of stop condition */
  uint8_t data_phase0_len; /* Length of data phase 0,affecting clock */
  uint8_t data_phase1_len; /* Length of data phase 1,affecting clock */
  uint8_t interval_len;    /* Length of interval between frame */
};

/* SPI configuration */

struct spi_cfg_s
{
  uint8_t deglitch_enable;   /* Enable or disable de-glitch function */
  uint8_t continuous_enable; /* Enable or disable master continuous transfer
                              * mode,enable:SS will stay asserted if next data
                              * is valid
                              */
  uint8_t byte_sequence;     /* The byte is sent first in SPI transfer ,0 is send
                              * 0byte first; 1 is send 3byte first
                              */
  uint8_t bit_sequence;      /* The bit is sent first in SPI transfer ,0 is each byte
                              * is sent out MSB first; 1 is each byte is sent out LSB
                              * first
                              */
};

/* SPI Device hardware configuration */

struct bl602_spi_config_s
{
  uint32_t clk_freq;    /* SPI clock frequency */
  enum spi_mode_e mode; /* SPI default mode */

  bool use_dma; /* Use DMA */
};

struct bl602_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* Port configuration */

  const struct bl602_spi_config_s *config;

  int refs; /* Referernce count */

  /* Held while chip is selected for mutual exclusion */

  sem_t exclsem;

  /* Interrupt wait semaphore */

  sem_t sem_isr_tx;
  sem_t sem_isr_rx;

  uint32_t frequency; /* Requested clock frequency */
  uint32_t actual;    /* Actual clock frequency */

  enum spi_mode_e mode; /* Actual SPI hardware mode */

  /* Actual SPI send/receive bits once transmission */

  uint8_t nbits;
  int8_t dma_txchan;
  int8_t dma_rxchan;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl602_spi_lock(struct spi_dev_s *dev, bool lock);

static void bl602_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected);

static uint32_t bl602_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void bl602_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void bl602_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int bl602_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t bl602_spi_status(struct spi_dev_s *dev,
                                uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int bl602_spi_cmddata(struct spi_dev_s *dev,
                              uint32_t devid, bool cmd);
#endif
static uint32_t bl602_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void bl602_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void bl602_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords);
static void bl602_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int bl602_spi_trigger(struct spi_dev_s *dev);
#endif
static void bl602_spi_init(struct spi_dev_s *dev);
static void bl602_spi_deinit(struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BL602_SPI0
static const struct bl602_spi_config_s bl602_spi_config =
{
    .clk_freq = SPI_FREQ_DEFAULT,
    .mode = SPIDEV_MODE0,
    .use_dma = false,
};

static const struct spi_ops_s bl602_spi_ops =
{
    .lock = bl602_spi_lock,
    .select = bl602_spi_select,
    .setfrequency = bl602_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
    .setdelay = bl602_spi_setdelay,
#endif
    .setmode = bl602_spi_setmode,
    .setbits = bl602_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures = bl602_spi_hwfeatures,
#endif
    .status = bl602_spi_status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata = bl602_spi_cmddata,
#endif
    .send = bl602_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange = bl602_spi_exchange,
#else
    .sndblock = bl602_spi_sndblock,
    .recvblock = bl602_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
    .trigger = bl602_spi_trigger,
#endif
    .registercallback = NULL,
};

static struct bl602_spi_priv_s bl602_spi_priv =
{
  .spi_dev =
              {
                .ops = &bl602_spi_ops
              },
  .config = &bl602_spi_config,
  .dma_rxchan = -1,
  .dma_txchan = -1,
};

#endif  /* CONFIG_BL602_SPI0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BL602_SPI_DMA
static void bl602_dma_rx_callback(uint8_t channel, uint8_t status, void *arg)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)arg;
  UNUSED(channel);
  spiinfo("RX interrupt fired with status %u\n", status);
  if (status == BL602_DMA_INT_EVT_TC)
    {
      nxsem_post(&priv->sem_isr_rx);
    }
  else
    {
      spierr("DMA transfer failed for RX.\n");
    }
}

static void bl602_dma_tx_callback(uint8_t channel, uint8_t status, void *arg)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)arg;
  UNUSED(channel);
  spiinfo("TX interrupt fired with status %u\n", status);
  if (status == BL602_DMA_INT_EVT_TC)
    {
      nxsem_post(&priv->sem_isr_tx);
    }
  else
    {
      spierr("DMA transfer failed for TX.\n");
    }
}
#endif

/****************************************************************************
 * Name: bl602_check_with_new_prescale
 *
 * Description:
 *   Check if the option prescale
 *
 ****************************************************************************/

static void
bl602_check_with_new_prescale(struct prescale_and_count_cal_ctx_s *p_ctx,
                                    uint32_t prescale_new)
{
  uint64_t count = p_ctx->desired_ticks / prescale_new;
  uint64_t error;

  if (count > p_ctx->count_max)
    {
      count = p_ctx->count_max;
    }

  error = p_ctx->desired_ticks - count * prescale_new;

  if (p_ctx->error > error)
    {
      p_ctx->error = error;
      p_ctx->count = count;
      p_ctx->prescale = prescale_new;
      p_ctx->update = 1;
    }
}

/****************************************************************************
 * Name: bl602_prescale_and_count_cal
 *
 * Description:
 *   prescale and count cal
 *
 ****************************************************************************/

static int
bl602_prescale_and_count_cal(uint32_t counter_width,
                            uint32_t prescale_max, size_t ticks,
                            uint32_t *p_prescale, size_t *p_count)
{
  struct prescale_and_count_cal_ctx_s ctx;

  uint32_t prescale_min;
  uint32_t prescale;

  size_t count_max;

  count_max = ((uint64_t)1ull << counter_width) - 1;

  if (ticks <= count_max)
    {
      *p_prescale = 1;
      *p_count = ticks;

      return 0;
    }

  prescale_min = ticks / count_max;

  ctx.count_max = count_max;
  ctx.desired_ticks = ticks;
  ctx.error = ticks;
  ctx.count = count_max;
  ctx.prescale = 1;
  ctx.update = 0;

  if (prescale_max < prescale_min)
    {
      return -1;
    }

  for (prescale = prescale_min; prescale <= prescale_max; prescale++)
    {
      bl602_check_with_new_prescale(&ctx, prescale);
      if (ctx.error == 0)
        {
          break;
        }
    }

  if (ctx.update)
    {
      *p_prescale = ctx.prescale;
      *p_count = ctx.count;

      return 0;
    }

  return -1;
}

/****************************************************************************
 * Name: bl602_set_spi_clk
 *
 * Description:
 *   set SPI clock
 *
 ****************************************************************************/

static void bl602_set_spi_clk(uint8_t enable, uint8_t div)
{
  modifyreg32(BL602_CLK_CFG3, CLK_CFG3_SPI_CLK_DIV_MASK, div);

  if (enable)
    {
      modifyreg32(BL602_CLK_CFG3, 0, CLK_CFG3_SPI_CLK_EN);
    }
  else
    {
      modifyreg32(BL602_CLK_CFG3, CLK_CFG3_SPI_CLK_EN, 0);
    }
}

/****************************************************************************
 * Name: bl602_clockconfig
 *
 * Description:
 *   SPI clock config
 *
 ****************************************************************************/

static void bl602_clockconfig(struct spi_clock_cfg_s *clockcfg)
{
  /* Configure length of data phase1/0 and start/stop condition */

  modifyreg32(BL602_SPI_PRD_0, SPI_PRD_0_CR_S_MASK,
              (clockcfg->start_len - 1));
  modifyreg32(BL602_SPI_PRD_0, SPI_PRD_0_CR_P_MASK,
              (clockcfg->stop_len - 1) << SPI_PRD_0_CR_P_SHIFT);
  modifyreg32(BL602_SPI_PRD_0, SPI_PRD_0_CR_D_PH_0_MASK,
              (clockcfg->data_phase0_len - 1) << SPI_PRD_0_CR_D_PH_0_SHIFT);
  modifyreg32(BL602_SPI_PRD_0, SPI_PRD_0_CR_D_PH_1_MASK,
              (clockcfg->data_phase1_len - 1) << SPI_PRD_0_CR_D_PH_1_SHIFT);

  /* Configure length of interval between frame */

  modifyreg32(BL602_SPI_PRD_1, SPI_PRD_1_CR_I_MASK,
              clockcfg->interval_len - 1);
}

/****************************************************************************
 * Name: bl602_spi_lock
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

static int bl602_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

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
 * Name: bl602_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If disable bl602_SPI_SWCS, driver will use hardware CS so that when
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

static void bl602_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  /* we used hardware CS */

  spiinfo("devid: %lu, CS: %s\n", devid, selected ? "select" : "free");

#ifdef CONFIG_SPI_CMDDATA
  /* revert MISO from GPIO Pin to SPI Pin */

  if (!selected)
    {
      bl602_configgpio(BOARD_SPI_MISO);
    }
#endif
}

/****************************************************************************
 * Name: bl602_spi_setfrequency
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

static uint32_t bl602_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;
  struct spi_clock_cfg_s clockcfg;
  size_t count;
  uint8_t ticks;
  uint8_t bclk_div;
  uint32_t clk_div;
  uint32_t sys_clock;
  uint32_t spi_basic_clk;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */

      return priv->actual;
    }

  bclk_div = bl602_glb_get_bclk_div();
  sys_clock = getreg32(BL602_HBN_RSV2);
  spi_basic_clk = sys_clock / (bclk_div + 1) / 2;
  ticks = spi_basic_clk / frequency;

  if (bl602_prescale_and_count_cal(8, 0xff, ticks, &clk_div, &count) != 0)
    {
      spierr("SPI div clk error\n");
      DEBUGPANIC();

      return -1;
    }

  bl602_set_spi_clk(1, clk_div - 1);

  clockcfg.start_len = count;
  clockcfg.stop_len = count;
  clockcfg.data_phase0_len = count;
  clockcfg.data_phase1_len = count;
  clockcfg.interval_len = count;

  bl602_clockconfig(&clockcfg);

  priv->frequency = frequency;

  spiinfo("frequency=%lu, actual=%lu\n", priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: bl602_spi_setdelay
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *   ifdelay    - The delay between frames
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int bl602_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                                uint32_t stopdelay, uint32_t csdelay,
                                uint32_t ifdelay)
{
  spierr("SPI CS delay control not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: bl602_spi_setmode
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

static void
bl602_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_SCLK_POL
                      | SPI_CFG_CR_SCLK_PH, 0);
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_SCLK_POL,
                      SPI_CFG_CR_SCLK_PH);
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_SCLK_PH,
                      SPI_CFG_CR_SCLK_POL);
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          modifyreg32(BL602_SPI_CFG, 0, SPI_CFG_CR_SCLK_POL
                      | SPI_CFG_CR_SCLK_PH);
          break;

        default:
          return;
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: bl602_spi_setbits
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

static void bl602_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Save the selection so that subsequent re-configurations
       * will be faster.
       */

    switch (nbits)
      {
      case 8:

        /* set valid width for each fifo entry 8 bit */

        modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_FRAME_SIZE_MASK, 0);
        break;

      case 16:
        modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_FRAME_SIZE_MASK,
                    1 << SPI_CFG_CR_FRAME_SIZE_SHIFT);
        break;

      default:
        return;
      }

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: bl602_spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

static uint8_t bl602_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

/****************************************************************************
 * Name: bl602_spi_cmddata
 *
 * Description:
 *   Some devices require an additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 *   This function reconfigures MISO from SPI Pin to GPIO Pin, and sets
 *   MISO to high (data) or low (command). bl602_spi_select() will revert
 *   MISO back from GPIO Pin to SPI Pin.  We must revert because the SPI Bus
 *   may be used by other drivers.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int bl602_spi_cmddata(struct spi_dev_s *dev,
                              uint32_t devid, bool cmd)
{
  spiinfo("devid: %" PRIu32 " CMD: %s\n", devid, cmd ? "command" :
          "data");

  if (devid == SPIDEV_DISPLAY(0))
    {
      gpio_pinset_t gpio;
      int ret;

      /* reconfigure MISO from SPI Pin to GPIO Pin */

      gpio = (BOARD_SPI_MISO & GPIO_PIN_MASK)
             | GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO;
      ret = bl602_configgpio(gpio);
      if (ret < 0)
        {
          spierr("Failed to configure MISO as GPIO\n");
          DEBUGPANIC();

          return ret;
        }

      /* set MISO to high (data) or low (command) */

      bl602_gpiowrite(gpio, !cmd);

      return OK;
    }

  spierr("SPI cmddata not supported\n");
  DEBUGPANIC();

  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: bl602_spi_hwfeatures
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
static int bl602_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  spierr("SPI hardware specific feature not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: lli_list_init
 *
 * Description:
 *   Configure the LLI structure for DMA transactions with SPI
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   tx_lli - A pointer to the LLI structures for TX.
 *   rx_lli - A pointer to the LLI structures for RX.
 *   tx_buffer - A pointer to the transaction buffer for TX.
 *   rx_buffer - A pointer to the transaction buffer for RX.
 *               Buffer is null if TX only.
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   Error state - 0 on success.
 *
 ****************************************************************************/

#ifdef CONFIG_BL602_SPI_DMA
static int lli_list_init(struct bl602_spi_priv_s *priv,
                         struct bl602_lli_ctrl_s **tx_lli,
                         struct bl602_lli_ctrl_s **rx_lli,
                         const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  uint8_t i;
  uint32_t count;
  uint32_t remainder;
  struct bl602_dma_ctrl_s  dma_ctrl;

  /* Determine the how many LLI entries will be required */

  count = (nwords * priv->nbits / 8) / LLI_BUFF_SIZE;
  remainder = (nwords * priv->nbits / 8) % LLI_BUFF_SIZE;

  /* If LLI entry cannot be fully packed with data, add an additional entry
   * for the remainder entry.
   */

  if (remainder != 0)
    {
      count = count + 1;
    }

  /* Set the base config that will be used for each entry */

  dma_ctrl.src_width = (priv->nbits / 8) - 1; /* 8/16/32 bits */
  dma_ctrl.dst_width = dma_ctrl.src_width;
  dma_ctrl.src_burst_size = 0; /* 1 item per transaction */
  dma_ctrl.dst_burst_size = 0; /* 1 item per transaction */
  dma_ctrl.protect = 0;
  dma_ctrl.tc_int_en = 0; /* We will overwrite this in the last entry */
  dma_ctrl.rsvd = 0;
  dma_ctrl.sld = 0; /* Not used for non mem-to-mem transfers. */

  /* We will set these per entry:
   *  - transfer_size
   *  - src_increment
   *  - dst_increment
   */

  /* Allocate the transfer block.
   * TODO consider supporting pre-allocation of these structures.
   * most transaction will only use a single LLI, so we could
   * actually place the single LLI structure on the stack (4*4 bytes)
   */

  *tx_lli = kmm_malloc(sizeof(struct bl602_lli_ctrl_s) * count);
  if (*tx_lli == NULL)
    {
      spierr("Failed to allocate lli for tx.\n");
      return -1;
    }

  if (rxbuffer != NULL)
    {
      *rx_lli = kmm_malloc(sizeof(struct bl602_lli_ctrl_s) * count);
      if (*rx_lli == NULL)
        {
          spierr("Failed to allocate lli for rx.\n");
          kmm_free(*tx_lli);
          return -1;
        }
    }
  else
    {
      *rx_lli = NULL;
    }

  for (i = 0; i < count; i++)
    {
      /* Check if this is the final entry and there is remainder set */

      if ((i == (count - 1)) && (remainder != 0))
        {
          dma_ctrl.transfer_size = remainder;
        }
      else
        {
          dma_ctrl.transfer_size = LLI_BUFF_SIZE;
        }

      /* Configure tx side */

        {
          dma_ctrl.dst_increment = 0;
          dma_ctrl.src_increment = 1;
          (*tx_lli)[i].dma_ctrl = dma_ctrl;
          (*tx_lli)[i].dst_addr = BL602_SPI_FIFO_WDATA;
          (*tx_lli)[i].src_addr = \
            (uint32_t)txbuffer + (dma_ctrl.src_width * i * LLI_BUFF_SIZE);

          /* Assume last entry, we will overwrite as needed. */

          (*tx_lli)[i].next_lli = 0;

          /* Link entry */

          if (i != 0)
            {
              (*tx_lli)[i - 1].next_lli = (uint32_t)&(*tx_lli)[i];
            }
        }

      /* Configure rx side */

      if (rxbuffer != NULL)
        {
          dma_ctrl.dst_increment = 1;
          dma_ctrl.src_increment = 0;
          (*rx_lli)[i].dma_ctrl = dma_ctrl;
          (*rx_lli)[i].dst_addr = \
            (uint32_t)rxbuffer + (dma_ctrl.dst_width * i * LLI_BUFF_SIZE);
          (*rx_lli)[i].src_addr = BL602_SPI_FIFO_RDATA;
          (*rx_lli)[i].next_lli = 0; /* Assume last entry, we will overwrite as needed. */

          /* Link entry */

          if (i != 0)
            {
              (*rx_lli)[i - 1].next_lli = (uint32_t)&(*rx_lli)[i];
            }
        }
    }

  (*tx_lli)[count - 1].dma_ctrl.tc_int_en = 1;

  if (rxbuffer != NULL)
    {
      (*rx_lli)[count - 1].dma_ctrl.tc_int_en = 1;
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: bl602_spi_dma_exchange
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

#ifdef CONFIG_BL602_SPI_DMA
static void bl602_spi_dma_exchange(struct bl602_spi_priv_s *priv,
                                   const void *txbuffer,
                                   void *rxbuffer, uint32_t nwords)
{
  int err;

  #ifdef CONFIG_DEBUG_DMA_INFO
    struct bl602_dmaregs_s regs;
  #endif

  struct bl602_lli_ctrl_s *tx_lli;
  struct bl602_lli_ctrl_s *rx_lli;

  /* Enable master */

  modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_S_EN, SPI_CFG_CR_M_EN);

  err = lli_list_init(priv, &tx_lli, &rx_lli, txbuffer, rxbuffer, nwords);

  if (err < 0)
    {
      spierr("Failed to initalize DMA LLI\n");
      return;
    }

  /* Configure DMA controller with LLI.
   * This needs to be set using the ROM API from TCM to initialize all 4
   * registers from the LLI structure.
   */

  bl602_romapi_memcpy_4(
    (uint32_t *)BL602_DMA_CH_N_REG(BL602_DMA_SRCADDR_OFFSET,
                                   priv->dma_txchan),
    (uint32_t *)tx_lli, 4);

  if (rxbuffer != NULL)
    {
      bl602_romapi_memcpy_4(
        (uint32_t *)BL602_DMA_CH_N_REG(BL602_DMA_SRCADDR_OFFSET,
                                      priv->dma_rxchan),
        (uint32_t *)rx_lli, 4);
    }

  /* Dump DMA register state */

  bl602_dmasample(&regs);
  bl602_dmadump(&regs, "Initialized DMA");

  /* Start channel */

  if (rxbuffer != NULL)
    {
      bl602_dma_channel_start(priv->dma_rxchan);
    }

  bl602_dma_channel_start(priv->dma_txchan);

  /* Dump DMA register state */

  bl602_dmasample(&regs);
  bl602_dmadump(&regs, "Post Start DMA");

  /* Wait for RX and TX to complete. */

  nxsem_wait_uninterruptible(&priv->sem_isr_tx);

  if (rxbuffer != NULL)
    {
      nxsem_wait_uninterruptible(&priv->sem_isr_rx);
    }

  /* Stop channels */

  bl602_dma_channel_stop(priv->dma_txchan);

  if (rxbuffer != NULL)
    {
      bl602_dma_channel_stop(priv->dma_rxchan);
    }

  kmm_free(tx_lli);

  if (rx_lli != NULL)
    {
      kmm_free(rx_lli);
    }

  return;
}
#endif

/****************************************************************************
 * Name: bl602_spi_poll_send
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

static uint32_t bl602_spi_poll_send(struct bl602_spi_priv_s *priv,
                                    uint32_t wd)
{
  uint32_t val;
  uint32_t tmp_val = 0;

  /* spi enable master */

  modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_S_EN, SPI_CFG_CR_M_EN);

  /* spi fifo clear  */

  modifyreg32(BL602_SPI_FIFO_CFG_0, SPI_FIFO_CFG_0_RX_CLR
              | SPI_FIFO_CFG_0_TX_CLR, 0);

  /* write data to tx fifo */

  putreg32(wd, BL602_SPI_FIFO_WDATA);

  while (0 == tmp_val)
    {
      /* get data from rx fifo */

      tmp_val = getreg32(BL602_SPI_FIFO_CFG_1);
      tmp_val = (tmp_val & SPI_FIFO_CFG_1_RX_CNT_MASK)
                >> SPI_FIFO_CFG_1_RX_CNT_SHIFT;
    }

  val = getreg32(BL602_SPI_FIFO_RDATA);

  spiinfo("send=%lx and recv=%lx\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: bl602_spi_dma_send
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

#ifdef CONFIG_BL602_SPI_DMA
static uint32_t bl602_spi_dma_send(struct bl602_spi_priv_s *priv,
                                   uint32_t wd)
{
  uint32_t rd = 0;

  bl602_spi_dma_exchange(priv, &wd, &rd, 1);

  return rd;
}
#endif

/****************************************************************************
 * Name: bl602_spi_send
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

static uint32_t bl602_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

#ifdef CONFIG_BL602_SPI_DMA
  uint32_t rd;

  if (priv->dma_txchan >= 0 && priv->dma_rxchan >= 0)
    {
      rd = bl602_spi_dma_send(priv, wd);
    }
  else
    {
      rd = bl602_spi_poll_send(priv, wd);
    }

  return rd;
#else
  return bl602_spi_poll_send(priv, wd);
#endif
}

/****************************************************************************
 * Name: bl602_spi_poll_exchange
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

static void bl602_spi_poll_exchange(struct bl602_spi_priv_s *priv,
                                    const void *txbuffer,
                                    void *rxbuffer, size_t nwords)
{
  int i;
  uint32_t w_wd = 0xffff;
  uint32_t r_wd;

  for (i = 0; i < nwords; i++)
    {
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

      r_wd = bl602_spi_poll_send(priv, w_wd);

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
 * Name: bl602_spi_exchange
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

static void bl602_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

#ifdef CONFIG_BL602_SPI_DMA
  if (priv->dma_txchan >= 0 && priv->dma_rxchan >= 0)
    {
      bl602_spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
    {
      bl602_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
#else
  bl602_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
#endif
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: bl602_spi_sndblock
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

static void bl602_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  bl602_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: bl602_spi_recvblock
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

static void bl602_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  bl602_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: bl602_spi_trigger
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
static int bl602_spi_trigger(struct spi_dev_s *dev)
{
  spierr("SPI trigger not supported\n");
  DEBUGPANIC();

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: bl602_set_spi_0_act_mode_sel
 *
 * Description:
 *   set spi act mode
 *
 * Input Parameters:
 *   mod      - mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl602_set_spi_0_act_mode_sel(uint8_t mod)
{
  if (mod)
    {
      modifyreg32(BL602_GLB_GLB_PARM, 0, GLB_PARM_REG_SPI_0_MASTER_MODE);
    }
  else
    {
      modifyreg32(BL602_GLB_GLB_PARM, GLB_PARM_REG_SPI_0_MASTER_MODE, 0);
    }
}

/****************************************************************************
 * Name: bl602_swap_spi_0_mosi_with_miso
 *
 * Description:
 *   Swap SPI0 MOSI with MISO
 *
 * Input Parameters:
 *   swap      - Non-zero to swap MOSI and MISO
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl602_swap_spi_0_mosi_with_miso(uint8_t swap)
{
  if (swap)
    {
      modifyreg32(BL602_GLB_GLB_PARM, 0, GLB_PARM_REG_SPI_0_SWAP);
    }
  else
    {
      modifyreg32(BL602_GLB_GLB_PARM, GLB_PARM_REG_SPI_0_SWAP, 0);
    }
}

#ifdef CONFIG_BL602_SPI_DMA
/****************************************************************************
 * Name: bl602_spi_dma_init
 *
 * Description:
 *   Initialize BL SPI connection to DMA engine.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_spi_dma_init(struct spi_dev_s *dev)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

  #ifdef CONFIG_DEBUG_DMA_INFO
    struct bl602_dmaregs_s regs;
  #endif

  /* NOTE DMA channels are limited on this device and not all SPI devices
   * care about the RX side.  Consider making the RX channel optional.
   */

  /* Request a DMA channel for SPI peripheral */

  priv->dma_rxchan = bl602_dma_channel_request(bl602_dma_rx_callback,
                                               (void *)priv);
  if (priv->dma_rxchan < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      DEBUGASSERT(false);
      return;
    }

  priv->dma_txchan = bl602_dma_channel_request(bl602_dma_tx_callback,
                                               (void *)priv);
  if (priv->dma_txchan < 0)
    {
      spierr("Failed to allocate GDMA channel\n");

      /* Release the RX channel since we won't be using it. */

      bl602_dma_channel_release(priv->dma_rxchan);
      priv->dma_rxchan = -1;

      DEBUGASSERT(false);
      return;
    }

  /* Configure channels for SPI DMA */

  /* Configure channel from SPI to Mem */

  modifyreg32(
    BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, priv->dma_rxchan),
    DMA_C0CONFIG_FLOWCNTRL_MASK | \
    DMA_C0CONFIG_DSTPERIPHERAL_MASK | \
    DMA_C0CONFIG_SRCPERIPHERAL_MASK,
    (BL602_DMA_TRNS_P2M << DMA_C0CONFIG_FLOWCNTRL_SHIFT) | \
    (BL602_DMA_REQ_SPI_RX << DMA_C0CONFIG_SRCPERIPHERAL_SHIFT));

  /* Configure channel from Mem to SPI */

  modifyreg32(
    BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, priv->dma_txchan),
    DMA_C0CONFIG_FLOWCNTRL_MASK | DMA_C0CONFIG_DSTPERIPHERAL_MASK | \
    DMA_C0CONFIG_SRCPERIPHERAL_MASK,
    (BL602_DMA_TRNS_M2P << DMA_C0CONFIG_FLOWCNTRL_SHIFT) | \
    (BL602_DMA_REQ_SPI_TX << DMA_C0CONFIG_DSTPERIPHERAL_SHIFT));

  /* Set FIFO threshold to trigger DMA */

  modifyreg32(
    BL602_SPI_FIFO_CFG_1,
    SPI_FIFO_CFG_1_RX_TH_MASK | SPI_FIFO_CFG_1_TX_TH_MASK,
    (0 << SPI_FIFO_CFG_1_RX_TH_SHIFT) | (0 << SPI_FIFO_CFG_1_TX_TH_SHIFT));

  /* Enable DMA support */

  modifyreg32(BL602_SPI_FIFO_CFG_0, 0,
              SPI_FIFO_CFG_0_DMA_RX_EN | SPI_FIFO_CFG_0_DMA_TX_EN);

  /* Dump DMA register state */

  bl602_dmasample(&regs);
  bl602_dmadump(&regs, "Initialized DMA");
}
#endif

/****************************************************************************
 * Name: bl602_spi_init
 *
 * Description:
 *   Initialize bl602 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl602_spi_init(struct spi_dev_s *dev)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;
  const struct bl602_spi_config_s *config = priv->config;

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);
  nxsem_init(&priv->sem_isr_rx, 0, 0);
  nxsem_init(&priv->sem_isr_tx, 0, 0);

  bl602_configgpio(BOARD_SPI_CS);
  bl602_configgpio(BOARD_SPI_MOSI);
  bl602_configgpio(BOARD_SPI_MISO);
  bl602_configgpio(BOARD_SPI_CLK);

  /* set master mode */

  bl602_set_spi_0_act_mode_sel(1);

  /* swap MOSI with MISO to be consistent with BL602 Reference Manual */

  bl602_swap_spi_0_mosi_with_miso(1);

  /* spi cfg  reg:
   * cr_spi_deg_en 1
   * cr_spi_m_cont_en 0
   * cr_spi_byte_inv 0
   * cr_spi_bit_inv 0
   */

  modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_M_CONT_EN
              | SPI_CFG_CR_BYTE_INV | SPI_CFG_CR_BIT_INV,
              SPI_CFG_CR_DEG_EN);

  /* Disable rx ignore.
   * Enable continuous mode.
   */

  modifyreg32(BL602_SPI_CFG, SPI_CFG_CR_RXD_IGNR_EN, SPI_CFG_CR_M_CONT_EN);

  bl602_spi_setfrequency(dev, config->clk_freq);
  bl602_spi_setbits(dev, 8);
  bl602_spi_setmode(dev, config->mode);

  /* spi fifo clear */

  modifyreg32(BL602_SPI_FIFO_CFG_0, SPI_FIFO_CFG_0_RX_CLR
              | SPI_FIFO_CFG_0_TX_CLR, 0);

#ifdef CONFIG_BL602_SPI_DMA
  bl602_spi_dma_init(dev);
#endif
}

/****************************************************************************
 * Name: bl602_spi_deinit
 *
 * Description:
 *   Deinitialize bl602 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl602_spi_deinit(struct spi_dev_s *dev)
{
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

  bl602_swrst_ahb_slave1(AHB_SLAVE1_SPI);

  priv->frequency = 0;
  priv->actual = 0;
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 0;
  priv->dma_txchan = -1;
  priv->dma_rxchan = -1;
}

/****************************************************************************
 * Name: bl602_spibus_initialize
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

struct spi_dev_s *bl602_spibus_initialize(int port)
{
  struct spi_dev_s *spi_dev;
  struct bl602_spi_priv_s *priv;
  irqstate_t flags;

  switch (port)
    {
#ifdef CONFIG_BL602_SPI0
    case 0:
      priv = &bl602_spi_priv;
      break;
#endif
  default:
    return NULL;
    }

  spi_dev = (struct spi_dev_s *)priv;

  flags = enter_critical_section();

  if (priv->refs != 0)
    {
      leave_critical_section(flags);

      return spi_dev;
    }

  bl602_spi_init(spi_dev);

  priv->refs++;

  leave_critical_section(flags);

  return spi_dev;
}

/****************************************************************************
 * Name: bl602_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int bl602_spibus_uninitialize(struct spi_dev_s *dev)
{
  irqstate_t flags;
  struct bl602_spi_priv_s *priv = (struct bl602_spi_priv_s *)dev;

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

  bl602_spi_deinit(dev);

  nxsem_destroy(&priv->exclsem);
  nxsem_destroy(&priv->sem_isr_rx);
  nxsem_destroy(&priv->sem_isr_tx);

  return OK;
}

