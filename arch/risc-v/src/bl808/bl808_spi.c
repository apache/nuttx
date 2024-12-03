/****************************************************************************
 * arch/risc-v/src/bl808/bl808_spi.c
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "bl808_gpio.h"
#include "bl808_spi.h"
#include "hardware/bl808_glb.h"
#include "hardware/bl808_mm_glb.h"
#include "hardware/bl808_spi.h"
#include "riscv_internal.h"

/* This file is based on bl602/bl602_spi.c */

#if defined(CONFIG_BL808_SPI0) || defined(CONFIG_BL808_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_FREQ_DEFAULT 400000
#define SPI_CLK_PRE_DIV 160000000

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

/* SPI Device hardware configuration */

struct bl808_spi_config_s
{
  uint32_t clk_freq;      /* SPI clock frequency */
  enum spi_mode_e mode;   /* SPI default mode */

  bool deglitch_enable;   /* Enable or disable de-glitch function */
  bool continuous_enable; /* Enable or disable master continuous transfer
                           * mode,enable:SS will stay asserted if next data
                           * is valid
                           */
  bool byte_invert;       /* The byte is sent first in SPI transfer ,0 is send
                           * 0byte first; 1 is send 3byte first
                           */
  bool bit_invert;        /* The bit is sent first in SPI transfer ,0 is each byte
                           * is sent out MSB first; 1 is each byte is sent out LSB
                           * first
                           */
};

struct bl808_spi_priv_s
{
  /* Externally visible part of the SPI interface */

  struct spi_dev_s spi_dev;

  /* SPI block ID */

  uint8_t idx;

  /* Port configuration */

  const struct bl808_spi_config_s *config;

  int refs; /* Referernce count */

  /* Held while chip is selected for mutual exclusion */

  mutex_t lock;

  uint32_t frequency; /* Requested clock frequency */
  uint32_t actual;    /* Actual clock frequency */

  enum spi_mode_e mode; /* Actual SPI hardware mode */

  /* Actual SPI send/receive bits once transmission */

  uint8_t nbits;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl808_spi_lock(struct spi_dev_s *dev, bool lock);

static void bl808_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected);

static uint32_t bl808_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void bl808_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void bl808_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int bl808_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t bl808_spi_status(struct spi_dev_s *dev,
                                uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int bl808_spi_cmddata(struct spi_dev_s *dev,
                              uint32_t devid, bool cmd);
#endif
static uint32_t bl808_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void bl808_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void bl808_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords);
static void bl808_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int bl808_spi_trigger(struct spi_dev_s *dev);
#endif
static void bl808_spi_init(struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s bl808_spi_ops =
{
    .lock = bl808_spi_lock,
    .select = bl808_spi_select,
    .setfrequency = bl808_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
    .setdelay = bl808_spi_setdelay,
#endif
    .setmode = bl808_spi_setmode,
    .setbits = bl808_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures = bl808_spi_hwfeatures,
#endif
    .status = bl808_spi_status,
#ifdef CONFIG_SPI_CMDDATA
    .cmddata = bl808_spi_cmddata,
#endif
    .send = bl808_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange = bl808_spi_exchange,
#else
    .sndblock = bl808_spi_sndblock,
    .recvblock = bl808_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
    .trigger = bl808_spi_trigger,
#endif
    .registercallback = NULL,
};

#ifdef CONFIG_BL808_SPI0
static const struct bl808_spi_config_s bl808_spi0_config =
{
    .clk_freq = SPI_FREQ_DEFAULT,
    .mode = SPIDEV_MODE0,

#ifdef CONFIG_BL808_SPI0_DEG_ENABLE
    .deglitch_enable = 1,
#else
    .deglitch_enable = 0,
#endif

#ifdef CONFIG_BL808_SPI0_CONT_ENABLE
    .continuous_enable = 1,
#else
    .continuous_enable = 0,
#endif

#ifdef CONFIG_BL808_SPI0_BYTE_INV
    .byte_invert = 1,
#else
    .byte_invert = 0,
#endif

#ifdef CONFIG_BL808_SPI0_BIT_INV
    .bit_invert = 1,
#else
    .bit_invert = 0,
#endif
};

static struct bl808_spi_priv_s bl808_spi0_priv =
{
  .spi_dev =
  {
    .ops      = &bl808_spi_ops
  },
  .idx        = 0,
  .config     = &bl808_spi0_config,
  .lock       = NXMUTEX_INITIALIZER,
};

#endif  /* CONFIG_BL808_SPI0 */

#ifdef CONFIG_BL808_SPI1
static const struct bl808_spi_config_s bl808_spi1_config =
{
    .clk_freq = SPI_FREQ_DEFAULT,
    .mode = SPIDEV_MODE0,

#ifdef CONFIG_BL808_SPI1_DEG_ENABLE
    .deglitch_enable = 1,
#else
    .deglitch_enable = 0,
#endif

#ifdef CONFIG_BL808_SPI1_CONT_ENABLE
    .continuous_enable = 1,
#else
    .continuous_enable = 0,
#endif

#ifdef CONFIG_BL808_SPI1_BYTE_INV
    .byte_invert = 1,
#else
    .byte_invert = 0,
#endif

#ifdef CONFIG_BL808_SPI1_BIT_INV
    .bit_invert = 1,
#else
    .bit_invert = 0,
#endif
};

static struct bl808_spi_priv_s bl808_spi1_priv =
{
  .spi_dev =
  {
    .ops      = &bl808_spi_ops
  },
  .idx        = 1,
  .config     = &bl808_spi1_config,
  .lock       = NXMUTEX_INITIALIZER,
};

#endif  /* CONFIG_BL808_SPI1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_check_with_new_prescale
 *
 * Description:
 *   Check if the option prescale
 *
 ****************************************************************************/

static void
bl808_check_with_new_prescale(struct prescale_and_count_cal_ctx_s *p_ctx,
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
 * Name: bl808_prescale_and_count_cal
 *
 * Description:
 *   prescale and count cal
 *
 ****************************************************************************/

static int
bl808_prescale_and_count_cal(uint32_t counter_width,
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
      bl808_check_with_new_prescale(&ctx, prescale);
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
 * Name: bl808_set_spi_clk
 *
 * Description:
 *   set SPI clock
 *
 ****************************************************************************/

static void bl808_set_spi_clk(uint8_t div, uint8_t idx)
{
  if (idx == 0)
    {
      modifyreg32(BL808_GLB_SPI_CFG0,
                  SPI_CFG_CLK_DIV_MASK,
                  (div << SPI_CFG_CLK_DIV_SHIFT));
    }
  else
    {
      modifyreg32(BL808_MM_GLB_CLK_CTRL_PERI,
                  CLK_CTRL_PERI_SPI_DIV_MASK,
                  (div << CLK_CTRL_PERI_SPI_DIV_SHIFT));
    }
}

/****************************************************************************
 * Name: bl808_clockconfig
 *
 * Description:
 *   SPI clock config
 *
 ****************************************************************************/

static void bl808_clockconfig(struct spi_clock_cfg_s *clockcfg, uint8_t idx)
{
  /* Configure length of data phase1/0 and start/stop condition */

  modifyreg32(BL808_SPI_PRD_0(idx), SPI_PRD_0_CR_S_MASK,
              (clockcfg->start_len - 1));
  modifyreg32(BL808_SPI_PRD_0(idx), SPI_PRD_0_CR_P_MASK,
              (clockcfg->stop_len - 1) << SPI_PRD_0_CR_P_SHIFT);
  modifyreg32(BL808_SPI_PRD_0(idx), SPI_PRD_0_CR_D_PH_0_MASK,
              (clockcfg->data_phase0_len - 1) << SPI_PRD_0_CR_D_PH_0_SHIFT);
  modifyreg32(BL808_SPI_PRD_0(idx), SPI_PRD_0_CR_D_PH_1_MASK,
              (clockcfg->data_phase1_len - 1) << SPI_PRD_0_CR_D_PH_1_SHIFT);

  /* Configure length of interval between frame */

  modifyreg32(BL808_SPI_PRD_1(idx), SPI_PRD_1_CR_I_MASK,
              clockcfg->interval_len - 1);
}

/****************************************************************************
 * Name: bl808_spi_lock
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

static int bl808_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;

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
 * Name: bl808_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If disable bl808_SPI_SWCS, driver will use hardware CS so that when
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

static void bl808_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  /* we used hardware CS */

  spiinfo("devid: %u, CS: %s\n", devid, selected ? "select" : "free");

#ifdef CONFIG_SPI_CMDDATA
  /* revert MISO from GPIO Pin to SPI Pin */

  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  if (!selected)
    {
#ifdef CONFIG_BL808_SPI0
      if (priv->idx == 0)
        {
          bl808_configgpio(CONFIG_BL808_SPI0_MISO,
                           GPIO_INPUT
                           | GPIO_DRV_1
                           | GPIO_SMT_EN
                           | GPIO_PULLUP
                           | GPIO_FUNC_SPI0);
        }
#endif

#ifdef CONFIG_BL808_SPI1
      if (priv->idx == 1)
        {
          bl808_configgpio(CONFIG_BL808_SPI1_MISO,
                           GPIO_INPUT
                           | GPIO_DRV_1
                           | GPIO_SMT_EN
                           | GPIO_PULLUP
                           | GPIO_FUNC_SPI0);
        }
#endif
    }
#endif
}

/****************************************************************************
 * Name: bl808_spi_setfrequency
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

static uint32_t bl808_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  uint8_t idx = priv->idx;
  struct spi_clock_cfg_s clockcfg;
  size_t count;
  uint32_t ticks;
  uint32_t clk_div;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */

      return priv->actual;
    }

  ticks = SPI_CLK_PRE_DIV / frequency;

  /* Width of SPI1 clk div is 8, vs 5 for SPI0 */

  uint32_t max_div = (idx == 0) ? 32 : 256;

  if (bl808_prescale_and_count_cal(8, max_div, ticks, &clk_div, &count) != 0)
    {
      spierr("SPI div clk error\n");
      DEBUGPANIC();

      return -1;
    }

  bl808_set_spi_clk(1, clk_div - 1);

  clockcfg.start_len = count;
  clockcfg.stop_len = count;
  clockcfg.data_phase0_len = count;
  clockcfg.data_phase1_len = count;
  clockcfg.interval_len = count;

  bl808_clockconfig(&clockcfg, idx);

  priv->frequency = frequency;

  spiinfo("frequency=%u, actual=%u\n", priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: bl808_spi_setdelay
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
static int bl808_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                                uint32_t stopdelay, uint32_t csdelay,
                                uint32_t ifdelay)
{
  spierr("SPI CS delay control not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: bl808_spi_setmode
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
bl808_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  uint8_t idx = priv->idx;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
          /* NOTE: CPHA definition in the register is inverted compared
           * to the standard. See reference manual or bouffalo_sdk.
           */

        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_SCLK_POL,
                      SPI_CFG_CR_SCLK_PH);
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_SCLK_POL
                      | SPI_CFG_CR_SCLK_PH, 0);
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          modifyreg32(BL808_SPI_CFG(idx), 0, SPI_CFG_CR_SCLK_POL
                      | SPI_CFG_CR_SCLK_PH);
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_SCLK_PH,
                      SPI_CFG_CR_SCLK_POL);
          break;

        default:
          return;
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: bl808_spi_setbits
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

static void bl808_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  uint8_t idx = priv->idx;

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

        modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_FRAME_SIZE_MASK, 0);
        break;

      case 16:
        modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_FRAME_SIZE_MASK,
                    1 << SPI_CFG_CR_FRAME_SIZE_SHIFT);
        break;

      case 24:
        modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_FRAME_SIZE_MASK,
                    2 << SPI_CFG_CR_FRAME_SIZE_SHIFT);
        break;

      case 32:
        modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_FRAME_SIZE_MASK,
                    3 << SPI_CFG_CR_FRAME_SIZE_SHIFT);
        break;

      default:
        return;
      }

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: bl808_spi_status
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

static uint8_t bl808_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

/****************************************************************************
 * Name: bl808_spi_cmddata
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
 *   MISO to high (data) or low (command). bl808_spi_select() will revert
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
static int bl808_spi_cmddata(struct spi_dev_s *dev,
                              uint32_t devid, bool cmd)
{
  spiinfo("devid: %" PRIu32 " CMD: %s\n", devid, cmd ? "command" :
          "data");
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;

  if (devid == SPIDEV_DISPLAY(0))
    {
      gpio_pinset_t gpio;
      int ret;

      /* reconfigure MISO from SPI Pin to GPIO Pin,
       * then write 0 for command or 1 for data
       */

#ifdef CONFIG_BL808_SPI0
      if (priv->idx == 0)
        {
          bl808_configgpio(BL808_SPI0_MISO, GPIO_OUTPUT
                           | GPIO_PULLUP
                           | GPIO_FUNC_SWGPIO);

          bl808_gpiowrite(BL808_SPI0_MISO, !cmd);
        }
#endif

#ifdef CONFIG_BL808_SPI1
      if (priv->idx == 1)
        {
          bl808_configgpio(BL808_SPI1_MISO, GPIO_OUTPUT
                           | GPIO_PULLUP
                           | GPIO_FUNC_SWGPIO);

          bl808_gpiowrite(BL808_SPI1_MISO, !cmd);
        }
#endif

      return OK;
    }

  spierr("SPI cmddata not supported\n");
  DEBUGPANIC();

  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: bl808_spi_hwfeatures
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
static int bl808_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  spierr("SPI hardware specific feature not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: bl808_spi_poll_send
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

static uint32_t bl808_spi_poll_send(struct bl808_spi_priv_s *priv,
                                    uint32_t wd)
{
  uint8_t idx = priv->idx;
  uint32_t val;
  uint32_t tmp_val = 0;

  /* spi fifo clear  */

  modifyreg32(BL808_SPI_FIFO_CFG_0(idx), 0,
              SPI_FIFO_CFG_0_RX_CLR
              | SPI_FIFO_CFG_0_TX_CLR);

  /* write data to tx fifo */

  putreg32(wd, BL808_SPI_FIFO_WDATA(idx));

  /* spi enable master */

  modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_S_EN, SPI_CFG_CR_M_EN);

  while (0 == tmp_val)
    {
      /* get data from rx fifo */

      tmp_val = getreg32(BL808_SPI_FIFO_CFG_1(idx));
      tmp_val = (tmp_val & SPI_FIFO_CFG_1_RX_CNT_MASK)
                >> SPI_FIFO_CFG_1_RX_CNT_SHIFT;
    }

  val = getreg32(BL808_SPI_FIFO_RDATA(idx));

  spiinfo("send=%x and recv=%x\n", wd, val);

  modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_M_EN, 0);

  return val;
}

/****************************************************************************
 * Name: bl808_spi_send
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

static uint32_t bl808_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  return bl808_spi_poll_send(priv, wd);
}

/****************************************************************************
 * Name: bl808_spi_poll_exchange
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

static void bl808_spi_poll_exchange(struct bl808_spi_priv_s *priv,
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

      r_wd = bl808_spi_poll_send(priv, w_wd);

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
 * Name: bl808_spi_exchange
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

static void bl808_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  bl808_spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: bl808_spi_sndblock
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

static void bl808_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  bl808_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: bl808_spi_recvblock
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

static void bl808_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  bl808_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: bl808_spi_trigger
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
static int bl808_spi_trigger(struct spi_dev_s *dev)
{
  spierr("SPI trigger not supported\n");
  DEBUGPANIC();

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: bl808_spi_init
 *
 * Description:
 *   Initialize bl808 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl808_spi_init(struct spi_dev_s *dev)
{
  struct bl808_spi_priv_s *priv = (struct bl808_spi_priv_s *)dev;
  const struct bl808_spi_config_s *config = priv->config;
  uint8_t idx = priv->idx;

#ifdef CONFIG_BL808_SPI0
  if (idx == 0)
    {
      bl808_configgpio(CONFIG_BL808_SPI0_MISO,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI0);

      bl808_configgpio(CONFIG_BL808_SPI0_MOSI,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI0);

      bl808_configgpio(CONFIG_BL808_SPI0_SCLK,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI0);

      bl808_configgpio(CONFIG_BL808_SPI0_SS,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI0);

      modifyreg32(BL808_GLB_PARM_CFG0, 0,
                  1 << PARM_SPI_0_MASTER_MODE_SHIFT);
    }
#endif

#ifdef CONFIG_BL808_SPI1
  if (idx == 1)
    {
      bl808_configgpio(CONFIG_BL808_SPI1_MISO,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI1);

      bl808_configgpio(CONFIG_BL808_SPI1_MOSI,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI1);

      bl808_configgpio(CONFIG_BL808_SPI1_SCLK,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI1);

      bl808_configgpio(CONFIG_BL808_SPI1_SS,
                       GPIO_INPUT
                       | GPIO_DRV_1
                       | GPIO_SMT_EN
                       | GPIO_PULLUP
                       | GPIO_FUNC_SPI1);

      modifyreg32(BL808_GLB_PARM_CFG0, 0,
                  1 << PARM_MM_SPI_MASTER_MODE_SHIFT);
    }
#endif

  /* Disable RX ignore */

  modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_RXD_IGNR_EN, 0);

  /* Set deglitch */

  if (config->deglitch_enable)
    {
      modifyreg32(BL808_SPI_CFG(idx), 0, SPI_CFG_CR_DEG_EN);
    }
  else
    {
      modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_DEG_EN, 0);
    }

  /* Set continuous transfer */

  if (config->continuous_enable)
    {
      modifyreg32(BL808_SPI_CFG(idx), 0, SPI_CFG_CR_M_CONT_EN);
    }
  else
    {
      modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_M_CONT_EN, 0);
    }

  /* Set byte inversion */

  if (config->byte_invert)
    {
      modifyreg32(BL808_SPI_CFG(idx), 0, SPI_CFG_CR_BYTE_INV);
    }
  else
    {
      modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_BYTE_INV, 0);
    }

  /* Set bit inversion */

  if (config->bit_invert)
    {
      modifyreg32(BL808_SPI_CFG(idx), 0, SPI_CFG_CR_BIT_INV);
    }
  else
    {
      modifyreg32(BL808_SPI_CFG(idx), SPI_CFG_CR_BIT_INV, 0);
    }

  bl808_spi_setfrequency(dev, config->clk_freq);
  bl808_spi_setbits(dev, 8);
  bl808_spi_setmode(dev, config->mode);

  /* spi fifo clear */

  modifyreg32(BL808_SPI_FIFO_CFG_0(idx), 0, SPI_FIFO_CFG_0_RX_CLR
              | SPI_FIFO_CFG_0_TX_CLR);
}

/****************************************************************************
 * Name: bl808_spibus_initialize
 *
 * Description:
 *   Initialize and register the configured SPI busses
 *
 ****************************************************************************/

struct spi_dev_s *bl808_spibus_initialize(int bus)
{
  struct spi_dev_s *spi_dev;
  struct bl808_spi_priv_s *priv;

#ifdef CONFIG_BL808_SPI0
  if (bus == 0)
    {
      priv = &bl808_spi0_priv;
    }
#endif

#ifdef CONFIG_BL808_SPI1
  if (bus == 1)
    {
      priv = &bl808_spi1_priv;
    }
#endif

  spi_dev = (struct spi_dev_s *)priv;

  nxmutex_lock(&priv->lock);
  if (priv->refs == 0)
    {
      bl808_spi_init(spi_dev);
    }

  priv->refs++;

  nxmutex_unlock(&priv->lock);

  return spi_dev;
}

#endif /* CONFIG_BL808_SPI0 || CONFIG_BL808_SPI1 */
