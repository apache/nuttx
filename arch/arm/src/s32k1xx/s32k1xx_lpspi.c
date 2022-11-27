/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpspi.c
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
 * The external functions, s32k1xx_lpspiNselect* and s32k1xx_lpspiNstatus
 * must be provided by board-specific logic.  They are implementations of the
 * select and status methods of the SPI interface defined by struct
 * s32k1xx_lpspi_ops_s (see include/nuttx/spi/spi.h).  All other methods
 * (including s32k1xx_lpspibus_initialize()) are provided by common S32K1XX
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k1xx_lpspiNselect() and s32k1xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
 *
 * NOTE*: If CONFIG_S32K1XX_LPSPI_HWPCS is selected, s32k1xx_lpspiNselect()
 *        does NOT need to provided by board-specific logic.  In this case a
 *        generic implementation is used that switches between native
 *        hardware chip select pins.  It is important that all pins are
 *        configured when the SPI bus is initialized.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <endian.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "chip.h"

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "hardware/s32k1xx_lpspi.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_lpspi.h"

#include <arch/board/board.h>

#if defined(CONFIG_S32K1XX_LPSPI0) || defined(CONFIG_S32K1XX_LPSPI1) || \
    defined(CONFIG_S32K1XX_LPSPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

#if defined(CONFIG_S32K1XX_LPSPI_DMA)
#  error "DMA mode is not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_S32K1XX_LPSPI_INTERRUPTS) && defined(CONFIG_S32K1XX_LPSPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_S32K1XX_PM_SPI_ACTIVITY)
#  define CONFIG_S32K1XX_PM_SPI_ACTIVITY 10
#endif

#if defined(CONFIG_PM_SPI0_STANDBY) || defined(CONFIG_PM_SPI0_SLEEP)
#   define CONFIG_PM_SPI0
#endif
#if defined(CONFIG_PM_SPI1_STANDBY) || defined(CONFIG_PM_SPI1_SLEEP)
#   define CONFIG_PM_SPI1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k1xx_lpspidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part of the SPI interface */
  uint32_t spibase;           /* SPIn base address */
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  uint8_t spiirq;             /* SPI IRQ number */
#endif
  mutex_t lock;               /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  int8_t nbits;               /* Width of word in bits */
  uint8_t mode;               /* Mode 0,1,2,3 */
#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
  uint32_t pcs;               /* Peripheral Chip Select currently used */
#endif
};

enum s32k1xx_delay_e
{
  LPSPI_PCS_TO_SCK = 1,       /* PCS-to-SCK delay. */
  LPSPI_LAST_SCK_TO_PCS,      /* Last SCK edge to PCS delay. */
  LPSPI_BETWEEN_TRANSFER      /* Delay between transfers. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline
uint32_t s32k1xx_lpspi_getreg32(struct s32k1xx_lpspidev_s *priv,
                                uint8_t offset);
static inline
void s32k1xx_lpspi_putreg32(struct s32k1xx_lpspidev_s *priv,
                            uint8_t offset, uint32_t value);
static inline
uint32_t s32k1xx_lpspi_readword(struct s32k1xx_lpspidev_s *priv);
static inline
void s32k1xx_lpspi_writeword(struct s32k1xx_lpspidev_s *priv,
                             uint32_t byte);
static inline
uint16_t s32k1xx_lpspi_9to16bitmode(struct s32k1xx_lpspidev_s *priv);
static uint32_t s32k1xx_lpspi_pckfreq(uintptr_t base);
static inline
void s32k1xx_lpspi_set_delays(struct s32k1xx_lpspidev_s *priv,
                              uint32_t delay_ns,
                              enum s32k1xx_delay_e type);
static inline
void s32k1xx_lpspi_set_delay_scaler(struct s32k1xx_lpspidev_s *priv,
                                    uint32_t scaler,
                                    enum s32k1xx_delay_e type);

/* SPI methods */

static int s32k1xx_lpspi_lock(struct spi_dev_s *dev, bool lock);
#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
static void s32k1xx_lpspi_select(struct spi_dev_s *dev, uint32_t devid,
                                 bool selected);
#endif
static uint32_t s32k1xx_lpspi_setfrequency(struct spi_dev_s *dev,
              uint32_t frequency);
static void s32k1xx_lpspi_setmode(struct spi_dev_s *dev,
              enum spi_mode_e mode);
static void s32k1xx_lpspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int s32k1xx_lpspi_hwfeatures(struct spi_dev_s *dev,
              s32k1xx_lpspi_hwfeatures_t features);
#endif
static uint32_t s32k1xx_lpspi_send(struct spi_dev_s *dev, uint32_t wd);
static void s32k1xx_lpspi_exchange(struct spi_dev_s *dev,
              const void *txbuffer, void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_sndblock(struct spi_dev_s *dev,
              const void *txbuffer, size_t nwords);
static void s32k1xx_lpspi_recvblock(struct spi_dev_s *dev,
                                    void *rxbuffer,
                                    size_t nwords);
#endif

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/* Initialization */

static void
s32k1xx_lpspi_bus_initialize(struct s32k1xx_lpspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock         = s32k1xx_lpspi_lock,
#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
  .select       = s32k1xx_lpspi_select,
#else
  .select       = s32k1xx_lpspi0select,
#endif
  .setfrequency = s32k1xx_lpspi_setfrequency,
  .setmode      = s32k1xx_lpspi_setmode,
  .setbits      = s32k1xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k1xx_lpspi_hwfeatures,
#endif
  .status       = s32k1xx_lpspi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k1xx_lpspi0cmddata,
#endif
  .send         = s32k1xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k1xx_lpspi_exchange,
#else
  .sndblock     = s32k1xx_lpspi_sndblock,
  .recvblock    = s32k1xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k1xx_lpspi0register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k1xx_lpspidev_s g_lpspi0dev =
{
  .spidev       =
  {
    .ops        = &g_spi0ops,
  },
  .spibase      = S32K1XX_LPSPI0_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI0,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_S32K1XX_LPSPI_DMA
  .rxch         = DMAMAP_LPSPI0_RX,
  .txch         = DMAMAP_LPSPI0_TX,
#endif
};
#endif

#ifdef CONFIG_S32K1XX_LPSPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock         = s32k1xx_lpspi_lock,
#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
  .select       = s32k1xx_lpspi_select,
#else
  .select       = s32k1xx_lpspi1select,
#endif
  .setfrequency = s32k1xx_lpspi_setfrequency,
  .setmode      = s32k1xx_lpspi_setmode,
  .setbits      = s32k1xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k1xx_lpspi_hwfeatures,
#endif
  .status       = s32k1xx_lpspi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k1xx_lpspi1cmddata,
#endif
  .send         = s32k1xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k1xx_lpspi_exchange,
#else
  .sndblock     = s32k1xx_lpspi_sndblock,
  .recvblock    = s32k1xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k1xx_lpspi1register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k1xx_lpspidev_s g_lpspi1dev =
{
  .spidev       =
  {
    .ops        = &g_spi1ops,
  },
  .spibase      = S32K1XX_LPSPI1_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI1,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_S32K1XX_LPSPI_DMA
  .rxch         = DMAMAP_LPSPI1_RX,
  .txch         = DMAMAP_LPSPI1_TX,
#endif
};
#endif

#ifdef CONFIG_S32K1XX_LPSPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock         = s32k1xx_lpspi_lock,
#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
  .select       = s32k1xx_lpspi_select,
#else
  .select       = s32k1xx_lpspi2select,
#endif
  .setfrequency = s32k1xx_lpspi_setfrequency,
  .setmode      = s32k1xx_lpspi_setmode,
  .setbits      = s32k1xx_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = s32k1xx_lpspi_hwfeatures,
#endif
  .status       = s32k1xx_lpspi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = s32k1xx_lpspi2cmddata,
#endif
  .send         = s32k1xx_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = s32k1xx_lpspi_exchange,
#else
  .sndblock     = s32k1xx_lpspi_sndblock,
  .recvblock    = s32k1xx_lpspi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = s32k1xx_lpspi2register,  /* Provided externally */
#else
  .registercallback = 0,                       /* Not implemented */
#endif
};

static struct s32k1xx_lpspidev_s g_lpspi2dev =
{
  .spidev       =
  {
    .ops        = &g_spi2ops,
  },
  .spibase      = S32K1XX_LPSPI2_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI2,
#endif
  .lock         = NXMUTEX_INITIALIZER,
#ifdef CONFIG_S32K1XX_LPSPI_DMA
  .rxch         = DMAMAP_LPSPI2_RX,
  .txch         = DMAMAP_LPSPI2_TX,
#endif
};
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_spi1_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpspi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline
uint32_t s32k1xx_lpspi_getreg32(struct s32k1xx_lpspidev_s *priv,
                                uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline
void s32k1xx_lpspi_putreg32(struct s32k1xx_lpspidev_s *priv,
                            uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   word as read
 *
 ****************************************************************************/

static inline
uint32_t s32k1xx_lpspi_readword(struct s32k1xx_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) == 0)
    {
    }

  /* Then return the received byte */

  return (uint32_t) s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RDR_OFFSET);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_writeword
 *
 * Description:
 *   Write one word to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline
void s32k1xx_lpspi_writeword(struct s32k1xx_lpspidev_s *priv,
                             uint32_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the word */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, word);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_write_dword
 *
 * Description:
 *   Write two words to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word0, word1 - words to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI_DWORD

static inline void s32k1xx_lpspi_write_dword(struct s32k1xx_lpspidev_s
                                            *priv,
                                             uint32_t word0,
                                             uint32_t word1)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the words, use the FIFO */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, word0);
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, word1);
}

#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *   On the S32K the frame size can grow to 4096 bit/frame
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   value: frame size
 *
 ****************************************************************************/

static inline uint16_t s32k1xx_lpspi_9to16bitmode(
    struct s32k1xx_lpspidev_s *priv)
{
  uint16_t ret;

  ret = ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_TCR_OFFSET) &
        LPSPI_TCR_FRAMESZ_MASK) + 1);
  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_modifyreg
 *
 * Description:
 *   Clear and set bits in register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   offset  - Register offset
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_lpspi_modifyreg32(struct s32k1xx_lpspidev_s *priv,
                                      uint8_t offset, uint32_t clrbits,
                                      uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: s32k1xx_lpspi_pckfreq
 *
 * Description:
 *   Get the peripheral clock frequency for the LPSPI peripheral
 *
 * Input Parameters:
 *   base - The base address of the LPSPI peripheral registers
 *
 * Returned Value:
 *   The frequency of the LPSPI functional input frequency
 *   (or zero on a failure)
 *
 ****************************************************************************/

static uint32_t s32k1xx_lpspi_pckfreq(uintptr_t base)
{
  enum clock_names_e clkname;
  uint32_t pccclk;
  int ret;

  /* Get the PCC source clock */

#ifdef CONFIG_S32K1XX_LPSPI0
  if (base == S32K1XX_LPSPI0_BASE)
    {
      clkname = LPSPI0_CLK;
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPSPI1
  if (base == S32K1XX_LPSPI1_BASE)
    {
      clkname = LPSPI1_CLK;
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPSPI2
  if (base == S32K1XX_LPSPI2_BASE)
    {
      clkname = LPSPI2_CLK;
    }
  else
#endif
    {
      DEBUGPANIC();
      return -EINVAL;
    }

  ret = s32k1xx_get_pclkfreq(clkname, &pccclk);
  DEBUGASSERT(ret >= 0);
  if (ret < 0)
    {
      return 0;
    }

  return pccclk;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_set_delays
 *
 * Description:
 *   SET LPSPI Delay times
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   scaler - scaler value
 *   type   - delay time type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void s32k1xx_lpspi_set_delay_scaler(struct
                                                  s32k1xx_lpspidev_s *priv,
                                                  uint32_t scaler,
                                                  enum s32k1xx_delay_e type)
{
  switch (type)
    {
    case LPSPI_PCS_TO_SCK:
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET,
                              LPSPI_CCR_PCSSCK_MASK, 0);
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_PCSSCK(scaler));
      break;

    case LPSPI_LAST_SCK_TO_PCS:
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET,
                              LPSPI_CCR_SCKPCS_MASK, 0);
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_SCKPCS(scaler));
      break;

    case LPSPI_BETWEEN_TRANSFER:
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET,
                                LPSPI_CCR_DBT_MASK,
                                0);
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_DBT(scaler));
      break;
    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_set_delays
 *
 * Description:
 *   SET LPSPI Delay times
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *   delay_ns - delay time in nano seconds
 *   type     - delay time type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline
void s32k1xx_lpspi_set_delays(struct s32k1xx_lpspidev_s *priv,
                              uint32_t delay_ns,
                              enum s32k1xx_delay_e type)
{
  uint32_t inclock;
  uint64_t real_delay;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t diff;
  uint32_t min_diff;
  uint64_t initial_delay_ns;
  uint32_t clock_div_prescaler;
  uint32_t additional_scaler;

  /* Get the frequency of the LPSPI functional input clock */

  inclock = s32k1xx_lpspi_pckfreq(priv->spibase);
  DEBUGASSERT(inclock != 0);

  /* Get the pre-scaled input clock */

  clock_div_prescaler = inclock /
              (1 << ((s32k1xx_lpspi_getreg32(priv,
                      S32K1XX_LPSPI_TCR_OFFSET) &
              LPSPI_TCR_PRESCALE_MASK) >> LPSPI_TCR_PRESCALE_SHIFT));

  min_diff = 0xffffffff;

  /* Initialize scaler to max value to generate the max delay */

  best_scaler = 0xff;

  if (type == LPSPI_BETWEEN_TRANSFER)
    {
      /* First calculate the initial, default delay, note min delay is 2
       * clock cycles. Due to large size of * calculated values (uint64_t),
       * we need to break up the calculation into several steps to ensure
       * accurate calculated results
       */

      initial_delay_ns = 1000000000U;
      initial_delay_ns *= 2;
      initial_delay_ns /= clock_div_prescaler;

      additional_scaler = 1U;
    }
  else
    {
      /* First calculate the initial, default delay, min delay is 1 clock
       * cycle. Due to large size of calculated values (uint64_t), we need to
       * break up the calculation into several steps to ensure accurate
       * calculated * results.
       */

      initial_delay_ns = 1000000000U;
      initial_delay_ns /= clock_div_prescaler;

      additional_scaler = 0;
    }

  /* If the initial, default delay is already greater than the desired delay,
   * then * set the delay to their initial value (0) and return the delay. In
   * other words, * there is no way to decrease the delay value further.
   */

  if (initial_delay_ns >= delay_ns)
    {
      s32k1xx_lpspi_set_delay_scaler(priv, 0, type);
    }
  else
    {
      /* If min_diff = 0, the exit for loop */

      for (scaler = 0; (scaler < 256) && min_diff; scaler++)
        {
          /* Calculate the real delay value as we cycle through the scaler
           * values. Due to large size of calculated values (uint64_t),
           * we need to break up the calculation into several steps to ensure
           * accurate calculated results
           */

          real_delay  = 1000000000U;
          real_delay *= (scaler + 1 + additional_scaler);
          real_delay /= clock_div_prescaler;

          /* calculate the delay difference based on the conditional
           * statement that states that the calculated delay must not be less
           * then the desired delay
           */

          if (real_delay >= delay_ns)
            {
              diff = real_delay - delay_ns;
              if (min_diff > diff)
                {
                  /* A better match found */

                  min_diff = diff;
                  best_scaler = scaler;
                }
            }
        }

      s32k1xx_lpspi_set_delay_scaler(priv, best_scaler, type);
    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int s32k1xx_lpspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  int ret;

  /* It could be that this needs to be disabled for low level debugging */

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

#ifdef CONFIG_S32K1XX_LPSPI_HWPCS
/****************************************************************************
 * Name: s32k1xx_lpspi_select
 *
 * Description:
 *   Change to another SPI chip select (hardware/native, not emulated with
 *   GPIO) to select another device.  The hardware itself controls when
 *   the chip select is enabled or disabled.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - Ignored, selection is controlled by hardware
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_lpspi_select(struct spi_dev_s *dev, uint32_t devid,
                                 bool selected)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;

  /* LPSPI on S32K1XX supports PCS 0-3 */

  DEBUGASSERT(SPIDEVID_INDEX(devid) <= 3);

  /* Has the Peripheral Chip Select changed? */

  if (devid != priv->pcs)
    {
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET,
                                LPSPI_TCR_PCS_MASK,
                                LPSPI_TCR_PCS(SPIDEVID_INDEX(devid)));
      priv->pcs = devid;
    }

  spiinfo("devid: %" PRId32 ", CS: hardware-controlled\n", devid);
}
#endif /* CONFIG_S32K1XX_LPSPI HWPCS */

/****************************************************************************
 * Name: s32k1xx_lpspi_setfrequency
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

static uint32_t s32k1xx_lpspi_setfrequency(struct spi_dev_s *dev,
                                           uint32_t frequency)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;

  uint32_t men;
  uint32_t regval;
  uint32_t inclock;
  uint32_t prescaler;
  uint32_t best_prescaler;
  uint32_t scaler;
  uint32_t best_scaler;
  uint32_t real_frequency;
  uint32_t best_frequency;
  uint32_t diff;
  uint32_t min_diff;

  /* Has the LPSPI bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable LPSPI if it is enabled */

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET,
                                    LPSPI_CR_MEN, 0);
        }

      /* Get the frequency of the LPSPI functional input clock */

      inclock = s32k1xx_lpspi_pckfreq(priv->spibase);
      DEBUGASSERT(inclock != 0);

      min_diff       = 0xffffffff;
      best_prescaler = 7;
      best_scaler    = 255;
      best_frequency = 0;

      for (prescaler = 0; (prescaler < 8) && min_diff; prescaler++)
        {
          for (scaler = 0; (scaler < 256) && min_diff; scaler++)
            {
              real_frequency = inclock / ((1 << prescaler) * (scaler + 2));

              /* Calculate the frequency difference based on conditional
               * statement that states that the calculated frequency must not
               * exceed desired frequency.
               */

              if (frequency >= real_frequency)
                {
                  diff = frequency - real_frequency;
                  if (min_diff > diff)
                    {
                      /* A better match found */

                      min_diff = diff;
                      best_prescaler = prescaler;
                      best_scaler = scaler;
                      best_frequency = real_frequency;
                    }
                }
            }
        }

      /* Write the best values in the CCR register */

      regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CCR_OFFSET);
      regval &= ~LPSPI_CCR_SCKDIV_MASK;
      regval |= LPSPI_CCR_SCKDIV(best_scaler);
      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, regval);

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET,
                              LPSPI_TCR_PRESCALE_MASK,
                              LPSPI_TCR_PRESCALE(best_prescaler));

      priv->frequency = frequency;
      priv->actual = best_frequency;

      s32k1xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_PCS_TO_SCK);
      s32k1xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_LAST_SCK_TO_PCS);
      s32k1xx_lpspi_set_delays(priv, 1000000000 / best_frequency,
                                    LPSPI_BETWEEN_TRANSFER);

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0,
                                  LPSPI_CR_MEN);
        }
    }

  return priv->actual;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e mode for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void s32k1xx_lpspi_setmode(struct spi_dev_s *dev,
                                  enum spi_mode_e mode)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t men;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET,
                                    LPSPI_CR_MEN, 0);
        }

      switch (mode)
        {
        case SPIDEV_MODE0:     /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE1:     /* CPOL=0; CPHA=1 */
          setbits = LPSPI_TCR_CPHA;
          clrbits = LPSPI_TCR_CPOL;
          break;

        case SPIDEV_MODE2:     /* CPOL=1; CPHA=0 */
          setbits = LPSPI_TCR_CPOL;
          clrbits = LPSPI_TCR_CPHA;
          break;

        case SPIDEV_MODE3:     /* CPOL=1; CPHA=1 */
          setbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET,
                                clrbits, setbits);

      while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RSR_OFFSET) &
              LPSPI_RSR_RXEMPTY) != LPSPI_RSR_RXEMPTY)
        {
          /* Flush SPI read FIFO */

          s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RSR_OFFSET);
        }

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET,
                                    0, LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_lpspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t men;
  int savbits = nbits;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      if (nbits < 2 || nbits > 4096)
        {
          return;
        }

      /* Disable LPSPI if it is enabled */

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
                                   LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET,
                                    LPSPI_CR_MEN, 0);
        }

      regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_TCR_OFFSET);
      regval &= ~LPSPI_TCR_FRAMESZ_MASK;
      regval |= LPSPI_TCR_FRAMESZ(nbits - 1);

      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, regval);

      /* Save the selection so that subsequent re-configurations will
       * be faster.
       */

      priv->nbits = savbits;    /* nbits has been clobbered... save the signed
                                 * value. */

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET,
                                    0, LPSPI_CR_MEN);
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_lpspi_hwfeatures
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
static int s32k1xx_lpspi_hwfeatures(struct spi_dev_s *dev,
                                    s32k1xx_lpspi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  int savbits = nbits;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = LPSPI_TCR_LSBF;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = LPSPI_TCR_LSBF;
    }

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET,
                            clrbits, setbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t s32k1xx_lpspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  s32k1xx_lpspi_writeword(priv, wd);

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) &
          LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret = s32k1xx_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET);

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32
          " Status: %02" PRIx32 "\n", wd, ret, regval);

  UNUSED(regval);
  return ret;
}

/****************************************************************************
 * Name: s32k1xx_lpspi_send_dword
 *
 * Description:
 *   Exchange two words on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd0, wd1  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI_DWORD

static uint32_t s32k1xx_lpspi_send_dword(struct spi_dev_s *dev,
                                         uint32_t wd0, uint32_t wd1,
                                         uint32_t *rw1)
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  /* check if the receive buffer is empty, if not clear it */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_RDF))
    {
      s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RDR_OFFSET);
    }

  s32k1xx_lpspi_write_dword(priv, wd0, wd1);

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET)
         & LPSPI_SR_RDF) != LPSPI_SR_RDF);

  ret  = s32k1xx_lpspi_readword(priv);
  *rw1 = s32k1xx_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET);

  spiinfo("Sent: %04" PRIx32 " %04" PRIx32 " Return: %04"
          PRIx32 " %04" PRIx32 " Status: %02" PRIx32 "\n",
          wd0, wd1, ret, *rw1, regval);

  UNUSED(regval);
  return ret;
}

#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_exchange (no DMA).  aka s32k1xx_lpspi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_S32K1XX_LPSPI_DMA) || defined(CONFIG_S32K1XX_DMACAPABLE)
#if !defined(CONFIG_S32K1XX_LPSPI_DMA)
static void s32k1xx_lpspi_exchange(struct spi_dev_s *dev,
                                   const void *txbuffer,
                                   void *rxbuffer,
                                   size_t nwords)
#else
static void s32k1xx_lpspi_exchange_nodma(struct spi_dev_s *dev,
                                         const void *txbuffer,
                                         void *rxbuffer, size_t nwords)
#endif
{
  struct s32k1xx_lpspidev_s *priv = (struct s32k1xx_lpspidev_s *)dev;
  uint16_t framesize;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

#if defined(CONFIG_PM) && CONFIG_S32K1XX_PM_SPI_ACTIVITY > 0
  /* Report serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_S32K1XX_PM_SPI_ACTIVITY);
#endif

  /* bit mode? */

  framesize = s32k1xx_lpspi_9to16bitmode(priv);
  if (framesize > 16 && framesize % 32 != 0)
    {
      /* 17-bit or higher, byte transfer due to padding
       * take care of big endian mode of hardware !!
       */

      const uint8_t *src = txbuffer;
      uint8_t *dest = rxbuffer;
      uint32_t word = 0x0;
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
      uint32_t word1 = 0x0;
      uint32_t rword1;
      bool     dwords = false;
#endif

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              /* read the required number of bytes */

            switch (framesize)
              {
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
              case 40:
                   word = (src[0] << 24) + (src[1] << 16)
                          + (src[2] << 8) + src[3];
                   word1 = src[4];
                   src += 5;
                   dwords = true;
                   break;
#endif
              default:
                      break;
              }
            }
          else
            {
              word = 0xffffffff;
            }

#ifdef CONFIG_S32K1XX_LPSPI_DWORD
          /* Exchange 2 words */

          if (dwords)
            {
              word = s32k1xx_lpspi_send_dword(dev, word, word1, &rword1);
            }
          else
#endif
            {
              word = s32k1xx_lpspi_send(dev, word);
            }

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
            switch (framesize)
              {
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
              case 40:
                   dest[0] = (word >> 24) & 0xff;
                   dest[1] = (word >> 16) & 0xff;
                   dest[2] = (word >>  8) & 0xff;
                   dest[3] =  word        & 0xff;
                   dest[4] =  rword1      & 0xff;
                   dest += 5;
                   break;
#endif

              default:

                      break;
            }
          }
        }
    }
  else if (framesize > 16)
    {
      /* 32-bit or 64 bit, word size memory transfers */

      const uint32_t *src = txbuffer;
      uint32_t *dest = rxbuffer;
      uint32_t word = 0x0;
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
      uint32_t word1 = 0x0;
      uint32_t rword1;
      bool     dwords = false;
#endif

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              /* read the required number of bytes */

            switch (framesize)
              {
              case 32:
                   word = __builtin_bswap32(*src);
                   src += 4;
                   break;
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
              case 64:
                   word  = __builtin_bswap32(src[0]);
                   word1 = __builtin_bswap32(src[1]);
                   src += 8;
                   dwords = true;
#endif
              default:
                      break;
              }
            }
          else
            {
              word = 0xffffffff;
            }

#ifdef CONFIG_S32K1XX_LPSPI_DWORD
          /* Exchange 2 words */

          if (dwords)
            {
              word = s32k1xx_lpspi_send_dword(dev, word, word1, &rword1);
            }
          else
#endif
            {
            word = s32k1xx_lpspi_send(dev, word);
            }

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
            switch (framesize)
              {
              case 32:
                   *dest = __builtin_bswap32(word);
                   dest += 4;
                   break;
#ifdef CONFIG_S32K1XX_LPSPI_DWORD
              case 64:
                   dest[0] = __builtin_bswap32(word);
                   dest[1] = __builtin_bswap32(rword1);
                   dest += 8;
                   break;
#endif

              default:

                      break;
            }
          }
        }
    }
  else if (framesize > 8)
    {
      /* 16-bit mode */

      const uint16_t *src = txbuffer;
      uint16_t *dest = rxbuffer;
      uint16_t word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = swap16(*src++);

              /* read the required number of bytes */
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t) s32k1xx_lpspi_send(dev, (uint32_t) word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = swap16(word);
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src = txbuffer;
      uint8_t *dest = rxbuffer;
      uint8_t word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xff;
            }

          /* Exchange one word */

          word = (uint8_t) s32k1xx_lpspi_send(dev, (uint32_t) word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}
#endif /* !CONFIG_S32K1XX_LPSPI_DMA || CONFIG_S32K1XX_DMACAPABLE */

/****************************************************************************
 * Name: s32k1xx_lpspi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_sndblock(struct spi_dev_s *dev,
                                   const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return s32k1xx_lpspi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number
 *              of bits-per-word selected for the SPI interface.  If
 *              nbits <= 8, the data is packed into uint8_t's; if nbits >8,
 *              the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_recvblock(struct spi_dev_s *dev,
                                    void *rxbuffer,
                                    size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return s32k1xx_lpspi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: s32k1xx_lpspi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void s32k1xx_lpspi_bus_initialize(struct s32k1xx_lpspidev_s *priv)
{
  uint32_t reg = 0;

  /* NOTE:
   * Clocking to the LPSPI peripheral must be provided by board-specific
   * logic as part of the clock configuration logic.
   */

  /* Reset to known status */

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_RST);
  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0,
                          LPSPI_CR_RTF | LPSPI_CR_RRF);
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0x00);

  /* Set LPSPI to master */

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CFGR1_OFFSET, 0,
                          LPSPI_CFGR1_MASTER);

  /* Set specific PCS to active high or low */

  /* TODO: Not needed for now */

  /* Set Configuration Register 1 related setting. */

  reg = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CFGR1_OFFSET);
  reg &= ~(LPSPI_CFGR1_OUTCFG |
           LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_NOSTALL);
  reg |= LPSPI_CFGR1_OUTCFG_RETAIN | LPSPI_CFGR1_PINCFG_SIN_SOUT;
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CFGR1_OFFSET, reg);

  /* Set frequency and delay times */

  s32k1xx_lpspi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set default watermarks */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_FCR_OFFSET,
                       LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));

  /* Set Transmit Command Register */

  s32k1xx_lpspi_setbits((struct spi_dev_s *)priv, 8);

  s32k1xx_lpspi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Enable LPSPI */

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
# ifdef CONFIG_PM_SPI0

  struct s32k1xx_lpspidev_s *priv0 = NULL;

  /* make the priv1ate struct for lpspi bus 0 */

  priv0 = &g_lpspi0dev;

# endif
# ifdef CONFIG_PM_SPI1

  struct s32k1xx_lpspidev_s *priv1 = NULL;

  /* make the priv1ate struct for lpspi bus 1  */

  priv1 = &g_lpspi1dev;

# endif

  unsigned int count = 0;   /* the amount of peripheral clocks to change */

  peripheral_clock_source_t clock_source;

  /* check if the transition is from the IDLE domain to the NORMAL domain */

  /* or the mode is already done */

  if (((pm_querystate(PM_IDLE_DOMAIN) == PM_IDLE) &&
    (pmstate == PM_NORMAL)) ||
    (((pm_querystate(PM_IDLE_DOMAIN) == pmstate))))
    {
      /* return */

      return;
    }

  /* check which PM it is  */

  switch (pmstate)
  {
    /* in case it needs to change to the RUN mode */

    case PM_NORMAL:
    {
      /* Logic for PM_NORMAL goes here */

      /* set the right clock source to go back to RUN mode */

      clock_source = CLK_SRC_SPLL_DIV2;

# ifdef CONFIG_PM_SPI0

      /* add 1 to count to do it for SPI0 */

      count++;
# endif

# ifdef CONFIG_PM_SPI1

      /* add 1 to count to do it for SPI1 */

      count++;
# endif
    }
    break;

    default:
    {
      /* don't do anything, just return OK */
    }
    break;
  }

  /* check if the LPSPI needs to change */

  if (count)
  {
    /* make the peripheral clock config struct */

    const struct peripheral_clock_config_s clock_config[] =
    {
# ifdef CONFIG_PM_SPI0

      {
        .clkname  =   LPSPI0_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      },
# endif
# ifdef CONFIG_PM_SPI1

      {
        .clkname  =   LPSPI1_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      }
# endif
    };

# ifdef CONFIG_PM_SPI0

    /* disable LPSP0 */

    s32k1xx_lpspi_modifyreg32(priv0, S32K1XX_LPSPI_CR_OFFSET, 0,
                              !LPSPI_CR_MEN);

# endif
# ifdef CONFIG_PM_SPI1

    /* disable LPSPI */

    s32k1xx_lpspi_modifyreg32(priv1, S32K1XX_LPSPI_CR_OFFSET, 0,
                              !LPSPI_CR_MEN);

# endif

    /* change the clock config for the new mode */

    s32k1xx_periphclocks(count, clock_config);

# ifdef CONFIG_PM_SPI0

    /* Enable LPSP0 */

    s32k1xx_lpspi_modifyreg32(priv0, S32K1XX_LPSPI_CR_OFFSET, 0,
                              LPSPI_CR_MEN);

# endif
# ifdef CONFIG_PM_SPI1

    /* Enable LPSPI */

    s32k1xx_lpspi_modifyreg32(priv1, S32K1XX_LPSPI_CR_OFFSET, 0,
                              LPSPI_CR_MEN);
# endif

    /* get the clock freq */
  }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

# ifdef CONFIG_PM_SPI0
  struct s32k1xx_lpspidev_s *priv0 = NULL;

  /* make the private struct for lpspi bus 0 */

  priv0 = &g_lpspi0dev;
# endif
# ifdef CONFIG_PM_SPI1
  struct s32k1xx_lpspidev_s *priv1 = NULL;

  /* make the private struct for lpspi bus 1  */

  priv1 = &g_lpspi1dev;
# endif

  unsigned int count = 0;   /* the amount of peripheral clocks to change */

  peripheral_clock_source_t clock_source;

  /* check if the transition to the mode is already done */

  if (pm_querystate(PM_IDLE_DOMAIN) == pmstate)
    {
      /* return */

      return OK;
    }

  /* check which PM it is  */

  switch (pmstate)
  {
    /* in case it needs to prepare for VLPR mode */

    case PM_STANDBY:
    {
      /* Logic for PM_STANDBY goes here */

       /* set the right clock source */

      clock_source = CLK_SRC_SIRC_DIV2;

# ifdef CONFIG_PM_SPI0_STANDBY

      /* increase count to change the SPI0  */

      count++;

# endif
# ifdef CONFIG_PM_SPI1_STANDBY

      /* increase count to change the SPI1 */

      count++;

# endif
    }
    break;

    /* in case it needs to prepare for VLPR mode */

    case PM_SLEEP:
    {
      /* Logic for PM_STANDBY goes here */

      /* set the right clock source */

      clock_source = CLK_SRC_SIRC_DIV2;

# ifdef CONFIG_PM_SPI0_SLEEP

      /* increase count to change the SPI0  */

      count++;

# endif
# ifdef CONFIG_PM_SPI1_SLEEP

      /* increase count to change the SPI1 */

      count++;

# endif     
    }
    break;

    default:
    {
      /* don't do anything, just return OK */
    }
    break;
  }

  /* check if you need to change something */

  if (count)
  {
    /* make the peripheral clock config struct */

    const struct peripheral_clock_config_s clock_config[] =
    {
# ifdef CONFIG_PM_SPI0
      {
        .clkname  =   LPSPI0_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      },
# endif
# ifdef CONFIG_PM_SPI1
      {
        .clkname  =   LPSPI1_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      }
# endif
    };

# ifdef CONFIG_PM_SPI0

    /* disable LPSPI0 */

    s32k1xx_lpspi_modifyreg32(priv0, S32K1XX_LPSPI_CR_OFFSET, 0,
                                     !LPSPI_CR_MEN);

# endif
# ifdef CONFIG_PM_SPI1

    /* disable LPSPI1 */

    s32k1xx_lpspi_modifyreg32(priv1, S32K1XX_LPSPI_CR_OFFSET, 0,
                              !LPSPI_CR_MEN);

# endif

    /* change the clock config for the new mode */

    s32k1xx_periphclocks(count, clock_config);

# ifdef CONFIG_PM_SPI0

    /* Enable LPSPI */

    s32k1xx_lpspi_modifyreg32(priv0, S32K1XX_LPSPI_CR_OFFSET, 0,
                              LPSPI_CR_MEN);

# endif
# ifdef CONFIG_PM_SPI1

    /* Enable LPSPI */

    s32k1xx_lpspi_modifyreg32(priv1, S32K1XX_LPSPI_CR_OFFSET, 0,
                              LPSPI_CR_MEN);

# endif
  }

  /* get the clock freq */

  /* return OK */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_lpspibus_initialize
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

struct spi_dev_s *s32k1xx_lpspibus_initialize(int bus)
{
  struct s32k1xx_lpspidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_S32K1XX_LPSPI0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_lpspi0dev;

      /* Only configure if the bus is not already configured */

      if ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          s32k1xx_pinconfig(PIN_LPSPI0_SCK);
          s32k1xx_pinconfig(PIN_LPSPI0_MISO);
          s32k1xx_pinconfig(PIN_LPSPI0_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k1xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPSPI1
  if (bus == 1)
    {
      #ifdef CONFIG_PM
        #if defined(CONFIG_PM_SPI_STANDBY) || defined(CONFIG_PM_SPI_SLEEP) 
          int ret;

          /* Register to receive power management callbacks */

          ret = pm_register(&g_spi1_pmcb);
          DEBUGASSERT(ret == OK);
          UNUSED(ret);
        #endif
      #endif
      /* Select SPI1 */

      priv = &g_lpspi1dev;

      /* Only configure if the bus is not already configured */

      if ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          s32k1xx_pinconfig(PIN_LPSPI1_SCK);
          s32k1xx_pinconfig(PIN_LPSPI1_MISO);
          s32k1xx_pinconfig(PIN_LPSPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k1xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_S32K1XX_LPSPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_lpspi2dev;

      /* Only configure if the bus is not already configured */

      if ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) &
          LPSPI_CR_MEN) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          s32k1xx_pinconfig(PIN_LPSPI2_SCK);
          s32k1xx_pinconfig(PIN_LPSPI2_MISO);
          s32k1xx_pinconfig(PIN_LPSPI2_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          s32k1xx_lpspi_bus_initialize(priv);
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

  leave_critical_section(flags);

  return (struct spi_dev_s *)priv;
}

#endif /* CONFIG_S32K1XX_LPSPI0 || CONFIG_S32K1XX_LPSPI1 || CONFIG_S32K1XX_LPSPI2 */
