/************************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpspi.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Teodora Kireva
 *            Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * The external functions, s32k1xx_lpspi0/1/2select and s32k1xx_lpspi0/1/2status
 * must be provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct s32k1xx_lpspi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including
 * s32k1xx_lpspibus_initialize()) are provided by common S32K1XX logic.  To use this
 * common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide s32k1xx_lpspi0/1/2select() and s32k1xx_lpspi0/1/2status()
 *      functions in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k1xx_lpspibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by s32k1xx_lpspibus_initialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_lpspislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pinmux.h"
#include "hardware/s32k1xx_lpspi.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_lpspi.h"

#include <arch/board/board.h>

#if defined(CONFIG_S32K1XX_LPSPI0) || defined(CONFIG_S32K1XX_LPSPI1) || \
    defined(CONFIG_S32K1XX_LPSPI2)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

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

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct s32k1xx_lpspidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part of the SPI interface */
  uint32_t spibase;           /* SPIn base address */
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  uint8_t spiirq;             /* SPI IRQ number */
#endif
  sem_t exclsem;              /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;         /* Requested clock frequency */
  uint32_t actual;            /* Actual clock frequency */
  int8_t nbits;               /* Width of word in bits */
  uint8_t mode;               /* Mode 0,1,2,3 */
};

enum s32k1xx_delay_e
{
  LPSPI_PCS_TO_SCK = 1,       /* PCS-to-SCK delay. */
  LPSPI_LAST_SCK_TO_PCS,      /* Last SCK edge to PCS delay. */
  LPSPI_BETWEEN_TRANSFER      /* Delay between transfers. */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint32_t s32k1xx_lpspi_getreg32(FAR struct s32k1xx_lpspidev_s *priv,
              uint8_t offset);
static inline void s32k1xx_lpspi_putreg32(FAR struct s32k1xx_lpspidev_s *priv,
              uint8_t offset, uint32_t value);
static inline uint32_t s32k1xx_lpspi_readword(FAR struct s32k1xx_lpspidev_s *priv);
static inline void s32k1xx_lpspi_writeword(FAR struct s32k1xx_lpspidev_s *priv,
              uint16_t byte);
static inline bool s32k1xx_lpspi_9to16bitmode(FAR struct s32k1xx_lpspidev_s *priv);
static uint32_t s32k1xx_lpspi_pckfreq(uintptr_t base);
static inline void s32k1xx_lpspi_set_delays(FAR struct s32k1xx_lpspidev_s
              *priv, uint32_t delay_ns, enum s32k1xx_delay_e type);
static inline void s32k1xx_lpspi_set_delay_scaler(FAR struct
              s32k1xx_lpspidev_s *priv, uint32_t scaler, enum s32k1xx_delay_e type);

/* SPI methods */

static int s32k1xx_lpspi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t s32k1xx_lpspi_setfrequency(FAR struct spi_dev_s *dev,
              uint32_t frequency);
static void s32k1xx_lpspi_setmode(FAR struct spi_dev_s *dev,
              enum spi_mode_e mode);
static void s32k1xx_lpspi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int s32k1xx_lpspi_hwfeatures(FAR struct spi_dev_s *dev,
              s32k1xx_lpspi_hwfeatures_t features);
#endif
static uint32_t s32k1xx_lpspi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void s32k1xx_lpspi_exchange(FAR struct spi_dev_s *dev,
              FAR const void *txbuffer, FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_sndblock(FAR struct spi_dev_s *dev,
              FAR const void *txbuffer, size_t nwords);
static void s32k1xx_lpspi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
              size_t nwords);
#endif

/* Initialization */

static void s32k1xx_lpspi_bus_initialize(FAR struct s32k1xx_lpspidev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock         = s32k1xx_lpspi_lock,
  .select       = s32k1xx_lpspi0select,
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
    &g_spi0ops
  },
  .spibase      = S32K1XX_LPSPI0_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI0,
#endif
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
  .select       = s32k1xx_lpspi1select,
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
    &g_spi1ops
  },
  .spibase      = S32K1XX_LPSPI1_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI1,
#endif
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
  .select       = s32k1xx_lpspi2select,
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
    &g_spi2ops
  },
  .spibase      = S32K1XX_LPSPI2_BASE,
#ifdef CONFIG_S32K1XX_LPSPI_INTERRUPTS
  .spiirq       = S32K1XX_IRQ_LPSPI2,
#endif
#ifdef CONFIG_S32K1XX_LPSPI_DMA
  .rxch         = DMAMAP_LPSPI2_RX,
  .txch         = DMAMAP_LPSPI2_TX,
#endif
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: s32k1xx_lpspi_getreg8
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 8-bit register
 *
 ************************************************************************************/

static inline uint8_t s32k1xx_lpspi_getreg8(FAR struct s32k1xx_lpspidev_s *priv,
                                            uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/************************************************************************************
 * Name: s32k1xx_lpspi_putreg8
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 ************************************************************************************/

static inline void s32k1xx_lpspi_putreg8(FAR struct s32k1xx_lpspidev_s *priv,
                                         uint8_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/************************************************************************************
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
 ************************************************************************************/

static inline uint32_t s32k1xx_lpspi_getreg32(FAR struct s32k1xx_lpspidev_s *priv,
                                              uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/************************************************************************************
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
 ************************************************************************************/

static inline void s32k1xx_lpspi_putreg32(FAR struct s32k1xx_lpspidev_s *priv,
                                          uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/************************************************************************************
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
 ************************************************************************************/

static inline uint32_t s32k1xx_lpspi_readword(FAR struct s32k1xx_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) & LPSPI_SR_RDF) == 0)
    {
    }

  /* Then return the received byte */

  return (uint32_t) s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_RDR_OFFSET);
}

/************************************************************************************
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
 ************************************************************************************/

static inline void s32k1xx_lpspi_writeword(FAR struct s32k1xx_lpspidev_s *priv,
                                           uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) & LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the word */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TDR_OFFSET, word);
}

/************************************************************************************
 * Name: s32k1xx_lpspi_readbyte
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint8_t s32k1xx_lpspi_readbyte(FAR struct s32k1xx_lpspidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) & LPSPI_SR_RDF) == 0)
    {
    }

  /* Then return the received byte */

  return s32k1xx_lpspi_getreg8(priv, S32K1XX_LPSPI_RDR_OFFSET);
}

/************************************************************************************
 * Name: s32k1xx_lpspi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void s32k1xx_lpspi_writebyte(FAR struct s32k1xx_lpspidev_s *priv,
                                           uint8_t byte)
{
  /* Wait until the transmit buffer is empty */

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) & LPSPI_SR_TDF) == 0)
    {
    }

  /* Then send the byte */

  s32k1xx_lpspi_putreg8(priv, S32K1XX_LPSPI_TDR_OFFSET, byte);
}

/************************************************************************************
 * Name: s32k1xx_lpspi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: >8 bit mode-bit mode, false: <= 8-bit mode
 *
 ************************************************************************************/

static inline bool s32k1xx_lpspi_9to16bitmode(FAR struct s32k1xx_lpspidev_s *priv)
{
  bool ret;

  if (((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_TCR_OFFSET) &
        LPSPI_TCR_FRAMESZ_MASK) + 1) < 9)
    {
      ret = false;
    }
  else
    {
      ret = true;
    }

  return ret;
}

/************************************************************************************
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
 ************************************************************************************/

static void s32k1xx_lpspi_modifyreg32(FAR struct s32k1xx_lpspidev_s *priv,
                                      uint8_t offset, uint32_t clrbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/************************************************************************************
 * Name: s32k1xx_lpspi_pckfreq
 *
 * Description:
 *   Get the peripheral clock frequency for the LPSPI peripheral
 *
 * Input Parameters:
 *   base - The base address of the LPSPI peripheral registers
 *
 * Returned Value:
 *   The frequency of the LPSPI functional input frequency (or zero on a failure)
 *
 ************************************************************************************/

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

/************************************************************************************
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
 ************************************************************************************/

static inline void s32k1xx_lpspi_set_delay_scaler(FAR struct
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
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, LPSPI_CCR_DBT_MASK,
                              0);
      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CCR_OFFSET, 0,
                              LPSPI_CCR_DBT(scaler));
      break;
    }
}

/************************************************************************************
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
 ************************************************************************************/

static inline void s32k1xx_lpspi_set_delays(FAR struct s32k1xx_lpspidev_s *priv,
                                                   uint32_t delay_ns,
                                                   enum s32k1xx_delay_e type)
{
  uint32_t inclock;
  uint64_t real_delay;
  uint64_t best_delay;
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
              (1 << ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_TCR_OFFSET) &
              LPSPI_TCR_PRESCALE_MASK) >> LPSPI_TCR_PRESCALE_SHIFT));

  min_diff = 0xffffffff;

  /* Initialize scaler to max value to generate the max delay */

  best_scaler = 0xff;

  if (type == LPSPI_BETWEEN_TRANSFER)
    {
      /* First calculate the initial, default delay, note min delay is 2 clock
       * cycles. Due to large size of * calculated values (uint64_t), we need
       * to break up the calculation into several steps to ensure * accurate
       * calculated results
       */

      initial_delay_ns = 1000000000U;
      initial_delay_ns *= 2;
      initial_delay_ns /= clock_div_prescaler;

      /* Calculate the maximum delay */

      best_delay = 1000000000U;

      /* based on DBT+2, or 255 + 2 */

      best_delay *= 257;
      best_delay /= clock_div_prescaler;

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

      /* Calculate the maximum delay */

      best_delay = 1000000000U;

      /* Based on SCKPCS+1 or PCSSCK+1, or 255 + 1 */

      best_delay *= 256;
      best_delay /= clock_div_prescaler;

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
           * values. Due to large size of calculated values (uint64_t), we need
           * to break up the calculation into several steps to ensure accurate
           * calculated results
           */

          real_delay  = 1000000000U;
          real_delay *= (scaler + 1 + additional_scaler);
          real_delay /= clock_div_prescaler;

          /* calculate the delay difference based on the conditional statement
           * that states that the calculated delay must not be less then the
           * desired delay
           */

          if (real_delay >= delay_ns)
            {
              diff = real_delay - delay_ns;
              if (min_diff > diff)
                {
                  /* A better match found */

                  min_diff = diff;
                  best_scaler = scaler;
                  best_delay = real_delay;
                }
            }
        }

      s32k1xx_lpspi_set_delay_scaler(priv, best_scaler, type);
    }
}

/************************************************************************************
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
 ************************************************************************************/

static int s32k1xx_lpspi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
  int ret;

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

/************************************************************************************
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
 ************************************************************************************/

static uint32_t s32k1xx_lpspi_setfrequency(FAR struct spi_dev_s *dev,
                                         uint32_t frequency)
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;

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

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
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

/************************************************************************************
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
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void s32k1xx_lpspi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t men;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Disable LPSPI if it is enabled */

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
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

      s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, clrbits, setbits);

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
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
        }
    }
}

/************************************************************************************
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
 ************************************************************************************/

static void s32k1xx_lpspi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
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

      men = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
        }

      regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_TCR_OFFSET);
      regval &= ~LPSPI_TCR_FRAMESZ_MASK;
      regval |= LPSPI_TCR_FRAMESZ(nbits - 1);

      s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, regval);

      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = savbits;    /* nbits has been clobbered... save the signed
                                 * value. */

      /* Re-enable LPSPI if it was enabled previously */

      if (men)
        {
          s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
        }
    }
}

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int s32k1xx_lpspi_hwfeatures(FAR struct spi_dev_s *dev,
                                    s32k1xx_lpspi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
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

  s32k1xx_lpspi_modigyreg32(priv, S32K1XX_LPSPI_TCR_OFFSET, clrbits, setbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/************************************************************************************
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
 ************************************************************************************/

static uint32_t s32k1xx_lpspi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  s32k1xx_lpspi_writeword(priv, wd);

  while ((s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET) & LPSPI_SR_RDF) !=
         LPSPI_SR_RDF);

  ret = s32k1xx_lpspi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = s32k1xx_lpspi_getreg32(priv, S32K1XX_LPSPI_SR_OFFSET);

  spiinfo("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);

  UNUSED(regval);
  return ret;
}

/************************************************************************************
 * Name: s32k1xx_lpspi_exchange (no DMA).  aka s32k1xx_lpspi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if !defined(CONFIG_S32K1XX_LPSPI_DMA) || defined(CONFIG_S32K1XX_DMACAPABLE)
#if !defined(CONFIG_S32K1XX_LPSPI_DMA)
static void s32k1xx_lpspi_exchange(FAR struct spi_dev_s *dev,
                                   FAR const void *txbuffer, FAR void *rxbuffer,
                                   size_t nwords)
#else
static void s32k1xx_lpspi_exchange_nodma(FAR struct spi_dev_s *dev,
                                       FAR const void *txbuffer,
                                       FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct s32k1xx_lpspidev_s *priv = (FAR struct s32k1xx_lpspidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (s32k1xx_lpspi_9to16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src = (const uint16_t *)txbuffer;
      uint16_t *dest = (uint16_t *) rxbuffer;
      uint16_t word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
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
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src = (const uint8_t *)txbuffer;
      uint8_t *dest = (uint8_t *) rxbuffer;
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

/************************************************************************************
 * Name: s32k1xx_lpspi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_sndblock(FAR struct spi_dev_s *dev,
                                 FAR const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return s32k1xx_lpspi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: s32k1xx_lpspi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void s32k1xx_lpspi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                    size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return s32k1xx_lpspi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Name: s32k1xx_lpspi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit, mode 0,
 *   etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void s32k1xx_lpspi_bus_initialize(struct s32k1xx_lpspidev_s *priv)
{
  uint32_t reg = 0;

  /* NOTE: Clocking to the LPSPI peripheral must be provided by board-specific logic
   * as part of the clock configuration logic.
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
  reg &= ~(LPSPI_CFGR1_OUTCFG | LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_NOSTALL);
  reg |= LPSPI_CFGR1_OUTCFG_RETAIN | LPSPI_CFGR1_PINCFG_SIN_SOUT;
  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_CFGR1_OFFSET, reg);

  /* Set frequency and delay times */

  s32k1xx_lpspi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Set default watermarks */

  s32k1xx_lpspi_putreg32(priv, S32K1XX_LPSPI_FCR_OFFSET,
                       LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));

  /* Set Transmit Command Register */

  s32k1xx_lpspi_setbits((FAR struct spi_dev_s *)priv, 8);

  s32k1xx_lpspi_setmode((FAR struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Enable LPSPI */

  s32k1xx_lpspi_modifyreg32(priv, S32K1XX_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 ************************************************************************************/

FAR struct spi_dev_s *s32k1xx_lpspibus_initialize(int bus)
{
  FAR struct s32k1xx_lpspidev_s *priv = NULL;

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
      return NULL;
    }

  leave_critical_section(flags);

  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_S32K1XX_LPSPI0 || CONFIG_S32K1XX_LPSPI1 || CONFIG_S32K1XX_LPSPI2 */
