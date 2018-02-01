/************************************************************************************
 * arch/arm/src/kinetis/kinetis_spi.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 * The external functions, kinetis_spi0/1/2select and kinetis_spi0/1/26status
 * must be provided by board-specific logic.  They are implementations of
 * the select and status methods of the SPI interface defined by structure
 * spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 * (including kinetis_spibus_initialize()) are provided by common Kinetis
 * logic.
 * To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide kinetis_spi[n]select() and kinetis_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to kinetis_spibus_initialize() in your low level
 *      application initialization logic.
 *   4. The handle returned by kinetis_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "kinetis.h"
#include "kinetis_spi.h"
#include "chip/kinetis_memorymap.h"
#include "chip/kinetis_sim.h"
#include "chip/kinetis_dspi.h"
#include "chip/kinetis_pinmux.h"

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || \
    defined(CONFIG_KINETIS_SPI2)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define KINETIS_SPI_CLK_MAX    (BOARD_BUS_FREQ / 2)
#define KINETIS_SPI_CLK_INIT   400000

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct kinetis_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* Base address of SPI registers */
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
  uint8_t          ctarsel;    /* Which CTAR */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(FAR struct kinetis_spidev_s *priv, uint8_t offset);
static inline void     spi_putreg(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                              uint32_t value);
static inline uint16_t spi_getreg16(FAR struct kinetis_spidev_s *priv, uint8_t offset);
static inline void     spi_putreg16(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                                    uint16_t value);
static inline uint8_t  spi_getreg8(FAR struct kinetis_spidev_s *priv, uint8_t offset);
static inline void     spi_putreg8(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                                   uint8_t value);
static inline uint16_t spi_readword(FAR struct kinetis_spidev_s *priv);
static inline void     spi_writeword(FAR struct kinetis_spidev_s *priv,
                                     uint16_t word);

static inline void     spi_run(FAR struct kinetis_spidev_s *priv, bool enable);
static inline void     spi_write_control(FAR struct kinetis_spidev_s *priv,
                                         uint32_t control);
static inline void     spi_write_status(FAR struct kinetis_spidev_s *priv,
                                         uint32_t status);
static inline void     spi_wait_status(FAR struct kinetis_spidev_s *priv,
                                         uint32_t status);
static uint16_t        spi_send_data(FAR struct kinetis_spidev_s *priv, uint16_t wd,
                                     bool last);

/* SPI methods */

static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(FAR struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint16_t    spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_KINETIS_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = kinetis_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#  endif
  .status            = kinetis_spi0status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = kinetis_spi0cmddata,
#  endif
  .send              = spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#  else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#  endif
#  ifdef CONFIG_SPI_CALLBACK
  .registercallback  = kinetis_spi0register,  /* provided externally */
#  else
  .registercallback  = 0,  /* not implemented */
#  endif
};

static struct kinetis_spidev_s g_spi0dev =
{
  .spidev            =
  {
    &g_spi0ops
  },
  .spibase           = KINETIS_SPI0_BASE,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
};
#endif

#ifdef CONFIG_KINETIS_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = kinetis_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#  endif
  .status            = kinetis_spi1status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = kinetis_spi1cmddata,
#  endif
  .send              = spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#  else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#  endif
#  ifdef CONFIG_SPI_CALLBACK
  .registercallback  = kinetis_spi1register,  /* provided externally */
#  else
  .registercallback  = 0,  /* not implemented */
#  endif
};

static struct kinetis_spidev_s g_spi1dev =
{
  .spidev            =
  {
    &g_spi1ops
  },
  .spibase           = KINETIS_SPI1_BASE,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
};
#endif

#ifdef CONFIG_KINETIS_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = kinetis_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#  endif
  .status            = kinetis_spi2status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = kinetis_spi2cmddata,
#  endif
  .send              = spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#  else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#  endif
#  ifdef CONFIG_SPI_CALLBACK
  .registercallback  = kinetis_spi2register,  /* provided externally */
#  else
  .registercallback  = 0,  /* not implemented */
#  endif
};

static struct kinetis_spidev_s g_spi2dev =
{
  .spidev            =
  {
    &g_spi2ops
  },
  .spibase           = KINETIS_SPI2_BASE,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the 32-bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ************************************************************************************/

static inline uint32_t spi_getreg(FAR struct kinetis_spidev_s *priv, uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   Nothing
 *
 ************************************************************************************/

static inline void spi_putreg(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_getreg16
 *
 * Description:
 *   Get the 16 bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline uint16_t spi_getreg16(FAR struct kinetis_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg16
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   Nothing
 *
 ************************************************************************************/

static inline void spi_putreg16(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                              uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_getreg8
 *
 * Description:
 *   Get the 8 bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 8-bit register
 *
 ************************************************************************************/

static inline uint8_t spi_getreg8(FAR struct kinetis_spidev_s *priv, uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 * Returned Value:
 *   Nothing
 *
 ************************************************************************************/

static inline void spi_putreg8(FAR struct kinetis_spidev_s *priv, uint8_t offset,
                              uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_write_status
 *
 * Description:
 *   Write the 32-bit status
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   status- any ones will clear flags.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_write_status(FAR struct kinetis_spidev_s *priv, uint32_t status)
{

  /* Write the SR Register */

  spi_putreg(priv, KINETIS_SPI_SR_OFFSET, status);
}

/************************************************************************************
 * Name: spi_wait_status
 *
 * Description:
 *   Wait for bit to be set in status
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   status- bit to wait on.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_wait_status(FAR struct kinetis_spidev_s *priv, uint32_t status)
{

  while (status != (spi_getreg(priv, KINETIS_SPI_SR_OFFSET) & status));
}

/************************************************************************************
 * Name: spi_write_control
 *
 * Description:
 *   Write the 16-bit control word to the TX FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   control- to write
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_write_control(FAR struct kinetis_spidev_s *priv, uint32_t control)
{

  /* Write the control word to the SPI Data Register */

  spi_putreg16(priv, KINETIS_SPI_PUSHR_OFFSET + 2, (uint16_t) (control >> 16));
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one 16 bit word to SPI TX FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - word to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct kinetis_spidev_s *priv, uint16_t word)
{
  /* Wait until there is space in the fifo */

  spi_wait_status(priv, SPI_SR_TFFF);

  /* Write the data to transmitted to the SPI Data Register */

  spi_putreg16(priv, KINETIS_SPI_PUSHR_OFFSET, SPI_PUSHR_TXDATA(word));
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one 16 bit word from SPI RX FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   The 8-bit value from the FIFO
  *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct kinetis_spidev_s *priv)
{
  /* Wait until transfer completes and the data is in the RX FIFO */

  spi_wait_status(priv, SPI_SR_RFDF | SPI_SR_TCF);

   /* Return the data */

  return spi_getreg16(priv, KINETIS_SPI_POPR_OFFSET);
}

/************************************************************************************
 * Name: spi_run
 *
 * Description:
 *   Sets or clears the HALT
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   enable - True clears HALT
 *
 * Returned Value:
 *   Last enable setting
 *
 ************************************************************************************/

void inline spi_run(FAR struct kinetis_spidev_s *priv, bool enable)
{
  uint32_t regval;

  regval = spi_getreg(priv, KINETIS_SPI_MCR_OFFSET);
  regval &= ~SPI_MCR_HALT;
  regval |= enable ? 0 : SPI_MCR_HALT;
  spi_putreg(priv, KINETIS_SPI_MCR_OFFSET, regval);
}

/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
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

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);
    }
  else
    {
      (void)nxsem_post(&priv->exclsem);
      ret = OK;
    }

  return ret;
}

/************************************************************************************
 * Name: spi_setfrequency
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;

  uint32_t prescale;
  uint32_t prescalev;
  uint32_t doublebr;
  uint32_t scaler;
  uint32_t scalerv;
  uint32_t diff;
  uint32_t actual;
  uint32_t regval;

  uint32_t pbr = 0;
  uint32_t dbr = 1;
  uint32_t br  = 0;
  uint32_t min  = UINT32_MAX;

  /* Check if requested frequency reasonable */

  if (frequency > KINETIS_SPI_CLK_MAX)
    {
      frequency = KINETIS_SPI_CLK_MAX;
    }
  else if (frequency == 0)
    {
      frequency = KINETIS_SPI_CLK_INIT;
    }

  /* Check if the requested frequency is the same as the frequency selection */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* The clock source for the SPI baud rate generator is the bus clock.
   * and the SCK is given by:
   *
   *   SCK = (fP /PBR) x [(1+DBR)/BR]
   *
   *   Where:
   *     fP  - the Bus Clock
   *     PBR - Baud Rate Prescaler {2, 3, 5, 7}
   *     DBR - Double Baud Rate    {0, 1}
   *     BR  - Baud Rate Scaler    {2, 4, 6, 8 ... 32,768}
   *
   *  We need find a PBR and BR resulting in the in baudrate closest to the
   *  requested value. We give preference to DBR of 0 to maintina a 50/50
   *  duty sysle
   *
   */

  for (doublebr = 1; min && doublebr <= 2; doublebr++)
    {
      for (prescalev = 0, prescale = 2;
           min && prescalev <= 3;
           prescalev ++, prescale == 2 ? prescale++ : (prescale += 2))
        {
          for (scalerv = 0, scaler = 2;
               min && scalerv <= 15;
               scalerv++, scaler < 8 ? (scaler += 2) : (scaler <<= 1))
            {
              actual = ((BOARD_BUS_FREQ * doublebr) / (prescale * scaler));
              if (frequency >= actual)
                {
                  diff = frequency - actual;
                  if (min > diff)
                    {
                      min = diff;
                      pbr = prescalev;
                      dbr = doublebr == 2 ? SPI_CTARM_DBR : 0;
                      br  = scalerv;
                      priv->actual    = actual;
                    }
                }
            }
        }
    }

  /* Write the new dividers to the CTAR register */

  regval = spi_getreg(priv, priv->ctarsel);
  regval &= ~(SPI_CTARM_BR_MASK | SPI_CTARM_PBR_MASK | SPI_CTARM_DBR);
  regval |= (SPI_CTARM_BR(br) | SPI_CTARM_PBR(pbr) | dbr);
  spi_putreg(priv, priv->ctarsel, regval);

  /* Save the frequency setting so that subsequent re-configurations will be
   * faster.
   */

  priv->frequency = frequency;

  spiinfo("Frequency %d->%d\n", frequency, priv->actual);
  return priv->actual;
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;
  uint32_t regval;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CTAR appropriately */

      regval = spi_getreg(priv, priv->ctarsel);
      regval &= ~(SPI_CTAR_CPOL | SPI_CTAR_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CTAR_CPHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CTAR_CPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CTAR_CPOL | SPI_CTAR_CPHA);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, priv->ctarsel, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/************************************************************************************
 * Name: spi_setbits
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

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;
  uint32_t regval;

  if (nbits != priv->nbits)
    {
      /* Set the number of bits (valid range 4-16) */

      if (nbits < 4 || nbits > 16)
        {
          return;
        }

      regval = spi_getreg(priv, priv->ctarsel);
      regval &= ~(SPI_CTARM_FMSZ_MASK);
      regval |= SPI_CTARM_FMSZ(nbits-1);
      spi_putreg(priv, priv->ctarsel, regval);

      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = nbits;
    }
}

/************************************************************************************
 * Name: spi_hwfeatures
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
static int spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  FAR struct kinetis_spidev_s *priv = (FAR struct spi_dev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = SPI_CTARM_LSBFE;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = SPI_CTARM_LSBFE;
    }

  regval = spi_getreg(priv, priv->ctarsel);
  regval &= ~clrbits;
  regval |= setbits;
  spi_putreg(priv, priv->ctarsel, regval);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
}
#endif

/************************************************************************************
 * Name: spi_send_data
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send_data(FAR struct kinetis_spidev_s *priv, uint16_t wd,
                              bool last)
{
  uint16_t ret;

  /* On first write set control word and start transfer */

  if (0 == (spi_getreg(priv, KINETIS_SPI_SR_OFFSET) & SPI_SR_TXRXS))
    {
      spi_run(priv, true);
      spi_write_control(priv, SPI_PUSHR_CTAS_CTAR0 | SPI_PUSHR_CTCNT);
    }

  spi_writeword(priv, wd);
  ret = spi_readword(priv);

  if (!last)
    {
      /* Clear the Transfer complete and the RX FIFO RDY */

      spi_write_status(priv, SPI_SR_TCF | SPI_SR_RFDF);
    }
  else
    {
      /* Clear all status */

      spi_write_status(priv, spi_getreg(priv, KINETIS_SPI_SR_OFFSET));
      spi_run(priv, false);
    }

  return ret;
}

/************************************************************************************
 * Name: spi_send
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

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;

  return spi_send_data(priv, wd, true);
}

/************************************************************************************
 * Name: spi_exchange
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
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct kinetis_spidev_s *priv = (FAR struct kinetis_spidev_s *)dev;
  uint8_t        *brxptr = (uint8_t *)rxbuffer;
  const uint8_t  *btxptr = (uint8_t *)txbuffer;
  uint16_t       *wrxptr = (uint16_t *)rxbuffer;
  const uint16_t *wtxptr = (const uint16_t *)txbuffer;
  uint8_t         byte;
  uint16_t        word;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (wtxptr)
            {
              word = *wtxptr++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = spi_send_data(priv, word, nwords ? false : true);

          /* Is there a buffer to receive the return value? */

          if (wrxptr)
            {
              *wrxptr++ = word;
            }
        }
    }
  else
    {

      /* 8-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (btxptr)
            {
              byte = *btxptr++;
            }
          else
            {
              byte = 0xff;
            }

          /* Exchange one word */

          byte = (uint8_t) spi_send_data(priv, (uint16_t)byte, nwords ? false : true);

          /* Is there a buffer to receive the return value? */

          if (brxptr)
            {
              *brxptr++ = byte;
            }
        }
    }
}
/************************************************************************************
 * Name: spi_sndblock
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
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: kinetis_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *kinetis_spibus_initialize(int port)
{
  FAR struct kinetis_spidev_s *priv;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select pins
   * must be configured by board-specific logic.  Most SPI pins multiple,
   * alternative pin selection.  Definitions in the board.h file must be\
   * provided to resolve the board-specific pin configuration like:
   *
   * #define PIN_SPI0_SCK PIN_SPI0_SCK_1
   */

#ifdef CONFIG_KINETIS_SPI0
  if (port == 0)
    {
      priv = &g_spi0dev;

      /* Configure pins for SPI0 */

      kinetis_pinconfig(PIN_SPI0_SCK);
      kinetis_pinconfig(PIN_SPI0_SIN);
      kinetis_pinconfig(PIN_SPI0_OUT);

      /* Enable clocking */

      regval  = getreg32(KINETIS_SIM_SCGC6);
      regval |= SIM_SCGC6_SPI0;
      putreg32(regval, KINETIS_SIM_SCGC6);
    }
  else
#endif
#ifdef CONFIG_KINETIS_SPI1
  if (port == 1)
    {
      priv = &g_spi1dev;

      /* Configure pins for SPI1 */

      kinetis_pinconfig(PIN_SPI1_SCK);
      kinetis_pinconfig(PIN_SPI1_SIN);
      kinetis_pinconfig(PIN_SPI1_OUT);

      /* Enable clocking */

      regval  = getreg32(KINETIS_SIM_SCGC6);
      regval |= SIM_SCGC6_SPI1;
      putreg32(regval, KINETIS_SIM_SCGC6);
    }
  else
#endif
#ifdef CONFIG_KINETIS_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;

      /* Configure pins for SPI1 */

      kinetis_pinconfig(PIN_SPI2_SCK);
      kinetis_pinconfig(PIN_SPI2_SIN);
      kinetis_pinconfig(PIN_SPI2_OUT);

      /* Enable clocking */

      regval  = getreg32(KINETIS_SIM_SCGC3);
      regval |= SIM_SCGC3_SPI2;
      putreg32(regval, KINETIS_SIM_SCGC3);
    }
  else
#endif
    {
      spierr("ERROR: Port %d not configured\n", port);
      return NULL;
    }

  /* Halt operations */

  spi_run(priv, false);

  /* Configure master mode:
   *   Master Mode                      - Enabled
   *   Continuous SCK                   - Disabled
   *   SPI Configuration                - SPI
   *   Freeze                           - Disabled
   *   Modified Transfer Format         - Disabled
   *   Peripheral Chip Select Strobe    - Peripheral Chip Select[5] signal
   *   Receive FIFO Overflow Overwrite  - Ignore incoming
   *   Chip Select x Inactive State     - High
   *   Doze                             -  Disabled
   *   Module Disable                   - Enables the module clocks.
   *   Disable Transmit FIFO            - yes
   *   Disable Receive FIFO             - yes
   *   Clear TX FIFO                    - No
   *   Clear RX FIFO                    - No
   *   Sample Point                     -  0 clocks between edge and sample
   *
   */

  spi_putreg(priv, KINETIS_SPI_MCR_OFFSET, SPI_MCR_MSTR | SPI_MCR_DCONF_SPI |
                   SPI_MCR_SMPL_PT_0CLKS | SPI_MCR_PCSIS_MASK | SPI_MCR_HALT|
                   SPI_MCR_DIS_RXF | SPI_MCR_DIS_TXF);

  /* Set the initial SPI configuration */

  spi_putreg(priv, priv->ctarsel, 0);

  /* MSB first, 8 bit */

  priv->nbits = 0;
  spi_setbits(&priv->spidev, 8);

  /* select mode 0 */

  priv->mode      = SPIDEV_MODE3;
  spi_setmode(&priv->spidev, SPIDEV_MODE0);

  /* Select a default frequency of approx. 400KHz */

  priv->frequency = 0;
  spi_setfrequency(&priv->spidev, KINETIS_SPI_CLK_INIT);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  return &priv->spidev;
}

#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 ||  CONFIG_KINETIS_SPI2 */
