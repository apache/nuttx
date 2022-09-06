/****************************************************************************
 * arch/arm/src/kinetis/kinetis_spi.c
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
 *   4. The handle returned by kinetis_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "kinetis.h"
#include "kinetis_edma.h"
#include "kinetis_spi.h"
#include "hardware/kinetis_dmamux.h"
#include "hardware/kinetis_memorymap.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_dspi.h"
#include "hardware/kinetis_pinmux.h"

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || \
    defined(CONFIG_KINETIS_SPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KINETIS_SPI_CLK_MAX    (BOARD_BUS_FREQ / 2)
#define KINETIS_SPI_CLK_INIT   400000

#define  SPI_SR_CLEAR   (SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF  | \
                         SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF | \
                         SPI_SR_TXRXS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kinetis_spidev_s
{
  struct spi_dev_s  spidev;     /* Externally visible part of the SPI interface */
  uint32_t          spibase;    /* Base address of SPI registers */
  mutex_t           lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t          frequency;  /* Requested clock frequency */
  uint32_t          actual;     /* Actual clock frequency */
  uint8_t           nbits;      /* Width of word in bits (8 to 16) */
  uint8_t           mode;       /* Mode 0,1,2,3 */
  uint8_t           ctarsel;    /* Which CTAR */
#ifdef CONFIG_KINETIS_SPI_DMA
  volatile uint32_t rxresult;   /* Result of the RX DMA */
  volatile uint32_t txresult;   /* Result of the TX DMA */
  const uint8_t     rxch;       /* The RX DMA channel number */
  const uint8_t     txch;       /* The TX DMA channel number */
  DMACH_HANDLE      rxdma;      /* DMA channel handle for RX transfers */
  DMACH_HANDLE      txdma;      /* DMA channel handle for TX transfers */
  sem_t             rxsem;      /* Wait for RX DMA to complete */
  sem_t             txsem;      /* Wait for TX DMA to complete */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(struct kinetis_spidev_s *priv,
                                  uint8_t offset);
static inline void     spi_putreg(struct kinetis_spidev_s *priv,
                                  uint8_t offset, uint32_t value);
static inline uint16_t spi_getreg16(struct kinetis_spidev_s *priv,
                                    uint8_t offset);
static inline void     spi_putreg16(struct kinetis_spidev_s *priv,
                                    uint8_t offset, uint16_t value);
static inline uint8_t  spi_getreg8(struct kinetis_spidev_s *priv,
                                   uint8_t offset);
static inline void     spi_putreg8(struct kinetis_spidev_s *priv,
                                   uint8_t offset, uint8_t value);
static inline uint16_t spi_readword(struct kinetis_spidev_s *priv);
static inline void     spi_writeword(struct kinetis_spidev_s *priv,
                                     uint16_t word);

static inline void     spi_run(struct kinetis_spidev_s *priv,
                               bool enable);
static inline void     spi_write_control(struct kinetis_spidev_s *priv,
                                         uint32_t control);
static inline void     spi_write_status(struct kinetis_spidev_s *priv,
                                        uint32_t status);
static inline void     spi_wait_status(struct kinetis_spidev_s *priv,
                                       uint32_t status);
static uint16_t        spi_send_data(struct kinetis_spidev_s *priv,
                                     uint16_t wd, bool last);

/* DMA support */

#ifdef CONFIG_KINETIS_SPI_DMA
static int         spi_dmarxwait(struct kinetis_spidev_s *priv);
static int         spi_dmatxwait(struct kinetis_spidev_s *priv);
static inline void spi_dmarxwakeup(struct kinetis_spidev_s *priv);
static inline void spi_dmatxwakeup(struct kinetis_spidev_s *priv);
static void        spi_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static void        spi_dmatxcallback(DMACH_HANDLE handle, void *arg,
                                     bool done, int result);
static inline void spi_dmarxstart(struct kinetis_spidev_s *priv);
static inline void spi_dmatxstart(struct kinetis_spidev_s *priv);
#endif

/* SPI methods */

static int         spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency);
static void        spi_setmode(struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void        spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t    spi_send(struct spi_dev_s *dev, uint32_t wd);
static void        spi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer,
                                void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords);
static void        spi_recvblock(struct spi_dev_s *dev,
                                 void *rxbuffer,
                                 size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  .lock     = NXMUTEX_INITIALIZER,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
#ifdef CONFIG_KINETIS_SPI_DMA
#  ifdef CONFIG_KINETIS_SPI0_DMA
  .rxch     = KINETIS_DMA_REQUEST_SRC_SPI0_RX,
  .txch     = KINETIS_DMA_REQUEST_SRC_SPI0_TX,
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
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
  .lock     = NXMUTEX_INITIALIZER,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
#ifdef CONFIG_KINETIS_SPI_DMA
#  ifdef CONFIG_KINETIS_SPI1_DMA
  .rxch     = KINETIS_DMA_REQUEST_SRC_SPI1_RX,
  .txch     = KINETIS_DMA_REQUEST_SRC_SPI1_TX,
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
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
  .lock     = NXMUTEX_INITIALIZER,
  .ctarsel           = KINETIS_SPI_CTAR0_OFFSET,
#ifdef CONFIG_KINETIS_SPI_DMA
#  ifdef CONFIG_KINETIS_SPI2_DMA
  .rxch     = KINETIS_DMA_REQUEST_SRC_FTM3_CH6__SPI2_RX,
  .txch     = KINETIS_DMA_REQUEST_SRC_FTM3_CH7__SPI2_TX,
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Atomic modification of the 32-bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv      - private SPI device structure
 *   offset    - offset to the register of interest
 *   clearbits - bits to clear
 *   clearbits - bits to set
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static inline void spi_modifyreg(struct kinetis_spidev_s *priv,
                                 uint8_t offset, uint32_t clearbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clearbits, setbits);
}
#endif

/****************************************************************************
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
 ****************************************************************************/

static inline uint32_t spi_getreg(struct kinetis_spidev_s *priv,
                                  uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_putreg(struct kinetis_spidev_s *priv,
                              uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline uint16_t spi_getreg16(struct kinetis_spidev_s *priv,
                                    uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_putreg16(struct kinetis_spidev_s *priv,
                                uint8_t offset,
                                uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline uint8_t spi_getreg8(struct kinetis_spidev_s *priv,
                                  uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_putreg8(struct kinetis_spidev_s *priv,
                               uint8_t offset,
                               uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_write_status(struct kinetis_spidev_s *priv,
                                    uint32_t status)
{
  /* Write the SR Register */

  spi_putreg(priv, KINETIS_SPI_SR_OFFSET, status);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_wait_status(struct kinetis_spidev_s *priv,
                                   uint32_t status)
{
  while (status != (spi_getreg(priv, KINETIS_SPI_SR_OFFSET) & status));
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_write_control(struct kinetis_spidev_s *priv,
                                     uint32_t control)
{
  /* Write the control word to the SPI Data Register */

  spi_putreg16(priv, KINETIS_SPI_PUSHR_OFFSET + 2,
              (uint16_t) (control >> 16));
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_writeword(struct kinetis_spidev_s *priv,
                                 uint16_t word)
{
  /* Wait until there is space in the fifo */

  spi_wait_status(priv, SPI_SR_TFFF);

  /* Write the data to transmitted to the SPI Data Register */

  spi_putreg16(priv, KINETIS_SPI_PUSHR_OFFSET, SPI_PUSHR_TXDATA(word));
}

/****************************************************************************
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
 ****************************************************************************/

static inline uint16_t spi_readword(struct kinetis_spidev_s *priv)
{
  /* Wait until transfer completes and the data is in the RX FIFO */

  spi_wait_status(priv, SPI_SR_RFDF | SPI_SR_TCF);

  /* Return the data */

  return spi_getreg16(priv, KINETIS_SPI_POPR_OFFSET);
}

/****************************************************************************
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
 ****************************************************************************/

void inline spi_run(struct kinetis_spidev_s *priv, bool enable)
{
  uint32_t regval;

  regval = spi_getreg(priv, KINETIS_SPI_MCR_OFFSET);
  regval &= ~SPI_MCR_HALT;
  regval |= enable ? 0 : SPI_MCR_HALT;
  spi_putreg(priv, KINETIS_SPI_MCR_OFFSET, regval);
}

/****************************************************************************
 * Name: spi_lock
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

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;
  int ret;

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
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;

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

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

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

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, priv->actual);
  return priv->actual;
}

/****************************************************************************
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
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;
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

/****************************************************************************
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
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;
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
      regval |= SPI_CTARM_FMSZ(nbits - 1);
      spi_putreg(priv, priv->ctarsel, regval);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
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
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev,
                          spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct kinetis_spidev_s *priv = (struct spi_dev_s *)dev;
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

/****************************************************************************
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
 ****************************************************************************/

static uint16_t spi_send_data(struct kinetis_spidev_s *priv, uint16_t wd,
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

/****************************************************************************
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
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;

  return (uint32_t)spi_send_data(priv, (uint16_t)wd, true);
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
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
 *              packed into uint8_t's; if nbits > 8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_STM32_SPI_DMA) || defined(CONFIG_STM32_SPI_DMATHRESHOLD)
#  if !defined(CONFIG_KINETIS_SPI_DMA)
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#  else
static void spi_exchange_nodma(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#  endif
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)dev;
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

          byte = (uint8_t) spi_send_data(priv, (uint16_t)byte,
                                         nwords ? false : true);

          /* Is there a buffer to receive the return value? */

          if (brxptr)
            {
              *brxptr++ = byte;
            }
        }
    }
}
#endif /* !defined(CONFIG_STM32_SPI_DMA) || defined(CONFIG_STM32_SPI_DMATHRESHOLD) */

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits > 8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  int                      ret;
  size_t                   adjust;
  ssize_t                  nbytes;
  static uint8_t           rxdummy[4] aligned_data(4);
  static const uint16_t    txdummy = 0xffff;
  struct kinetis_spidev_s *priv    = (struct kinetis_spidev_s *)dev;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv && priv->spibase);
  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

  /* Invalid DMA channels fall back to non-DMA method. */

  if (priv->rxdma == NULL || priv->txdma == NULL
#ifdef CONFIG_KINETIS_SPI_DMATHRESHOLD
      /* If this is a small SPI transfer, then let spi_exchange_nodma()
       * do the work.
       */

      || nbytes <= CONFIG_KINETIS_SPI_DMATHRESHOLD
#endif
      )
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* Halt the SPI */

  spi_run(priv, false);

  /* Flush FIFOs */

  spi_modifyreg(priv, KINETIS_SPI_MCR_OFFSET,
                SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF,
                SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF);

  /* Clear all status bits */

  spi_write_status(priv, SPI_SR_CLEAR);

  /* disable DMA */

  spi_modifyreg(priv, KINETIS_SPI_RSER_OFFSET,
                SPI_RSER_RFDF_RE | SPI_RSER_TFFF_RE |
                SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_DIRS,
                0);

  /* Set up the DMA */

  adjust = (priv->nbits > 8) ? 2 : 1;

  struct kinetis_edma_xfrconfig_s config;

  config.saddr  = priv->spibase + KINETIS_SPI_POPR_OFFSET;
  config.daddr  = (uint32_t) (rxbuffer ? rxbuffer : rxdummy);
  config.soff   = 0;
  config.doff   = rxbuffer ? adjust : 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.ttype  = EDMA_PERIPH2MEM;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  kinetis_dmach_xfrsetup(priv->rxdma, &config);

  config.saddr  = (uint32_t) (txbuffer ? txbuffer : &txdummy);
  config.daddr  = priv->spibase + KINETIS_SPI_PUSHR_OFFSET;
  config.soff   = txbuffer ? adjust : 0;
  config.doff   = 0;
  config.iter   = nbytes;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.dsize  = adjust == 1 ? EDMA_8BIT : EDMA_16BIT;
  config.ttype  = EDMA_MEM2PERIPH;
  config.nbytes = adjust;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  kinetis_dmach_xfrsetup(priv->txdma, &config);

  spi_modifyreg(priv, KINETIS_SPI_RSER_OFFSET, 0 ,
                SPI_RSER_RFDF_RE | SPI_RSER_TFFF_RE |
                SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_DIRS);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_run(priv, true);

  spi_putreg(priv, KINETIS_SPI_TCR_OFFSET, 0);
  spi_write_control(priv, SPI_PUSHR_CTAS_CTAR0);

  spi_dmatxstart(priv);

  /* Then wait for each to complete */

  ret = spi_dmarxwait(priv);

  if (ret < 0)
    {
      ret = spi_dmatxwait(priv);
    }

  /* Reset any status */

  spi_write_status(priv, spi_getreg(priv, KINETIS_SPI_SR_OFFSET));

  /* Halt SPI */

  spi_run(priv, false);

  /* Disable DMA */

  spi_modifyreg(priv, KINETIS_SPI_RSER_OFFSET,
                SPI_RSER_RFDF_RE | SPI_RSER_TFFF_RE |
                SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_DIRS,
                0);
}

#endif  /* CONFIG_KINETIS_SPI_DMA */
/****************************************************************************
 * Name: spi_sndblock
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
static void spi_sndblock(struct spi_dev_s *dev,
                         const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static int spi_dmarxwait(struct kinetis_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   *  DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static int spi_dmatxwait(struct kinetis_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static inline void spi_dmarxwakeup(struct kinetis_spidev_s *priv)
{
  nxsem_post(&priv->rxsem);
}
#endif

/****************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static inline void spi_dmatxwakeup(struct kinetis_spidev_s *priv)
{
  nxsem_post(&priv->txsem);
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static void spi_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)arg;

  priv->rxresult = result | 0x80000000;  /* assure non-zero */
  spi_dmarxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static void spi_dmatxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct kinetis_spidev_s *priv = (struct kinetis_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = result | 0x80000000;  /* assure non-zero */
  spi_dmatxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static inline void spi_dmarxstart(struct kinetis_spidev_s *priv)
{
  priv->rxresult = 0;
  kinetis_dmach_start(priv->rxdma, spi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI_DMA
static inline void spi_dmatxstart(struct kinetis_spidev_s *priv)
{
  priv->txresult = 0;
  kinetis_dmach_start(priv->txdma, spi_dmatxcallback, priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *kinetis_spibus_initialize(int port)
{
  struct kinetis_spidev_s *priv;
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

  /* Read MCR register and clear MDIS (to Enable module clock)
   * It is necessary because to disable RX and TX FIFO the MDIS
   * bit should be cleared first.
   */

  regval  = spi_getreg(priv, KINETIS_SPI_MCR_OFFSET);
  regval &= ~(SPI_MCR_MDIS);
  spi_putreg(priv, KINETIS_SPI_MCR_OFFSET, regval);

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

  regval |= SPI_MCR_MSTR | SPI_MCR_DCONF_SPI | SPI_MCR_SMPL_PT_0CLKS |
            SPI_MCR_PCSIS_MASK | SPI_MCR_HALT | SPI_MCR_DIS_RXF |
            SPI_MCR_DIS_TXF;
  spi_putreg(priv, KINETIS_SPI_MCR_OFFSET, regval);

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

#ifdef CONFIG_KINETIS_SPI_DMA
  if (priv->rxch && priv->txch)
    {
      if (priv->txdma == NULL && priv->rxdma == NULL)
        {
          priv->txdma = kinetis_dmach_alloc(priv->txch | DMAMUX_CHCFG_ENBL,
                                            0);
          priv->rxdma = kinetis_dmach_alloc(priv->rxch | DMAMUX_CHCFG_ENBL,
                                            0);
          DEBUGASSERT(priv->rxdma && priv->txdma);
          spi_modifyreg(priv, KINETIS_SPI_MCR_OFFSET,
                        0,
                        SPI_MCR_DIS_RXF | SPI_MCR_DIS_TXF
                        );
        }
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  return &priv->spidev;
}

#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 ||  CONFIG_KINETIS_SPI2 */
