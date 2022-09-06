/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_spim.c
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
 * The external functions, max326_spi1/2/3select and max326_spi1/2/3status
 * must be provided by board-specific logic.  They are implementations of
 * the select and status methods of the SPI interface defined by struct
 * spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 * max326_spibus_initialize()) are provided by common MAX326 logic.  To use
 * this common SPI logic on your board:
 *
 *   1. Provide logic in max326_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide max326_spi1/2/3select() and max326_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to max326_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by max326_spibus_initialize() may then be used
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/max326_pinmux.h"
#include "max326_clockconfig.h"
#include "max326_periphclks.h"
#include "max326_gpio.h"
#include "max326_dma.h"
#include "max326_spim.h"

#include <arch/board/board.h>

#if defined(CONFIG_MAX326XX_HAVE_SPIM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
#  error DMA not yet supported
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_MAX326_SPI_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct max326_spidev_s
{
  struct spi_dev_s dev;          /* Externally visible part of the SPI interface */
  const void      *txbuffer;     /* Tx buffer pointer (may be NULL) */
  void            *rxbuffer;     /* Rx buffer pointer (may be NULL) */
  uint32_t         base;         /* SPI base address */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  mutex_t          lock;         /* Held while chip is selected for mutual exclusion */
  uint16_t         rxbytes;      /* Number of bytes received into rxbuffer */
  uint16_t         txbytes;      /* Number of bytes sent from txbuffer */
  uint16_t         xfrlen;       /* Transfer length */
  bool             initialized;  /* True: SPI interface been initialized */
  bool             data16;       /* True: 16- vs 8-bit data transfers */
  bool             wire3;        /* True: 3- vs 4-pin mode */
  bool             busy;         /* True: Transfer started */
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  uint8_t          irq;          /* SPI IRQ number */
#endif
  uint8_t          nbits;        /* Width of word in bits (4 through 16) */
  uint8_t          mode;         /* Mode 0,1,2,3 */

#ifdef CONFIG_MAX326_SPI_REGDEBUG
  /* Debug stuff */

  bool             wrlast;       /* Last was a write */
  uint32_t         addrlast;     /* Last address */
  uint32_t         vallast;      /* Last value */
  int              ntimes;       /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_MAX326_SPI_REGDEBUG
static bool spi_checkreg(struct max326_spidev_s *priv, bool wr,
              uint32_t regval, uintptr_t regaddr);
static uint32_t spi_getreg(struct max326_spidev_s *priv,
              unsigned int offset);
static void spi_putreg(struct max326_spidev_s *priv,
              unsigned int offset, uint32_t value);
#else
#  define spi_checkreg(priv,wr,regval,regaddr) (false)
static inline uint32_t spi_getreg(struct max326_spidev_s *priv,
              unsigned int offset);
static inline void spi_putreg(struct max326_spidev_s *priv,
              unsigned int offset, uint32_t value);
#endif

static void spi_modify_ctrl0(struct max326_spidev_s *priv, uint32_t setbits,
              uint32_t clrbits);
static void spi_modify_ctrl1(struct max326_spidev_s *priv, uint32_t setbits,
              uint32_t clrbits);
static void spi_modify_ctrl2(struct max326_spidev_s *priv, uint32_t setbits,
              uint32_t clrbits);
static void spi_modify_dma(struct max326_spidev_s *priv, uint32_t setbits,
              uint32_t clrbits);
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
static void spi_modify_inten(struct max326_spidev_s *priv, uint32_t setbits,
              uint32_t clrbits)
#endif

/* Interrupt support */

static int  spi_poll(struct max326_spidev_s *priv);
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
static int  spi_interrupt(int irq, void *context, void *arg);
#endif

/* SPI methods */

static int  spi_lock(struct spi_dev_s *dev, bool lock);
#ifdef CONFIG_MAX326XX_SPIM0
static void spi0_select(struct spi_dev_s *dev, uint32_t devid,
              bool selected);
#endif
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
              uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int  spi_hwfeatures(struct spi_dev_s *dev,
              spi_hwfeatures_t features);
#endif
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
              void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
              size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
              size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(struct max326_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NOTE:  This is somewhat over-designed since there is only a single SPI
 * peripheral.  However, it supports simple group to additional SPI
 * peripherals by extending this logic.
 */

#ifdef CONFIG_MAX326XX_SPIM0
static const struct spi_ops_s g_sp0iops =
{
  .lock              = spi_lock,
  .select            = spi0_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = max326_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = max326_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = max326_spi0register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct max326_spidev_s g_spi0dev =
{
  .dev      =
    {
      &g_sp0iops
    },
  .base     = MAX326_SPI0_BASE,
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  .irq      = MAX326_IRQ_SPI,
#endif
#ifdef CONFIG_MAX326XX_3WIRE
  .wire3    = true,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_MAX326_SPI_REGDEBUG
static bool spi_checkreg(struct max326_spidev_s *priv, bool wr,
                         uint32_t regval, uintptr_t regaddr)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      regval  == priv->vallast &&    /* Same value? */
      regaddr == priv->addrlast)     /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          mcinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg
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

#ifdef CONFIG_MAX326_SPI_REGDEBUG
static uint32_t spi_getreg(struct max326_spidev_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->base + offset;
  uint32_t regval   = getreg32(regaddr);

  if (spi_checkreg(priv, false, regval, regaddr))
    {
      mcinfo("%08x->%08x\n", regaddr, regval);
    }

  return regval;
}
#else
static inline uint32_t spi_getreg(struct max326_spidev_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - offset to the register of interest
 *   regval - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326_SPI_REGDEBUG
static void spi_putreg(struct max326_spidev_s *priv, unsigned int offset,
                       uint32_t regval)
{
  uintptr_t regaddr = priv->base + offset;

  if (spi_checkreg(priv, true, regval, regaddr))
    {
      mcinfo("%08x<-%08x\n", regaddr, regval);
    }

  putreg32(regval, regaddr);
}
#else
static inline void spi_putreg(struct max326_spidev_s *priv,
                              unsigned int offset, uint32_t regval)
{
  putreg32(regval, priv->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_modify_ctrl0
 *
 * Description:
 *   Clear and set bits in the CTRL0 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modify_ctrl0(struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t ctrl0;

  ctrl0  = spi_getreg(priv, MAX326_SPI_CTRL0_OFFSET);
  ctrl0 &= ~clrbits;
  ctrl0 |= setbits;
  spi_putreg(priv, MAX326_SPI_CTRL0_OFFSET, ctrl0);
}

/****************************************************************************
 * Name: spi_modify_ctrl1
 *
 * Description:
 *   Clear and set bits in the CTRL1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modify_ctrl1(struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t ctrl1;

  ctrl1  = spi_getreg(priv, MAX326_SPI_CTRL1_OFFSET);
  ctrl1 &= ~clrbits;
  ctrl1 |= setbits;
  spi_putreg(priv, MAX326_SPI_CTRL1_OFFSET, ctrl1);
}

/****************************************************************************
 * Name: spi_modify_ctrl2
 *
 * Description:
 *   Clear and set bits in the CTRL1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modify_ctrl2(struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t ctrl2;

  ctrl2  = spi_getreg(priv, MAX326_SPI_CTRL2_OFFSET);
  ctrl2 &= ~clrbits;
  ctrl2 |= setbits;
  spi_putreg(priv, MAX326_SPI_CTRL2_OFFSET, ctrl2);
}

/****************************************************************************
 * Name: spi_modify_dma
 *
 * Description:
 *   Clear and set bits in the DMA register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modify_dma(struct max326_spidev_s *priv, uint32_t setbits,
                           uint32_t clrbits)
{
  uint32_t dma;

  dma  = spi_getreg(priv, MAX326_SPI_DMA_OFFSET);
  dma &= ~clrbits;
  dma |= setbits;
  spi_putreg(priv, MAX326_SPI_DMA_OFFSET, dma);
}

/****************************************************************************
 * Name: spi_modify_inten
 *
 * Description:
 *   Clear and set bits in the INTEN register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326_SPI_INTERRUPTS
static void spi_modify_inten(struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t inten;

  inten  = spi_getreg(priv, MAX326_SPI_INTEN_OFFSET);
  inten &= ~clrbits;
  inten |= setbits;
  spi_putreg(priv, MAX326_SPI_INTEN_OFFSET, inten);
}
#endif

/****************************************************************************
 * Name: spi_poll
 *
 * Description:
 *   Handle SPI events.  This may be called repeatedly in polled mode or may
 *   be called from spi_interrupt() in interrupt mode.
 *
 * Returned Value:
 *   Returns OK when the transfer is complete, -EBUSY is the transfer is
 *   still in progress.  Could also return other negated errno values if
 *   an error is detected.
 *
 ****************************************************************************/

static int spi_poll(struct max326_spidev_s *priv)
{
  const uint8_t *src;
  uint8_t *dest;
  uint32_t inten;
  uint32_t length;
  uint32_t regval;
  uint32_t tmp;
  int txavail;
  int rxavail;
  int remaining;

  /* Get the transfer size in units of bytes */

  length = priv->data16 ? priv->xfrlen << 1 : priv->xfrlen;

  /* Is there a Tx source buffer? */

  inten = 0;
  if (priv->txbuffer != NULL)
    {
      /* Need to know when all bytes are transmitted. */

      inten |= SPI_INT_TXEMPTY;

      /* Calculate how many bytes we can write to the FIFO */

      regval  = spi_getreg(priv, MAX326_SPI_DMA_OFFSET);
      tmp     = (regval & SPI_DMA_TXFIFOCNT_MASK) >> SPI_DMA_TXFIFOCNT_SHIFT;
      txavail =  MAX326_SPI_FIFO_DEPTH - tmp;

      if ((length - priv->txbytes) < txavail)
        {
          txavail = (length - priv->txbytes);
        }

      if (priv->data16)
        {
          txavail &= ~1;
        }

      /* Write Tx buffer data to the Tx FIFO */

      src = &((const uint8_t *)priv->txbuffer)[priv->txbytes];
      while (txavail > 0)
        {
          dest = (uint8_t *)(priv->base + MAX326_SPI_DATA_OFFSET);

          if (txavail > 3)
            {
              *dest++        = *src++;
              *dest++        = *src++;
              *dest++        = *src++;
              *dest          = *src++;

              txavail       -= 4;
              priv->txbytes += 4;
            }
          else if (txavail > 1)
            {
              *dest++        = *src++;
              *dest          = *src++;

              txavail       -= 2;
              priv->txbytes += 2;
            }
          else if (!priv->data16)
            {
              *dest          = *src++;

              txavail       -= 1;
              priv->txbytes += 1;
            }
        }
    }

  remaining = length - priv->txbytes;

  /* Set the TX interrupts */

  if (remaining > 0)
    {
      if (remaining > MAX326_SPI_FIFO_DEPTH)
        {
          /* Set the TX FIFO almost empty interrupt if we have to refill */

          regval = SPI_DMA_TXFIFOLVL(MAX326_SPI_FIFO_DEPTH);
        }
      else
        {
          regval = SPI_DMA_TXFIFOLVL(remaining);
        }

      spi_modify_dma(priv, regval, SPI_DMA_TXFIFOLVL_MASK);
      inten |= SPI_INT_TXLEVEL;
    }

  /* Break out if we've transmitted all the bytes and not receiving */

  if ((priv->rxbuffer == NULL) && priv->txbytes == length &&
      (spi_getreg(priv, MAX326_SPI_DMA_OFFSET) &
       SPI_DMA_TXFIFOCNT_MASK) == 0)
    {
      goto done;
    }

  /* Read from the RX FIFO */

  if (priv->rxbuffer != NULL)
    {
      /* Wait for there to be data in the RX FIFO */

      regval  = spi_getreg(priv, MAX326_SPI_DMA_OFFSET);
      rxavail = (regval & SPI_DMA_RXFIFOCNT_MASK) >> SPI_DMA_RXFIFOCNT_SHIFT;

      if (length - priv->rxbytes < rxavail)
        {
          rxavail = length - priv->rxbytes;
        }

      dest = &((uint8_t *)priv->rxbuffer)[priv->rxbytes];
      if (!priv->data16 || rxavail >= sizeof(uint16_t))
        {
          /* Read data from the Rx FIFO */

          src = (const uint8_t *)(priv->base + MAX326_SPI_DATA_OFFSET);
          while (rxavail > 0)
            {
              if (rxavail > 3)
                {
                  *dest++        = *src++;
                  *dest++        = *src++;
                  *dest++        = *src++;
                  *dest++        = *src;

                  rxavail       -= 4;
                  priv->rxbytes += 4;
                }
              else if (rxavail > 1)
                {
                  *dest++        = *src++;
                  *dest++        = *src;

                  rxavail       -= 2;
                  priv->rxbytes += 2;
                }
              else
                {
                  *dest++        = *src;

                  rxavail       -= 1;
                  priv->rxbytes += 1;
                }

              /* Don't read less than 2 bytes if we are using 16-bit data
               * transfers.
               */

              if (rxavail < sizeof(uint16_t) && priv->data16)
                {
                  break;
                }
            }
        }

      remaining = length - priv->rxbytes;
      if (remaining > 0)
        {
          if (remaining > MAX326_SPI_FIFO_DEPTH)
            {
              regval = SPI_DMA_RXFIFOLVL(2);
            }
          else
            {
              regval = SPI_DMA_RXFIFOLVL(remaining - 1);
            }

          spi_modify_dma(priv, regval, SPI_DMA_RXFIFOLVL_MASK);
          inten |= SPI_INT_RXLEVEL;
        }

      /* Break out if we've received all the bytes and we're not
       * transmitting.
       */

      if (priv->txbuffer == NULL && priv->rxbytes == length)
        {
          goto done;
        }
    }

  /* Break out once we've transmitted and received all of the data. */

  if (priv->rxbytes == length && priv->txbytes == length)
    {
      regval = spi_getreg(priv, MAX326_SPI_DMA_OFFSET);
      if ((regval & SPI_DMA_TXFIFOCNT_MASK) == 0)
        {
          goto done;
        }
    }

#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  /* Enable interrupts for the next phase */

  spi_putreg(priv, MAX326_SPI_INTEN_OFFSET, inten);
#endif

  /* Return busy to indicate that we are not finished with the transfer */

  return -EBUSY;

done:
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  /* We are done.. disable interrupts */

  spi_putreg(priv, MAX326_SPI_INTEN_OFFSET, 0);
#endif

  /* Return OK to indicate that we are finished with the transfer */

  return OK;
}

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   This is the SPI interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'.
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326_SPI_INTERRUPTS
static int spi_interrupt(int irq, void *context, void *arg)
{
  struct max326_spidev_s *priv = (struct max326_spidev_s *)arg;
  uint32_t intfl;
  uint32_t regval;
  unsigned int rxavail;
  unsigned int rxlevel;
  int ret;

  /* Read pending interrupt flags, interrupt enables, and SPI status
   * registers.
   */

  intfl = spi_getreg(priv, MAX326_SPI_INTFL_OFFSET);

  /* Disable all interrupts */

  spi_putreg(priv, MAX326_SPI_INTEN_OFFSET, 0);

  /* Clear pending interrupt flags */

  max326_serialout(priv, MAX326_SPI_INTFL_OFFSET, intfl & SPI_INT_ALL);

  /* Handle any active request */

  if (intfl != 0)
    {
      do
        {
          ret = spi_poll(priv);
          DEBUGASSERT(ret == OK || ret == -EBUSY);

          /* Check if there is more Rx data to be read */

          regval  = spi_getreg(priv, MAX326_SPI_DMA_OFFSET);
          rxavail = (regval & SPI_DMA_TXFIFOCNT_MASK) >>
                     SPI_DMA_TXFIFOCNT_SHIFT;
          rxlevel = (regval & SPI_DMA_RXFIFOLVL_MASK) >>
                     SPI_DMA_RXFIFOLVL_SHIFT;
        }
      while (/* RX buffer != NULL && */ rxavail > rxlevel);
    }

  return OK;
}
#endif

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
  struct max326_spidev_s *priv = (struct max326_spidev_s *)dev;
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
 * Name: SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_SPIM0
static void spi0_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct max326_spidev_s *priv;

  /* Forward the call to the board-specific implementation */

  DEBUGASSERT(dev != NULL);
  max326_spi0select(dev, devid, selected);

  /* The hardware requires that the SPI block be disabled and re-enabled at
   * end of each transaction to cancel the on ongoing transaction (while
   * chip select is inactive).
   */

  if (!selected)
    {
      priv = (struct max326_spidev_s *)dev;
      spi_modify_ctrl0(priv, 0, SPI_CTRL0_SPIEN);
    }
}
#endif

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

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct max326_spidev_s *priv = (struct max326_spidev_s *)dev;

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      uint32_t pclk;
      uint32_t tmpbaud;
      uint32_t actual;
      uint32_t error;
      uint32_t regval;
      unsigned int tmpscale;
      unsigned int tmphigh;
      unsigned int scale;
      unsigned int high;

      /* The SPI clock derives from the PCLK:
       *
       *   Fspi = Fpclk / (1 << CLKCFG.scale)
       *
       * The SPI high time is determined by Tspi and CLKCFG.high:
       *
       *   Thi = Tspi * CLKCFG.high
       *
       * Similar the SPI low time id determined by Tspi and CLKCFG.low:
       *
       *   Tlow = Tspi * CLKCFG.low
       *
       * The BAUD is the given by:
       *
       *   Fbaud = 1 / (Thi + Tlow)
       *
       * If we assume that Thi == Tlow, then:
       *
       *   Thi   = Tspi * CLKCFG.high
       *   Fbaud = 1 / (2 * Thi)
       *
       * And
       *
       *   Fbaud       = Fpclk / CLKCFG.high / (1 << (CLKCFG.scale + 1)
       *   CLKCFG.high = Fpclk / (1 << (CLKCFG.scale + 1)) / Fbaud
       *
       * Example:  Fpclk = 48MHz, Fbaud=1MHz
       *
       *   scale  CLKCFG.high Resulting Fbaud
       *     0    24         1,000,0000 (exact)
       *     1    12         1,000,0000 (exact)
       *     2    6          1,000,0000 (exact)
       *     3    3          1,000,0000 (exact)
       *     4    1          1,500,0000
       *     5    0          Invalid
       *     6    0          Invalid
       *     7    0          Invalid
       *     8    0          Invalid
       *
       *   Fbaud / 2 = Fspi / CLKCFG.high
       *   Fbaud = 2 * Fspi / CLKCFG.high
       */

      /* There are only 9 possible values for CLKCFG.scale.  Let's try them
       * all and pick the best.
       */

      pclk  = max326_pclk_frequency();
      DEBUGASSERT(frequency > 0 && frequency <= pclk);
      error = UINT32_MAX;

      for (tmpscale = 0; tmpscale < 9; tmpscale++)
        {
          tmphigh = (pclk / frequency) >> (tmpscale + 1);
          if (tmphigh > 0)
            {
              uint32_t tmperr;

              /* Calculate the frequency difference */

              tmpbaud = (pclk / tmphigh) >> (tmpscale + 1);
              if (tmpbaud > frequency)
                {
                  tmperr = tmpbaud - frequency;
                }
              else
                {
                  tmperr = frequency - tmpbaud;
                }

              /* Save the best result we find (or the lowest scale in case
               * of ties).
               */

              if (tmperr < error)
                {
                  error  = tmperr;
                  actual = tmpbaud;
                  scale  = tmpscale;
                  high   = tmphigh;
                }
            }
        }

      regval  = spi_getreg(priv, MAX326_SPI_CLKCFG_OFFSET);
      regval &= ~(SPI_CLKCFG_LO_MASK | SPI_CLKCFG_HI_MASK |
                  SPI_CLKCFG_SCALE_MASK);
      regval |= (SPI_CLKCFG_LO(high) | SPI_CLKCFG_HI(high) |
                 SPI_CLKCFG_SCALE(scale));
      spi_putreg(priv, MAX326_SPI_CLKCFG_OFFSET, regval);

      /* Save the frequency selection so that subsequent reconfigurations
       * will be faster.
       */

      spiinfo("Frequency %d->%d\n", frequency, actual);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

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
  struct max326_spidev_s *priv = (struct max326_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CTRL2_CLKPOL | SPI_CTRL2_CLKPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CTRL2_CLKPHA;
          clrbits = SPI_CTRL2_CLKPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CTRL2_CLKPOL;
          clrbits = SPI_CTRL2_CLKPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CTRL2_CLKPOL | SPI_CTRL2_CLKPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

        spi_modify_ctrl2(priv, setbits, clrbits);

        /* Save the mode so that subsequent re-configurations will be
         * faster.
         */

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
  struct max326_spidev_s *priv = (struct max326_spidev_s *)dev;

  spiinfo("nbits=%d\n", nbits);
  DEBUGASSERT(nbits > 0 && nbits <= 16);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set CTRL2 appropriately */

      spi_modify_ctrl2(priv, SPI_CTRL2_NUMBITS(nbits),
                       SPI_CTRL2_NUMBITS_MASK);

      /* Will we have to do 16- or 8-bit data transfers? */

      priv->data16 = (nbits > 8);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits  = nbits;
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
static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  return -ENOSYS;
}
#endif

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
  uint32_t ret;

  spiinfo("wd=%04u\n", wd);
  spi_exchange(dev, &wd, &ret, 1);
  return ret;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data on SPI
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

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct max326_spidev_s *priv = (struct max326_spidev_s *)dev;
#ifndef CONFIG_MAX326_SPI_INTERRUPTS
  uint32_t regval;
#endif
  int ret;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
  DEBUGASSERT(priv != NULL && nwords > 0 && nwords <= UINT16_MAX);
  DEBUGASSERT(txbuffer != NULL || rxbuffer != NULL);

  /* Setup for the transfer */

  priv->txbuffer = txbuffer;
  priv->rxbuffer = rxbuffer;
  priv->txbytes  = 0;
  priv->rxbytes  = 0;
  priv->xfrlen   = nwords;
  priv->busy     = false;

  /* NOTE: The MAX326 slave select pin is not used.  It is controlled as a
   * GPIO output from the upper half driver.
   */

  /* Setup the Rx number of words to exchange */

  if (rxbuffer != NULL)
    {
      /* Note: If the SPI port is set to operate in 4-wire mode, the RXNUM
       * field is ignored and the TXNUM field is used for both the number
       * of words to receive or transmit.
       */

      if (priv->wire3)
        {
          spi_modify_ctrl1(priv, SPI_CTRL1_RXNUMCH(nwords),
                           SPI_CTRL1_RXNUMCH_MASK);
        }
       else
        {
          spi_modify_ctrl1(priv, SPI_CTRL1_TXNUMCH(nwords),
                           SPI_CTRL1_TXNUMCH_MASK);
        }

      /* Enable the DMA Rx FIFO */

      spi_modify_dma(priv, SPI_DMA_RXFIFOEN, 0);
    }
  else
    {
      /* No Rx data sink */

      spi_modify_ctrl1(priv, 0, SPI_CTRL1_RXNUMCH_MASK);
      spi_modify_dma(priv, 0, SPI_DMA_RXFIFOEN);
    }

  /* Four wire mode always operates in full duplex.  We must have some Tx
   * buffer available in all cases.
   */

  if (!priv->wire3 && txbuffer == NULL)
    {
      size_t nbytes = priv->data16 ? nwords << 1 : nwords;

      /* We will dual-purpose the the Rx buffer, initialized to zero */

      memset(priv->rxbuffer, 0, nbytes);
      priv->txbuffer = priv->rxbuffer;
    }

  /* Set up the number of Tx words to exchange */

  if (priv->txbuffer != NULL)
    {
      spi_modify_ctrl1(priv, SPI_CTRL1_TXNUMCH(nwords),
                       SPI_CTRL1_TXNUMCH_MASK);

      /* Enable the DMA Tx FIFO */

      spi_modify_dma(priv, SPI_DMA_TXFIFOEN, 0);
    }
  else
    {
      /* No Tx data source */

      spi_modify_dma(priv, 0, SPI_DMA_TXFIFOEN);
    }

  /* Flush the Rx and Tx FIFOs */

  spi_modify_dma(priv, SPI_DMA_TXFIFOCLR | SPI_DMA_RXFIFOCLR, 0);

  /* Clear pending interrupts */

  regval = spi_getreg(priv, MAX326_SPI_INTFL_OFFSET);
  spi_putreg(priv, MAX326_SPI_INTFL_OFFSET, regval);

  /* Re-enable SPI to start the next transfer */

  spi_modify_ctrl0(priv, SPI_CTRL0_SPIEN, 0);

  /* Poll one time to setup the transfer */

  ret = spi_poll(priv);
  DEBUGASSERT(ret == OK || ret == -EBUSY);

  /* Initiate data transmission */

  spi_modify_ctrl0(priv, SPI_CTRL0_START, 0);
  priv->busy = true;

#ifndef CONFIG_MAX326_SPI_INTERRUPTS
  /* Poll repeatedly until the transfer is complete */

  do
    {
      ret = spi_poll(priv);
    }
  while (ret != OK);

  /* Then wait for the master done interrupt */

  do
    {
      regval = spi_getreg(priv, MAX326_SPI_INTFL_OFFSET);
    }
  while ((regval & SPI_INT_MDONE) == 0);
#endif
}

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
 *              words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  spi_exchange(dev, txbuffer, NULL, nwords);
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
  spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_bus_initialize
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

static void spi_bus_initialize(struct max326_spidev_s *priv)
{
  uint32_t setbits;
  uint32_t clrbits;
  uint32_t regval;

  /* Enable SPI */

  spi_modify_ctrl0(priv, SPI_CTRL0_SPIEN, 0);

  /* Setup slaved select timing (even in Master mode?) */

  regval = (SPI_SSTIME_SSACT1(1) | SPI_SSTIME_SSACT2(1) |
            SPI_SSTIME_SSINACT(1));
  spi_putreg(priv, MAX326_SPI_SSTIME_OFFSET, regval);

  /* Configure CTRL0. Default configuration:
   *   Mode 0:                        CTRL2: CLKPHA=0 and CLKPOL=0
   *   8-bit:                         CTRL2: NUMBITS=8
   */

  clrbits = SPI_CTRL2_CLKPHA | SPI_CTRL2_CLKPOL | SPI_CTRL2_NUMBITS_MASK;
  setbits = SPI_CTRL2_NUMBITS(8);
  spi_modify_ctrl2(priv, setbits, clrbits);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Enable Master mode */

  spi_modify_ctrl0(priv, SPI_CTRL0_MMEN, 0);

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Set up 3- or 4-pin mode */

  regval = priv->wire3 ? SPI_CTRL2_DATWIDTH_SINGLE : SPI_CTRL2_DATWIDTH_DUAL;
  spi_modify_ctrl2(priv, regval, SPI_CTRL2_DATWIDTH_MASK);

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);

  /* Disable all interrupts at the peripheral */

  spi_putreg(priv, MAX326_SPI_INTEN_OFFSET, 0);

  /* Clear pending interrupts */

  regval = spi_getreg(priv, MAX326_SPI_INTFL_OFFSET);
  spi_putreg(priv, MAX326_SPI_INTFL_OFFSET, regval);

#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  /* Attach the interrupt handler and enable the IRQ at the NVIC.
   * Interrupts are (probably) still disabled at the SPI peripheral.
   */

  ret = irq_attach(priv->irq, spi_interrupt, priv);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_spibus_initialize
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

struct spi_dev_s *max326_spibus_initialize(int bus)
{
  struct max326_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_MAX326XX_SPIM0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Enable peripheral clocking */

          max326_spi0_enableclk();

          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          max326_gpio_config(GPIO_SPI0_SCK);
          max326_gpio_config(GPIO_SPI0_MISO);
          max326_gpio_config(GPIO_SPI0_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
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

#endif /* CONFIG_MAX326XX_HAVE_SPIM */
