/****************************************************************************
 * arch/arm/src/imx6/imx_ecspi.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "imx_gpio.h"
#include "imx_ecspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The i.MX6 supports 25SPI interfaces.  Which have been enabled? */

#define __SPI1_NDX       0

#ifdef CONFIG_IMX6_ECSPI1
#  define SPI1_NDX       __SPI1_NDX
#  define __SPI1_PRESENT 1
#  define __SPI2_NDX     (__SPI1_NDX + 1)
#else
#  define __SPI1_PRESENT 0
#  define __SPI2_NDX     __SPI1_NDX
#endif

#ifdef CONFIG_IMX6_ECSPI2
#  define SPI2_NDX       __SPI2_NDX
#  define __SPI2_PRESENT 1
#  define __SPI3_NDX     (__SPI2_NDX + 1)
#else
#  define __SPI2_PRESENT 0
#  define __SPI3_NDX     __SPI2_NDX
#endif

#ifdef CONFIG_IMX6_ECSPI3
#  define SPI3_NDX       __SPI3_NDX
#  define __SPI3_PRESENT 1
#  define __SPI4_NDX     (__SPI3_NDX + 1)
#else
#  define __SPI3_PRESENT 0
#  define __SPI4_NDX     __SPI3_NDX
#endif

#ifdef CONFIG_IMX6_ECSPI4
#  define SPI4_NDX       __SPI4_NDX
#  define __SPI4_PRESENT 1
#  define __SPI5_NDX     (__SPI4_NDX + 1)
#else
#  define __SPI4_PRESENT 0
#  define __SPI5_NDX     __SPI5_NDX
#endif

#ifdef CONFIG_IMX6_ECSPI5
#  define SPI5_NDX       __SPI5_NDX
#  define __SPI5_PRESENT 1
#  define __SPI6_NDX     (__SPI5_NDX + 1)
#else
#  define __SPI5_PRESENT 0
#  define __SPI6_NDX     __SPI5_NDX
#endif

#define NSPIS            (__SPI1_PRESENT + __SPI2_PRESENT + __SPI3_PRESENT + \
                          __SPI4_PRESENT + __SPI5_PRESENT)

/* Compile the rest of the file only if at least one SPI interface has been
 * enabled.
 */

#if NSPIS > 0

/* The number of words that will fit in the Tx FIFO */

#define IMX_TXFIFO_WORDS 8

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Per SPI callouts to board-specific logic */

typedef void (*imx_select_t)(struct spi_dev_s *dev,
                             uint32_t devid, bool selected);
typedef uint8_t (*imx_status_t)(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
typedef int (*imx_cmddata_t)(struct spi_dev_s *dev,
                             uint32_t devid, bool cmd);
#endif

struct imx_spidev_s
{
  const struct spi_ops_s *ops;  /* Common SPI operations */
#ifndef CONFIG_SPI_POLLWAIT
  sem_t waitsem;                /* Wait for transfer to complete */
#endif
  mutex_t lock;                 /* Supports mutually exclusive access */

  /* These following are the source and destination buffers of the transfer.
   * they are retained in this structure so that they will be accessible
   * from an interrupt handler.  The actual type of the buffer is uint8_t is
   * nbits <=8 and uint16_t is nbits >8.
   */

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  /* These are functions pointers that are configured to perform the
   * appropriate transfer for the particular kind of exchange that is
   * occurring.  Different functions may be selected depending on (1)
   * if the tx or txbuffer is NULL and depending on the number of bits
   * per word.
   */

  void  (*txword)(struct imx_spidev_s *priv);
  void  (*rxword)(struct imx_spidev_s *priv);

  uint32_t base;                /* SPI register base address */
  uint32_t frequency;           /* Current desired SCLK frequency */
  uint32_t actual;              /* Current actual SCLK frequency */

  int    ntxwords;              /* Number of words left to transfer on the Tx FIFO */
  int    nrxwords;              /* Number of words received on the Rx FIFO */
  int    nwords;                /* Number of words to be exchanged */

  uint8_t  mode;                /* Current mode */
  uint8_t  nbits;               /* Current number of bits per word */
  uint8_t  spindx;              /* SPI index */
#ifndef CONFIG_SPI_POLLWAIT
  uint8_t  irq;                 /* SPI IRQ number */
#endif

  /* Per SPI callouts to board-specific logic */

  imx_select_t select;          /* Select callout */
  imx_status_t status;          /* Status callout */
#ifdef CONFIG_SPI_CMDDATA
  imx_cmddata_t cmddata;        /* Cmddata callout */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI register access */

static inline uint32_t spi_getreg(struct imx_spidev_s *priv,
                                  unsigned int offset);
static inline void spi_putreg(struct imx_spidev_s *priv,
                              unsigned int offset, uint32_t value);

/* SPI data transfer */

static void   spi_txnull(struct imx_spidev_s *priv);
static void   spi_txuint16(struct imx_spidev_s *priv);
static void   spi_txuint8(struct imx_spidev_s *priv);
static void   spi_rxnull(struct imx_spidev_s *priv);
static void   spi_rxuint16(struct imx_spidev_s *priv);
static void   spi_rxuint8(struct imx_spidev_s *priv);
static int    spi_performtx(struct imx_spidev_s *priv);
static inline void spi_performrx(struct imx_spidev_s *priv);
static int    spi_transfer(struct imx_spidev_s *priv, const void *txbuffer,
                           void *rxbuffer, unsigned int nwords);

/* Interrupt handling */

#ifndef CONFIG_SPI_POLLWAIT
static int    spi_interrupt(int irq, void *context, void *arg);
#endif

/* SPI methods */

static int    spi_lock(struct spi_dev_s *dev, bool lock);
static void   spi_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void   spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void   spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#endif
#ifdef CONFIG_SPI_EXCHANGE
static void   spi_exchange(struct spi_dev_s *dev,
                           const void *txbuffer,
                           void *rxbuffer, size_t nwords);
#else
static void   spi_sndblock(struct spi_dev_s *dev,
                           const void *buffer, size_t nwords);
static void   spi_recvblock(struct spi_dev_s *dev,
                            void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SPI operations */

static const struct spi_ops_s g_spiops =
{
  .lock         = spi_lock,
  .select       = spi_select,       /* Provided externally by board logic */
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                /* Not supported */
#endif
  .status       = spi_status,       /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = spi_cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
};

/* This supports is up to five SPI buses/ports */

static struct imx_spidev_s g_spidev[] =
{
#ifdef CONFIG_IMX6_ECSPI1
    {
      .ops     = &g_spiops,
      .base    = IMX_ECSPI1_VBASE,
      .spindx  = SPI1_NDX,
#ifndef CONFIG_SPI_POLLWAIT
      .irq     = IMX_IRQ_ECSPI1,
#endif
      .select  = imx_spi1select,
      .status  = imx_spi1status,
#ifdef CONFIG_SPI_CMDDATA
      .cmddata = imx_spi1cmddata,
#endif
    },
#endif

#ifdef CONFIG_IMX6_ECSPI2
    {
      .ops     = &g_spiops,
      .base    = IMX_ECSPI2_VBASE,
      .spindx  = SPI2_NDX,
#ifndef CONFIG_SPI_POLLWAIT
      .irq     = IMX_IRQ_ECSPI2,
#endif
      .select  = imx_spi2select,
      .status  = imx_spi2status,
#ifdef CONFIG_SPI_CMDDATA
      .cmddata = imx_spi2cmddata,
#endif
    },
#endif

#ifdef CONFIG_IMX6_ECSPI3
    {
      .ops     = &g_spiops,
      .base    = IMX_ECSPI3_VBASE,
      .spindx  = SPI3_NDX,
#ifndef CONFIG_SPI_POLLWAIT
      .irq     = IMX_IRQ_ECSPI3,
#endif
      .select  = imx_spi3select,
      .status  = imx_spi3status,
#ifdef CONFIG_SPI_CMDDATA
      .cmddata = imx_spi3cmddata,
#endif
    },
#endif

#ifdef CONFIG_IMX6_ECSPI4
    {
      .ops     = &g_spiops,
      .base    = IMX_ECSPI4_VBASE,
      .spindx  = SPI4_NDX,
#ifndef CONFIG_SPI_POLLWAIT
      .irq     = IMX_IRQ_ECSPI4,
#endif
      .select  = imx_spi4select,
      .status  = imx_spi4status,
#ifdef CONFIG_SPI_CMDDATA
      .cmddata = imx_spi4cmddata,
#endif
    },
#endif

#ifdef CONFIG_IMX6_ECSPI5
    {
      .ops     = &g_spiops,
      .base    = IMX_ECSPI5_VBASE,
      .spindx  = SPI5_NDX,
#ifndef CONFIG_SPI_POLLWAIT
      .irq     = IMX_IRQ_ECSPI5,
#endif
      .select  = imx_spi5select,
      .status  = imx_spi5status,
#ifdef CONFIG_SPI_CMDDATA
      .cmddata = imx_spi5cmddata,
#endif
    }
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Read the SPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct imx_spidev_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write the value to the SPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(struct imx_spidev_s *priv, unsigned int offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: spi_txnull, spi_txuint16, and spi_txuint8
 *
 * Description:
 *   Transfer all ones, a uint8_t, or uint16_t to Tx FIFO and update the
 *   txbuffer pointer appropriately.  The selected function dependes on
 *   (1) if there is a source txbuffer provided, and (2) if the number of
 *   bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_txnull(struct imx_spidev_s *priv)
{
  spi_putreg(priv, ECSPI_TXDATA_OFFSET, 0xffff);
}

static void spi_txuint16(struct imx_spidev_s *priv)
{
  uint16_t *ptr = (uint16_t *)priv->txbuffer;
  spi_putreg(priv, ECSPI_TXDATA_OFFSET, *ptr++);
  priv->txbuffer = (void *)ptr;
}

static void spi_txuint8(struct imx_spidev_s *priv)
{
  uint8_t *ptr = (uint8_t *)priv->txbuffer;
  spi_putreg(priv, ECSPI_TXDATA_OFFSET, *ptr++);
  priv->txbuffer = (void *)ptr;
}

/****************************************************************************
 * Name: spi_rxnull,spi_rxuint16, and spi_rxuint8
 *
 * Description:
 *   Discard input, save a uint8_t, or or save a uint16_t from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a destination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_rxnull(struct imx_spidev_s *priv)
{
  spi_getreg(priv, ECSPI_RXDATA_OFFSET);
}

static void spi_rxuint16(struct imx_spidev_s *priv)
{
  uint16_t *ptr = (uint16_t *)priv->rxbuffer;
  *ptr++ = (uint16_t)spi_getreg(priv, ECSPI_TXDATA_OFFSET);
  priv->rxbuffer = (void *)ptr;
}

static void spi_rxuint8(struct imx_spidev_s *priv)
{
  uint8_t *ptr = (uint8_t *)priv->rxbuffer;
  *ptr++ = (uint8_t)spi_getreg(priv, ECSPI_TXDATA_OFFSET);
  priv->rxbuffer = (void *)ptr;
}

/****************************************************************************
 * Name: spi_performtx
 *
 * Description:
 *   If the Tx FIFO is empty, then transfer as many words as we can to
 *   the FIFO.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   The number of words written to the Tx FIFO (a value from 0 to 8,
 *   inclusive).
 *
 ****************************************************************************/

static int spi_performtx(struct imx_spidev_s *priv)
{
  uint32_t regval;
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is empty */

  if ((spi_getreg(priv, ECSPI_STATREG_OFFSET) & ECSPI_INT_TE) != 0)
    {
      /* Check if all of the Tx words have been sent */

      if (priv->ntxwords > 0)
        {
          /* No.. Transfer more words until either the TxFIFO is full or
           * until all of the user provided data has been sent.
           */

          for (; ntxd < priv->ntxwords && ntxd < IMX_TXFIFO_WORDS; ntxd++)
            {
               priv->txword(priv);
            }

          /* Update the count of words to be transferred */

          priv->ntxwords -= ntxd;
        }
      else
        {
          /* Yes..
           * The transfer is complete, disable Tx FIFO empty interrupt
           */

          regval = spi_getreg(priv, ECSPI_INTREG_OFFSET);
          regval &= ~ECSPI_INT_TE;
          spi_putreg(priv, ECSPI_INTREG_OFFSET, regval);
        }
    }

  return ntxd;
}

/****************************************************************************
 * Name: spi_performrx
 *
 * Description:
 *   Transfer as many bytes as possible from the Rx FIFO to the user Rx
 *   buffer (if one was provided).
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_performrx(struct imx_spidev_s *priv)
{
  /* Loop while data is available in the Rx FIFO */

  while ((spi_getreg(priv, ECSPI_STATREG_OFFSET) & ECSPI_INT_RR) != 0)
    {
      /* Have all of the requested words been transferred from the Rx FIFO? */

      if (priv->nrxwords < priv->nwords)
        {
          /* No.. Read more data from Rx FIFO */

          priv->rxword(priv);
          priv->nrxwords++;
        }
    }
}

/****************************************************************************
 * Name: spi_startxfr
 *
 * Description:
 *   If data was added to the Tx FIFO, then start the exchange
 *
 * Input Parameters:
 *   priv  - Device-specific state data
 *   ntxd  - The number of bytes added to the Tx FIFO by spi_performtx.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_startxfr(struct imx_spidev_s *priv, int ntxd)
{
  uint32_t regval;

  /* The XCH bit initiates an exchange in master mode.  It remains set
   * remains set while the exchange is in progress but is automatically
   * clear when all data in the Tx FIFO and shift register are shifted out.
   * So if we have added data to the Tx FIFO on this interrupt, we must
   * set the XCH bit to resume the exchange.
   */

  if (ntxd > 0)
    {
      regval = spi_getreg(priv, ECSPI_CONREG_OFFSET);
      regval |= ECSPI_CONREG_XCH;
      spi_putreg(priv, ECSPI_CONREG_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of
 *              uint8_t's; if the interface uses >8 bits per word, then this
 *              is the number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

static int spi_transfer(struct imx_spidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
#ifndef CONFIG_SPI_POLLWAIT
  irqstate_t flags;
  uint32_t regval;
#endif
  int ntxd;

  /* Set up to perform the transfer */

  priv->txbuffer     = (uint8_t *)txbuffer; /* Source buffer */
  priv->rxbuffer     = (uint8_t *)rxbuffer; /* Destination buffer */
  priv->ntxwords     = nwords;              /* Number of words left to send */
  priv->nrxwords     = 0;                   /* Number of words received */
  priv->nwords       = nwords;              /* Total number of exchanges */

  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 8)
    {
      priv->txword = spi_txuint16;
      priv->rxword = spi_rxuint16;
    }
  else
    {
      priv->txword = spi_txuint8;
      priv->rxword = spi_rxuint8;
    }

  if (!txbuffer)
    {
      priv->txword = spi_txnull;
    }

  if (!rxbuffer)
    {
      priv->rxword = spi_rxnull;
    }

  /* Prime the Tx FIFO to start the sequence (saves one interrupt) */

#ifndef CONFIG_SPI_POLLWAIT
  flags = enter_critical_section();
  ntxd  = spi_performtx(priv);
  spi_startxfr(priv, ntxd);

  /* Enable transmit empty interrupt */

  regval  = spi_getreg(priv, ECSPI_INTREG_OFFSET);
  regval |= ECSPI_INT_TE;
  spi_putreg(priv, ECSPI_INTREG_OFFSET, regval);
  leave_critical_section(flags);

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  /* Wait to be signaled from the interrupt handler */

  return nxsem_wait_uninterruptible(&priv->waitsem);

#else
  /* Perform the transfer using polling logic.  This will totally
   * dominate the CPU until the transfer is complete.  Only recommended
   * if (1) your SPI is very fast, and (2) if you only use very short
   * transfers.
   */

  do
    {
      /* Handle outgoing Tx FIFO transfers */

      ntxd = spi_performtx(priv);

      /* Handle incoming Rx FIFO transfers */

      spi_performrx(priv);

      /* Resume the transfer */

      spi_startxfr(priv, ntxd);

      /* If there are other threads at this same priority level,
       * the following may help:
       */

      sched_yield();
    }
  while (priv->nrxwords < priv->nwords);

  return OK;
#endif
}

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   Common ECSPI interrupt handling logic
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int spi_interrupt(int irq, void *context, void *arg)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)arg;
  int ntxd;

  DEBUGASSERT(priv != NULL);

  /* Handle outgoing Tx FIFO transfers */

  ntxd = spi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */

  spi_performrx(priv);

  /* Resume the transfer */

  spi_startxfr(priv, ntxd);

  /* Check if the transfer is complete */

  if (priv->nrxwords >= priv->nwords)
    {
      /* Yes, wake up the waiting thread */

      nxsem_post(&priv->waitsem);
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
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
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
 * Name: spi_select
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

static void spi_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;

  DEBUGASSERT(priv != NULL && priv->select != NULL);
  priv->select(dev, devid, selected);
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
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint32_t actual;

  DEBUGASSERT(priv != NULL);
  actual = priv->actual;

  if (frequency != priv->frequency)
    {
      uint32_t freqbits;
      uint32_t regval;

      if (frequency >= IMX_PERCLK2_FREQ / 4)
        {
          freqbits = ECSPI_CONREG_DIV4;
          actual   = IMX_PERCLK2_FREQ / 4;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 8)
        {
          freqbits = ECSPI_CONREG_DIV8;
          actual   = IMX_PERCLK2_FREQ / 8;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 16)
        {
          freqbits = ECSPI_CONREG_DIV16;
          actual   = IMX_PERCLK2_FREQ / 16;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 32)
        {
          freqbits = ECSPI_CONREG_DIV32;
          actual   = IMX_PERCLK2_FREQ / 32;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 64)
        {
          freqbits = ECSPI_CONREG_DIV64;
          actual   = IMX_PERCLK2_FREQ / 64;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 128)
        {
          freqbits = ECSPI_CONREG_DIV128;
          actual   = IMX_PERCLK2_FREQ / 128;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 256)
        {
          freqbits = ECSPI_CONREG_DIV256;
          actual   = IMX_PERCLK2_FREQ / 256;
        }
      else /* if (frequency >= IMX_PERCLK2_FREQ / 512) */
        {
          freqbits = ECSPI_CONREG_DIV512;
          actual   = IMX_PERCLK2_FREQ / 512;
        }

      /* Then set the selected frequency */

      regval = spi_getreg(priv, ECSPI_CONREG_OFFSET);
      regval &= ~(ECSPI_CONREG_DATARATE_MASK);
      regval |= freqbits;
      spi_putreg(priv, ECSPI_CONREG_OFFSET, regval);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  if (priv && mode != priv->mode)
    {
      uint32_t modebits;
      uint32_t regval;

      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0 CPHA=0 */
          modebits = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0 CPHA=1 */
          modebits = ECSPI_CONREG_PHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1 CPHA=0 */
          modebits = ECSPI_CONREG_POL;
         break;

        case SPIDEV_MODE3: /* CPOL=1 CPHA=1 */
          modebits = ECSPI_CONREG_PHA | ECSPI_CONREG_POL;
          break;

        default:
          return;
        }

      /* Then set the selected mode */

      regval = spi_getreg(priv, ECSPI_CONREG_OFFSET);
      regval &= ~(ECSPI_CONREG_PHA | ECSPI_CONREG_POL);
      regval |= modebits;
      spi_putreg(priv, ECSPI_CONREG_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  if (priv && nbits != priv->nbits && nbits > 0 && nbits <= 16)
    {
      uint32_t regval = spi_getreg(priv, ECSPI_CONREG_OFFSET);
      regval       &= ~ECSPI_CONREG_BITCOUNT_MASK;
      regval       |= ((nbits - 1) << ECSPI_CONREG_BITCOUNT_SHIFT);
      spi_putreg(priv, ECSPI_CONREG_OFFSET, regval);
      priv->nbits   = nbits;
    }
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
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint32_t response = 0;

  spi_transfer(priv, &wd, &response, 1);
  return response;
}

/****************************************************************************
 * Name: spi_status
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

static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint8_t ret = 0;

  DEBUGASSERT(priv != NULL);

  if (priv->status != NULL)
    {
      ret = priv->select(dev, devid);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
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
static int spi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                       bool cmd)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  int ret = -ENOSYS;

  DEBUGASSERT(priv != NULL);

  if (priv->cmddata != NULL)
    {
      ret = priv->cmddata(dev, devid, cmd);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   buffer   - A pointer to the buffer of data to be sent
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

#ifdef CONFIG_SPI_EXCHANGE
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  spi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
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

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  spi_transfer(priv, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  spi_transfer(priv, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.
 *   However, If multiple devices on on the bus, then multiple chip selects
 *   will be required.  Therefore, all GPIO chip management is deferred to
 *   board-specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *imx_spibus_initialize(int port)
{
  struct imx_spidev_s *priv;
  uint8_t regval;

  /* Only the SPI1 interface is supported */

  switch (port)
    {
#ifdef CONFIG_IMX6_ECSPI1
    case 1:

      /* Select SPI1 */

      priv = &g_spidev[SPI1_NDX];

      /* Configure SPI1 GPIOs (NOTE that SS is not initialized here, the
       * logic in this file makes no assumptions about chip select)
       */

      imxgpio_configpfinput(GPIOC, 13);  /* Port C, pin 13: RDY */
      imxgpio_configpfoutput(GPIOC, 14); /* Port C, pin 14: SCLK */
      imxgpio_configpfinput(GPIOC, 16);  /* Port C, pin 16: MISO */
      imxgpio_configpfoutput(GPIOC, 17); /* Port C, pin 17: MOSI */
      break;
#endif /* CONFIG_IMX6_ECSPI1 */

#ifdef CONFIG_IMX6_ECSPI2
    case 2:

      /* Select SPI2 */

      priv = &g_spidev[SPI2_NDX];

      /* Configure SPI2 GPIOs */

      /* SCLK: AIN of Port A, pin 0 -OR- AIN of Port D, pin 7 */

#if 1
      imxgpio_configoutput(GPIOA, 0); /* Set GIUS=1 OCR=0 DIR=OUT */
#else
      imxgpio_configoutput(GPIOD, 7); /* Set GIUS=1 OCR=0 DIR=OUT */
#endif

      /* SS: AIN of Port A, pin 17 -OR- AIN of Port D, pin 8.(NOTE that SS
       * is not initialized here, the logic in this file makes no assumptions
       * about chip select)
       */

      /* RXD: AOUT of Port A, pin 1 -OR- AOUT of Port D, pin 9 */

#if 1
      imxgpio_configinput(GPIOA, 1); /* Set GIUS=1 OCR=0 DIR=IN */

      /* Select input from SPI2_RXD_0 pin (AOUT Port A, pin 1) */

      regval = getreg32(IMX_SC_FMCR);
      regval &= ~FMCR_SPI2_RXDSEL;
      putreg32(regval, IMX_SC_FMCR);
#else
      imxgpio_configinput(GPIOD, 9); /* Set GIUS=1 OCR=0 DIR=IN */

      /* Select input from SPI2_RXD_1 pin (AOUT Port D, pin 9) */

      regval = getreg32(IMX_SC_FMCR);
      regval |= FMCR_SPI2_RXDSEL;
      putreg32(regval, IMX_SC_FMCR);
#endif

      /* TXD: BIN of Port D, pin 31 -OR- AIN of Port D, pin 10 */

#if 1
      imxgpio_configinput(GPIOD, 31);
      imxgpio_ocrbin(GPIOD, 31);
      imxgpio_dirout(GPIOD, 31);
#else
      imxgpio_configoutput(GPIOD, 10);
#endif
      break;
#endif /* CONFIG_IMX6_ECSPI2 */

    default:
      return NULL;
    }

  /* Initialize the state structure */

  /* Initialize Semaphores */

#ifndef CONFIG_SPI_POLLWAIT
  nxsem_init(&priv->waitsem, 0, 0);
#endif
  nxmutex_init(&priv->lock);

  /* Initialize control register:
   * min frequency, ignore ready, master mode, mode=0, 8-bit
   */

  spi_putreg(priv, ECSPI_CONREG_OFFSET,
             ECSPI_CONREG_DIV512 |                /* Lowest frequency */
             ECSPI_CONREG_DRCTL_IGNRDY |          /* Ignore ready */
             ECSPI_CONREG_MODE |                  /* Master mode */
             (7 << ECSPI_CONREG_BITCOUNT_SHIFT)); /* 8-bit data */

  /* Make sure state agrees with data */

  priv->mode  = SPIDEV_MODE0;
  priv->nbits = 8;

  /* Set the initial clock frequency for identification mode < 400kHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Enable interrupts on data ready (and certain error conditions */

#ifndef CONFIG_SPI_POLLWAIT
  spi_putreg(priv, ECSPI_INTREG_OFFSET,
             ECSPI_INT_RR |                 /* RXFIFO Data Ready Interrupt Enable */
             ECSPI_INT_RO |                 /* RXFIFO Overflow Interrupt Enable */
             ECSPI_INT_BO);                 /* Bit Count Overflow Interrupt Enable */
#else
  spi_putreg(priv, ECSPI_INTREG_OFFSET, 0);      /* No interrupts */
#endif

  /* Set the clock source=bit clock and number of clocks inserted between
   * transactions = 2.
   */

  spi_putreg(priv, ECSPI_PERIODREG_OFFSET, 2);

  /* No DMA */

  spi_putreg(priv, ECSPI_DMAREG_OFFSET, 0);

  /* Attach the interrupt */

#ifndef CONFIG_SPI_POLLWAIT
  DEBUGVERIFY(irq_attach(priv->irq, spi_interrupt, priv));
#endif

  /* Enable SPI */

  regval = spi_getreg(priv, ECSPI_CONREG_OFFSET);
  regval |= ECSPI_CONREG_SPIEN;
  spi_putreg(priv, ECSPI_CONREG_OFFSET, regval);

  /* Enable SPI interrupts */

#ifndef CONFIG_SPI_POLLWAIT
  up_enable_irq(priv->irq);
#endif
  return (struct spi_dev_s *)priv;
}

#endif /* NSPIS > 0 */
