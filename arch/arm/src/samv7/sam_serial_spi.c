/****************************************************************************
 * arch/arm/src/samv7/sam_serial_spi.c
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
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_config.h"

#include "hardware/sam_pinmap.h"
#include "hardware/sam_uart.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_serial_spi.h"

#ifdef CONFIG_SAMV7_USART_IS_SPI_MASTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAST_USART_CLOCK   BOARD_MCK_FREQUENCY
#define SLOW_USART_CLOCK   (BOARD_MCK_FREQUENCY >> 3)

#define SERIAL_SPI_MAX_DIVIDER 65534
#define SERIAL_SPI_MIN_DIVIDER 6

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_serial_spi_pins_s
{
  uint32_t mosi;
  uint32_t miso;
  uint32_t sck;
  uint32_t nss;
};

struct sam_serial_spi_s
{
  struct sam_serial_spi_pins_s pins;
  uint32_t base;               /* SPI controller register base address */
  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */
  uint8_t mode;                /* Mode 0,1,2,3 */
  uint8_t nbits;               /* Width of word in bits (8 or 9) */
  mutex_t spilock;             /* Assures mutually exclusive access to SPI */
  bool initialized;            /* TRUE: Controller has been initialized */
};

/* The overall state of one SPI controller */

struct sam_spidev_s
{
  const struct spi_ops_s *ops;
  struct sam_serial_spi_s  *priv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t serial_getreg(struct sam_serial_spi_s *priv,
                                     int offset);
static inline void serial_putreg(struct sam_serial_spi_s *priv, int offset,
                                 uint32_t value);
static inline void serial_flush(struct sam_serial_spi_s *priv);

/* SPI master methods */

static int  serial_spi_lock(struct spi_dev_s *dev, bool lock);
static void serial_spi_select(struct spi_dev_s *dev, uint32_t devid,
                            bool selected);
static uint32_t serial_spi_setfrequency(struct spi_dev_s *dev,
                                      uint32_t frequency);
static void serial_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void serial_spi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t serial_spi_status(struct spi_dev_s *dev, uint32_t devid);
static uint32_t serial_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void serial_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                              void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void serial_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                                size_t nwords);
static void serial_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                 size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SERIAL_SPI driver operations */

static const struct spi_ops_s g_spiops =
{
  .lock              = serial_spi_lock,
  .select            = serial_spi_select,
  .setfrequency      = serial_spi_setfrequency,
  .setmode           = serial_spi_setmode,
  .setbits           = serial_spi_setbits,
  .status            = serial_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = NULL,
#endif
  .send              = serial_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = serial_spi_exchange,
#else
  .sndblock          = serial_spi_sndblock,
  .recvblock         = serial_spi_recvblock,
#endif
  .registercallback  = NULL,
};

/* This is the overall state of the SPI0 controller */

#ifdef CONFIG_SAMV7_USART0_SPI_MASTER
static struct sam_serial_spi_s sam_serial0spi_priv =
{
  .pins         =
  {
    .mosi       = GPIO_USART0_TXD,
    .miso       = GPIO_USART0_RXD,
    .sck        = GPIO_USART0_SCK,
    .nss        = GPIO_USART0_RTS,
  },
  .base         = SAM_USART0_BASE,
  .actual       = 0,
  .mode         = 0,
  .nbits        = 0,
  .spilock      = NXMUTEX_INITIALIZER,
  .initialized  = false,
};
#endif

#ifdef CONFIG_SAMV7_USART1_SPI_MASTER
static struct sam_serial_spi_s sam_serial1spi_priv =
{
  .pins         =
  {
    .mosi       = GPIO_USART1_TXD,
    .miso       = GPIO_USART1_RXD,
    .sck        = GPIO_USART1_SCK,
    .nss        = GPIO_USART1_RTS,
  },
  .base         = SAM_USART1_BASE,
  .actual       = 0,
  .mode         = 0,
  .nbits        = 0,
  .spilock      = NXMUTEX_INITIALIZER,
  .initialized  = false,
};
#endif

#ifdef CONFIG_SAMV7_USART2_SPI_MASTER
static struct sam_serial_spi_s sam_serial2spi_priv =
{
  .pins         =
  {
    .mosi       = GPIO_USART2_TXD,
    .miso       = GPIO_USART2_RXD,
    .sck        = GPIO_USART2_SCK,
    .nss        = GPIO_USART2_RTS,
  },
  .base         = SAM_USART2_BASE,
  .actual       = 0,
  .mode         = 0,
  .nbits        = 0,
  .spilock      = NXMUTEX_INITIALIZER,
  .initialized  = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t serial_getreg(struct sam_serial_spi_s *priv,
                                     int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sam_serialout
 ****************************************************************************/

static inline void serial_putreg(struct sam_serial_spi_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: serial_flush
 ****************************************************************************/

static inline void serial_flush(struct sam_serial_spi_s *priv)
{
  uint32_t status;

  /* Make sure the no TX activity is in progress... waiting if necessary */

  status = serial_getreg(priv, SAM_UART_SR_OFFSET);
  while ((status & UART_INT_TXRDY) == 0)
    {
      nxsig_usleep(100);
      status = serial_getreg(priv, SAM_UART_SR_OFFSET);
    }

  /* Then make sure that there is no pending RX data .. reading as
   * discarding as necessary.
   */

  while ((serial_getreg(priv, SAM_UART_SR_OFFSET) & UART_INT_RXRDY) != 0)
    {
       serial_getreg(priv, SAM_UART_RHR_OFFSET);
    }
}

/****************************************************************************
 * Name: serial_spi_lock
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

static int serial_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct sam_spidev_s *spi = (struct sam_spidev_s *)dev;
  struct sam_serial_spi_s *priv = (struct sam_serial_spi_s *)spi->priv;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->spilock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->spilock);
    }

  return ret;
}

/****************************************************************************
 * Name: serial_spi_select
 *
 * Description:
 *   This function does not actually set the chip select line.  Rather, it
 *   simply maps the device ID into a chip select number and retains that
 *   chip select number for later use.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   devid    - Device ID
 *   selected - true if CS is selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void serial_spi_select(struct spi_dev_s *dev, uint32_t devid,
                              bool selected)
{
  struct sam_spidev_s *spi = (struct sam_spidev_s *)dev;
  struct sam_serial_spi_s *priv = (struct sam_serial_spi_s *)spi->priv;

  /* There is only one CS. */

  spiinfo("Chip select %d\n", selected);

  if (selected)
    {
      serial_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_FCS);
    }
  else
    {
      serial_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RCS);
    }
}

/****************************************************************************
 * Name: serial_spi_setfrequency
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

static uint32_t serial_spi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  struct sam_spidev_s *spi = (struct sam_spidev_s *)dev;
  struct sam_serial_spi_s *priv = (struct sam_serial_spi_s *)spi->priv;
  uint32_t intpart;
  uint32_t regval;
  uint32_t selected_clk;
  uint32_t actual;

  spiinfo("frequency=%ld\n", frequency);

  if (priv->frequency == frequency)
    {
      return priv->actual;
    }

  /* Disable receiver and transmitter */

  serial_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RXDIS | UART_CR_TXDIS);

  /* Configure the SPI frequency/baud rate:
   *
   *   divisor = selected clock / baud rate
   */

  selected_clk = FAST_USART_CLOCK;
  intpart = (selected_clk + (frequency >> 1)) / frequency;

  if ((intpart & ~UART_BRGR_CD_MASK) != 0)
    {
      /* Use the divided USART clock */

      selected_clk = SLOW_USART_CLOCK;
      intpart = (selected_clk + (frequency >> 1)) /
                frequency;

      /* Re-select the clock source */

      regval  = serial_getreg(priv, SAM_UART_MR_OFFSET);
      regval &= ~UART_MR_USCLKS_MASK;
      regval |= UART_MR_USCLKS_MCKDIV;
      serial_putreg(priv, SAM_UART_MR_OFFSET, regval);

      /* Value written in UART_BRGR_CD must be even to ensure a
       * 50:50 mark of the SCK pin. This applies only if
       * UART_MR_USCLKS_MCKDIV is selected
       */

      if (intpart % 2 != 0)
        {
          intpart += 1;
        }
    }

  /* Value written in UART_BRGR_CD greater or equal to 6. */

  if (intpart < SERIAL_SPI_MIN_DIVIDER)
    {
      intpart = SERIAL_SPI_MIN_DIVIDER;
    }
  else if (intpart > SERIAL_SPI_MAX_DIVIDER)
    {
      intpart = SERIAL_SPI_MAX_DIVIDER;
    }

  regval = UART_BRGR_CD(intpart);

  serial_putreg(priv, SAM_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  serial_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RXEN | UART_CR_TXEN);

  actual = selected_clk / intpart;

  spiinfo("Frequency %ld->%ld\n", priv->frequency, actual);

  priv->frequency = frequency;
  priv->actual    = actual;

  return actual;
}

/****************************************************************************
 * Name: serial_spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional. See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void serial_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct sam_spidev_s *spi = (struct sam_spidev_s *)dev;
  struct sam_serial_spi_s *priv = (struct sam_serial_spi_s *)spi->priv;
  uint32_t regval;

  spiinfo("mode=%d\n", mode);

  /* Perform operation only if mode has changed. */

  if (mode != priv->mode)
    {
      /* Yes. Set the mode:
       *
       * MODE
       * SPI  CPOL NCPHA
       *  0    0    1
       *  1    0    0
       *  2    1    1
       *  3    1    0
       */

      regval  = serial_getreg(priv, SAM_UART_MR_OFFSET);
      regval &= ~(UART_MR_CPOL | UART_MR_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=1 */
          regval |= UART_MR_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=1 */
          regval |= UART_MR_CPOL | UART_MR_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=0 */
          regval |= UART_MR_CPOL;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      serial_putreg(priv, SAM_UART_MR_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: serial_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void serial_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  /*  Only 8 bit transfer is supported. */

  spiinfo("Only 8 bit transfer is supported on USART SPI.\n");
}

/****************************************************************************
 * Name: serial_spi_status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

static uint8_t serial_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: serial_spi_send
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

static uint32_t serial_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint8_t txbyte;
  uint8_t rxbyte;

  txbyte = (uint8_t)wd;
  rxbyte = (uint8_t)0;
  serial_spi_exchange(dev, &txbyte, &rxbyte, 1);

  spierr("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint32_t)rxbyte;
}

/****************************************************************************
 * Name: serial_spi_exchange
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

static void serial_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                                void *rxbuffer, size_t nwords)
{
  struct sam_spidev_s *spi = (struct sam_spidev_s *)dev;
  struct sam_serial_spi_s *priv = (struct sam_serial_spi_s *)spi->priv;
  uint32_t data;
  uint32_t status;
  uint8_t *rxptr;
  uint8_t *txptr;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Set up working pointers */

  rxptr  = (uint8_t *)rxbuffer;
  txptr  = (uint8_t *)txbuffer;

  /* Make sure that any previous transfer is flushed from the hardware */

  serial_flush(priv);

  for (; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source). */

      if (txptr)
        {
          data = (uint32_t)*txptr++;
        }
      else
        {
          data = 0xffff;
        }

      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */

      status = serial_getreg(priv, SAM_UART_SR_OFFSET);
      while ((status & UART_INT_TXRDY) == 0)
        {
          nxsig_usleep(100);
          status = serial_getreg(priv, SAM_UART_SR_OFFSET);
        }

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      serial_putreg(priv, SAM_UART_THR_OFFSET, data);

      /* Wait for the read data to be available in the RDR. */

      status = serial_getreg(priv, SAM_UART_SR_OFFSET);
      while ((status & UART_INT_RXRDY) == 0)
        {
          nxsig_usleep(100);
          status = serial_getreg(priv, SAM_UART_SR_OFFSET);
        }

      /* Read the received data from the SPI Data Register. */

      data = serial_getreg(priv, SAM_UART_RHR_OFFSET);
      if (rxptr)
        {
          *rxptr++ = (uint8_t)data;
        }
    }
}

/****************************************************************************
 * Name: serial_spi_sndblock
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
static void serial_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                                size_t nwords)
{
  /* spi_exchange can do this. */

  serial_spi_exchange(dev, buffer, NULL, nwords);
}

/****************************************************************************
 * Name: serial_spi_recvblock
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

static void serial_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                                 size_t nwords)
{
  /* spi_exchange can do this. */

  serial_spi_exchange(dev, NULL, buffer, nwords);
}
#endif  /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serial_spi_initialize
 *
 * Description:
 *   Initialize the selected SPI port in master mode
 *
 * Input Parameters:
 *   port - USART interface to be used (0-2)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *sam_serial_spi_initialize(int port)
{
  struct sam_serial_spi_s *priv;
  struct sam_spidev_s *spi;
  irqstate_t flags;
  uint32_t regval;

  spiinfo("Initializing USART%d as SPI\n", port);
  DEBUGASSERT(port >= 0 && port < SAMV7_NUSART);

  switch (port)
    {
#ifdef CONFIG_SAMV7_USART0_SPI_MASTER
      case 0:
        sam_usart0_enableclk();
        priv = &sam_serial0spi_priv;
        break;
#endif
#ifdef CONFIG_SAMV7_USART1_SPI_MASTER
      case 1:
        sam_usart1_enableclk();
        priv = &sam_serial1spi_priv;
        break;
#endif
#ifdef CONFIG_SAMV7_USART2_SPI_MASTER
      case 2:
        sam_usart2_enableclk();
        priv = &sam_serial2spi_priv;
        break;
#endif
      default:
        spierr("ERROR: Incorrect port number %d\n", port);
        return NULL;
    }

  spi = kmm_zalloc(sizeof(struct sam_spidev_s));
  if (!spi)
    {
      spierr("ERROR: Could not allocate sam_spics_s structure!\n");
      return NULL;
    }

  /* Select the SPI operations */

  spi->ops  = &g_spiops;
  spi->priv = priv;

  if (!priv->initialized)
    {
      flags = enter_critical_section();

      sam_configgpio(priv->pins.mosi);
      sam_configgpio(priv->pins.miso);
      sam_configgpio(priv->pins.sck);
      sam_configgpio(priv->pins.nss);

      /* Disable write protection */

      serial_putreg(priv, SAM_UART_WPMR_OFFSET, USART_WPMR_WPKEY);

      serial_putreg(priv, SAM_UART_MR_OFFSET, 0);
      serial_putreg(priv, SAM_UART_RTOR_OFFSET, 0);
      serial_putreg(priv, SAM_UART_TTGR_OFFSET, 0);

      /* Reset and disable receiver and transmitter */

      serial_putreg(priv, SAM_UART_CR_OFFSET,
                    (UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS |
                    UART_CR_TXDIS));

      /* Reset status bits */

      serial_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RSTSTA);

      leave_critical_section(flags);

      /* Configure mode register. */

      regval = UART_MR_MODE_SPIMSTR | UART_MR_CLKO | UART_MR_CHRL_8BITS;
      serial_putreg(priv, SAM_UART_MR_OFFSET, regval);

      /* Enable receiver & transmitter */

      serial_putreg(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN | UART_CR_TXEN));

      spi->priv->mode       = 0;
      spi->priv->nbits      = 8;
      spi->priv->frequency  = 0;
      spi->priv->actual     = 0;

      priv->initialized = true;
    }

  return (struct spi_dev_s *)spi;
}

#endif /* CONFIG_SAMV7_USART_IS_SPI_MASTER */
