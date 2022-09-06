/****************************************************************************
 * arch/z80/src/ez80/ez80_spi.c
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

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <arch/io.h>

#include "z80_internal.h"
#include "chip.h"
#include "ez80_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_EZ80F91) || defined(CONFIG_ARCH_CHIP_EZ80F92)

/* PB2/#SS is controlled by board specific logic and is not used by this
 * driver.  This permits supporting multiple, different devices with
 * different chip selects on the bus.
 */

#  define GPIOB_SPI_SS      (1 << 2)  /* PB2: /SS (not used by driver) */
#  define GPIOB_SPI_SCK     (1 << 3)  /* PB3: SCK */
#  define GPIOB_SPI_MISO    (1 << 6)  /* PB6: MISO */
#  define GPIOB_SPI_MOSI    (1 << 7)  /* PB7: MOSI */

#  define GPIOB_SPI_PINSET  (GPIOB_SPI_SCK | GPIOB_SPI_MISO | GPIOB_SPI_MOSI)
#else
#  error "Check GPIO initialization for this chip"
#endif

#define SPIF_RETRIES 1000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int    spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                uint32_t frequency);
static void   spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void   spi_exchange(FAR struct spi_dev_s *dev,
                FAR const void *txbuffer, FAR void *rxbuffer,
                size_t nwords);
#else
static void   spi_sndblock(FAR struct spi_dev_s *dev,
                FAR const uint8_t *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR uint8_t *buffer,
                size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  spi_lock,            /* lock() */
  ez80_spiselect,      /* select(): Provided externally by board logic */
  spi_setfrequency,    /* setfrequency() */
#ifdef CONFIG_SPI_DELAY_CONTROL
  NULL,                /* setdelay() */
#endif
  spi_setmode,
  NULL,                /* setbits() */
#ifdef CONFIG_SPI_HWFEATURES
  NULL,                /* hwfeatures() */
#endif
  ez80_spistatus,      /* status(): Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  ez80_spicmddata,     /* cmddata(): Provided externally by board logic */
#endif
  spi_send,            /* send() */
#ifdef CONFIG_SPI_EXCHANGE
  spi_exchange,        /* exchange() */
#else
  spi_sndblock,        /* sndblock() */
  spi_recvblock,       /* recvblock() */
#endif
#ifdef CONFIG_SPI_TRIGGER
  NULL,                /* trigger() */
#endif
  NULL                 /* registercallback() */
};

/* This supports is only a single SPI bus/port.  If you port this to an
 * architecture with multiple SPI buses/ports, then (1) you must create
 * a structure, say ez80_spidev_s, containing both struct spi_dev_s and
 * the mutual exclusion semaphored, and (2) the following must become an
 * array with one 'struct spi_dev_s' instance per bus.
 */

static struct spi_dev_s g_spidev =
{
  &g_spiops
};

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/* These are used to perform reconfigurations only when necessary. */

static uint32_t g_spi_frequency;
static uint32_t g_spi_actual;
static enum spi_mode_e g_spi_mode = SPIDEV_MODE3;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&g_lock);
    }
  else
    {
      ret = nxmutex_unlock(&g_lock);
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  if (frequency != g_spi_frequency)
    {
     uint32_t brg;

     spiinfo("frequency: %lu\n", (unsigned long)frequency);

      /* We want select divisor to provide the highest frequency (SPIR) that
       * does NOT exceed the requested frequency.:
       *
       *   SPIR <= System Clock Frequency / (2 * BRG)
       *
       * So
       *
       *   BRG >= System Clock Frequency / (2 * SPIR)
       */

      brg = ((EZ80_SYS_CLK_FREQ + 1) / 2 + frequency - 1) / frequency;

      /* "When configured as a Master, the 16-bit divisor value must be
       * between 0003h and FFFFh, inclusive. When configured as a Slave, the
       * 16-bit divisor value must be between 0004h and FFFFh, inclusive."
       */

      if (brg < 3)
        {
          brg = 3;
        }
      else if (brg > 0xffff)
        {
          brg = 0xffff;
        }

      outp(EZ80_SPI_BRG_L, brg & 0xff);
      outp(EZ80_SPI_BRG_H, (brg >> 8) & 0xff);

      g_spi_frequency = frequency;
      g_spi_actual    = ((EZ80_SYS_CLK_FREQ + 1) / 2 + brg - 1) / brg;

      finfo("BRG=%lu Actual=%lu\n",
            (unsigned long)brg, (unsigned long)g_spi_actual);
    }

  return g_spi_actual;
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

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  if (mode != g_spi_mode)
    {
      uint8_t modebits;
      uint8_t regval;

      spiinfo("mode: %d\n", (int)mode);

      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0 CPHA=0 */
            modebits = 0;
            break;

          case SPIDEV_MODE1: /* CPOL=0 CPHA=1 */
            modebits = SPI_CTL_CPHA;
            break;

          case SPIDEV_MODE2: /* CPOL=1 CPHA=0 */
            modebits = SPI_CTL_CPOL;
            break;

          case SPIDEV_MODE3: /* CPOL=1 CPHA=1 */
            modebits = (SPI_CTL_CPOL | SPI_CTL_CPHA);
            break;

          default:
            return;
        }

      /* Then set those bits in the CTL register */

      regval  = inp(EZ80_SPI_CTL);
      regval &= ~(SPI_CTL_CPOL | SPI_CTL_CPHA);
      regval |= modebits;
      outp(EZ80_SPI_CTL, regval);

      g_spi_mode = mode;
    }
}

/****************************************************************************
 * Name: spi_waitspif
 *
 * Description:
 *   Wait for the SPIF bit to be set in the status register signifying the
 *   the data transfer was finished.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK if the transferred completed without error.  Otherwise, a negated
 *   errno value is returned indicating the nature of the error.
 *
 ****************************************************************************/

static int spi_waitspif(void)
{
  uint8_t status;
  int retries;

  /* Wait for the device to be ready to accept another byte (or for an error
   * to be reported or for a timeout to occur).
   */

  for (retries = 0; retries < SPIF_RETRIES; retries++)
    {
      status = inp(EZ80_SPI_SR);
      if ((status & (SPI_SR_WCOL | SPI_SR_MODF)) != 0)
        {
          return -EIO;
        }
      else if ((status & SPI_SR_SPIF) != 0)
        {
          return OK;
        }
    }

  spierr("ERROR: SPI timed out\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Send one byte on SPI, return the response
 *
 * Input Parameters:
 *   chout - The byte to send
 *   chin  - The location to save the returned byte (may be NULL)
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static int spi_transfer(uint8_t chout, FAR uint8_t *chin)
{
  uint8_t response;
  int ret;

  /* Send the byte */

  outp(EZ80_SPI_TSR, chout);

  /* Wait for the device to be ready to accept another byte */

  ret = spi_waitspif();
  if (ret < 0)
    {
      spierr("ERROR: spi_waitspif returned %d\n", ret);
      return ret;
    }

  response = inp(EZ80_SPI_RBR);
  if (chin != NULL)
    {
      *chin = response;
    }

  return OK;
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

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  uint8_t response;
  int ret;

  ret = spi_transfer((uint8_t)wd, &response);
  if (ret < 0)
    {
      spierr("ERROR: spi_transfer returned %d\n", ret);
      return (uint32_t)0xff;
    }

  spiinfo("cmd: %04" PRIx32 " resp: %02x\n", wd, response);
  return (uint32_t)response;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI. Required.
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

#ifdef CONFIG_SPI_EXCHANGE
static void spi_exchange(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer, FAR void *rxbuffer,
                         size_t nwords)
{
  FAR const uint8_t *inptr = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *outptr = (FAR uint8_t *)rxbuffer;

  spiinfo("txbuffer: %p rxbuffer: %p nwords: %lu\n",
          txbuffer, rxbuffer, (unsigned long)nwords);

  /* Loop while there are bytes remaining to be sent */

  while (nwords-- > 0)
    {
      uint8_t outword;
      int ret;

      /* Send 0xff if there is no outgoing TX stream */

      outword = (inptr == NULL) ? 0xff : *inptr++;

      /* Send the outgoing word and obtain the response */

      ret = spi_transfer(outword, outptr);
      if (ret < 0)
        {
          spierr("ERROR: spi_transfer returned %d\n", ret);
          break;
        }

      /* Conditionally increment the output buffer pointer. */

      if (outptr != NULL)
        {
          outptr++;
        }
    }
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
 *   buflen - the length of data to send from the buffer in number of words.
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
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t buflen)
{
  FAR const uint8_t *ptr = (FAR const uint8_t *)buffer;
  int ret;

  spiinfo("buffer: %p buflen: %lu\n", buffer, (unsigned long)nwords);

  /* Loop while there are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      ret = spi_transfer(*ptr++, NULL);
      if (ret < 0)
        {
          spierr("ERROR: spi_transfer returned %d\n", ret);
          break;
        }
    }
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
 *   buflen - The length of data that can be received in the buffer in
 *            number of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t buflen)
{
  FAR uint8_t *ptr = (FAR uint8_t *)buffer;

  spiinfo("buffer: %p buflen: %lu\n", buffer, (unsigned long)nwords);

  /* Loop while there are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      ret = spi_transfer(0xff, ptr++);
      if (ret < 0)
        {
          spierr("ERROR: spi_transfer returned %d\n", ret);
          break;
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.
 *   However, if multiple devices on on the bus, then multiple chip
 *   selects will be required.  Therefore, all GPIO chip management is
 *   deferred to board-specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *ez80_spibus_initialize(int port)
{
  uint8_t regval;

#ifdef CONFIG_DEBUG_FEATURES
  /* Only the SPI1 interface is supported */

  if (port != 1)
    {
      return NULL;
    }
#endif

  /* Disable SPI */

  outp(EZ80_SPI_CTL, 0);

  /* Configure GPIOs.  For the eZ80F91, the pin mapping for the four SPI pins
   * is:
   *
   *  GPIO ALT   MASTER  SLAVE   COMMENT
   *  ---- ----- ------- ------- ---------------------------------
   *  PB2  SS    INPUT   INPUT   Managed by board specific logic
   *  PB3  SCLK  OUTPUT  INPUT
   *  PB6  MISO  INPUT   OUTPUT
   *  PB7  MOSI  OUTPUT  INPUT
   *
   * Select the alternate function for PB2-3,6-7:
   */

#if defined(CONFIG_ARCH_CHIP_EZ80F91) || defined(CONFIG_ARCH_CHIP_EZ80F92)
  regval  = inp(EZ80_PB_DDR);
  regval |= GPIOB_SPI_PINSET;
  outp(EZ80_PB_DDR, regval);

  regval  = inp(EZ80_PB_ALT1);
  regval &= ~GPIOB_SPI_PINSET;
  outp(EZ80_PB_ALT1, regval);

  regval  = inp(EZ80_PB_ALT2);
  regval |= GPIOB_SPI_PINSET;
  outp(EZ80_PB_ALT2, regval);
#else
#  error "Check GPIO initialization for this chip"
#endif

  /* Set the initial clock frequency for identification mode < 400kHz
   * and Mode 0.
   */

  spi_setfrequency(NULL, 400000);
  spi_setmode(NULL, SPIDEV_MODE0);

  /* Enable the SPI.
   * NOTE 1: Interrupts are not used in this driver version.
   * NOTE 2: Initial mode is mode=0
   */

  outp(EZ80_SPI_CTL, SPI_CTL_SPIEN | SPI_CTL_MASTEREN);

  return &g_spidev;
}
