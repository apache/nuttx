/****************************************************************************
 * drivers/spi/spi_bitbang.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>

#include <nuttx/mutex.h>

#ifdef CONFIG_SPI_BITBANG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This file holds the static, device-independent portion of the generic
 * SPI-bit-bang driver.  The full driver consists of 5 files:
 *
 * 1. drivers/spi/spi_bitbang.c:  This file.  This file holds the basic
 *    SPI driver framework and not perform any direct bit-bang operations.
 *    Rather, it will could out to board-specific logic to perform the
 *    low level data transfers.
 * 2. include/nuttx/spi/spi_bitbang.h:  This header file provides the
 *    data types and function prototypes needed to utilize the logic in
 *    this file.
 * 3. boards/<arch>/<chip>/<board>/src/<file>:  The implementation of the
 *    low-level bit-bang logic resides in some file in the board source
 *    directory.  This board-specific logic includes the bit-bang skeleton
 *    logic provided in include/nuttx/spi/spi_bitbang.c.
 * 4. include/nuttx/spi/spi_bitbang.c.  Despite the .c extension, this is
 *    really an included file.  It is used in this way:  1) The board-
 *    specific logic in boards/<arch>/<chip>/<board>/src/<file> provides
 *    some definitions then 2) includes include/nuttx/spi/spi_bitbang.c.
 *    That file will then use those definitions to implement the low-level
 *    bit-bang logic.  The board-specific logic then calls
 *    spi_create_bitbang() in this file to instantiate the complete SPI
 *    driver.
 *
 *    See include/nuttx/spi/spi_bitbang.c for more detailed usage
 *    information.
 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void     spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                  bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                  uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                  enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(FAR struct spi_dev_s *dev,
                   FAR const void *txbuffer, FAR void *rxbuffer,
                   size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                  FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                  size_t nwords);
#endif
static uint8_t  spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int      spi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                  bool cmd);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI driver operations */

static const struct spi_ops_s g_spiops =
{
  spi_lock,           /* lock */
  spi_select,         /* select */
  spi_setfrequency,   /* setfrequency */
  spi_setmode,        /* setmode */
  spi_setbits,        /* setbits */
#ifdef CONFIG_SPI_HWFEATURES
  0,                  /* hwfeatures */
#endif
  spi_status,         /* status */
#ifdef CONFIG_SPI_CMDDATA
  spi_cmddata,        /* cmddata */
#endif
  spi_send,           /* send */
#ifdef CONFIG_SPI_EXCHANGE
  spi_exchange,       /* exchange */
#else
  spi_sndblock,       /* sndblock */
  spi_recvblock,      /* recvblock */
#endif
  0                   /* registercallback */
};

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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
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
 *   Set/clear the chip select line for the selected device.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   devid    - Identifies the device to be selected
 *   selected - select or de-select device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  spiinfo("devid=%" PRIu32 " selected=%d\n", devid, selected);
  DEBUGASSERT(priv && priv->low->select);
  priv->low->select(priv, devid, selected);
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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  uint32_t actual;

  DEBUGASSERT(priv && priv->low->setfrequency);
  actual = priv->low->setfrequency(priv, frequency);
  spiinfo("frequency=%" PRIu32 " holdtime=%" PRIu32 " actual=%" PRIu32 "\n",
          frequency, priv->holdtime, actual);
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

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  DEBUGASSERT(priv && priv->low->setmode);
  priv->low->setmode(priv, mode);
  spiinfo("mode=%d exchange=%p\n", mode, priv->exchange);
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  spiinfo("nbits=%d\n", nbits);
  DEBUGASSERT(priv && nbits > 0 && nbits <= 16);
  priv->nbits = nbits;
#else
  spiinfo("nbits=%d\n", nbits);
  DEBUGASSERT(nbits == 8);
#endif
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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERT(priv && priv->low && priv->low->exchange);

  return priv->low->exchange(priv, (uint16_t)wd);
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
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

static void spi_exchange(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer, FAR void *rxbuffer,
                         size_t nwords)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  FAR const uint8_t *src = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *dest = (FAR uint8_t *)rxbuffer;
  uint16_t dataout;
  uint16_t datain;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
  DEBUGASSERT(priv && priv->low && priv->low->exchange);

  /* If there is no data source, send 0xff */

  if (!src)
    {
      dataout = 0xff;
    }

  /* Exchange each word */

  while (nwords-- > 0)
    {
      /* If there is source data, get the next word from the source */

      if (src)
        {
          dataout = (uint16_t)*src++;

#ifdef CONFIG_SPI_BITBANG_VARWIDTH
          if (priv->nbits > 8)
            {
#ifdef CONFIG_ENDIAN_BIG
              dataout <<= 8;
              dataout |= *src++;
#else
              dataout |= (uint16_t)(*src++) << 8;
#endif
            }
#endif
        }

      /* Exchange the word of data */

      datain = priv->low->exchange(priv, dataout);

      /* If there is a data sink, transfer the data to the receive buffer */

      if (dest)
        {
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
          if (priv->nbits > 8)
            {
#ifdef CONFIG_ENDIAN_BIG
             *dest++ = (uint8_t)(datain >> 8);
             *dest++ = (uint8_t)datain;
#else
             *dest++ = (uint8_t)datain;
             *dest++ = (uint8_t)(datain >> 8);
#endif
            }
#else
          *dest++ = (uint8_t)datain;
#endif
        }
    }
}

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
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, buffer, NULL, nwords);
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
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get status bits associated with the device associated with 'devid'
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device of interest
 *
 * Returned Value:
 *   Bit encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERT(priv && priv->low && priv->low->status);

  return priv->low->status(priv, devid);
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Control the SPI CMD/DATA like for the device associated with 'devid'
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device of interest
 *   cmd   - True:CMD False:DATA
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERT(priv && priv->low && priv->low->status);

  return priv->low->cmddata(priv, devid, cmd);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  spi_create_bitbang
 *
 * Description:
 *   Create an instance of the SPI bit-bang driver.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_create_bitbang(FAR const struct
                                         spi_bitbang_ops_s *low)
{
  FAR struct spi_bitbang_s *priv;

  DEBUGASSERT(low);

  /* Allocate an instance of the SPI bit bang structure */

  priv = (FAR struct spi_bitbang_s *)
    kmm_zalloc(sizeof(struct spi_bitbang_s));
  if (!priv)
    {
      spierr("ERROR: Failed to allocate the device structure\n");
      return NULL;
    }

  /* Initialize the driver structure */

  priv->dev.ops = &g_spiops;
  priv->low     = low;
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
  priv->nbits   = 8;
#endif

  nxmutex_init(&priv->lock);

  /* Select an initial state of mode 0, 8-bits, and 400KHz */

  low->setmode(priv, SPIDEV_MODE0);
  low->setfrequency(priv, 400000);

  /* And return the initialized driver structure */

  return &priv->dev;
}

#endif /* CONFIG_SPI_BITBANG */
