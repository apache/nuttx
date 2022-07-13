/****************************************************************************
 * arch/z16/src/z16f/z16f_espi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "chip.h"
#include "z16_internal.h"

#ifdef CONFIG_Z16F_ESPI

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The overall state of one SPI controller */

struct z16f_spi_s
{
  struct spi_dev_s spi;        /* Externally visible part of the SPI interface */
  bool initialized;            /* TRUE: Controller has been initialized */
  uint8_t nbits;               /* Width of word in bits (1-8) */
  uint8_t mode;                /* Mode 0,1,2,3 */
  sem_t exclsem;               /* Assures mutually exclusive access to SPI */
  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */

  /* Debug stuff */

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
  bool wr;                    /* Last was a write */
  uint16_t regval;            /* Last value */
  int ntimes;                 /* Number of times */
  uintptr_t regaddr;          /* Last address */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
static bool     spi_checkreg(FAR struct z16f_spi_s *priv, bool wr,
                  uint16_t regval, uintptr_t regaddr);
static uint8_t  spi_getreg8(FAR struct z16f_spi_s *priv, uintptr_t regaddr);
static void     spi_putreg8(FAR struct z16f_spi_s *priv, uint8_t regval,
                  uintptr_t regaddr);
static void     spi_putreg16(FAR struct z16f_spi_s *priv, uint16_t regval,
                  uintptr_t regaddr);
#else
# define        spi_getreg8(priv,regaddr)         getreg8(regaddr)
# define        spi_putreg8(priv,regval,regaddr)  putreg8(regval, regaddr)
# define        spi_putreg16(priv,regval,regaddr) putreg16(regval, regaddr)
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
static void     spi_dumpregs(FAR struct z16f_spi_s *priv, const char *msg);
#else
# define        spi_dumpregs(priv,msg)
#endif

static void     spi_flush(FAR struct z16f_spi_s *priv);

/* SPI methods */

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SEPI driver operations */

static const struct spi_ops_s g_epsiops =
{
  spi_lock,
  z16f_espi_select,
  spi_setfrequency,
  spi_setmode,
  spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  NULL, /* hwfeatures:  Not supported */
#endif
  z16f_espi_status,
#ifdef CONFIG_SPI_CMDDATA
  z16f_espi_cmddata,
#endif
  spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  spi_exchange,
#else
  spi_sndblock,
  spi_recvblock,
#endif
  NULL  /* registercallback: Not implemented */
};

/* ESPI driver state */

static struct z16f_spi_s g_espi;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   wr      - true:write false:read
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
static bool spi_checkreg(FAR struct z16f_spi_s *priv, bool wr,
                         uint16_t regval, uintptr_t regaddr)
{
  if (wr      == priv->wr &&      /* Same kind of access? */
      regval  == priv->regval &&  /* Same value? */
      regaddr == priv->regaddr)   /* Same address? */
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

          syslog(LOG_INFO, "...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wr      = wr;
      priv->regval  = regval;
      priv->regaddr = regaddr;
      priv->ntimes  = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg8
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
static uint8_t spi_getreg8(FAR struct z16f_spi_s *priv, uintptr_t regaddr)
{
  uint8_t regval = getreg8(regaddr);

  if (spi_checkreg(priv, false, (uint16_t)regval, regaddr))
    {
      syslog(LOG_INFO, "%06x->%02x\n", regaddr, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
static void spi_putreg8(FAR struct z16f_spi_s *priv, uint8_t regval,
                        uintptr_t regaddr)
{
  if (spi_checkreg(priv, true, (uint16_t)regval, regaddr))
    {
      syslog(LOG_INFO, "%06x<-%02x\n", regaddr, regval);
    }

  putreg8(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: spi_putreg16
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

#ifdef CONFIG_Z16F_ESPI_REGDEBUG
static void spi_putreg16(FAR struct z16f_spi_s *priv, uint16_t regval,
                         uintptr_t regaddr)
{
  if (spi_checkreg(priv, true, regval, regaddr))
    {
      syslog(LOG_INFO, "%06x<-%04x\n", regaddr, regval);
    }

  putreg8(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: spi_dumpregs
 *
 * Description:
 *   Dump the contents of all SPI registers
 *
 * Input Parameters:
 *   priv - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(FAR struct z16f_spi_s *priv, FAR const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("   DCR: %02x  CTL: %02x MODE: %02x STAT: %02x\n",
          getreg8(Z16F_ESPI_DCR),  getreg8(Z16F_ESPI_CTL),
          getreg8(Z16F_ESPI_MODE), getreg8(Z16F_ESPI_STAT));
  spiinfo(" STATE: %02x   BR: %02x %02x\n",
          getreg8(Z16F_ESPI_STATE),  getreg8(Z16F_ESPI_BRH),
          getreg8(Z16F_ESPI_BRL));
}
#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Make sure that there are now dangling SPI transfer in progress
 *
 * Input Parameters:
 *   priv - SPI controller state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_flush(FAR struct z16f_spi_s *priv)
{
  /* Make sure the no transfer is in progress... waiting if necessary */

  while ((spi_getreg8(priv, Z16F_ESPI_STAT) & Z16F_ESPI_STAT_TFST) != 0);

  /* Then make sure that there is no pending RX data .. reading as
   * discarding as necessary.
   */

  while ((spi_getreg8(priv, Z16F_ESPI_STAT) & Z16F_ESPI_STAT_RDRF) != 0)
    {
      spi_getreg8(priv, Z16F_ESPI_DATA);
    }
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
 *   lock - true: Lock SPI bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct z16f_spi_s *priv = (FAR struct z16f_spi_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
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
  FAR struct z16f_spi_s *priv = (FAR struct z16f_spi_s *)dev;
  uint32_t actual;
  uint32_t brg;

  spiinfo("frequency=%d\n", frequency);

  /* Check if the requested frequency is the same as the frequency
   * selection.
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Fbaud = Fsystem / (2 * BRG)
   * BRG   = Fsystem / (2 * Fbaud)
   *
   * Example, Fsystem = 18.432MHz, Fbaud = 9600
   * BRG   = 960
   */

  brg = (BOARD_SYSTEM_FREQUENCY >> 1) / frequency;
  if (brg > 0xffff)
    {
      brg = 0xffff;
    }

  /* Save the new BRG setting */

  spi_putreg16(priv, (uint16_t)brg, Z16F_ESPI_BR);

  /* Calculate the new actual frequency */

  actual = (BOARD_SYSTEM_FREQUENCY >> 1) / brg;
  spiinfo("BR=%04x actual=%ld\n", (unsigned int)brg, (long)actual);

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %d->%d\n", frequency, actual);
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
  FAR struct z16f_spi_s *priv = (FAR struct z16f_spi_s *)dev;
  uint8_t regval;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the mode appropriately:
       *
       * SPI  CPOL CPHA
       * MODE
       *  0    0    0
       *  1    0    1
       *  2    1    0
       *  3    1    1
       */

      regval  = spi_getreg8(priv, Z16F_ESPI_CTL);
      regval &= ~(Z16F_ESPI_CTL_PHASE | Z16F_ESPI_CTL_CLKPOL);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; NCPHA=1 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; NCPHA=0 */
          regval |= Z16F_ESPI_CTL_PHASE;
          break;

        case SPIDEV_MODE2: /* CPOL=1; NCPHA=1 */
          regval |= Z16F_ESPI_CTL_CLKPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; NCPHA=0 */
          regval |= (Z16F_ESPI_CTL_PHASE | Z16F_ESPI_CTL_CLKPOL);
          break;

        default:
          DEBUGPANIC();
          return;
        }

      spi_putreg8(priv, regval, Z16F_ESPI_CTL);
      spiinfo("ESPI CTL: %02x\n", regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
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

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct z16f_spi_s *priv = (FAR struct z16f_spi_s *)dev;
  uint8_t regval;

  spiinfo("nbits=%d\n", nbits);
  DEBUGASSERT(priv && nbits > 0 && nbits <= 8);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set number of bits appropriately */

      regval  = spi_getreg8(priv, Z16F_ESPI_MODE);
      regval &= ~Z16F_ESPI_MODE_NUMBITS_MASK;

      /* The register value of zero is 8-bit */

      if (nbits < 8)
        {
          regval |= ((uint8_t)nbits << Z16F_ESPI_MODE_NUMBITS_SHIFT);
        }

      spi_putreg8(priv, regval, Z16F_ESPI_MODE);
      spiinfo("ESPI MODE: %02x\n", regval);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
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

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  uint8_t txbyte;
  uint8_t rxbyte;

  /* spi_exchange can do this. Note: right now, this only deals with 8-bit
   * words.  If the SPI interface were configured for words of other sizes,
   * this would fail.
   */

  txbyte = (uint8_t)wd;
  rxbyte = (uint8_t)0;
  spi_exchange(dev, &txbyte, &rxbyte, 1);

  spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint32_t)rxbyte;
}

/****************************************************************************
 * Name: spi_exchange
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

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct z16f_spi_s *priv = (struct z16f_spi_s *)dev;
  uint8_t data;
  FAR uint8_t *rxptr = rxbuffer;
  FAR const uint8_t *txptr = txbuffer;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(priv);

  /* Make sure the TEOF bit is not set (SSV must also be zero) */

  spi_putreg8(priv, 0, Z16F_ESPI_CTL);

  /* Loop, sending each word in the user-provided data buffer.
   *
   * Note 2: This loop might be made more efficient.  Would logic
   *         like the following improve the throughput?  Or would it
   *         just add the risk of overruns?
   *
   *   Get byte 1;
   *   Send byte 1;  Now word 1 is "in flight"
   *   nwords--;
   *   for ( ; byte > 0; byte--)
   *     {
   *       Get byte N.
   *       Wait for TDRE meaning that byte N-1 has moved to the shift
   *          register.
   *       Disable interrupts to keep the following atomic
   *       Send byte N.  Now both work N-1 and N are "in flight"
   *       Wait for RDRF meaning that byte N-1 is available
   *       Read byte N-1.
   *       Re-enable interrupts.
   *       Save byte N-1.
   *     }
   *   Wait for RDRF meaning that the final byte is available
   *   Read the final byte.
   *   Save the final byte.
   */

  for (; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source). */

      if (txptr)
        {
          data = *txptr++;
        }
      else
        {
          data = 0xff;
        }

      /* Do we need to set the TEOF bit in the CTL register too? */

      if (nwords == 1)
        {
          spi_putreg8(priv, Z16F_ESPI_DCR_TEOF, Z16F_ESPI_CTL);
        }

      /* Wait for any transmit data register to be empty. */

      while ((spi_getreg8(priv, Z16F_ESPI_STAT) & Z16F_ESPI_STAT_TDRE) == 0);

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      spi_putreg8(priv, data, Z16F_ESPI_DATA);

      /* Wait for the read data to be available in the data register. */

      while ((spi_getreg8(priv, Z16F_ESPI_STAT) & Z16F_ESPI_STAT_RDRF) == 0);

      /* Read the received data from the SPI Data Register. */

      data = spi_getreg8(priv, Z16F_ESPI_DATA);
      if (rxptr)
        {
          *rxptr++ = (uint8_t)data;
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   port - Identifies the "logical" SPI port.  Must be zero in this case.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *z16_spibus_initialize(int port)
{
  FAR struct z16f_spi_s *priv;
  irqstate_t flags;
  uint8_t regval;

  spiinfo("port: %d\n", port);
  DEBUGASSERT(port == 0);

  /* Check if we have already initialized the ESPI */

  priv = &g_espi;
  if (priv->initialized)
    {
      /* Initialize the ESPI state structure */

      flags = enter_critical_section();
      priv->spi.ops = &g_epsiops;
      nxsem_init(&priv->exclsem, 0, 1);

      /* Set up the SPI pin configuration (board-specific logic is required
       * to configure and manage all chip selects).
       *
       *   SCK  - PC3, Alternate function 1
       *   MOSI - PC4, Alternate function 1
       *   MISO - PC5, Alternate function 1
       */

      regval  = spi_getreg8(priv, Z16F_GPIOC_AFL);
      regval |= 0x38;
      spi_putreg8(priv, regval, Z16F_GPIOC_AFL);

      regval  = spi_getreg8(priv, Z16F_GPIOC_AFH);
      regval &= ~0x38;
      spi_putreg8(priv, regval, Z16F_GPIOC_AFH);

      /* Initialize the ESPI peripheral.  Master, Mode 0, 8-bits, 400KHz */

      spi_putreg8(priv, 0x00, Z16F_ESPI_CTL);    /* Disabled the ESPI */
      spi_putreg8(priv, 0x00, Z16F_ESPI_DCR);    /* Disabled slave select; clear TEOF */

      regval = Z16F_ESPI_MODE_SSIO | Z16F_ESPI_MODE_NUMBITS_8BITS |
               Z16F_ESPI_MODE_SSMD_SPI;
      spi_putreg8(priv, regval, Z16F_ESPI_MODE); /* SPI mode, 8-bit */

      regval = Z16F_ESPI_CTL_ESPIEN0 | Z16F_ESPI_CTL_MMEN |
               Z16F_ESPI_CTL_ESPIEN1;
      spi_putreg8(priv, 0x00, Z16F_ESPI_CTL);    /* TX/RX mode, Master mode */

      /* Make sure that we are all in agreement about the configuration and
       * set the BRG for 400KHz operation.
       */

      spi_setfrequency(&priv->spi, 400000);
      spi_setmode(&priv->spi, SPIDEV_MODE0);
      spi_setbits(&priv->spi, 8);

      /* Now we are initialized */

      priv->initialized = true;
      leave_critical_section(flags);
    }

  spi_dumpregs(priv, "After initialization");
  return &priv->spi;
}

#endif /* CONFIG_Z16F_ESPI */
