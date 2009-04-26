/****************************************************************************
 * arch/arm/src/imx/imx_spi.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <arch/io.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "imx_cspi.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The i.MX1/L supports 2 SPI interfaces.  Which have been endabled? */

#ifndef CONFIG_SPI1_DISABLE
#  define SPI1_NDX 0           /* Index to SPI1 in g_spidev[] */
#  ifndef CONFIG_SPI2_DISABLE
#   define SPI2_NDX 1          /* Index to SPI2 in g_spidev[] */
#   define NSPIS 2             /* Two SPI interfaces: SPI1 & SPI2 */
#  else
#   define NSPIS 1             /* One SPI interface: SPI1 */
#  endif
#else
#  ifndef CONFIG_SPI2_DISABLE
#   define SPI2_NDX 0          /* Index to SPI2 in g_spidev[] */
#   define NSPIS 1             /* One SPI interface: SPI2 */
#  else
#   define NSPIS 0             /* No SPI interfaces */
#  endif
#endif

/* Compile the rest of the file only if at least one SPI interface has been
 * enabled.
 */

#if NSPIS > 0

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct imx_spidev_s
{
  const struct spi_ops_s *ops;  /* Common SPI operations */
  uint32 base;                  /* SPI register base address */
  uint32 frequency;             /* Current desired SCLK frequency */
  uint32 actual;                /* Current actual SCLK frequency */
  ubyte  mode;                  /* Current mode */
  ubyte  nbytes;                /* Current number of bits per word */
  ubyte  irq;                   /* SPI IRQ number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* SPI helpers */
 
static inline uint32 spi_getreg(struct imx_spidev_s *priv, unsigned int offset);
static inline void spi_putreg(struct imx_spidev_s *priv, unsigned int offset, uint32 value);
static ubyte  spi_waitspif(struct imx_spidev_s *priv);
static ubyte  spi_transfer(struct imx_spidev_s *priv, ubyte ch);

/* SPI methods */

static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency);
static void   spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static ubyte  spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch);
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SPI operations */

static const struct spi_ops_s g_spiops =
{
  imx_spiselect,    /* Provided externally by board logic */
  spi_setfrequency,
  spi_setmode,
  imx_spistatus,    /* Provided externally by board logic */
  spi_sndbyte,
  spi_sndblock,
  spi_recvblock,
};

/* This supports is up to two SPI busses/ports */

static struct imx_spidev_s g_spidev[] =
{
#ifndef CONFIG_SPI1_DISABLE
  {
    .ops  = &g_spiops,
    .base = IMX_CSPI1_VBASE
    .irq  = IMX_IRQ_CSPI1,
  },
#endif
#ifndef CONFIG_SPI1_DISABLE
  {
    .ops  = &g_spiops,
    .base = IMX_CSPI2_VBASE
    .irq  = IMX_IRQ_CSPI2,
  },
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

static inline uint32 spi_getreg(struct imx_spidev_s *priv, unsigned int offset)
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

static inline void spi_putreg(struct imx_spidev_s *priv, unsigned int offset, uint32 value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: spi_waitspif
 *
 * Description:
 *   Wait space available in the Tx FIFO.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   Status register mode bits
 *
 ****************************************************************************/

static uint32 spi_waitspif(struct imx_spidev_s *priv)
{
  uint32 status;

  /* Wait for the device to be ready to accept another byte (or for an error
   * to be reported).
   */
#error "Missing logic"
  return status;
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Send one byte on SPI, return the response
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   ch - the byte to send
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static ubyte spi_transfer(struct imx_spidev_s *priv, ubyte ch)
{
   ubyte status;

  /* Send the byte, repeating if some error occurs */

  for(;;)
    {
#error "Missing logic"

      /* Wait for the device to be ready to accept another byte */

      status = spi_waitspif(oriv);

      /* Return the next byte from the Rx FIFO */
#error "Missing logic"
    }
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

static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint32 actual = priv->actual;

  if (priv & frequency != priv->frequency)
    {
      uint32 freqbits;
      uint32 regval;

      if (frequency >= PERCLK2 / 4)
        {
          freqbits = CSPI_CTRL_DIV4;
          actual   = PERCLK2 / 4;
        }
      else if (frequency >= PERCLK2 / 8)
        {
          freqbits = CSPI_CTRL_DIV8;
          actual   = PERCLK2 / 8;
        }
      else if (frequency >= PERCLK2 / 16)
        {
          freqbits = CSPI_CTRL_DIV16;
          actual   = PERCLK2 / 16;
        }
      else if (frequency >= PERCLK2 / 32)
        {
          freqbits = CSPI_CTRL_DIV32;
          actual   = PERCLK2 / 32;
        }
      else if (frequency >= PERCLK2 / 64)
        {
          freqbits = CSPI_CTRL_DIV64;
          actual   = PERCLK2 / 64;
        }
      else if (frequency >= PERCLK2 / 128)
        {
          freqbits = CSPI_CTRL_DIV128;
          actual   = PERCLK2 / 128;
        }
      else if (frequency >= PERCLK2 / 256)
        {
          freqbits = CSPI_CTRL_DIV256;
          actual   = PERCLK2 / 256;
        }
      else /*if (frequency >= PERCLK2 / 512) */
        {
          freqbits = CSPI_CTRL_DIV512;
          actual   = PERCLK2 / 512;
        }

      /* Then set the selected frequency */

      regval = spi_regreg(priv, CSPI_CTRL_OFFSET);
      regval &= ~(CSPI_CTRL_DATARATE_MASK);
      regval |= freqbits;
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);

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

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  if (priv & mode != priv->mode)
    {
      uint32 modebits;
      uint32 regval;

      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0 CHPHA=0 */
          modebits = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0 CHPHA=1 */
          modebits = CSPI_CTRL_PHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1 CHPHA=0 */
          modebits = CSPI_CTRL_POL;
         break;

        case SPIDEV_MODE3: /* CPOL=1 CHPHA=1 */
          modebits = CSPI_CTRL_PHA|CSPI_CTRL_POL;
          break;

        default:
          return;
        }

      /* Then set the selected mode */

      regval = spi_regreg(priv, CSPI_CTRL_OFFSET);
      regval &= ~(CSPI_CTRL_PHA|CSPI_CTRL_POL);
      regval |= modebits;
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: spi_sndbyte
 *
 * Description:
 *   Send one byte on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   ch -  The byte to send
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static ubyte spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  return spi_transfer(priv, ch);
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint32 response;

  /* Loop while thre are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      response = spi_transfer(priv, *buffer++);
    }
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   buflen - the length of data that can be received in the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;

  /* Loop while thre are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      *buffer = (ubyte)spi_transfer(prive, 0xff);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.  However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Theregore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  struct imx_spidev_s *priv;
  ubyte regval;

  /* Only the SPI1 interface is supported */

  switch (port)
    {
#ifndef CONFIG_SPI1_DISABLE
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
#endif

#ifndef CONFIG_SPI2_DISABLE
    case 2:
      /* Select SPI1 */

      priv = &g_spidev[SPI2_NDX];

      /* Configure SPI1 GPIOs */
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
#endif

    default:
      return NULL;
    }

  /* Disable SPI */
#error "Missing logic"

  /* Initialize control rebistger: min frequency, ignore ready, master mode, mode=0, 8-bit */

  spi_putreg(priv, IMX_CSPI_CTRL_OFFSET, 
             CSPI_CTRL_DIV512 |                /* Lowest frequency */
             CSPI_CTRL_DRCTL_IGNRDY |          /* Ignore ready */
             CSPI_CTRL_MODE |                  /* Master mode */
             (7 << CSPI_CTRL_BITCOUNT_SHIFT)); /* 8-bit data */

  /* Make sure state agrees with data */

  priv->mode  = SPIDEV_MODE0;
  priv->nbits = 8;

  /* Set the initial clock frequency for identification mode < 400kHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Enable interrupts on data ready (and certain error conditions */

  spi_putreg(priv, CSPI_INTCS_OFFSET,
             CSPI_INTCS_RREN |                 /* RXFIFO Data Ready Interrupt Enable */
             CSPI_INTCS_ROEN |                 /* RXFIFO Overflow Interrupt Enable */
             CSPI_INTCS_BOEN);                 /* Bit Count Overflow Interrupt Enable */

  /* Set the clock source=bit clock and number of clocks inserted between
   * transactions = 2.
   */

  spi_putreg(priv, CSPI_SPCR_OFFSET, 2);

  /* No DMA */

  spi_putreg(priv, CSPI_DMA_OFFSET, 0);

  /* Attach the interrupt */

  irq_attach(priv->irq, (xcpt_t)imx_spinterrupt);

  /* Enable SPI */

  regval = spi_getreg(priv, IMX_CSPI_CTRL_OFFSET);
  regval |= CSPI_CTRL_SPIEN;
  spi_putreg(priv, IMX_CSPI_CTRL_OFFSET, regval);

  /* Enable SPI interrupts */

  up_enable_irq(priv->irq);
  return (FAR struct spi_dev_s *)priv;
}

#endife /* NSPIS > 0 */