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
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* SPI helpers */
 
static inline uint32 spi_getreg(struct imx_spidev_s *priv, unsigned int offset);
static inline void spi_readreg(struct imx_spidev_s *priv, unsigned int offset, uint32 value);
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
  },
#endif
#ifndef CONFIG_SPI1_DISABLE
  {
    .ops  = &g_spiops,
    .base = IMX_CSPI2_VBASE
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

static inline void spi_readreg(struct imx_spidev_s *priv, unsigned int offset, uint32 value)
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
#error "Missing logic"
  return frequency;
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
  uint32 modebits;
  ubyte regval;

  /* Select the CTL register bits based on the selected mode */

  switch (mode)
    {
    case SPIDEV_MODE0: /* CPOL=0 CHPHA=0 */
#error "Missing logic"
      break;

    case SPIDEV_MODE1: /* CPOL=0 CHPHA=1 */
#error "Missing logic"
      break;

    case SPIDEV_MODE2: /* CPOL=1 CHPHA=0 */
#error "Missing logic"
      break;

    case SPIDEV_MODE3: /* CPOL=1 CHPHA=1 */
#error "Missing logic"
      break;

    default:
      return;
    }

    /* Then set the selected mode */
#error "Missing logic"
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

      /* Configure SPI1 GPIOs */
#error "Missing logic"
      break;
#endif

#ifndef CONFIG_SPI2_DISABLE
    case 2:
      /* Select SPI1 */

      priv = &g_spidev[SPI2_NDX];

      /* Configure SPI1 GPIOs */
#error "Missing logic"
      break;
#endif

    default:
      return NULL;
    }

  /* Disable SPI */
#error "Missing logic"

  /* Set the initial clock frequency for indentification mode < 400kHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Enable the SPI.
   * NOTE 1: Interrupts are not used in this driver version.
   * NOTE 2: Initial mode is mode=0.
   */

#error "Missing logic"
  return (FAR struct spi_dev_s *)priv;
}

#endife /* NSPIS > 0 */