/****************************************************************************
 * config/olimex-strp711/src/up_spi.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This logic emulates the Prolific PL2303 serial/USB converter
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

#warning "The following is incorrect"
/****************************************************************************
 * One the Olimex lpc214x board, the MMC slot is connect via SPI with the
 * following SPI mode pinout:
 *
 *   1 CS: Chip select (low) - SSEL1      5 SCLK: Clock         - SCK1
 *   2 DI: Data input        - MOSI1      6 Vss2: Supply Voltage- GRND
 *   3 Vss: Supply Voltage   - GRND       7 DO: Data Output     - MISO1
 *   4 Vdd: Power Supply     - Vcc        8 - N/C
 *
 * The LPC214x supports two buffered SPI ports (SBPI0 and BSPI1).  BSPI1
 * is used to interface with the MMC connect
 *
 *   SCK1  - pin 47, P0.17/CAP1.2/SCK1/MAT1.2
 *   MISO1 - pin 53, P0.18/CAP1.3/MISO1/MAT1.3
 *   MOSI1 - pin 54, P0.19/MAT1.2/MOSI1/CAP1.2
 *   SSEL1 - pin 55, P0.20/MAT1.3/SSEL1/EINT3
 *
 * BSPI0 is available on the Olimex board (pins 27, 29, 30, and 31).
 * Pin 27 is dedicated to a chip select, pins 30 and 31 connect to keys, nd
 * pin 29 is unconnected.
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

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void   spi_select(FAR struct spi_dev_s *dev, boolean selected);
static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency);
static ubyte  spi_status(FAR struct spi_dev_s *dev);
static ubyte  spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch);
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
  .sndbyte           = spi_sndbyte,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
};

static struct spi_dev_s g_spidev = { &g_spiops };

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select
 *
 * Input Parameters:
 *   selected: TRUE: chip selected, FALSE: chip de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, boolean selected)
{
#warning "To be provided"
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   frequency: The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency)
{
#warning "To be provided"
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static ubyte spi_status(FAR struct spi_dev_s *dev)
{
#warning "To be provided"
}

/****************************************************************************
 * Name: spi_sndbyte
 *
 * Description:
 *   Send one byte on SPI
 *
 * Input Parameters:
 *   ch - the byte to send
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static ubyte spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch)
{
#warning "To be provided"
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen)
{
#warning "To be provided"
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   buffer - A pointer to the buffer in which to recieve data
 *   buflen - the length of data that can be received in the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen)
{
#warning "To be provided"
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
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
#warning "To be provided"
}
