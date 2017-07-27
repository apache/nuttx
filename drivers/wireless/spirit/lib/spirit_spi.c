/******************************************************************************
 * drivers/wireless/spirit/lib//spirit_spi.c
 * NuttX SPIRIT SPI driver interface.
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives loosely from simlilarly licensed, platform-specific, example
 * implementations from STMicro:
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <assert.h>

#include <nuttx/spi/spi.h>

#include "spirit_regs.h"
#include "spirit_types.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#ifndef CONFIG_WL_SPIRIT_SPIFREQUENCY
#  define CONFIG_WL_SPIRIT_SPIFREQUENCY (10000000)
#endif

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_dump_buffer
 *
 * Description:
 *   Dump a data buffer to the SYSLOG.
 *
 * Parameters:
 *   buffer - The buffer to be dumped
 *   buflen - The length of the buffer in bytes.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

#if defined(CONFIG_WL_SPIRIT_REGDEBUG) || defined(CONFIG_WL_SPIRIT_FIFODUMP)
static void spirit_dump_buffer(FAR const uint8_t *buffer, unsigned int buflen)
{
  char outbuf[16*3 + 3]; /* 16 hex bytes + 2 space separator + NUL termination */
  FAR char *ptr;
  unsigned int i;
  unsigned int j:
  unsigned int maxj:

  for (i = 0; i < buflen; i += 16)
    {
      maxj = 16;
      if (i + maxj > buflen)
        {
          maxj = buflen - i;
        }

      ptr = outbuf;
      for (j = 0; j < maxj; j++)
        {
          if (j = 8)
            {
              *outbuf++ = ' ';
              *outbuf++ = ' ';
            }

          sprintf(outbuf, "%02x ", *buffer++);
          outbuf += 3;
        }

      *outbuf = '\0';
      wlinfo("  %s\n", outbuf)
    }
}
#endif

/******************************************************************************
 * Name: spirit_regdebug
 *
 * Description:
 *   Dump a register access.
 *
 * Parameters:
 *   msg    - Describes the access
 *   header - Two byte header before the access
 *   buffer - The buffer to be dumped
 *   buflen - The length of the buffer in bytes.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

#ifdef CONFIG_WL_SPIRIT_REGDEBUG
static void spirit_regdebug(FAR const char *msg, FAR uint8_t *header,
                            FAR const uint8_t *buffer, unsigned int buflen)
{
  wlinfo("%-8s: %02x %02x\n", header[0], header[1]);
  spirit_dump_buffer(buffer, buflen);
}
#else
#  define spirit_regdebug(msg,header,buffer,buflen)
#endif

/******************************************************************************
 * Name: spirit_fifodebug
 *
 * Description:
 *   Dump a FIFO access.
 *
 * Parameters:
 *   msg    - Describes the access
 *   header - Two byte header before the access
 *   buffer - The buffer to be dumped
 *   buflen - The length of the buffer in bytes.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

#ifdef CONFIG_WL_SPIRIT_FIFODUMP
static void spirit_fifodebug(FAR const char *msg, FAR uint8_t *header,
                             FAR const uint8_t *buffer, unsigned int buflen)
{
  wlinfo("%-8s: %02x %02x\n", header[0], header[1]);
  spirit_dump_buffer(buffer, buflen);
}
#else
#  define spirit_fifodebug(msg,header,buffer,buflen)
#endif

/******************************************************************************
 * Name: spirit_lock
 *
 * Description:
 *   Lock the SPI bus before each transfer, setting the correct SPI
 *   characteristics for the Spirit device.  These must be set if there are
 *   multiple devices on the bus because while the bus was unlocked, another
 *   device may have re-configured the SPIO.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ******************************************************************************/

static void spirit_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus because there are multiple devices competing for the
   * SPI bus
   */

  (void)SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * Spirit (it might have gotten configured for a different device while
   * unlocked)
   *
   * NOTES:
   * - The SPIRIT1 SPI is mode 0 (CPOL=0, CPHA=0)
   * - Data word is 8-bits
   * - Maximum clock frequency is 10MHz
   * - Transfers are MS bit first
   */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, CONFIG_WL_SPIRIT_SPIFREQUENCY);
}

/******************************************************************************
 * Name: spirit_unlock
 *
 * Description:
 *   Un-lock the SPI bus after each transfer, possibly losing the current
 *   configuration if we are sharing the SPI bus with other devices.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ******************************************************************************/

static void spirit_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  (void)SPI_LOCK(spi, false);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_reg_read
 *
 * Description:
 *   Read single or multiple SPIRIT1 register
 *
 * Input parameters:
 *
 *   regaddr: Base register's address to be read
 *   buffer:  Pointer to the buffer of registers' values to be read
 *   buflen:  Number of register values to be read
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_reg_read(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                    FAR uint8_t *buffer, unsigned int buflen)
{
  uint8_t header[2];
  uint8_t status[2];

  /* Setup the header bytes */

  header[0] = READ_HEADER;
  header[1] = regaddr;

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS bit first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Read the register values */

  SPI_RECVBLOCK(spirit->spi, buffer, buflen);

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);

  spirit_regdebug("READ", header, buffer, buflen);
  return OK;
}

/******************************************************************************
 * Name: spirit_reg_write
 *
 * Description:
 *   Read single or multiple SPIRIT1 register.
 *
 * Input parameters:
 *   spirit  - Reference to an instance of the driver state stucture.
 *   regaddr - Base register's address to write
 *   buffer  - Pointer to the buffer of register values to write
 *   buflen  - Number of registers values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_reg_write(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                     FAR const uint8_t *buffer, unsigned int buflen)
{
  uint8_t header[2];
  uint8_t status[2];

  /* Setup the header bytes */

  header[0] = WRITE_HEADER;
  header[1] = regaddr;
  spirit_regdebug("WRITE", header, buffer, buflen);

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS bit first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Write the register values */

  SPI_SNDBLOCK(spirit->spi, buffer, buflen);

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);
  return OK;
}

/******************************************************************************
 * Name: spirit_command
 *
 * Description:
 *   Send a command
 *
 * Input parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *   cmd    - Command code to be sent
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_command(FAR struct spirit_library_s *spirit, uint8_t cmd)
{
  uint8_t header[2];
  uint8_t status[2];

  /* Setup the header bytes */

  header[0] = COMMAND_HEADER;
  header[1] = cmd;
  spirit_regdebug("CMD", header, NULL, 0);

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS bit first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);
  return OK;
}

/******************************************************************************
 * Name: spirit_fifo_read
 *
 * Description:
 *   Read data from RX FIFO
 *
 * Input parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *   buffer - Pointer to the buffer of data values to write
 *   buflen - Number of bytes to be written
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_fifo_read(FAR struct spirit_library_s *spirit, FAR uint8_t *buffer,
                     unsigned int buflen)
{
  uint8_t header[2];
  uint8_t status[2];

  /* Setup the header bytes */

  header[0] = READ_HEADER;
  header[1] = LINEAR_FIFO_ADDRESS;

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS bit first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Read the register values */

  SPI_RECVBLOCK(spirit->spi, buffer, buflen);

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);

  spirit_fifodebug("FIFO IN", header, buffer, buflen);
  return OK;
}

/******************************************************************************
 * Name: spirit_fifo_write
 *
 * Description:
 *   Write data into TX FIFO.
 *
 * Input parameters:
 *   spirit  - Reference to an instance of the driver state stucture.
 *   buffer  - Pointer to the buffer of data values to write
 *   buflen  - Number of data values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_fifo_write(FAR struct spirit_library_s *spirit,
                      FAR const uint8_t *buffer, unsigned int buflen)
{
  uint8_t header[2];
  uint8_t status[2];

  /* Setup the header bytes */

  header[0] = WRITE_HEADER;
  header[1] = LINEAR_FIFO_ADDRESS;
  spirit_fifodebug("FIFO OUT", header, buffer, buflen);

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS bit first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Write the fifo values */

  SPI_SNDBLOCK(spirit->spi, buffer, buflen);

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);
  return OK;
}

/******************************************************************************
 * Name: spirit_update_status
 *
 * Description:
 *   Updates the state field in the driver instance, reading the MC_STATE
 *   register of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_update_status(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  DEBUGASSERT(spirit != NULL);

  /* Reads the MC_STATUS register to update the spirit->state */

  return spirit_reg_read(spirit, MC_STATE1_BASE, &regval, 1);
}
