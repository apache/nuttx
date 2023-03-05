/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_spi.c
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
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
 * Input Parameters:
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
  char outbuf[16 * 3 + 3]; /* 16 hex bytes + 2 space separator + NUL termination */
  FAR char *ptr;
  unsigned int i;
  unsigned int j;
  unsigned int maxj;

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
          if (j == 8)
            {
              *ptr++ = ' ';
              *ptr++ = ' ';
            }

          snprintf(ptr, sizeof(outbuf) - (ptr - outbuf), "%02x ", *buffer++);
          ptr += 3;
        }

      *ptr = '\0';
      wlinfo("  %s\n", ptr);
    }
}
#endif

/******************************************************************************
 * Name: spirit_regdebug
 *
 * Description:
 *   Dump a register access.
 *
 * Input Parameters:
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
 * Input Parameters:
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
 * Input Parameters:
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

  SPI_LOCK(spi, true);

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
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_WL_SPIRIT_SPIFREQUENCY);
}

/******************************************************************************
 * Name: spirit_unlock
 *
 * Description:
 *   Un-lock the SPI bus after each transfer, possibly losing the current
 *   configuration if we are sharing the SPI bus with other devices.
 *
 * Input Parameters:
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

  SPI_LOCK(spi, false);
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
 * Input Parameters:
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

  /* Update Spirit status. 16-bit status is returned MS byte first */

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
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
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

  /* Update Spirit status. 16-bit status is returned MS byte first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Write the register values */

  SPI_SNDBLOCK(spirit->spi, buffer, buflen);

  /* Deselect the Spirit device and return the result */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  spirit_unlock(spirit->spi);
  return OK;
}

/******************************************************************************
 * Name: spirit_reg_modify
 *
 * Description:
 *   Perform atomic read/modify/write on a single SPIRIT1 register.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
 *   regaddr - Base register's address to write
 *   clrbits - Bits to clear in the register
 *   setbits - Bits to set in the register
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_reg_modify(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                      uint8_t setbits, uint8_t clrbits)
{
  uint8_t header[2];
  uint8_t status[2];
  uint8_t regval;

  /* Setup the header byte to read the register */

  header[0] = READ_HEADER;
  header[1] = regaddr;

  /* Lock the SPI bus and select the Spirit device */

  spirit_lock(spirit->spi);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes (ignoring the returned SPIRIT1 status bytes) */

  SPI_SNDBLOCK(spirit->spi, header, 2);

  /* Read the register value */

  regval = SPI_SEND(spirit->spi, 0xff);
  spirit_regdebug("READ", header, &regval, 1);

  /* Modify the register value */

  regval &= ~clrbits;
  regval |=  setbits;

  /* Setup the header byte for the write operation */

  header[0] = WRITE_HEADER;
  header[1] = regaddr;
  spirit_regdebug("WRITE", header, &regval, 1);

  /* Toggle Chip select so that we get the correct status */

  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), false);
  SPI_SELECT(spirit->spi, SPIDEV_WIRELESS(0), true);

  /* Write the header bytes and read the SPIRIT1 status bytes */

  SPI_EXCHANGE(spirit->spi, header, status, 2);

  /* Update Spirit status. 16-bit status is returned MS byte first */

  spirit->u.u16 = ((uint16_t)status[0] << 8) | (uint16_t)status[1];

  /* Write the register value */

  SPI_SEND(spirit->spi, regval);

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
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state structure.
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

  /* Update Spirit status. 16-bit status is returned MS byte first */

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
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state structure.
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

  /* Update Spirit status. 16-bit status is returned MS byte first */

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
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
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

  /* Update Spirit status. 16-bit status is returned MS byte first */

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
 *   spirit - Reference to an instance of the driver state structure.
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

/******************************************************************************
 * Name: spirit_waitstatus
 *
 * Description:
 *   Poll until the Spirit status is the requested value or until a timeout
 *   occurs.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   state  - That that we are waiting for.
 *   msec   - Timeout in millisedonds
 *
 * Returned Value:
 *   OK on success; a negated errno on a timeout
 *
 * Assumptions:
 *   We have exclusive access to the driver state and to the spirit library.
 *
 ******************************************************************************/

int spirit_waitstatus(FAR struct spirit_library_s *spirit,
                      enum spirit_state_e state, unsigned int msec)
{
  clock_t start;
  clock_t ticks;
  clock_t elapsed;
  int ret;

#ifdef CONFIG_DEBUG_SPI_INFO
  /* If the SPI logic is generating debug output, then a longer timeout is
   * probably needed.
   */

  msec <<= 4;
#endif

  /* Convert the MSEC timedelay to clock ticks, making sure that the
   * resulting delay in ticks is greater than or equal to the requested time
   * in MSEC.
   *
   * REVISIT: If USEC_PER_TICK and 'msec' are large, then the second
   * computation may overflow!
   */

#if (MSEC_PER_TICK * USEC_PER_MSEC) == USEC_PER_TICK
  ticks = (msec + (MSEC_PER_TICK - 1)) / MSEC_PER_TICK;
#else
  ticks = ((clock_t)msec * USEC_PER_MSEC + (USEC_PER_TICK - 1)) /
           USEC_PER_TICK;
#endif

  /* The time that we started the wait */

  start = clock_systime_ticks();

  /* Loop until the status change occurs (or the wait times out) */

  do
    {
      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }

      elapsed = clock_systime_ticks() - start;
    }
  while (spirit->u.state.MC_STATE != state && elapsed <= ticks);

  if (spirit->u.state.MC_STATE == state)
    {
      return OK;
    }

  /* This is probably not an error.  In a busy radio environment, there
   * are many race conditions.  Most typically, just when the driver is
   * setting up to perform a transmission, the hardware commits to a
   * reception.  The symptom is that the the above loop times out out
   * waiting to go into the TX state (because it is in the RX state).
   *
   * Complaining with too much debug output just aggravates the problem.
   */

  wlinfo("Timed out: %lu > %lu (%u msec)\n",
         (unsigned long)elapsed, (unsigned long)ticks, msec);
  wlinfo("with MC status: current=%02x desired=%02x\n",
         spirit->u.state.MC_STATE, state);
  return -ETIMEDOUT;
}
