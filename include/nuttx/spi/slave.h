/****************************************************************************
 * include/nuttx/spi/slave.h
 *
 *   Copyright(C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SPI_SLAVE_H
#define __INCLUDE_NUTTX_SPI_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SPI_SLAVE - Enable support for SPI slave features
 * CONFIG_SPI_SLAVE_DMA - Enable support for DMA data transfers (not
 *   implemented in initial version).
 */

/* Access macros ************************************************************/

/****************************************************************************
 * Name: SPI_SCTRLR_BIND
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface.  Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   sdev   - SPI slave device interface instance
 *   mode   - The SPI mode requested
 *   nbits  - The number of bits requests.
 *            If value is greater > 0 then it implies MSB first
 *            If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SCTRLR_BIND(c,d,m,n) ((c)->bind(c,d,m,n))

/****************************************************************************
 * Name: SPI_SCTRLR_UNBIND
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface.  Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state,
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SCTRLR_UNBIND(c) ((c)->unbind(c))

/****************************************************************************
 * Name: SPI_SCTRLR_SETDATA
 *
 * Description:
 *   Set the next value to be shifted out from the interface.  This primes
 *   the controller driver for the next transfer but has no effect on any
 *   in-process or currently "committed" transfers
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SCTRLR_SETDATA(c,v)  ((c)->setdata(c,v))

/****************************************************************************
 * Name: SPI_SDEV_SELECTED
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   driver detects any change in the chip select pin.
 *
 * Input Parameters:
 *   sdev       - SPI device interface instance
 *   isselected - True: chip select is low (selected);
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SDEV_SELECTED(d,i) ((c)->selected(d,i))

/****************************************************************************
 * Name: SPI_SDEV_CMDDATA
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   driver detects any change command/data condition.
 *
 *   Normally only LCD devices distinguish command and data.   For devices
 *   that do not distinguish between command and data, this method may be
 *   a stub.; For devices that do make that distinction, they should treat
 *   all subsequent calls to getdata() or exchange() appropriately for the
 *   current command/data selection.
 *
 * Input Parameters:
 *   sdev   - SPI device interface instance
 *   isdata - True: Data is selected
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SDEV_CMDDATA(d,i) ((c)->cmddata(d,i))

/****************************************************************************
 * Name: SPI_SDEV_GETDATA
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   requires data be shifted out at the next leading clock edge.  This
 *   is necessary to "prime the pump" so that the SPI controller driver
 *   can keep pace with the shifted-in data.
 *
 *   The SPI controller driver will prime for both command and data
 *   transfers as determined by a preceding call to the device cmddata()
 *   method.  Normally only LCD devices distinguish command and data.
 *
 * Input Parameters:
 *   sdev - SPI device interface instance
 *
 * Returned Value:
 *   The next data value to be shifted out
 *
 ****************************************************************************/

#define SPI_SDEV_GETDATA(d,v)  ((d)->getdata(d,v))

/****************************************************************************
 * Name: SPI_SDEV_EXCHANGE
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   receives a new value shifted in and requires the next value to be
 *   shifted out.  Notice that these values my be out of synchronization by
 *   as much as two words:  The value to be shifted out may be two words
 *   beyond the value that was just shifted in.
 *
 * Input Parameters:
 *   sdev - SPI device interface instance
 *   data - The last command/data value that was shifted in
 *
 * Returned Value:
 *   The next data value to be shifted out
 *
 ****************************************************************************/

#define SPI_SDEV_EXCHANGE(d,v)  ((d)->exchange(d,v))

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* There are two interfaces defined for the implementation of SPI slave:
 *
 * 1) struct spi_sctrlr_s - Defines one interface between the SPI
 *    slave device and the SPI slave controller hardware.  This interface
 *    is implemented by the SPI slave device controller lower-half driver
 *    and is provided to the the SPI slave device driver when that driver
 *    is initialization.  That SPI slave device initialization function might
 *    look like:
 *
 *      int xyz_dev_initialize(FAR struct spi_sctrlr_s *sctrlr);
 *
 * 2) struct spi_sdev_s - Defines the second center between the SPI
 *    slave device and the SPI slave controller hardware.  This interface
 *    is implemented by the SPI slave device.  The slave device passes this
 *    interface to the struct spi_sctrlr_s during initialization
 *    be calling the bind() method of the struct spi_sctrlr_s
 *    interface.
 *
 * The basic initialization steps are:
 *
 * 1) Board-specific logic calls board- or chip-specific logic to create an
 *    instance of the SPI slave controller interface, struct spi_sctrlr_s.
 * 2) Board-specific logic then calls xyz_dev_initialize() to initialize
 *    the SPI slave device.  The board-specific logic passes the instance
 *    of struct spi_sctrlr_s to support the initialization.
 * 3) The SPI slave device driver creates and initializes an instance of
 *    struct spi_sdev_s; it passes this instance to the bind() method of
 *    of the SPI slave controller interface.
 * 4) The SPI slave controller will (1) call the slaved device's cmddata()
 *    method to indicate the initial state of any command/data selection,
 *    then (2) call the slave device's getdata() method to get the value
 *    that will be shifted out the SPI clock is detected.  The kind of
 *    data returned the getdata() method may be contingent on the current
 *    command/data setting previous reported the device cmddata() method.
 *    driver can change the next word to be shifted out at any time by
 *    The calling the SPI slave controller's setdata() method.
 * 5) Upon return from the bind method, the SPI slave controller will be
 *    fully "armed" and ready to begin normal SPI data transfers.
 *
 * A typical (non-DMA) data transfer proceeds as follows:
 *
 * 1) Internally, the SPI slave driver detects that the SPI chip select
 *    has gone low, selecting this device for data transfer.  If the SPI
 *    slave device's select method is non-NULL, the SPI slave controller
 *    will notify the slave device by called its selected() method.
 * 2) If a change in the command/data status changes any time before,
 *    during, or after the chip is selected, that new command state state
 *    will reported to the device driver via the cmddata() method.
 * 3) As the first word is shifted in, the command or data word will be
 *    shifted out.  As soon as the clock is detected, the SPI controller
 *    driver will call the getdata() method again to get the second word
 *    to be shifted out.  NOTE: the SPI slave device has only one word in
 *    bit times to provide this value!
 * 4) When the first word is shifted in, the SPI controller driver will
 *    call the device's exchange() method to both provide the master
 *    command that was just shifted in as well to obtain the next value
 *    to shift out.  If the SPI device responds with this value before
 *    clocking begins for the next word, that that value will be used
 *    (and the backup value obtained in 3) will be discarded).
 * 5) The SPI device's echange_cmd/data() will will be called in a similar
 *    way after each subsequent word is clocked in.  The only difference
 *    is that word returned from the previous call to exchange/cmddata()
 *    will not be discard.
 * 6) The activity of 5) will continue until the master raises the chip
 *    select signal.  In that case, the SPI slave controller driver will
 *    again call the SPI device's selected().  At this point, the SPI
 *    controller driver may have two words buffered.  If will discard the
 *    last and retain only the current word prepared to be shifted out.
 *    That value can be changed by the  SPI device driver by calling the
 *    setdata() method.
 *
 * A typical DMA data transfer processes as follows:
 * To be provided
 */

enum spi_smode_e
{
  SPISLAVE_MODE0 = 0,     /* CPOL=0 CHPHA=0 */
  SPISLAVE_MODE1,         /* CPOL=0 CHPHA=1 */
  SPISLAVE_MODE2,         /* CPOL=1 CHPHA=0 */
  SPISLAVE_MODE3          /* CPOL=1 CHPHA=1 */
};

/* The SPI slave controller driver vtable */

struct spi_sctrlr_s;
struct spi_sdev_s;
struct spi_slaveops_s
{
  CODE void     (*bind)(FAR struct spi_sctrlr_s *sctrlr, 
                   FAR spi_sdev_s *sdev, enum spi_mode_e mode, int nbits);
  CODE void     (*unbind)(FAR struct spi_sctrlr_s *sctrlr);
  CODE void     (*setdata)(FAR struct spi_sctrlr_s *sctrlr, uint16_t data);
};

/* SPI slave controller private data.  This structure only defines the
 * initial fields of the structure visible to the SPI device driver.  The
 * specific implementation may add additional, device specific fields after
 * the vtable structure pointer.
 */

struct spi_sctrlr_s
{
  FAR const struct spi_slaveops_s *ops;

  /* Private SPI slave controller driver data may follow */
};

/* The SPI slave device driver vtable */

struct spi_sdevops_s
{
  CODE void     (*selected)(FAR struct spi_sdev_s *sdev, bool isselected);
  CODE void     (*cmddata)(FAR struct spi_sdev_s *sdev, bool isdata);
  CODE uint16_t (*getdata)(FAR struct spi_sdev_s *sdev);
  CODE uint16_t (*exchange)(FAR struct spi_sdev_s *sdev), uint16_t cmd);
};

/* SPI slave device private data.  This structure only defines the initial
 * fields of the structure visible to the SPI slave controller driver.  The
 * specific implementation may add additional, device specific fields after
 * the vtable structure pointer.
 */

struct spi_sdev_s
{
  FAR const struct spi_sdevops_s *ops;

  /* Private SPI slave device driver data may follow */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SPI_SLAVE_H */
