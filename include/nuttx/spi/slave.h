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

#define SPI_SCTRLR_BIND(c,d,m,n) ((c)->ops->bind(c,d,m,n))

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

#define SPI_SCTRLR_UNBIND(c) ((c)->ops->unbind(c))

/****************************************************************************
 * Name: SPI_SCTRLR_ENQUEUE
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface.  This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   Zero if the word was successfully queue; A negated errno valid is
 *   returned on any failure to enqueue the word (such as if the queue is
 *   full).
 *
 ****************************************************************************/

#define SPI_SCTRLR_ENQUEUE(c,v)  ((c)->ops->enqueue(c,v))

/****************************************************************************
 * Name: SPI_SCTRLR_QFULL
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full
 *
 ****************************************************************************/

#define SPI_SCTRLR_QFULL(c)  ((c)->ops->qfull(c))

/****************************************************************************
 * Name: SPI_SCTRLR_QFLUSH
 *
 * Description:
 *   Discard all saved values in the output queue.  On return from this
 *   function the output queue will be empty.  Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SCTRLR_QFLUSH(c)  ((c)->ops->qflush(c))

/****************************************************************************
 * Name: SPI_SDEV_SELECT
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   driver detects any change in the chip select pin.
 *
 * Input Parameters:
 *   sdev     - SPI device interface instance
 *   selected - True: chip select is low (selected);
 *
 * Returned Value:
 *   none
 *
 * Assumptions:
 *   May be called from an interrupt handler.  Processing should be as
 *   brief as possible.
 *
 ****************************************************************************/

#define SPI_SDEV_SELECT(d,s) ((d)->ops->select(d,s))

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
 *   all subsequent calls to enqueue() or rece() appropriately for the
 *   current command/data selection.
 *
 * Input Parameters:
 *   sdev - SPI device interface instance
 *   data - True: Data is selected
 *
 * Returned Value:
 *   none
 *
 * Assumptions:
 *   May be called from an interrupt handler.  Processing should be as
 *   brief as possible.
 *
 ****************************************************************************/

#define SPI_SDEV_CMDDATA(d,i) ((d)->ops->cmddata(d,i))

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
 * Assumptions:
 *   May be called from an interrupt handler and the response is usually
 *   time critical.
 *
 ****************************************************************************/

#define SPI_SDEV_GETDATA(d)  ((d)->ops->getdata(d))

/****************************************************************************
 * Name: SPI_SDEV_RECEIVE
 *
 * Description:
 *   This is a SPI device callback that used when the SPI device controller
 *   receives a new value shifted in and requires the next value to be
 *   shifted out.  Notice that these values my be out of synchronization by
 *   several words.
 *
 * Input Parameters:
 *   sdev - SPI device interface instance
 *   data - The last command/data value that was shifted in
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May be called from an interrupt handler and in time-critical
 *   circumstances.  A good implementation might just add the newly
 *   received word to a queue, post a processing task, and return as
 *   quickly as possible to avoid any data overrun problems.
 *
 ****************************************************************************/

#define SPI_SDEV_RECEIVE(d,v)  ((d)->ops->receive(d,v))

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* There are two interfaces defined for the implementation of SPI slave:
 *
 * 1) struct spi_sctrlr_s:   Defines one interface between the SPI
 *    slave device and the SPI slave controller hardware.  This interface
 *    is implemented by the SPI slave device controller lower-half driver
 *    and is provided to the SPI slave device driver when that driver
 *    is initialized.  That SPI slave device initialization function is
 *    unique to the SPI slave implementation.  The prototype is probably
 *    something like:
 *
 *      FAR struct spi_sctrlr_s *xyz_spi_slave_initialize(int port);
 *
 *    Given an SPI port number, this function returns an instance of the
 *    SPI slave controller interface.
 *
 *    The actual prototype and more detailed usage instructions should
 *    appear in a header file associated with the specific SPI slave
 *    implementation.
 *
 * 2) struct spi_sdev_s:  Defines the second interface between the SPI
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
 *
 * 2) Board-specific logic then calls up_dev_initialize() to initialize
 *    the SPI slave device.  The board-specific logic passes the instance
 *    of struct spi_sctrlr_s to support the initialization.
 *
 * 3) The SPI slave device driver creates and initializes an instance of
 *    struct spi_sdev_s; it passes this instance to the bind() method of
 *    of the SPI slave controller interface.
 *
 * 4) The SPI slave controller will (1) call the slaved device's select()
 *    and cmddata() methods to indicate the initial state of the chip select
 *    and any command/data selection, then (2) call the slave device's
 *    getdata() method to get the value that will be shifted out the SPI
 *    clock is detected.  The kind of data returned the getdata() method
 *    may be contingent on the current command/data setting reported the
 *    device cmddata() method.  The driver may enqueue additional words
 *    to be shifted out at any time by The calling the SPI slave
 *    controller's enqueue() method.
 *
 * 5) Upon return from the bind method, the SPI slave controller will be
 *    fully "armed" and ready to begin normal SPI data transfers.
 *
 * A typical (non-DMA) data transfer proceeds as follows:
 *
 * 1) Internally, the SPI slave driver detects that the SPI chip select
 *    has gone low, selecting this device for data transfer.  The SPI
 *    slave controller will notify the slave device by called its
 *    select() method.
 *
 * 2) If a change in the command/data state changes any time before,
 *    during, or after the chip is selected, that new command/data state
 *    will reported to the device driver via the cmddata() method.
 *
 * 3) As the first word is shifted in, the command or data word obtained
 *    by the initial call to getdata() will be shifted out.  As soon as
 *    the clock is detected, the SPI controller driver will call the
 *    getdata() method again to get a default second word to be shifted
 *    out.  NOTES: (1) the SPI slave device has only one word in bit
 *    times to provide this value! (2) The SPI device probably cannot
 *    really output anything meaning until it receives a decodes the
 *    first word received from the master.
 *
 * 4) When the first word from the master is shifted in, the SPI
 *    controller driver will call the device's receive() method to
 *    provide the master with the command word that was just shifted
 *    in.
 *
 *    For the case of bi-directional data transfer or of a transfer of
 *    data from the SPI device to the master, the SPI device driver
 *    should call the controller's enqueue() method to provide the next
 *    value(s) to be shifted out. If the SPI device responds with this
 *    value before clocking begins for the next word, that that value
 *    will be used.  Otherwise, the value obtained from getdata() in
 *    step 3 will be shifted out.
 *
 * 5) The SPI device's receive() method will be called in a similar
 *    way after each subsequent word is clocked in.
 *
 *    For the case of bi-directional data transfer or of a uni-directional
 *    transfer of data from the SPI device to the master, the SPI device
 *    driver can call the enqueue() methods as it has new data to be shifted
 *    out.  The goal of the SPI device driver for this kind of transfer is
 *    to supply valid output data at such a rate that data underruns do not
 *    occur.  In the event of a data underrun, the SPI slave controller
 *    driver will fallback to the default output value obtained from the
 *    last getdata() call.
 *
 *    The SPI device driver can detect if there is space to enqueue
 *    additional data by calling the qfull() method.
 *
 *    For the case of uni-directional transfer of data from the master to
 *    the SPI device, there is no need to call the enqueue() method at all;
 *    the value that is shifted out is not important that fallback behavior
 *    is sufficient.
 *
 *    The SPI slave controller driver, of course, has no sense of the
 *    directionality of a data transfer; its role is only to exchange the
 *    data shifted in from the master with new data to be shifted out from
 *    the SPI device driver.
 *
 * 6) The activity of 5) will continue until the master raises the chip
 *    select signal.  In that case, the SPI slave controller driver will
 *    again call the SPI device's select() method.  At this point, the SPI
 *    controller driver may have several words enqueued.  It will not
 *    discard these unless the SPI device driver calls the qflush()
 *    method.
 *
 *    Some master side implementations may simply tie the chip select signal
 *    to ground if there are no other devices on the SPI bus.  In that case,
 *    the initial indication of chip selected will be the only call to the
 *    select() method that is made.
 *
 *    Other SPI peripherals (such as Atmel) do not make the state of the
 *    chip select pin available (only the final rising edge transitions).
 *    So the SPI device driver implementation may use the chip select
 *    reports to optimize performance, but the design should never depend
 *    upon it.
 *
 * A typical DMA data transfer processes as follows:
 * To be provided -- I do not have a design in mind to support DMA on the
 * Slave side.  The design might be very complex because:
 *
 * 1) You need DMA buffers of fixed size, but you cannot know the size of a
 *    transfer in advance, it could be much larger than your buffer or much
 *    smaller.  The DMA would fail in either case.
 *
 * 2) You cannot setup the DMA before the transfer.  In most SPI protocols,
 *    the first word send is a command to read or write something following
 *    by a sequence of transfers to implement the write.  So you have very,
 *    very limited time window to setup the correct DMA to respond to the
 *    command.  I am not certain that it can be done reliably.
 *
 *    Inserting dummy words into the protocol between the first command word
 *    and the remaining data transfer could allow time to set up the DMA.
 *
 * 3) I mentioned that you do not know the size of the transfer in advance.
 *    If you set up the DMA to terminate to soon, then you lose the last part
 *    of the transfer.  If you set the DMA up to be too large, then you will
 *    get no indication when the transfer completes.
 *
 *    The chip select going high would be one possibility to detect the end
 *    of a transfer.  You could cancel a DMA in progress if the CS changes,
 *    but I do not know if that would work.  If there is only one device on
 *    the SPI bus, then most board designs will save a pin and simply tie CS
 *    to ground.  So the CS is not always a reliable indicator of when the
 *    transfer completes.
 *
 * 4) The option is to use a timer but that would really slow down the
 *    transfers if each DMA has to end with a timeout.  It would be faster
 *    non-DMA transfers.
 *
 *    If the device as a very restricted protocol, like just register reads
 *    and writes, then it might possible to implement DMA.  However, that
 *    solution would not be general and probably not an appropriate part of
 *    a general OS.  But if the interface is unpredictable, such as reading/
 *    variable amounts of data from FLASH, there is more risk.  A general
 *    solution might not be possible.
 */

enum spi_smode_e
{
  SPISLAVE_MODE0 = 0,     /* CPOL=0 CHPHA=0 */
  SPISLAVE_MODE1,         /* CPOL=0 CHPHA=1 */
  SPISLAVE_MODE2,         /* CPOL=1 CHPHA=0 */
  SPISLAVE_MODE3          /* CPOL=1 CHPHA=1 */
};

/* The SPI slave controller driver vtable */

struct spi_sctrlr_s; /* Forward reference */
struct spi_sdev_s;   /* Forward reference */

struct spi_sctrlrops_s
{
  CODE void     (*bind)(FAR struct spi_sctrlr_s *sctrlr,
                   FAR struct spi_sdev_s *sdev, enum spi_smode_e mode,
                   int nbits);
  CODE void     (*unbind)(FAR struct spi_sctrlr_s *sctrlr);
  CODE int      (*enqueue)(FAR struct spi_sctrlr_s *sctrlr, uint16_t data);
  CODE bool     (*qfull)(FAR struct spi_sctrlr_s *sctrlr);
  CODE void     (*qflush)(FAR struct spi_sctrlr_s *sctrlr);
};

/* SPI slave controller private data.  This structure only defines the
 * initial fields of the structure visible to the SPI device driver.  The
 * specific implementation may add additional, device specific fields after
 * the vtable structure pointer.
 */

struct spi_sctrlr_s
{
  FAR const struct spi_sctrlrops_s *ops;

  /* Private SPI slave controller driver data may follow */
};

/* The SPI slave device driver vtable */

struct spi_sdevops_s
{
  CODE void     (*select)(FAR struct spi_sdev_s *sdev, bool selected);
  CODE void     (*cmddata)(FAR struct spi_sdev_s *sdev, bool data);
  CODE uint16_t (*getdata)(FAR struct spi_sdev_s *sdev);
  CODE void     (*receive)(FAR struct spi_sdev_s *sdev, uint16_t cmd);
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
