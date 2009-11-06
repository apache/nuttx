/****************************************************************************
 * include/nuttx/mmcsd.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __NUTTX_MMCSD_H
#define __NUTTX_MMCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* MMC/SD events needed by the driver */

#define MMCSDEVENT_EJECTED       (1 << 0) /* Bit 0: CD/DAT3 transition low, media removed */
#define MMCSDEVENT_INSERTED      (1 << 1) /* Bit 1: CD/DAT3 transition high, media inserted */
#define MMCSDEVENT_CMDDONE       (1 << 2) /* Bit 2: Command+response complete */
#define MMCSDEVENT_READCMDDONE   (1 << 3) /* Bit 3: Read command done */
#define MMCSDEVENT_WRITECMDDONE  (1 << 4) /* Bit 4: Write command done */
#define MMCSDEVENT_READDATADONE  (1 << 5) /* Bit 5: Read data done */
#define MMCSDEVENT_WRITEDATADONE (1 << 6) /* Bit 6: Write data done */
#define MMCSDEVENT_CMDBUSYDONE   (1 << 7) /* Bit 7: Command with transition to not busy */

#define MMCSDEVENT_ALLEVENTS     0xff

/****************************************************************************
 * Name: MMCSD_RESET
 *
 * Description:
 *   Reset the MMC/SD controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define MMCSD_RESET(dev) ((dev)->reset(dev))

/****************************************************************************
 * Name: MMCSD_STATUS
 *
 * Description:
 *   Get MMC/SD status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see MMCSD_STATUS_* defines)
 *
 ****************************************************************************/

#define MMCSD_STATUS(dev)        ((d)->status(dev))

/* MMC/SD status bits */

#define MMCSD_STATUS_PRESENT     0x01 /* Bit 0=1: MMC/SD card present */
#define MMCSD_STATUS_WRPROTECTED 0x02 /* Bit 1=1: MMC/SD card write protected */

#define MMCSD_PRESENT(dev)       ((MMCSD_STATUS(dev) & MMCSD_STATUS_PRESENT) != 0)
#define MMCSD_WRPROTECTED(dev)   ((MMCSD_STATUS(dev) & MMCSD_STATUS_WRPROTECTED) != 0)

/****************************************************************************
 * Name: MMCSD_WIDEBUS
 *
 * Description:
 *   Enable/disable wide (4-bit) data bus
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   enable - TRUE: enable wide bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define MMCSD_WIDEBUS(dev,enable) ((dev)->widebus(dev,enable))

/****************************************************************************
 * Name: MMCSD_CLOCK
 *
 * Description:
 *   Enable/disable MMC/SD clocking
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   rate - Specifies the clocking to use (see enum mmcsd_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define MMCSD_CLOCK(dev,rate) ((dev)->clock(dev,rate))

/****************************************************************************
 * Name: MMCSD_SETBLOCKLEN
 *
 * Description:
 *   Set the MMC/SD block length and block count
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   blocklen - The block length
 *   nblocks  - The block count
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

#define MMCSD_SETBLOCKLEN(dev,blocklen,nblocks) \
  ((dev)->setblocklen(dev,blocklen,nblocks))

/****************************************************************************
 * Name: MMCSD_ATTACH
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

#define MMCSD_ATTACH(dev) ((dev)->attach(dev))

/****************************************************************************
 * Name: MMCSD_SENDCMD
 *
 * Description:
 *   Send the MMC/SD command
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   cmd  - The command to send
 *   arg  - 32-bit argument required with some commands
 *   data - A reference to data required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define MMCSD_SENDCMD(dev,cmd,arg,data) ((dev)->sendcmd(dev,cmd,arg,data))

/****************************************************************************
 * Name: MMCSD_SENDDATA
 *
 * Description:
 *   Send more MMC/SD data
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   data - Data to be sent
 *
 * Returned Value:
 *   Number of bytes sent on succes; a negated errno on failure
 *
 ****************************************************************************/

#define MMCSD_SENDDATA(dev,data) ((dev)->senddata(dev,data))

/****************************************************************************
 * Name: MMCSD_RECVRx
 *
 * Description:
 *   Receive response to MMC/SD command
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on succes; a negated errno on failure
 *
 ****************************************************************************/

#define MMCSD_RECVR1(dev,buffer) ((dev)->recvR1(dev,buffer))
#define MMCSD_RECVR2(dev,buffer) ((dev)->recvR2(dev,buffer))
#define MMCSD_RECVR3(dev,buffer) ((dev)->recvR3(dev,buffer))
#define MMCSD_RECVR4(dev,buffer) ((dev)->recvR4(dev,buffer))
#define MMCSD_RECVR5(dev,buffer) ((dev)->recvR5(dev,buffer))
#define MMCSD_RECVR6(dev,buffer) ((dev)->recvR6(dev,buffer))

/****************************************************************************
 * Name: MMCSD_RECVDATA
 *
 * Description:
 *   Receive data from MMC/SD
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - Buffer in which to receive the data
 *
 * Returned Value:
 *   Number of bytes sent on succes; a negated errno on failure
 *
 ****************************************************************************/

#define MMCSD_RECVDATA(dev,buffer) ((dev)->recvdata(dev,buffer))

/****************************************************************************
 * Name: MMCSD_EVENTENABLE
 *
 * Description:
 *   Enable/disable notification of a set of MMC/SD events
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   eventset - A bitset of events to enable or disable (see MMCSDEVENT_*
 *              definitions
 *   enable   - TRUE: enable event; FALSE: disable events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define MMCSD_EVENTENABLE(dev,eventset)  ((dev)->eventenable(dev,eventset,TRUE))
#define MMCSD_EVENTDISABLE(dev,eventset) ((dev)->eventenable(dev,eventset,FALSE))
#define MMCSD_EVENTDISABLEALL(dev)       ((dev)->eventenable(dev,MMCSDEVENT_ALLEVENTS,FALSE))

/****************************************************************************
 * Name: MMCSD_EVENTWAIT
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout)
 *
 * Input Parameters:
 *   dev     - An instance of the MMC/SD device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means no timeout.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  If no events the
 *   returned event set is zero, then the wait was terminated by the timeout.
 *
 ****************************************************************************/

#define MMCSD_EVENTWAIT(dev,timeout)  ((dev)->eventwait(dev,timeout))

/****************************************************************************
 * Name: MMCSD_EVENTS
 *
 * Description:
 *   Return the current event set.  This supports polling for MMC/SD (vs.
 *   waiting).
 *
 * Input Parameters:
 *   dev     - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   Event set containing the current events (cleared after reading).
 *
 ****************************************************************************/

#define MMCSD_EVENTS(dev)  ((dev)->events(dev))

/****************************************************************************
 * Name: MMCSD_DMASUPPORTED
 *
 * Description:
 *   Return TRUE if the hardware can support DMA
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   TRUE if DMA is supported.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMASUPPORTED(dev) ((dev)->dmasupported(dev))
#else
#  define MMCSD_DMASUPPORTED(dev) (FALSE)
#endif

/****************************************************************************
 * Name: MMCSD_COHERENT
 *
 * Description:
 *   If the processor supports a data cache, then this method will make sure
 *   that the contents of the DMA memory and the data cache are coherent in
 *   preparation for the DMA transfer.  For write transfers, this may mean
 *   flushing the data cache, for read transfers this may mean invalidating
 *   the data cache.
 *
 * Input Parameters:
 *   dev   - An instance of the MMC/SD device interface
 *   addr  - The beginning address of the DMA
 *   len   - The length of the DMA
 *   write - TRUE: A write DMA will be performed; FALSE: a read DMA will be
 *           performed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_MMCSD_DMA) && defined(CONFIG_DATA_CACHE)
#  define MMCSD_COHERENT(dev,addr,len,write) ((dev)->coherent(dev,addr,len,write))
#else
#  define MMCSD_COHERENT(dev,addr,len,write)
#endif

/****************************************************************************
 * Name: MMCSD_DMAREADSETUP
 *
 * Description:
 *   Setup to perform a read DMA
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA from
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMAREADSETUP(dev,buffer) ((dev)->dmareadsetup(dev,buffer))
#else
#  define MMCSD_DMAREADSETUP(dev,buffer) (-ENOSYS)
#endif

/****************************************************************************
 * Name: MMCSD_DMAWRITESETUP
 *
 * Description:
 *   Setup to perform a write DMA
 *
 * Input Parameters:
 *   dev    - An instance of the MMC/SD device interface
 *   buffer - The memory to DMA into
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMAWRITESETUP(dev,buffer) ((dev)->dmawritesetup(dev,buffer))
#else
#  define MMCSD_DMAWRITESETUP(dev,buffer) (-ENOSYS)
#endif

/****************************************************************************
 * Name: MMCSD_DMASTART
 *
 * Description:
 *   Start the DMA
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMASTART(dev) ((dev)->dmastart(dev))
#else
#  define MMCSD_DMASTART(dev) (-ENOSYS)
#endif

/****************************************************************************
 * Name: MMCSD_DMASTOP
 *
 * Description:
 *   Stop the DMA
 *
 * Input Parameters:
 *   dev - An instance of the MMC/SD device interface
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMASTOP(dev) ((dev)->dmastop(dev))
#else
#  define MMCSD_DMASTOP(dev) (-ENOSYS)
#endif

/****************************************************************************
 * Name: MMCSD_DMASTATUS
 *
 * Description:
 *   Returnt the number of bytes remaining in the DMA transfer
 *
 * Input Parameters:
 *   dev       - An instance of the MMC/SD device interface
 *   remaining - A pointer to location in which to return the number of bytes
 *               remaining in the transfer.
 *
 * Returned Value:
 *   OK on succes; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_DMA
#  define MMCSD_DMASTATUS(dev,remaining) ((dev)->dmastatus(dev,remaining))
#else
#  define MMCSD_DMASTATUS(dev,remaining) (-ENOSYS)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Various clocking used by the MMC/SD driver */

enum mmcsd_clock_e
{
  CLOCK_MMCSD_DISABLED = 0, /* Clock is disabled */
  CLOCK_MMC_SLOW,           /* MMC initialization clocking */
  CLOCK_SD_SLOW,            /* SD initialization clocking */
  CLOCK_MMC_FAST,           /* MMC normal operation clocking */
  CLOCK_SD_FAST,            /* SD normal operation clocking */
};

/* This structure defines the interface between the NuttX MMC/SD
 * driver and the chip- or board-specific MMC/SD interface.  This
 * interface is only used in architectures that support MMC/SD
 * 1- or 4-bit data busses.  This interface is registered with
 * the NuttX MMC/SD driver by calling mmcsd_slotinitialize() as
 * described below.
 *
 * Architectures that use an SPI interface to the MMC/SD slot
 * should support, instead, the struct spi_dev_s interface (see
 * include/nuttx/spi.h and the mmcsd_spislotinitialize() interface
 * below.
 */

struct mmcsd_dev_s
{
  /* See descriptions of each method in the access macros provided
   * above.
   */

  /* Initialization/setup */

  void  (*reset)(FAR struct mmcsd_dev_s *dev);
  ubyte (*status)(FAR struct mmcsd_dev_s *dev);
  void  (*widebus)(FAR struct mmcsd_dev_s *dev, boolean enable);
  void  (*clock)(FAR struct mmcsd_dev_s *dev, enum mmcsd_clock_e rate);
  int   (*setblocklen)(FAR struct mmcsd_dev_s *dev, int blocklen, int nblocks);
  int   (*attach)(FAR struct mmcsd_dev_s *dev);

  /* Command/Status/Data Transfer */

  void  (*sendcmd)(FAR struct mmcsd_dev_s *dev, ubyte cmd, uint32 arg, FAR const ubyte *data);
  int   (*senddata)(FAR struct mmcsd_dev_s *dev, FAR const ubyte *buffer);

  int   (*recvR1)(FAR struct mmcsd_dev_s *dev, uint16 buffer[3]);
  int   (*recvR2)(FAR struct mmcsd_dev_s *dev, uint16 buffer[8]);
  int   (*recvR3)(FAR struct mmcsd_dev_s *dev, uint16 buffer[3]);
  int   (*recvR4)(FAR struct mmcsd_dev_s *dev, uint16 buffer[3]);
  int   (*recvR5)(FAR struct mmcsd_dev_s *dev, uint16 buffer[3]);
  int   (*recvR6)(FAR struct mmcsd_dev_s *dev, uint16 buffer[3]);
  int   (*recvdata)(FAR struct mmcsd_dev_s *dev, FAR ubyte *buffer);

  /* EVENT handler */

  void  (*eventenable)(FAR struct mmcsd_dev_s *dev, ubyte eventset, boolean enable);
  ubyte (*eventwait)(FAR struct mmcsd_dev_s *dev, uint32 timeout);
  ubyte (*events)(FAR struct mmcsd_dev_s *dev);

  /* DMA */

#ifdef CONFIG_MMCSD_DMA
  boolean (*dmasupported)(FAR struct mmcsd_dev_s *dev);
#ifdef CONFIG_DATA_CACHE
  void  (*coherent)(FAR struct mmcsd_dev_s *dev, FAR void *addr, size_t len, boolean write);
#endif
  int   (*dmareadsetup)(FAR struct mmcsd_dev_s *dev, FAR ubyte *buffer);
  int   (*dmawritesetup)(FAR struct mmcsd_dev_s *dev, FAR const ubyte *buffer);
  int   (*dmaenable)(FAR struct mmcsd_dev_s *dev);
  int   (*dmastop)(FAR struct mmcsd_dev_s *dev);
  int   (*dmastatus)(FAR struct mmcsd_dev_s *dev, size_t *remaining);
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

EXTERN int mmcsd_slotinitialize(int minor, int slotno, FAR struct mmcsd_dev_s *dev);

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called
 *     up_spiinitialize() with the appropriate port number (see spi.h)
 *
 ****************************************************************************/

EXTERN int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_MMCSD_H */
