/****************************************************************************
 * include/nuttx/sdio.h
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

#ifndef __NUTTX_SDIO_H
#define __NUTTX_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* MMC/SD events needed by the driver */

#define SDIOEVENT_EJECTED       (1 << 0) /* Bit 0: CD/DAT3 transition low, media removed */
#define SDIOEVENT_INSERTED      (1 << 1) /* Bit 1: CD/DAT3 transition high, media inserted */
#define SDIOEVENT_CMDDONE       (1 << 2) /* Bit 2: Command+response complete */
#define SDIOEVENT_READCMDDONE   (1 << 3) /* Bit 3: Read command done */
#define SDIOEVENT_WRITECMDDONE  (1 << 4) /* Bit 4: Write command done */
#define SDIOEVENT_READDATADONE  (1 << 5) /* Bit 5: Read data done */
#define SDIOEVENT_WRITEDATADONE (1 << 6) /* Bit 6: Write data done */
#define SDIOEVENT_CMDBUSYDONE   (1 << 7) /* Bit 7: Command with transition to not busy */

#define SDIOEVENT_ALLEVENTS     0xff

/****************************************************************************
 * Name: SDIO_RESET
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

#define SDIO_RESET(dev) ((dev)->reset(dev))

/****************************************************************************
 * Name: SDIO_STATUS
 *
 * Description:
 *   Get MMC/SD status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_STATUS_* defines)
 *
 ****************************************************************************/

#define SDIO_STATUS(dev)        ((d)->status(dev))

/* MMC/SD status bits */

#define SDIO_STATUS_PRESENT     0x01 /* Bit 0=1: MMC/SD card present */
#define SDIO_STATUS_WRPROTECTED 0x02 /* Bit 1=1: MMC/SD card write protected */

#define SDIO_PRESENT(dev)       ((SDIO_STATUS(dev) & SDIO_STATUS_PRESENT) != 0)
#define SDIO_WRPROTECTED(dev)   ((SDIO_STATUS(dev) & SDIO_STATUS_WRPROTECTED) != 0)

/****************************************************************************
 * Name: SDIO_WIDEBUS
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

#define SDIO_WIDEBUS(dev,enable) ((dev)->widebus(dev,enable))

/****************************************************************************
 * Name: SDIO_CLOCK
 *
 * Description:
 *   Enable/disable MMC/SD clocking
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_CLOCK(dev,rate) ((dev)->clock(dev,rate))

/****************************************************************************
 * Name: SDIO_SETBLOCKLEN
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

#define SDIO_SETBLOCKLEN(dev,blocklen,nblocks) \
  ((dev)->setblocklen(dev,blocklen,nblocks))

/****************************************************************************
 * Name: SDIO_ATTACH
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

#define SDIO_ATTACH(dev) ((dev)->attach(dev))

/****************************************************************************
 * Name: SDIO_SENDCMD
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

#define SDIO_SENDCMD(dev,cmd,arg,data) ((dev)->sendcmd(dev,cmd,arg,data))

/****************************************************************************
 * Name: SDIO_SENDDATA
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

#define SDIO_SENDDATA(dev,data) ((dev)->senddata(dev,data))

/****************************************************************************
 * Name: SDIO_RECVRx
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

#define SDIO_RECVR1(dev,buffer) ((dev)->recvR1(dev,buffer))
#define SDIO_RECVR2(dev,buffer) ((dev)->recvR2(dev,buffer))
#define SDIO_RECVR3(dev,buffer) ((dev)->recvR3(dev,buffer))
#define SDIO_RECVR4(dev,buffer) ((dev)->recvR4(dev,buffer))
#define SDIO_RECVR5(dev,buffer) ((dev)->recvR5(dev,buffer))
#define SDIO_RECVR6(dev,buffer) ((dev)->recvR6(dev,buffer))

/****************************************************************************
 * Name: SDIO_RECVDATA
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

#define SDIO_RECVDATA(dev,buffer) ((dev)->recvdata(dev,buffer))

/****************************************************************************
 * Name: SDIO_EVENTENABLE
 *
 * Description:
 *   Enable/disable notification of a set of MMC/SD events
 *
 * Input Parameters:
 *   dev      - An instance of the MMC/SD device interface
 *   eventset - A bitset of events to enable or disable (see SDIOEVENT_*
 *              definitions
 *   enable   - TRUE: enable event; FALSE: disable events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_EVENTENABLE(dev,eventset)  ((dev)->eventenable(dev,eventset,TRUE))
#define SDIO_EVENTDISABLE(dev,eventset) ((dev)->eventenable(dev,eventset,FALSE))
#define SDIO_EVENTDISABLEALL(dev)       ((dev)->eventenable(dev,SDIOEVENT_ALLEVENTS,FALSE))

/****************************************************************************
 * Name: SDIO_EVENTWAIT
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

#define SDIO_EVENTWAIT(dev,timeout)  ((dev)->eventwait(dev,timeout))

/****************************************************************************
 * Name: SDIO_EVENTS
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

#define SDIO_EVENTS(dev)  ((dev)->events(dev))

/****************************************************************************
 * Name: SDIO_DMASUPPORTED
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMASUPPORTED(dev) ((dev)->dmasupported(dev))
#else
#  define SDIO_DMASUPPORTED(dev) (FALSE)
#endif

/****************************************************************************
 * Name: SDIO_COHERENT
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

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_DATA_CACHE)
#  define SDIO_COHERENT(dev,addr,len,write) ((dev)->coherent(dev,addr,len,write))
#else
#  define SDIO_COHERENT(dev,addr,len,write)
#endif

/****************************************************************************
 * Name: SDIO_DMAREADSETUP
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMAREADSETUP(dev,buffer) ((dev)->dmareadsetup(dev,buffer))
#else
#  define SDIO_DMAREADSETUP(dev,buffer) (-ENOSYS)
#endif

/****************************************************************************
 * Name: SDIO_DMAWRITESETUP
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMAWRITESETUP(dev,buffer) ((dev)->dmawritesetup(dev,buffer))
#else
#  define SDIO_DMAWRITESETUP(dev,buffer) (-ENOSYS)
#endif

/****************************************************************************
 * Name: SDIO_DMASTART
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMASTART(dev) ((dev)->dmastart(dev))
#else
#  define SDIO_DMASTART(dev) (-ENOSYS)
#endif

/****************************************************************************
 * Name: SDIO_DMASTOP
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMASTOP(dev) ((dev)->dmastop(dev))
#else
#  define SDIO_DMASTOP(dev) (-ENOSYS)
#endif

/****************************************************************************
 * Name: SDIO_DMASTATUS
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

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMASTATUS(dev,remaining) ((dev)->dmastatus(dev,remaining))
#else
#  define SDIO_DMASTATUS(dev,remaining) (-ENOSYS)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Various clocking used by the MMC/SD driver */

enum sdio_clock_e
{
  CLOCK_SDIO_DISABLED = 0, /* Clock is disabled */
  CLOCK_MMC_SLOW,           /* MMC initialization clocking */
  CLOCK_SD_SLOW,            /* SD initialization clocking */
  CLOCK_MMC_FAST,           /* MMC normal operation clocking */
  CLOCK_SD_FAST             /* SD normal operation clocking */
};

/* This structure defines the interface between the NuttX MMC/SD
 * driver and the chip- or board-specific MMC/SD interface.  This
 * interface is only used in architectures that support SDIO
 * 1- or 4-bit data busses.  For MMC/SD support this interface is
 * registered with the NuttX MMC/SD driver by calling
 * sdio_slotinitialize().
 */

struct sdio_dev_s
{
  /* See descriptions of each method in the access macros provided
   * above.
   */

  /* Initialization/setup */

  void  (*reset)(FAR struct sdio_dev_s *dev);
  ubyte (*status)(FAR struct sdio_dev_s *dev);
  void  (*widebus)(FAR struct sdio_dev_s *dev, boolean enable);
  void  (*clock)(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate);
  int   (*setblocklen)(FAR struct sdio_dev_s *dev, int blocklen, int nblocks);
  int   (*attach)(FAR struct sdio_dev_s *dev);

  /* Command/Status/Data Transfer */

  void  (*sendcmd)(FAR struct sdio_dev_s *dev, ubyte cmd, uint32 arg, FAR const ubyte *data);
  int   (*senddata)(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer);

  int   (*recvR1)(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
  int   (*recvR2)(FAR struct sdio_dev_s *dev, uint16 buffer[8]);
  int   (*recvR3)(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
  int   (*recvR4)(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
  int   (*recvR5)(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
  int   (*recvR6)(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
  int   (*recvdata)(FAR struct sdio_dev_s *dev, FAR ubyte *buffer);

  /* EVENT handler */

  void  (*eventenable)(FAR struct sdio_dev_s *dev, ubyte eventset, boolean enable);
  ubyte (*eventwait)(FAR struct sdio_dev_s *dev, uint32 timeout);
  ubyte (*events)(FAR struct sdio_dev_s *dev);

  /* DMA */

#ifdef CONFIG_SDIO_DMA
  boolean (*dmasupported)(FAR struct sdio_dev_s *dev);
#ifdef CONFIG_DATA_CACHE
  void  (*coherent)(FAR struct sdio_dev_s *dev, FAR void *addr, size_t len, boolean write);
#endif
  int   (*dmareadsetup)(FAR struct sdio_dev_s *dev, FAR ubyte *buffer);
  int   (*dmawritesetup)(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer);
  int   (*dmaenable)(FAR struct sdio_dev_s *dev);
  int   (*dmastop)(FAR struct sdio_dev_s *dev);
  int   (*dmastatus)(FAR struct sdio_dev_s *dev, size_t *remaining);
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_SDIO_H */
