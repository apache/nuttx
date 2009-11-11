/****************************************************************************
 * arch/arm/src/stm32/stm32_sdio.c
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
#include <sys/types.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_internal.h"
#include "stm32_sdio.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDIO interface */

struct stm32_dev_s
{
  struct sdio_dev_s dev; /* Standard, base MMC/SD interface */
  
  /* STM32-specific extensions */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void  stm32_reset(FAR struct sdio_dev_s *dev);
static ubyte stm32_status(FAR struct sdio_dev_s *dev);
static void  stm32_widebus(FAR struct sdio_dev_s *dev, boolean enable);
static void  stm32_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int   stm32_setblocklen(FAR struct sdio_dev_s *dev, int blocklen,
              int nblocks);
static int   stm32_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static void  stm32_sendcmd(FAR struct sdio_dev_s *dev, ubyte cmd,
               uint32 arg, FAR const ubyte *data);
static int   stm32_senddata(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer);

static int   stm32_recvR1(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
static int   stm32_recvR2(FAR struct sdio_dev_s *dev, uint16 buffer[8]);
static int   stm32_recvR3(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
static int   stm32_recvR4(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
static int   stm32_recvR5(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
static int   stm32_recvR6(FAR struct sdio_dev_s *dev, uint16 buffer[3]);
static int   stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer);

/* EVENT handler */

static void  stm32_eventenable(FAR struct sdio_dev_s *dev, ubyte eventset,
               boolean enable);
static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout);
static ubyte stm32_events(FAR struct sdio_dev_s *dev);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev);
#ifdef CONFIG_DATA_CACHE
static void  stm32_coherent(FAR struct sdio_dev_s *dev, FAR void *addr,
               size_t len, boolean write);
#endif
static int   stm32_dmareadsetup(FAR struct sdio_dev_s *dev,
               FAR ubyte *buffer);
static int   stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
               FAR const ubyte *buffer);
static int   stm32_dmaenable(FAR struct sdio_dev_s *dev);
static int   stm32_dmastart(FAR struct sdio_dev_s *dev);
static int   stm32_dmastop(FAR struct sdio_dev_s *dev);
static int   stm32_dmastatus(FAR struct sdio_dev_s *dev,
               size_t *remaining);
#endif

/* Initialization/uninitialization/reset ************************************/

static void  stm32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_dev_s g_mmcsd =
{
  .dev =
  {
    .reset         = stm32_reset,
    .status        = stm32_status,
    .widebus       = stm32_widebus,
    .clock         = stm32_clock,
    .setblocklen   = stm32_setblocklen,
    .attach        = stm32_attach,
    .sendcmd       = stm32_sendcmd,
    .senddata      = stm32_senddata,
    .recvR1        = stm32_recvR1,
    .recvR2        = stm32_recvR2,
    .recvR3        = stm32_recvR3,
    .recvR4        = stm32_recvR4,
    .recvR5        = stm32_recvR5,
    .recvR6        = stm32_recvR6,
    .recvdata      = stm32_recvdata,
    .eventenable   = stm32_eventenable,
    .eventwait     = stm32_eventwait,
    .events        = stm32_events,
#ifdef CONFIG_SDIO_DMA
    .dmasupported  = stm32_dmasupported,
#ifdef CONFIG_DATA_CACHE
    .coherent      = stm32_coherent,
#endif
    .dmareadsetup  = stm32_dmareadsetup,
    .dmawritesetup = stm32_dmawritesetup,
    .dmaenable     = stm32_dmaenable,
    .dmastart      = stm32_dmastart,
    .dmastop       = stm32_dmastop,
    .dmastatus     = stm32_dmastatus,
#endif
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_reset
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

static void stm32_reset(FAR struct sdio_dev_s *dev)
{
}

/****************************************************************************
 * Name: stm32_status
 *
 * Description:
 *   Get MMC/SD status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see stm32_status_* defines)
 *
 ****************************************************************************/

static ubyte stm32_status(FAR struct sdio_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: SDIO_WIDEBUS
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the MMC/SD device interface
 *   wide - TRUE: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_widebus(FAR struct sdio_dev_s *dev, boolean wide)
{
}

/****************************************************************************
 * Name: stm32_clock
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

static void stm32_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
}

/****************************************************************************
 * Name: stm32_setblocklen
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

static int stm32_setblocklen(FAR struct sdio_dev_s *dev, int blocklen, int nblocks)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_attach
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

static int stm32_attach(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_sendcmd
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

static void stm32_sendcmd(FAR struct sdio_dev_s *dev, ubyte cmd,
                          uint32 arg, FAR const ubyte *data)
{
}

/****************************************************************************
 * Name: stm32_senddata
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

static int stm32_senddata(FAR struct sdio_dev_s *dev, FAR const ubyte *buffer)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_recvRx
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

static int stm32_recvR1(FAR struct sdio_dev_s *dev, uint16 buffer[3])
{
  return -ENOSYS;
}

static int stm32_recvR2(FAR struct sdio_dev_s *dev, uint16 buffer[8])
{
  return -ENOSYS;
}

static int stm32_recvR3(FAR struct sdio_dev_s *dev, uint16 buffer[3])
{
  return -ENOSYS;
}

static int stm32_recvR4(FAR struct sdio_dev_s *dev, uint16 buffer[3])
{
  return -ENOSYS;
}

static int stm32_recvR5(FAR struct sdio_dev_s *dev, uint16 buffer[3])
{
  return -ENOSYS;
}

static int stm32_recvR6(FAR struct sdio_dev_s *dev, uint16 buffer[3])
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_recvdata
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

static int stm32_recvdata(FAR struct sdio_dev_s *dev, FAR ubyte *buffer)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_eventenable
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

static void stm32_eventenable(FAR struct sdio_dev_s *dev, ubyte eventset,
                              boolean enable)
{
}

/****************************************************************************
 * Name: stm32_eventwait
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

static ubyte stm32_eventwait(FAR struct sdio_dev_s *dev, uint32 timeout)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_events
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

static ubyte stm32_events(FAR struct sdio_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_dmasupported
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
static boolean stm32_dmasupported(FAR struct sdio_dev_s *dev)
{
  return FALSE;
}
#endif

/****************************************************************************
 * Name: stm32_coherent
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
static void stm32_coherent(FAR struct sdio_dev_s *dev, FAR void *addr,
                           size_t len, boolean write)
{
}
#endif

/****************************************************************************
 * Name: stm32_dmareadsetup
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
static int  tm32_dmareadsetup(FAR struct sdio_dev_s *dev, FAR ubyte *buffer)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmawritesetup
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
static int stm32_dmawritesetup(FAR struct sdio_dev_s *dev,
                               FAR const ubyte *buffer)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmastart
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
static int stm32_dmastart(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmastop
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
static int stm32_dmastop(FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: stm32_dmastatus
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
static int stm32_dmastatus(FAR struct sdio_dev_s *dev, size_t *remaining)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_default
 *
 * Description:
 *   Restore SDIO registers to their default, reset values
 *
 ****************************************************************************/

static void stm32_default(void)
{
  putreg32(SDIO_POWER_RESET,  STM32_SDIO_POWER);
  putreg32(SDIO_CLKCR_RESET,  STM32_SDIO_CLKCR);
  putreg32(SDIO_ARG_RESET,    STM32_SDIO_ARG);
  putreg32(SDIO_CMD_RESET,    STM32_SDIO_CMD);
  putreg32(SDIO_DTIMER_RESET, STM32_SDIO_DTIMER);
  putreg32(SDIO_DLEN_RESET,   STM32_SDIO_DLEN);
  putreg32(SDIO_DCTRL_RESET,  STM32_SDIO_DCTRL);
  putreg32(SDIO_ICR_RESET,    STM32_SDIO_ICR);
  putreg32(SDIO_MASK_RESET,   STM32_SDIO_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int mmcsd_slotinitialize(int minor, int slotno, FAR struct sdio_dev_s *dev)
{
  return -ENOSYS;
}

