/************************************************************************************
 * arch/arm/src/stm32/stm32_internal.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H
#define __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"
#include "stm32_gpio.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_VERBOSE)
#  undef CONFIG_DEBUG_DMA
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xff /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */


/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t isr, void *arg);

#ifdef CONFIG_DEBUG_DMA
struct stm32_dmaregs_s
{
  uint32_t isr;
  uint32_t ccr;
  uint32_t cndtr;
  uint32_t cpar;
  uint32_t cmar;
};
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* This symbol references the Cortex-M3 vector table (as positioned by the the linker
 * script, ld.script or ld.script.dfu.  The standard location for the vector table is
 * at the beginning of FLASH at address 0x0800:0000.  If we are using the STMicro DFU
 * bootloader, then the vector table will be offset to a different location in FLASH
 * and we will need to set the NVIC vector location to this alternative location.
 */

extern uint32_t stm32_vectors[];	/* See stm32_vectors.S */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization.
 *
 ************************************************************************************/

EXTERN void stm32_lowsetup(void);

/************************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 ************************************************************************************/

EXTERN void stm32_clockconfig(void);


/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chan' argument.
 *   DMA channels are shared on the STM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32_dma.h.
 *
 *   If the DMA channel is not available, then stm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Returned Value:
 *   Provided that 'chan' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chan' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

EXTERN DMA_HANDLE stm32_dmachannel(int chan);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to stm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

EXTERN void stm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

EXTERN void stm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                           size_t ntransfers, uint32_t ccr);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

EXTERN void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback,
                           void *arg, bool half);

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

EXTERN void stm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
EXTERN void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs);
#else
#  define stm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
EXTERN void stm32_dmadump(DMA_HANDLE handle, const struct stm32_dmaregs_s *regs,
                          const char *msg);
#else
#  define stm32_dmadump(handle,regs,msg)
#endif

/************************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then bould specific logic
 *   must implement up_netinitialize() and call this function to initialize
 *   the desiresed interfaces.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ************************************************************************************/

#if STM32_NTHERNET > 1
EXTERN int stm32_ethinitialize(int intf);
#endif

/************************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select, stm32_spi1/2/3status, and
 *   stm32_spi1/2/3cmddata must be provided by board-specific logic.  These are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods
 *   (including up_spiinitialize()) are provided by common STM32 logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file, then
 *      provide stm32_spi1/2/3cmddata() functions in your board-specific logic. 
 *      These functions will perform cmd/data selection operations using GPIOs in the
 *      way your board is configured.
 *   4. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   5. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;
EXTERN void  stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
EXTERN void  stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
EXTERN void  stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi3cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

/************************************************************************************
 * Name:  stm32_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide stm32_pullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following EXTERN can be redefined to be
 *   NULL.
 *
 ************************************************************************************/

struct usbdev_s;
EXTERN int stm32_usbpullup(FAR struct usbdev_s *dev,  bool enable);

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

struct usbdev_s;
EXTERN void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume);

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See include/nuttx/sdio.h */
EXTERN FAR struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a 
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H */
