/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-internal.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INTERNAL_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"
#include "pic32mx-config.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Bit-encoded input to pic32mx_configgpio() ****************************************/
/* To be provided */

/* GPIO pin definitions *************************************************************/
/* To be provided */

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct pic32mx_dmaglobalregs_s
{
  /* Global Registers */
#warning "Missing definitions"
};

struct pic32mx_dmachanregs_s
{
  /* Channel Registers */
#warning "Missing definitions"
};

struct pic32mx_dmaregs_s
{
  /* Global Registers */

  struct pic32mx_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct pic32mx_dmachanregs_s   ch;
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

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_lowinit
 *
 * Description:
 *   This performs basic low-level initialization of the system.
 *
 ************************************************************************************/

EXTERN void pic32mx_lowinit(void);

/************************************************************************************
 * Name: pic32mx_lowsetup
 *
 * Description:
 *   Performs low level initialization of the console UART.  This UART done early so
 *   that the serial console is available for debugging very early in the boot
 *   sequence.
 *
 ************************************************************************************/

EXTERN void pic32mx_consoleinit(void);

/******************************************************************************
 * Name: pic32mx_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
EXTERN void pic32mx_uartreset(uintptr_t uart_base);
#endif

/******************************************************************************
 * Name: pic32mx_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
EXTERN void pic32mx_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                                  unsigned int parity, unsigned int nbits,
                                  bool stop2);
#endif

/************************************************************************************
 * Name: pic32mx_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void pic32mx_gpioirqinitialize(void);
#else
#  define pic32mx_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the  directory
 *   configs/<board-name>/up_boot.c.
 *
 ************************************************************************************/

EXTERN void pic32mx_boardinitialize(void);

/************************************************************************************
 * Name: pic32mx_decodeirq
 *
 * Description:
 *   Called from assembly language logic when an interrrupt exception occurs.  This
 *   function decodes and dispatches the interrupt.
 *
 ************************************************************************************/

EXTERN uint32_t *pic32mx_decodeirq(uint32_t *regs);

/************************************************************************************
 * Name: pic32mx_dobev
 *
 * Description:
 *   Called from assembly language logic on all other exceptions.
 *
 ************************************************************************************/

EXTERN uint32_t *pic32mx_dobev(uint32_t *regs);

/************************************************************************************
 * Name: pic32mx_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int pic32mx_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: pic32mx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void pic32mx_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: pic32mx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool pic32mx_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: pic32mx_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void pic32mx_gpioirqenable(int irq);
#else
#  define pic32mx_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: pic32mx_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void pic32mx_gpioirqdisable(int irq);
#else
#  define pic32mx_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  pic32mx_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int pic32mx_dumpgpio(uint16_t pinset, const char *msg);
#else
#  define pic32mx_dumpgpio(p,m)
#endif

/************************************************************************************
 * Name:  pic32mx_spiNselect, pic32mx_spiNstatus, and pic32mx_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common PIC32MX logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide pic32mx_spiNselect() and pic32mx_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mx_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;

#ifdef CONFIG_PIC32MX_SPI1
EXTERN void  pic32mx_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t pic32mx_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int pic32mx_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif
#ifdef CONFIG_PIC32MX_SPI2
EXTERN void  pic32mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int pic32mx_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: pic32mx_spiflush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct spi_dev_s;
#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2)
EXTERN void pic32mx_spiflush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: pic32mx_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN void pic32mx_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: pic32mx_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN DMA_HANDLE pic32mx_dmachannel(void);
#endif

/****************************************************************************
 * Name: pic32mx_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until pic32mx_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN void pic32mx_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: pic32mx_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN int pic32mx_dmarxsetup(DMA_HANDLE handle,
                            uint32_t control, uint32_t config,
                            uint32_t srcaddr, uint32_t destaddr,
                            size_t nbytes);
#endif

/****************************************************************************
 * Name: pic32mx_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN int pic32mx_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: pic32mx_dmastop
 *
 * Description:
 *   Cancel the DMA.  After pic32mx_dmastop() is called, the DMA channel is
 *   reset and pic32mx_dmasetup() must be called before pic32mx_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
EXTERN void pic32mx_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: pic32mx_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void pic32mx_dmasample(DMA_HANDLE handle, struct pic32mx_dmaregs_s *regs);
#else
#  define pic32mx_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: pic32mx_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_pic32mx_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void pic32mx_dmadump(DMA_HANDLE handle, const struct pic32mx_dmaregs_s *regs,
                          const char *msg);
#else
#  define pic32mx_dmadump(handle,regs,msg)
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_INTERNAL_H */
