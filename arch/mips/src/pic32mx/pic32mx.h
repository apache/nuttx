/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx.h
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "chip.h"
#include "pic32mx-config.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GPIO settings used in the configport, readport, writeport, etc.
 *
 * General encoding:
 * MMAV IIDx RRRx PPPP
 */

#define GPIO_MODE_SHIFT   (14)      /* Bits 14-15: I/O mode */
#define GPIO_MODE_MASK    (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT      (0 << GPIO_MODE_SHIFT) /* 00 Normal input */
#  define GPIO_OUTPUT     (2 << GPIO_MODE_SHIFT) /* 10 Normal output */
#  define GPIO_OPENDRAN   (3 << GPIO_MODE_SHIFT) /* 11 Open drain output */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#define GPIO_ANALOG_MASK   (1 << 13) /* Bit 13: Analog */
#  define GPIO_ANALOG      (1 << 13)
#  define GPIO_DIGITAL     (0)
#endif

#define GPIO_VALUE_MASK   (1 << 12) /* Bit 12: Initial output value */
#  define GPIO_VALUE_ONE  (1 << 12)
#  define GPIO_VALUE_ZERO (0)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define GPIO_PULLUP     (1 << 11) /* Bit 11: Change notification pull-up */
#endif

#define GPIO_INT_SHIFT    (10)      /* Bits 10-11: Interrupt mode */
#define GPIO_INT_MASK     (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_NONE   (0 << GPIO_INT_SHIFT) /* Bit 00: No interrupt */
#  define GPIO_INT        (1 << GPIO_INT_SHIFT) /* Bit 01: Change notification enable */
#  define GPIO_PUINT      (3 << GPIO_INT_SHIFT) /* Bit 11: Pulled-up interrupt input */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define GPIO_PULLDOWN   (1 << 9) /* Bit 11: Change notification pull-down */
#endif

#define GPIO_PORT_SHIFT   (5)       /* Bits 5-7: Port number */
#define GPIO_PORT_MASK    (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA      (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB      (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC      (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD      (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE      (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORTF      (5 << GPIO_PORT_SHIFT)
#  define GPIO_PORTG      (6 << GPIO_PORT_SHIFT)

#define GPIO_PIN_SHIFT    0        /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK     (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN0         (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1         (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2         (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3         (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4         (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5         (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6         (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7         (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8         (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9         (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10        (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11        (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12        (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13        (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14        (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15        (15 << GPIO_PIN_SHIFT)

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
extern "C"
{
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

void pic32mx_lowinit(void);

/************************************************************************************
 * Name: pic32mx_consoleinit
 *
 * Description:
 *   Performs low level initialization of the console UART.  This UART done early so
 *   that the serial console is available for debugging very early in the boot
 *   sequence.
 *
 ************************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void pic32mx_consoleinit(void);
#else
#  define pic32mx_consoleinit()
#endif

/****************************************************************************
 * Name: pic32mx_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: pic32mx_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                           unsigned int parity, unsigned int nbits, bool stop2);
#endif

/************************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the  directory
 *   configs/<board-name>/pic32_boot.c.
 *
 ************************************************************************************/

void pic32mx_boardinitialize(void);

/************************************************************************************
 * Name: pic32mx_decodeirq
 *
 * Description:
 *   Called from assembly language logic when an interrupt exception occurs.  This
 *   function decodes and dispatches the interrupt.
 *
 ************************************************************************************/

uint32_t *pic32mx_decodeirq(uint32_t *regs);

/************************************************************************************
 * Name: pic32mx_exception
 *
 * Description:
 *   Called from assembly language logic on all other exceptions.
 *
 ************************************************************************************/

uint32_t *pic32mx_exception(uint32_t *regs);

/************************************************************************************
 * Name: pic32mx_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin (the interrupt
 *   will be configured when pic32mx_attach() is called).
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ************************************************************************************/

int pic32mx_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: pic32mx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void pic32mx_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: pic32mx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool pic32mx_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: pic32mx_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.  This
 *   function is called internally by the system on power up and should not be
 *   called again.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_GPIOIRQ
void pic32mx_gpioirqinitialize(void);
#else
#  define pic32mx_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: pic32mx_gpioattach
 *
 * Description:
 *   Attach an interrupt service routine to a GPIO interrupt.  This will also
 *   reconfigure the pin as an interrupting input.  The change notification number is
 *   associated with all interrupt-capable GPIO pins.  The association could,
 *   however, differ from part to part and must be  provided by the caller.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin.  In that
 *   case, all attached handlers will be called.  Each handler must maintain state
 *   and determine if the underlying GPIO input value changed.
 *
 * Input Parameters:
 *   pinset  - GPIO pin configuration
 *   cn      - The change notification number associated with the pin.
 *   handler - Interrupt handler (may be NULL to detach)
 *   arg     - The argument that accompanies the interrupt
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated error value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_GPIOIRQ
int pic32mx_gpioattach(uint32_t pinset, unsigned int cn, xcpt_t handler,
                       void *arg);
#else
#  define pic32mx_gpioattach(p,c,h,a) (0)
#endif

/************************************************************************************
 * Name: pic32mx_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_GPIOIRQ
void pic32mx_gpioirqenable(unsigned int cn);
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

#ifdef CONFIG_PIC32MX_GPIOIRQ
void pic32mx_gpioirqdisable(unsigned int cn);
#else
#  define pic32mx_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  pic32mx_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void pic32mx_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define pic32mx_dumpgpio(p,m)
#endif

/****************************************************************************
 * Name: pic32mx_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s;  /* Forward reference */
FAR struct spi_dev_s *pic32mx_spibus_initialize(int port);

/************************************************************************************
 * Name:  pic32mx_spiNselect, pic32mx_spiNstatus, and pic32mx_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including pic32mx_spibus_initialize()) are provided by common PIC32MX logic.  To use
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
 *   3. Add a call to pic32mx_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by pic32mx_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_SPI1
void  pic32mx_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t pic32mx_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI2
void  pic32mx_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI3
void  pic32mx_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t pic32mx_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI3
void  pic32mx_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t pic32mx_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/************************************************************************************
 * Name: pic32mx_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmainitilaize(void);
#endif

/************************************************************************************
 * Name: pic32mx_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and gives the
 *  caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel handle.  NULL
 *   is returned on any failure.  This function can fail only if no DMA channel is
 *   available.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
DMA_HANDLE pic32mx_dmachannel(void);
#endif

/************************************************************************************
 * Name: pic32mx_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must NEVER be
 *   used again until pic32mx_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmafree(DMA_HANDLE handle);
#endif

/************************************************************************************
 * Name: pic32mx_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
int pic32mx_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                       uint32_t srcaddr, uint32_t destaddr, size_t nbytes);
#endif

/************************************************************************************
 * Name: pic32mx_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
int pic32mx_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/************************************************************************************
 * Name: pic32mx_dmastop
 *
 * Description:
 *   Cancel the DMA.  After pic32mx_dmastop() is called, the DMA channel is reset
 *   and pic32mx_dmasetup() must be called before pic32mx_dmastart() can be called
 *   again
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
void pic32mx_dmastop(DMA_HANDLE handle);
#endif

/************************************************************************************
 * Name: pic32mx_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
#ifdef CONFIG_DEBUG_DMA
void pic32mx_dmasample(DMA_HANDLE handle, struct pic32mx_dmaregs_s *regs);
#else
#  define pic32mx_dmasample(handle,regs)
#endif
#endif

/************************************************************************************
 * Name: pic32mx_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_DMA
#ifdef CONFIG_DEBUG_DMA
void pic32mx_dmadump(DMA_HANDLE handle, const struct pic32mx_dmaregs_s *regs,
                     const char *msg);
#else
#  define pic32mx_dmadump(handle,regs,msg)
#endif
#endif

/************************************************************************************
 * Name: pic32mx_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide pic32mx_pullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following can be redefined to be
 *   NULL.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV
struct usbdev_s;
int pic32mx_usbpullup(FAR struct usbdev_s *dev,  bool enable);
#endif

/************************************************************************************
 * Name: pic32mx_usbsuspend
 *
 * Description:
 *   Board logic must provide the pic32mx_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc. while
 *   the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV
void pic32mx_usbsuspend(FAR struct usbdev_s *dev, bool resume);
#endif

/************************************************************************************

 * Name: pic32mx_usbattach and pic32mx_usbdetach
 *
 * Description:
 *   The USB stack must be notified when the device is attached or detached by
 *   calling one of these functions.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV
void pic32mx_usbattach(void);
void pic32mx_usbdetach(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_H */
