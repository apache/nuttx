/************************************************************************************
 * arch/arm/src/kinetis/kinetis_internal.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_INTERNAL_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "kinetis_config.h"
#include "chip.h"
#include "kinetis_port.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Bit-encoded input to kinetis_configgpio() ****************************************/
/* General form (32-bits, only 20 bits are unused in the encoding):
 *
 * oooo mmmf iiii ---- ---- -ppp ---b bbbb
 */

/* Bits 25-31: 7 bits are used to encode the basic pin configuration:
 *
 * oooo mmm- ---- ---- ---- ---- ---- ----
 * oooommm:
 * |   `--- mmm: mode
 * `------- oooo: options (may be combined)
 */

#define _GPIO_MODE_SHIFT        (25) /* Bits 25-27: Pin mode */
#define _GPIO_MODE_MASK         (7 << _GPIO_MODE_SHIFT)
#define _GPIO_OPTIONS_SHIFT     (28) /* Bits 28-31: Pin mode options */
#define _GPIO_OPTIONS_MASK      (15 << _GPIO_OPTIONS_SHIFT)

#define _GPIO_MODE_ANALOG       (0)  /* 000 Pin Disabled (Analog) */
#define _GPIO_MODE_GPIO         (1)  /* 001 Alternative 1 (GPIO) */
#define _GPIO_MODE_ALT2         (2)  /* 010 Alternative 2 */
#define _GPIO_MODE_ALT3         (3)  /* 011 Alternative 3 */
#define _GPIO_MODE_ALT4         (4)  /* 100 Alternative 4 */
#define _GPIO_MODE_ALT5         (5)  /* 101 Alternative 5 */
#define _GPIO_MODE_ALT6         (6)  /* 110 Alternative 6 */
#define _GPIO_MODE_ALT7         (7)  /* 111 Alternative 7 */

#define _GPIO_IO_MASK           (1)  /* xxx1 GPIO input/output mask */
#define _GPIO_INPUT             (0)  /* xxx0 GPIO input */
#define _GPIO_INPUT_PULLMASK    (6)  /* x11x Mask for pull-up or -down bits */
#define _GPIO_INPUT_PULLENABLE  (2)  /* x010 Enables pull-up or -down */
#define _GPIO_INPUT_PULLDOWN    (2)  /* x010 Input with internal pull-down resistor */
#define _GPIO_INPUT_PULLUP      (6)  /* x110 Input with internal pull-up resistor */
#define _GPIO_INPUT_FILTER_MASK (8)  /* 1xxx Mask to test if passive filter enabled */
#define _GPIO_INPUT_FILTER      (8)  /* 1xx0 Input with passive filter enabled */

#define _GPIO_OUTPUT            (1)  /* xxx1 GPIO output */
#define _GPIO_OUTPUT_SLEW_MASK  (2)  /* xx1x Mask to test for slow slew rate */
#define _GPIO_OUTPUT_FAST       (1)  /* xx01 Output with fast slew rate */
#define _GPIO_OUTPUT_SLOW       (3)  /* xx11 Output with slow slew rate */
#define _GPIO_OUTPUT_OD_MASK    (4)  /* x1xx Mask to test for open drain */
#define _GPIO_OUTPUT_OPENDRAIN  (5)  /* x1x1 Output with open drain enabled */
#define _GPIO_OUTPUT_DRIVE_MASK (4)  /* 1xxx Mask to test for high drive strengh */
#define _GPIO_OUTPUT_LOWDRIVE   (1)  /* 0xx1 Output with low drive strength */
#define _GPIO_OUTPUT_HIGHDRIVE  (9)  /* 1xx1 Output with high drive strength */

#define GPIO_ANALOG             (_GPIO_MODE_ANALOG       << _GPIO_MODE_SHIFT)

#define GPIO_INPUT              ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_INPUT            << _GPIO_OPTIONS_SHIFT))
#define GPIO_PULLDOWN           ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_INPUT_PULLDOWN   << _GPIO_OPTIONS_SHIFT))
#define GPIO_PULLUP             ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_INPUT_PULLUP     << _GPIO_OPTIONS_SHIFT))
#define GPIO_FILTER             ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_INPUT_FILTER     << _GPIO_OPTIONS_SHIFT))

#define GPIO_OUTPUT             ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT           << _GPIO_OPTIONS_SHIFT))
#define GPIO_FAST               ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT_FAST      << _GPIO_OPTIONS_SHIFT))
#define GPIO_SLOW               ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT_SLOW      << _GPIO_OPTIONS_SHIFT))
#define GPIO_OPENDRAIN          ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT_LOWDRIVE  << _GPIO_OPTIONS_SHIFT))
#define GPIO_LOWDRIVE           ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT_OPENDRAIN << _GPIO_OPTIONS_SHIFT))
#define GPIO_HIGHDRIVE          ((_GPIO_MODE_GPIO        << _GPIO_MODE_SHIFT) | \
                                 (_GPIO_OUTPUT_HIGHDRIVE << _GPIO_OPTIONS_SHIFT))

#define GPIO_ALT2               (_GPIO_MODE_ALT2         << _GPIO_MODE_SHIFT)
#define GPIO_ALT3               (_GPIO_MODE_ALT3         << _GPIO_MODE_SHIFT)
#define GPIO_ALT4               (_GPIO_MODE_ALT4         << _GPIO_MODE_SHIFT)
#define GPIO_ALT5               (_GPIO_MODE_ALT5         << _GPIO_MODE_SHIFT)
#define GPIO_ALT6               (_GPIO_MODE_ALT6         << _GPIO_MODE_SHIFT)
#define GPIO_ALT7               (_GPIO_MODE_ALT7         << _GPIO_MODE_SHIFT)

/* One bit is used to enable the digital filter:
 *
 * ---- ---f ---- ---- ---- ---- ---- ----
 */

#define GPIO_DIGFILTER          (1 << 24)  /* Bit 24: Enable digital filter */

/* Four bits are used to incode DMA/interupt options:
 *
 * ---- ---- iiii ---- ---- ---- ---- ----
 */

#define _GPIO_INT_SHIFT         (20)
#define _GPIO_INT_MASK          (15 << _GPIO_MODE_SHIFT)

#define _GPIO_INTDMA_MASK       (1)  /* xxx1 DMA/interrupt mask */
#define _GPIO_DMA               (0)  /* xxx0 DMA (vs interrupt) */
#define _GPIO_DMA_EDGE_MASK     (6)  /* x11x Mask to test edge */
#define _GPIO_DMA_RISING        (2)  /* x010 DMA Request on rising edge */
#define _GPIO_DMA_FALLING       (4)  /* x100 DMA Request on falling edge */
#define _GPIO_DMA_BOTH          (6)  /* x110 DMA Request on either edge */

#define _GPIO_INTERRUPT         (1)  /* xxx1 Interrupt (vs DMA) */
#define _GPIO_INT_ZERO          (1)  /* 0001 Interrupt when logic zero */
#define _GPIO_INT_RISING        (3)  /* 0011 Interrupt on rising edge */
#define _GPIO_INT_FALLING       (5)  /* 0101 Interrupt on falling edge */
#define _GPIO_INT_BOTH          (7)  /* 0111 Interrupt on either edge */
#define _GPIO_INT_ONE           (9)  /* 1001 Interrupt when logic one */

#define GPIO_DMA_RISING         (_GPIO_DMA_RISING  << _GPIO_MODE_SHIFT)
#define GPIO_DMA_FALLING        (_GPIO_DMA_FALLING << _GPIO_MODE_SHIFT)
#define GPIO_DMA_BOTH           (_GPIO_DMA_BOTH    << _GPIO_MODE_SHIFT)
#define GPIO_INT_ZERO           (_GPIO_INT_ZERO    << _GPIO_MODE_SHIFT)
#define GPIO_INT_RISING         (_GPIO_INT_RISING  << _GPIO_MODE_SHIFT)
#define GPIO_INT_FALLING        (_GPIO_INT_FALLING << _GPIO_MODE_SHIFT)
#define GPIO_INT_BOTH           (_GPIO_INT_BOTH    << _GPIO_MODE_SHIFT)
#define GPIO_INT_ONE            (_GPIO_INT_ONE     << _GPIO_MODE_SHIFT)

/* Three bits are used to define the port number:
 *
 * oooo mmmf iiii ---- ---- -ppp ---b bbbb
 */

#define _GPIO_PORT_SHIFT        (8)  /* Bits 8-10: port number */
#define _GPIO_PORT_MASK         (7 << _GPIO_PORT_SHIFT)

#define GPIO_PORTA              (KINETIS_PORTA << _GPIO_PORT_SHIFT)
#define GPIO_PORTB              (KINETIS_PORTB << _GPIO_PORT_SHIFT)
#define GPIO_PORTC              (KINETIS_PORTC << _GPIO_PORT_SHIFT)
#define GPIO_PORTD              (KINETIS_PORTD << _GPIO_PORT_SHIFT)
#define GPIO_PORTE              (KINETIS_PORTE << _GPIO_PORT_SHIFT)


/* Five bits are used to define the pin number:
 *
 * oooo mmmf iiii ---- ---- -ppp ---b bbbb
 */

#define _GPIO_PIN_SHIFT         (0)  /* Bits 0-4: port number */
#define _GPIO_PIN_MASK          (31 << _GPIO_PIN_SHIFT)

#define GPIO_PIN(n)             ((n) << _GPIO_PIN_SHIFT)
#define GPIO_PIN0               (0 << _GPIO_PIN_SHIFT)
#define GPIO_PIN1               (1 << _GPIO_PIN_SHIFT)
#define GPIO_PIN2               (2 << _GPIO_PIN_SHIFT)
#define GPIO_PIN3               (3 << _GPIO_PIN_SHIFT)
#define GPIO_PIN4               (4 << _GPIO_PIN_SHIFT)
#define GPIO_PIN5               (5 << _GPIO_PIN_SHIFT)
#define GPIO_PIN6               (6 << _GPIO_PIN_SHIFT)
#define GPIO_PIN7               (7 << _GPIO_PIN_SHIFT)
#define GPIO_PIN8               (8 << _GPIO_PIN_SHIFT)
#define GPIO_PIN9               (9 << _GPIO_PIN_SHIFT)
#define GPIO_PIN10              (10 << _GPIO_PIN_SHIFT)
#define GPIO_PIN11              (11 << _GPIO_PIN_SHIFT)
#define GPIO_PIN12              (12 << _GPIO_PIN_SHIFT)
#define GPIO_PIN13              (13 << _GPIO_PIN_SHIFT)
#define GPIO_PIN14              (14 << _GPIO_PIN_SHIFT)
#define GPIO_PIN15              (15 << _GPIO_PIN_SHIFT)
#define GPIO_PIN16              (16 << _GPIO_PIN_SHIFT)
#define GPIO_PIN17              (17 << _GPIO_PIN_SHIFT)
#define GPIO_PIN18              (18 << _GPIO_PIN_SHIFT)
#define GPIO_PIN19              (19 << _GPIO_PIN_SHIFT)
#define GPIO_PIN20              (20 << _GPIO_PIN_SHIFT)
#define GPIO_PIN21              (21 << _GPIO_PIN_SHIFT)
#define GPIO_PIN22              (22 << _GPIO_PIN_SHIFT)
#define GPIO_PIN23              (23 << _GPIO_PIN_SHIFT)
#define GPIO_PIN24              (24 << _GPIO_PIN_SHIFT)
#define GPIO_PIN25              (25 << _GPIO_PIN_SHIFT)
#define GPIO_PIN26              (26 << _GPIO_PIN_SHIFT)
#define GPIO_PIN27              (27 << _GPIO_PIN_SHIFT)
#define GPIO_PIN28              (28 << _GPIO_PIN_SHIFT)
#define GPIO_PIN29              (29 << _GPIO_PIN_SHIFT)
#define GPIO_PIN30              (30 << _GPIO_PIN_SHIFT)
#define GPIO_PIN31              (31 << _GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct kinetis_dmaglobalregs_s
{
#warning "Missing logic"
  /* Global Registers */
};

struct kinetis_dmachanregs_s
{
#warning "Missing logic"
  /* Channel Registers */
};

struct kinetis_dmaregs_s
{
  /* Global Registers */

  struct kinetis_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct kinetis_dmachanregs_s   ch;
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
 * Name: kinetis_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is needed to
 *   put the  MCU in a usable state.  This includes the initialization of clocking
 *   using the settings in board.h.
 *
 ************************************************************************************/

EXTERN void kinetis_clockconfig(void);

/************************************************************************************
 * Name: kinetis_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

EXTERN void kinetis_lowsetup(void);

/******************************************************************************
 * Name: kinetis_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
EXTERN void kinetis_uartreset(uintptr_t uart_base);
#endif

/******************************************************************************
 * Name: kinetis_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
EXTERN void kinetis_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                                  unsigned int parity, unsigned int nbits,
                                  bool stop2);
#endif

/************************************************************************************
 * Name: kinetis_wddisable
 *
 * Description:
 *   Disable the watchdog timer
 *
 ************************************************************************************/

EXTERN void kinetis_wddisable(void);

/************************************************************************************
 * Name: kinetis_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void kinetis_gpioirqinitialize(void);
#else
#  define kinetis_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: kinetis_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int kinetis_configgpio(uint32_t cfgset);

/************************************************************************************
 * Name: kinetis_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void kinetis_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: kinetis_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool kinetis_gpioread(uint32_t pinset);

/************************************************************************************
 * Name: kinetis_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void kinetis_gpioirqenable(int irq);
#else
#  define kinetis_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: kinetis_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void kinetis_gpioirqdisable(int irq);
#else
#  define kinetis_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  kinetis_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int kinetis_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define kinetis_dumpgpio(p,m)
#endif

/************************************************************************************
 * Name: kinetis_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be required
 *   for most interrupts.
 *
 ************************************************************************************/

EXTERN void kinetis_clrpend(int irq);

/************************************************************************************
 * Name:  kinetis_spi/ssp0/ssp1select, kinetis_spi/ssp0/ssp1status, and
 *        kinetis_spi/ssp0/ssp1cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common Kinetis logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide kinetis_spi/ssp0/ssp1select() and kinetis_spi/ssp0/ssp1status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kinetis_spi/ssp0/ssp1cmddata() functions in your board-specific logic.  These
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

#ifdef CONFIG_KINETIS_SPI
EXTERN void  kinetis_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t kinetis_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int kinetis_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif
#ifdef CONFIG_KINETIS_SSP0
EXTERN void  kinetis_ssp0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t kinetis_ssp0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int kinetis_ssp0cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif
#ifdef CONFIG_KINETIS_SSP1
EXTERN void  kinetis_ssp1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t kinetis_ssp1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int kinetis_ssp1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
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
#ifdef CONFIG_KINETIS_SPI
EXTERN void spi_flush(FAR struct spi_dev_s *dev);
#endif
#if defined(CONFIG_KINETIS_SSP0) || defined(CONFIG_KINETIS_SSP1)
EXTERN void ssp_flush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
EXTERN void kinetis_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: kinetis_dmachannel
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

#ifdef CONFIG_KINETIS_GPDMA
EXTERN DMA_HANDLE kinetis_dmachannel(void);
#endif

/****************************************************************************
 * Name: kinetis_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kinetis_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
EXTERN void kinetis_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
EXTERN int kinetis_dmarxsetup(DMA_HANDLE handle,
                            uint32_t control, uint32_t config,
                            uint32_t srcaddr, uint32_t destaddr,
                            size_t nbytes);
#endif

/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
EXTERN int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
EXTERN void kinetis_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs);
#else
#  define kinetis_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: kinetis_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void kinetis_dmadump(DMA_HANDLE handle, const struct kinetis_dmaregs_s *regs,
                          const char *msg);
#else
#  define kinetis_dmadump(handle,regs,msg)
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_INTERNAL_H */
