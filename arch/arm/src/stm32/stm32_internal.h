/************************************************************************************
 * arch/arm/src/stm32/stm32_internal.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H
#define __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* NVIC priority levels */

#define NVIC_SYSH_PRIORITY_MIN     0xff /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

/* Bit-encoded input to stm32_configgpio() *******************************************/

/* Encoding:
 * FFFn nPPP IIIn nnnn nnnn nnnn VPPP BBBB
 *
 * These bits set the primary function of the pin:
 * FFFn nnnn nnnn nnnn nnnn nnnn nnnn nnnn
 */

#define GPIO_FUNC_SHIFT               29                         /* Bit 31-29: GPIO function */
#define GPIO_FUNC_MASK                (7 << GPIO_FUNC_SHIFT)
#define GPIO_FUNC_INFLOAT             (0 << GPIO_FUNC_SHIFT)     /* Input floating */
#define GPIO_FUNC_INPULLUP            (1 << GPIO_FUNC_SHIFT)     /* Input pull-up */
#define GPIO_FUNC_INPULLDWN           (2 << GPIO_FUNC_SHIFT)     /* Input pull-down */
#define GPIO_FUNC_ANALOGIN            (3 << GPIO_FUNC_SHIFT)     /* Analog input */
#define GPIO_FUNC_OUTOD               (4 << GPIO_FUNC_SHIFT)     /* Output open-drain */
#define GPIO_FUNC_OUTPP               (5 << GPIO_FUNC_SHIFT)     /* Output push-pull */
#define GPIO_FUNC_AFPP                (6 << GPIO_FUNC_SHIFT)     /* Altnernate function push-pull */
#define GPIO_FUNC_AFOD                (7 << GPIO_FUNC_SHIFT)     /* Altnernate function open-drain */

/* If the pin is an GPIO digital output, then this identifies the initial output value:
 * nnnn nnnn nnnn nnnn nnnn nnnn Vnnn nnnn
 */

#define GPIO_VALUE_SHIFT              7                          /* Bit 7: If output, inital value of output */
#define GPIO_VALUE_MASK               (1 << GPIO_VALUE_SHIFT)
#define GPIO_VALUE_ZERO               (0 << GPIO_VALUE_SHIFT)    /*   Initial value is zero */
#define GPIO_VALUE_ONE                (1 << GPIO_VALUE_SHIFT)    /*   Initial value is one */

/* This identifies the GPIO port:
 * nnnn nnnn nnnn nnnn nnnn nnnn nPPP nnnn
 */

#define GPIO_PORT_SHIFT               4                          /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#define GPIO_PORTA                    (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#define GPIO_PORTB                    (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#define GPIO_PORTC                    (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#define GPIO_PORTD                    (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#define GPIO_PORTE                    (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#define GPIO_PORTF                    (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#define GPIO_PORTG                    (6 << GPIO_PORT_SHIFT)     /*   GPIOG */

/* This identifies the bit in the port:
 * nnnn nnnn nnnn nnnn nnnn nnnn nnnn BBBB
 */

#define GPIO_NUMBER_SHIFT             0                           /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_NUMBER_MASK              (15 << GPIO_NUMBER_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization.
 *
 ****************************************************************************/

EXTERN void up_lowsetup(void);

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to change to new clock based on desired rcc and rcc2 settings.
 *   This is use to set up the initial clocking but can be used later to
 *   support slow clocked, low power consumption modes.
 *
 ****************************************************************************/

EXTERN void stm32_clockconfig(uint32 newrcc, uint32 newrcc2);

/****************************************************************************
 * Name: up_clockconfig
 *
 * Description:
 *   Called early in the bootsequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

EXTERN void up_clockconfig(void);

/****************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

EXTERN int stm32_configgpio(uint32 cfgset);

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

EXTERN void stm32_gpiowrite(uint32 pinset, boolean value);

/****************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

EXTERN boolean stm32_gpioread(uint32 pinset, boolean value);

/****************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

EXTERN int stm32_dumpgpio(uint32 pinset, const char *msg);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

EXTERN int weak_function gpio_irqinitialize(void);

/****************************************************************************
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
 ****************************************************************************/

#if STM32_NTHERNET > 1
EXTERN int stm32_ethinitialize(int intf);
#endif

/****************************************************************************
 * The external functions, stm32_spi1/2select and stm32_spi1/2status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi.h). All other methods (including up_spiinitialize())
 * are provided by common STM32 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2select() and stm32_spi1/2status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;
enum spi_dev_e;
EXTERN void  stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN void  stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H */
