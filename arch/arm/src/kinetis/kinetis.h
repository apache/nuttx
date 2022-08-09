/****************************************************************************
 * arch/arm/src/kinetis/kinetis.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "kinetis_config.h"
#include "chip.h"
#include "hardware/kinetis_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Bit-encoded input to kinetis_pinconfig() *********************************/

/* General form (32-bits, only 22 bits are unused in the encoding):
 *
 * oooo mmmv iiii ifd- ---- -ppp ---b bbbb
 */

/* Bits 25-31: 7 bits are used to encode the basic pin configuration:
 *
 * oooo mmm- ---- ---- ---- ---- ---- ----
 * |    `--- mmm: mode
 * `------- oooo: options (may be combined)
 */

#define _PIN_MODE_SHIFT        (25)                    /* Bits 25-27: Pin mode */
#define _PIN_MODE_MASK         (7 << _PIN_MODE_SHIFT)
#define _PIN_OPTIONS_SHIFT     (28)                    /* Bits 28-31: Pin mode options */
#define _PIN_OPTIONS_MASK      (15 << _PIN_OPTIONS_SHIFT)

/* Port Modes */

                                                    /* Unshifted versions: */
#define PIN_MODE_ANALOG        (0)                     /*   000 Pin Disabled (Analog) */
#define PIN_MODE_ALT1          (1)                     /*   001 Alternative 1 */
#define PIN_MODE_GPIO          PIN_MODE_ALT1           /*   001 Alternative 1 (GPIO) */
#define PIN_MODE_ALT2          (2)                     /*   010 Alternative 2 */
#define PIN_MODE_ALT3          (3)                     /*   011 Alternative 3 */
#define PIN_MODE_ALT4          (4)                     /*   100 Alternative 4 */
#define PIN_MODE_ALT5          (5)                     /*   101 Alternative 5 */
#define PIN_MODE_ALT6          (6)                     /*   110 Alternative 6 */
#define PIN_MODE_ALT7          (7)                     /*   111 Alternative 7 */

                                                   /* Shifted versions: */
#define _PIN_MODE_ANALOG       (0 << _PIN_MODE_SHIFT)  /*   000 Pin Disabled (Analog) */
#define _PIN_MODE_ALT1         (1 << _PIN_MODE_SHIFT)  /*   001 Alternative 1 */
#define _PIN_MODE_GPIO         (1 << _PIN_MODE_SHIFT)  /*   001 Alternative 1 (GPIO) */
#define _PIN_MODE_ALT2         (2 << _PIN_MODE_SHIFT)  /*   010 Alternative 2 */
#define _PIN_MODE_ALT3         (3 << _PIN_MODE_SHIFT)  /*   011 Alternative 3 */
#define _PIN_MODE_ALT4         (4 << _PIN_MODE_SHIFT)  /*   100 Alternative 4 */
#define _PIN_MODE_ALT5         (5 << _PIN_MODE_SHIFT)  /*   101 Alternative 5 */
#define _PIN_MODE_ALT6         (6 << _PIN_MODE_SHIFT)  /*   110 Alternative 6 */
#define _PIN_MODE_ALT7         (7 << _PIN_MODE_SHIFT)  /*   111 Alternative 7 */

/* Options for all digital modes (Alternatives 1-7).
 * None of the digital options apply if the analog mode is selected.
 */

#define _PIN_IO_MASK           (1 << _PIN_OPTIONS_SHIFT) /* xxx1 Digital input/output mask */
#define _PIN_INPUT             (0 << _PIN_OPTIONS_SHIFT) /* xxx0 Digital input */
#define _PIN_OUTPUT            (1 << _PIN_OPTIONS_SHIFT) /* xxx1 Digital output */

#define _PIN_INPUT_PULLMASK    (7 << _PIN_OPTIONS_SHIFT) /* x111 Mask for pull-up or -down bits */
#define _PIN_INPUT_PULLDOWN    (2 << _PIN_OPTIONS_SHIFT) /* x010 Input with internal pull-down resistor */
#define _PIN_INPUT_PULLUP      (6 << _PIN_OPTIONS_SHIFT) /* x110 Input with internal pull-up resistor */

#define _PIN_OUTPUT_SLEW_MASK  (3 << _PIN_OPTIONS_SHIFT) /* xx11 Mask to test for slow slew rate */
#define _PIN_OUTPUT_FAST       (1 << _PIN_OPTIONS_SHIFT) /* xx01 Output with fast slew rate */
#define _PIN_OUTPUT_SLOW       (3 << _PIN_OPTIONS_SHIFT) /* xx11 Output with slow slew rate */
#define _PIN_OUTPUT_OD_MASK    (5 << _PIN_OPTIONS_SHIFT) /* x1x1 Mask to test for open drain */
#define _PIN_OUTPUT_OPENDRAIN  (5 << _PIN_OPTIONS_SHIFT) /* x1x1 Output with open drain enabled */
#define _PIN_OUTPUT_DRIVE_MASK (9 << _PIN_OPTIONS_SHIFT) /* 1xx1 Mask to test for high drive strength */
#define _PIN_OUTPUT_LOWDRIVE   (1 << _PIN_OPTIONS_SHIFT) /* 0xx1 Output with low drive strength */
#define _PIN_OUTPUT_HIGHDRIVE  (9 << _PIN_OPTIONS_SHIFT) /* 1xx1 Output with high drive strength */

/* End-user pin modes and configurations.
 *  Notes:
 * (1) None of the digital  options are available for the analog mode,
 * (2) digital settings may be combined (OR'ed) provided that input-only
 *     and output-only options are not intermixed.
 */

#define PIN_ANALOG             _PIN_MODE_ANALOG

#define GPIO_INPUT             (_PIN_MODE_GPIO | _PIN_INPUT)
#define GPIO_PULLDOWN          (_PIN_MODE_GPIO | _PIN_INPUT_PULLDOWN)
#define GPIO_PULLUP            (_PIN_MODE_GPIO | _PIN_INPUT_PULLUP)
#define GPIO_OUTPUT            (_PIN_MODE_GPIO | _PIN_OUTPUT)
#define GPIO_FAST              (_PIN_MODE_GPIO | _PIN_OUTPUT_FAST)
#define GPIO_SLOW              (_PIN_MODE_GPIO | _PIN_OUTPUT_SLOW)
#define GPIO_OPENDRAIN         (_PIN_MODE_GPIO | _PIN_OUTPUT_OPENDRAIN)
#define GPIO_LOWDRIVE          (_PIN_MODE_GPIO | _PIN_OUTPUT_LOWDRIVE)
#define GPIO_HIGHDRIVE         (_PIN_MODE_GPIO | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT1               _PIN_MODE_ALT1
#define PIN_ALT1_INPUT         (_PIN_MODE_ALT1 | _PIN_INPUT)
#define PIN_ALT1_PULLDOWN      (_PIN_MODE_ALT1 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT1_PULLUP        (_PIN_MODE_ALT1 | _PIN_INPUT_PULLUP)
#define PIN_ALT1_OUTPUT        (_PIN_MODE_ALT1 | _PIN_OUTPUT)
#define PIN_ALT1_FAST          (_PIN_MODE_ALT1 | _PIN_OUTPUT_FAST)
#define PIN_ALT1_SLOW          (_PIN_MODE_ALT1 | _PIN_OUTPUT_SLOW)
#define PIN_ALT1_OPENDRAIN     (_PIN_MODE_ALT1 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT1_LOWDRIVE      (_PIN_MODE_ALT1 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT1_HIGHDRIVE     (_PIN_MODE_ALT1 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT2               _PIN_MODE_ALT2
#define PIN_ALT2_INPUT         (_PIN_MODE_ALT2 | _PIN_INPUT)
#define PIN_ALT2_PULLDOWN      (_PIN_MODE_ALT2 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT2_PULLUP        (_PIN_MODE_ALT2 | _PIN_INPUT_PULLUP)
#define PIN_ALT2_OUTPUT        (_PIN_MODE_ALT2 | _PIN_OUTPUT)
#define PIN_ALT2_FAST          (_PIN_MODE_ALT2 | _PIN_OUTPUT_FAST)
#define PIN_ALT2_SLOW          (_PIN_MODE_ALT2 | _PIN_OUTPUT_SLOW)
#define PIN_ALT2_OPENDRAIN     (_PIN_MODE_ALT2 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT2_LOWDRIVE      (_PIN_MODE_ALT2 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT2_HIGHDRIVE     (_PIN_MODE_ALT2 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT3               _PIN_MODE_ALT3
#define PIN_ALT3_INPUT         (_PIN_MODE_ALT3 | _PIN_INPUT)
#define PIN_ALT3_PULLDOWN      (_PIN_MODE_ALT3 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT3_PULLUP        (_PIN_MODE_ALT3 | _PIN_INPUT_PULLUP)
#define PIN_ALT3_OUTPUT        (_PIN_MODE_ALT3 | _PIN_OUTPUT)
#define PIN_ALT3_FAST          (_PIN_MODE_ALT3 | _PIN_OUTPUT_FAST)
#define PIN_ALT3_SLOW          (_PIN_MODE_ALT3 | _PIN_OUTPUT_SLOW)
#define PIN_ALT3_OPENDRAIN     (_PIN_MODE_ALT3 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT3_LOWDRIVE      (_PIN_MODE_ALT3 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT3_HIGHDRIVE     (_PIN_MODE_ALT3 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT4               _PIN_MODE_ALT4
#define PIN_ALT4_INPUT         (_PIN_MODE_ALT4 | _PIN_INPUT)
#define PIN_ALT4_PULLDOWN      (_PIN_MODE_ALT4 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT4_PULLUP        (_PIN_MODE_ALT4 | _PIN_INPUT_PULLUP)
#define PIN_ALT4_OUTPUT        (_PIN_MODE_ALT4 | _PIN_OUTPUT)
#define PIN_ALT4_FAST          (_PIN_MODE_ALT4 | _PIN_OUTPUT_FAST)
#define PIN_ALT4_SLOW          (_PIN_MODE_ALT4 | _PIN_OUTPUT_SLOW)
#define PIN_ALT4_OPENDRAIN     (_PIN_MODE_ALT4 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT4_LOWDRIVE      (_PIN_MODE_ALT4 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT4_HIGHDRIVE     (_PIN_MODE_ALT4 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT5               _PIN_MODE_ALT5
#define PIN_ALT5_INPUT         (_PIN_MODE_ALT5 | _PIN_INPUT)
#define PIN_ALT5_PULLDOWN      (_PIN_MODE_ALT5 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT5_PULLUP        (_PIN_MODE_ALT5 | _PIN_INPUT_PULLUP)
#define PIN_ALT5_OUTPUT        (_PIN_MODE_ALT5 | _PIN_OUTPUT)
#define PIN_ALT5_FAST          (_PIN_MODE_ALT5 | _PIN_OUTPUT_FAST)
#define PIN_ALT5_SLOW          (_PIN_MODE_ALT5 | _PIN_OUTPUT_SLOW)
#define PIN_ALT5_OPENDRAIN     (_PIN_MODE_ALT5 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT5_LOWDRIVE      (_PIN_MODE_ALT5 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT5_HIGHDRIVE     (_PIN_MODE_ALT5 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT6               _PIN_MODE_ALT6
#define PIN_ALT6_INPUT         (_PIN_MODE_ALT6 | _PIN_INPUT)
#define PIN_ALT6_PULLDOWN      (_PIN_MODE_ALT6 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT6_PULLUP        (_PIN_MODE_ALT6 | _PIN_INPUT_PULLUP)
#define PIN_ALT6_OUTPUT        (_PIN_MODE_ALT6 | _PIN_OUTPUT)
#define PIN_ALT6_FAST          (_PIN_MODE_ALT6 | _PIN_OUTPUT_FAST)
#define PIN_ALT6_SLOW          (_PIN_MODE_ALT6 | _PIN_OUTPUT_SLOW)
#define PIN_ALT6_OPENDRAIN     (_PIN_MODE_ALT6 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT6_LOWDRIVE      (_PIN_MODE_ALT6 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT6_HIGHDRIVE     (_PIN_MODE_ALT6 | _PIN_OUTPUT_HIGHDRIVE)

#define PIN_ALT7               _PIN_MODE_ALT7
#define PIN_ALT7_INPUT         (_PIN_MODE_ALT7 | _PIN_INPUT)
#define PIN_ALT7_PULLDOWN      (_PIN_MODE_ALT7 | _PIN_INPUT_PULLDOWN)
#define PIN_ALT7_PULLUP        (_PIN_MODE_ALT7 | _PIN_INPUT_PULLUP)
#define PIN_ALT7_OUTPUT        (_PIN_MODE_ALT7 | _PIN_OUTPUT)
#define PIN_ALT7_FAST          (_PIN_MODE_ALT7 | _PIN_OUTPUT_FAST)
#define PIN_ALT7_SLOW          (_PIN_MODE_ALT7 | _PIN_OUTPUT_SLOW)
#define PIN_ALT7_OPENDRAIN     (_PIN_MODE_ALT7 | _PIN_OUTPUT_OPENDRAIN)
#define PIN_ALT7_LOWDRIVE      (_PIN_MODE_ALT7 | _PIN_OUTPUT_LOWDRIVE)
#define PIN_ALT7_HIGHDRIVE     (_PIN_MODE_ALT7 | _PIN_OUTPUT_HIGHDRIVE)

/* The initial value for GPIO (Alternative 1 outputs):
 *
 * ---- ---v ---- ---- ---- ---- ---- ----
 *
 * Passive Filter and digital filter enable are valid in all digital pin
 * muxing modes.
 */

#define GPIO_OUTPUT_ONE        (1 << 24)  /* Bit 24: 1:Initial output value=1 */
#define GPIO_OUTPUT_ZERO       (0)        /* Bit 24: 0:Initial output value=0 */

/* Five bits are used to incode DMA/interrupt options:
 *
 * ---- ---- iiii i--- ---- ---- ---- ----
 *
 * The pin interrupt configuration is valid in all digital pin muxing modes
 * (restricted to inputs).
 */

#define _PIN_INT_SHIFT         (19)
#define _PIN_INT_MASK          (31 << _PIN_INT_SHIFT)

#define _PIN_INTDMA_MASK       (3 << _PIN_INT_SHIFT)
#define _PIN_INTDMA_NONE       (0 << _PIN_INT_SHIFT)
#define _PIN_DMA               (1 << _PIN_INT_SHIFT)
#define _PIN_INTERRUPT         (2 << _PIN_INT_SHIFT)

#define PIN_DMA_RISING         (5  << _PIN_INT_SHIFT) /* 00101 DMA Request on rising edge */
#define PIN_DMA_FALLING        (9  << _PIN_INT_SHIFT) /* 01001 DMA Request on falling edge */
#define PIN_DMA_BOTH           (13 << _PIN_INT_SHIFT) /* 01101 DMA Request on either edge */
#define PIN_INT_ZERO           (2  << _PIN_INT_SHIFT) /* 00010 Interrupt when logic zero */
#define PIN_INT_RISING         (6  << _PIN_INT_SHIFT) /* 00110 Interrupt on rising edge */
#define PIN_INT_FALLING        (10 << _PIN_INT_SHIFT) /* 01010 Interrupt on falling edge */
#define PIN_INT_BOTH           (14 << _PIN_INT_SHIFT) /* 01110 Interrupt on either edge */
#define PIN_INT_ONE            (18 << _PIN_INT_SHIFT) /* 10010 Interrupt when logic one */

/* Two bits is used to enable the filter options:
 *
 * ---- ---- ---- -fd- ---- ---- ---- ----
 *
 * Passive Filter and digital filter enable are valid in all digital pin
 * muxing modes.
 */

#define PIN_PASV_FILTER        (1 << 18)  /* Bit 18: Enable passive filter */
#define PIN_DIG_FILTER         (1 << 17)  /* Bit 17: Enable digital filter */

/* Three bits are used to define the port number:
 *
 * ---- ---- ---- ---- ---- -ppp ---- ----
 */

#define _PIN_PORT_SHIFT        (8)  /* Bits 8-10: port number */
#define _PIN_PORT_MASK         (7 << _PIN_PORT_SHIFT)

#define PIN_PORTA              (KINETIS_PORTA << _PIN_PORT_SHIFT)
#define PIN_PORTB              (KINETIS_PORTB << _PIN_PORT_SHIFT)
#define PIN_PORTC              (KINETIS_PORTC << _PIN_PORT_SHIFT)
#define PIN_PORTD              (KINETIS_PORTD << _PIN_PORT_SHIFT)
#define PIN_PORTE              (KINETIS_PORTE << _PIN_PORT_SHIFT)

/* Five bits are used to define the pin number:
 *
 * ---- ---- ---- ---- ---- ---- ---b bbbb
 */

#define _PIN_SHIFT             (0)  /* Bits 0-4: port number */
#define _PIN_MASK              (31 << _PIN_SHIFT)

#define PIN(n)                 ((n) << _PIN_SHIFT)
#define PIN0                   (0 << _PIN_SHIFT)
#define PIN1                   (1 << _PIN_SHIFT)
#define PIN2                   (2 << _PIN_SHIFT)
#define PIN3                   (3 << _PIN_SHIFT)
#define PIN4                   (4 << _PIN_SHIFT)
#define PIN5                   (5 << _PIN_SHIFT)
#define PIN6                   (6 << _PIN_SHIFT)
#define PIN7                   (7 << _PIN_SHIFT)
#define PIN8                   (8 << _PIN_SHIFT)
#define PIN9                   (9 << _PIN_SHIFT)
#define PIN10                  (10 << _PIN_SHIFT)
#define PIN11                  (11 << _PIN_SHIFT)
#define PIN12                  (12 << _PIN_SHIFT)
#define PIN13                  (13 << _PIN_SHIFT)
#define PIN14                  (14 << _PIN_SHIFT)
#define PIN15                  (15 << _PIN_SHIFT)
#define PIN16                  (16 << _PIN_SHIFT)
#define PIN17                  (17 << _PIN_SHIFT)
#define PIN18                  (18 << _PIN_SHIFT)
#define PIN19                  (19 << _PIN_SHIFT)
#define PIN20                  (20 << _PIN_SHIFT)
#define PIN21                  (21 << _PIN_SHIFT)
#define PIN22                  (22 << _PIN_SHIFT)
#define PIN23                  (23 << _PIN_SHIFT)
#define PIN24                  (24 << _PIN_SHIFT)
#define PIN25                  (25 << _PIN_SHIFT)
#define PIN26                  (26 << _PIN_SHIFT)
#define PIN27                  (27 << _PIN_SHIFT)
#define PIN28                  (28 << _PIN_SHIFT)
#define PIN29                  (29 << _PIN_SHIFT)
#define PIN30                  (30 << _PIN_SHIFT)
#define PIN31                  (31 << _PIN_SHIFT)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void kinetis_clockconfig(void);

/****************************************************************************
 * Name: kinetis_earlyserialinit
 *
 * Description:
 *   Performs the low level UART/LPUART initialization early in debug so that
 *   the serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void kinetis_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: kinetis_uart_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void kinetis_uart_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: kinetis_lpuart_earlyserialinit
 *
 * Description:
 *   Performs the low level LPUART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void kinetis_lpuart_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: kinetis_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void kinetis_lowsetup(void);

/****************************************************************************
 * Name: kinetis_uart_serialinit
 *
 * Description:
 *   Register all UART based serial console and serial ports.  This assumes
 *   that kinetis_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   first: - First TTY number to assign
 *
 * Returned Value:
 *   The next TTY number available for assignment
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
unsigned int kinetis_uart_serialinit(unsigned int first);
#endif

/****************************************************************************
 * Name: kinetis_lpuart_serialinit
 *
 * Description:
 *   Register all LPUART based serial console and serial ports.  This assumes
 *   that kinetis_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   first: - First TTY number to assign
 *
 * Returned Value:
 *   The next TTY number available for assignment
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
unsigned int kinetis_lpuart_serialinit(unsigned int first);
#endif

/****************************************************************************
 * Name: kinetis_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: kinetis_lpuartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
void kinetis_lpuartreset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: kinetis_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartconfigure(uintptr_t uart_base,
                           uint32_t baud, uint32_t clock,
                           unsigned int parity, unsigned int nbits,
                           unsigned int stop2,
                           bool iflow, bool oflow);
#endif

/****************************************************************************
 * Name: kinetis_lpuartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
void kinetis_lpuartconfigure(uintptr_t uart_base,
                             uint32_t baud, uint32_t clock,
                             unsigned int parity, unsigned int nbits,
                             unsigned int stop2,
                             bool iflow, bool oflow);
#endif

/****************************************************************************
 * Name: kinetis_wddisable
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

void kinetis_wddisable(void);

/****************************************************************************
 * Name: kinetis_pinconfig
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int kinetis_pinconfig(uint32_t cfgset);

/****************************************************************************
 * Name: kinetis_pinfilter
 *
 * Description:
 *   Configure the digital filter associated with a port. The digital filter
 *   capabilities of the PORT module are available in all digital pin muxing
 *   modes.
 *
 * Input Parameters:
 *   port  - See KINETIS_PORTn definitions in kinetis_port.h
 *   lpo   - true: Digital Filters are clocked by the bus clock
 *           false: Digital Filters are clocked by the 1 kHz LPO clock
 *   width - Filter Length
 *
 ****************************************************************************/

int kinetis_pinfilter(unsigned int port, bool lpo, unsigned int width);

/****************************************************************************
 * Name: kinetis_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void kinetis_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: kinetis_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool kinetis_gpioread(uint32_t pinset);

/****************************************************************************
 * Name: kinetis_pinirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPIOIRQ
void kinetis_pinirqinitialize(void);
#else
#  define kinetis_pinirqinitialize()
#endif

/****************************************************************************
 * Name: kinetis_pinirqattach
 *
 * Description:
 *   Attach a pin interrupt handler.  The normal initialization sequence is:
 *
 *   1. Call kinetis_pinconfig() to configure the interrupting pin (pin
 *      interrupts will be disabled.
 *   2. Call kinetis_pinirqattach() to attach the pin interrupt handling
 *      function.
 *   3. Call kinetis_pinirqenable() to enable interrupts on the pin.
 *
 * Input Parameters:
 *   pinset -  Pin configuration
 *   pinisr -  Pin interrupt service routine
 *   arg    -  An argument that will be provided to the interrupt service
 *             routine.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int kinetis_pinirqattach(uint32_t pinset, xcpt_t pinisr, void *arg);

/****************************************************************************
 * Name: kinetis_pinirqenable
 *
 * Description:
 *   Enable the interrupt for specified pin IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPIOIRQ
void kinetis_pinirqenable(uint32_t pinset);
#else
#  define kinetis_pinirqenable(pinset)
#endif

/****************************************************************************
 * Name: kinetis_pinirqdisable
 *
 * Description:
 *   Disable the interrupt for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_GPIOIRQ
void kinetis_pinirqdisable(uint32_t pinset);
#else
#  define kinetis_pinirqdisable(pinset)
#endif

/****************************************************************************
 * Name: kinetis_pindmaenable
 *
 * Description:
 *   Enable DMA for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_DMA
void kinetis_pindmaenable(uint32_t pinset);
#endif

/****************************************************************************
 * Name: kinetis_pindmadisable
 *
 * Description:
 *   Disable DMA for specified pin
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_DMA
void kinetis_pindmadisable(uint32_t pinset);
#endif

/****************************************************************************
 * Function:  kinetis_pindump
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the
 *   provided pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void kinetis_pindump(uint32_t pinset, const char *msg);
#else
#  define kinetis_pindump(p,m)
#endif

/****************************************************************************
 * Name: kinetis_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.
 *
 ****************************************************************************/

void kinetis_clrpend(int irq);

/****************************************************************************
 * Name: sdhc_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SDHC
struct sdio_dev_s;
struct sdio_dev_s *sdhc_initialize(int slotno);
#endif

/****************************************************************************
 * Name: sdhc_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SDHC
void sdhc_mediachange(struct sdio_dev_s *dev, bool cardinslot);
#endif

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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SDHC
void sdhc_wrprotect(struct sdio_dev_s *dev, bool wrprotect);
#endif
#undef EXTERN
#if defined(__cplusplus)
}
#endif

/****************************************************************************
 * Name: kinetis_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_ENET
int kinetis_netinitialize(int intf);
#endif

/****************************************************************************
 * Function: kinetis_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN, this value
 *          identifies which CAN is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef CONFIG_KINETIS_FLEXCAN
int kinetis_caninitialize(int intf);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_H */
