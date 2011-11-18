/************************************************************************************
 * arch/arm/src/stm32/stm32_gpio.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *           Uros Platise <uros.platise@isotel.eu>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_GPIO_H
#define __ARCH_ARM_SRC_STM32_STM32_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32F10XX)
#  include "chip/stm32f10xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/stm32f40xxx_gpio.h"
#else
#  error "Unrecognized STM32 chip"
#endif

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif
 
/* Bit-encoded input to stm32_configgpio() */

#if defined(CONFIG_STM32_STM32F10XX)

/* 16-bit Encoding:
 * OFFS SX.. VPPP BBBB
 */

/* Output mode:
 *
 * O... .... .... ....
 */

#define GPIO_INPUT                    (1 << 15)                  /* Bit 15: 1=Input mode */
#define GPIO_OUTPUT                   (0)                        /*         0=Output or alternate function */
#define GPIO_ALT                      (0)

/* If the pin is a GPIO digital output, then this identifies the initial output value.
 * If the pin is an input, this bit is overloaded to provide the qualifier to\
 * distinquish input pull-up and -down:
 *
 * .... .... V... ....
 */

#define GPIO_OUTPUT_SET               (1 << 7)                   /* Bit 7: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* These bits set the primary function of the pin:
 * .FF. .... .... ....
 */

#define GPIO_CNF_SHIFT                13                         /* Bits 13-14: GPIO function */
#define GPIO_CNF_MASK                 (3 << GPIO_CNF_SHIFT)

#  define GPIO_CNF_ANALOGIN           (0 << GPIO_CNF_SHIFT)      /* Analog input */
#  define GPIO_CNF_INFLOAT            (1 << GPIO_CNF_SHIFT)      /* Input floating */
#  define GPIO_CNF_INPULLUD           (2 << GPIO_CNF_SHIFT)      /* Input pull-up/down general bit, since up is composed of two parts */
#  define GPIO_CNF_INPULLDWN          (2 << GPIO_CNF_SHIFT)      /* Input pull-down */
#  define GPIO_CNF_INPULLUP          ((2 << GPIO_CNF_SHIFT) | GPIO_OUTPUT_SET) /* Input pull-up */

#  define GPIO_CNF_OUTPP              (0 << GPIO_CNF_SHIFT)      /* Output push-pull */
#  define GPIO_CNF_OUTOD              (1 << GPIO_CNF_SHIFT)      /* Output open-drain */
#  define GPIO_CNF_AFPP               (2 << GPIO_CNF_SHIFT)      /* Alternate function push-pull */
#  define GPIO_CNF_AFOD               (3 << GPIO_CNF_SHIFT)      /* Alternate function open-drain */

/* Maximum frequency selection:
 * ...S S... .... ....
 */

#define GPIO_MODE_SHIFT               11                         /* Bits 11-12: GPIO frequency selection */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT             (0 << GPIO_MODE_SHIFT)     /* Input mode (reset state) */
#  define GPIO_MODE_10MHz             (1 << GPIO_MODE_SHIFT)     /* Output mode, max speed 10 MHz */
#  define GPIO_MODE_2MHz              (2 << GPIO_MODE_SHIFT)     /* Output mode, max speed 2 MHz */
#  define GPIO_MODE_50MHz             (3 << GPIO_MODE_SHIFT)     /* Output mode, max speed 50 MHz */

/* External interrupt selection (GPIO inputs only):
 * .... .X.. .... ....
 */

#define GPIO_EXTI                     (1 << 10)                   /* Bit 10: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 * .... .... .PPP ....
 */

#define GPIO_PORT_SHIFT               4                          /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */

/* This identifies the bit in the port:
 * .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                0                          /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN0                     (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1                     (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2                     (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3                     (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4                     (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5                     (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6                     (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7                     (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8                     (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9                     (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10                    (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                    (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                    (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                    (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                    (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                    (15 << GPIO_PIN_SHIFT)

#elif defined(CONFIG_STM32_STM32F40XX)

/* 16-bit Encoding:
 * Inputs:                MMUU X... PPPP BBBB
 * Outputs:               MMUU FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA PPPP BBBB
 * Analog:                MMUU .... PPPP BBBB
 */

/* Common mode encodings ***********************************************************/
/* Mode:
 *
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT               (14)                       /* Bits 14-15: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#define GPIO_INPUT                    (0 << GPIO_MODE_SHIFT)     /* Input mode */
#define GPIO_OUTPUT                   (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#define GPIO_ALT                      (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */
#define GPIO_ANALOG                   (3 << GPIO_MODE_SHIFT)     /* Analog mode */

/* Output pull-ups/downs:
 * ..UU .... .... ....
 */

#define GPIO_PUPD_SHIFT               (12)                       /* Bits 12-13: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLLOAT                 (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLUP                 (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Input (only) mode encodings *****************************************************/
/* Outputs: MMUU X... PPPP BBBB */

/* External interrupt selection (GPIO inputs only):
 * .... X... .... ....
 */

#define GPIO_EXTI                     (1 << 11)                  /* Bit 11: Configure as EXTI interrupt */

/* Output (only) mode encodings ****************************************************/
/* Outputs: MMUU FFOV PPPP BBBB */

/* Output frequency selection:
 * .... FF.. .... ....
 */

#define GPIO_OUTPUT_MODE_SHIFT        (10)                       /* Bits 10-11: GPIO frequency selection */
#define GPIO_OUTPUT_MODE_MASK         (3 << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT_MODE_2MHz       (0 << GPIO_MODE_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_OUTPUT_MODE_25MHz      (1 << GPIO_MODE_SHIFT)     /* 25 MHz Medium speed output */
#  define GPIO_OUTPUT_MODE_20MHz      (2 << GPIO_MODE_SHIFT)     /* 50 MHz Fast speed output  */
#  define GPIO_OUTPUT_MODE_100MHz     (3 << GPIO_MODE_SHIFT)     /* 100 MHz High speed output */

/* Output type selection:
 *  .... ..O. .... ....
 */

#define GPIO_OUTPUT_OPENDRAM          (1 << 9)                   /* Open-drain output */
#define GPIO_OUTPUT_PUSHPULL          (0)                        /* Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial output value.
 * If the pin is an input, this bit is overloaded to provide the qualifier to
 * distinquish input pull-up and -down:
 *
 * .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                   /* Bit 8: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* Alternate function (only) mode encodings ****************************************/
/* Alternate Functions:   MMUU AAAA PPPP BBBB */

#define GPIO_ALTFUNC_SHIFT            (8)                        /* Bits 8-11: Alternate function */
#define GPIO_ALTFUNC_MASK             (15 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC(n)             ((n) << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_0              (0 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_1              (1 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_2              (2 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_3              (3 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_4              (4 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_5              (5 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_6              (6 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_7              (7 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_8              (8 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_9              (9 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_10             (10 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_11             (11 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_12             (12 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_13             (13 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_14             (14 << GPIO_ALTFUNC_SHIFT)
#  define GPIO_ALTFUNC_15             (15 << GPIO_ALTFUNC_SHIFT)

/* Common port encodings ***********************************************************/
/* This identifies the GPIO port:
 * .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               4                          /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#  define GPIO_PORTH                  (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTI                  (8 << GPIO_PORT_SHIFT)     /*   GPIOI */

/* This identifies the bit in the port:
 * .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                0                          /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

#else
#  error "Unrecognized STM32 chip"
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...) 
 *   function, it must be unconfigured with stm32_unconfiggpio() with 
 *   the same cfgset first before it can be set to non-alternative function.
 * 
 * Returns:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 * 
 ************************************************************************************/

EXTERN int stm32_configgpio(uint32_t cfgset);

/************************************************************************************
 * Name: stm32_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previsouly selected as alternative function (GPIO_ALT|GPIO_CNF_AFPP|...).
 * 
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger 
 *   over-current/alarm function. 
 * 
 * Returns:
 *  OK on success
 *  ERROR on invalid port
 *
 ************************************************************************************/

EXTERN int stm32_unconfiggpio(uint32_t cfgset);

/************************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void stm32_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool stm32_gpioread(uint32_t pinset);

/************************************************************************************
 * Name: stm32_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 * 
 * Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 * 
 * Returns: 
 *  The previous value of the interrupt handler function pointer.  This value may,
 *  for example, be used to restore the previous handler when multiple handlers are
 *  used.
 *
 ************************************************************************************/

EXTERN xcpt_t stm32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge, 
                                 bool event, xcpt_t func);

/************************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG
EXTERN int stm32_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define stm32_dumpgpio(p,m)
#endif

/************************************************************************************
 * Function:  stm32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions. 
 * 
 * Typically called from stm32_start().
 ************************************************************************************/

EXTERN void stm32_gpioinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_GPIO_H */
