/****************************************************************************
 * arch/arm/src/stm32/stm32_gpio.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_GPIO_H
#define __ARCH_ARM_SRC_STM32_STM32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32L15XX)
#  include "hardware/stm32l15xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F10XX)
#  include "hardware/stm32f10xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "hardware/stm32f20xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
      defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f30xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_gpio.h"
#elif defined(CONFIG_STM32_STM32G4XXX)
#  include "hardware/stm32g4xxxx_gpio.h"
#else
#  error "Unrecognized STM32 chip"
#endif

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to stm32_configgpio() */

#if defined(CONFIG_STM32_STM32F10XX)

/* 16-bit Encoding:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * OFFS SX.. VPPP BBBB
 */

/* Output mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * O... .... .... ....
 */

#define GPIO_INPUT                    (1 << 15)                  /* Bit 15: 1=Input mode */
#define GPIO_OUTPUT                   (0)                        /*         0=Output or alternate function */
#define GPIO_ALT                      (0)

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to
 * provide the qualifier to\ distinguish input pull-up and -down:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... V... ....
 */

#define GPIO_OUTPUT_SET               (1 << 7)                   /* Bit 7: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* These bits set the primary function of the pin:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .FF. .... .... ....
 */

#define GPIO_CNF_SHIFT                13                         /* Bits 13-14: GPIO function */
#define GPIO_CNF_MASK                 (3 << GPIO_CNF_SHIFT)

#  define GPIO_CNF_ANALOGIN           (0 << GPIO_CNF_SHIFT)      /* Analog input */
#  define GPIO_CNF_INFLOAT            (1 << GPIO_CNF_SHIFT)      /* Input floating */
#  define GPIO_CNF_INPULLUD           (2 << GPIO_CNF_SHIFT)      /* Input pull-up/down general bit, since up is composed of two parts */
#  define GPIO_CNF_INPULLDWN          (2 << GPIO_CNF_SHIFT)      /* Input pull-down */
#  define GPIO_CNF_INPULLUP          ((2 << GPIO_CNF_SHIFT) \
                                      | GPIO_OUTPUT_SET)         /* Input pull-up */

#  define GPIO_CNF_OUTPP              (0 << GPIO_CNF_SHIFT)      /* Output push-pull */
#  define GPIO_CNF_OUTOD              (1 << GPIO_CNF_SHIFT)      /* Output open-drain */
#  define GPIO_CNF_AFPP               (2 << GPIO_CNF_SHIFT)      /* Alternate function push-pull */
#  define GPIO_CNF_AFOD               (3 << GPIO_CNF_SHIFT)      /* Alternate function open-drain */

/* Maximum frequency selection:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...S S... .... ....
 */

#define GPIO_MODE_SHIFT               11                         /* Bits 11-12: GPIO frequency selection */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT             (0 << GPIO_MODE_SHIFT)     /* Input mode (reset state) */
#  define GPIO_MODE_10MHz             (1 << GPIO_MODE_SHIFT)     /* Output mode, max speed 10 MHz */
#  define GPIO_MODE_2MHz              (2 << GPIO_MODE_SHIFT)     /* Output mode, max speed 2 MHz */
#  define GPIO_MODE_50MHz             (3 << GPIO_MODE_SHIFT)     /* Output mode, max speed 50 MHz */

#define GPIO_ADJUST_MODE(p, m) (((p) & ~GPIO_MODE_MASK) | (m))

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .X.. .... ....
 */

#define GPIO_EXTI                     (1 << 10)                   /* Bit 10: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
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
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
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

#elif defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
      defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) || \
      defined(CONFIG_STM32_STM32G4XXX)
/* Each port bit of the general-purpose I/O (GPIO) ports can be
 * individually configured by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU .... ...X PPPP BBBB
 * Outputs:               MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA FFO. PPPP BBBB
 * Analog:                MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)                       /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */
#  define GPIO_ANALOG                 (3 << GPIO_MODE_SHIFT)     /* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)                       /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)                       /* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (15 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)
#  define GPIO_AF8                    (8 << GPIO_AF_SHIFT)
#  define GPIO_AF9                    (9 << GPIO_AF_SHIFT)
#  define GPIO_AF10                   (10 << GPIO_AF_SHIFT)
#  define GPIO_AF11                   (11 << GPIO_AF_SHIFT)
#  define GPIO_AF12                   (12 << GPIO_AF_SHIFT)
#  define GPIO_AF13                   (13 << GPIO_AF_SHIFT)
#  define GPIO_AF14                   (14 << GPIO_AF_SHIFT)
#  define GPIO_AF15                   (15 << GPIO_AF_SHIFT)

/* Output/Alt function frequency selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT              (10)                       /* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK               (3 << GPIO_SPEED_SHIFT)
#if defined(CONFIG_STM32_STM32L15XX)
#  define GPIO_SPEED_400KHz           (0 << GPIO_SPEED_SHIFT)    /* 400 kHz Very low speed output */
#  define GPIO_SPEED_2MHz             (1 << GPIO_SPEED_SHIFT)    /* 2 MHz Low speed output */
#  define GPIO_SPEED_10MHz            (2 << GPIO_SPEED_SHIFT)    /* 10 MHz Medium speed output */
#  define GPIO_SPEED_40MHz            (3 << GPIO_SPEED_SHIFT)    /* 40 MHz High speed output */
#elif defined(CONFIG_STM32_STM32G4XXX)                           /* With C=50pF, 2.7<VDD<3.6, DS12288 Rev2 Table 59 */
#  define GPIO_SPEED_5MHz             (0 << GPIO_SPEED_SHIFT)    /* 5 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)    /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)    /* 50 MHz Fast speed output */
#  define GPIO_SPEED_120MHz           (3 << GPIO_SPEED_SHIFT)    /* 120 MHz High speed output */
#else
#  define GPIO_SPEED_2MHz             (0 << GPIO_SPEED_SHIFT)    /* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)    /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)    /* 50 MHz Fast speed output  */
#ifndef CONFIG_STM32_STM32F30XX
#  define GPIO_SPEED_100MHz           (3 << GPIO_SPEED_SHIFT)    /* 100 MHz High speed output */
#endif
#endif

/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)                   /* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL                 (0)                        /* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to
 * provide the qualifier to distinguish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                   /* Bit 8: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...X .... ....
 */

#define GPIO_EXTI                     (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               (4)                        /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#if defined (CONFIG_STM32_STM32L15XX)
#  define GPIO_PORTH                  (5 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTF                  (6 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (7 << GPIO_PORT_SHIFT)     /*   GPIOG */
#else
#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#  define GPIO_PORTH                  (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTI                  (8 << GPIO_PORT_SHIFT)     /*   GPIOI */
#  define GPIO_PORTJ                  (9 << GPIO_PORT_SHIFT)     /*   GPIOJ */
#  define GPIO_PORTK                  (10 << GPIO_PORT_SHIFT)    /*   GPIOK */
#endif

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                (0)                        /* Bits 0-3: GPIO number: 0-15 */
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_gpiobase[STM32_NGPIO_PORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ****************************************************************************/

int stm32_configgpio(uint32_t cfgset);

/****************************************************************************
 * Name: stm32_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from shocks, as
 *   unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 *   while it should operate in PWM mode could produce excessive on-board
 *   currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  ERROR on invalid port
 *
 ****************************************************************************/

int stm32_unconfiggpio(uint32_t cfgset);

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool stm32_gpioread(uint32_t pinset);

/****************************************************************************
 * Name: stm32_iocompensation
 *
 * Description:
 *   Enable I/O compensation.
 *
 *   By default the I/O compensation cell is not used. However when the I/O
 *   output buffer speed is configured in 50 MHz or 100 MHz mode, it is
 *   recommended to use the compensation cell for slew rate control on I/O
 *   tf(IO)out)/tr(IO)out commutation to reduce the I/O noise on power
 *   supply.
 *
 *   The I/O compensation cell can be used only when the supply voltage
 *   ranges from 2.4 to 3.6 V.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_HAVE_IOCOMPENSATION
void stm32_iocompensation(void);
#endif

/****************************************************************************
 * Name: stm32_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int stm32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg);

/****************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int stm32_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define stm32_dumpgpio(p,m)
#endif

/****************************************************************************
 * Function:  stm32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from stm32_start().
 *
 ****************************************************************************/

void stm32_gpioinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_GPIO_H */
