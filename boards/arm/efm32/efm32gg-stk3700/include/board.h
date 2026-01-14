/****************************************************************************
 * boards/arm/efm32/efm32gg-stk3700/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_EFM32_EFM32GG_STK3700_INCLUDE_BOARD_H
#define __BOARDS_ARM_EFM32_EFM32GG_STK3700_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Clock Sources
 *   - 1-28 MHz High Frequency RC Oscillator (HFRCO)
 *   - 4-32 MHz High Frequency Crystal Oscillator (HFXO)
 *   - 32.768 kHz Low Frequency RC Oscillator (LFRCO)
 *   - 32.768 kHz Low Frequency Crystal Oscillator (LFXO)
 *   - 1KHz Ultra Low Frequency RC Oscillator (ULFRCO)
 *
 * The device boots with 14 MHz HFRCO as the HFCLK source.
 */

#define BOARD_HAVE_HFXO        1        /* Have High frequency crystal oscillator */
#define BOARD_HAVE_LFXO        1        /* Have Loq frequency crystal oscillator */

#define BOARD_HFRCO_FREQUENCY  14000000 /* 14MHz on reset */
#define BOARD_HFXO_FREQUENCY   48000000 /* 48MHz crystal on board */
#define BOARD_LFRCO_FREQUENCY  32768    /* Low frequency oscillator */
#define BOARD_LFXO_FREQUENCY   32768    /* 32MHz crystal on board */
#define BOARD_ULFRCO_FREQUNCY  1000     /* Ultra low frequency oscillator */

/* HFCLK - High Frequency Clock
 *
 * HFCLK is the selected High Frequency Clock. This clock is used by the CMU
 * and drives the two prescalers that generate HFCORECLK and HFPERCLK. The
 * HFCLK can be driven by a high-frequency oscillator (HFRCO or HFXO) or one
 * of the low-frequency oscillators (LFRCO or LFXO). By default the HFRCO is
 * selected.
 */

#define BOARD_HFCLKSEL            _CMU_CMD_HFCLKSEL_HFXO
#define BOARD_HFCLKDIV            0     /* Does not apply to EFM32G */
#define BOARD_HFCLK_FREQUENCY     BOARD_HFXO_FREQUENCY

/* HFCORECLK - High Frequency Core Clock
 *
 * HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 * Modules, which consists of the CPU and modules that are tightly coupled
 * to the CPU, e.g. MSC, DMA etc.  The frequency of HFCORECLK is set using
 * the CMU_HFCORECLKDIV register.
 */

#define BOARD_HFCORECLKDIV        _CMU_HFCORECLKDIV_HFCORECLKDIV_DEFAULT
#define BOARD_HFCORECLK_FREQUENCY BOARD_HFXO_FREQUENCY

/* HFPERCLK - High Frequency Peripheral Clock
 *
 * Like HFCORECLK, HFPERCLK can also be a prescaled version of HFCLK. This
 * clock drives the High-Frequency Peripherals. The frequency of HFPERCLK is
 * set using the CMU_HFPERCLKDIV register.
 */

#define BOARD_HFPERCLKDIV        _CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT
#define BOARD_HFPERCLK_FREQUENCY BOARD_HFXO_FREQUENCY

/* LFACLK - Low Frequency A Clock
 *
 * LFACLK is the selected clock for the Low Energy A Peripherals. There are
 * four selectable sources for LFACLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFACLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFA
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy A
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable.
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFXO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFRCO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif

/* LFBCLK - Low Frequency B Clock
 *
 * LFBCLK is the selected clock for the Low Energy B Peripherals. There are
 * four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFBCLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFB
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy B
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable.
 * ULFRCO is a special case.
 */

#define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFXO
#undef  BOARD_LFBCLK_ULFRCO
#define BOARD_LFBCLK_FREQUENCY    BOARD_LFXO_FREQUENCY

/* PCNTnCLK - Pulse Counter n Clock
 *
 * Each available pulse counter is driven by its own clock, PCNTnCLK where
 * n is the pulse counter instance number. Each pulse counter can be
 * configured to use an external pin (PCNTn_S0) or LFACLK as PCNTnCLK.
 */

/* WDOGCLK - Watchdog Timer Clock
 *
 * The Watchdog Timer (WDOG) can be configured to use one of three different
 * clock sources: LFRCO, LFXO or ULFRCO. ULFRCO (Ultra Low Frequency RC
 * Oscillator) is a separate 1 kHz RC oscillator that also runs in EM3.
 */

/* AUXCLK - Auxiliary Clock
 *
 * AUXCLK is a 1-28 MHz clock driven by a separate RC oscillator, AUXHFRCO.
 * This clock is used for flash programming and Serial Wire Output (SWO).
 * During flash programming this clock will be active. If the AUXHFRCO has
 * not been enabled explicitly by software, the MSC will automatically
 * start and stop it. The AUXHFRCO is enabled by writing a 1 to AUXHFRCOEN
 * in CMU_OSCENCMD. This explicit enabling is required when SWO is used.
 */

/* LEDs *********************************************************************/

/* The EFM32 Giant Gecko Start Kit has two yellow LEDs marked LED0 and LED1.
 * These LEDs are controlled by GPIO pins on the EFM32.  The LEDs are
 * connected to pins PE2 and PE3 in an active high configuration:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * E2/BCK_VOUT/EBI_A09 #0/               MCU_PE2 UIF_LED0
 *   TIM3_CC2 #1/U1_TX #3/ACMP0_O #1
 * E3/BCK_STAT/EBI_A10 #0/U1_RX #3/      MCU_PE3 UIF_LED1
 *   ACMP1_O #1
 * ------------------------------------- --------------------
 *
 * All LEDs are grounded and so are illuminated by outputting a high
 * value to the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/efm32_autoleds.c.  The LEDs are used to
 * encode OS-related events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                         LED0     LED1
 *      ----------------- ---   -----------------------  -------- --------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF      OFF      */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF      OFF      */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF      OFF      */
#define LED_STACKCREATED  1  /* Idle stack created         ON       OFF      */
#define LED_INIRQ         2  /* In an interrupt              No change       */
#define LED_SIGNAL        2  /* In a signal handler          No change       */
#define LED_ASSERTION     2  /* An assertion failed          No change       */
#define LED_PANIC         3  /* The system has crashed     OFF      Blinking */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used        */

/* Buttons ******************************************************************/

/* The EFM32 Giant Gecko Start Kit has two buttons marked PB0 and PB1. They
 * are connected to the EFM32, and are debounced by RC filters with a time
 * constant of 1ms. The buttons are connected to pins PB9 and PB10:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * B9/EBI_A03/U1_TX #2                   MCU_PB9  UIF_PB0
 * B10/EBI_A04/U1_RX #2                  MCU_PB10 UIF_PB1
 * ------------------------------------- --------------------
 *
 * Buttons are connected to ground so they will read low when closed.
 */

#define BUTTON_PB0        0
#define BUTTON_PB1        1
#define NUM_BUTTONS       2

#define BUTTON_PB0_BIT    (1 << BUTTON_PB0)
#define BUTTON_PB1_BIT    (1 << BUTTON_PB1)

/* Pin routing **************************************************************/

/* UART0:
 *
 *   The kit contains a board controller that is responsible for performing
 *   various board level tasks, such as handling the debugger and the
 *   Advanced Energy Monitor.
 *   An interface is provided between the EFM32 and the board controller in
 *   the form of a UART connection. The connection is enabled by
 *   setting the EFM_BC_EN (PF7) line high, and using the lines EFM_BC_TX
 *   (PE0) and EFM_BC_RX (PE1) for communicating.
 *
 *   U0_TX #1 PE0 MCU_PE0, UART0_TX #0, EFM_BC_RX, BC_UART_RX
 *   U0_RX #1 PE1 MCU_PE1, UART0_TX #1, EFM_BC_TX, BC_UART_TX
 */

#define BOARD_UART0_RX_GPIO          (GPIO_PORTE|GPIO_PIN1)
#define BOARD_UART0_TX_GPIO          (GPIO_PORTE|GPIO_PIN0)
#define BOARD_UART0_ROUTE_LOCATION   _USART_ROUTE_LOCATION_LOC1

/* LEUART0:
 *
 *   LEU0_TX #0 PD4 Available on TP122 and EXP pin 12
 *   LEU0_RX #0 PD5 Available on TP123 and EXP pin 14
 */

#define BOARD_LEUART0_RX_GPIO        (GPIO_PORTD|GPIO_PIN5)
#define BOARD_LEUART0_TX_GPIO        (GPIO_PORTD|GPIO_PIN4)
#define BOARD_LEUART0_ROUTE_LOCATION _LEUART_ROUTE_LOCATION_LOC0

/* Clocking *****************************************************************/

/* Clock Sources
 *   - 1-28 MHz High Frequency RC Oscillator (HFRCO)
 *   - 4-32 MHz High Frequency Crystal Oscillator (HFXO)
 *   - 32.768 kHz Low Frequency RC Oscillator (LFRCO)
 *   - 32.768 kHz Low Frequency Crystal Oscillator (LFXO)
 *   - 1KHz Ultra Low Frequency RC Oscillator (ULFRCO)
 *
 * The device boots with 14 MHz HFRCO as the HFCLK source.
 */

#define BOARD_HAVE_HFXO        1        /* Have High frequency crystal oscillator */
#define BOARD_HAVE_LFXO        1        /* Have Low frequency crystal oscillator */

#define BOARD_HFRCO_FREQUENCY  14000000 /* 14MHz on reset */
#define BOARD_HFXO_FREQUENCY   48000000 /* 32MHz crystal on board */
#define BOARD_LFRCO_FREQUENCY  32768    /* Low frequency oscillator */
#define BOARD_LFXO_FREQUENCY   32768    /* 32KHz crystal on board */
#define BOARD_ULFRCO_FREQUNCY  1000     /* Ultra low frequency oscillator */

#if BOARD_HAVE_HFXO
#   define BOARD_SYSTEM_FREQUENCY  BOARD_HFXO_FREQUENCY
#else
#   define BOARD_SYSTEM_FREQUENCY  BOARD_HFRCO_FREQUENCY
#endif

/* HFCLK - High Frequency Clock
 *
 * HFCLK is the selected High Frequency Clock. This clock is used by the CMU
 * and drives the two prescalers that generate HFCORECLK and HFPERCLK. The
 * HFCLK can be driven by a high-frequency oscillator (HFRCO or HFXO) or one
 * of the low-frequency oscillators (LFRCO or LFXO). By default the HFRCO is
 * selected.
 */

#define BOARD_HFCLKSEL            _CMU_CMD_HFCLKSEL_HFXO
#define BOARD_HFCLKDIV            0     /* Does not apply to EFM32G */
#define BOARD_HFCLK_FREQUENCY     BOARD_SYSTEM_FREQUENCY

/* HFCORECLK - High Frequency Core Clock
 *
 * HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 * Modules, which consists of the CPU and modules that are tightly coupled
 * to the CPU, e.g. MSC, DMA etc.  The frequency of HFCORECLK is set using
 * the CMU_HFCORECLKDIV register.
 */

#define BOARD_HFCORECLKDIV        _CMU_HFCORECLKDIV_HFCORECLKDIV_DEFAULT
#define BOARD_HFCORECLK_FREQUENCY BOARD_SYSTEM_FREQUENCY

/* HFPERCLK - High Frequency Peripheral Clock
 *
 * Like HFCORECLK, HFPERCLK can also be a prescaled version of HFCLK. This
 * clock drives the High-Frequency Peripherals. The frequency of HFPERCLK is
 * set using the CMU_HFPERCLKDIV register.
 */

#define BOARD_HFPERCLKDIV        _CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT
#define BOARD_HFPERCLK_FREQUENCY BOARD_SYSTEM_FREQUENCY

/* LFACLK - Low Frequency A Clock
 *
 * LFACLK is the selected clock for the Low Energy A Peripherals. There are
 * four selectable sources for LFACLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFACLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFA
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy A
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFXO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFRCO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif

/* LFBCLK - Low Frequency B Clock
 *
 * LFBCLK is the selected clock for the Low Energy B Peripherals. There are
 * four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFBCLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFB
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy B
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFB_DISABLED to disable
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFXO
#   undef  BOARD_LFBCLK_ULFRCO
#   define BOARD_LFBCLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFRCO
#   undef  BOARD_LFBCLK_ULFRCO
#   define BOARD_LFBCLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif

/* BURTC Clock source
 * select clock source from following value:
 *  - BURTC_CTRL_CLKSEL_LFRCO
 *  - BURTC_CTRL_CLKSEL_LFXO
 *  - BURTC_CTRL_CLKSEL_ULFRCO
 */
#define BOARD_BURTC_CLKSRC  BURTC_CTRL_CLKSEL_LFXO

/* BURTC Prescaler
 * select Prescaler from following value:
 *  - BURTC_CTRL_PRESC_DIV1
 *  - BURTC_CTRL_PRESC_DIV2
 *  - BURTC_CTRL_PRESC_DIV4
 *  - BURTC_CTRL_PRESC_DIV8
 *  - BURTC_CTRL_PRESC_DIV16
 *  - BURTC_CTRL_PRESC_DIV32
 *  - BURTC_CTRL_PRESC_DIV64
 *  - BURTC_CTRL_PRESC_DIV128
 */
#define BOARD_BURTC_PRESC   BURTC_CTRL_PRESC_DIV1

/* BURTC Mode
 * select enable mode from following value:
 *  - BURTC_CTRL_MODE_EM2EN
 *  - BURTC_CTRL_MODE_EM3EN
 *  - BURTC_CTRL_MODE_EM4EN
 */
#define BOARD_BURTC_MODE    BURTC_CTRL_MODE_EM4EN

/* PCNTnCLK - Pulse Counter n Clock
 *
 * Each available pulse counter is driven by its own clock, PCNTnCLK where
 * n is the pulse counter instance number. Each pulse counter can be
 * configured to use an external pin (PCNTn_S0) or LFACLK as PCNTnCLK.
 */

/* WDOGCLK - Watchdog Timer Clock
 *
 * The Watchdog Timer (WDOG) can be configured to use one of three different
 * clock sources: LFRCO, LFXO or ULFRCO. ULFRCO (Ultra Low Frequency RC
 * Oscillator) is a separate 1 kHz RC oscillator that also runs in EM3.
 */

/* AUXCLK - Auxiliary Clock
 *
 * AUXCLK is a 1-28 MHz clock driven by a separate RC oscillator, AUXHFRCO.
 * This clock is used for flash programming and Serial Wire Output (SWO).
 * During flash programming this clock will be active. If the AUXHFRCO has
 * not been enabled explicitly by software, the MSC will automatically
 * start and stop it. The AUXHFRCO is enabled by writing a 1 to AUXHFRCOEN
 * in CMU_OSCENCMD. This explicit enabling is required when SWO is used.
 */
#define BOARD_AUXCLK_FREQUENCY 14000000

/* SWO Location - Where SWO goes out.
 *
 * On some board there is possible to use many location for swo output.
 */
#define BOARD_SWOPORT_LOCATION  0

/* SWO Location - Where SWO goes out.
 *
 * On some board there is possible to different 2 pin in function of
 * swo location output.
 */
#define BOARD_GPIO_SWOPORT   ( GPIO_OUTPUT_PUSHPULL | \
                               GPIO_PORTF           | \
                               GPIO_PIN2                )

/* LEDs *********************************************************************/

/* The EFM32 Gecko Starter Kit supports 4 yellow LEDs.  One side is grounded
 * so these LEDs are illuminated by outputting a high value.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with efm32_setled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_SHIFT     BOARD_LED1
#define BOARD_LED_SHIFT2    BOARD_LED2

/* LED bits for use with efm32_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* Pin routing **************************************************************/

/* USART0 is SPI:
 *
 *   U0_CLK  #2 PC9
 *   U0_MISO #2 PC10
 *   U0_MOSI #2 PC11
 */

#define BOARD_USART0_CLK_GPIO       (GPIO_PORTC|GPIO_PIN9 )
#define BOARD_USART0_RX_GPIO        (GPIO_PORTC|GPIO_PIN10)
#define BOARD_USART0_TX_GPIO        (GPIO_PORTC|GPIO_PIN11)
#define BOARD_USART0_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC2

/* Pin routing **************************************************************/

/* UART1 for KLINE:
 *
 *   U1_RX #1 PD1
 *   U1_TX #1 PD0
 */

#define BOARD_USART1_RX_GPIO        (GPIO_PORTD|GPIO_PIN1)
#define BOARD_USART1_TX_GPIO        (GPIO_PORTD|GPIO_PIN0)
#define BOARD_USART1_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC1

/* Pin routing **************************************************************/

/* UART2 for GPS:
 *
 *   U2_RX #1 PC3
 *   U2_TX #1 PC2
 */

#define BOARD_USART2_RX_GPIO        (GPIO_PORTC|GPIO_PIN3)
#define BOARD_USART2_TX_GPIO        (GPIO_PORTC|GPIO_PIN2)
#define BOARD_USART2_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC0

/* Pin routing **************************************************************/

/* PWM on TIMER0 for backlight:
 *
 *   TIMER0 Channel 1 #4 PC0
 */
#define BOARD_PWM_TIMER0_PINCFG     (GPIO_PORTC|GPIO_PIN0|GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)
#define BOARD_PWM_TIMER0_PINLOC     _TIMER_ROUTE_LOCATION_LOC4
#define BOARD_PWM_TIMER0_CLKIN      BOARD_SYSTEM_FREQUENCY

/* IC1:
 *
 * The pnbfano board one I2C.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PC7                   For external spi (WIFI)
 * PC8                   for SDCARD
 * --------------------- ---------------------
 */

#define BOARD_I2C1_SDA   (GPIO_PORTC|GPIO_PIN4|GPIO_OUTPUT_WIREDAND|GPIO_OUTPUT_SET)
#define BOARD_I2C1_SCL   (GPIO_PORTC|GPIO_PIN5|GPIO_OUTPUT_WIREDAND|GPIO_OUTPUT_SET)
#define BOARD_I2C1_ROUTE_LOCATION _I2C_ROUTE_LOCATION_LOC0

/* VCMP:
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * VDD                   level 3.1 V
 * --------------------- ---------------------
 */

#define BOARD_VCMP_HALFBIAS  1      /* Enable half bias          */
#define BOARD_VCMP_BIASPROG  8      /* Half bias 8 => 1µA        */
#define BOARD_VCMP_WARMUP    0      /* 4 cycle (quickest)        */
#define BOARD_VCMP_LEVEL    (3.1)   /* Set to 3V1               */

/* ACMP:
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * VBAT (PD6)             10V
 * --------------------- ---------------------
 */

/* #define BOARD_ACMP_ENABLE */

#define BOARD_ACMP_WARMUP    0 /* 4 cycle (quickest)*/
#define BOARD_ACMP_LEVEL    -1 /* TODO BOARD_ACMP_LEVEL     */
#define BOARD_ACMP_INPUT     0 /* CH0 */
#define BOARD_ACMP_LOCATION  0 /* Location 0 */

#define BOARD_SDHC_BLOCK_DEV_PATH   "/dev/mmcsd0"
#define BOARD_SDHC_MOUNT_PATH       "/mnt"

#define BOARD_SPI0_CLK       (GPIO_PORTC|GPIO_PIN9 |GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)
#define BOARD_SPI0_MOSI      (GPIO_PORTC|GPIO_PIN11|GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)

#define BOARD_SPI0_MISO      (GPIO_PORTC|GPIO_PIN10|GPIO_INPUT_PULLUP)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  efm32_ledinit, efm32_setled, and efm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

int board_format_sdcard(void);
int board_mount_sdcard(void);
int board_umount_sdcard(void);
int board_is_usb_connected(void);
int board_is_usb_enabled(void);
int board_enable_usbmsc(void);
int board_disable_usbmsc(void);

#ifndef CONFIG_ARCH_LEDS
void efm32_ledinit(void);
void efm32_setled(int led, bool ledon);
void efm32_setleds(uint8_t ledset);
#endif

#endif /* __BOARDS_ARM_EFM32_EFM32GG_STK3700_INCLUDE_BOARD_H */
