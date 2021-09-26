/****************************************************************************
 * boards/arm/stm32/nucleo-f446re/include/board.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F446RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_NUCLEO_F446RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The NUCLEOF446RE supports both HSE and LSE crystals (X2 and X3).
 * However, as shipped, the X2 and X3 crystals are not populated.
 * Therefore the Nucleo-FF446RE will need to run off the 16MHz HSI clock.
 *
 *   System Clock source           : PLL (HSI)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL config
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 1            (STM32_RCC_CFGR_PPRE2)
 *   HSI Frequency(Hz)             : 16000000     (nominal)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 216          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 4            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 9            (STM32_PLLCFG_PPQ)
 *   Flash Latency(WS)             : 4
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - not installed
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_BOARD_USEHSI      1

/* Main PLL Configuration.
 *
 * Formulae:
 *
 *  target 180 MHz, source 16 MHz -> ratio = 11.25 = 22.5 x 2 = 45 x 4
 *  so we can select a divider of 4 and a multiplier of 45
 *  However multiplier must be between 50 and 432
 *  so we double again to choose a multiplier of 90, and a divider of 8
 *  VCO output frequency must be in range 100...432 MHz
 *
 *   VCO input frequency        = PLL input clock frequency / PLLM,
 *     2 <= PLLM <= 63
 *   VCO output frequency       = VCO input frequency × PLLN,
 *     50 <= PLLN <= 432 (50-99 only if VCO input > 1 MHz)
 *   PLL output clock frequency = VCO frequency / PLLP,
 *     PLLP = 2, 4, 6, or 8
 *   USB OTG FS clock frequency = VCO frequency / PLLQ,
 *     2 <= PLLQ <= 15
 *

 * PLLQ = 7.5 PLLP = 2 PLLN=90 PLLM=4
 *
 * We will configure like this
 *
 *   PLL source is HSI
 *   PLL_VCO = (STM32_HSI_FREQUENCY / PLLM) * PLLN
 *           = (16,000,000 / 4) * 90
 *           = 360 MHz
 *   SYSCLK  = PLL_VCO / PLLP
 *           = 360,000,000 / 2 = 180,000,000
 *   USB OTG FS and SDIO Clock
 *           = TODO 7.5 is not possible
 *
 * REVISIT: Trimming of the HSI is not yet supported.
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(90)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(15)

#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (104MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (52MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 (REVISIT) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (104MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (STM32_PCLK2_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 *
 * REVISIT
 */

#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 *
 * REVISIT
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 *
 * REVISIT
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3 <- may later be used by SPI DMA
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

/* Need to VERIFY fwb */

#define DMACHAN_SPI1_RX DMAMAP_SPI1_RX_1
#define DMACHAN_SPI1_TX DMAMAP_SPI1_TX_1
#define DMACHAN_SPI2_RX DMAMAP_SPI2_RX
#define DMACHAN_SPI2_TX DMAMAP_SPI2_TX

/* ADC 1 */

#define ADC1_DMA_CHAN           DMAMAP_ADC1_1

/* Alternate function pin selections ****************************************/

/* USART1:
 *   RXD: PA10  CN9 pin 3, CN10 pin 33
 *        PB7   CN7 pin 21
 *   TXD: PA9   CN5 pin 1, CN10 pin 21
 *        PB6   CN5 pin 3, CN10 pin 17
 */

#if !defined(CONFIG_BOARD_STM32_IHM08M1)
#  define GPIO_USART1_RX GPIO_USART1_RX_1    /* PA10 */
#  define GPIO_USART1_TX GPIO_USART1_TX_1    /* PA9  */
#else
#  define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */
#  define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6  */
#endif

/* USART2:
 *   RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
 *        PD6
 *   TXD: PA2   CN9 pin 2(See SB13, 14, 62, 63). CN10 pin 35
 *        PD5
 */

#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3 */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2

/* USART6:
 *  RXD: PC7    CN5 pin2, CN10 pin 19
 *       PA12   CN10, pin 12
 *  TXD: PC6    CN10, pin 4
 *       PA11   CN10, pin 14
 */

#define GPIO_USART6_RX   GPIO_USART6_RX_1    /* PC7 */
#define GPIO_USART6_TX   GPIO_USART6_TX_1    /* PC6 */

/* UART RX DMA configurations */

#define DMAMAP_USART1_RX DMAMAP_USART1_RX_2
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
    GPIO_PORTB|GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
    GPIO_PORTB|GPIO_PIN9)

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1
#define GPIO_I2C2_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
    GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
    GPIO_PORTB|GPIO_PIN11)

/* SPI
 *
 * There are sensors on SPI1, and SPI2 is connected to the FRAM.
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_1
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_1
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_1

/* CAN */

#define GPIO_CAN1_RX     GPIO_CAN1_RX_2
#define GPIO_CAN1_TX     GPIO_CAN1_TX_2

#define GPIO_CAN2_RX     GPIO_CAN2_RX_2
#define GPIO_CAN2_TX     GPIO_CAN2_TX_2

/* LEDs
 *
 * The Nucleo F446RE and F411RE boards provide a single user LED, LD2.  LD2
 * is the green LED connected to Arduino signal D13 corresponding to MCU I/O
 * PA5 (pin 21) or PB13 (pin 34) depending on the STM32 target.
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LD2         0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LD2_BIT     (1 << BOARD_LD2)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD2
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons
 *
 *   B1 USER: the user button is connected to the I/O PC13 (pin 2) of
 *   the STM32 microcontroller.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* TIM2 input ***************************************************************/

#ifndef CONFIG_NUCLEO_F446RE_QETIMER_TIM2_IHM08M1_MAP
#  define GPIO_TIM2_CH1IN (GPIO_TIM2_CH1IN_1 | GPIO_PULLUP) /* PA8 */
#  define GPIO_TIM2_CH2IN (GPIO_TIM2_CH2IN_1 | GPIO_PULLUP) /* PB0 */
#else
#  define GPIO_TIM2_CH1IN (GPIO_TIM2_CH1IN_2 | GPIO_PULLUP) /* PA15 */
#  define GPIO_TIM2_CH2IN (GPIO_TIM2_CH2IN_2 | GPIO_PULLUP) /* PB3 */
#endif

/* TIM3 configuration *******************************************************/

#define GPIO_TIM3_CH1OUT GPIO_TIM3_CH1OUT_1

#ifdef CONFIG_BOARD_STM32_IHM08M1

/* Configuration specific to the X-NUCLEO-IHM08M1 expansion board with
 * the L6398 gate drivers.
 */

/* TIM1 configuration *******************************************************/

#define GPIO_TIM1_CH1OUT   GPIO_TIM1_CH1OUT_1 /* TIM1 CH1  - PA8  - U high */
#define GPIO_TIM1_CH1NOUT  GPIO_TIM1_CH1N_1   /* TIM1 CH1N - PA7  - U low */
#define GPIO_TIM1_CH2OUT   GPIO_TIM1_CH2OUT_1 /* TIM1 CH2  - PA9  - V high */
#define GPIO_TIM1_CH2NOUT  GPIO_TIM1_CH2N_1   /* TIM1 CH2N - PB0  - V low */
#define GPIO_TIM1_CH3OUT   GPIO_TIM1_CH3OUT_1 /* TIM1 CH3  - PA10 - W high */
#define GPIO_TIM1_CH3NOUT  GPIO_TIM1_CH3N_1   /* TIM1 CH3N - PB1  - W low */
#define GPIO_TIM1_CH4OUT   0                  /* not used as output */

/* Board LED */

#  define GPIO_FOC_LED2   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)

/* Debug pin */

#  define GPIO_FOC_DEBUG0 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_FOC_DEBUG1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_FOC_DEBUG2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_FOC_DEBUG3 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)

#endif  /* CONFIG_BOARD_STM32_IHM08M1 */

#endif /* __BOARDS_ARM_STM32_NUCLEO_F446RE_INCLUDE_BOARD_H */
