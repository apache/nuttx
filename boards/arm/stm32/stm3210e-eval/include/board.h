/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/include/board.h
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

#ifndef __BOARDS_ARM_STM32_STM3210E_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM3210E_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Logic in arch/arm/src and boards/ may need to include these file prior to
 * including board.h:  stm32_rcc.h, stm32_sdio.h, stm32.h.  They cannot be
 * included here because board.h is used in other contexts where the STM32
 * internal header files are not available.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* On-board crystal frequency is 8MHz (HSE) */

#define STM32_BOARD_XTAL        8000000ul

/* PLL source is HSE/1, PLL multipler is 9:
 *      PLL frequency is 8MHz (XTAL) x 9 = 72MHz
 */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 2-7, 12-14 */

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* SRAM definitions *********************************************************/

/* The 8 Mbit SRAM is provided on the PT3 board using the FSMC_NE3 chip
 * select.
 */

/* This is the Bank1 SRAM3 address: */

#define BOARD_SRAM_BASE    0x68000000     /* Bank2 SRAM3 base address */
#define BOARD_SRAM_SIZE    (1*1024*1024)  /* 8-Mbit = 1-Mbyte */

/* LED definitions **********************************************************/

/* The STM3210E-EVAL board has 4 LEDs that we will encode as: */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* The STM3210E-EVAL supports several buttons
 *
 *  Reset           -- Connected to NRST
 *   Wakeup          -- Connected to PA.0
 *   Tamper          -- Connected to PC.13
 *   Key             -- Connected to PG.8
 *
 * And a Joystick
 *
 *   Joystick center -- Connected to PG.7
 *   Joystick down   -- Connected to PD.3
 *   Joystick left   -- Connected to PG.14
 *   Joystick right  -- Connected to PG.13
 *   Joystick up     -- Connected to PG.15
 *
 * The Joystick is treated like the other buttons unless CONFIG_DJOYSTICK
 * is defined, then it is assumed that they should be used by the discrete
 * joystick driver.
 */

#define BUTTON_WAKEUP        0
#define BUTTON_TAMPER        1
#define BUTTON_KEY           2

#ifdef CONFIG_DJOYSTICK
#  define NUM_BUTTONS        3
#else
#  define JOYSTICK_SEL       3
#  define JOYSTICK_DOWN      4
#  define JOYSTICK_LEFT      5
#  define JOYSTICK_RIGHT     6
#  define JOYSTICK_UP        7

#  define NUM_BUTTONS        8
#endif

#define BUTTON_WAKEUP_BIT    (1 << BUTTON_WAKEUP)
#define BUTTON_TAMPER_BIT    (1 << BUTTON_TAMPER)
#define BUTTON_KEY_BIT       (1 << BUTTON_KEY)

#ifndef CONFIG_DJOYSTICK
#  define JOYSTICK_SEL_BIT   (1 << JOYSTICK_SEL)
#  define JOYSTICK_DOWN_BIT  (1 << JOYSTICK_DOWN)
#  define JOYSTICK_LEFT_BIT  (1 << JOYSTICK_LEFT)
#  define JOYSTICK_RIGHT_BIT (1 << JOYSTICK_RIGHT)
#  define JOYSTICK_UP_BIT    (1 << JOYSTICK_UP)
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  stm3210e_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM3210E-EVAL board.
 *   Because of the various rotations, clearing the display in the normal
 *   way by writing a sequences of runs that covers the entire display can
 *   be very slow.  Here the display is cleared by simply setting all GRAM
 *   memory to the specified color.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm3210e_lcdclear(uint16_t color);
#endif

/****************************************************************************
 * Name: stm32_lm75initialize
 *
 * Description:
 *   Initialize and register the LM-75 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C) && defined(CONFIG_STM32_I2C1)
int stm32_lm75initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_lm75attach
 *
 * Description:
 *   Attach the LM-75 interrupt handler
 *
 * Input Parameters:
 *   irqhandler - the LM-75 interrupt handler
 *   arg        - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C) && defined(CONFIG_STM32_I2C1)
int stm32_lm75attach(xcpt_t irqhandler, void *arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM3210E_EVAL_INCLUDE_BOARD_H */
