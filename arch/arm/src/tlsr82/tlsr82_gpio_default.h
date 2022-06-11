/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_gpio_default.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_DEFAULT_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_DEFAULT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Group A ~ D, IE, OEN, OUT, DS, ACT GET */

#define GPIO_DEFAULT_IE_GET(G) \
  ((P##G##0_INPUT_ENABLE << 0) | (P##G##1_INPUT_ENABLE << 1) | \
   (P##G##2_INPUT_ENABLE << 2) | (P##G##3_INPUT_ENABLE << 3) | \
   (P##G##4_INPUT_ENABLE << 4) | (P##G##5_INPUT_ENABLE << 5) | \
   (P##G##6_INPUT_ENABLE << 6) | (P##G##7_INPUT_ENABLE << 7))

#define GPIO_DEFAULT_OEN_GET(G) \
  (((P##G##0_OUTPUT_ENABLE ? 0 : 1) << 0) | \
   ((P##G##1_OUTPUT_ENABLE ? 0 : 1) << 1) | \
   ((P##G##2_OUTPUT_ENABLE ? 0 : 1) << 2) | \
   ((P##G##3_OUTPUT_ENABLE ? 0 : 1) << 3) | \
   ((P##G##4_OUTPUT_ENABLE ? 0 : 1) << 4) | \
   ((P##G##5_OUTPUT_ENABLE ? 0 : 1) << 5) | \
   ((P##G##6_OUTPUT_ENABLE ? 0 : 1) << 6) | \
   ((P##G##7_OUTPUT_ENABLE ? 0 : 1) << 7))

#define GPIO_DEFAULT_OUT_GET(G) \
  ((P##G##0_DATA_OUT << 0) | (P##G##1_DATA_OUT << 1) | \
   (P##G##2_DATA_OUT << 2) | (P##G##3_DATA_OUT << 3) | \
   (P##G##4_DATA_OUT << 4) | (P##G##5_DATA_OUT << 5) | \
   (P##G##6_DATA_OUT << 6) | (P##G##7_DATA_OUT << 7))

#define GPIO_DEFAULT_DS_GET(G) \
  ((P##G##0_DATA_STRENGTH << 0) | (P##G##1_DATA_STRENGTH << 1) | \
   (P##G##2_DATA_STRENGTH << 2) | (P##G##3_DATA_STRENGTH << 3) | \
   (P##G##4_DATA_STRENGTH << 4) | (P##G##5_DATA_STRENGTH << 5) | \
   (P##G##6_DATA_STRENGTH << 6) | (P##G##7_DATA_STRENGTH << 7))

#define GPIO_DEFAULT_ACT_GET(G) \
  ((P##G##0_FUNC == AS_GPIO ? BIT (0) : 0) | \
   (P##G##1_FUNC == AS_GPIO ? BIT (1) : 0) | \
   (P##G##2_FUNC == AS_GPIO ? BIT (2) : 0) | \
   (P##G##3_FUNC == AS_GPIO ? BIT (3) : 0) | \
   (P##G##4_FUNC == AS_GPIO ? BIT (4) : 0) | \
   (P##G##5_FUNC == AS_GPIO ? BIT (5) : 0) | \
   (P##G##6_FUNC == AS_GPIO ? BIT (6) : 0) | \
   (P##G##7_FUNC == AS_GPIO ? BIT (7) : 0))

/* Group E, IE, OEN, OUT, DS, ACT GET */

#define GPIO_DEFAULT_IE_GET_E \
  ((PE0_INPUT_ENABLE << 0) | (PE1_INPUT_ENABLE << 1) | \
   (PE2_INPUT_ENABLE << 2) | (PE3_INPUT_ENABLE << 3))

#define GPIO_DEFAULT_OEN_GET_E \
  (((PE0_OUTPUT_ENABLE ? 0 : 1) << 0) | \
   ((PE1_OUTPUT_ENABLE ? 0 : 1) << 1) | \
   ((PE2_OUTPUT_ENABLE ? 0 : 1) << 2) | \
   ((PE3_OUTPUT_ENABLE ? 0 : 1) << 3))

#define GPIO_DEFAULT_OUT_GET_E \
  ((PE0_DATA_OUT << 0) | (PE1_DATA_OUT << 1) | \
   (PE2_DATA_OUT << 2) | (PE3_DATA_OUT << 3))

#define GPIO_DEFAULT_DS_GET_E \
  ((PE0_DATA_STRENGTH << 0) | (PE1_DATA_STRENGTH << 1) | \
   (PE2_DATA_STRENGTH << 2) | (PE3_DATA_STRENGTH << 3))

#define GPIO_DEFAULT_ACT_GET_E \
  ((PE0_FUNC == AS_GPIO ? BIT (0) : 0) | \
   (PE1_FUNC == AS_GPIO ? BIT (1) : 0) | \
   (PE2_FUNC == AS_GPIO ? BIT (2) : 0) | \
   (PE3_FUNC == AS_GPIO ? BIT (3) : 0))

/* GPIO GROUP A Default Definition */

#ifndef PA0_INPUT_ENABLE
#define PA0_INPUT_ENABLE 0
#endif
#ifndef PA1_INPUT_ENABLE
#define PA1_INPUT_ENABLE 0
#endif
#ifndef PA2_INPUT_ENABLE
#define PA2_INPUT_ENABLE 0
#endif
#ifndef PA3_INPUT_ENABLE
#define PA3_INPUT_ENABLE 0
#endif
#ifndef PA4_INPUT_ENABLE
#define PA4_INPUT_ENABLE 0
#endif
#ifndef PA5_INPUT_ENABLE
#define PA5_INPUT_ENABLE 0 /* USB */
#endif
#ifndef PA6_INPUT_ENABLE
#define PA6_INPUT_ENABLE 0 /* USB */
#endif
#ifndef PA7_INPUT_ENABLE
#define PA7_INPUT_ENABLE 1 /* SWS */
#endif
#ifndef PA0_OUTPUT_ENABLE
#define PA0_OUTPUT_ENABLE 0
#endif
#ifndef PA1_OUTPUT_ENABLE
#define PA1_OUTPUT_ENABLE 0
#endif
#ifndef PA2_OUTPUT_ENABLE
#define PA2_OUTPUT_ENABLE 0
#endif
#ifndef PA3_OUTPUT_ENABLE
#define PA3_OUTPUT_ENABLE 0
#endif
#ifndef PA4_OUTPUT_ENABLE
#define PA4_OUTPUT_ENABLE 0
#endif
#ifndef PA5_OUTPUT_ENABLE
#define PA5_OUTPUT_ENABLE 0
#endif
#ifndef PA6_OUTPUT_ENABLE
#define PA6_OUTPUT_ENABLE 0
#endif
#ifndef PA7_OUTPUT_ENABLE
#define PA7_OUTPUT_ENABLE 0
#endif
#ifndef PA0_DATA_STRENGTH
#define PA0_DATA_STRENGTH 1
#endif
#ifndef PA1_DATA_STRENGTH
#define PA1_DATA_STRENGTH 1
#endif
#ifndef PA2_DATA_STRENGTH
#define PA2_DATA_STRENGTH 1
#endif
#ifndef PA3_DATA_STRENGTH
#define PA3_DATA_STRENGTH 1
#endif
#ifndef PA4_DATA_STRENGTH
#define PA4_DATA_STRENGTH 1
#endif
#ifndef PA5_DATA_STRENGTH
#define PA5_DATA_STRENGTH 1
#endif
#ifndef PA6_DATA_STRENGTH
#define PA6_DATA_STRENGTH 1
#endif
#ifndef PA7_DATA_STRENGTH
#define PA7_DATA_STRENGTH 1
#endif
#ifndef PA0_DATA_OUT
#define PA0_DATA_OUT 0
#endif
#ifndef PA1_DATA_OUT
#define PA1_DATA_OUT 0
#endif
#ifndef PA2_DATA_OUT
#define PA2_DATA_OUT 0
#endif
#ifndef PA3_DATA_OUT
#define PA3_DATA_OUT 0
#endif
#ifndef PA4_DATA_OUT
#define PA4_DATA_OUT 0
#endif
#ifndef PA5_DATA_OUT
#define PA5_DATA_OUT 0
#endif
#ifndef PA6_DATA_OUT
#define PA6_DATA_OUT 0
#endif
#ifndef PA7_DATA_OUT
#define PA7_DATA_OUT 0
#endif
#ifndef PA0_FUNC
#define PA0_FUNC AS_GPIO
#endif
#ifndef PA1_FUNC
#define PA1_FUNC AS_GPIO
#endif
#ifndef PA2_FUNC
#define PA2_FUNC AS_GPIO
#endif
#ifndef PA3_FUNC
#define PA3_FUNC AS_GPIO
#endif
#ifndef PA4_FUNC
#define PA4_FUNC AS_GPIO
#endif
#ifndef PA5_FUNC
#define PA5_FUNC AS_GPIO
#endif
#ifndef PA6_FUNC
#define PA6_FUNC AS_GPIO
#endif
#ifndef PA7_FUNC
#define PA7_FUNC AS_SWIRE
#endif
#ifndef PULL_WAKEUP_SRC_PA0
#define PULL_WAKEUP_SRC_PA0 0
#endif
#ifndef PULL_WAKEUP_SRC_PA1
#define PULL_WAKEUP_SRC_PA1 0
#endif
#ifndef PULL_WAKEUP_SRC_PA2
#define PULL_WAKEUP_SRC_PA2 0
#endif
#ifndef PULL_WAKEUP_SRC_PA3
#define PULL_WAKEUP_SRC_PA3 0
#endif
#ifndef PULL_WAKEUP_SRC_PA4
#define PULL_WAKEUP_SRC_PA4 0
#endif
#ifndef PULL_WAKEUP_SRC_PA5
#define PULL_WAKEUP_SRC_PA5 0
#endif
#ifndef PULL_WAKEUP_SRC_PA6
#define PULL_WAKEUP_SRC_PA6 0
#endif
#ifndef PULL_WAKEUP_SRC_PA7
#define PULL_WAKEUP_SRC_PA7 PM_PIN_PULLUP_1M /* sws pullup */
#endif

/* GPIO GROUP B Default Definition */

#ifndef PB0_INPUT_ENABLE
#define PB0_INPUT_ENABLE 0
#endif
#ifndef PB1_INPUT_ENABLE
#define PB1_INPUT_ENABLE 0
#endif
#ifndef PB2_INPUT_ENABLE
#define PB2_INPUT_ENABLE 0
#endif
#ifndef PB3_INPUT_ENABLE
#define PB3_INPUT_ENABLE 0
#endif
#ifndef PB4_INPUT_ENABLE
#define PB4_INPUT_ENABLE 0
#endif
#ifndef PB5_INPUT_ENABLE
#define PB5_INPUT_ENABLE 0
#endif
#ifndef PB6_INPUT_ENABLE
#define PB6_INPUT_ENABLE 0
#endif
#ifndef PB7_INPUT_ENABLE
#define PB7_INPUT_ENABLE 0
#endif
#ifndef PB0_OUTPUT_ENABLE
#define PB0_OUTPUT_ENABLE 0
#endif
#ifndef PB1_OUTPUT_ENABLE
#define PB1_OUTPUT_ENABLE 0
#endif
#ifndef PB2_OUTPUT_ENABLE
#define PB2_OUTPUT_ENABLE 0
#endif
#ifndef PB3_OUTPUT_ENABLE
#define PB3_OUTPUT_ENABLE 0
#endif
#ifndef PB4_OUTPUT_ENABLE
#define PB4_OUTPUT_ENABLE 0
#endif
#ifndef PB5_OUTPUT_ENABLE
#define PB5_OUTPUT_ENABLE 0
#endif
#ifndef PB6_OUTPUT_ENABLE
#define PB6_OUTPUT_ENABLE 0
#endif
#ifndef PB7_OUTPUT_ENABLE
#define PB7_OUTPUT_ENABLE 0
#endif
#ifndef PB0_DATA_STRENGTH
#define PB0_DATA_STRENGTH 1
#endif
#ifndef PB1_DATA_STRENGTH
#define PB1_DATA_STRENGTH 1
#endif
#ifndef PB2_DATA_STRENGTH
#define PB2_DATA_STRENGTH 1
#endif
#ifndef PB3_DATA_STRENGTH
#define PB3_DATA_STRENGTH 1
#endif
#ifndef PB4_DATA_STRENGTH
#define PB4_DATA_STRENGTH 1
#endif
#ifndef PB5_DATA_STRENGTH
#define PB5_DATA_STRENGTH 1
#endif
#ifndef PB6_DATA_STRENGTH
#define PB6_DATA_STRENGTH 1
#endif
#ifndef PB7_DATA_STRENGTH
#define PB7_DATA_STRENGTH 1
#endif
#ifndef PB0_DATA_OUT
#define PB0_DATA_OUT 0
#endif
#ifndef PB1_DATA_OUT
#define PB1_DATA_OUT 0
#endif
#ifndef PB2_DATA_OUT
#define PB2_DATA_OUT 0
#endif
#ifndef PB3_DATA_OUT
#define PB3_DATA_OUT 0
#endif
#ifndef PB4_DATA_OUT
#define PB4_DATA_OUT 0
#endif
#ifndef PB5_DATA_OUT
#define PB5_DATA_OUT 0
#endif
#ifndef PB6_DATA_OUT
#define PB6_DATA_OUT 0
#endif
#ifndef PB7_DATA_OUT
#define PB7_DATA_OUT 0
#endif
#ifndef PB0_FUNC
#define PB0_FUNC AS_GPIO
#endif
#ifndef PB1_FUNC
#define PB1_FUNC AS_GPIO
#endif
#ifndef PB2_FUNC
#define PB2_FUNC AS_GPIO
#endif
#ifndef PB3_FUNC
#define PB3_FUNC AS_GPIO
#endif
#ifndef PB4_FUNC
#define PB4_FUNC AS_GPIO
#endif
#ifndef PB5_FUNC
#define PB5_FUNC AS_GPIO
#endif
#ifndef PB6_FUNC
#define PB6_FUNC AS_GPIO
#endif
#ifndef PB7_FUNC
#define PB7_FUNC AS_GPIO
#endif
#ifndef PULL_WAKEUP_SRC_PB0
#define PULL_WAKEUP_SRC_PB0 0
#endif
#ifndef PULL_WAKEUP_SRC_PB1
#define PULL_WAKEUP_SRC_PB1 0
#endif
#ifndef PULL_WAKEUP_SRC_PB2
#define PULL_WAKEUP_SRC_PB2 0
#endif
#ifndef PULL_WAKEUP_SRC_PB3
#define PULL_WAKEUP_SRC_PB3 0
#endif
#ifndef PULL_WAKEUP_SRC_PB4
#define PULL_WAKEUP_SRC_PB4 0
#endif
#ifndef PULL_WAKEUP_SRC_PB5
#define PULL_WAKEUP_SRC_PB5 0
#endif
#ifndef PULL_WAKEUP_SRC_PB6
#define PULL_WAKEUP_SRC_PB6 0
#endif
#ifndef PULL_WAKEUP_SRC_PB7
#define PULL_WAKEUP_SRC_PB7 0
#endif

/* GPIO GROUP C Default Definition */

#ifndef PC0_INPUT_ENABLE
#define PC0_INPUT_ENABLE 0
#endif
#ifndef PC1_INPUT_ENABLE
#define PC1_INPUT_ENABLE 0
#endif
#ifndef PC2_INPUT_ENABLE
#define PC2_INPUT_ENABLE 0
#endif
#ifndef PC3_INPUT_ENABLE
#define PC3_INPUT_ENABLE 0
#endif
#ifndef PC4_INPUT_ENABLE
#define PC4_INPUT_ENABLE 0
#endif
#ifndef PC5_INPUT_ENABLE
#define PC5_INPUT_ENABLE 0
#endif
#ifndef PC6_INPUT_ENABLE
#define PC6_INPUT_ENABLE 0
#endif
#ifndef PC7_INPUT_ENABLE
#define PC7_INPUT_ENABLE 0
#endif
#ifndef PC0_OUTPUT_ENABLE
#define PC0_OUTPUT_ENABLE 0
#endif
#ifndef PC1_OUTPUT_ENABLE
#define PC1_OUTPUT_ENABLE 0
#endif
#ifndef PC2_OUTPUT_ENABLE
#define PC2_OUTPUT_ENABLE 0
#endif
#ifndef PC3_OUTPUT_ENABLE
#define PC3_OUTPUT_ENABLE 0
#endif
#ifndef PC4_OUTPUT_ENABLE
#define PC4_OUTPUT_ENABLE 0
#endif
#ifndef PC5_OUTPUT_ENABLE
#define PC5_OUTPUT_ENABLE 0
#endif
#ifndef PC6_OUTPUT_ENABLE
#define PC6_OUTPUT_ENABLE 0
#endif
#ifndef PC7_OUTPUT_ENABLE
#define PC7_OUTPUT_ENABLE 0
#endif
#ifndef PC0_DATA_STRENGTH
#define PC0_DATA_STRENGTH 1
#endif
#ifndef PC1_DATA_STRENGTH
#define PC1_DATA_STRENGTH 1
#endif
#ifndef PC2_DATA_STRENGTH
#define PC2_DATA_STRENGTH 1
#endif
#ifndef PC3_DATA_STRENGTH
#define PC3_DATA_STRENGTH 1
#endif
#ifndef PC4_DATA_STRENGTH
#define PC4_DATA_STRENGTH 1
#endif
#ifndef PC5_DATA_STRENGTH
#define PC5_DATA_STRENGTH 1
#endif
#ifndef PC6_DATA_STRENGTH
#define PC6_DATA_STRENGTH 1
#endif
#ifndef PC7_DATA_STRENGTH
#define PC7_DATA_STRENGTH 1
#endif
#ifndef PC0_DATA_OUT
#define PC0_DATA_OUT 0
#endif
#ifndef PC1_DATA_OUT
#define PC1_DATA_OUT 0
#endif
#ifndef PC2_DATA_OUT
#define PC2_DATA_OUT 0
#endif
#ifndef PC3_DATA_OUT
#define PC3_DATA_OUT 0
#endif
#ifndef PC4_DATA_OUT
#define PC4_DATA_OUT 0
#endif
#ifndef PC5_DATA_OUT
#define PC5_DATA_OUT 0
#endif
#ifndef PC6_DATA_OUT
#define PC6_DATA_OUT 0
#endif
#ifndef PC7_DATA_OUT
#define PC7_DATA_OUT 0
#endif
#ifndef PC0_FUNC
#define PC0_FUNC AS_GPIO
#endif
#ifndef PC1_FUNC
#define PC1_FUNC AS_GPIO
#endif
#ifndef PC2_FUNC
#define PC2_FUNC AS_GPIO
#endif
#ifndef PC3_FUNC
#define PC3_FUNC AS_GPIO
#endif
#ifndef PC4_FUNC
#define PC4_FUNC AS_GPIO
#endif
#ifndef PC5_FUNC
#define PC5_FUNC AS_GPIO
#endif
#ifndef PC6_FUNC
#define PC6_FUNC AS_GPIO
#endif
#ifndef PC7_FUNC
#define PC7_FUNC AS_GPIO
#endif
#ifndef PULL_WAKEUP_SRC_PC0
#define PULL_WAKEUP_SRC_PC0 0
#endif
#ifndef PULL_WAKEUP_SRC_PC1
#define PULL_WAKEUP_SRC_PC1 0
#endif
#ifndef PULL_WAKEUP_SRC_PC2
#define PULL_WAKEUP_SRC_PC2 0
#endif
#ifndef PULL_WAKEUP_SRC_PC3
#define PULL_WAKEUP_SRC_PC3 0
#endif
#ifndef PULL_WAKEUP_SRC_PC4
#define PULL_WAKEUP_SRC_PC4 0
#endif
#ifndef PULL_WAKEUP_SRC_PC5
#define PULL_WAKEUP_SRC_PC5 0
#endif
#ifndef PULL_WAKEUP_SRC_PC6
#define PULL_WAKEUP_SRC_PC6 0
#endif
#ifndef PULL_WAKEUP_SRC_PC7
#define PULL_WAKEUP_SRC_PC7 0
#endif

/* GPIO GROUP D Default Definition */

#ifndef PD0_INPUT_ENABLE
#define PD0_INPUT_ENABLE 0
#endif
#ifndef PD1_INPUT_ENABLE
#define PD1_INPUT_ENABLE 0
#endif
#ifndef PD2_INPUT_ENABLE
#define PD2_INPUT_ENABLE 0
#endif
#ifndef PD3_INPUT_ENABLE
#define PD3_INPUT_ENABLE 0
#endif
#ifndef PD4_INPUT_ENABLE
#define PD4_INPUT_ENABLE 0
#endif
#ifndef PD5_INPUT_ENABLE
#define PD5_INPUT_ENABLE 0
#endif
#ifndef PD6_INPUT_ENABLE
#define PD6_INPUT_ENABLE 0
#endif
#ifndef PD7_INPUT_ENABLE
#define PD7_INPUT_ENABLE 0
#endif
#ifndef PD0_OUTPUT_ENABLE
#define PD0_OUTPUT_ENABLE 0
#endif
#ifndef PD1_OUTPUT_ENABLE
#define PD1_OUTPUT_ENABLE 0
#endif
#ifndef PD2_OUTPUT_ENABLE
#define PD2_OUTPUT_ENABLE 0
#endif
#ifndef PD3_OUTPUT_ENABLE
#define PD3_OUTPUT_ENABLE 0
#endif
#ifndef PD4_OUTPUT_ENABLE
#define PD4_OUTPUT_ENABLE 0
#endif
#ifndef PD5_OUTPUT_ENABLE
#define PD5_OUTPUT_ENABLE 0
#endif
#ifndef PD6_OUTPUT_ENABLE
#define PD6_OUTPUT_ENABLE 0
#endif
#ifndef PD7_OUTPUT_ENABLE
#define PD7_OUTPUT_ENABLE 0
#endif
#ifndef PD0_DATA_STRENGTH
#define PD0_DATA_STRENGTH 1
#endif
#ifndef PD1_DATA_STRENGTH
#define PD1_DATA_STRENGTH 1
#endif
#ifndef PD2_DATA_STRENGTH
#define PD2_DATA_STRENGTH 1
#endif
#ifndef PD3_DATA_STRENGTH
#define PD3_DATA_STRENGTH 1
#endif
#ifndef PD4_DATA_STRENGTH
#define PD4_DATA_STRENGTH 1
#endif
#ifndef PD5_DATA_STRENGTH
#define PD5_DATA_STRENGTH 1
#endif
#ifndef PD6_DATA_STRENGTH
#define PD6_DATA_STRENGTH 1
#endif
#ifndef PD7_DATA_STRENGTH
#define PD7_DATA_STRENGTH 1
#endif
#ifndef PD0_DATA_OUT
#define PD0_DATA_OUT 0
#endif
#ifndef PD1_DATA_OUT
#define PD1_DATA_OUT 0
#endif
#ifndef PD2_DATA_OUT
#define PD2_DATA_OUT 0
#endif
#ifndef PD3_DATA_OUT
#define PD3_DATA_OUT 0
#endif
#ifndef PD4_DATA_OUT
#define PD4_DATA_OUT 0
#endif
#ifndef PD5_DATA_OUT
#define PD5_DATA_OUT 0
#endif
#ifndef PD6_DATA_OUT
#define PD6_DATA_OUT 0
#endif
#ifndef PD7_DATA_OUT
#define PD7_DATA_OUT 0
#endif
#ifndef PD0_FUNC
#define PD0_FUNC AS_GPIO
#endif
#ifndef PD1_FUNC
#define PD1_FUNC AS_GPIO
#endif
#ifndef PD2_FUNC
#define PD2_FUNC AS_GPIO
#endif
#ifndef PD3_FUNC
#define PD3_FUNC AS_GPIO
#endif
#ifndef PD4_FUNC
#define PD4_FUNC AS_GPIO
#endif
#ifndef PD5_FUNC
#define PD5_FUNC AS_GPIO
#endif
#ifndef PD6_FUNC
#define PD6_FUNC AS_GPIO
#endif
#ifndef PD7_FUNC
#define PD7_FUNC AS_GPIO
#endif
#ifndef PULL_WAKEUP_SRC_PD0
#define PULL_WAKEUP_SRC_PD0 0
#endif
#ifndef PULL_WAKEUP_SRC_PD1
#define PULL_WAKEUP_SRC_PD1 0
#endif
#ifndef PULL_WAKEUP_SRC_PD2
#define PULL_WAKEUP_SRC_PD2 0
#endif
#ifndef PULL_WAKEUP_SRC_PD3
#define PULL_WAKEUP_SRC_PD3 0
#endif
#ifndef PULL_WAKEUP_SRC_PD4
#define PULL_WAKEUP_SRC_PD4 0
#endif
#ifndef PULL_WAKEUP_SRC_PD5
#define PULL_WAKEUP_SRC_PD5 0
#endif
#ifndef PULL_WAKEUP_SRC_PD6
#define PULL_WAKEUP_SRC_PD6 0
#endif
#ifndef PULL_WAKEUP_SRC_PD7
#define PULL_WAKEUP_SRC_PD7 0
#endif

/* GPIO GROUP E Default Definition */

#ifndef PE0_INPUT_ENABLE
#define PE0_INPUT_ENABLE 1 /* MSPI */
#endif
#ifndef PE1_INPUT_ENABLE
#define PE1_INPUT_ENABLE 1 /* MSPI */
#endif
#ifndef PE2_INPUT_ENABLE
#define PE2_INPUT_ENABLE 1 /* MSPI */
#endif
#ifndef PE3_INPUT_ENABLE
#define PE3_INPUT_ENABLE 1 /* MSPI */
#endif
#ifndef PE0_OUTPUT_ENABLE
#define PE0_OUTPUT_ENABLE 0
#endif
#ifndef PE1_OUTPUT_ENABLE
#define PE1_OUTPUT_ENABLE 0
#endif
#ifndef PE2_OUTPUT_ENABLE
#define PE2_OUTPUT_ENABLE 0
#endif
#ifndef PE3_OUTPUT_ENABLE
#define PE3_OUTPUT_ENABLE 0
#endif

#ifndef PE0_DATA_STRENGTH
#define PE0_DATA_STRENGTH 1
#endif
#ifndef PE1_DATA_STRENGTH
#define PE1_DATA_STRENGTH 1
#endif
#ifndef PE2_DATA_STRENGTH
#define PE2_DATA_STRENGTH 1
#endif
#ifndef PE3_DATA_STRENGTH
#define PE3_DATA_STRENGTH 1
#endif

#ifndef PE0_DATA_OUT
#define PE0_DATA_OUT 0
#endif
#ifndef PE1_DATA_OUT
#define PE1_DATA_OUT 0
#endif
#ifndef PE2_DATA_OUT
#define PE2_DATA_OUT 0
#endif
#ifndef PE3_DATA_OUT
#define PE3_DATA_OUT 0
#endif

#ifndef PE0_FUNC
#define PE0_FUNC AS_MSPI
#endif
#ifndef PE1_FUNC
#define PE1_FUNC AS_MSPI
#endif
#ifndef PE2_FUNC
#define PE2_FUNC AS_MSPI
#endif
#ifndef PE3_FUNC
#define PE3_FUNC AS_MSPI
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enum is from telink sdk */

enum gpio_func_e
{
  AS_GPIO   = 0,
  AS_MSPI   = 1,
  AS_SWIRE  = 2,
  AS_UART   = 3,
  AS_I2C    = 4,
  AS_SPI    = 5,
  AS_I2S    = 6,
  AS_AMIC   = 7,
  AS_DMIC   = 8,
  AS_SDM    = 9,
  AS_USB    = 10,
  AS_ADC    = 11,
  AS_CMP    = 12,
  AS_ATS    = 13,
  AS_PWM0   = 20,
  AS_PWM1   = 21,
  AS_PWM2   = 22,
  AS_PWM3   = 23,
  AS_PWM4   = 24,
  AS_PWM5   = 25,
  AS_PWM0_N = 26,
  AS_PWM1_N = 27,
  AS_PWM2_N = 28,
  AS_PWM3_N = 29,
  AS_PWM4_N = 30,
  AS_PWM5_N = 31,
};

#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_DEFAULT_H */
