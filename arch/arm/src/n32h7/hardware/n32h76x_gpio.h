/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_gpio.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_GPIO_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/n32h7/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_GPIO_MODER_OFFSET   0x0000  /* Port mode register */
#define N32_GPIO_OTYPER_OFFSET  0x0004  /* Output type register */
#define N32_GPIO_SR_OFFSET      0x0008  /* Output speed register */
#define N32_GPIO_PUPD_OFFSET    0x000c  /* Pull-up/pull-down register */
#define N32_GPIO_PID_OFFSET     0x0010  /* Input data register */
#define N32_GPIO_POD_OFFSET     0x0014  /* Output data register */
#define N32_GPIO_PBSC_OFFSET    0x0018  /* Bit set/reset register */
#define N32_GPIO_PLOCK_OFFSET   0x001c  /* Configuration lock register */
#define N32_GPIO_AFL_OFFSET     0x0020  /* Alternate function low register */
#define N32_GPIO_AFH_OFFSET     0x0024  /* Alternate function high register */
#define N32_GPIO_PBC_OFFSET     0x0028  /* Bit clear register */
#define N32_GPIO_DS_OFFSET      0x002c  /* Driver strength configuration register */

/* Register Addresses *******************************************************/

#if N32H7_NGPIO > 0
#  define N32_GPIOA_MODER          (N32_GPIOA_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOA_OTYPER         (N32_GPIOA_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOA_SR             (N32_GPIOA_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOA_PUPD           (N32_GPIOA_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOA_PID            (N32_GPIOA_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOA_POD            (N32_GPIOA_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOA_PBSC           (N32_GPIOA_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOA_PLOCK          (N32_GPIOA_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOA_AFL            (N32_GPIOA_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOA_AFH            (N32_GPIOA_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOA_PBC            (N32_GPIOA_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOA_DS             (N32_GPIOA_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 1
#  define N32_GPIOB_MODER          (N32_GPIOB_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOB_OTYPER         (N32_GPIOB_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOB_SR             (N32_GPIOB_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOB_PUPD           (N32_GPIOB_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOB_PID            (N32_GPIOB_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOB_POD            (N32_GPIOB_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOB_PBSC           (N32_GPIOB_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOB_PLOCK          (N32_GPIOB_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOB_AFL            (N32_GPIOB_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOB_AFH            (N32_GPIOB_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOB_PBC            (N32_GPIOB_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOB_DS             (N32_GPIOB_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 2
#  define N32_GPIOC_MODER          (N32_GPIOC_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOC_OTYPER         (N32_GPIOC_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOC_SR             (N32_GPIOC_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOC_PUPD           (N32_GPIOC_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOC_PID            (N32_GPIOC_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOC_POD            (N32_GPIOC_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOC_PBSC           (N32_GPIOC_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOC_PLOCK          (N32_GPIOC_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOC_AFL            (N32_GPIOC_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOC_AFH            (N32_GPIOC_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOC_PBC            (N32_GPIOC_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOC_DS             (N32_GPIOC_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 3
#  define N32_GPIOD_MODER          (N32_GPIOD_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOD_OTYPER         (N32_GPIOD_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOD_SR             (N32_GPIOD_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOD_PUPD           (N32_GPIOD_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOD_PID            (N32_GPIOD_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOD_POD            (N32_GPIOD_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOD_PBSC           (N32_GPIOD_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOD_PLOCK          (N32_GPIOD_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOD_AFL            (N32_GPIOD_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOD_AFH            (N32_GPIOD_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOD_PBC            (N32_GPIOD_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOD_DS             (N32_GPIOD_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 4
#  define N32_GPIOE_MODER          (N32_GPIOE_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOE_OTYPER         (N32_GPIOE_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOE_SR             (N32_GPIOE_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOE_PUPD           (N32_GPIOE_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOE_PID            (N32_GPIOE_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOE_POD            (N32_GPIOE_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOE_PBSC           (N32_GPIOE_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOE_PLOCK          (N32_GPIOE_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOE_AFL            (N32_GPIOE_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOE_AFH            (N32_GPIOE_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOE_PBC            (N32_GPIOE_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOE_DS             (N32_GPIOE_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 5
#  define N32_GPIOF_MODER          (N32_GPIOF_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOF_OTYPER         (N32_GPIOF_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOF_SR             (N32_GPIOF_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOF_PUPD           (N32_GPIOF_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOF_PID            (N32_GPIOF_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOF_POD            (N32_GPIOF_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOF_PBSC           (N32_GPIOF_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOF_PLOCK          (N32_GPIOF_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOF_AFL            (N32_GPIOF_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOF_AFH            (N32_GPIOF_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOF_PBC            (N32_GPIOF_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOF_DS             (N32_GPIOF_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 6
#  define N32_GPIOG_MODER          (N32_GPIOG_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOG_OTYPER         (N32_GPIOG_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOG_SR             (N32_GPIOG_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOG_PUPD           (N32_GPIOG_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOG_PID            (N32_GPIOG_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOG_POD            (N32_GPIOG_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOG_PBSC           (N32_GPIOG_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOG_PLOCK          (N32_GPIOG_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOG_AFL            (N32_GPIOG_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOG_AFH            (N32_GPIOG_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOG_PBC            (N32_GPIOG_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOG_DS             (N32_GPIOG_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 7
#  define N32_GPIOH_MODER          (N32_GPIOH_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOH_OTYPER         (N32_GPIOH_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOH_SR             (N32_GPIOH_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOH_PUPD           (N32_GPIOH_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOH_PID            (N32_GPIOH_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOH_POD            (N32_GPIOH_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOH_PBSC           (N32_GPIOH_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOH_PLOCK          (N32_GPIOH_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOH_AFL            (N32_GPIOH_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOH_AFH            (N32_GPIOH_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOH_PBC            (N32_GPIOH_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOH_DS             (N32_GPIOH_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 8
#  define N32_GPIOI_MODER          (N32_GPIOI_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOI_OTYPER         (N32_GPIOI_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOI_SR             (N32_GPIOI_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOI_PUPD           (N32_GPIOI_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOI_PID            (N32_GPIOI_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOI_POD            (N32_GPIOI_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOI_PBSC           (N32_GPIOI_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOI_PLOCK          (N32_GPIOI_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOI_AFL            (N32_GPIOI_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOI_AFH            (N32_GPIOI_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOI_PBC            (N32_GPIOI_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOI_DS             (N32_GPIOI_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 9
#  define N32_GPIOJ_MODER          (N32_GPIOJ_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOJ_OTYPER         (N32_GPIOJ_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOJ_SR             (N32_GPIOJ_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOJ_PUPD           (N32_GPIOJ_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOJ_PID            (N32_GPIOJ_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOJ_POD            (N32_GPIOJ_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOJ_PBSC           (N32_GPIOJ_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOJ_PLOCK          (N32_GPIOJ_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOJ_AFL            (N32_GPIOJ_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOJ_AFH            (N32_GPIOJ_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOJ_PBC            (N32_GPIOJ_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOJ_DS             (N32_GPIOJ_BASE + N32_GPIO_DS_OFFSET)
#endif

#if N32H7_NGPIO > 10
#  define N32_GPIOK_MODER          (N32_GPIOK_BASE + N32_GPIO_MODER_OFFSET)
#  define N32_GPIOK_OTYPER         (N32_GPIOK_BASE + N32_GPIO_OTYPER_OFFSET)
#  define N32_GPIOK_SR             (N32_GPIOK_BASE + N32_GPIO_SR_OFFSET)
#  define N32_GPIOK_PUPD           (N32_GPIOK_BASE + N32_GPIO_PUPD_OFFSET)
#  define N32_GPIOK_PID            (N32_GPIOK_BASE + N32_GPIO_PID_OFFSET)
#  define N32_GPIOK_POD            (N32_GPIOK_BASE + N32_GPIO_POD_OFFSET)
#  define N32_GPIOK_PBSC           (N32_GPIOK_BASE + N32_GPIO_PBSC_OFFSET)
#  define N32_GPIOK_PLOCK          (N32_GPIOK_BASE + N32_GPIO_PLOCK_OFFSET)
#  define N32_GPIOK_AFL            (N32_GPIOK_BASE + N32_GPIO_AFL_OFFSET)
#  define N32_GPIOK_AFH            (N32_GPIOK_BASE + N32_GPIO_AFH_OFFSET)
#  define N32_GPIOK_PBC            (N32_GPIOK_BASE + N32_GPIO_PBC_OFFSET)
#  define N32_GPIOK_DS             (N32_GPIOK_BASE + N32_GPIO_DS_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT           (0) /* Input */
#define GPIO_MODER_OUTPUT          (1) /* General purpose output mode */
#define GPIO_MODER_ALT             (2) /* Alternate mode */
#define GPIO_MODER_ANALOG          (3) /* Analog mode */

#define GPIO_MODER_SHIFT(n)       ((n) * 2)
#define GPIO_MODER_MASK(n)        (3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT          (0)
#define GPIO_MODER0_MASK           (3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT          (2)
#define GPIO_MODER1_MASK           (3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT          (4)
#define GPIO_MODER2_MASK           (3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT          (6)
#define GPIO_MODER3_MASK           (3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT          (8)
#define GPIO_MODER4_MASK           (3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT          (10)
#define GPIO_MODER5_MASK           (3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT          (12)
#define GPIO_MODER6_MASK           (3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT          (14)
#define GPIO_MODER7_MASK           (3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT          (16)
#define GPIO_MODER8_MASK           (3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT          (18)
#define GPIO_MODER9_MASK           (3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT         (20)
#define GPIO_MODER10_MASK          (3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT         (22)
#define GPIO_MODER11_MASK          (3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT         (24)
#define GPIO_MODER12_MASK          (3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT         (26)
#define GPIO_MODER13_MASK          (3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT         (28)
#define GPIO_MODER14_MASK          (3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT         (30)
#define GPIO_MODER15_MASK          (3 << GPIO_MODER15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_OD(n)          (1 << (n)) /* 1=Output open-drain */
#define GPIO_OTYPER_PP(n)          (0)        /* 0=Output push-pull */

/* GPIO port slew rate register */

#define GPIO_SR_SLOW_SLEW(n)       (1 << (n)) /* 1=Slow slew rate */
#define GPIO_SR_FAST_SLEW(n)       (0)        /* 0=Fast slew rate */

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE            (0) /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP          (1) /* Pull-up */
#define GPIO_PUPDR_PULLDOWN        (2) /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)        ((n) << 1)
#define GPIO_PUPDR_MASK(n)         (3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT          (0)
#define GPIO_PUPDR0_MASK           (3 << GPIO_PUPDR0_SHIFT)
#define GPIO_PUPDR1_SHIFT          (2)
#define GPIO_PUPDR1_MASK           (3 << GPIO_PUPDR1_SHIFT)
#define GPIO_PUPDR2_SHIFT          (4)
#define GPIO_PUPDR2_MASK           (3 << GPIO_PUPDR2_SHIFT)
#define GPIO_PUPDR3_SHIFT          (6)
#define GPIO_PUPDR3_MASK           (3 << GPIO_PUPDR3_SHIFT)
#define GPIO_PUPDR4_SHIFT          (8)
#define GPIO_PUPDR4_MASK           (3 << GPIO_PUPDR4_SHIFT)
#define GPIO_PUPDR5_SHIFT          (10)
#define GPIO_PUPDR5_MASK           (3 << GPIO_PUPDR5_SHIFT)
#define GPIO_PUPDR6_SHIFT          (12)
#define GPIO_PUPDR6_MASK           (3 << GPIO_PUPDR6_SHIFT)
#define GPIO_PUPDR7_SHIFT          (14)
#define GPIO_PUPDR7_MASK           (3 << GPIO_PUPDR7_SHIFT)
#define GPIO_PUPDR8_SHIFT          (16)
#define GPIO_PUPDR8_MASK           (3 << GPIO_PUPDR8_SHIFT)
#define GPIO_PUPDR9_SHIFT          (18)
#define GPIO_PUPDR9_MASK           (3 << GPIO_PUPDR9_SHIFT)
#define GPIO_PUPDR10_SHIFT         (20)
#define GPIO_PUPDR10_MASK          (3 << GPIO_PUPDR10_SHIFT)
#define GPIO_PUPDR11_SHIFT         (22)
#define GPIO_PUPDR11_MASK          (3 << GPIO_PUPDR11_SHIFT)
#define GPIO_PUPDR12_SHIFT         (24)
#define GPIO_PUPDR12_MASK          (3 << GPIO_PUPDR12_SHIFT)
#define GPIO_PUPDR13_SHIFT         (26)
#define GPIO_PUPDR13_MASK          (3 << GPIO_PUPDR13_SHIFT)
#define GPIO_PUPDR14_SHIFT         (28)
#define GPIO_PUPDR14_MASK          (3 << GPIO_PUPDR14_SHIFT)
#define GPIO_PUPDR15_SHIFT         (30)
#define GPIO_PUPDR15_MASK          (3 << GPIO_PUPDR15_SHIFT)

/* GPIO port input data register */

#define GPIO_PID(n)                (1 << (n))

/* GPIO port output data register */

#define GPIO_POD(n)                (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_PBSC_SET(n)           (1 << (n))
#define GPIO_PBSC_RESET(n)         (1 << (n + 16))

/* GPIO port configuration lock register */

#define GPIO_PLOCKR(n)             (1 << (n))
#define GPIO_PLOCKK                (1 << 16) /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_AFR_SHIFT(n)          ((n) << 2)
#define GPIO_AFR_MASK(n)           (15 << GPIO_AFR_SHIFT(n))

#define GPIO_AFRL0_SHIFT           (0)
#define GPIO_AFRL0_MASK            (15 << GPIO_AFRL0_SHIFT)
#define GPIO_AFRL1_SHIFT           (4)
#define GPIO_AFRL1_MASK            (15 << GPIO_AFRL1_SHIFT)
#define GPIO_AFRL2_SHIFT           (8)
#define GPIO_AFRL2_MASK            (15 << GPIO_AFRL2_SHIFT)
#define GPIO_AFRL3_SHIFT           (12)
#define GPIO_AFRL3_MASK            (15 << GPIO_AFRL3_SHIFT)
#define GPIO_AFRL4_SHIFT           (16)
#define GPIO_AFRL4_MASK            (15 << GPIO_AFRL4_SHIFT)
#define GPIO_AFRL5_SHIFT           (20)
#define GPIO_AFRL5_MASK            (15 << GPIO_AFRL5_SHIFT)
#define GPIO_AFRL6_SHIFT           (24)
#define GPIO_AFRL6_MASK            (15 << GPIO_AFRL6_SHIFT)
#define GPIO_AFRL7_SHIFT           (28)
#define GPIO_AFRL7_MASK            (15 << GPIO_AFRL7_SHIFT)

#define GPIO_AFRH8_SHIFT           (0)
#define GPIO_AFRH8_MASK            (15 << GPIO_AFRH8_SHIFT)
#define GPIO_AFRH9_SHIFT           (4)
#define GPIO_AFRH9_MASK            (15 << GPIO_AFRH9_SHIFT)
#define GPIO_AFRH10_SHIFT          (8)
#define GPIO_AFRH10_MASK           (15 << GPIO_AFRH10_SHIFT)
#define GPIO_AFRH11_SHIFT          (12)
#define GPIO_AFRH11_MASK           (15 << GPIO_AFRH11_SHIFT)
#define GPIO_AFRH12_SHIFT          (16)
#define GPIO_AFRH12_MASK           (15 << GPIO_AFRH12_SHIFT)
#define GPIO_AFRH13_SHIFT          (20)
#define GPIO_AFRH13_MASK           (15 << GPIO_AFRH13_SHIFT)
#define GPIO_AFRH14_SHIFT          (24)
#define GPIO_AFRH14_MASK           (15 << GPIO_AFRH14_SHIFT)
#define GPIO_AFRH15_SHIFT          (28)
#define GPIO_AFRH15_MASK           (15 << GPIO_AFRH15_SHIFT)

/* GPIO port data bit clear register */

#define GPIO_PBC(n)                (1 << (n))

/* GPIO port driver strength configuration register */

#define GPIO_DS_2mA                 (0) /* Drive strength is 2mA  */
#define GPIO_DS_4mA                 (2) /* Drive strength is 4mA  */
#define GPIO_DS_8mA                 (1) /* Drive strength is 8mA  */
#define GPIO_DS_12mA                (3) /* Drive strength is 12mA */

#define GPIO_DS_SHIFT(n)            ((n) << 1)
#define GPIO_DS_MASK(n)             (3 << GPIO_DS_SHIFT(n))

#define GPIO_DS_SHIFT0              (0)
#define GPIO_DS_MASK0               (3 << GPIO_DS_SHIFT0)
#define GPIO_DS_SHIFT1              (2)
#define GPIO_DS_MASK1               (3 << GPIO_DS_SHIFT1)
#define GPIO_DS_SHIFT2              (4)
#define GPIO_DS_MASK2               (3 << GPIO_DS_SHIFT2)
#define GPIO_DS_SHIFT3              (6)
#define GPIO_DS_MASK3               (3 << GPIO_DS_SHIFT3)
#define GPIO_DS_SHIFT4              (8)
#define GPIO_DS_MASK4               (3 << GPIO_DS_SHIFT4)
#define GPIO_DS_SHIFT5              (10)
#define GPIO_DS_MASK5               (3 << GPIO_DS_SHIFT5)
#define GPIO_DS_SHIFT6              (12)
#define GPIO_DS_MASK6               (3 << GPIO_DS_SHIFT6)
#define GPIO_DS_SHIFT7              (14)
#define GPIO_DS_MASK7               (3 << GPIO_DS_SHIFT7)
#define GPIO_DS_SHIFT8              (16)
#define GPIO_DS_MASK8               (3 << GPIO_DS_SHIFT8)
#define GPIO_DS_SHIFT9              (18)
#define GPIO_DS_MASK9               (3 << GPIO_DS_SHIFT9)
#define GPIO_DS_SHIFT10             (20)
#define GPIO_DS_MASK10              (3 << GPIO_DS_SHIFT10)
#define GPIO_DS_SHIFT11             (22)
#define GPIO_DS_MASK11              (3 << GPIO_DS_SHIFT11)
#define GPIO_DS_SHIFT12             (24)
#define GPIO_DS_MASK12              (3 << GPIO_DS_SHIFT12)
#define GPIO_DS_SHIFT13             (26)
#define GPIO_DS_MASK13              (3 << GPIO_DS_SHIFT13)
#define GPIO_DS_SHIFT14             (28)
#define GPIO_DS_MASK14              (3 << GPIO_DS_SHIFT14)
#define GPIO_DS_SHIFT15             (30)
#define GPIO_DS_MASK15              (3 << GPIO_DS_SHIFT15)

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_GPIO_H */
