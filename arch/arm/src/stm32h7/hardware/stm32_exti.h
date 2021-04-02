/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/stm32_memorymap.h"

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_STM32H7_STM32H7X3XX) || defined(CONFIG_STM32H7_STM32H7X7XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_EXTI_INDEX(n)         ((n) >> 5)
#define STM32_EXTI_SHIFT(n)         ((n) & 31)
#define STM32_EXTI_MASK(n)          (1 << STM32_EXTI_SHIFT(n))

#define STM32_EXTI_RTSR_OFFSET(n)   (0x0000 | ((n) & 31))
#define STM32_EXTI_FTSR_OFFSET(n)   (0x0004 | ((n) & 31))
#define STM32_EXTI_SWIER_OFFSET(n)  (0x0008 | ((n) & 31))
#define STM32_EXTI_D3PMR_OFFSET(n)  (0x000c | ((n) & 31))
#define STM32_EXTI_D3PCRL_OFFSET(n) (0x0010 | ((n) & 31))
#define STM32_EXTI_D3PCRH_OFFSET(n) (0x0014 | ((n) & 31))

#define STM32_EXTI_RTSR1_OFFSET     0x0000  /* Rising Trigger selection register 1 */
#define STM32_EXTI_FTSR1_OFFSET     0x0004  /* Falling Trigger selection register 1 */
#define STM32_EXTI_SWIER1_OFFSET    0x0008  /* Software interrupt event register 1 */
#define STM32_EXTI_D3PMR1_OFFSET    0x000c  /* D3 pending mask register 1 */
#define STM32_EXTI_D3PCR1L_OFFSET   0x0010  /* D3 pending clear selection register low 1 */
#define STM32_EXTI_D3PCR1H_OFFSET   0x0014  /* D3 pending clear selection register high 1 */

#define STM32_EXTI_RTSR2_OFFSET     0x0020  /* Rising Trigger selection register 2 */
#define STM32_EXTI_FTSR2_OFFSET     0x0024  /* Falling Trigger selection register 2 */
#define STM32_EXTI_WIER2_OFFSET     0x0028  /* Software interrupt event register 2 */
#define STM32_EXTI_D3PMR2_OFFSET    0x002c  /* D3 pending mask register 2 */
#define STM32_EXTI_D3PCR2L_OFFSET   0x0030  /* D3 pending clear selection register low 2 */
#define STM32_EXTI_D3PCR2H_OFFSET   0x0034  /* D3 pending clear selection register high 2 */

#define STM32_EXTI_RTSR3_OFFSET     0x0040  /* Rising Trigger selection register 3 */
#define STM32_EXTI_FTSR3_OFFSET     0x0044  /* Falling Trigger selection register 3 */
#define STM32_EXTI_WIER3_OFFSET     0x0048  /* Software interrupt event register 3 */
#define STM32_EXTI_D3PMR3_OFFSET    0x004c  /* D3 pending mask register 3 */
#define STM32_EXTI_D3PCR3L_OFFSET   0x0050  /* D3 pending clear selection register low 3 */
#define STM32_EXTI_D3PCR3H_OFFSET   0x0054  /* D3 pending clear selection register high 3 */

#define STM32_EXTI_CPUIMR_OFFSET(n) (0x0080 | (((n) >> 1 )& 15))
#define STM32_EXTI_CPUEMR_OFFSET(n) (0x0084 | (((n) >> 1 )& 15))
#define STM32_EXTI_CPUPR_OFFSET(n)  (0x0088 | (((n) >> 1 )& 15))

#define STM32_EXTI_CPUIMR1_OFFSET   0x0080  /* EXTI interrupt mask register 1 */
#define STM32_EXTI_CPUEMR1_OFFSET   0x0084  /* EXTI event mask register 1 */
#define STM32_EXTI_CPUPR1_OFFSET    0x0088  /* EXTI pending register 1 */

#define STM32_EXTI_CPUIMR2_OFFSET   0x0090  /* EXTI interrupt mask register 2 */
#define STM32_EXTI_CPUEMR2_OFFSET   0x0094  /* EXTI event mask register 2 */
#define STM32_EXTI_CPUPR2_OFFSET    0x0098  /* EXTI pending register 2 */

#define STM32_EXTI_CPUIMR3_OFFSET   0x00a0  /* EXTI interrupt mask register 3 */
#define STM32_EXTI_CPUEMR3_OFFSET   0x00a4  /* EXTI event mask register 3 */
#define STM32_EXTI_CPUPR3_OFFSET    0x00a8  /* EXTI pending register 3 */

/* Register Addresses *******************************************************/

#define STM32_EXTI_RTSR(n)          (STM32_EXTI_BASE + STM32_EXTI_RTSR_OFFSET(n))
#define STM32_EXTI_FTSR(n)          (STM32_EXTI_BASE + STM32_EXTI_FTSR_OFFSET(n))
#define STM32_EXTI_SWIER(n)         (STM32_EXTI_BASE + STM32_EXTI_SWIER_OFFSET(n))
#define STM32_EXTI_D3PMR(n)         (STM32_EXTI_BASE + STM32_EXTI_D3PMR_OFFSET(n))
#define STM32_EXTI_D3PCR(n)         (STM32_EXTI_BASE + STM32_EXTI_D3PCRL_OFFSET(n))
#define STM32_EXTI_D3PCRH(n)        (STM32_EXTI_BASE + STM32_EXTI_D3PCRH_OFFSET(n))

#define STM32_EXTI_RTSR1            (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1            (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1           (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_D3PMR1           (STM32_EXTI_BASE + STM32_EXTI_D3PMR1_OFFSET)
#define STM32_EXTI_D3PCR1L          (STM32_EXTI_BASE + STM32_EXTI_D3PCR1L_OFFSET)
#define STM32_EXTI_D3PCR1H          (STM32_EXTI_BASE + STM32_EXTI_D3PCR1H_OFFSET)

#define STM32_EXTI_RTSR2            (STM32_EXTI_BASE + STM32_EXTI_RTSR2_OFFSET)
#define STM32_EXTI_FTSR2            (STM32_EXTI_BASE + STM32_EXTI_FTSR2_OFFSET)
#define STM32_EXTI_WIER2            (STM32_EXTI_BASE + STM32_EXTI_WIER2_OFFSET)
#define STM32_EXTI_D3PMR2           (STM32_EXTI_BASE + STM32_EXTI_D3PMR2_OFFSET)
#define STM32_EXTI_D3PCR2L          (STM32_EXTI_BASE + STM32_EXTI_D3PCR2L_OFFSET)
#define STM32_EXTI_D3PCR2H          (STM32_EXTI_BASE + STM32_EXTI_D3PCR2H_OFFSET)

#define STM32_EXTI_RTSR3            (STM32_EXTI_BASE + STM32_EXTI_RTSR3_OFFSET)
#define STM32_EXTI_FTSR3            (STM32_EXTI_BASE + STM32_EXTI_FTSR3_OFFSET)
#define STM32_EXTI_WIER3            (STM32_EXTI_BASE + STM32_EXTI_WIER3_OFFSET)
#define STM32_EXTI_D3PMR3           (STM32_EXTI_BASE + STM32_EXTI_D3PMR3_OFFSET)
#define STM32_EXTI_D3PCR3L          (STM32_EXTI_BASE + STM32_EXTI_D3PCR3L_OFFSET)
#define STM32_EXTI_D3PCR3H          (STM32_EXTI_BASE + STM32_EXTI_D3PCR3H_OFFSET)

#define STM32_EXTI_CPUIMR(n)        (STM32_EXTI_BASE + STM32_EXTI_CPUIMR_OFFSET(n))
#define STM32_EXTI_CPUEMR(n)        (STM32_EXTI_BASE + STM32_EXTI_CPUEMR_OFFSET(n))
#define STM32_EXTI_CPUPR(n)         (STM32_EXTI_BASE + STM32_EXTI_CPUPR_OFFSET(n))

#define STM32_EXTI_CPUIMR1          (STM32_EXTI_BASE + STM32_EXTI_CPUIMR1_OFFSET)
#define STM32_EXTI_CPUEMR1          (STM32_EXTI_BASE + STM32_EXTI_CPUEMR1_OFFSET)
#define STM32_EXTI_CPUPR1           (STM32_EXTI_BASE + STM32_EXTI_CPUPR1_OFFSET)

#define STM32_EXTI_CPUIMR2          (STM32_EXTI_BASE + STM32_EXTI_CPUIMR2_OFFSET)
#define STM32_EXTI_CPUEMR2          (STM32_EXTI_BASE + STM32_EXTI_CPUEMR2_OFFSET)
#define STM32_EXTI_CPUPR2           (STM32_EXTI_BASE + STM32_EXTI_CPUPR2_OFFSET)

#define STM32_EXTI_CPUIMR3          (STM32_EXTI_BASE + STM32_EXTI_CPUIMR3_OFFSET)
#define STM32_EXTI_CPUEMR3          (STM32_EXTI_BASE + STM32_EXTI_CPUEMR3_OFFSET)
#define STM32_EXTI_CPUPR3           (STM32_EXTI_BASE + STM32_EXTI_CPUPR3_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Rising Trigger selection register 1-3,
 * Falling Trigger selection register 1-3,
 * Software interrupt event register 1-3,
 * and D3 pending mask register 1-3:
 *
 * Bit encoded, each bit corresponding to one of the input events enumerated
 * below.
 */

#define EXIT_EVENT(n)               STM32_EXTI_MASK(n)

#define EXTI_RTC_ALARM              (1 << 17) /* EXTI line 17 = RTC Alarm event */
#define EXTI_RTC_TAMPER             (1 << 18) /* EXTI line 18 = RTC tamper, RTC timestamp, RCC LSECSS */
#define EXTI_RTC_WAKEUP             (1 << 19) /* EXTI line 19 = RTC wakeup timer */

/* D3 pending clear selection register low 1-3.
 * D3 pending clear selection register high 1-3
 *
 * Two bit fields for each of the input events enumerated below.
 */

#define EXTI_D2PCR_DMACH6           (0)  /* DMA ch6 event selected as D3 domain pendclear source */
#define EXTI_D2PCR_DMACH7           (1)  /* DMA ch7 event selected as D3 domain pendclear source */
#define EXTI_D2PCR_LPTIM4           (2)  /* LPTIM4 out selected as D3 domain pendclear source */
#define EXTI_D2PCR_LPTIM5           (3)  /* LPTIM5 out selected as D3 domain pendclear source */

#define EXTI_D3PCRL_SHIFT(n)        (((n) & 0xf) << 1)  /* For (n & 31) < 16 */
#define EXTI_D3PCRL_MASK(n)         (3 << EXTI_D3PCRL_SHIFT(n))
#  define EXTI_D3PCRL(n,v)          ((uint32_t)(v) << EXTI_D3PCRL_SHIFT(n))

#define EXTI_D3PCRH_SHIFT(n)        (((n) & 0xf) << 1)  /* For (n & 31) >= 16 */
#define EXTI_D3PCRH_MASK(n)         (3 << EXTI_D3PCRL_SHIFT(n))
#  define EXTI_D3PCRH(n,v)          ((uint32_t)(v) << EXTI_D3PCRL_SHIFT(n))

/* EXTI event input mapping *************************************************/

#define EXTI_EVENT_EXTI(n)          (1 << (n) /* 0-15 EXTI[15:0] */
#define EXTI_EVENT_PVDAVD           16        /* PVD and AVD */
#define EXTI_EVENT_RTCALARM         17        /* RTC alarms */
#define EXTI_EVENT_RTC              18        /* RTC tamper, RTC timestamp, RCC LSECSS */
#define EXTI_EVENT_RTCWKUP          19        /* RTC wakeup timer */
#define EXTI_EVENT_COMP1            20        /* COMP1 */
#define EXTI_EVENT_COMP2            21        /* COMP2 */
#define EXTI_EVENT_I2C1WKUP         22        /* I2C1 wakeup */
#define EXTI_EVENT_I2C2WKUP         23        /* I2C2 wakeup */
#define EXTI_EVENT_I2C3WKUP         24        /* I2C3 wakeup */
#define EXTI_EVENT_I2C4WKUP         25        /* I2C4 wakeup */
#define EXTI_EVENT_USART1WKUP       26        /* USART1 wakeup */
#define EXTI_EVENT_USART2WKUP       27        /* USART2 wakeup */
#define EXTI_EVENT_USART3WKUP       28        /* USART3 wakeup */
#define EXTI_EVENT_USART6WKUP       29        /* USART6 wakeup */
#define EXTI_EVENT_UART4WKUP        30        /* UART4 wakeup */
#define EXTI_EVENT_UART5WKUP        31        /* UART5 wakeup */
#define EXTI_EVENT_UART7WKUP        32        /* UART7 wakeup */
#define EXTI_EVENT_UART8WKUP        33        /* UART8 wakeup */
#define EXTI_EVENT_LPUARTRXWKUP     34        /* LPUART1 RX wakeup */
#define EXTI_EVENT_LPUARTTXWKUP     35        /* LPUART1 TX wakeup */
#define EXTI_EVENT_SPI1WKUP         36        /* SPI1 wakeup */
#define EXTI_EVENT_SPI2WKUP         37        /* SPI2 wakeup */
#define EXTI_EVENT_SPI3WKUP         38        /* SPI3 wakeup */
#define EXTI_EVENT_SPI4WKUP         39        /* SPI4 wakeup */
#define EXTI_EVENT_SPI5WKUP         40        /* SPI5 wakeup */
#define EXTI_EVENT_SPI6WKUP         41        /* SPI6 wakeup */
#define EXTI_EVENT_MDIOWKUP         42        /* MDIO wakeup */
#define EXTI_EVENT_USB1WKUP         43        /* USB1 wakeup */
#define EXTI_EVENT_USB2WKUP         44        /* USB2 wakeup */
#define EXTI_EVENT_LPTIM1WKUP       47        /* LPTIM1 wakeup */
#define EXTI_EVENT_LPTIM2WKUP       48        /* LPTIM2 wakeup */
#define EXTI_EVENT_LPTM2OUT         49        /* LPTIM2 output */
#define EXTI_EVENT_LPTIM3WKUP       50        /* LPTIM3 wakeup */
#define EXTI_EVENT_LPTIM3OUT        51        /* LPTIM3 output */
#define EXTI_EVENT_LPTIM4WKUP       52        /* LPTIM4 wakeup */
#define EXTI_EVENT_LPTIM5WKUP       53        /* LPTIM5 wakeup */
#define EXTI_EVENT_SWPMIWKUP        54        /* SWPMI wakeup */
#define EXTI_EVENT_WKUP1            55        /* WKUP1 */
#define EXTI_EVENT_WKUP2            56        /* WKUP2 */
#define EXTI_EVENT_WKUP3            57        /* WKUP3 */
#define EXTI_EVENT_WKUP4            58        /* WKUP4 */
#define EXTI_EVENT_WKUP5            59        /* WKUP5 */
#define EXTI_EVENT_WKUP6            60        /* WKUP6 */
#define EXTI_EVENT_RCC              61        /* RCC interrupt */
#define EXTI_EVENT_I2C4EV           62        /* I2C4 Event interrupt */
#define EXTI_EVENT_I2C4ERR          63        /* I2C4 Error interrupt */
#define EXTI_EVENT_LPUART1          64        /* LPUART1 global Interrupt */
#define EXTI_EVENT_SPI6             65        /* SPI6 interrupt */
#define EXTI_EVENT_BDMA0            66        /* BDMA CH0 interrupt */
#define EXTI_EVENT_BDMA1            67        /* BDMA CH1 interrupt */
#define EXTI_EVENT_BDMA2            68        /* BDMA CH2 interrupt */
#define EXTI_EVENT_BDMA3            69        /* BDMA CH3 interrupt */
#define EXTI_EVENT_BDMA4            70        /* BDMA CH4 interrupt */
#define EXTI_EVENT_RDMB5            71        /* BDMA CH5 interrupt */
#define EXTI_EVENT_BDMA6            72        /* BDMA CH6 interrupt */
#define EXTI_EVENT_BDMA7            73        /* BDMA CH7 interrupt */
#define EXTI_EVENT_DMAMUX2          74        /* DMAMUX2 interrupt */
#define EXTI_EVENT_ADC3             75        /* ADC3 interrupt */
#define EXTI_EVENT_SAI4             76        /* SAI4 interrupt */
#define EXTI_EVENT_CECWKUP          85        /* HDMI-CEC wakeup */
#define EXTI_EVENT_ETHWKUP          86        /* Ethernet wakeup */
#define EXTI_EVENT_HSECSS           87        /* HSECSS interrupt */

#endif /* CONFIG_STM32H7_STM32H7X3XX || CONFIG_STM32H7_STM32H7X7XX */
#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_EXTI_H */
