/****************************************************************************
 * arch/arm/include/phy62xx/phy62xx_irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_PHYPLUS_PHY62XX_IRQ_H
#define __ARCH_ARM_INCLUDE_PHYPLUS_PHY62XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/phy62xx/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in nuttx/arch/arm/include/stm32f0l0g0/irq.h
 */

#define PHY62XX_IRQ_BB_IRQn       (PHY62XX_IRQ_EXTINT + 4)         /* 4: RCC and CRS */
#define PHY62XX_IRQ_KSCAN_IRQn       (PHY62XX_IRQ_EXTINT + 5)      /* 5: EXTI0_1 */
#define PHY62XX_IRQ_RTC_IRQn       (PHY62XX_IRQ_EXTINT + 6)        /* 6: EXTI2_3 */
#define PHY62XX_IRQ_WDT_IRQn       (PHY62XX_IRQ_EXTINT + 10)       /* 10: DMA2_CH2 */
#define PHY62XX_IRQ_UART0_IRQn       (PHY62XX_IRQ_EXTINT + 11)     /* 11: DMA1_CH4 */
#define PHY62XX_IRQ_I2C0_IRQn           (PHY62XX_IRQ_EXTINT + 12)  /* 12: ADC */
#define PHY62XX_IRQ_I2C1_IRQn      (PHY62XX_IRQ_EXTINT + 13)       /* 13: TIM1_BRK_UP_TRG_COM */
#define PHY62XX_IRQ_SPI0_IRQn       (PHY62XX_IRQ_EXTINT + 14)      /* 14: TIM1_CC */
#define PHY62XX_IRQ_SPI1_IRQn          (PHY62XX_IRQ_EXTINT + 15)   /* 15: TIM2 */
#define PHY62XX_IRQ_GPIO_IRQn          (PHY62XX_IRQ_EXTINT + 16)   /* 16: TIM3 */
#define PHY62XX_IRQ_UART1_IRQn          (PHY62XX_IRQ_EXTINT + 17)  /* 17: TIM6 */
#define PHY62XX_IRQ_SPIF_IRQn           (PHY62XX_IRQ_EXTINT + 18)  /* 17: DAC */
#define PHY62XX_IRQ_DMAC_IRQn          (PHY62XX_IRQ_EXTINT + 19)   /* 18: TIM7 */
#define PHY62XX_IRQ_TIM1_IRQn         (PHY62XX_IRQ_EXTINT + 20)    /* 20: TIM15 */
#define PHY62XX_IRQ_TIM2_IRQn         (PHY62XX_IRQ_EXTINT + 21)    /* 21: TIM16 */
#define PHY62XX_IRQ_TIM3_IRQn         (PHY62XX_IRQ_EXTINT + 22)    /* 22: TIM17 */
#define PHY62XX_IRQ_TIM4_IRQn           (PHY62XX_IRQ_EXTINT + 23)  /* 23: I2C1 */
#define PHY62XX_IRQ_TIM5_IRQn          (PHY62XX_IRQ_EXTINT + 24)   /* 24: I2C2 */
#define PHY62XX_IRQ_TIM6_IRQn          (PHY62XX_IRQ_EXTINT + 25)   /* 25: SPI1 */

#define PHY62XX_IRQ_AES_IRQn        (PHY62XX_IRQ_EXTINT + 28)      /* 28: USART2 */
#define PHY62XX_IRQ_ADCC_IRQn        (PHY62XX_IRQ_EXTINT + 29)     /* 29: USART3 */
#define PHY62XX_IRQ_QDEC_IRQn           (PHY62XX_IRQ_EXTINT + 30)  /* 30: HDMI CAN */
#define PHY62XX_IRQ_RNG_IRQn           (PHY62XX_IRQ_EXTINT + 31)   /* 31: USB */

#define PHY62XX_IRQ_NEXTINT       (32)                             /* 32 external interrupts */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_STM32F0L0G0_STM32F0_IRQ_H */
