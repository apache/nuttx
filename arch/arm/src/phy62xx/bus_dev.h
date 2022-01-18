/****************************************************************************
 * arch/arm/src/phy62xx/bus_dev.h
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

/****************************************************************************
 *    Filename:       bus_dev.h
 *    Revised:
 *    Revision:
 *
 *    Description:    This file contains the SoC MCU relate definitions
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_PHY62XX_BUS_DEV_H
#define __ARCH_ARM_SRC_PHY62XX_BUS_DEV_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include "mcu.h"

#define PHY_MCU_TYPE MCU_BUMBEE_M0

enum
{
    RSTC_COLD_UP = 0,
    RSTC_WARM_UP = 1,
    RSTC_OFF_MODE = 2,
    RSTC_WAKE_IO = 3,
    RSTC_WAKE_RTC = 4,
    RSTC_WARM_NDWC = 5  /* user mode, no dwc */
};

/* ----  Interrupt Number Definition  ---- */

typedef enum IRQn
{
    /* ----  Cortex-M0 Processor Exceptions Numbers  ---- */

    NonMaskableInt_IRQn           = -14,      /*  2 Non Maskable Interrupt */
    HardFault_IRQn                = -13,      /*  3 HardFault Interrupt */
    SVCall_IRQn                   =  -5,      /* 11 SV Call Interrupt */
    PendSV_IRQn                   =  -2,      /* 14 Pend SV Interrupt */
    SysTick_IRQn                  =  -1,      /* 15 System Tick Interrupt */

    /* ----  PHY BUMBEE M0 Interrupt Numbers  ---- */

    BB_IRQn                       =   4,      /* Base band Interrupt */
    KSCAN_IRQn                    =   5,      /* Key scan Interrupt */
    RTC_IRQn                      =   6,      /* RTC Timer Interrupt */

    WDT_IRQn                      =  10,      /* Watchdog Timer Interrupt */
    UART0_IRQn                    =  11,      /* UART0 Interrupt */
    I2C0_IRQn                     =  12,      /* I2C0 Interrupt */
    I2C1_IRQn                     =  13,      /* I2C1 Interrupt */
    SPI0_IRQn                     =  14,      /* SPI0 Interrupt */
    SPI1_IRQn                     =  15,      /* SPI1 Interrupt */
    GPIO_IRQn                     =  16,      /* GPIO Interrupt */
    UART1_IRQn                    =  17,      /* UART1 Interrupt */
    SPIF_IRQn                     =  18,      /* SPIF Interrupt */
    DMAC_IRQn                     =  19,      /* DMAC Interrupt */
    TIM1_IRQn                     =  20,      /* Timer1 Interrupt */
    TIM2_IRQn                     =  21,      /* Timer2 Interrupt */
    TIM3_IRQn                     =  22,      /* Timer3 Interrupt */
    TIM4_IRQn                     =  23,      /* Timer4 Interrupt */
    TIM5_IRQn                     =  24,      /* Timer5 Interrupt */
    TIM6_IRQn                     =  25,      /* Timer6 Interrupt */

    AES_IRQn                      =  28,      /* AES Interrupt */
    ADCC_IRQn                     =  29,      /* ADC Interrupt */
    QDEC_IRQn                     =  30,      /* QDEC Interrupt */
    RNG_IRQn                      =  31,      /* RNG Interrupt */
} IRQn_Type;

#ifdef __cplusplus
  #define   __I     volatile             /* < Defines 'read only' permissions */
#else
  #define   __I     volatile const       /* < Defines 'read only' permissions */
#endif
#define     __O     volatile             /* < Defines 'write only' permissions */
#define     __IO    volatile             /* < Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /* Defines 'read only' structure member permissions */
#define     __OM     volatile            /* Defines 'write only' structure member permissions */
#define     __IOM    volatile            /* Defines 'read / write' structure member permissions */

#define NVIC_DisableIRQ(irqid)      up_disable_irq(irqid)
#define NVIC_EnableIRQ(irqid)       up_enable_irq(irqid)
#define NVIC_SetPriority(i,p)

#if 0
#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
#define ATTRIBUTE_ISR
#include "core_bumbee_m0.h"
#endif
#if(PHY_MCU_TYPE == MCU_BUMBEE_CK802)
#define ATTRIBUTE_ISR  __attribute__((isr))
#include "core_802.h"
#endif
#endif //0

#if (PHY_MCU_TYPE == MCU_BUMBEE_M0 || PHY_MCU_TYPE == MCU_BUMBEE_CK802)
#include "mcu_phy_bumbee.h"
#elif ((PHY_MCU_TYPE == MCU_PRIME_A1) ||(PHY_MCU_TYPE == MCU_PRIME_A2))
#include "mcu_phy_prime.h"
#endif

#endif /* __ARCH_ARM_SRC_PHY62XX_BUS_DEV_H */
