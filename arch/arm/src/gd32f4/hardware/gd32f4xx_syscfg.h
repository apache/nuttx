/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SYSCFG_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_SYSCFG_CFG0_OFFSET          0x0000                    /* System configuration register 0 */
#define GD32_SYSCFG_CFG1_OFFSET          0x0004                    /* System configuration register 1 */
#define GD32_SYSCFG_EXTISS0_OFFSET       0x0008                    /* EXTI sources selection register 0 */
#define GD32_SYSCFG_EXTISS1_OFFSET       0x000c                    /* EXTI sources selection register 0 */
#define GD32_SYSCFG_EXTISS2_OFFSET       0x0010                    /* EXTI sources selection register 0 */
#define GD32_SYSCFG_EXTISS3_OFFSET       0x0014                    /* EXTI sources selection register 0 */
#define GD32_SYSCFG_CPSCTLOFFSET         0x0020                    /* System I/O compensation control register */

/* Register Addresses *******************************************************/

#define GD32_SYSCFG_CFG0                 (GD32_SYSCFG_BASE+GD32_SYSCFG_CFG0_OFFSET)
#define GD32_SYSCFG_CFG1                 (GD32_SYSCFG_BASE+GD32_SYSCFG_CFG1_OFFSET)
#define GD32_SYSCFG_EXTISS0              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS0_OFFSET)
#define GD32_SYSCFG_EXTISS1              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS1_OFFSET)
#define GD32_SYSCFG_EXTISS2              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS2_OFFSET)
#define GD32_SYSCFG_EXTISS3              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS3_OFFSET)
#define GD32_SYSCFG_CPSCTL               (GD32_SYSCFG_BASE+GD32_SYSCFG_CPSCTLOFFSET)

/* Register Bitfield Definitions ********************************************/

/* System configuration register 0 */

/* boot mode definitions */
#define SYSCFG_CFG0_BOOT_MODE_SHIFT      (0)                                    /* Bits 2:0 BOOT_MODE: SYSCFG memory remap config */
#define SYSCFG_CFG0_BOOT_MODE_MASK       (7 << SYSCFG_CFG0_BOOT_MODE_SHIFT)
#  define SYSCFG_BOOTMODE_FLASH          (0 << SYSCFG_CFG0_BOOT_MODE_SHIFT)     /* 000: Main flash memory remap */
#  define SYSCFG_BOOTMODE_BOOTLOADER     (1 << SYSCFG_CFG0_BOOT_MODE_SHIFT)     /* 001: Boot loader remap */
#  define SYSCFG_BOOTMODE_EXMC_SRAM      (2 << SYSCFG_CFG0_BOOT_MODE_SHIFT)     /* 010: SRAM/NOR 0 and 1 of EXMC remap */
#  define SYSCFG_BOOTMODE_SRAM           (3 << SYSCFG_CFG0_BOOT_MODE_SHIFT)     /* 011: SRAM0 of on-chip SRAM remap */
#  define SYSCFG_BOOTMODE_EXMC_SDRAM     (4 << SYSCFG_CFG0_BOOT_MODE_SHIFT)     /* 100: SDRAM bank0 of EXMC remap */

/* FMC swap definitions */

#define SYSCFG_CFG0_FMC_SWP_SHIFT        (8)                                    /* FMC memory swap config */
#define SYSCFG_CFG0_FMC_SWP_MASK         (1 << SYSCFG_CFG0_FMC_SWP_SHIFT)
#  define SYSCFG_FMC_SWP_BANK0           (0 << SYSCFG_CFG0_FMC_SWP_SHIFT)       /*  Main flash Bank 0 is mapped at address 0x08000000 */
#  define SYSCFG_FMC_SWP_BANK1           (1 << SYSCFG_CFG0_FMC_SWP_SHIFT)       /*  Main flash Bank 1 is mapped at address 0x08000000 */

/* EXMC swap enable/disable */

#define SYSCFG_CFG0_EXMC_SWP_SHIFT       (9)                                    /* EXMC memory swap config */
#define SYSCFG_CFG0_EXMC_SWP_MASK        (1 << SYSCFG_CFG0_EXMC_SWP_SHIFT)
#  define SYSCFG_EXMC_SWP_ENABLE         (1 << SYSCFG_CFG0_EXMC_SWP_SHIFT)      /* SDRAM bank 0 and bank 1 are swapped with NAND bank 1 and PC card */
#  define SYSCFG_EXMC_SWP_DISABLE        (0 << SYSCFG_CFG0_EXMC_SWP_SHIFT)      /* No memory mapping swap */

/* System configuration register 1 */

/* Ethernet PHY selection */
#define SYSCFG_CFG1_ENET_PHY_SEL_SHIFT   (23)                                   /* Ethernet PHY selection config */
#define SYSCFG_CFG1_ENET_PHY_SEL_MASK    (1 << SYSCFG_CFG1_ENET_PHY_SEL_SHIFT)
#  define SYSCFG_ENET_PHY_MII            (0<< SYSCFG_CFG1_ENET_PHY_SEL_SHIFT)   /* MII is selected for the Ethernet MAC */
#  define SYSCFG_ENET_PHY_RMII           (1 << SYSCFG_CFG1_ENET_PHY_SEL_SHIFT)  /* RMII is selected for the Ethernet MAC */

/* SYSCFG external interrupt configuration register 0-3 */

/* SYSCFG_EXTISS0 bits definitions */
#define SYSCFG_EXTISS0_EXTI0_SHIFT       (0)
#define SYSCFG_EXTISS0_EXTI0_MASK        (15 << SYSCFG_EXTISS0_EXTI0_SHIFT)
#define SYSCFG_EXTISS0_EXTI1_SHIFT       (4)
#define SYSCFG_EXTISS0_EXTI1_MASK        (15 << SYSCFG_EXTISS0_EXTI1_SHIFT)
#define SYSCFG_EXTISS0_EXTI2_SHIFT       (8)
#define SYSCFG_EXTISS0_EXTI2_MASK        (15 << SYSCFG_EXTISS0_EXTI2_SHIFT)
#define SYSCFG_EXTISS0_EXTI3_SHIFT       (12)
#define SYSCFG_EXTISS0_EXTI3_MASK        (15 << SYSCFG_EXTISS0_EXTI3_MASK)

/* SYSCFG_EXTISS1 bits definitions */
#define SYSCFG_EXTISS1_EXTI0_SHIFT       (0)
#define SYSCFG_EXTISS1_EXTI0_MASK        (15 << SYSCFG_EXTISS1_EXTI0_SHIFT)
#define SYSCFG_EXTISS1_EXTI1_SHIFT       (4)
#define SYSCFG_EXTISS1_EXTI1_MASK        (15 << SYSCFG_EXTISS1_EXTI1_SHIFT)
#define SYSCFG_EXTISS1_EXTI2_SHIFT       (8)
#define SYSCFG_EXTISS1_EXTI2_MASK        (15 << SYSCFG_EXTISS1_EXTI2_SHIFT)
#define SYSCFG_EXTISS1_EXTI3_SHIFT       (12)
#define SYSCFG_EXTISS1_EXTI3_MASK        (15 << SYSCFG_EXTISS1_EXTI3_SHIFT)

/* SYSCFG_EXTISS2 bits definitions */
#define SYSCFG_EXTISS2_EXTI0_SHIFT       (0)
#define SYSCFG_EXTISS2_EXTI0_MASK        (15 << SYSCFG_EXTISS2_EXTI0_SHIFT)
#define SYSCFG_EXTISS2_EXTI1_SHIFT       (4)
#define SYSCFG_EXTISS2_EXTI1_MASK        (15 << SYSCFG_EXTISS2_EXTI1_SHIFT)
#define SYSCFG_EXTISS2_EXTI2_SHIFT       (8)
#define SYSCFG_EXTISS2_EXTI2_MASK        (15 << SYSCFG_EXTISS2_EXTI2_SHIFT)
#define SYSCFG_EXTISS2_EXTI3_SHIFT       (12)
#define SYSCFG_EXTISS2_EXTI3_MASK        (15 << SYSCFG_EXTISS2_EXTI3_SHIFT)

/* SYSCFG_EXTISS3 bits definitions */
#define SYSCFG_EXTISS3_EXTI0_SHIFT       (0)
#define SYSCFG_EXTISS3_EXTI0_MASK        (15 << SYSCFG_EXTISS3_EXTI0_SHIFT)
#define SYSCFG_EXTISS3_EXTI1_SHIFT       (4)
#define SYSCFG_EXTISS3_EXTI1_MASK        (15 << SYSCFG_EXTISS3_EXTI1_SHIFT)
#define SYSCFG_EXTISS3_EXTI2_SHIFT       (8)
#define SYSCFG_EXTISS3_EXTI2_MASK        (15 << SYSCFG_EXTISS3_EXTI2_SHIFT)
#define SYSCFG_EXTISS3_EXTI3_SHIFT       (12)
#define SYSCFG_EXTISS3_EXTI3_MASK        (15 << SYSCFG_EXTISS3_EXTI3_SHIFT)

/* EXTI source select definition */
#define SYSCFG_EXTISS0                   (0x00)                                 /* EXTI source select GPIOx pin 0~3 */
#define SYSCFG_EXTISS1                   (0x01)                                 /* EXTI source select GPIOx pin 4~7 */
#define SYSCFG_EXTISS2                   (0x02)                                 /* EXTI source select GPIOx pin 8~11 */
#define SYSCFG_EXTISS3                   (0x03)                                 /* EXTI source select GPIOx pin 12~15 */

/* EXTI source select mask bits definition */
#define SYSCFG_EXTI_SS_MASK              (15)                                   /* EXTI source select mask */

/* EXTI source select jumping step definition */
#define SYSCFG_EXTI_SS_JSTEP             (0x04)                                 /* EXTI source select jumping step */

/* EXTI source select moving step definition */
#define SYSCFG_EXTI_SS_MSTEP(pin)        (SYSCFG_EXTI_SS_JSTEP*((pin)%SYSCFG_EXTI_SS_JSTEP))   /* EXTI source select moving step */

#define EXTI_SOURCE_GPIOA                (0)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOB                (1)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOC                (2)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOD                (3)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOE                (4)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOF                (5)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOG                (6)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOH                (7)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOI                (8)                                    /* EXTI GPIOA configuration */

/* System I/O compensation control register */

/* I/O compensation cell enable/disable */
#define SYSCFG_CPSCTL_CPS_EN_SHILT       (0)                                    /* I/O compensation cell enable bit */
#define SYSCFG_CPSCTL_CPS_EN_MASK        (1 << SYSCFG_CPSCTL_CPS_EN_SHILT)
#  define SYSCFG_COMPENSATION_ENABLE     (1 << SYSCFG_CPSCTL_CPS_EN_SHILT)      /* I/O compensation cell enable */
#  define SYSCFG_COMPENSATION_DISABLE    (1 << SYSCFG_CPSCTL_CPS_EN_SHILT)      /* I/O compensation cell disable */

#define SYSCFG_CPSCTL_CPS_RDY_SHIFT      (8)                                    /* I/O compensation cell is ready or not */
#define SYSCFG_CPSCTL_CPS_RDY_MASK       (1 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)
#define SYSCFG_CPSCTL_CPS_RDY_SET        (1 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)     /* Ready */
#define SYSCFG_CPSCTL_CPS_RDY_RESET      (1 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)     /* Not ready */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SYSCFG_H */
