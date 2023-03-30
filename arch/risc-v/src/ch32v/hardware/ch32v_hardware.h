/****************************************************************************
 * arch/risc-v/src/ch32v/hardware/ch32v_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_CH32V_HARDWARE_CH32V_HARDWARE_H
#define __ARCH_RISCV_SRC_CH32V_HARDWARE_CH32V_HARDWARE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/*
 * CH32V notes: These come /svdtoheaders/demo/h/ch32v307_map.h
 * is is machine-genreated from the SVD and believed to be canonical.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CH32V Address Blocks *****************************************************/
#define CH32V_ADC1_BASE      0x40012400 /* 0x40012400-0x400127ff: 1kB Analog to digital converter */
#define CH32V_ADC2_BASE      0x40012800 /* 0x40012800-0x40012bff: 1kB Analog to digital converter */
#define CH32V_AFIO_BASE      0x40010000 /* 0x40010000-0x400103ff: 1kB Alternate function I/O */
#define CH32V_BKP_BASE       0x40006c00 /* 0x40006c00-0x40006fff: 1kB Backup registers */
#define CH32V_CAN1_BASE      0x40006400 /* 0x40006400-0x400067ff: 1kB Controller area network */
#define CH32V_CAN2_BASE      0x40006800 /* 0x40006800-0x40006bff: 1kB CAN2 */
#define CH32V_CRC_BASE       0x40023000 /* 0x40023000-0x400233ff: 1kB CRC calculation unit */
#define CH32V_DAC_BASE       0x40007400 /* 0x40007400-0x400077ff: 1kB Digital to analog converter */
#define CH32V_DBG_BASE       0xe000d000 /* 0xe000d000-0xe000d3ff: 1kB Debug support */
#define CH32V_DMA1_BASE      0x40020000 /* 0x40020000-0x400203ff: 1kB DMA1 controller */
#define CH32V_DMA2_BASE      0x40020400 /* 0x40020400-0x400207ff: 1kB DMA2 controller */
#define CH32V_DVP_BASE       0x50050000 /* 0x50050000-0x500503ff: 1kB Digital Video Port */
#define CH32V_ETHERNET_DMA_BASE 0x40029000 /* 0x40029000-0x400293ff: 1kB Ethernet: DMA controller operation */
#define CH32V_ETHERNET_MAC_BASE 0x40028000 /* 0x40028000-0x400283ff: 1kB Ethernet: media access control */
#define CH32V_ETHERNET_MMC_BASE 0x40028100 /* 0x40028100-0x400284ff: 1kB Ethernet: MAC management counters */
#define CH32V_ETHERNET_PTP_BASE 0x40028700 /* 0x40028700-0x40028aff: 1kB Ethernet: Precision time protocol */
#define CH32V_EXTEND_BASE    0x40023800 /* 0x40023800-0x40023bff: 1kB Extend configuration */
#define CH32V_EXTI_BASE      0x40010400 /* 0x40010400-0x400107ff: 1kB EXTI */
#define CH32V_FLASH_BASE     0x40022000 /* 0x40022000-0x400223ff: 1kB FLASH */
#define CH32V_FSMC_BASE      0xa0000000 /* 0xa0000000-0xa0000fff: 4kB Flexible static memory controller */
#define CH32V_GPIOA_BASE     0x40010800 /* 0x40010800-0x40010bff: 1kB General purpose I/O */
#define CH32V_GPIOB_BASE     0x40010c00 /* 0x40010c00-0x40010fff: 1kB GPIOB */
#define CH32V_GPIOC_BASE     0x40011000 /* 0x40011000-0x400113ff: 1kB GPIOC */
#define CH32V_GPIOD_BASE     0x40011400 /* 0x40011400-0x400117ff: 1kB GPIOD */
#define CH32V_GPIOE_BASE     0x40011800 /* 0x40011800-0x40011bff: 1kB GPIOE */
#define CH32V_I2C1_BASE      0x40005400 /* 0x40005400-0x400057ff: 1kB Inter integrated circuit */
#define CH32V_I2C2_BASE      0x40005800 /* 0x40005800-0x40005bff: 1kB I2C2 */
#define CH32V_IWDG_BASE      0x40003000 /* 0x40003000-0x400033ff: 1kB Independent watchdog */
#define CH32V_OPA_BASE       0x40023804 /* 0x40023804-0x40023a03: 1kB OPA configuration */
#define CH32V_PFIC_BASE      0xe000e000 /* 0xe000e000-0xe000f0ff: 4kB Programmable Fast Interrupt Controller */
#define CH32V_PWR_BASE       0x40007000 /* 0x40007000-0x400073ff: 1kB Power control */
#define CH32V_RCC_BASE       0x40021000 /* 0x40021000-0x400213ff: 1kB Reset and clock control */
#define CH32V_RNG_BASE       0x40023c00 /* 0x40023c00-0x40023fff: 1kB Random number generator */
#define CH32V_RTC_BASE       0x40002800 /* 0x40002800-0x40002bff: 1kB Real time clock */
#define CH32V_SDIO_BASE      0x40018000 /* 0x40018000-0x400183ff: 1kB Secure digital input/output interface */
#define CH32V_SPI1_BASE      0x40013000 /* 0x40013000-0x400133ff: 1kB Serial peripheral interface */
#define CH32V_SPI2_BASE      0x40003800 /* 0x40003800-0x40003bff: 1kB Serial peripheral interface */
#define CH32V_SPI3_BASE      0x40003c00 /* 0x40003c00-0x40003fff: 1kB SPI3 */
#define CH32V_TIM10_BASE     0x40015000 /* 0x40015000-0x400153ff: 1kB TIM10 */
#define CH32V_TIM1_BASE      0x40012c00 /* 0x40012c00-0x40012fff: 1kB Advanced timer */
#define CH32V_TIM2_BASE      0x40000000 /* 0x40000000-0x400003ff: 1kB General purpose timer */
#define CH32V_TIM3_BASE      0x40000400 /* 0x40000400-0x400007ff: 1kB TIM3 */
#define CH32V_TIM4_BASE      0x40000800 /* 0x40000800-0x40000bff: 1kB TIM4 */
#define CH32V_TIM5_BASE      0x40000c00 /* 0x40000c00-0x40000fff: 1kB TIM5 */
#define CH32V_TIM6_BASE      0x40001000 /* 0x40001000-0x400013ff: 1kB Basic timer */
#define CH32V_TIM7_BASE      0x40001400 /* 0x40001400-0x400017ff: 1kB TIM7 */
#define CH32V_TIM8_BASE      0x40013400 /* 0x40013400-0x400137ff: 1kB TIM8 */
#define CH32V_TIM9_BASE      0x40014c00 /* 0x40014c00-0x40014fff: 1kB TIM9 */
#define CH32V_UART4_BASE     0x40004c00 /* 0x40004c00-0x40004fff: 1kB UART4 */
#define CH32V_UART5_BASE     0x40005000 /* 0x40005000-0x400053ff: 1kB UART5 */
#define CH32V_UART6_BASE     0x40001800 /* 0x40001800-0x40001bff: 1kB UART6 */
#define CH32V_UART7_BASE     0x40001c00 /* 0x40001c00-0x40001fff: 1kB UART7 */
#define CH32V_UART8_BASE     0x40002000 /* 0x40002000-0x400023ff: 1kB UART8 */
#define CH32V_USART1_BASE    0x40013800 /* 0x40013800-0x40013bff: 1kB Universal synchronous asynchronous receiver transmitter */
#define CH32V_USART2_BASE    0x40004400 /* 0x40004400-0x400047ff: 1kB USART2 */
#define CH32V_USART3_BASE    0x40004800 /* 0x40004800-0x40004bff: 1kB USART3 */
#define CH32V_USBHD_BASE     0x40023400 /* 0x40023400-0x400237ff: 1kB USB register */
#define CH32V_USB_BASE       0x40005c00 /* 0x40005c00-0x40005fff: 1kB Universal serial bus full-speed device interface */
#define CH32V_USB_OTG_FS_BASE 0x50000000 /* 0x50000000-0x5003ffff: 256kB USB FS OTG register */
#define CH32V_WWDG_BASE      0x40002c00 /* 0x40002c00-0x40002fff: 1kB Window watchdog */

#endif /* __ARCH_RISCV_SRC_CH32V7_HARDWARE_CH32V_HARDWARE_H */
