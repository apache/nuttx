/****************************************************************************
 * arch/arm/src/str71x/str71x_map.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_MAP_H
#define __ARCH_ARM_SRC_STR71X_STR71X_MAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

#define STR71X_FLASHRAMEMI_BASE (0x00000000) /* Flash alias for booting */
#define STR71X_RAM_BASE         (0x20000000)
#define STR71X_FLASH_BASE       (0x40000000)
#define STR71X_FLASHREG_BASE    (0x40100000)
#define STR71X_EXTMEM_BASE      (0x60000000)
#define STR71X_EMI_BASE         (STR71X_EXTMEM_BASE + 0x0c000000)
#define STR71X_RCCU_BASE        (0xa0000000)
#define STR71X_PCU_BASE         (0xa0000040)
#define STR71X_APB1_BASE        (0xc0000000)
#define STR71X_I2C0_BASE        (STR71X_APB1_BASE + 0x1000)
#define STR71X_I2C1_BASE        (STR71X_APB1_BASE + 0x2000)
#define STR71X_UART0_BASE       (STR71X_APB1_BASE + 0x4000)
#define STR71X_UART1_BASE       (STR71X_APB1_BASE + 0x5000)
#define STR71X_UART2_BASE       (STR71X_APB1_BASE + 0x6000)
#define STR71X_UART3_BASE       (STR71X_APB1_BASE + 0x7000)
#define STR71X_USBRAM_BASE      (STR71X_APB1_BASE + 0x8000)
#define STR71X_USB_BASE         (STR71X_APB1_BASE + 0x8800)
#define STR71X_CAN_BASE         (STR71X_APB1_BASE + 0x9000)
#define STR71X_BSPI0_BASE       (STR71X_APB1_BASE + 0xa000)
#define STR71X_BSPI1_BASE       (STR71X_APB1_BASE + 0xb000)
#define STR71X_HDLCRAM_BASE     (STR71X_APB1_BASE + 0xe000)
#define STR71X_APB2_BASE        (0xe0000000)
#define STR71X_XTI_BASE         (STR71X_APB2_BASE + 0x1000)
#define STR71X_GPIO0_BASE       (STR71X_APB2_BASE + 0x3000)
#define STR71X_GPIO1_BASE       (STR71X_APB2_BASE + 0x4000)
#define STR71X_GPIO2_BASE       (STR71X_APB2_BASE + 0x5000)
#define STR71X_ADC12_BASE       (STR71X_APB2_BASE + 0x7000)
#define STR71X_CLKOUT_BASE      (STR71X_APB2_BASE + 0x8000)
#define STR71X_TIMER0_BASE      (STR71X_APB2_BASE + 0x9000)
#define STR71X_TIMER1_BASE      (STR71X_APB2_BASE + 0xa000)
#define STR71X_TIMER2_BASE      (STR71X_APB2_BASE + 0xb000)
#define STR71X_TIMER3_BASE      (STR71X_APB2_BASE + 0xc000)
#define STR71X_RTC_BASE         (STR71X_APB2_BASE + 0xd000)
#define STR71X_WDOG_BASE        (STR71X_APB2_BASE + 0xe000)
#define STR71X_EIC_BASE         (0xfffff800)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif // __ARCH_ARM_SRC_STR71X_STR71X_MAP_H
