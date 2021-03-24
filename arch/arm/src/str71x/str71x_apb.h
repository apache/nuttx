/****************************************************************************
 * arch/arm/src/str71x/str71x_apb.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_APB_H
#define __ARCH_ARM_SRC_STR71X_STR71X_APB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* APB register offsets *****************************************************/

#define STR71X_APB_CKDIS_OFFSET   (0x0010) /* 32-bits wide */
#define STR71X_APB_SWRES_OFFSET   (0x0014) /* 32-bits wide */

/* APB register addresses ***************************************************/

#define STR71X_APB1_CKDIS         (STR71X_APB1_BASE + STR71X_APB_CKDIS_OFFSET)
#define STR71X_APB1_SWRES         (STR71X_APB1_BASE + STR71X_APB_SWRES_OFFSET)

#define STR71X_APB2_CKDIS         (STR71X_APB2_BASE + STR71X_APB_CKDIS_OFFSET)
#define STR71X_APB2_SWRES         (STR71X_APB2_BASE + STR71X_APB_SWRES_OFFSET)

/* Register bit settings ****************************************************/

/* APB1 periperals */

#define STR71X_APB1_I2C0          (0x0001) /* Bit 0:  I2C0 */
#define STR71X_APB1_I2C1          (0x0002) /* Bit 1:  I2C1 */
#define STR71X_APB1_UART0         (0x0008) /* Bit 3:  UART0 */
#define STR71X_APB1_UART1         (0x0010) /* Bit 4:  UART1 */
#define STR71X_APB1_UART2         (0x0020) /* Bit 5:  UART2 */
#define STR71X_APB1_UART3         (0x0040) /* Bit 6:  UART3 */
#define STR71X_APB1_USB           (0x0080) /* Bit 7:  USB */
#define STR71X_APB1_CAN           (0x0100) /* Bit 8:  CAN */
#define STR71X_APB1_BSPI0         (0x0200) /* Bit 9:  BSPI0 */
#define STR71X_APB1_BSPI1         (0x0400) /* Bit 10: BSPI1 */
#define STR71X_APB1_HDLC          (0x2000) /* Bit 13: HDLC */
#define STR71X_APB1_APB1ALL       (0x27fb)

/* APB2 Peripherals */

#define STR71X_APB2_XTI           (0x0001) /* Bit 0:  XTI */
#define STR71X_APB2_GPIO0         (0x0004) /* Bit 2:  IOPORT0 */
#define STR71X_APB2_GPIO1         (0x0008) /* Bit 3:  IOPORT1 */
#define STR71X_APB2_GPIO2         (0x0010) /* Bit 4:  IOPORT2 */
#define STR71X_APB2_ADC12         (0x0040) /* Bit 6:  ADC */
#define STR71X_APB2_CKOUT         (0x0080) /* Bit 7:  CKOUT */
#define STR71X_APB2_TIM0          (0x0100) /* Bit 8:  TIMER0 */
#define STR71X_APB2_TIM1          (0x0200) /* Bit 9:  TIMER1 */
#define STR71X_APB2_TIM2          (0x0400) /* Bit 10: TIMER2 */
#define STR71X_APB2_TIM3          (0x0800) /* Bit 11: TIMER3 */
#define STR71X_APB2_RTC           (0x1000) /* Bit 12: RTC */
#define STR71X_APB2_EIC           (0x4000) /* Bit 14: EIC */
#define STR71X_APB2_APB2ALL       (0x5fdd)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_APB_H */
