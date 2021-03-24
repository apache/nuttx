/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_rtc.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_RTC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_RTC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RTC_CRH_OFFSET    0x0000    /* RTC control register High (16-bit) */
#define STM32_RTC_CRL_OFFSET    0x0004    /* RTC control register low (16-bit) */
#define STM32_RTC_PRLH_OFFSET   0x0008    /* RTC prescaler load register high (16-bit) */
#define STM32_RTC_PRLL_OFFSET   0x000c    /* RTC prescaler load register low (16-bit) */
#define STM32_RTC_DIVH_OFFSET   0x0010    /* RTC prescaler divider register high (16-bit) */
#define STM32_RTC_DIVL_OFFSET   0x0014    /* RTC prescaler divider register low (16-bit) */
#define STM32_RTC_CNTH_OFFSET   0x0018    /* RTC counter register high (16-bit) */
#define STM32_RTC_CNTL_OFFSET   0x001c    /* RTC counter register low (16-bit) */
#define STM32_RTC_ALRH_OFFSET   0x0020    /* RTC alarm register high (16-bit) */
#define STM32_RTC_ALRL_OFFSET   0x0024    /* RTC alarm register low (16-bit) */

/* Register Addresses *******************************************************/

#define STM32_RTC_CRH           (STM32_RTC_BASE+STM32_RTC_CRH_OFFSET)
#define STM32_RTC_CRL           (STM32_RTC_BASE+STM32_RTC_CRL_OFFSET)
#define STM32_RTC_PRLH          (STM32_RTC_BASE+STM32_RTC_PRLH_OFFSET)
#define STM32_RTC_PRLL          (STM32_RTC_BASE+STM32_RTC_PRLL_OFFSET)
#define STM32_RTC_DIVH          (STM32_RTC_BASE+STM32_RTC_DIVH_OFFSET)
#define STM32_RTC_DIVL          (STM32_RTC_BASE+STM32_RTC_DIVL_OFFSET)
#define STM32_RTC_CNTH          (STM32_RTC_BASE+STM32_RTC_CNTH_OFFSET)
#define STM32_RTC_CNTL          (STM32_RTC_BASE+STM32_RTC_CNTL_OFFSET)
#define STM32_RTC_ALRH          (STM32_RTC_BASE+STM32_RTC_ALRH_OFFSET)
#define STM32_RTC_ALRL          (STM32_RTC_BASE+STM32_RTC_ALRL_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RTC control register High (16-bit) */

#define RTC_CRH_SECIE           (1 << 0)  /* Bit 0 : Second Interrupt Enable */
#define RTC_CRH_ALRIE           (1 << 1)  /* Bit 1: Alarm Interrupt Enable */
#define RTC_CRH_OWIE            (1 << 2)  /* Bit 2: OverfloW Interrupt Enable */

/* RTC control register low (16-bit) */

#define RTC_CRL_SECF            (1 << 0)  /* Bit 0: Second Flag */
#define RTC_CRL_ALRF            (1 << 1)  /* Bit 1: Alarm Flag */
#define RTC_CRL_OWF             (1 << 2)  /* Bit 2: Overflow Flag */
#define RTC_CRL_RSF             (1 << 3)  /* Bit 3: Registers Synchronized Flag */
#define RTC_CRL_CNF             (1 << 4)  /* Bit 4: Configuration Flag */
#define RTC_CRL_RTOFF           (1 << 5)  /* Bit 5: RTC operation OFF */

/* RTC prescaler load register high (16-bit) */

#define RTC_PRLH_PRL_SHIFT      (0)      /* Bits 3-0: RTC Prescaler Reload Value High */
#define RTC_PRLH_PRL_MASK       (0x0f << RTC_PRLH_PRL_SHIFT)

/* RTC prescaler divider register high (16-bit) */

#define RTC_DIVH_RTC_DIV_SHIFT  (0)      /* Bits 3-0: RTC Clock Divider High */
#define RTC_DIVH_RTC_DIV_MASK   (0x0f << RTC_DIVH_RTC_DIV_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_RTC_H */
