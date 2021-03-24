/****************************************************************************
 * arch/arm/src/str71x/str71x_rccu.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H
#define __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reset and Clock Control Unit (RCCU) register offsets *********************/

/* All registers are 32-bits wide, but with the top 16 bits "reserved" */

#define STR71X_RCCU_CCR_OFFSET       (0x0000)    /* 32-bits wide */
#define STR71X_RCCU_CFR_OFFSET       (0x0008)    /* 32-bits wide */
#define STR71X_RCCU_PLL1CR_OFFSET    (0x0018)    /* 32-bits wide */
#define STR71X_RCCU_PER_OFFSET       (0x001c)    /* 32-bits wide */
#define STR71X_RCCU_SMR_OFFSET       (0x0020)    /* 32-bits wide */

/* Reset and Clock Control Unit (RCCU) register addresses *******************/

#define STR71X_RCCU_CCR              (STR71X_RCCU_BASE + STR71X_RCCU_CCR_OFFSET)
#define STR71X_RCCU_CFR              (STR71X_RCCU_BASE + STR71X_RCCU_CFR_OFFSET)
#define STR71X_RCCU_PLL1CR           (STR71X_RCCU_BASE + STR71X_RCCU_PLL1CR_OFFSET)
#define STR71X_RCCU_PER              (STR71X_RCCU_BASE + STR71X_RCCU_PER_OFFSET)
#define STR71X_RCCU_SMR              (STR71X_RCCU_BASE + STR71X_RCCU_SMR_OFFSET)

/* Register bit settings ****************************************************/

/* RCCU CCR register bit definitions */

#define STR71X_RCCUCCR_LPOWFI        (0x00000001) /* Bit 0: Low power mode in wait-for-interrupt mode */
#define STR71X_RCCUCCR_WFICLKSEL     (0x00000002) /* Bit 1: WFI clock select */
#define STR71X_RCCUCCR_CKAFSEL       (0x00000004) /* Bit 2: Alternate function clock select */
#define STR71X_RCCUCCR_SRESEN        (0x00000008) /* Bit 3: Software reset enable */
#define STR71X_RCCUCCR_ENCLOCK       (0x00000080) /* Bit 7: Lock interrupt enable */
#define STR71X_RCCUCCR_ENCKAF        (0x00000100) /* Bit 8: CKAF interrupt enable */
#define STR71X_RCCUCCR_ENCK216       (0x00000200) /* Bit 9: CK2_16 interrupt enable */
#define STR71X_RCCUCCR_ENSTOP        (0x00000400) /* Bit 10: Stop interrupt enable */
#define STR71X_RCCUCCR_ENHALT        (0x00000800) /* Bit 11: Enable halt */

/* RCCU CFR register bit definitions */

#define STR71X_RCCUCFR_CSUCKSEL      (0x00000001) /* Bit 0: CSU clock select */
#define STR71X_RCCUCFR_LOCK          (0x00000002) /* Bit 1: PLL locked-in */
#define STR71X_RCCUCFR_CKAFST        (0x00000004) /* Bit 2: CK_AF status */
#define STR71X_RCCUCFR_CK216         (0x00000008) /* Bit 3: CLK2/16 selection */
#define STR71X_RCCUCFR_CKSTOPEN      (0x00000010) /* Bit 4: Clock stop enable */
#define STR71X_RCCUCFR_SOFTRES       (0x00000020) /* Bit 5: Software reset */
#define STR71X_RCCUCFR_WDGRES        (0x00000040) /* Bit 6: Watchdog reset */
#define STR71X_RCCUCFR_RTCALARM      (0x00000080) /* Bit 7: RTC alarm reset */
#define STR71X_RCCUCFR_LVDRES        (0x00000200) /* Bit 9: Voltage regulator low voltage detector reset */
#define STR71X_RCCUCFR_WKPRES        (0x00000400) /* Bit 10: External wakeup */
#define STR71X_RCCUCFR_LOCKI         (0x00000800) /* Bit 11: Lock interrupt pending */
#define STR71X_RCCUCFR_CKAFI         (0x00001000) /* Bit 12: CK_AF switching interrupt pending */
#define STR71X_RCCUCFR_CK216I        (0x00002000) /* Bit 13: CK2_16 switching interrupt pending */
#define STR71X_RCCUCFR_STOPI         (0x00004000) /* Bit 14: Stop interrupt pending */
#define STR71X_RCCUCFR_DIV2          (0x00008000) /* Bit 15: OSCIN divided by 2 */

/* RCCU PPL1CR register bit definitions */

#define STR71X_RCCUPLL1CR_DXMASK     (0x00000003) /* Bit 0-2: PLL1 clock divisor */
#define STR71X_RCCUPLL1CR_DIV1       (0x00000000) /*   PLLCK / 1 */
#define STR71X_RCCUPLL1CR_DIV2       (0x00000001) /*   PLLCK / 2 */
#define STR71X_RCCUPLL1CR_DIV3       (0x00000002) /*   PLLCK / 3 */
#define STR71X_RCCUPLL1CR_DIV4       (0x00000003) /*   PLLCK / 4 */
#define STR71X_RCCUPLL1CR_DIV5       (0x00000004) /*   PLLCK / 5 */
#define STR71X_RCCUPLL1CR_DIV6       (0x00000005) /*   PLLCK / 6 */
#define STR71X_RCCUPLL1CR_DIV7       (0x00000006) /*   PLLCK / 7 */
#define STR71X_RCCUPLL1CR_CLK2       (0x00000007) /*   FREEN==0: CLK2 */
#define STR71X_RCCUPLL1CR_FREERM     (0x00000007) /*   FREEN==1: PLL1 in free running mode */
#define STR71X_RCCUPLL1CR_MXMASK     (0x00000030) /* Bit 4-5: PLL1 clock multiplier */
#define STR71X_RCCUPLL1CR_MUL20      (0x00000000) /*   CLK2 * 20 */
#define STR71X_RCCUPLL1CR_MUL12      (0x00000010) /*   CLK2 * 12 */
#define STR71X_RCCUPLL1CR_MUL24      (0x00000020) /*   CLK2 * 24 */
#define STR71X_RCCUPLL1CR_MUL16      (0x00000030) /*   CLK2 * 16 */
#define STR71X_RCCUPLL1CR_FREFRANGE  (0x00000040) /* Bit 6: Reference frequency range select */
#define STR71X_RCCUPLL1CR_FREEN      (0x00000080) /* Bit 7: PKL free running mode */

/* RCCU PER register bit definitions */

#define STR71X_RCCUPER_EMI           (0x00000004) /* Bit 2: EMI */
#define STR71X_RCCUPER_USBKERNEL     (0x00000010) /* Bit 4: USB Kernel */

/* RCCU SMR register bit definitions */

#define STR71X_RCCUSMR_WFI           (0x00000001) /* Bit 0: Wait for interrupt */
#define STR71X_RCCUSMR_HALT          (0x00000000) /* Bit 1: Halt */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H */
