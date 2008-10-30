/************************************************************************************
 * arch/arm/src/str71x/str71x_rccu.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H
#define __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Reset and Clock Control Unit (RCCU) registers ************************************/

/* All registers are 32-bits wide, but with the top 16 bits "reserved" */

#define STR71X_RCCU_CCR              (STR71X_RCCU_BASE + 0x0000)    /* 32-bits wide */
#define STR71X_RCCU_CFR              (STR71X_RCCU_BASE + 0x0008)    /* 32-bits wide */
#define STR71X_RCCU_PLL1CR           (STR71X_RCCU_BASE + 0x0018)    /* 32-bits wide */
#define STR71X_RCCU_PER              (STR71X_RCCU_BASE + 0x001c)    /* 32-bits wide */
#define STR71X_RCCU_SMR              (STR71X_RCCU_BASE + 0x0020)    /* 32-bits wide */

/* Register bit settings ************************************************************/
/* RCCU Clock Div */

#define STR71X_RCCU_DEFAULT          (0x00)
#define STR71X_RCCU_RCLK2            (0x01)
#define STR71X_RCCU_RCLK4            (0x02)
#define STR71X_RCCU_RCLK8            (0x03)

/* RCCU RCLK Clocks */

#define STR71X_RCCU_PLL1_OUTPUT      (0)
#define STR71X_RCCU_CLOCK216         (1)
#define STR71X_RCCU_CLOCK2           (2)
#define STR71X_RCCU_CK_AF            (3)

/* RCCU PLL1 Multipliers */

#define STR71X_RCCU_PLL1_MUL12       (0x01)
#define STR71X_RCCU_PLL1_MUL16       (0x03)
#define STR71X_RCCU_PLL1_MUL20       (0x00)
#define STR71X_RCCU_PLL1_MUL24       (0x02)

/* RCCU PLL1 Multipliers */

#define STR71X_RCCU_PLL2_MUL12       (0x01)
#define STR71X_RCCU_PLL2_MUL16       (0x03)
#define STR71X_RCCU_PLL2_MUL20       (0x00)
#define STR71X_RCCU_PLL2_MUL28       (0x02)

/* RCCU PLL Divisors */

#define STR71X_RCCU_DIV1             (0x00)
#define STR71X_RCCU_DIV2             (0x01)
#define STR71X_RCCU_DIV3             (0x02)
#define STR71X_RCCU_DIV4             (0x03)
#define STR71X_RCCU_DIV5             (0x04)
#define STR71X_RCCU_DIV6             (0x05)
#define STR71X_RCCU_DIV7             (0x06)

/* RCCU USB Clocks */

#define STR71X_RCCU_PLL2_OUTPUT      (0x01)
#define STR71X_RCCU_USBCK            (0x00)

/* RCCU Clocks */

#define STR71X_RCCU_CLK2             (0)
#define STR71X_RCCU_RCLK             (1)
#define STR71X_RCCU_MCLK             (2)
#define STR71X_RCCU_PCLK2            (3)
#define STR71X_RCCU_PCLK1            (4)

/* RCCU Interrupts */

#define STR71X_RCCU_INTPLL1LOCK      (0x0080)
#define STR71X_RCCU_INTCKAF          (0x0100)
#define STR71X_RCCU_INTCK216         (0x0200)
#define STR71X_RCCU_INTSTOP          (0x0400)

/* RCCU Flags */

#define STR71X_RCCU_PLL1LOCK         (0x0002)
#define STR71X_RCCU_CKAFST           (0x0004)
#define STR71X_RCCU_PLL1LOCKI        (0x0800)
#define STR71X_RCCU_CKAFI            (0x1000)
#define STR71X_RCCU_CK216I           (0x2000)
#define STR71X_RCCU_STOPI            (0x4000

/* RCCU Reset Sources */

#define STR71X_RCCU_RESETSOURCESMASK (0x000006e0)
#define STR71X_RCCU_EXTERNALRESET    (0x00000000)
#define STR71X_RCCU_SOFTWARERESET    (0x00000020)
#define STR71X_RCCU_WDGRESET         (0x00000040)
#define STR71X_RCCU_RTCALARMRESET    (0x00000080)
#define STR71X_RCCU_LVDRESET         (0x00000200)
#define STR71X_RCCU_WKPRESET         (0x00000400

/* RCCU PLL1 free running modes */

#define STR71X_RCCU_PLL1FRM125       (0)
#define STR71X_RCCU_PLL1FRM250       (1)
#define STR71X_RCCU_PLL1FRM500       (2)

#define STR71X_RCCU_DIV2_MASK        (0x00008000)
#define STR71X_RCCU_DIV2_INDEX       (0x0f)
#define STR71X_RCCU_FACT_MASK        (0x0003)

#define STR71X_RCCU_FACT1_MASK       (0x0003)

#define STR71X_RCCU_FACT2_MASK       (0x0300)
#define STR71X_RCCU_FACT2_INDEX      (0x08)

#define STR71X_RCCU_MX_MASK          (0x00000030)
#define STR71X_RCCU_MX_INDEX         (0x04)

#define STR71X_RCCU_DX_MASK          (0x00000007)

#define STR71X_RCCU_FREFRANGE_MASK   (0x00000040)

#define STR71X_RCCU_FRQRNG_MASK      (0x00000040)

#define STR71X_RCCU_FREEN_MASK       (0x00000080)

#define STR71X_RCCU_PLLEN_MASK       (0x00000080)

#define STR71X_RCCU_CSU_CKSEL_MASK   (0x00000001)

#define STR71X_RCCU_CK2_16_MASK      (0x00000008)

#define STR71X_RCCU_CKAF_SEL_MASK    (0x00000004)

#define STR71X_RCCU_LOCK_MASK        (0x00000002)

#define STR71X_RCCU_USBEN_MASK       (0x0100)
#define STR71X_RCCU_USBEN_INDEX      (0x08)

/* RTC Oscillator Frequency value = 32 768 Hz */

#define STR71X_RCCU_RTC_OSC          (32768)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_RCCU_H */
