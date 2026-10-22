/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_adc_chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_ADC_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_ADC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip ADC wiring for RTL8730E (AmebaSmart, Cortex-A32), read out of the
 * SoC ameba_adc.h, sysreg_lsys.h, ameba_pinmux.h and the CA32 section of
 * ameba_vector_table.h.  The shared driver
 * (arch/arm/src/common/ameba/ameba_adc.c) drives the converter through the
 * SDK fwlib API, so no register base appears here.
 *
 * Channels: ADC_CH_NUM is 10.  CH0..CH5 are external and bound to fixed pads
 * PA0..PA5 (ADC_CHx_PIN); the remaining channels are internal and have no
 * pad.  On the RTL8730EAM/EAH packages PA0 and PA1 are not bonded out, so
 * only CH2..CH5 (PA2..PA5) are usable there.
 *
 * Pad mux: the pads reach the ADC through the cap-touch function
 * (PINMUX_FUNCTION_CAPTOUCH, FUNC_ID18, "TOUCHn_ADCn" in the pinmux table).
 * PINMUX_FUNCTION_AUXIN is 16 here, which is FUNC_ID16 = SPK, so it must
 * not be used.  The ADC and the cap-touch controller share these pads, and
 * the SDK enables the cap-touch (CTC) clock together with the ADC clock, so
 * AMEBA_ADC_AUXCLK_* gate APBPeriph_CTC as well.
 *
 * Conversion word: ADC_Read() keeps only the low 16 bits of the global data
 * register on this chip, so the channel id is in [15:12] and the 12-bit
 * sample in [11:0].  Other chips keep 20 bits (id in [19:16]); the shared
 * driver's defaults assume that, hence the overrides below.
 */

#define AMEBA_ADC_NCHAN           10    /* ADC_CH_NUM: CH0..CH5 ext pads       */
#define AMEBA_ADC_MAXLIST         16    /* Channel-switch list depth (Cvlist)  */
#define AMEBA_ADC_PINMUX_FID      18    /* PINMUX_FUNCTION_CAPTOUCH             */
#define AMEBA_ADC_IRQ             46    /* ADC_IRQ (unused by this polling drv) */

/* sizeof(fwlib ADC_InitTypeDef): OpMode/CvlistLen/Cvlist[16]/ClkDiv/
 * RxThresholdLevel/SpecialCh (21 bytes), ChanInType (u32, 4-aligned at 24),
 * ChIDEn -> 32 bytes.
 */

#define AMEBA_ADC_INIT_SIZE       32

/* APBPeriph_ADC / APBPeriph_ADC_CLOCK: bit 20 / bit 15 of the LSYS block,
 * with the bit30 group selector clear.  APBPeriph_CTC / _CTC_CLOCK are bit
 * 21 / bit 14.
 */

#define AMEBA_ADC_APBPERIPH       ((uint32_t)1 << 20)
#define AMEBA_ADC_APBPERIPH_CLK   ((uint32_t)1 << 15)
#define AMEBA_ADC_AUXCLK_PERIPH   ((uint32_t)1 << 21)
#define AMEBA_ADC_AUXCLK_CLK      ((uint32_t)1 << 14)

/* ADC_InitTypeDef has a ClkDiv field at offset 18 on this chip. */

#define AMEBA_ADC_HAS_CLKDIV      1

/* Conversion word layout, see above. */

#define AMEBA_ADC_GET_CHNO(x)     (((x) >> 12) & 0x0f)
#define AMEBA_ADC_GET_DATA(x)     ((x) & 0x0fff)

/* PAD_InputCtrl() is a header-only inline in the SDK on this chip, so the
 * ROM has no symbol for it.  It gates the digital input path of the ADC/CTC
 * pads (PA0..PA8) with one bit per pad in REG_LSYS_DUMMY_094 of the LSYS
 * block (SYSTEM_CTRL_BASE for the CA32 is 0x42008000, offset 0x2f8).
 */

#define RTL8730E_ADC_PADIN_REG    0x420082f8
#define RTL8730E_ADC_PADIN_MAXPAD 8     /* _PA_8 */

static inline void rtl8730e_adc_padinput(uint8_t pin, uint32_t enable)
{
  volatile uint32_t *reg = (volatile uint32_t *)RTL8730E_ADC_PADIN_REG;

  if (pin > RTL8730E_ADC_PADIN_MAXPAD)
    {
      return;
    }

  if (enable)
    {
      *reg |= (uint32_t)1 << pin;
    }
  else
    {
      *reg &= ~((uint32_t)1 << pin);
    }
}

#define AMEBA_ADC_PAD_INPUTCTRL(pin, state) \
  rtl8730e_adc_padinput(pin, state)

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_ADC_CHIP_H */
