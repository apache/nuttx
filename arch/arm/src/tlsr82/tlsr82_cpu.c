/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_cpu.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <assert.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "tlsr82_cpu.h"
#include "tlsr82_analog.h"
#include "tlsr82_clock.h"
#include "tlsr82_gpio.h"
#include "tlsr82_timer.h"
#include "hardware/tlsr82_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define XTAL_READY_CHECK_TIMING_OPTIMIZE 1
#define BLC_PM_DEEP_RETENTION_MODE_EN 0

void locate_code (".ram_code") tlsr82_pm_wait_bbpll_done(void)
{
  volatile uint8_t i;
  uint8_t j;
  uint8_t ana_81 = tlsr82_analog_read(0x81);

  tlsr82_analog_write(0x81, ana_81 | BIT (6));
  for (j = 0; j < 3; j++)
    {
      /* Delay about 20us */

      for (i = 0; i < 30; i++)
        {
          asm("tnop");
        }

      if (BIT (5) == (tlsr82_analog_read(0x88) & BIT (5)))
        {
          tlsr82_analog_write(0x81, ana_81 & 0xbf);
          break;
        }
      else
        {
          if (j == 0)
            {
              tlsr82_analog_write(0x01, 0x4e);
            }
          else if (j == 1)
            {
              tlsr82_analog_write(0x01, 0x4b);
            }
          else
            {
              tlsr82_analog_write(0x81, ana_81 & 0xbf);
            }
        }
    }
}

void tlsr82_cpu_wakeup_init(power_mode_t power_mode, xtal_t xtal)
{
  /* This code strictly follow the telink code, just modity the comment */

  /* poweron_default : 0x7c -> 0x00
   * poweron_default : 0xff -> 0x00
   * poweron_default : 0xcf -> 0xc8
   * <7> : mcic1,R  default:1
   * <6> : risc1,R  default:1
   * <3> : trng     default:1
   */

  write_reg8(0x60, 0x00);
  write_reg8(0x61, 0x00);
  write_reg8(0x62, 0x08);

  /* poweron_default : 0x83 -> 0xff
   * poweron_default : 0x00 -> 0x9f
   * <5/6> : undefined, default:0
   */

  write_reg8(0x63, 0xff);
  write_reg8(0x64, 0xff);

  /* poweron_default : 0x30 -> 0x37
   * <0> : aif -> disable 32k for qdec
   * <5/6> : undefined, default:0
   */

  write_reg8(0x65, 0xf7);

  /* when load code twice without power down dut, dut will
   * use crystal clock in here, xo_quick_settle manual mode
   * need to use in RC colck.
   * poweron_default : set RC clock as system clock.
   */

  write_reg8(0x66, 0x06);

  /* poweron_default : 0xcf -> 0xce
   * <0> : power on bbpll LDO
   * <4> : power on otp LDO
   */

  tlsr82_analog_write(0x06, 0xce);

  /* xo_quict_settle with auto will cause the pragram crash
   * in high temperature, must be set to manual mode.
   * (add by Yi Bao, confirmed by wenfeng 20190911)
   */

  unsigned char ana_2c = tlsr82_analog_read(0x2c);
  unsigned char ana_05 = tlsr82_analog_read(0x05);

  /* 0x2c<5>: xo_quick_rst
   *          <5> 1b'1: xtal 24M quick settle count
   * 0x05<3>: 24M_xtl_pd
   *          <3> 1b'0 : Power up 24MHz XTL oscillator
   *          <3> 1b'1 : Power down 24MHz XTL oscillator
   */

  tlsr82_analog_write(0x2c, ana_2c | 0x20);
  tlsr82_analog_write(0x2c, ana_2c & 0xdf);
  tlsr82_analog_write(0x05, ana_05 | 0x08);
  tlsr82_analog_write(0x05, ana_05 & 0xf7);

  /* poweron_default: 0x05 -> 0x25
   * <3:0>: bbpll setting
   *   <5>: enable 48M clock to digital core
   *   <6>: enable signal of 24M clock to sar
   */

  tlsr82_analog_write(0x82, 0x65);

  /* poweron_default: 0x07 -> 0x80
   * <0>: power on baseband
   * <1>: power on usb
   * <2>: power on audio
   * <7>: enable change power sequence clk
   */

  tlsr82_analog_write(0x34, 0x80);

  /* When using the BDT tool to download a program through USB,
   * if the dp pull-up is turned off, the device will be disconnected,
   * so the dp pull-up is set to be keep, modify by kaixin(2019.12.27).
   */

  /* default: 0x7b
   *   <7>: enable_b signal of 1.5K pullup resistor for DP PAD--keep
   *   <6>: enable signal of 1M pullup resistor for mscn PAD, avoid current
   *        leakage 1->0
   * <5:4>: reference scale select 11->11
   * <1:0>: power on native 1.8v/1.4v 11->00
   */

  tlsr82_analog_write(0x0b, (tlsr82_analog_read(0x0b) & 0x80) | 0x38);

  /* poweron_default: 0x00
   * <1> set 1: reg_xo_en_clk_ana_ana, to enable external 24M crystal
   */

  tlsr82_analog_write(0x8c, 0x02);

  /* poweron_default: 0x18
   * <7> MSB of ret_ldo_trim, 0: 0.8-1.15V; 1: 0.6-0.95V
   */

  tlsr82_analog_write(0x00, tlsr82_analog_read(0x00) & 0x7f);

  /* poweron_default: 0xa4,
   * <2:0> ret_ldo_trim, set 0x04: 1.00V
   */

  tlsr82_analog_write(0x02, 0xa4);

  /* pragram can crash in high temperature,
   * ana_01 and pm_wait_bbpll_done() is order to solve this problem.
   */

  tlsr82_analog_write(0x01, 0x4d);

  if (xtal == EXTERNAL_XTAL_48M)
    {
      /* use external 48M crystal instead of external 24M crystal
       * poweron_default: 0x15
       * <6>: 0 - 24M XTAL
       *      1 - 48M XTAL
       */

      tlsr82_analog_write(0x2d, tlsr82_analog_read(0x2d) | BIT (6));
    }

  /* poweron_default: 0x40
   * <1:0>: 00 - LDO
   *        01 - LDO_DC
   *        11 - DCDC
   * when use the LDO_1P4_DCDC_1P8, should use synchronize mode
   * (0x0a <2>: 1-synchronize mode, 0-asynchronize mode) to avoid
   * the current abnormal in A0.
   */

  tlsr82_analog_write(0x0a, power_mode);

  if (LDO_MODE != power_mode)
    {
      /* poweron_default: 0xc4, 1.4v voltage turn dowm 100mv */

      tlsr82_analog_write(0x0c, 0xa4);
    }

  /* poweron_default: 0x00, PA gpio wakeup disable */

  tlsr82_analog_write(0x27, 0x00);

  /* poweron_default: 0x00 PB gpio wakeup disable */

  tlsr82_analog_write(0x28, 0x00);

  /* poweron_default: 0x00 PC gpio wakeup disable */

  tlsr82_analog_write(0x29, 0x00);

  /* poweron_default: 0x00 PD gpio wakeup disable */

  tlsr82_analog_write(0x2a, 0x00);

  /* poweron_default: 0x00000000 -> 0x04040404 */

  write_reg32(0xc40, 0x04040404);

  /* poweron_default: 0x00000000 -> 0x04040404 */

  write_reg32(0xc44, 0x04040404);

  /* poweron_default: 0x00 -> 0x04 */

  write_reg8(0xc48, 0x04);

  /* core_c20/c21 power on default all enable, so we disable them first,
   * then if use, enable them by manual
   * note that: PWM/RF Tx/RF Rx/AES code/AES dcode dma may be affected
   * by this, you must handle them when initialization.
   */

  DMA_IRQ_EN_REG          = 0;
  DMA_IRQ_MASK_REG        = 0;
  GPIO_IRQ_NORMAL_ALL_REG = GPIO_IRQ_NORMAL_ALL_WAKEUP |
                            GPIO_IRQ_NORMAL_ALL_WAKEUP;

  /* xo_ready check should be done after Xtal manual on_off, we put it here
   * to save code running time, code running time between Xtal manual on_off
   * and xo_ready check can be used as Xtal be stable timimg
   * 0x88<7>: xo_ready_ana
   */

  while (BIT (7) != (tlsr82_analog_read(0x88) & (BIT (7))));

  tlsr82_pm_wait_bbpll_done();

  /* System timer config, enable system timer and interrupt */

  SYSTIMER_CTRL_REG     |= SYSTIMER_CTRL_TIMER_EN | SYSTIMER_CTRL_CALI_EN;
  SYSTIMER_IRQ_MASK_REG |= SYSTIMER_IRQ_MASK_EN;
}
