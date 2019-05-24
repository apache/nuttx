/****************************************************************************
 * arch/arm/src/lpc54xx/lpc546x_reset.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC546X_RESET_H
#define __ARCH_ARM_SRC_LPC54XX_LPC546X_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define lpc54_reset_peripheral0(m) \
  lpc54_reset(LPC54_SYSCON_PRESETCTRLSET0, LPC54_SYSCON_PRESETCTRLCLR0, \
              LPC54_SYSCON_PRESETCTRL0, (m))

#define lpc54_reset_peripheral1(m) \
  lpc54_reset(LPC54_SYSCON_PRESETCTRLSET1, LPC54_SYSCON_PRESETCTRLCLR1, \
              LPC54_SYSCON_PRESETCTRL1, (m))

#define lpc54_reset_peripheral2(m) \
  lpc54_reset(LPC54_SYSCON_PRESETCTRLSET2, LPC54_SYSCON_PRESETCTRLCLR2, \
              LPC54_SYSCON_PRESETCTRL2, (m))

#define lpc54_reset_async_peripheral(m) \
  lpc54_reset(LPC54_SYSCON_ASYNCPRESETCTRLSET, LPC54_SYSCON_ASYNCPRESETCTRLCLR, \
              LPC54_SYSCON_ASYNCPRESETCTRL, (m))

#define lpc54_reset_flash()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_FLASH)
#define lpc54_reset_fmc()       lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_FMC)
#define lpc54_reset_eeprom()    lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_EEPROM)
#define lpc54_reset_spifi()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_SPIFI)
#define lpc54_reset_inputmux()  lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_INPUTMUX)
#define lpc54_reset_iocon()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_IOCON)
#define lpc54_reset_gpio0()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_GPIO0)
#define lpc54_reset_gpio1()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_GPIO1)
#define lpc54_reset_gpio2()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_GPIO2)
#define lpc54_reset_gpio3()     lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_GPIO3)
#define lpc54_reset_pint()      lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_PINT)
#define lpc54_reset_gint()      lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_GINT)
#define lpc54_reset_dma()       lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_DMA)
#define lpc54_reset_crc()       lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_CRC)
#define lpc54_reset_wwdt()      lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_WWDT)
#define lpc54_reset_rtc()       lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_RTC)
#define lpc54_reset_adc0()      lpc54_reset_peripheral0(SYSCON_PRESETCTRL0_ADC0)

#define lpc54_reset_mrt()       lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_MRT)
#define lpc54_reset_sct0()      lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_SCT0)
#define lpc54_reset_mcan0()     lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_MCAN0)
#define lpc54_reset_mcan1()     lpc54_reset_peripheral1SYSCON_PRESETCTRL1_MCAN1)
#define lpc54_reset_utick()     lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_UTICK)
#define lpc54_reset_flexcomm0() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM0)
#define lpc54_reset_flexcomm1() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM1)
#define lpc54_reset_flexcomm2() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM2)
#define lpc54_reset_flexcomm3() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM3)
#define lpc54_reset_flexcomm4() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM4)
#define lpc54_reset_flexcomm5() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM5)
#define lpc54_reset_flexcomm6() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM6)
#define lpc54_reset_flexcomm7() lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_FLEXCOMM7)
#define lpc54_reset_dmic()      lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_DMIC)
#define lpc54_reset_ctimer2()   lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_CTIMER2)
#define lpc54_reset_usb0d()     lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_USB0D)
#define lpc54_reset_ctimer0()   lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_CTIMER0)
#define lpc54_reset_ctimer1()   lpc54_reset_peripheral1(SYSCON_PRESETCTRL1_CTIMER1)

#define lpc54_reset_lcd()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_LCD)
#define lpc54_reset_sdio()      lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_SDIO)
#define lpc54_reset_usb1h()     lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_USB1H)
#define lpc54_reset_usb1d()     lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_USB1D)
#define lpc54_reset_usb1ram()   lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_USB1RAM)
#define lpc54_reset_emc()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_EMC)
#define lpc54_reset_eth()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_ETH)
#define lpc54_reset_gpio4()     lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_GPIO4)
#define lpc54_reset_gpio5()     lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_GPIO5)
#define lpc54_reset_otp()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_OTP)
#define lpc54_reset_rng()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_RNG)
#define lpc54_reset_flexcomm8() lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_FLEXCOMM8)
#define lpc54_reset_flexcomm9() lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_FLEXCOMM9)
#define lpc54_reset_usb0hmr()   lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_USB0HMR)
#define lpc54_reset_usb0hsl()   lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_USB0HSL)
#define lpc54_reset_sha()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_SHA)
#define lpc54_reset_sc0()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_SC0)
#define lpc54_reset_sc1()       lpc54_reset_peripheral2(SYSCON_PRESETCTRL2_SC1)

#define lpc54_reset_ctimer3()   lpc54_reset_async_peripheral(SYSCON_ASYNCPRESET_CTIMER3)
#define lpc54_reset_ctimer4()   lpc54_reset_async_peripheral(SYSCON_ASYNCPRESET_CTIMER4)

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC546X_RESET_H */
