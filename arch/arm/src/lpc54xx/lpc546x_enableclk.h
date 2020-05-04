/****************************************************************************
 * arch/arm/src/lpc54xx/lpc546x_enableclk.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC546X_ENABLECLK_H
#define __ARCH_ARM_SRC_LPC54XX_LPC546X_ENABLECLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"
#include "hardware/lpc54_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define lpc54_periph0_enableclk(b)   putreg32((b), LPC54_SYSCON_AHBCLKCTRLSET0)
#define lpc54_periph1_enableclk(b)   putreg32((b), LPC54_SYSCON_AHBCLKCTRLSET1)
#define lpc54_periph2_enableclk(b)   putreg32((b), LPC54_SYSCON_AHBCLKCTRLSET2)
#define lpc54_asynch_enableclk(b)    putreg32((b), LPC54_SYSCON_ASYNCAPBCLKCTRLSET)

#define lpc54_periph0_disableclk(b)  putreg32((b), LPC54_SYSCON_AHBCLKCTRLCLR0)
#define lpc54_periph1_disableclk(b)  putreg32((b), LPC54_SYSCON_AHBCLKCTRLCLR1)
#define lpc54_periph2_disableclk(b)  putreg32((b), LPC54_SYSCON_AHBCLKCTRLCLR2)
#define lpc54_asynch_disableclk(b)   putreg32((b), LPC54_SYSCON_ASYNCAPBCLKCTRLCLR)

#define lpc54_periph0_isenabled(b)   ((getreg32(LPC54_SYSCON_AHBCLKCTRL0) & (b)) != 0)
#define lpc54_periph1_isenabled(b)   ((getreg32(LPC54_SYSCON_AHBCLKCTRL1) & (b)) != 0)
#define lpc54_periph2_isenabled(b)   ((getreg32(LPC54_SYSCON_AHBCLKCTRL2) & (b)) != 0)
#define lpc54_asynch_isenabled(b)    ((getreg32(LPC54_SYSCON_ASYNCAPBCLKCTRL) & (b)) != 0)

/* Enable peripheral clocking */

#define lpc54_rom_enableclk()        lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_ROM)
#define lpc54_sram1_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_SRAM1)
#define lpc54_sram2_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_SRAM2)
#define lpc54_sram3_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_SRAM3)
#define lpc54_flash_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_FLASH)
#define lpc54_fmc_enableclk()        lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_FMC)
#define lpc54_eeprom_enableclk()     lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_EEPROM)
#define lpc54_spifi_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_SPIFI)
#define lpc54_inputmux_enableclk()   lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_INPUTMUX)
#define lpc54_iocon_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_IOCON)
#define lpc54_gpio0_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_GPIO0)
#define lpc54_gpio1_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_GPIO1)
#define lpc54_gpio2_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_GPIO2)
#define lpc54_gpio3_enableclk()      lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_GPIO3)
#define lpc54_pint_enableclk()       lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_PINT)
#define lpc54_gint_enableclk()       lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_GINT)
#define lpc54_dma_enableclk()        lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_DMA)
#define lpc54_crc_enableclk()        lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_CRC)
#define lpc54_wwdt_enableclk()       lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_WWDT)
#define lpc54_rtc_enableclk()        lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_RTC)
#define lpc54_adc0_enableclk()       lpc54_periph0_enableclk(SYSCON_AHBCLKCTRL0_ADC0)

#define lpc54_mrt_enableclk()        lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_MRT)
#define lpc54_rit_enableclk()        lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_RIT)
#define lpc54_sct0_enableclk()       lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_SCT0)
#define lpc54_mcan0_enableclk()      lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_MCAN0)
#define lpc54_mcan1_enableclk()      lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_MCAN1)
#define lpc54_utick_enableclk()      lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_UTICK)
#define lpc54_flexcomm0_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM0)
#define lpc54_flexcomm1_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM1)
#define lpc54_flexcomm2_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM2)
#define lpc54_flexcomm3_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM3)
#define lpc54_flexcomm4_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM4)
#define lpc54_flexcomm5_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM5)
#define lpc54_flexcomm6_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM6)
#define lpc54_flexcomm7_enableclk()  lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM7)
#define lpc54_dmic_enableclk()       lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_DMIC)
#define lpc54_ctimer2_enableclk()    lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_CTIMER2)
#define lpc54_usb0d_enableclk()      lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_USB0D)
#define lpc54_ctimer0_enableclk()    lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_CTIMER0)
#define lpc54_ctimer1_enableclk()    lpc54_periph1_enableclk(SYSCON_AHBCLKCTRL1_CTIMER1)

#define lpc54_lcd_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_LCD)
#define lpc54_sdmmc_enableclk()      lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_SDIO)
#define lpc54_usb1h_enableclk()      lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_USB1H)
#define lpc54_usb1d_enableclk()      lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_USB1D)
#define lpc54_usb1ram_enableclk()    lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_USB1RAM)
#define lpc54_emc_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_EMC)
#define lpc54_eth_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_ETH)
#define lpc54_gpio4_enableclk()      lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_GPIO4)
#define lpc54_gpio5_enableclk()      lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_GPIO5)
#define lpc54_otp_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_OTP)
#define lpc54_rng_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_RNG)
#define lpc54_flexcomm8_enableclk()  lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_FLEXCOMM8)
#define lpc54_flexcomm9_enableclk()  lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_FLEXCOMM9)
#define lpc54_usb0hmr_enableclk()    lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_USB0HMR)
#define lpc54_usb0hsl_enableclk()    lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_USB0HSL)
#define lpc54_sha_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_SHA)
#define lpc54_sc0_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_SC0)
#define lpc54_sc1_enableclk()        lpc54_periph2_enableclk(SYSCON_AHBCLKCTRL2_SC1)

#define lpc54_ctimer3_enableclk()    lpc54_asynch_enableclk(SYSCON_ASYNCAPBCLKCTRL_CTIMER3)
#define lpc54_ctimer4_enableclk()    lpc54_asynch_enableclk(SYSCON_ASYNCAPBCLKCTRL_CTIMER4)

/* Disable peripheral clocking */

#define lpc54_rom_disableclk()       lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_ROM)
#define lpc54_sram1_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_SRAM1)
#define lpc54_sram2_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_SRAM2)
#define lpc54_sram3_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_SRAM3)
#define lpc54_flash_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_FLASH)
#define lpc54_fmc_disableclk()       lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_FMC)
#define lpc54_eeprom_disableclk()    lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_EEPROM)
#define lpc54_spifi_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_SPIFI)
#define lpc54_inputmux_disableclk()  lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_INPUTMUX)
#define lpc54_iocon_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_IOCON)
#define lpc54_gpio0_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_GPIO0)
#define lpc54_gpio1_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_GPIO1)
#define lpc54_gpio2_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_GPIO2)
#define lpc54_gpio3_disableclk()     lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_GPIO3)
#define lpc54_pint_disableclk()      lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_PINT)
#define lpc54_gint_disableclk()      lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_GINT)
#define lpc54_dma_disableclk()       lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_DMA)
#define lpc54_crc_disableclk()       lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_CRC)
#define lpc54_wwdt_disableclk()      lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_WWDT)
#define lpc54_rtc_disableclk()       lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_RTC)
#define lpc54_adc0_disableclk()      lpc54_periph0_disableclk(SYSCON_AHBCLKCTRL0_ADC0)

#define lpc54_mrt_disableclk()       lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_MRT)
#define lpc54_rit_disableclk()       lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_RIT)
#define lpc54_sct0_disableclk()      lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_SCT0)
#define lpc54_mcan0_disableclk()     lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_MCAN0)
#define lpc54_mcan1_disableclk()     lpc54_periph1_disableclkSYSCON_AHBCLKCTRL1_MCAN1)
#define lpc54_utick_disableclk()     lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_UTICK)
#define lpc54_flexcomm0_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM0)
#define lpc54_flexcomm1_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM1)
#define lpc54_flexcomm2_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM2)
#define lpc54_flexcomm3_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM3)
#define lpc54_flexcomm4_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM4)
#define lpc54_flexcomm5_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM5)
#define lpc54_flexcomm6_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM6)
#define lpc54_flexcomm7_disableclk() lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_FLEXCOMM7)
#define lpc54_dmic_disableclk()      lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_DMIC)
#define lpc54_ctimer2_disableclk()   lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_CTIMER2)
#define lpc54_usb0d_disableclk()     lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_USB0D)
#define lpc54_ctimer0_disableclk()   lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_CTIMER0)
#define lpc54_ctimer1_disableclk()   lpc54_periph1_disableclk(SYSCON_AHBCLKCTRL1_CTIMER1)

#define lpc54_lcd_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_LCD)
#define lpc54_sdmmc_disableclk()     lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_SDIO)
#define lpc54_usb1h_disableclk()     lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_USB1H)
#define lpc54_usb1d_disableclk()     lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_USB1D)
#define lpc54_usb1ram_disableclk()   lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_USB1RAM)
#define lpc54_emc_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_EMC)
#define lpc54_eth_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_ETH)
#define lpc54_gpio4_disableclk()     lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_GPIO4)
#define lpc54_gpio5_disableclk()     lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_GPIO5)
#define lpc54_otp_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_OTP)
#define lpc54_rng_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_RNG)
#define lpc54_flexcomm8_disableclk() lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_FLEXCOMM8)
#define lpc54_flexcomm9_disableclk() lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_FLEXCOMM9)
#define lpc54_usb0hmr_disableclk()   lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_USB0HMR)
#define lpc54_usb0hsl_disableclk()   lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_USB0HSL)
#define lpc54_sha_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_SHA)
#define lpc54_sc0_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_SC0)
#define lpc54_sc1_disableclk()       lpc54_periph2_disableclk(SYSCON_AHBCLKCTRL2_SC1)

#define lpc54_ctimer3_disableclk()   lpc54_asynch_disableclk(SYSCON_ASYNCAPBCLKCTRL_CTIMER3)
#define lpc54_ctimer4_disableclk()   lpc54_asynch_disableclk(SYSCON_ASYNCAPBCLKCTRL_CTIMER4)

/* Check if peripheral clocking is enabled */

#define lpc54_rom_isenabled()        lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_ROM)
#define lpc54_sram1_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_SRAM1)
#define lpc54_sram2_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_SRAM2)
#define lpc54_sram3_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_SRAM3)
#define lpc54_flash_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_FLASH)
#define lpc54_fmc_isenabled()        lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_FMC)
#define lpc54_eeprom_isenabled()     lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_EEPROM)
#define lpc54_spifi_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_SPIFI)
#define lpc54_inputmux_isenabled()   lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_INPUTMUX)
#define lpc54_iocon_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_IOCON)
#define lpc54_gpio0_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_GPIO0)
#define lpc54_gpio1_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_GPIO1)
#define lpc54_gpio2_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_GPIO2)
#define lpc54_gpio3_isenabled()      lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_GPIO3)
#define lpc54_pint_isenabled()       lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_PINT)
#define lpc54_gint_isenabled()       lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_GINT)
#define lpc54_dma_isenabled()        lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_DMA)
#define lpc54_crc_isenabled()        lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_CRC)
#define lpc54_wwdt_isenabled()       lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_WWDT)
#define lpc54_rtc_isenabled()        lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_RTC)
#define lpc54_adc0_isenabled()       lpc54_periph0_isenabled(SYSCON_AHBCLKCTRL0_ADC0)

#define lpc54_mrt_isenabled()        lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_MRT)
#define lpc54_rit_isenabled()        lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_RIT)
#define lpc54_sct0_isenabled()       lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_SCT0)
#define lpc54_mcan0_isenabled()      lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_MCAN0)
#define lpc54_mcan1_isenabled()      lpc54_periph1_isenabledSYSCON_AHBCLKCTRL1_MCAN1)
#define lpc54_utick_isenabled()      lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_UTICK)
#define lpc54_flexcomm0_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM0)
#define lpc54_flexcomm1_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM1)
#define lpc54_flexcomm2_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM2)
#define lpc54_flexcomm3_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM3)
#define lpc54_flexcomm4_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM4)
#define lpc54_flexcomm5_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM5)
#define lpc54_flexcomm6_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM6)
#define lpc54_flexcomm7_isenabled()  lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_FLEXCOMM7)
#define lpc54_dmic_isenabled()       lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_DMIC)
#define lpc54_ctimer2_isenabled()    lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_CTIMER2)
#define lpc54_usb0d_isenabled()      lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_USB0D)
#define lpc54_ctimer0_isenabled()    lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_CTIMER0)
#define lpc54_ctimer1_isenabled()    lpc54_periph1_isenabled(SYSCON_AHBCLKCTRL1_CTIMER1)

#define lpc54_lcd_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_LCD)
#define lpc54_sdmmc_isenabled()      lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_SDIO)
#define lpc54_usb1h_isenabled()      lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_USB1H)
#define lpc54_usb1d_isenabled()      lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_USB1D)
#define lpc54_usb1ram_isenabled()    lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_USB1RAM)
#define lpc54_emc_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_EMC)
#define lpc54_eth_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_ETH)
#define lpc54_gpio4_isenabled()      lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_GPIO4)
#define lpc54_gpio5_isenabled()      lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_GPIO5)
#define lpc54_otp_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_OTP)
#define lpc54_rng_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_RNG)
#define lpc54_flexcomm8_isenabled()  lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_FLEXCOMM8)
#define lpc54_flexcomm9_isenabled()  lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_FLEXCOMM9)
#define lpc54_usb0hmr_isenabled()    lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_USB0HMR)
#define lpc54_usb0hsl_isenabled()    lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_USB0HSL)
#define lpc54_sha_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_SHA)
#define lpc54_sc0_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_SC0)
#define lpc54_sc1_isenabled()        lpc54_periph2_isenabled(SYSCON_AHBCLKCTRL2_SC1)

#define lpc54_ctimer3_isenabled()    lpc54_asynch_isenabled(SYSCON_ASYNCAPBCLKCTRL_CTIMER3)
#define lpc54_ctimer4_isenabled()    lpc54_asynch_isenabled(SYSCON_ASYNCAPBCLKCTRL_CTIMER4)

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC546X_ENABLECLK_H */
