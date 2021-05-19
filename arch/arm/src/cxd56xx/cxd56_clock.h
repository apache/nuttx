/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_clock.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CLOCK_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/cxd5602_topreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_spif_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spif_clock_enable(void);

/****************************************************************************
 * Name: cxd56_spif_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spif_clock_disable(void);

/****************************************************************************
 * Name: cxd56_get_cpu_baseclk
 *
 * Description:
 *   Get CPU clock.
 *
 ****************************************************************************/

uint32_t cxd56_get_cpu_baseclk(void);

/****************************************************************************
 * Name: cxd56_cpu_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_clock_enable(int cpu);

/****************************************************************************
 * Name: cxd56_cpulist_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_clock_enable(uint32_t cpus);

/****************************************************************************
 * Name: cxd56_cpu_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_clock_disable(int cpu);

/****************************************************************************
 * Name: cxd56_cpulist_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_clock_disable(uint32_t cpus);

/****************************************************************************
 * Name: cxd56_cpu_reset
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpu_reset(int cpu);

/****************************************************************************
 * Name: cxd56_cpulist_reset
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_cpulist_reset(uint32_t cpus);

/****************************************************************************
 * Name: cxd56_usb_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_usb_clock_enable(void);

/****************************************************************************
 * Name: cxd56_usb_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_usb_clock_disable(void);

/****************************************************************************
 * Name: cxd56_emmc_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_emmc_clock_enable(uint32_t div, uint32_t driver, uint32_t sample);

/****************************************************************************
 * Name: cxd56_emmc_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_emmc_clock_disable(void);

/****************************************************************************
 * Name: cxd56_sdio_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_sdio_clock_enable(void);

/****************************************************************************
 * Name: cxd56_sdio_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_sdio_clock_disable(void);

/****************************************************************************
 * Name: cxd56_audio_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_audio_clock_enable(uint32_t clk, uint32_t div);

/****************************************************************************
 * Name: cxd56_audio_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_audio_clock_disable(void);

/****************************************************************************
 * Name: cxd56_audio_clock_is_enabled
 *
 * Description:
 *
 ****************************************************************************/

bool cxd56_audio_clock_is_enabled(void);

/****************************************************************************
 * Name: cxd56_spi_clock_enable
 *
 * Description:
 *   Enable SPI device clock.
 *
 ****************************************************************************/

void cxd56_spi_clock_enable(int port);

/****************************************************************************
 * Name: cxd56_spi_clock_disable
 *
 * Description:
 *   Disable SPI device clock.
 *
 ****************************************************************************/

void cxd56_spi_clock_disable(int port);

/****************************************************************************
 * Name: cxd56_spi_clock_gate_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gate_enable(int port);

/****************************************************************************
 * Name: cxd56_spi_clock_gate_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gate_disable(int port);

/****************************************************************************
 * Name: cxd56_spi_clock_gear_adjust
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_spi_clock_gear_adjust(int port, uint32_t maxfreq);

/****************************************************************************
 * Name: cxd56_i2c0_clock_enable
 *
 * Description:
 *   Enable I2C device clock.
 *
 ****************************************************************************/

void cxd56_i2c_clock_enable(int port);

/****************************************************************************
 * Name: cxd56_i2c_clock_dsiable
 *
 * Description:
 *   Disable I2C device clock.
 *
 ****************************************************************************/

void cxd56_i2c_clock_disable(int port);

/****************************************************************************
 * Name: cxd56_i2c_clock_gate_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_gate_enable(int port);

/****************************************************************************
 * Name: cxd56_i2c_clock_gate_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_i2c_clock_gate_disable(int port);

/****************************************************************************
 * Name: cxd56_scuseq_is_clock_enabled
 *
 * Description:
 *   Get whether Sensor Control Unit Sequencer clock is enabled or not
 *
 ****************************************************************************/

bool cxd56_scuseq_clock_is_enabled(void);

/****************************************************************************
 * Name: cxd56_scuseq_clock_enable
 *
 * Description:
 *   Enable Sensor Control Unit Sequencer clock.
 *
 ****************************************************************************/

int cxd56_scuseq_clock_enable(void);

/****************************************************************************
 * Name: cxd56_scuseq_release_reset
 *
 * Description:
 *   Release sequencer reset. This function must be call after
 *   cxd56_scuseq_clock_enable() and copy sequencer firmware.
 *
 ****************************************************************************/

void cxd56_scuseq_release_reset(void);

/****************************************************************************
 * Name: cxd56_scuseq_clock_dsiable
 *
 * Description:
 *   Disable Sensor Control Unit Sequencer clock.
 *
 ****************************************************************************/

void cxd56_scuseq_clock_disable(void);

/****************************************************************************
 * Name: cxd56_img_uart_clock_enable
 *
 * Description:
 *   Enable img uart clock.
 *
 ****************************************************************************/

void cxd56_img_uart_clock_enable(void);

/****************************************************************************
 * Name: cxd56_img_uart_clock_dsiable
 *
 * Description:
 *   Disable img uart clock.
 *
 ****************************************************************************/

void cxd56_img_uart_clock_disable(void);

/****************************************************************************
 * Name: cxd56_get_img_uart_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_img_uart_baseclock(void);

/****************************************************************************
 * Name: cxd56_img_cisif_clock_enable
 *
 * Description:
 *   Enable cisif clock.
 *
 ****************************************************************************/

void cxd56_img_cisif_clock_enable(void);

/****************************************************************************
 * Name: cxd56_img_cisif_clock_dsiable
 *
 * Description:
 *   Disable cisif clock.
 *
 ****************************************************************************/

void cxd56_img_cisif_clock_disable(void);

/****************************************************************************
 * Name: cxd56_img_ge2d_clock_enable
 *
 * Description:
 *   Enable ge2d clock.
 *
 ****************************************************************************/

void cxd56_img_ge2d_clock_enable(void);

/****************************************************************************
 * Name: cxd56_img_ge2d_clock_dsiable
 *
 * Description:
 *   Disable ge2d clock.
 *
 ****************************************************************************/

void cxd56_img_ge2d_clock_disable(void);

/****************************************************************************
 * Name: cxd56_get_com_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_com_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_sdio_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_sdio_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_spi_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_spi_baseclock(int port);

/****************************************************************************
 * Name: cxd56_get_i2c_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_i2c_baseclock(int port);

/****************************************************************************
 * Name: cxd56_get_pwm_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_pwm_baseclock(void);

/****************************************************************************
 * Name: cxd56_udmac_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_udmac_clock_enable(void);

/****************************************************************************
 * Name: cxd56_udmac_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_udmac_clock_disable(void);

/****************************************************************************
 * Name: cxd56_lpadc_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_lpadc_clock_enable(uint32_t div);

/****************************************************************************
 * Name: cxd56_lpadc_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_lpadc_clock_disable(void);

/****************************************************************************
 * Name: cxd56_hpadc_clock_enable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_hpadc_clock_enable(uint32_t div);

/****************************************************************************
 * Name: cxd56_hpadc_clock_disable
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_hpadc_clock_disable(void);

/****************************************************************************
 * Name: cxd56_get_xosc_clock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_xosc_clock(void);

/****************************************************************************
 * Name: cxd56_get_rcosc_clock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_rcosc_clock(void);

/****************************************************************************
 * Name: cxd56_get_rtc_clock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_rtc_clock(void);

/****************************************************************************
 * Name: cxd56_get_syspll_clock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_syspll_clock(void);

/****************************************************************************
 * Name: cxd56_get_sys_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_sys_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_sys_ahb_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_sys_ahb_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_sys_apb_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_sys_apb_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_sys_sfc_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_sys_sfc_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_scu_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_scu_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_hpadc_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_hpadc_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_lpadc_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_lpadc_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_pmui2c_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_pmui2c_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_gps_cpu_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_gps_cpu_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_gps_ahb_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_gps_ahb_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_img_spi_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_img_spi_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_img_wspi_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_img_wspi_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_usb_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_usb_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_img_vsync_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_img_vsync_baseclock(void);

/****************************************************************************
 * Name: cxd56_get_appsmp_baseclock
 *
 * Description:
 *
 ****************************************************************************/

uint32_t cxd56_get_appsmp_baseclock(void);

/****************************************************************************
 * Name: cxd56_hostif_clock_enable
 *
 * Description:
 *   Enable clock of the hostif block
 *
 ****************************************************************************/

int cxd56_hostif_clock_enable(void);

/****************************************************************************
 * Name: cxd56_hostif_clock_disable
 *
 * Description:
 *   Disable clock of the hostif block
 *
 ****************************************************************************/

int cxd56_hostif_clock_disable(void);

/****************************************************************************
 * Name: cxd56_hostseq_clock_enable
 *
 * Description:
 *   Enable clock of the hostif sequencer block
 *
 ****************************************************************************/

int cxd56_hostseq_clock_enable(void);

/****************************************************************************
 * Name: cxd56_hostseq_clock_disable
 *
 * Description:
 *   Disable clock of the hostif sequencer block
 *
 ****************************************************************************/

int cxd56_hostseq_clock_disable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CLOCK_H */
