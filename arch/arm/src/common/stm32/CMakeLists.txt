# ##############################################################################
# arch/arm/src/common/stm32/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

target_include_directories(arch BEFORE PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(CONFIG_BUILD_PROTECTED)
  target_include_directories(arch_interface BEFORE
                             PUBLIC ${CMAKE_CURRENT_LIST_DIR})
endif()

set(SRCS)

if(CONFIG_STM32_COMMON_LEGACY)
  list(
    APPEND
    SRCS
    stm32_allocateheap_m3m4_v1.c
    stm32_start_m3m4_v1.c
    stm32_lse_m3m4_v1.c
    stm32_lsi_m3m4_v1.c
    stm32_irq_m3m4_v1.c
    stm32_waste_m3m4_v1.c
    stm32_capture_m3m4_v1.c)

  if(CONFIG_STM32_HAVE_IP_GPIO_M3M4_V1)
    list(APPEND SRCS stm32_gpio_m3m4_v1v2.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_EXTI_V1 OR CONFIG_STM32_HAVE_IP_EXTI_V2)
    list(APPEND SRCS stm32_exti_gpio_m3m4_v1v2.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_SPI_V1
     OR CONFIG_STM32_HAVE_IP_SPI_V2
     OR CONFIG_STM32_HAVE_IP_SPI_V3
     OR CONFIG_STM32_HAVE_IP_SPI_V4)
    list(APPEND SRCS stm32_spi_m3m4_v2v3v4.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_I2S_M3M4_V1)
    list(APPEND SRCS stm32_i2s_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_SDIO_M3M4_V1)
    list(APPEND SRCS stm32_sdio_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_TIMERS)
    list(APPEND SRCS stm32_tim_m3m4_v1v2v3.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_CCM_M3M4_V1)
    list(APPEND SRCS stm32_ccm_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_UID_M3M4_V1)
    list(APPEND SRCS stm32_uid_m3m4_v1v2.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_DFUMODE_M3M4_V1)
    list(APPEND SRCS stm32_dfumode_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_USART_V1
     OR CONFIG_STM32_HAVE_IP_USART_V2
     OR CONFIG_STM32_HAVE_IP_USART_V3
     OR CONFIG_STM32_HAVE_IP_USART_V4)
    list(APPEND SRCS stm32_lowputc_usart_m3m4_v1v2v3v4.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_FLASH_M3M4_L1)
    list(APPEND SRCS stm32_flash_m3m4_l1.c)
  elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_F1F3)
    list(APPEND SRCS stm32_flash_m3m4_f1f3.c)
  elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_F2F4)
    list(APPEND SRCS stm32_flash_m3m4_f2f4.c)
  elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_G4)
    list(APPEND SRCS stm32_flash_m3m4_g4.c)
  endif()

  if(CONFIG_STM32_TICKLESS_TIMER)
    list(APPEND SRCS stm32_tickless_m3m4_v1.c)
  elseif(CONFIG_ARCH_ARMV6M)
    list(APPEND SRCS stm32_timerisr_armv6m.c)
  else()
    list(APPEND SRCS stm32_timerisr_armv7m.c)
  endif()

  if(CONFIG_BUILD_PROTECTED)
    list(APPEND SRCS stm32_userspace_m3m4_v1.c stm32_mpuinit_m3m4_v1.c)
  endif()

  if(NOT CONFIG_ARCH_IDLE_CUSTOM)
    list(APPEND SRCS stm32_idle_m3m4_v1.c)
  endif()

  list(APPEND SRCS stm32_pmstop_m3m4_v1.c stm32_pmstandby_m3m4_v1.c
       stm32_pmsleep_m3m4_v1.c)

  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS stm32_pminitialize_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_USART)
    if(CONFIG_STM32_HAVE_IP_USART_V1
       OR CONFIG_STM32_HAVE_IP_USART_V2
       OR CONFIG_STM32_HAVE_IP_USART_V3
       OR CONFIG_STM32_HAVE_IP_USART_V4)
      list(APPEND SRCS stm32_serial_m3m4_v1v2v3v4.c)
    endif()
  endif()

  if(CONFIG_STM32_DMA)
    if(CONFIG_STM32_HAVE_IP_DMA_V1_7CH_DMAMUX)
      list(APPEND SRCS stm32_dma_m0_v1_7ch_dmamux.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_7CH)
      list(APPEND SRCS stm32_dma_m0_v1_7ch.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_8CH_DMAMUX)
      list(APPEND SRCS stm32_dma_m3m4_v1_8ch_dmamux.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_8CH)
      list(APPEND SRCS stm32_dma_m3m4_v1_8ch.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V2_STREAM)
      list(APPEND SRCS stm32_dma_m3m4_v2_stream.c)
    endif()
  endif()

  if(CONFIG_TIMER AND CONFIG_STM32_HAVE_IP_TIMERS)
    list(APPEND SRCS stm32_tim_m3m4_v1v2v3_lowerhalf.c)
  endif()

  if(CONFIG_STM32_ONESHOT)
    list(APPEND SRCS stm32_oneshot_m3m4_v1.c stm32_oneshot_m3m4_v1_lowerhalf.c)
  endif()

  if(CONFIG_STM32_FREERUN)
    list(APPEND SRCS stm32_freerun_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_I2C_M3M4_V1)
    if(CONFIG_STM32_I2C_ALT)
      list(APPEND SRCS stm32_i2c_m3m4_v1_alt.c)
    elseif(CONFIG_STM32_STM32F4XXX)
      list(APPEND SRCS stm32_i2c_m3m4_v1_f40xxx.c)
    else()
      list(APPEND SRCS stm32_i2c_m3m4_v1.c)
    endif()
  elseif(CONFIG_STM32_HAVE_IP_I2C_M3M4_V2)
    list(APPEND SRCS stm32_i2c_m3m4_v2.c)
    if(CONFIG_STM32_I2C_SLAVE)
      list(APPEND SRCS stm32_i2c_m3m4_v2_slave.c)
    endif()
  endif()

  if(CONFIG_USBDEV)
    if(CONFIG_STM32_USB AND CONFIG_STM32_HAVE_IP_USBDEV_M3M4_V1)
      list(APPEND SRCS stm32_usbdev_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_USBFS AND CONFIG_STM32_HAVE_IP_USBFS_M3M4_V1)
      list(APPEND SRCS stm32_usbfs_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_OTGFS AND CONFIG_STM32_HAVE_IP_OTGFS_M3M4_V1)
      list(APPEND SRCS stm32_otgfsdev_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_OTGHS AND CONFIG_STM32_HAVE_IP_OTGHS_M3M4_V1)
      list(APPEND SRCS stm32_otghsdev_m3m4_v1.c)
    endif()
  endif()

  if(CONFIG_STM32_USBHOST)
    if(CONFIG_STM32_OTGFS AND CONFIG_STM32_HAVE_IP_OTGFS_M3M4_V1)
      list(APPEND SRCS stm32_otgfshost_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_OTGHS AND CONFIG_STM32_HAVE_IP_OTGHS_M3M4_V1)
      list(APPEND SRCS stm32_otghshost_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_HAVE_COMMON_USBHOST_DEBUG AND (CONFIG_USBHOST_TRACE
                                                   OR CONFIG_DEBUG_USB))
      list(APPEND SRCS stm32_usbhost_m3m4_v1.c)
    endif()
  endif()

  if(CONFIG_STM32_ETHMAC AND CONFIG_STM32_HAVE_IP_ETHMAC_M3M4_V1)
    list(APPEND SRCS stm32_eth_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_PWR AND CONFIG_STM32_HAVE_IP_PWR_M3M4_V1)
    list(APPEND SRCS stm32_pwr_m3m4_v1.c stm32_exti_pwr_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_RTC)
    if(CONFIG_STM32_HAVE_IP_RTC_COUNTER_M3M4_V1)
      list(APPEND SRCS stm32_rtcounter_m3m4_v1.c)
    elseif(CONFIG_STM32_HAVE_IP_RTCC_M3M4_L1)
      list(APPEND SRCS stm32_rtcc_m3m4_l1.c)
    elseif(CONFIG_STM32_HAVE_IP_RTCC_M3M4_F4)
      list(APPEND SRCS stm32_rtcc_m3m4_f4.c)
    elseif(CONFIG_STM32_HAVE_IP_RTCC_M3M4_V1)
      list(APPEND SRCS stm32_rtcc_m3m4_v1.c)
    endif()
    if(CONFIG_RTC_ALARM)
      list(APPEND SRCS stm32_exti_alarm_m3m4_v1.c)
    endif()
    if(CONFIG_RTC_PERIODIC)
      list(APPEND SRCS stm32_exti_wakeup_m3m4_v1.c)
    endif()
    if(CONFIG_RTC_DRIVER AND CONFIG_STM32_HAVE_IP_RTC_M3M4_V1)
      list(APPEND SRCS stm32_rtc_m3m4_v1_lowerhalf.c)
    endif()
  endif()

  if(CONFIG_STM32_ADC
     AND (CONFIG_STM32_HAVE_IP_ADC_M3M4_V1
          OR CONFIG_STM32_HAVE_IP_ADC_M3M4_V2))
    list(APPEND SRCS stm32_adc_m3m4_v1v2.c)
  endif()

  if(CONFIG_STM32_SDADC AND CONFIG_STM32_HAVE_IP_SDADC_M3M4_V1)
    list(APPEND SRCS stm32_sdadc_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_DAC
     AND (CONFIG_STM32_HAVE_IP_DAC_M3M4_V1
          OR CONFIG_STM32_HAVE_IP_DAC_M3M4_V2))
    list(APPEND SRCS stm32_dac_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_COMP)
    if(CONFIG_STM32_HAVE_IP_COMP_M3M4_V1)
      list(APPEND SRCS stm32_comp_m3m4_v1.c)
    elseif(CONFIG_STM32_HAVE_IP_COMP_M3M4_V2)
      list(APPEND SRCS stm32_comp_m3m4_v2.c)
    endif()
  endif()

  if(CONFIG_STM32_OPAMP AND CONFIG_STM32_HAVE_IP_OPAMP_M3M4_V1)
    list(APPEND SRCS stm32_opamp_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HRTIM AND CONFIG_STM32_HAVE_IP_HRTIM_M3M4_V1)
    list(APPEND SRCS stm32_hrtim_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_1WIREDRIVER)
    list(APPEND SRCS stm32_1wire_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_HCIUART)
    list(APPEND SRCS stm32_hciuart_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_RNG AND CONFIG_STM32_HAVE_IP_RNG_M3M4_V1)
    list(APPEND SRCS stm32_rng_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_LTDC AND CONFIG_STM32_HAVE_IP_LTDC_M3M4_V1)
    list(APPEND SRCS stm32_ltdc_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_DMA2D AND CONFIG_STM32_HAVE_IP_DMA2D_M3M4_V1)
    list(APPEND SRCS stm32_dma2d_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_PWM)
    list(APPEND SRCS stm32_pwm_m3m4_v1v2v3.c)
  endif()

  if(CONFIG_STM32_PULSECOUNT)
    list(APPEND SRCS stm32_pulsecount_m3m4_v1v2v3.c)
  endif()

  if(CONFIG_STM32_CAP)
    list(APPEND SRCS stm32_capture_m3m4_v1_lowerhalf.c)
  endif()

  if(CONFIG_SENSORS_QENCODER AND CONFIG_STM32_QE)
    list(APPEND SRCS stm32_qencoder_m3m4_v1v2v3.c)
  endif()

  if(CONFIG_STM32_CAN AND CONFIG_STM32_HAVE_IP_CAN_BXCAN_M3M4_V1)
    if(CONFIG_STM32_CAN_CHARDRIVER)
      list(APPEND SRCS stm32_can_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_CAN_SOCKET)
      list(APPEND SRCS stm32_can_m3m4_v1_sock.c)
    endif()
  endif()

  if(CONFIG_STM32_FDCAN AND CONFIG_STM32_HAVE_IP_FDCAN_MCAN_M3M4_V1)
    if(CONFIG_STM32_FDCAN_CHARDRIVER)
      list(APPEND SRCS stm32_fdcan_m3m4_v1.c)
    endif()
    if(CONFIG_STM32_FDCAN_SOCKET)
      list(APPEND SRCS stm32_fdcan_m3m4_v1_sock.c)
    endif()
  endif()

  if(CONFIG_STM32_IWDG AND CONFIG_STM32_HAVE_IP_WDG_M3M4_V1)
    list(APPEND SRCS stm32_iwdg_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_WWDG AND CONFIG_STM32_HAVE_IP_WDG_M3M4_V1)
    list(APPEND SRCS stm32_wwdg_m3m4_v1.c)
  endif()

  if(CONFIG_DEBUG_FEATURES
     AND (CONFIG_STM32_HAVE_IP_DBGMCU_M3M4_V1
          OR CONFIG_STM32_HAVE_IP_DBGMCU_M3M4_V2
          OR CONFIG_STM32_HAVE_IP_DBGMCU_M3M4_V3))
    list(APPEND SRCS stm32_dumpgpio_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_AES AND CONFIG_STM32_HAVE_IP_AES_M3M4_V1)
    list(APPEND SRCS stm32_aes_m3m4_v1.c)
  endif()

  if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE AND CONFIG_STM32_HAVE_IP_CRYPTO_M3M4_V1)
    list(APPEND SRCS stm32_crypto_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_BBSRAM AND CONFIG_STM32_HAVE_IP_BBSRAM_M3M4_V1)
    list(APPEND SRCS stm32_bbsram_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_FMC AND CONFIG_STM32_HAVE_IP_FMC_M3M4_V1)
    list(APPEND SRCS stm32_fmc_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_FSMC AND CONFIG_STM32_HAVE_IP_FSMC_M3M4_V1)
    list(APPEND SRCS stm32_fsmc_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_FOC)
    list(APPEND SRCS stm32_foc_m3m4_v1.c)
  endif()

  if(CONFIG_STM32_CORDIC AND CONFIG_STM32_HAVE_IP_CORDIC_M3M4_V1)
    list(APPEND SRCS stm32_cordic_m3m4_v1.c)
  endif()
endif()

if(CONFIG_ARCH_CORTEXM0)
  list(
    APPEND
    SRCS
    stm32_irq_m0_v1.c
    stm32_start_m0_v1.c
    stm32_lsi_m0_v1.c)

  if(CONFIG_STM32_HAVE_IP_GPIO_M0_V1)
    list(APPEND SRCS stm32_gpio_m0_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_EXTI_V1 OR CONFIG_STM32_HAVE_IP_EXTI_V2)
    list(APPEND SRCS stm32_exti_gpio_m0_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_UID_M0_V1)
    list(APPEND SRCS stm32_uid_m0_v1.c)
  endif()

  if(CONFIG_STM32_HAVE_IP_USART_V3)
    list(APPEND SRCS stm32_lowputc_usart_m0_v3.c stm32_serial_m0_v3.c)
  elseif(CONFIG_STM32_HAVE_IP_USART_V4)
    list(APPEND SRCS stm32_lowputc_usart_m0_v4.c stm32_serial_m0_v4.c)
  endif()

  if(CONFIG_STM32_RTC_LSECLOCK OR CONFIG_LCD_LSECLOCK)
    list(APPEND SRCS stm32_lse_m0_v1.c)
  endif()

  if(NOT CONFIG_ARCH_IDLE_CUSTOM)
    list(APPEND SRCS stm32_idle_m0_v1.c)
  endif()

  if(NOT CONFIG_SCHED_TICKLESS)
    if(CONFIG_ARCH_ARMV6M)
      list(APPEND SRCS stm32_timerisr_armv6m.c)
    else()
      list(APPEND SRCS stm32_timerisr_armv7m.c)
    endif()
  endif()

  if(CONFIG_STM32_PWR)
    if(CONFIG_STM32_HAVE_IP_PWR_M0_V1)
      list(APPEND SRCS stm32_pwr_m0_v1.c)
    elseif(CONFIG_STM32_HAVE_IP_PWR_G0)
      list(APPEND SRCS stm32_pwr_m0_g0.c)
    endif()
  endif()

  if(CONFIG_STM32_DMA)
    if(CONFIG_STM32_HAVE_IP_DMA_V1_7CH_DMAMUX)
      list(APPEND SRCS stm32_dma_m0_v1_7ch_dmamux.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_7CH)
      list(APPEND SRCS stm32_dma_m0_v1_7ch.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_8CH_DMAMUX)
      list(APPEND SRCS stm32_dma_m3m4_v1_8ch_dmamux.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V1_8CH)
      list(APPEND SRCS stm32_dma_m3m4_v1_8ch.c)
    elseif(CONFIG_STM32_HAVE_IP_DMA_V2_STREAM)
      list(APPEND SRCS stm32_dma_m3m4_v2_stream.c)
    endif()
  endif()

  if(CONFIG_STM32_PROGMEM)
    if(CONFIG_STM32_HAVE_IP_FLASH_M0_G0C0)
      list(APPEND SRCS stm32_flash_m0_g0c0.c)
    elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_L1)
      list(APPEND SRCS stm32_flash_m3m4_l1.c)
    elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_F1F3)
      list(APPEND SRCS stm32_flash_m3m4_f1f3.c)
    elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_F2F4)
      list(APPEND SRCS stm32_flash_m3m4_f2f4.c)
    elseif(CONFIG_STM32_HAVE_IP_FLASH_M3M4_G4)
      list(APPEND SRCS stm32_flash_m3m4_g4.c)
    endif()
  endif()

  if(CONFIG_STM32_HAVE_HSI48)
    list(APPEND SRCS stm32_hsi48_m0_v1.c)
  endif()

  if(CONFIG_STM32_USB AND CONFIG_STM32_HAVE_IP_USBDEV_M0_V1)
    list(APPEND SRCS stm32_usbdev_m0_v1.c)
  endif()

  if(CONFIG_STM32_I2C)
    list(APPEND SRCS stm32_i2c_m0_v1.c)
  endif()

  if(CONFIG_STM32_SPI)
    list(APPEND SRCS stm32_spi_m0_v1.c)
  endif()

  if(CONFIG_STM32_PWM)
    list(APPEND SRCS stm32_pwm_m0_v1.c)
  endif()

  if(CONFIG_PULSECOUNT AND CONFIG_STM32_TIM1_PULSECOUNT)
    list(APPEND SRCS stm32_pulsecount_m0_v1.c)
  endif()

  if(CONFIG_STM32_ADC AND CONFIG_STM32_HAVE_IP_ADC_M0_V1)
    list(APPEND SRCS stm32_adc_m0_v1.c)
  endif()

  if(CONFIG_STM32_AES AND CONFIG_STM32_HAVE_IP_AES_M0_V1)
    list(APPEND SRCS stm32_aes_m0_v1.c)
  endif()

  if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE
     AND CONFIG_STM32_AES
     AND CONFIG_STM32_HAVE_IP_AES_M0_V1)
    list(APPEND SRCS stm32_crypto_m0_v1.c)
  endif()

  if(CONFIG_STM32_RNG AND CONFIG_STM32_HAVE_IP_RNG_M0_V1)
    list(APPEND SRCS stm32_rng_m0_v1.c)
  endif()

  if(CONFIG_STM32_TIM AND CONFIG_STM32_HAVE_IP_TIMERS_M0_V1)
    list(APPEND SRCS stm32_tim_m0_v1.c stm32_tim_m0_v1_lowerhalf.c)
  endif()

  if(CONFIG_STM32_IWDG AND CONFIG_STM32_HAVE_IP_WDG_M0_V1)
    list(APPEND SRCS stm32_iwdg_m0_v1.c)
  endif()

  if(CONFIG_STM32_WWDG AND CONFIG_STM32_HAVE_IP_WDG_M0_V1)
    list(APPEND SRCS stm32_wwdg_m0_v1.c)
  endif()

  if(CONFIG_STM32_FDCAN AND CONFIG_STM32_HAVE_IP_FDCAN_MCAN_M0_V1)
    if(CONFIG_STM32_FDCAN_CHARDRIVER)
      list(APPEND SRCS stm32_fdcan_m0_v1.c)
    endif()
    if(CONFIG_STM32_FDCAN_SOCKET)
      list(APPEND SRCS stm32_fdcan_m0_v1_sock.c)
    endif()
  endif()

  if(CONFIG_SENSORS_QENCODER)
    list(APPEND SRCS stm32_qencoder_m0_v1.c)
  endif()
endif()

if(CONFIG_SENSORS_HALL3PHASE)
  list(APPEND SRCS stm32_hall3ph.c)
endif()

target_sources(arch PRIVATE ${SRCS})
