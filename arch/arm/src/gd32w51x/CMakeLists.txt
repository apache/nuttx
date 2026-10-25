# ##############################################################################
# arch/arm/src/gd32w51x/CMakeLists.txt
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

set(SRCS
    gd32w51x_start.c
    gd32w51x_clockconfig.c
    gd32w51x_irq.c
    gd32w51x_timerisr.c
    gd32w51x_lowputc.c
    gd32w51x_serial.c
    gd32w51x_tzinit.c
    gd32w51x_idle.c
    gd32w51x_gpio.c)

if(CONFIG_PM)
  list(APPEND SRCS gd32w51x_pm.c)
endif()

if(CONFIG_RTC)
  list(APPEND SRCS gd32w51x_rtc.c)
endif()

if(CONFIG_GD32W51X_DMA)
  list(APPEND SRCS gd32w51x_dma.c)
endif()

if(CONFIG_GD32W51X_GPIO_IRQ)
  list(APPEND SRCS gd32w51x_exti.c)
endif()

if(CONFIG_GD32W51X_SPI)
  list(APPEND SRCS gd32w51x_spi.c)
endif()

if(CONFIG_GD32W51X_QSPI)
  list(APPEND SRCS gd32w51x_qspi.c)
endif()

if(CONFIG_GD32W51X_TSI)
  list(APPEND SRCS gd32w51x_tsi.c)
endif()

if(CONFIG_GD32W51X_I2C)
  list(APPEND SRCS gd32w51x_i2c.c)
endif()

if(CONFIG_GD32W51X_ADC)
  list(APPEND SRCS gd32w51x_adc.c)
endif()

if(CONFIG_GD32W51X_PWM)
  list(APPEND SRCS gd32w51x_pwm.c)
endif()

if(CONFIG_GD32W51X_CAPTURE)
  list(APPEND SRCS gd32w51x_capture.c)
endif()

if(CONFIG_GD32W51X_FWDGT)
  list(APPEND SRCS gd32w51x_fwdgt.c)
endif()

if(CONFIG_GD32W51X_WWDGT)
  list(APPEND SRCS gd32w51x_wwdgt.c)
endif()

if(CONFIG_GD32W51X_PROGMEM)
  list(APPEND SRCS gd32w51x_progmem.c)
endif()

if(CONFIG_GD32W51X_RNG)
  list(APPEND SRCS gd32w51x_rng.c)
endif()

if(CONFIG_GD32W51X_CRC)
  list(APPEND SRCS gd32w51x_crc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
