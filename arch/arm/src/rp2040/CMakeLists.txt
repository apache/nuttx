# ##############################################################################
# nuttx/arch/arm/src/rp2040/hardware/CMakeLists.txt
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

add_subdirectory(hardware)

set(SRCS
  rp2040_adc.c
  rp2040_clock.c
  rp2040_cpuidlestack.c
  rp2040_cpuindex.c
  rp2040_cpustart.c
  rp2040_cyw43439.c
  rp2040_dmac.c
  rp2040_flash_initialize.S
  rp2040_flash_mtd.c
  rp2040_gpio.c
  rp2040_i2c.c
  rp2040_i2c_slave.c
  rp2040_i2s.c
  rp2040_i2s_pio.c
  rp2040_idle.c
  rp2040_irq.c
  rp2040_pio.c
  rp2040_pll.c
  rp2040_pwm.c
  rp2040_serial.c
  rp2040_smpcall.c
  rp2040_spi.c
  rp2040_start.c
  rp2040_testset.c
  rp2040_timerisr.c
  rp2040_uart.c
  rp2040_usbdev.c
  rp2040_wdt.c
  rp2040_ws2812.c
  rp2040_xosc.c
)

target_sources(arch PRIVATE ${SRCS})