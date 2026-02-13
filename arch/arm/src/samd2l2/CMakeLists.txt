# ##############################################################################
# arch/arm/src/samd2l2/CMakeLists.txt
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

list(
  APPEND
  SRCS
  sam_irq.c
  sam_lowputc.c
  sam_port.c
  sam_sercom.c
  sam_serial.c
  sam_start.c
  sam_usart.c)

# Configuration-dependent SAM D/L files

if(CONFIG_ARCH_FAMILY_SAMD20)
  list(APPEND SRCS samd_clockconfig.c samd_gclk.c)
elseif(CONFIG_ARCH_FAMILY_SAMD21)
  list(APPEND SRCS samd_clockconfig.c samd_gclk.c)
elseif(CONFIG_ARCH_FAMILY_SAML21)
  list(APPEND SRCS saml_clockconfig.c saml_gclk.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS sam_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sam_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS sam_userspace.c)
endif()

if(CONFIG_ARCH_IRQPRIO)
  list(APPEND SRCS sam_irqprio.c)
endif()

if(CONFIG_SAMD2L2_ADC)
  list(APPEND SRCS sam_adc.c)
endif()

if(CONFIG_SAMD2L2_DMAC)
  list(APPEND SRCS sam_dmac.c)
endif()

if(CONFIG_SAMD2L2_HAVE_SPI)
  list(APPEND SRCS sam_spi.c)
endif()

if(CONFIG_SAMD2L2_HAVE_I2C)
  list(APPEND SRCS sam_i2c_master.c)
endif()

if(CONFIG_SAMD2L2_USB)
  list(APPEND SRCS sam_usb.c)
endif()

if(CONFIG_SAMD2L2_EIC)
  list(APPEND SRCS sam_eic.c)
endif()

if(CONFIG_SAMD2L2_AC)
  list(APPEND SRCS sam_ac.c)
endif()

target_sources(arch PRIVATE ${SRCS})
