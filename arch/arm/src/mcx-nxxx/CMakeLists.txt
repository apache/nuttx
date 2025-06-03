# ##############################################################################
# arch/arm/src/mcx-nxxx/CMakeLists.txt
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
    nxxx_start.c
    nxxx_clockconfig.c
    nxxx_gpio.c
    nxxx_port.c
    nxxx_irq.c
    nxxx_timerisr.c
    nxxx_idle.c)

if(CONFIG_NXXX_LPUART)
  list(APPEND SRCS nxxx_lpuart.c nxxx_lowputc.c)
endif()

if(CONFIG_NXXX_GPIO_IRQ)
  list(APPEND SRCS nxxx_gpioirq.c)
endif()

if(CONFIG_NXXX_LPI2C)
  list(APPEND SRCS nxxx_lpi2c.c)
endif()

target_sources(arch PRIVATE ${SRCS})
