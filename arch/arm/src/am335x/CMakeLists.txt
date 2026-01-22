# ##############################################################################
# arch/arm/src/am335x/CMakeLists.txt
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

# AM335x-specific C source files

set(SRCS
    am335x_boot.c
    am335x_clockconfig.c
    am335x_pinmux.c
    am335x_irq.c
    am335x_gpio.c
    am335x_lowputc.c
    am335x_serial.c
    am335x_wdog.c
    am335x_sysclk.c
    am335x_i2c.c
    am335x_can.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS am335x_timerisr.c)
endif()

if(CONFIG_AM335X_GPIO_IRQ)
  list(APPEND SRCS am335x_gpioirq.c)
endif()

if(CONFIG_AM335X_LCDC)
  list(APPEND SRCS am335x_lcdc.c am335x_edid.c)
endif()

target_sources(arch PRIVATE ${SRCS})
