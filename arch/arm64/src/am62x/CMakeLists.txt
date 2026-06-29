# ##############################################################################
# arch/arm64/src/am62x/CMakeLists.txt
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

# AM62x SoC sources that are always compiled

list(APPEND SRCS am62x_boot.c am62x_16550serial.c)

if(CONFIG_AM62X_GPIO)
  list(APPEND SRCS am62x_gpio.c)
endif()

if(CONFIG_AM62X_I2C0
   OR CONFIG_AM62X_I2C1
   OR CONFIG_AM62X_I2C2
   OR CONFIG_AM62X_I2C3)
  list(APPEND SRCS am62x_i2c.c)
endif()

if(CONFIG_AM62X_TISCI)
  list(APPEND SRCS am62x_tisci.c)
endif()

# Early UART assembly (needed for CONFIG_ARCH_EARLY_PRINT / semihosting console
# before the full serial driver is up)

if(CONFIG_ARCH_EARLY_PRINT)
  list(APPEND SRCS am62x_lowputc.S)
endif()

target_sources(arch PRIVATE ${SRCS})
