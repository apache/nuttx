# ##############################################################################
# arch/arm/src/ht32f491x3/CMakeLists.txt
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
  ht32f491x3_start.c
  ht32f491x3_irq.c
  ht32f491x3_timerisr.c
  ht32f491x3_gpio.c
  ht32f491x3_lowputc.c
  ht32f491x3_serial.c)

if(CONFIG_PWM)
  list(APPEND SRCS ht32f491x3_pwm.c)
endif()

if(CONFIG_ARCH_HAVE_CUSTOM_VECTORS)
  list(APPEND SRCS arm_vectors.c)
endif()

target_sources(arch PRIVATE ${SRCS})
