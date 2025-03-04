# ##############################################################################
# drivers/timers/CMakeLists.txt
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

set(SRCS)

if(CONFIG_WATCHDOG)
  list(APPEND SRCS watchdog.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS timer.c)
endif()

if(CONFIG_TIMER_ARCH)
  list(APPEND SRCS arch_timer.c)
endif()

if(CONFIG_ONESHOT)
  list(APPEND SRCS oneshot.c)
endif()

if(CONFIG_ALARM_ARCH)
  list(APPEND SRCS arch_alarm.c)
endif()

if(CONFIG_RTC_DSXXXX)
  list(APPEND SRCS ds3231.c)
endif()

if(CONFIG_RTC_PCF85263)
  list(APPEND SRCS pcf85263.c)
endif()

if(CONFIG_RTC_PL031)
  list(APPEND SRCS pl031.c)
endif()

if(CONFIG_RTC_MCP794XX)
  list(APPEND SRCS mcp794xx.c)
endif()

if(CONFIG_RTC_RX8010SJ)
  list(APPEND SRCS rx8010.c)
endif()

if(CONFIG_RTC_RPMSG)
  list(APPEND SRCS rpmsg_rtc.c)
endif()

if(CONFIG_RTC_ARCH)
  list(APPEND SRCS arch_rtc.c)
endif()

if(CONFIG_RTC_DRIVER)
  list(APPEND SRCS rtc.c)
endif()

if(CONFIG_TIMERS_CS2100CP)
  list(APPEND SRCS cs2100-cp.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS pwm.c)
endif()

if(CONFIG_CAPTURE)
  list(APPEND SRCS capture.c)
endif()

if(CONFIG_GOLDFISH_TIMER)
  list(APPEND SRCS goldfish_timer.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
