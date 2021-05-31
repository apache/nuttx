############################################################################
# drivers/timers/CMakeLists.txt
# These drivers support various timer devices
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(SRCS)

if (CONFIG_WATCHDOG)
  list(APPEND SRCS watchdog.c)
endif()

if (CONFIG_TIMER)
  list(APPEND SRCS timer.c)
endif()

if (CONFIG_TIMER_ARCH)
  list(APPEND SRCS arch_timer.c)
endif()

if (CONFIG_ONESHOT)
  list(APPEND SRCS oneshot.c)
endif()

if (CONFIG_ALARM_ARCH)
  list(APPEND SRCS arch_alarm.c)
endif()

if (CONFIG_RTC_DSXXXX)
  list(APPEND SRCS ds3231.c)
endif()

if (CONFIG_RTC_PCF85263)
  list(APPEND SRCS pcf85263.c)
endif()

if (CONFIG_RTC_MCP794XX)
  list(APPEND SRCS mcp794xx.c)
endif()

if (CONFIG_RTC_RPMSG)
  list(APPEND SRCS rpmsg_rtc.c)
endif()

if (CONFIG_RTC_ARCH)
  list(APPEND SRCS arch_rtc.c)
endif()

if (CONFIG_RTC_DRIVER)
  list(APPEND SRCS rtc.c)
endif()

if (CONFIG_TIMERS_CS2100CP)
  list(APPEND SRCS cs2100-cp.c)
endif()

if (CONFIG_PWM)
  list(APPEND SRCS pwm.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
