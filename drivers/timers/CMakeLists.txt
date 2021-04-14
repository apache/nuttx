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

# Include timer drivers

set(CSRCS)

if("${CONFIG_WATCHDOG}" STREQUAL "y")
	list(APPEND CSRCS watchdog.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CSRCS timer.c)
endif()

if("${CONFIG_TIMER_ARCH}" STREQUAL "y")
	list(APPEND CSRCS arch_timer.c)
endif()

if("${CONFIG_ONESHOT}" STREQUAL "y")
	list(APPEND CSRCS oneshot.c)
endif()

if("${CONFIG_ALARM_ARCH}" STREQUAL "y")
	list(APPEND CSRCS arch_alarm.c)
endif()

if("${CONFIG_RTC_DSXXXX}" STREQUAL "y")
	list(APPEND CSRCS ds3231.c)
endif()

if("${CONFIG_RTC_PCF85263}" STREQUAL "y")
	list(APPEND CSRCS pcf85263.c)
endif()

if("${CONFIG_RTC_ARCH}" STREQUAL "y")
	list(APPEND CSRCS arch_rtc.c)
endif()

if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
	list(APPEND CSRCS rtc.c)
endif()

if("${CONFIG_TIMERS_CS2100CP}" STREQUAL "y")
	list(APPEND CSRCS cs2100-cp.c)
endif()

# Include timer build support (if any were selected)

set_parent_srcs(${CSRCS})
