############################################################################
# drivers/CMakeLists.txt
#
#   Copyright (C) 2007-2014, 2016 Gregory Nutt. All rights reserved.
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

nuttx_add_kernel_library(drivers)

add_subdirectory(1wire)
add_subdirectory(analog)
add_subdirectory(audio)
add_subdirectory(bch)
add_subdirectory(can)
add_subdirectory(contactless)
add_subdirectory(crypto)
add_subdirectory(eeprom)
add_subdirectory(efuse)
add_subdirectory(i2c)
add_subdirectory(i2s)
add_subdirectory(input)
add_subdirectory(ioexpander)
add_subdirectory(lcd)
add_subdirectory(leds)
add_subdirectory(loop)
add_subdirectory(mmcsd)
add_subdirectory(modem)
add_subdirectory(motor)
add_subdirectory(mtd)
add_subdirectory(net)
add_subdirectory(pipes)
add_subdirectory(power)
add_subdirectory(rc)
add_subdirectory(rf)
add_subdirectory(rptun)
add_subdirectory(sensors)
add_subdirectory(serial)
add_subdirectory(spi)
add_subdirectory(syslog)
add_subdirectory(timers)
add_subdirectory(usbdev)
add_subdirectory(usbhost)
add_subdirectory(usbmisc)
add_subdirectory(usbmonitor)
add_subdirectory(video)
add_subdirectory(wireless)

if (CONFIG_SPECIFIC_DRIVERS)
  add_subdirectory(platform)
endif()

set(SRCS misc/dev_null.c misc/dev_zero.c)
  list(APPEND SRCS drivers_initialize.c)

if (CONFIG_DEV_SIMPLE_ADDRENV)
  list(APPEND SRCS addrenv.c)
endif()

if (CONFIG_LWL_CONSOLE)
  list(APPEND SRCS lwl_console.c)
endif()

if (NOT CONFIG_DISABLE_MOUNTPOINT)
  list(APPEND SRCS misc/ramdisk.c)
  if (CONFIG_DRVR_MKRD)
    list(APPEND SRCS mkrd.c)
  endif()
  if (CONFIG_DRVR_WRITEBUFFER)
    list(APPEND SRCS rwbuffer.c)
  else()
    if (CONFIG_DRVR_READAHEAD)
      list(APPEND SRCS rwbuffer.c)
    endif()
  endif()
endif()

target_sources(drivers PRIVATE ${SRCS})
