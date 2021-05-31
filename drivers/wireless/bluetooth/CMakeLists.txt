############################################################################
# drivers/bluetooth/CMakeLists.txt
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

if (CONFIG_DRIVERS_BLUETOOTH)
  set(SRCS)

  if (CONFIG_BLUETOOTH_UART)
    list(APPEND SRCS bt_uart.c)

    if (CONFIG_BLUETOOTH_UART_GENERIC)
      list(APPEND SRCS bt_uart_generic.c)
    endif()
    if (CONFIG_BLUETOOTH_UART_SHIM)
      list(APPEND SRCS bt_uart_shim.c)
    endif()
    if (CONFIG_BLUETOOTH_UART_CC2564)
      list(APPEND SRCS bt_uart_cc2564.c)
    endif()
    if (CONFIG_BLUETOOTH_BCM4343X)
      list(APPEND SRCS bt_uart_bcm4343x.c)
    endif()
  endif()

  if (CONFIG_BLUETOOTH_NULL)
    list(APPEND SRCS bt_null.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()

