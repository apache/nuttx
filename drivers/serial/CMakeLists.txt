############################################################################
# drivers/serial/CMakeLists.txt
#
#   Copyright (C) 2009, 2011, 2017 Gregory Nutt. All rights reserved.
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

set(SRCS serial.c serial_io.c)

if (CONFIG_SERIAL_RXDMA)
  list(APPEND SRCS serial_dma.c)
elseif (CONFIG_SERIAL_TXDMA)
  list(APPEND SRCS serial_dma.c)
endif()

if (CONFIG_16550_UART)
  list(APPEND SRCS uart_16550.c)
endif()

if (CONFIG_RPMSG_UART)
  list(APPEND SRCS uart_rpmsg.c)
endif()

# termios support

if (CONFIG_SERIAL_TERMIOS)
  list(APPEND SRCS tcdrain.c)
endif()

# Pseudo-terminal support

if (CONFIG_PSEUDOTERM)
  list(APPEND SRCS pty.c)

  if (CONFIG_PSEUDOTERM_SUSV1)
    list(APPEND SRCS ptmx.c)
  endif()
endif()

target_sources(drivers PRIVATE ${SRCS})
