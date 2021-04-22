############################################################################
# graphics/nxglib/Make.defs
#
#   Copyright (C) 2008, 2010-2011, 2013, 2016 Gregory Nutt. All rights reserved.
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

set(OPERATIONS setpixel fillrectangle getrectangle
  filltrapezoid moverectangle copyrectangle)
set(BPPS 1 2 8 4 16 24 32)

if (CONFIG_NX_LCDDRIVER)
  set(BLITDIR lcd)
else()
  set(BLITDIR fb)
endif()

set(BLIT_SRCS)
foreach(op OPERATIONS)
  foreach(bpp BPPS)
    add_custom_command(
      OUTPUT nxglib_${op}_${bpp}bpp.c
      COMMAND ${CMAKE_C_COMPILER}
      ARGS -E ${CMAKE_C_FLAGS} -c ${BLITDIR}/nxglib_${op}.c -o nxglib_${op}_${bpp}bpp.c
      DEPENDS ${BLITDIR}/nxglib_${op}.c
    )

    list(APPEND BLIT_SRCS nxglib_${op}_${bpp}bpp.c)
  endforeach()
endforeach()

set(SRCS ${SRCS} ${BLIT_SRCS})
add_custom_target(nxglib_blit ${BLIT_SRCS})
add_dependencies(graphics nxglib_blit)

if (CONFIG_NX_RAMBACKED)
  set(PWFB_SRCS)

  foreach(op OPERATIONS)
    foreach(bpp BPPS)
      add_custom_command(
        OUTPUT pwfb_${op}_${bpp}bpp.c
        COMMAND ${CMAKE_C_COMPILER}
        ARGS -E ${CMAKE_C_FLAGS} -c pwfb/pwfb_${op}.c -o pwfb_${op}_${bpp}bpp.c
        DEPENDS pwfb/pwfb_${op}.c
      )

      list(APPEND PWFB_SRCS pwfb_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()

  add_custom_target(nxglib_pwfb DEPEND ${PWFB_SRCS})
  add_dependencies(graphics nxglib_pwfb)
  set(SRCS ${SRCS} ${PWFB_SRCS})
endif()

if (CONFIG_NX_SWCURSOR)
  set(CURSOR_SRCS)
  set(CURSOR_OPS draw erase backup)
  set(CURSOR_BPPS 8 16 24 32)

  foreach(op CURSOR_OPS)
    foreach(bpp CURSOR_BPPS)
      add_custom_command(
        OUTPUT nxglib_cursor_${op}_${bpp}bpp.c
        COMMAND ${CMAKE_C_COMPILER}
        ARGS -E ${CMAKE_C_FLAGS} -c cursor/nxglib_cursor_${op}.c -o nxglib_cursor_${op}_${bpp}bpp.c
        DEPENDS cursor/nxglib_cursor_${op}.c
      )

      list(APPEND CURSOR_SRCS ${CMAKE_CURRENT_BINARY_DIR}/nxglib_cursor_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()

  add_custom_target(nxglib_cursor DEPEND ${CURSOR_SRCS})
  add_dependencies(graphics nxglib_cursor)
  set(SRCS ${SRCS} ${CURSOR_SRCS})
endif()

target_sources(graphics PRIVATE ${SRCS})
