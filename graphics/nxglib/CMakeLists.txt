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

nuttx_add_aux_library(nxglib)
set(SRCS)

set(OPERATIONS setpixel fillrectangle getrectangle
  filltrapezoid moverectangle copyrectangle)
set(BPPS 1 2 8 4 16 24 32)

if (CONFIG_NX_LCDDRIVER)
  set(BLITDIR lcd)
else()
  set(BLITDIR fb)
endif()

foreach(op ${OPERATIONS})
  foreach(bpp ${BPPS})
    configure_file(${BLITDIR}/nxglib_${op}.c.in nxglib_${op}_${bpp}bpp.c)
    set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/nxglib_${op}_${bpp}bpp.c PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
    list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/nxglib_${op}_${bpp}bpp.c)
  endforeach()
endforeach()

if (CONFIG_NX_RAMBACKED)
  foreach(op ${OPERATIONS})
    foreach(bpp ${BPPS})
      configure_file(pwfb/pwfb_${op}.c.in pwfb_${op}_${bpp}bpp.c)
      set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/pwfb_${op}_${bpp}bpp.c PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
      list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/pwfb_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()
endif()

if (CONFIG_NX_SWCURSOR)
  set(CURSOR_OPS draw erase backup)
  set(CURSOR_BPPS 8 16 24 32)

  foreach(op ${CURSOR_OPS})
    foreach(bpp ${CURSOR_BPPS})
      configure_file(cursor/nxglib_cursor_${op}.c.in nxglib_cursor_${op}_${bpp}bpp.c)
      set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/nxglib_cursor_${op}_${bpp}bpp.c PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
      list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/nxglib_cursor_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()
endif()

target_sources(nxglib PRIVATE ${SRCS})
target_include_directories(nxglib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(graphics PRIVATE nxglib)
