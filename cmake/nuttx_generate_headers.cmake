# ##############################################################################
# cmake/nuttx_generate_headers.cmake
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

# setup target to generate config.h and version.h from mkconfig and mkversion

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include/nuttx)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/nuttx)
endif()

include(nuttx_mkconfig)
include(nuttx_mkversion)

# Setup symbolic link generation

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include_arch/arch)
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
                          ${CMAKE_BINARY_DIR}/include_arch/arch)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include_apps)
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
                          ${CMAKE_BINARY_DIR}/include_apps)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include/arch)
  nuttx_create_symlink(${NUTTX_DIR}/arch/${CONFIG_ARCH}/include
                       ${CMAKE_BINARY_DIR}/include/arch)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include_arch/arch/board)
  if(EXISTS ${NUTTX_BOARD_DIR}/include)
    nuttx_create_symlink(${NUTTX_BOARD_DIR}/include
                         ${CMAKE_BINARY_DIR}/include_arch/arch/board)
  elseif(EXISTS ${NUTTX_BOARD_DIR}/../common/include)
    nuttx_create_symlink(${NUTTX_BOARD_DIR}/../common/include
                         ${CMAKE_BINARY_DIR}/include_arch/arch/board)
  endif()
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/include_arch/arch/chip)
  if(CONFIG_ARCH_CHIP_CUSTOM)
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CHIP_ABS_DIR}/include
              ${CMAKE_BINARY_DIR}/include_arch/arch/chip)
  else()
    execute_process(
      COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}
        ${CMAKE_BINARY_DIR}/include_arch/arch/chip)
  endif()
endif()

# Optional symbolic links

# Target used to copy include/nuttx/lib/stdarg.h.  If CONFIG_ARCH_STDARG_H is
# defined, then there is an architecture specific stdarg.h header file that will
# be included indirectly from include/lib/stdarg.h.  But first, we have to copy
# stdarg.h from include/nuttx/. to include/.

if(CONFIG_ARCH_STDARG_H)
  nuttx_create_symlink(${NUTTX_DIR}/include/nuttx/lib/stdarg.h
                       ${CMAKE_BINARY_DIR}/include/stdarg.h)
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/stdarg.h)
endif()

# Target used to copy include/nuttx/lib/math.h.  If CONFIG_ARCH_MATH_H is
# defined, then there is an architecture specific math.h header file that will
# be included indirectly from include/math.h.  But first, we have to copy math.h
# from include/nuttx/. to include/.  Logic within include/nuttx/lib/math.h will
# hand the redirection to the architecture- specific math.h header file.
#
# If the CONFIG_LIBM is defined, the Rhombus libm will be built at libc/math.
# Definitions and prototypes for the Rhombus libm are also contained in
# include/nuttx/lib/math.h and so the file must also be copied in that case.
#
# If neither CONFIG_ARCH_MATH_H nor CONFIG_LIBM is defined, then no math.h
# header file will be provided.  You would want that behavior if (1) you don't
# use libm, or (2) you want to use the math.h and libm provided within your
# toolchain.

if(CONFIG_ARCH_MATH_H OR CONFIG_LIBM)
  set(NEED_MATH_H true)
endif()

if(NEED_MATH_H)
  nuttx_create_symlink(${NUTTX_DIR}/include/nuttx/lib/math.h
                       ${CMAKE_BINARY_DIR}/include/math.h)
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/math.h)
endif()

# The float.h header file defines the properties of your floating point
# implementation.  It would always be best to use your toolchain's float.h
# header file but if none is available, a default float.h header file will
# provided if this option is selected.  However there is no assurance that the
# settings in this float.h are actually correct for your platform!

if(CONFIG_ARCH_FLOAT_H)
  nuttx_create_symlink(${NUTTX_DIR}/include/nuttx/lib/float.h
                       ${CMAKE_BINARY_DIR}/include/float.h)
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/float.h)
endif()

# Target used to copy include/nuttx/lib/setjmp.h.  If CONFIG_ARCH_SETJMP_H is
# defined, then there is an architecture specific setjmp.h header file that will
# be included indirectly from include/lib/setjmp.h.  But first, we have to copy
# setjmp.h from include/nuttx/. to include/.

if(CONFIG_ARCH_SETJMP_H)
  nuttx_create_symlink(${NUTTX_DIR}/include/nuttx/lib/setjmp.h
                       ${CMAKE_BINARY_DIR}/include/setjmp.h)
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/setjmp.h)
endif()

# Add final context target that ties together all of the above The context
# target is invoked on each target build to assure that NuttX is properly
# configured.  The basic configuration steps include creation of the the
# config.h and version.h header files in the include/nuttx directory and the
# establishment of symbolic links to configured directories.

add_custom_target(
  nuttx_context
  DEPENDS
    ${CMAKE_BINARY_DIR}/include/nuttx/config.h
    ${CMAKE_BINARY_DIR}/include/nuttx/version.h
    $<$<BOOL:${CONFIG_ARCH_STDARG_H}>:${CMAKE_BINARY_DIR}/include/stdarg.h>
    $<$<BOOL:${NEED_MATH_H}>:${CMAKE_BINARY_DIR}/include/math.h>
    $<$<BOOL:${CONFIG_ARCH_FLOAT_H}>:${CMAKE_BINARY_DIR}/include/float.h>
    $<$<BOOL:${CONFIG_ARCH_SETJMP_H}>:${CMAKE_BINARY_DIR}/include/setjmp.h>)
