# ##############################################################################
# cmake/menuconfig.cmake
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

# menuconfig target this triggers a reconfiguration (TODO: do only if config
# changes)

set(KCONFIG_ENV
    "KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config" "EXTERNALDIR=dummy"
    "APPSDIR=${NUTTX_APPS_DIR}" "DRIVERS_PLATFORM_DIR=dummy"
    "APPSBINDIR=${NUTTX_APPS_BINDIR}" "BINDIR=${CMAKE_BINARY_DIR}")

# Use qconfig instead of menuconfig since PowerShell not support curses
# redirection

if(WIN32)
  set(MENUCONFIG guiconfig)
else()
  set(MENUCONFIG menuconfig)
endif()

add_custom_target(
  menuconfig
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} ${MENUCONFIG}
  COMMAND ${CMAKE_COMMAND} -E remove -f
          ${CMAKE_BINARY_DIR}/include/nuttx/config.h # invalidate existing
                                                     # config
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
  WORKING_DIRECTORY ${NUTTX_DIR}
  USES_TERMINAL)

# qconfig target

add_custom_target(
  qconfig
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} guiconfig
  COMMAND ${CMAKE_COMMAND} -E remove -f
          ${CMAKE_BINARY_DIR}/include/nuttx/config.h # invalidate existing
                                                     # config
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
  WORKING_DIRECTORY ${NUTTX_DIR}
  USES_TERMINAL)

# utility target to restore .config from board's defconfig
add_custom_target(
  resetconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DEFCONFIG}
          ${CMAKE_BINARY_DIR}/.config
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} olddefconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/.config
          ${CMAKE_BINARY_DIR}/.config.orig
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
  WORKING_DIRECTORY ${NUTTX_DIR})

# utility target to refresh .config from board's defconfig for GITHUB
add_custom_target(
  refreshsilent
  COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/SAVEconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/.config
          ${CMAKE_BINARY_DIR}/SAVEconfig
  COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/.config
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DEFCONFIG}
          ${CMAKE_BINARY_DIR}/.config
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} olddefconfig
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} savedefconfig --out
          ${CMAKE_BINARY_DIR}/defconfig.tmp
  COMMAND ${CMAKE_COMMAND} -P ${NUTTX_DIR}/cmake/savedefconfig.cmake
          ${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/defconfig.tmp
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/defconfig
          ${NUTTX_DEFCONFIG}
  WORKING_DIRECTORY ${NUTTX_DIR})

# utility target to replace defconfig to board's defconfig
add_custom_target(
  savedefconfig
  COMMAND ${CMAKE_COMMAND} -E env ${KCONFIG_ENV} savedefconfig --out
          ${CMAKE_BINARY_DIR}/defconfig.tmp
  COMMAND ${CMAKE_COMMAND} -P ${NUTTX_DIR}/cmake/savedefconfig.cmake
          ${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/defconfig.tmp
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/defconfig
          ${NUTTX_DEFCONFIG}
  WORKING_DIRECTORY ${NUTTX_DIR})
