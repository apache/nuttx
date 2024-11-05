# ##############################################################################
# drivers/mtd/CMakeLists.txt
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

if(CONFIG_MTD)
  set(SRCS ftl.c)

  if(CONFIG_MTD_CONFIG_FAIL_SAFE)
    list(APPEND SRCS mtd_config_fs.c)
  elseif(CONFIG_MTD_CONFIG)
    list(APPEND SRCS mtd_config.c)
  endif()

  if(CONFIG_MTD_PARTITION)
    list(APPEND SRCS mtd_partition.c)
  endif()

  if(CONFIG_MTD_SECT512)
    list(APPEND SRCS sector512.c)
  endif()

  if(CONFIG_MTD_WRBUFFER)
    list(APPEND SRCS mtd_rwbuffer.c)
  elseif(CONFIG_MTD_READAHEAD)
    list(APPEND SRCS mtd_rwbuffer.c)
  endif()

  if(CONFIG_MTD_PROGMEM)
    list(APPEND SRCS mtd_progmem.c)
  endif()

  if(CONFIG_MTD_NAND)
    list(
      APPEND
      SRCS
      mtd_nand.c
      mtd_onfi.c
      mtd_nandscheme.c
      mtd_nandmodel.c
      mtd_modeltab.c)
    if(CONFIG_MTD_NAND_SWECC)
      list(APPEND SRCS mtd_nandecc.c hamming.c)
    endif()
  endif()

  if(CONFIG_RAMMTD)
    list(APPEND SRCS rammtd.c)
  endif()

  if(CONFIG_FILEMTD)
    list(APPEND SRCS filemtd.c)
  endif()

  if(CONFIG_NULLMTD)
    list(APPEND SRCS nullmtd.c)
  endif()

  if(CONFIG_RPMSGMTD)
    list(APPEND SRCS rpmsgmtd.c)
  endif()

  if(CONFIG_RPMSGMTD_SERVER)
    list(APPEND SRCS rpmsgmtd_server.c)
  endif()

  if(CONFIG_MTD_AT24XX)
    list(APPEND SRCS at24xx.c)
  endif()

  if(CONFIG_MTD_AT25EE)
    list(APPEND SRCS at25ee.c)
  endif()

  if(CONFIG_MTD_AT45DB)
    list(APPEND SRCS at45db.c)
  endif()

  if(CONFIG_MTD_RAMTRON)
    list(APPEND SRCS ramtron.c)
  endif()

  if(CONFIG_MTD_SST25)
    list(APPEND SRCS sst25.c)
  endif()

  if(CONFIG_MTD_SST25XX)
    list(APPEND SRCS sst25xx.c)
  endif()

  if(CONFIG_MTD_SST26)
    list(APPEND SRCS sst26.c)
  endif()

  if(CONFIG_MTD_SST39FV)
    list(APPEND SRCS sst39vf.c)
  endif()

  if(CONFIG_MTD_W25)
    list(APPEND SRCS w25.c)
  endif()

  if(CONFIG_MTD_GD25)
    list(APPEND SRCS gd25.c)
  endif()

  if(CONFIG_MTD_GD5F)
    list(APPEND SRCS gd5f.c)
  endif()

  if(CONFIG_MTD_AT25)
    list(APPEND SRCS at25.c)
  endif()

  if(CONFIG_MTD_M25P)
    list(APPEND SRCS m25px.c)
  endif()

  if(CONFIG_MTD_MX25L)
    list(APPEND SRCS mx25lx.c)
  endif()

  if(CONFIG_MTD_MX35)
    list(APPEND SRCS mx35.c)
  endif()

  if(CONFIG_MTD_S25FL1)
    list(APPEND SRCS s25fl1.c)
  endif()

  if(CONFIG_MTD_N25QXXX)
    list(APPEND SRCS n25qxxx.c)
  endif()

  if(CONFIG_MTD_W25QXXXJV)
    list(APPEND SRCS w25qxxxjv.c)
  endif()

  if(CONFIG_MTD_MX25RXX)
    list(APPEND SRCS mx25rxx.c)
  endif()

  if(CONFIG_MTD_IS25XP)
    list(APPEND SRCS is25xp.c)
  endif()

  if(CONFIG_MTD_SMART AND CONFIG_FS_SMARTFS)
    list(APPEND SRCS smart.c)
  endif()

  if(CONFIG_MTD_DHARA)
    if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/dhara)
      FetchContent_Declare(
        dhara
        URL https://github.com/dlbeer/dhara/archive/refs/heads/master.zip
            SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/dhara BINARY_DIR
            ${CMAKE_BINARY_DIR}/drivers/mtd/dhara
        DOWNLOAD_NO_PROGRESS true)
      FetchContent_Populate(dhara)
    endif()

    list(APPEND SRCS dhara.c mtd/dhara/dhara/map.c mtd/dhara/dhara/error.c
         mtd/dhara/dhara/journal.c)
    target_include_directories(drivers PRIVATE ${CMAKE_CURRENT_LIST_DIR}/dhara)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
