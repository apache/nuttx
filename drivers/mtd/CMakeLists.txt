############################################################################
# drivers/mtd/CMakeLists.txt
# These driver supports various Memory Technology Devices (MTD) using the
# NuttX MTD interface.
#
#   Copyright (C) 2009-2013, 2015-2016 Gregory Nutt. All rights reserved.
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

if (CONFIG_MTD)
  set(SRCS ftl.c mtd_config.c)

  if (CONFIG_MTD_PARTITION)
    list(APPEND SRCS mtd_partition.c)
  endif()

  if (CONFIG_MTD_SECT512)
    list(APPEND SRCS sector512.c)
  endif()

  if (CONFIG_MTD_WRBUFFER)
    list(APPEND SRCS mtd_rwbuffer.c)
  else()
    if (CONFIG_MTD_READAHEAD)
      list(APPEND SRCS mtd_rwbuffer.c)
    endif()
  endif()

  if (CONFIG_MTD_PROGMEM)
    list(APPEND SRCS mtd_progmem.c)
  endif()

  if (CONFIG_MTD_NAND)
    list(APPEND SRCS mtd_nand.c mtd_onfi.c mtd_nandscheme.c mtd_nandmodel.c mtd_modeltab.c)

    if (CONFIG_MTD_NAND_SWECC)
      list(APPEND SRCS mtd_nandecc.c hamming.c)
    endif()
  endif()

  if (CONFIG_RAMMTD)
    list(APPEND SRCS rammtd.c)
  endif()

  if (CONFIG_FILEMTD)
    list(APPEND SRCS filemtd.c)
  endif()

  if (CONFIG_MTD_AT24XX)
    list(APPEND SRCS at24xx.c)
  endif()

  if (CONFIG_MTD_AT45DB)
    list(APPEND SRCS at45db.c)
  endif()

  if (CONFIG_MTD_RAMTRON)
    list(APPEND SRCS ramtron.c)
  endif()

  if (CONFIG_MTD_SST25)
    list(APPEND SRCS sst25.c)
  endif()

  if (CONFIG_MTD_SST25XX)
    list(APPEND SRCS sst25xx.c)
  endif()

  if (CONFIG_MTD_SST26)
    list(APPEND SRCS sst26.c)
  endif()

  if (CONFIG_MTD_SST39FV)
    list(APPEND SRCS sst39vf.c)
  endif()

  if (CONFIG_MTD_W25)
    list(APPEND SRCS w25.c)
  endif()

  if (CONFIG_MTD_GD25)
    list(APPEND SRCS gd25.c)
  endif()

  if (CONFIG_MTD_GD5F)
    list(APPEND SRCS gd5f.c)
  endif()

  if (CONFIG_MTD_AT25)
    list(APPEND SRCS at25.c)
  endif()

  if (CONFIG_MTD_M25P)
    list(APPEND SRCS m25px.c)
  endif()

  if (CONFIG_MTD_MX25L)
    list(APPEND SRCS mx25lx.c)
  endif()

  if (CONFIG_MTD_MX35)
    list(APPEND SRCS mx35.c)
  endif()

  if (CONFIG_MTD_S25FL1)
    list(APPEND SRCS s25fl1.c)
  endif()

  if (CONFIG_MTD_N25QXXX)
    list(APPEND SRCS n25qxxx.c)
  endif()

  if (CONFIG_MTD_W25QXXXJV)
    list(APPEND SRCS w25qxxxjv.c)
  endif()

  if (CONFIG_MTD_MX25RXX)
    list(APPEND SRCS mx25rxx.c)
  endif()

  if (CONFIG_MTD_IS25XP)
    list(APPEND SRCS is25xp.c)
  endif()

  if (CONFIG_MTD_SMART)
    if (CONFIG_FS_SMARTFS)
      list(APPEND SRCS smart.c)
    endif()
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()

