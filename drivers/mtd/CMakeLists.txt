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

# Include MTD drivers

set(CSRCS)

if("${CONFIG_MTD}" STREQUAL "y")

	list(APPEND CSRCS ftl.c mtd_config.c)
	
	if("${CONFIG_MTD_PARTITION}" STREQUAL "y")
		list(APPEND CSRCS mtd_partition.c)
	endif()
	
	if("${CONFIG_MTD_SECT512}" STREQUAL "y")
		list(APPEND CSRCS sector512.c)
	endif()
	
	if("${CONFIG_MTD_WRBUFFER}" STREQUAL "y")
		list(APPEND CSRCS mtd_rwbuffer.c)
	else()
		if("${CONFIG_MTD_READAHEAD}" STREQUAL "y")
			list(APPEND CSRCS mtd_rwbuffer.c)
		endif()
	endif()
	
	if("${CONFIG_MTD_PROGMEM}" STREQUAL "y")
		list(APPEND CSRCS mtd_progmem.c)
	endif()
	
	if("${CONFIG_MTD_NAND}" STREQUAL "y")
		list(APPEND CSRCS mtd_nand.c mtd_onfi.c mtd_nandscheme.c mtd_nandmodel.c mtd_modeltab.c)
		if("${CONFIG_MTD_NAND_SWECC}" STREQUAL "y")
			list(APPEND CSRCS mtd_nandecc.c hamming.c)
		endif()
	endif()
	
	if("${CONFIG_RAMMTD}" STREQUAL "y")
		list(APPEND CSRCS rammtd.c)
	endif()
	
	if("${CONFIG_FILEMTD}" STREQUAL "y")
		list(APPEND CSRCS filemtd.c)
	endif()
	
	if("${CONFIG_MTD_AT24XX}" STREQUAL "y")
		list(APPEND CSRCS at24xx.c)
	endif()
	
	if("${CONFIG_MTD_AT45DB}" STREQUAL "y")
		list(APPEND CSRCS at45db.c)
	endif()
	
	if("${CONFIG_MTD_RAMTRON}" STREQUAL "y")
		list(APPEND CSRCS ramtron.c)
	endif()
	
	if("${CONFIG_MTD_SST25}" STREQUAL "y")
		list(APPEND CSRCS sst25.c)
	endif()
	
	if("${CONFIG_MTD_SST25XX}" STREQUAL "y")
		list(APPEND CSRCS sst25xx.c)
	endif()
	
	if("${CONFIG_MTD_SST26}" STREQUAL "y")
		list(APPEND CSRCS sst26.c)
	endif()
	
	if("${CONFIG_MTD_SST39FV}" STREQUAL "y")
		list(APPEND CSRCS sst39vf.c)
	endif()
	
	if("${CONFIG_MTD_W25}" STREQUAL "y")
		list(APPEND CSRCS w25.c)
	endif()
	
	if("${CONFIG_MTD_GD25}" STREQUAL "y")
		list(APPEND CSRCS gd25.c)
	endif()
	
	if("${CONFIG_MTD_AT25}" STREQUAL "y")
		list(APPEND CSRCS at25.c)
	endif()
	
	if("${CONFIG_MTD_M25P}" STREQUAL "y")
		list(APPEND CSRCS m25px.c)
	endif()
	
	if("${CONFIG_MTD_MX25L}" STREQUAL "y")
		list(APPEND CSRCS mx25lx.c)
	endif()
	
	if("${CONFIG_MTD_MX35}" STREQUAL "y")
		list(APPEND CSRCS mx35.c)
	endif()
	
	if("${CONFIG_MTD_S25FL1}" STREQUAL "y")
		list(APPEND CSRCS s25fl1.c)
	endif()
	
	if("${CONFIG_MTD_N25QXXX}" STREQUAL "y")
		list(APPEND CSRCS n25qxxx.c)
	endif()
	
	if("${CONFIG_MTD_MX25RXX}" STREQUAL "y")
		list(APPEND CSRCS mx25rxx.c)
	endif()
	
	if("${CONFIG_MTD_IS25XP}" STREQUAL "y")
		list(APPEND CSRCS is25xp.c)
	endif()
	
	if("${CONFIG_MTD_SMART}" STREQUAL "y")
		if("${CONFIG_FS_SMARTFS}" STREQUAL "y")
			list(APPEND CSRCS smart.c)
		endif()
	endif()

	# Include MTD driver support
	
	set_parent_srcs(${CSRCS})

endif()
