############################################################################
# tools/imx9/Config.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# These are the macros that will be used in the NuttX make system to compile
# and assembly source files and to insert the resulting object files into an
# archive.  These replace the default definitions at tools/Config.mk

# POSTBUILD -- Perform post build operations

ifeq ($(CONFIG_IMX9_BOOTLOADER),y)
	MK_BASE_URL = https://raw.githubusercontent.com/nxp-imx/imx-mkimage/cbb99377cc2bb8f7cf213794c030e1c60423ef1f/src
	MK_SCRIPTS_URL = https://raw.githubusercontent.com/nxp-imx/imx-mkimage/cbb99377cc2bb8f7cf213794c030e1c60423ef1f/scripts
	BASE_PATH = $(TOPDIR)$(DELIM)tools$(DELIM)imx9$(DELIM)
	FILE_1 = imx8qxb0.c
	FILE_1_PATH = $(BASE_PATH)$(FILE_1)
	FILE_2 = mkimage_common.h
	FILE_2_PATH = $(BASE_PATH)$(FILE_2)
	FILE_3 = mkimage_imx8.c
	FILE_3_PATH = $(BASE_PATH)$(FILE_3)
	FILE_EXE = $(BASE_PATH)mkimage_imx9
	AHAB_BASE_URL = https://www.nxp.com/lgfiles/NMG/MAD/YOCTO
	AHAB = firmware-ele-imx-0.1.1
	AHAB_BINARY = $(AHAB).bin
	AHAB_PATH = $(BASE_PATH)$(AHAB_BINARY)
	FSPI_HEADER = fspi_header
	FSPI_HEADER_PATH = $(BASE_PATH)$(FSPI_HEADER)
	FCB_TOOL = fspi_fcb_gen.sh
	FCB_TOOL_PATH = $(BASE_PATH)$(FCB_TOOL)
	EXTRAFLAGS +=  -mstrict-align

define DOWNLOAD_FILES
	$(call DOWNLOAD,$(MK_BASE_URL),$(FILE_1),$(FILE_1_PATH))
	$(call DOWNLOAD,$(MK_BASE_URL),$(FILE_2),$(FILE_2_PATH))
	$(call DOWNLOAD,$(MK_BASE_URL),$(FILE_3),$(FILE_3_PATH))
	$(call DOWNLOAD,$(MK_SCRIPTS_URL),$(FSPI_HEADER),$(FSPI_HEADER_PATH))
	$(call DOWNLOAD,$(MK_SCRIPTS_URL),$(FCB_TOOL),$(FCB_TOOL_PATH))
	$(call DOWNLOAD,$(AHAB_BASE_URL),$(AHAB_BINARY),$(AHAB_PATH))
	$(Q) chmod a+x $(BASE_PATH)$(AHAB_BINARY)
	$(Q) (cd $(BASE_PATH) && ./$(AHAB_BINARY) --auto-accept)
endef

ifeq ("$(wildcard $(FILE_EXE))","")
	MKIMAGE_NOT_PRESENT = 1
endif

define POSTBUILD
	$(Q) echo "Removing sections"
	$(Q) $(OBJCOPY) -O binary -R .bss -R .initstack $(BIN) nuttx.bin
	$(Q) ([ $$? -eq 0 ] && echo "Done.")

	$(Q) echo "Constructing sd image"
	$(Q) echo "#define MKIMAGE_COMMIT 0xcbb99377" > $(BASE_PATH)build_info.h

	$(if $(MKIMAGE_NOT_PRESENT),$(call DOWNLOAD_FILES))

	+$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools$(DELIM)imx9 -f Makefile.host
	$(Q) tools$(DELIM)imx9$(DELIM)mkimage_imx9$(HOSTEXEEXT) -soc IMX9 -append $(BASE_PATH)$(AHAB)$(DELIM)mx93a1-ahab-container.img -c -ap nuttx.bin a55 0x2049a000 -out flash.bin 1>/dev/null 2>&1
	$(Q) dd if=/dev/zero of=imx9-sdimage.img bs=1k count=32 1>/dev/null 2>&1
	$(Q) cat flash.bin >> imx9-sdimage.img
	$(Q) rm flash.bin
	$(Q) echo "imx9-sdimage.img" >> nuttx.manifest
	$(Q) echo "Created imx9-sdimage.img"


	$(Q) sh tools$(DELIM)imx9$(DELIM)fspi_fcb_gen.sh tools$(DELIM)imx9$(DELIM)/fspi_header
	$(Q) tools$(DELIM)imx9$(DELIM)mkimage_imx9$(HOSTEXEEXT) -soc IMX9 -dev flexspi -append $(BASE_PATH)$(AHAB)$(DELIM)mx93a1-ahab-container.img -c -ap nuttx.bin a55 0x2049a000 -fcb fcb.bin 0x204F0000 -out flash.bin 1>/dev/null 2>&1
	$(Q) mv flash.bin flash.tmp

	$(Q) echo "Append FCB to flash.bin"
	$(Q) dd if=fcb.bin of=flash.bin bs=1k seek=1
	$(Q) dd if=flash.tmp of=flash.bin bs=1k seek=4
	$(Q) rm flash.tmp
	$(Q) rm fcb.bin

	$(Q) cp flash.bin  imx9-norimage.img
	$(Q) rm flash.bin
	$(Q) echo "imx9-norimage.img" >> nuttx.manifest
	$(Q) echo "Created imx9-norimage.img"
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools$(DELIM)imx9 -f Makefile.host clean
endef
endif
