############################################################################
# tools/mpfs/Config.mk
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

ifeq ($(CONFIG_MPFS_OPENSBI),y)
define POSTBUILD
	$(Q) echo "SBI: Creating nuttx.sbi file"
	$(Q) $(OBJCOPY) -O binary -j .text.sbi -j .ddrstorage $(BIN) nuttx.sbi
	$(Q) ([ $$? -eq 0 ] && echo "Done.")
	$(Q) echo nuttx.sbi >> nuttx.manifest
	$(Q) echo "SBI: Removing unnecessary sections from nuttx binary"
	$(Q) $(OBJCOPY) -O binary -R .text.sbi -R .l2_scratchpad -R .ddrstorage $(BIN) nuttx.bin
	$(Q) ([ $$? -eq 0 ] && echo "Done.")
endef
endif
