############################################################################
# tools/rp2040/Config.mk
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

ifeq ($(CONFIG_RP2040_UF2_BINARY),y)
ifdef PICO_SDK_PATH
define POSTBUILD
	$(Q)echo "Generating: nuttx.uf2"; \

	+$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools$(DELIM)rp2040 -f Makefile.host
	tools$(DELIM)rp2040$(DELIM)elf2uf2$(HOSTEXEEXT) nuttx nuttx.uf2;
	$(Q)([ $$? -eq 0 ] && echo nuttx.uf2 >> nuttx.manifest && echo "Done.")
endef
else
define POSTBUILD
	$(Q) echo "PICO_SDK_PATH must be specified for flash boot"
endef
endif
endif
