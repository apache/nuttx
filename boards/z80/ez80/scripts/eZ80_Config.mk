############################################################################
# boards/z80/ez80/scripts/eZ80_Config.defs
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

ifeq ($(CONFIG_WINDOWS_NATIVE),y)

define PREPROCESS
	@echo CPP: $(1)->$(2)
	$(Q) $(CPP) $(CPPFLAGS) $($(strip $(1))_CPPFLAGS) $(1) -o $(2)
endef

define COMPILE
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
endef

define ASSEMBLE
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
endef

define MOVEOBJ
	$(call MOVEFILE, "$(1).obj", "$(2)$(DELIM)$(1).obj")
	$(call MOVEFILE, "$(1).lst", "$(2)$(DELIM)$(1).lst")
	$(call MOVEFILE, "$(1).src", "$(2)$(DELIM)$(1).src")
endef

define ARCHIVE
	for %%G in ($(2)) do ( $(AR) $(ARFLAGS) $(1)=-+%%G )
endef

define CLEAN
	$(Q) if exist *.obj (del /f /q *.obj)
	$(Q) if exist *.src (del /f /q *.src)
	$(Q) if exist *.lib (del /f /q *.lib)
	$(Q) if exist *.hex (del /f /q *.hex)
	$(Q) if exist *.lod (del /f /q *.lod)
	$(Q) if exist *.lst (del /f /q *.lst)
endef

else

define PREPROCESS
	@echo "CPP: $(1)->$(2)"
	$(Q) $(CPP) $(CPPFLAGS) $($(strip $(1))_CPPFLAGS) $(1) -o $(2)
endef

define COMPILE
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) `cygpath -w "$(1)"`
endef

define ASSEMBLE
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) `cygpath -w "$(1)"`
endef

define MOVEOBJ
	$(call MOVEFILE, "$(1).obj", "$(2)$(DELIM)$(1).obj")
	$(call MOVEFILE, "$(1).lst", "$(2)$(DELIM)$(1).lst")
	$(call MOVEFILE, "$(1).src", "$(2)$(DELIM)$(1).src")
endef

define ARCHIVE
	for __obj in $(2) ; do \
		$(AR) $(ARFLAGS) $(1)=-+$$__obj \
	done
endef

define CLEAN
	$(Q) rm -f *.obj *.src *.lib *.hex *.lod *.lst
endef
endif
