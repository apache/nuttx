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
	$(call DELFILE, $(2))
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
	if not exist $(2) $(call MOVEFILE, $(subst .c,.obj,$(1)), $(2))
endef

define ASSEMBLE
	$(call DELFILE, $(2))
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
	if not exist $(2) $(call MOVEFILE, $(subst .asm,.obj,$(1)), $(2))
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

# COMPILE, ASSEMBLE, MOVEOBJ
#
# The ZDS-II compiler and assembler both generate object files in the
# current directory with the same name as the source file, but with the .obj
# extension.  The build system expects these behaviors when compiling some
# file, say foo.c:
#
# 1. If the foo.obj object belongs in a lower level directory (such as bin/),
#    then the relative path will be preface the object file name (such as
#    bin/foo.obj)).  In this case, the build system will call MOVEOBJ to
#    move the objects in place and nothing special need be done here.
# 2. In other cases, the build system may decorate the object file name such
#    as a.b.c.foo.obj.  This case is distinguished here by because does not
#    lie in a lower directory, but lies in the current directory and is
#    handled within COMPILE and ASSEMBLE.

define COMPILE
	$(call DELFILE, $(2))
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) `cygpath -w "$(1)"`
	$(Q) ( \
			set -x ; \
			__rename=`basename $(2)` ;\
			if [ ! -e $${__rename} ] ; then \
				__src=`basename $(1) | cut -d'.' -f1` ; \
				__dest=`echo $(2) | sed -e "s/.obj//g"` ; \
				mv -f $${__src}.obj $(2) ; \
				mv -f $${__src}.lst $${__dest}.lst ; \
				mv -f $${__src}.src $${__dest}.src ; \
			fi ; \
		)
endef

define ASSEMBLE
	$(call DELFILE, $(2))
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) `cygpath -w "$(1)"`
	$(Q) ( \
			set -x ; \
			__rename=`basename $(2)` ; \
			if [ ! -e $${__rename} ] ; then \
				__src=`basename $(1) | cut -d'.' -f1` ; \
				__dest=`echo $(2) | sed -e "s/.obj//g"` ; \
				mv -f $${__src}.obj $(2) ; \
				mv -f $${__src}.lst $${__dest}.lst ; \
				mv -f $${__src}.src $${__dest}.src ; \
			fi ; \
		)
endef

define MOVEOBJ
	$(call MOVEFILE, "$(1).obj", "$(2)$(DELIM)$(1).obj")
	$(call MOVEFILE, "$(1).lst", "$(2)$(DELIM)$(1).lst")
	$(call MOVEFILE, "$(1).src", "$(2)$(DELIM)$(1).src")
endef

# ARCHIVE will move a list of object files into the library.  This is
# complex because:
#
# 1. The ez80lib.exe archive expects the library to reside within the
#    current directory; it expects the library argument to a file name
#    like foo.lib.
# 2. Normally, the library file is in the current directory, but other
#    times, the library is an absolute path such as
#    D:\Spuda\Documents\projects\nuttx\master\apps-fork\libapps.lib.  In
#    this case, the base library name is extact as the ARCHIVE logic CD's
#    to the directory containing the library.

define ARCHIVE
	( \
		set -x ;\
		__wd=`pwd` ;\
		__home=`dirname $(1)` ; \
		if [ -z "$${__home}" ] ; then \
			__lib=$(1) ; \
		else \
			__lib=`basename $(1)` ; \
			cd $${__home} ; \
		fi ; \
		for __obj in $(2) ; do \
			$(AR) $(ARFLAGS) $${__lib}=-+$$__obj ; \
		done ; \
		cd $${__wd} ; \
	)
endef

define CLEAN
	$(Q) rm -f *.obj *.src *.lib *.hex *.lod *.lst
endef
endif
