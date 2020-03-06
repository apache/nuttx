############################################################################
# tools/zds/Config.mk
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

# PREPROCESS
#
# Run a file through the C Pre-processor

define PREPROCESS
	@echo "CPP: $(1)->$(2)"
	$(Q) $(CPP) $(CPPFLAGS) $($(strip $(1))_CPPFLAGS) $(1) -o $(2)
endef

# COMPILE, ASSEMBLE, and helpers
#
# The ZDS-II compiler and assembler both generate object files in the
# current directory with the same name as the source file, but with the .obj
# extension.  The build system expects these behaviors when compiling some
# file, say foo.c:
#
# 1. If the foo.obj object belongs in a lower level directory (such as bin/),
#    then the relative path will be preface the object file name (such as
#    bin/foo.obj)).
# 2. In other cases, the build system may decorate the object file name such
#    as a.b.c.foo.obj.  This case is distinguished here by because does not
#    lie in a lower directory, but lies in the current directory.

define RMOBJS
	$(call DELFILE, $(1))
	$(call DELFILE, $(subst .obj,.lst,$(1)))
	$(call DELFILE, $(subst .obj,.src,$(1)))
endef

ifeq ($(CONFIG_WINDOWS_NATIVE),y)

define CONDMOVE
	$(Q) if not exist $1 if exist $2 (move /Y $2 $3)
endef

define MVOBJS
	$(call CONDMOVE, $(1),$(subst .obj,.src,$(2)),$(subst .obj,.src,$(3)))
	$(call CONDMOVE, $(1),$(subst .obj,.lst,$(2)),$(subst .obj,.lst,$(3)))
	$(call CONDMOVE, $(1),$(2),$(3))
endef

define COMPILE
	$(call RMOBJS, $(2))
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
	$(call MVOBJS, $(2), $(subst .c,.obj,$(notdir $(1))), $(2))
endef

define ASSEMBLE
	$(call RMOBJS, $(2))
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) ${shell echo $(1) | sed -e "s/\//\\/g"}
	$(call MVOBJS, $(2), $(subst .asm,.obj,$(notdir $(1))), $(2))
endef

else

define CONDMOVE
	$(Q) if [ ! -e $(1) -a -e $(2) ] ; then mv -f $(2) $(3) ; fi
endef

define MVOBJS
	$(call CONDMOVE, $(1),$(subst .obj,.src,$(2)),$(subst .obj,.src,$(3)))
	$(call CONDMOVE, $(1),$(subst .obj,.lst,$(2)),$(subst .obj,.lst,$(3)))
	$(call CONDMOVE, $(1),$(2),$(3))
endef

define COMPILE
	$(call RMOBJS, $(2))
	$(Q) $(CC) $(CFLAGS) $($(strip $(1))_CFLAGS) `cygpath -w "$(1)"`
	$(call MVOBJS, $(2), $(subst .c,.obj,$(notdir $(1))), $(2))
endef

define ASSEMBLE
	$(call RMOBJS, $(2))
	$(Q) $(AS) $(AFLAGS) $($(strip $(1))_AFLAGS) `cygpath -w "$(1)"`
	$(call MVOBJS, $(2), $(subst .asm,.obj,$(notdir $(1))), $(2))
endef

endif

# ARCHIVE will move a list of object files into the library.  This is
# complex because:
#
# 1. The ZDS-II librarian expects the library to reside within the
#    current directory; it expects the library argument to a file name
#    like foo.lib.
# 2. Normally, the library file is in the current directory, but other
#    times, the library is an absolute path such as
#    D:\Spuda\Documents\projects\nuttx\master\apps-fork\libapps.lib.  In
#    this case, the base library name is extract as the ARCHIVE logic CD's
#    to the directory containing the library.

ifeq ($(CONFIG_WINDOWS_NATIVE),y)

define ARCHIVE
	for %%G in ($(2)) do ( $(AR) $(ARFLAGS) $(1)=-+%%G )
endef

define ARCHIVE
	$(MAKE) -C $(TOPDIR)\tools\zds zdsar.exe
	$(TOPDIR)\tools\zds\zdsar.exe --ar "$(AR)" --ar_flags "$(ARFLAGS)" --library "$(1)" $(2)
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

define ARCHIVE
	$(MAKE) -C $(TOPDIR)/tools/zds zdsar.exe
	$(TOPDIR)/tools/zds/zdsar.exe --ar "$(AR)" --ar_flags "$(ARFLAGS)" --library $(1) $(2)
endef

define CLEAN
	$(Q) rm -f *.obj *.src *.lib *.hex *.lod *.lst
endef

endif
