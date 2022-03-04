############################################################################
# boards/arm/lpc31xx/ea3131/locked/Makefile
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

include $(TOPDIR)/Make.defs

# Board-specific directory, board library, and application library

PASS1_SRCDIR   = arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
PASS1_LIBBOARD = libboard$(LIBEXT)
PASS1_LIBAPPS  = libapps$(LIBEXT)

# Remove the application library (libapps) from the list of libraries.  Add
# the boad library (liboard)

PASS1_LINKLIBS  = $(filter-out $(PASS1_LIBAPPS),$(LINKLIBS))
PASS1_LINKLIBS += $(PASS1_LIBBOARD)

# Get the paths to the libraries and the links script path in format that
# is appropriate for the host OS

PASS1_LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)staging)
PASS1_LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)boards$(DELIM)arm$(DELIM)lpc31xx$(DELIM)ea3131$(DELIM)src)
PASS1_LDSCRIPT = -T $(call CONVERT_PATH,$(TOPDIR)$(DELIM)boards$(DELIM)arm$(DELIM)lpc31xx$(DELIM)ea3131$(DELIM)locked$(DELIM)ld-locked.inc)

PASS1_LDFLAGS = -r $(PASS1_LDSCRIPT)
PASS1_LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(PASS1_LINKLIBS)))
PASS1_LIBGCC = "${shell $(CC) -print-libgcc-file-name}"

# Targets:

all: $(PASS1_SRCDIR)$(DELIM)locked.r

.PHONY:	depend clean distclean

# Create include-able linker script that specifies all of the symbols to be
# resolved in the locked.r file.

ld-locked.inc: mklocked.sh $(TOPDIR)$(DELIM).config
	@echo "MK:  ld-locked.inc"
	$(Q) .$(DELIM)mklocked.sh "$(TOPDIR)"

# Make the board library. This is normally done in arch/arm/src/Makefile.
# However, we need it earlier here when doing a two-pass build so that libboard.a
# is available to link against.

$(PASS1_LIBBOARD):
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)boards$(DELIM)arm$(DELIM)lpc31xx$(DELIM)ea3131$(DELIM)src libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

# Create the locked.r file containing all of the code (except the start-up code)
# that needs to lie in the locked text region.

locked.r: ld-locked.inc $(PASS1_LIBBOARD)
	@echo "LD:  locked.r"
	$(Q) $(LD) -o $@ $(PASS1_LDFLAGS) $(PASS1_LIBPATHS) --start-group $(PASS1_LDLIBS) --end-group $(PASS1_LIBGCC)
	$(Q) $(NM) $@ > locked.map
	$(Q) fgrep " U " locked.map | grep -v nx_start
	$(Q) $(CROSSDEV)size $@

$(PASS1_SRCDIR)$(DELIM)locked.r: locked.r
	$(Q) cp -a locked.r $(TOPDIR)$(DELIM)$(PASS1_SRCDIR)$(DELIM)locked.r

.depend:

depend: .depend

clean:
	$(call DELFILE, locked.r)
	$(call DELFILE, $(TOPDIR)$(DELIM)$(PASS1_SRCDIR)$(DELIM)locked.r)
	$(call DELFILE, locked.map)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, ld-locked.inc)
