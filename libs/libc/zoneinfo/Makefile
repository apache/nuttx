############################################################################
# libs/libc/zoneinfo/Makefile
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

ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
CSRCS += tzromfs.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ZONEINFO_PATH = $(TOPDIR)/libs/libc/zoneinfo
TZBIN_PATH = $(ZONEINFO_PATH)/tzbin
TZCODE_PATH = $(ZONEINFO_PATH)/tzcode

# Common build

all: .built
.PHONY: romfs register context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: .tzbuilt romfs $(OBJS)
	$(call ARCHIVE_ADD, ..$(DELIM)$(BIN), $(OBJS))
	$(Q) touch .built

# ROMFS file system containing the TZ database

ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)

checkgenromfs:
	@genromfs -h 1>/dev/null 2>&1 || { \
 echo "Host executable genromfs not available in PATH"; \
 echo "You may need to download in from http://romfs.sourceforge.net/"; \
 exit 1; \
	}

romfs_zoneinfo.img : checkgenromfs .tzbuilt
	@genromfs -f $@ -d $(TZBIN_PATH)/usr/share/zoneinfo -V "TZDatbase" || { echo "genromfs failed" ; exit 1 ; }

romfs_zoneinfo.h : romfs_zoneinfo.img
	@xxd -i $< >$@ || { echo "xxd of $< failed" ; exit 1 ; }

romfs: romfs_zoneinfo.h

else

romfs:

endif # CONFIG_LIBC_ZONEINFO_ROMFS

# Build TZ database

tzcode:
	mkdir tzcode

tzbin:
	mkdir tzbin

tzcode-latest.tar.gz:
	$(call DOWNLOAD,ftp://ftp.iana.org/tz,tzcode-latest.tar.gz)

tzdata-latest.tar.gz:
	$(call DOWNLOAD,ftp://ftp.iana.org/tz,tzdata-latest.tar.gz)

.tzunpack: tzcode tzcode-latest.tar.gz tzdata-latest.tar.gz
	$(Q) tar zx -C tzcode -f tzcode-latest.tar.gz
	$(Q) tar zx -C tzcode -f tzdata-latest.tar.gz
	$(Q) touch .tzunpack

.tzbuilt: tzcode tzbin .tzunpack
	$(Q) $(MAKE) -C tzcode TOPDIR=$(TZBIN_PATH) install
	$(Q) touch .tzbuilt

# Create initial context

context: .tzbuilt romfs

# Create dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, .tzunpack)
	$(call DELFILE, .tzbuilt)
	$(call DELFILE, romfs_zoneinfo.img)
	$(call DELFILE, romfs_zoneinfo.h)
	$(call DELFILE, tzdata-latest.tar.gz)
	$(call DELFILE, tzcode-latest.tar.gz)
	$(call DELDIR, tzbin)
	$(call DELDIR, tzcode)

-include Make.dep
