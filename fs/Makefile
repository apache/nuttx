############################################################################
# fs/Makefile
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

CSRCS = fs_initialize.c

include inode/Make.defs
include vfs/Make.defs
include driver/Make.defs
include aio/Make.defs
include mmap/Make.defs

# OS resources

include semaphore/Make.defs
include mqueue/Make.defs
include shm/Make.defs
include socket/Make.defs

# Additional files required is mount-able file systems are supported

ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)

include mount/Make.defs
include partition/Make.defs
include fat/Make.defs
include romfs/Make.defs
include cromfs/Make.defs
include tmpfs/Make.defs
include nxffs/Make.defs
include nfs/Make.defs
include smartfs/Make.defs
include binfs/Make.defs
include procfs/Make.defs
include spiffs/Make.defs
include unionfs/Make.defs
include userfs/Make.defs
include hostfs/Make.defs
include littlefs/Make.defs
include exfat/Make.defs
include rpmsgfs/Make.defs
include fatfs/Make.defs

endif

CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)fs}
CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)sched}

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libfs$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $^)

context::

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
