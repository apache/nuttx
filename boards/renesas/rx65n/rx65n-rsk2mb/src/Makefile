############################################################################
# configs/rx65n-rsk2mb/src/Makefile
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

ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
CFLAGS += -I=$(ARCH_SRCDIR)/chip -I=$(ARCH_SRCDIR)/common -I=$(TOPDIR)/sched
CFLAGS += -I=$(ARCH_SRCDIR)/chip

ASRCS =
AOBJS = $(ASRCS:.asm=$(OBJEXT))
CSRCS = rx65n_appinit.c rx65n_bringup.c rx65n_sbram.c rx65n_gpio.c
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

CFLAGS += -I=$(TOPDIR)/arch/$(CONFIG_ARCH)/src
CFLAGS += -I=$(TOPDIR)/arch/$(CONFIG_ARCH)/include

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

ifneq ($(BOARD_CONTEXT),y)
context:
endif

-include Make.dep
