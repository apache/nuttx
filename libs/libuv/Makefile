############################################################################
# libs/libuv/Makefile
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

ifeq ($(CONFIG_LIBUV),y)
CSRCS += libuv/src/unix/core.c
CSRCS += libuv/src/unix/poll.c
CSRCS += libuv/src/unix/loop.c
CSRCS += libuv/src/unix/thread.c
CSRCS += libuv/src/uv-common.c
CSRCS += libuv/src/strscpy.c
CSRCS += libuv/src/unix/nuttx.c
CSRCS += libuv/src/unix/no-proctitle.c

ifeq ($(CONFIG_LIBUV_LOOP_WATCHERS),y)
CSRCS += libuv/src/unix/loop-watcher.c
endif

ifeq ($(CONFIG_LIBUV_TIMER),y)
CSRCS += libuv/src/timer.c
endif

ifeq ($(CONFIG_LIBUV_WQ),y)
CSRCS += libuv/src/threadpool.c
endif

ifeq ($(CONFIG_LIBUV_ASYNC),y)
CSRCS += libuv/src/unix/async.c
endif

# FIXME signal does not work yet
ifeq ($(CONFIG_LIBUV_SIGNAL),y)
CSRCS += libuv/src/unix/signal.c
endif

# FIXME process does not work yet
ifeq ($(CONFIG_LIBUV_PROCESS),y)
CSRCS += libuv/src/unix/process.c
endif

endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

CFLAGS += -Ilibuv/src -D__NUTTX__

BIN ?= libuv$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
