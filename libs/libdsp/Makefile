############################################################################
# libs/libdsp/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

ifeq ($(CONFIG_LIBDSP),y)
CSRCS += lib_pid.c
CSRCS += lib_svm.c
CSRCS += lib_transform.c
CSRCS += lib_observer.c
CSRCS += lib_foc.c
CSRCS += lib_misc.c
CSRCS += lib_motor.c
CSRCS += lib_pmsm_model.c

CSRCS += lib_pid_b16.c
CSRCS += lib_svm_b16.c
CSRCS += lib_transform_b16.c
CSRCS += lib_observer_b16.c
CSRCS += lib_foc_b16.c
CSRCS += lib_misc_b16.c
CSRCS += lib_motor_b16.c
CSRCS += lib_pmsm_model_b16.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN ?= libdsp$(LIBEXT)

all: $(BIN)
.PHONY:  depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $^)

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

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
