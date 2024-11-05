############################################################################
# libs/libxx/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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
#
###########################################################################

include $(TOPDIR)/Make.defs

# Include the uClibc++ Make.defs file if selected.  If it is included,
# the uClibc++/Make.defs file will add its files to the source file list,
# add its DEPPATH info, and will add the appropriate paths to the VPATH
# variable
#
# Note that an error will occur if you select CONFIG_LIBXX_UCLIBCXX
# without installing the uClibc++ package.  This is intentional to let
# you know about the configuration problem.  Refer to the README.txt file
# in the NuttX uClibc++ GIT repository for more information

BIN = libxx$(LIBEXT)
BINDIR = bin

ifeq ($(CONFIG_UCLIBCXX),y)
include uClibc++.defs
else ifeq ($(CONFIG_LIBCXX),y)
include libcxx.defs
else ifeq ($(CONFIG_LIBCXXMINI),y)
include libcxxmini.defs
endif

ifeq ($(CONFIG_ETL),y)
include etl.defs
endif

ifeq ($(CONFIG_LIBCXXABI),y)
include libcxxabi.defs
endif

# Object Files

AOBJS = $(addprefix $(BINDIR)$(DELIM), $(ASRCS:.S=$(OBJEXT)))
COBJS = $(addprefix $(BINDIR)$(DELIM), $(CSRCS:.c=$(OBJEXT)))
CXXOBJS = $(addprefix $(BINDIR)$(DELIM), $(CXXSRCS:.cxx=$(OBJEXT)))
CPPOBJS = $(addprefix $(BINDIR)$(DELIM), $(CPPSRCS:.cpp=$(OBJEXT)))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS) $(CPPSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS) $(CPPOBJS)

all: $(BIN)
.PHONY: depend clean distclean context

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(CPPOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

context::

makedepfile: $(CXXSRCS:.cxx=.ddx) $(CPPSRCS:.cpp=.ddp)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(Q) $(MAKE) -C bin clean
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean:: clean
	$(Q) $(MAKE) -C bin distclean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
