############################################################################
# libs/libxx/Makefile
#
#   Copyright (C) 2009, 2012, 2016-2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

-include $(TOPDIR)/Make.defs

ifeq ($(CONFIG_LIBCXX),y)
LIBXX=libcxx
else
LIBXX=libxx
endif

# Sources

ASRCS =
CSRCS =

CXXSRCS  = libxx_cxapurevirtual.cxx libxx_eabi_atexit.cxx  libxx_cxa_atexit.cxx
ifneq (,$(findstring y,$(CONFIG_UCLIBCXX_EXCEPTION) $(CONFIG_LIBCXX_EXCEPTION)))
CXXSRCS += libxx__gnu_unwind_find_exidx.cxx
endif

# Some of the libs/libxx/ files are not need if uClibc++ or libcxx is installed
# because uClibx++ or libcxx will replace them

ifeq (,$(findstring y,$(CONFIG_UCLIBCXX) $(CONFIG_LIBCXX)))
CXXSRCS += libxx_delete.cxx libxx_delete_sized.cxx libxx_deletea.cxx
CXXSRCS += libxx_deletea_sized.cxx libxx_new.cxx libxx_newa.cxx
CXXSRCS += libxx_stdthrow.cxx
else
ifeq (,$(findstring y,$(CONFIG_UCLIBCXX_EXCEPTION) $(CONFIG_LIBCXX_EXCEPTION)))
CXXSRCS += libxx_stdthrow.cxx
endif
endif

# uClibc++ doesn't need this file

ifneq ($(CONFIG_UCLIBCXX),y)
CXXSRCS += libxx_cxa_guard.cxx
endif

# Paths

DEPPATH = --dep-path .
VPATH = .

# Include the uClibc++ Make.defs file if selected.  If it is included,
# the uClibc++/Make.defs file will add its files to the source file list,
# add its DEPPATH info, and will add the appropriate paths to the VPATH
# variable
#
# Note that an error will occur if you select CONFIG_LIBXX_UCLIBCXX
# without installing the uClibc++ package.  This is intentional to let
# you know about the configuration problem.  Refer to the README.txt file
# in the NuttX uClibc++ GIT repository for more information

ifeq ($(CONFIG_UCLIBCXX),y)
include uClibc++/Make.defs
endif

ifeq ($(CONFIG_LIBCXX),y)
include libcxx/Make.defs
endif

# Object Files

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS)

BIN = $(LIBXX)$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
