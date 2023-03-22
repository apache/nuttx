############################################################################
# tools/Export.mk
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

include $(TOPDIR)/.config
include $(EXPORTDIR)/Make.defs

ifneq ($(ARCHSCRIPT),)
  LDPATH = $(call CONVERT_PATH,$(ARCHSCRIPT))
  LDNAME = ${notdir ${LDPATH}}
  LDDIR = ${dir ${LDPATH}}
endif

ARCHSUBDIR = "arch/$(CONFIG_ARCH)/src"
ARCHDIR ="$(TOPDIR)/$(ARCHSUBDIR)"

all: $(EXPORTDIR)/makeinfo.sh
default: all
.PHONY: clean

$(EXPORTDIR)/makeinfo.sh: $(TOPDIR)/.config $(EXPORTDIR)/Make.defs
	@echo "#!/bin/bash" > $(EXPORTDIR)/makeinfo.sh
	@echo "" >> $(EXPORTDIR)/makeinfo.sh
	@echo "AR=\"$(AR)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHCFLAGS=\"$(ARCHCFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHCPUFLAGS=\"$(ARCHCPUFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHCXXFLAGS=\"$(ARCHCXXFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHDIR=\"$(ARCHDIR)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHOPTIMIZATION=\"$(ARCHOPTIMIZATION)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHPICFLAGS=\"$(ARCHPICFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHSUBDIR=\"$(ARCHSUBDIR)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHWARNINGS=\"$(ARCHWARNINGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "ARCHWARNINGSXX=\"$(ARCHWARNINGSXX)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "CC=\"$(CC)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "CPP=\"$(CPP)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "CROSSDEV=\"$(CROSSDEV)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "CXX=\"$(CXX)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "EXEEXT=\"$(EXEEXT)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "EXTRA_LIBPATHS=\"$(EXTRA_LIBPATHS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "EXTRA_LIBS=\"$(EXTRA_LIBS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "EXTRA_OBJS=\"$(EXTRA_OBJS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "HOSTCC=\"$(HOSTCC)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "HOSTCFLAGS=\"$(HOSTCFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "HOSTEXEEXT=\"$(HOSTEXEEXT)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "HOSTINCLUDES=\"$(HOSTINCLUDES)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "HOSTLDFLAGS=\"$(HOSTLDFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
ifdef ARCHSCRIPT
	@echo "LDDIR=\"$(LDDIR)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDNAME=\"$(LDNAME)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDPATH=\"$(LDPATH)\"" >> $(EXPORTDIR)/makeinfo.sh
endif
	@echo "LD=\"$(LD)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDENDGROUP=\"$(LDENDGROUP)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDELFFLAGS=\"$(LDELFFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDFLAGS=\"$(LDFLAGS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDLIBS=\"$(LDLIBS)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDSCRIPT=\"$(LDSCRIPT)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LDSTARTGROUP=\"$(LDSTARTGROUP)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "LIBEXT=\"$(LIBEXT)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "NM=\"$(NM)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "NXFLATLDFLAGS1=\"$(NXFLATLDFLAGS1)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "NXFLATLDFLAGS2=\"$(NXFLATLDFLAGS2)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "OBJCOPY=\"$(OBJCOPY)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "OBJDUMP=\"$(OBJDUMP)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "OBJEXT=\"$(OBJEXT)\"" >> $(EXPORTDIR)/makeinfo.sh
	@echo "STRIP=\"$(STRIP)\"" >> $(EXPORTDIR)/makeinfo.sh
	$(Q) chmod 755 $(EXPORTDIR)/makeinfo.sh

clean:
	$(Q) rm -f $(EXPORTDIR)/makeinfo.sh
