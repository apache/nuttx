############################################################
# Makefile
#
#   Copyright (C) 2007 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
############################################################

TOPDIR		= ${shell pwd}
-include ${TOPDIR}/.config
-include ${TOPDIR}/Make.defs

ARCH_DIR	= arch/$(CONFIG_ARCH)
ARCH_SRC	= $(ARCH_DIR)/src

SUBDIRS		= sched lib $(ARCH_SRC) mm fs drivers examples/$(CONFIG_EXAMPLE)

LINKOBJS	= $(ARCH_SRC)/up_head$(OBJEXT)
LINKLIBS	= sched/libsched$(LIBEXT) $(ARCH_SRC)/libarch$(LIBEXT) mm/libmm$(LIBEXT) \
		  fs/libfs$(LIBEXT) drivers/libdrivers$(LIBEXT) lib/liblib$(LIBEXT) \
		  examples/$(CONFIG_EXAMPLE)/lib$(CONFIG_EXAMPLE)$(LIBEXT)

BIN		= nuttx

all: $(BIN)
.PHONY: clean context clean_context distclean

tools/mkconfig:
	$(MAKE) -C tools -f Makefile.mkconfig TOPDIR=$(TOPDIR)  mkconfig

include/nuttx/config.h: $(ARCH_DIR)/defconfig tools/mkconfig
	tools/mkconfig $(ARCH_DIR) > include/nuttx/config.h

include/arch: include/nuttx/config.h
	ln -sf $(TOPDIR)/$(ARCH_DIR)/include include/arch

context: check_context include/nuttx/config.h include/arch

clean_context:
	rm -f include/nuttx/config.h
	rm -f include/arch

check_context:
	@if [ ! -e ${TOPDIR}/.config -o ! -e ${TOPDIR}/Make.defs ]; then \
		echo "" ; echo "Nuttx has not been configured:" ; \
		echo "  cd tools; ./configure.sh <target>\n" ; echo "" ;\
		exit 1 ; \
	fi

sched/libsched$(LIBEXT): context
	$(MAKE) -C sched TOPDIR=$(TOPDIR) libsched$(LIBEXT)

lib/liblib$(LIBEXT): context
	$(MAKE) -C lib TOPDIR=$(TOPDIR) liblib$(LIBEXT)

$(ARCH_SRC)/libarch$(LIBEXT): context
	$(MAKE) -C $(ARCH_SRC) TOPDIR=$(TOPDIR) libarch$(LIBEXT)

$(ARCH_SRC)/up_head$(OBJEXT): context
	$(MAKE) -C $(ARCH_SRC) TOPDIR=$(TOPDIR) up_head$(OBJEXT)

mm/libmm$(LIBEXT): context
	$(MAKE) -C mm TOPDIR=$(TOPDIR) libmm$(LIBEXT)

fs/libfs$(LIBEXT): context
	$(MAKE) -C fs TOPDIR=$(TOPDIR) libfs$(LIBEXT)

drivers/libdrivers$(LIBEXT): context
	$(MAKE) -C drivers TOPDIR=$(TOPDIR) libdrivers$(LIBEXT)

examples/$(CONFIG_EXAMPLE)/lib$(CONFIG_EXAMPLE)$(LIBEXT): context
	$(MAKE) -C examples/$(CONFIG_EXAMPLE) TOPDIR=$(TOPDIR) lib$(CONFIG_EXAMPLE)$(LIBEXT)

$(BIN):	context depend $(LINKOBJS) $(LINKLIBS)
	$(MAKE) -C $(ARCH_SRC) TOPDIR=$(TOPDIR) LINKOBJS="$(LINKOBJS)" LINKLIBS="$(LINKLIBS)" $(BIN)

depend:
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR=$(TOPDIR) depend ; \
	done

clean:
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR=$(TOPDIR) clean ; \
	done
	$(MAKE) -C tools -f Makefile.mkconfig TOPDIR=$(TOPDIR) clean
	$(MAKE) -C mm -f Makefile.test TOPDIR=$(TOPDIR) clean
	rm -f $(BIN) $(BIN).rr $(BIN).stripped $(BIN).flashimage mm_test System.map *~

distclean: clean clean_context
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR=$(TOPDIR) distclean ; \
	done
	$(MAKE) -C examples/$(CONFIG_EXAMPLE) TOPDIR=$(TOPDIR) distclean
	rm -f Make.defs setenv.sh .config


