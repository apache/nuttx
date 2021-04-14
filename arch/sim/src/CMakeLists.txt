############################################################################
# arch/sim/src/CMakeLists.txt
#
#   Copyright (C) 2007, 2008, 2011-2012, 2014, 2016, 2018 Gregory Nutt. All
#     rights reserved.
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
############################################################################

-include $(TOPDIR)/Make.defs

CPPFLAGS += -I$(TOPDIR)/sched $(EXTRADEFINES)
CFLAGS += -I$(TOPDIR)/sched $(EXTRADEFINES)
CXXFLAGS += -I$(TOPDIR)/sched $(EXTRADEFINES)

ASRCS =

if("${CONFIG_HOST_X86_64}" STREQUAL "y")
if("${CONFIG_SIM_M32}" STREQUAL "y")
  ASRCS += up_setjmp32.S
else()
  ASRCS += up_setjmp64.S
endif()
elseif("${CONFIG_HOST_X86}" STREQUAL "y")
  ASRCS += up_setjmp32.S
elseif("${CONFIG_HOST_ARM}" STREQUAL "y")
  ASRCS += up_setjmp_arm.S
endif()

AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS  = up_initialize.c up_idle.c up_interruptcontext.c up_initialstate.c
CSRCS += up_createstack.c up_usestack.c up_releasestack.c up_stackframe.c
CSRCS += up_unblocktask.c up_blocktask.c up_releasepending.c
CSRCS += up_reprioritizertr.c up_exit.c up_schedulesigaction.c up_spiflash.c
CSRCS += up_allocateheap.c up_devconsole.c up_qspiflash.c

HOSTSRCS = up_hostusleep.c

if("${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
  CSRCS += up_tickless.c
endif()

if("${CONFIG_SPINLOCK}" STREQUAL "y")
  HOSTSRCS += up_testset.c
endif()

if("${CONFIG_SMP}" STREQUAL "y")
  CSRCS += up_smpsignal.c up_smphook.c up_cpuidlestack.c
  HOSTSRCS += up_simsmp.c
endif()

if("${CONFIG_SCHED_INSTRUMENTATION}" STREQUAL "y")
ifneq ($(CONFIG_SCHED_INSTRUMENTATION_BUFFER}" STREQUAL "y")
  CSRCS += up_schednote.c
endif()
endif()

if("${CONFIG_DEV_CONSOLE}" STREQUAL "y")
  CSRCS += up_uartwait.c
  HOSTSRCS += up_simuart.c
endif()

if("${CONFIG_ONESHOT}" STREQUAL "y")
  CSRCS += up_oneshot.c
endif()

if("${CONFIG_SCHED_CRITMONITOR}" STREQUAL "y")
  HOSTSRCS += up_critmon.c
endif()

if("${CONFIG_NX_LCDDRIVER}" STREQUAL "y")
  CSRCS += board_lcd.c
else()
  CSRCS += up_framebuffer.c
if("${CONFIG_SIM_X11FB}" STREQUAL "y")
  HOSTSRCS += up_x11framebuffer.c
if("${CONFIG_SIM_TOUCHSCREEN}" STREQUAL "y")
  CSRCS += up_touchscreen.c
  HOSTSRCS += up_x11eventloop.c
else()
if("${CONFIG_SIM_AJOYSTICK}" STREQUAL "y")
  CSRCS += up_ajoystick.c
  HOSTSRCS += up_x11eventloop.c
endif()
endif()
endif()
endif()

if("${CONFIG_SIM_IOEXPANDER}" STREQUAL "y")
  CSRCS += up_ioexpander.c
endif()

if("${CONFIG_FS_FAT}" STREQUAL "y")
  CSRCS += up_blockdevice.c up_deviceimage.c
endif()

if("${CONFIG_ARCH_ROMGETC}" STREQUAL "y")
  CSRCS += up_romgetc.c
endif()

if("${CONFIG_SIM_NETDEV}" STREQUAL "y")
if("${CONFIG_NET_ETHERNET}" STREQUAL "y")
  CSRCS += up_netdriver.c
  HOSTCFLAGS += -DNETDEV_BUFSIZE=$(CONFIG_NET_ETH_PKTSIZE)
ifneq ($(HOSTOS),Cygwin)
  HOSTSRCS += up_tapdev.c up_netdev.c
if("${CONFIG_SIM_NET_BRIDGE}" STREQUAL "y")
  HOSTCFLAGS += -DCONFIG_SIM_NET_BRIDGE
  HOSTCFLAGS += -DCONFIG_SIM_NET_BRIDGE_DEVICE=\"$(CONFIG_SIM_NET_BRIDGE_DEVICE)\"
endif()
if("${CONFIG_SIM_NET_HOST_ROUTE}" STREQUAL "y")
  HOSTCFLAGS += -DCONFIG_SIM_NET_HOST_ROUTE
endif()
else # HOSTOS != Cygwin
  HOSTSRCS += up_wpcap.c up_netdev.c
  DRVLIB = /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a
endif # HOSTOS != Cygwin
endif # CONFIG_NET_ETHERNET
endif # CONFIG_SIM_NETDEV

if("${CONFIG_SMP}" STREQUAL "y")
  HOSTCFLAGS += -DCONFIG_SMP=1 -DCONFIG_SMP_NCPUS=$(CONFIG_SMP_NCPUS)
endif()

if("${CONFIG_FS_HOSTFS}" STREQUAL "y")
  HOSTSRCS += up_hostfs.c

up_hostfs.c: hostfs.h

hostfs.h: $(TOPDIR)/include/nuttx/fs/hostfs.h
	@echo "CP:  $<"
	$(Q) cp $< $@
endif()

COBJS = $(CSRCS:.c=$(OBJEXT))

NUTTXOBJS = $(AOBJS) $(COBJS)
HOSTOBJS = $(HOSTSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(HOSTSRCS)
OBJS = $(AOBJS) $(COBJS) $(HOSTOBJS)

# Determine which standard libraries will need to be linked in

if("${CONFIG_SIM_X11FB}" STREQUAL "y")
  STDLIBS += -lX11 -lXext
endif()

# Override in Make.defs if linker is not 'ld'

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=

if("${CONFIG_FS_FAT}" STREQUAL "y")
STDLIBS += -lz
endif()

STDLIBS += -lc
STDLIBS += -lpthread

# Determine which objects are required in the link.  The
# up_head object normally draws in all that is needed, but
# there are a fews that must be included because they
# are called only from the host OS-specific logic (HOSTOBJS)

LINKOBJS = up_head$(OBJEXT)
REQUIREDOBJS = $(LINKOBJS)

if("${CONFIG_DEV_CONSOLE}" STREQUAL "y")
  REQUIREDOBJS += up_uartwait$(OBJEXT)
endif()

if("${CONFIG_SIM_X11FB}" STREQUAL "y")
if("${CONFIG_SIM_TOUCHSCREEN}" STREQUAL "y")
  REQUIREDOBJS += up_touchscreen$(OBJEXT)
endif()
endif()

if("${CONFIG_SMP}" STREQUAL "y")
  REQUIREDOBJS += up_smpsignal$(OBJEXT) up_smphook$(OBJEXT)
endif()

# Determine which NuttX libraries will need to be linked in
# Most are provided by LINKLIBS on the MAKE command line

LINKLIBS ?=
RELLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
RELPATHS += -L"$(TOPDIR)/staging"

# Add the board-specific library and directory

LIBPATHS += -L board
RELPATHS += -L board
RELLIBS += -lboard

# Make targets begin here

all: up_head$(OBJEXT) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT) export_startup clean distclean cleanrel depend

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(HOSTOBJS): %$(OBJEXT): %.c
	$(Q) echo "CC:  $<"
	$(Q) "$(CC)" -c $(HOSTCFLAGS) $< -o $@

# The architecture-specific library

libarch$(LIBEXT): $(NUTTXOBJS)
	$(call ARCHIVE, $@, $(NUTTXOBJS))

# The "board"-specific library. Of course, there really are no boards in
# the simulation.  However, this is a good place to keep parts of the simulation
# that are not hardware-related.

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

# A partially linked object containing only NuttX code (no interface to host OS)
# Change the names of most symbols that conflict with libc symbols.

GNU:
	$(Q) mkdir -p ./GNU

GNU/Linux-names.dat: GNU nuttx-names.dat
	$(Q) cp nuttx-names.dat $@

Msys-names.dat: GNU nuttx-names.dat
	$(Q) cp nuttx-names.dat $@

Cygwin-names.dat: nuttx-names.dat
ifeq ($())CONFIG_SIM_CYGWIN_DECORATED}" STREQUAL "y")
	$(Q) cat $^ | sed -e "s/^/_/g" >$@
else()
	$(Q) cp nuttx-names.dat $@
endif()

nuttx.rel : libarch$(LIBEXT) board/libboard$(LIBEXT) $(HOSTOS)-names.dat $(LINKOBJS)
	$(Q) echo "LD:  nuttx.rel"
	$(Q) $(LD) -r $(LDLINKFLAGS) $(RELPATHS) $(EXTRA_LIBPATHS) -o $@ $(REQUIREDOBJS) $(LDSTARTGROUP) $(RELLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
	$(Q) $(OBJCOPY) --redefine-syms=$(HOSTOS)-names.dat $@

# Generate the final NuttX binary by linking the host-specific objects with the NuttX
# specific objects (with munged names)

nuttx$(EXEEXT): cleanrel nuttx.rel $(HOSTOBJS)
	$(Q) echo "LD:  nuttx$(EXEEXT)"
	$(Q) "$(CC)" $(CCLINKFLAGS) $(LIBPATHS) -o $(TOPDIR)/$@ nuttx.rel $(HOSTOBJS) $(DRVLIB) $(STDLIBS)
	$(Q) $(NM) $(TOPDIR)/$@ | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		sort > $(TOPDIR)/System.map
	$(Q) rm -f nuttx.rel

# This is part of the top-level export target

export_startup: board/libboard$(LIBEXT) up_head.o $(HOSTOBJS)
	cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup
	cp nuttx-names.dat ${EXPORT_DIR}/libs
	echo main NXmain >> ${EXPORT_DIR}/libs/nuttx-names.dat

# Dependencies

.depend: CMakeLists.txt $(SRCS)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

cleanrel:
	$(Q) rm -f nuttx.rel GNU/Linux-names.dat Cygwin-names.dat

clean: cleanrel
	$(Q) if [ -e board/CMakeLists.txt ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
	$(call DELFILE, nuttx.rel)
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(Q) if [ -e board/CMakeLists.txt ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, hostfs.h)
	$(Q) rm -rf GNU

-include Make.dep
