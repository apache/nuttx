############################################################################
# arch/sim/src/Makefile
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

-include $(TOPDIR)/Make.defs

ARCH_SRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src

INCLUDES += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)}
INCLUDES += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip}
INCLUDES += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)sched}

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

# Determine which objects are required in the link.The
# up_head object normally draws in all that is needed, but
# there are a fews that must be included because they
# are called only from the host OS-specific logic(HOSTOBJS)

LINKOBJS = up_head$(OBJEXT)
REQUIREDOBJS = $(LINKOBJS)

ifeq ($(CONFIG_HOST_X86_64),y)
ifeq ($(CONFIG_SIM_M32),y)
  ASRCS += up_vfork32.S
else
  ASRCS += up_vfork64.S
endif
else ifeq ($(CONFIG_HOST_X86),y)
  ASRCS += up_vfork32.S
else ifeq ($(CONFIG_HOST_ARM),y)
  ASRCS += up_vfork_arm.S
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS  = up_initialize.c up_idle.c up_interruptcontext.c up_initialstate.c
CSRCS += up_createstack.c up_usestack.c up_releasestack.c up_stackframe.c
CSRCS += up_unblocktask.c up_blocktask.c up_releasepending.c
CSRCS += up_reprioritizertr.c up_exit.c up_schedulesigaction.c
CSRCS += up_heap.c up_uart.c up_assert.c
CSRCS += up_copyfullstate.c
CSRCS += up_sigdeliver.c

ifeq ($(CONFIG_ARCH_HAVE_VFORK),y)
ifeq ($(CONFIG_SCHED_WAITPID),y)
CSRCS += up_vfork.c
endif
endif

VPATH = sim
DEPPATH = $(patsubst %,--dep-path %,$(subst :, ,$(VPATH)))

HOSTCFLAGS += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)}
ifeq ($(CONFIG_HOST_MACOS),y)
  HOSTCFLAGS += -Wno-deprecated-declarations
endif

HOSTCFLAGS += -DCONFIG_USEC_PER_TICK=$(CONFIG_USEC_PER_TICK)
HOSTSRCS = up_hostirq.c up_hostmemory.c up_hosttime.c up_simuart.c
STDLIBS += -lpthread
ifeq ($(CONFIG_HOST_MACOS),y)
ifeq ($(CONFIG_LIBCXX),y)
  STDLIBS += -lc++abi
endif
else
  STDLIBS += -lrt
endif

ifeq ($(CONFIG_STACK_COLORATION),y)
  CSRCS += up_checkstack.c
endif

ifeq ($(CONFIG_SPINLOCK),y)
  HOSTSRCS += up_testset.c
endif

ifeq ($(CONFIG_SMP),y)
  CSRCS += up_smpsignal.c up_cpuidlestack.c
  REQUIREDOBJS += up_smpsignal$(OBJEXT)
  HOSTCFLAGS += -DCONFIG_SMP=1 -DCONFIG_SMP_NCPUS=$(CONFIG_SMP_NCPUS)
ifeq ($(CONFIG_SCHED_INSTRUMENTATION),y)
  HOSTCFLAGS += -DCONFIG_SCHED_INSTRUMENTATION=1
endif
  HOSTSRCS += up_simsmp.c
endif

ifeq ($(CONFIG_ONESHOT),y)
  CSRCS += up_oneshot.c
endif

ifeq ($(CONFIG_RTC_DRIVER),y)
  CSRCS += up_rtc.c
endif

ifeq ($(CONFIG_SIM_LCDDRIVER),y)
  CSRCS += up_lcd.c
else
  ifeq ($(CONFIG_SIM_FRAMEBUFFER),y)
    CSRCS += up_framebuffer.c
  endif
endif

ifeq ($(CONFIG_SIM_X11FB),y)
ifeq ($(CONFIG_SIM_X11NOSHM),y)
  HOSTCFLAGS += -DCONFIG_SIM_X11NOSHM=1
endif
  HOSTSRCS += up_x11framebuffer.c
  STDLIBS += -lX11 -lXext
ifeq ($(CONFIG_SIM_TOUCHSCREEN),y)
  CSRCS += up_touchscreen.c
  REQUIREDOBJS += up_touchscreen$(OBJEXT)
  HOSTCFLAGS += -DCONFIG_SIM_TOUCHSCREEN=1
  HOSTSRCS += up_x11eventloop.c
else ifeq ($(CONFIG_SIM_AJOYSTICK),y)
  CSRCS += up_ajoystick.c
  HOSTCFLAGS += -DCONFIG_SIM_AJOYSTICK=1
  HOSTSRCS += up_x11eventloop.c
else ifeq ($(CONFIG_SIM_BUTTONS),y)
  HOSTCFLAGS += -DCONFIG_SIM_BUTTONS=1
  HOSTSRCS += up_x11eventloop.c
endif
endif

ifeq ($(CONFIG_SIM_IOEXPANDER),y)
  CSRCS += up_ioexpander.c
endif

ifeq ($(CONFIG_SIM_SPIFLASH),y)
  CSRCS += up_spiflash.c
endif

ifeq ($(CONFIG_SIM_QSPIFLASH),y)
  CSRCS += up_qspiflash.c
endif

ifeq ($(CONFIG_FS_FAT),y)
  CSRCS += up_blockdevice.c up_deviceimage.c
  STDLIBS += -lz
endif

ifeq ($(CONFIG_ARCH_ROMGETC),y)
  CSRCS += up_romgetc.c
endif

ifeq ($(CONFIG_SIM_MOTOR_FOC),y)
CSRCS += up_foc.c
endif

ifeq ($(CONFIG_SIM_NETDEV_TAP),y)
  CSRCS += up_netdriver.c
  HOSTCFLAGS += -DNETDEV_BUFSIZE=$(CONFIG_NET_ETH_PKTSIZE)
ifneq ($(CONFIG_WINDOWS_CYGWIN),y)
  HOSTSRCS += up_tapdev.c
ifeq ($(CONFIG_SIM_NET_BRIDGE),y)
  HOSTCFLAGS += -DCONFIG_SIM_NET_BRIDGE
  HOSTCFLAGS += -DCONFIG_SIM_NET_BRIDGE_DEVICE=\"$(CONFIG_SIM_NET_BRIDGE_DEVICE)\"
endif
ifeq ($(CONFIG_SIM_NET_HOST_ROUTE),y)
  HOSTCFLAGS += -DCONFIG_SIM_NET_HOST_ROUTE
endif
else # CONFIG_WINDOWS_CYGWIN != y
  HOSTSRCS += up_wpcap.c
  STDLIBS = /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a
endif # CONFIG_WINDOWS_CYGWIN != y
else ifeq ($(CONFIG_SIM_NETDEV_VPNKIT),y)
  CSRCS += up_netdriver.c
  HOSTCFLAGS += -DCONFIG_SIM_NETDEV_VPNKIT_PATH=\"$(CONFIG_SIM_NETDEV_VPNKIT_PATH)\"
  HOSTSRCS += up_vpnkit.c
  VPATH += :sim/vpnkit
  HOSTSRCS += protocol.c negotiate.c
endif

ifeq ($(CONFIG_SIM_HCISOCKET),y)
  HOSTSRCS += up_hcisocket_host.c
  CSRCS += up_hcisocket.c
endif

ifeq ($(CONFIG_I2C_RESET),y)
  HOSTCFLAGS += -DCONFIG_I2C_RESET=1
endif

ifeq ($(CONFIG_SIM_I2CBUS_LINUX),y)
  HOSTSRCS += up_i2cbuslinux.c
endif

ifeq ($(CONFIG_RPTUN),y)
  CSRCS += up_rptun.c
endif

ifeq ($(CONFIG_SIM_SOUND_ALSA),y)
  CSRCS += up_alsa.c
  STDLIBS += -lasound
endif

ifeq ($(CONFIG_FS_HOSTFS),y)
ifneq ($(CONFIG_FS_HOSTFS_RPMSG),y)
  HOSTSRCS += up_hostfs.c
  HOSTCFLAGS += -DCONFIG_NAME_MAX=$(CONFIG_NAME_MAX)

up_hostfs.c: hostfs.h

hostfs.h: $(TOPDIR)/include/nuttx/fs/hostfs.h
	@echo "CP:  $<"
	$(Q) cp $< $@
endif
endif

COBJS = $(CSRCS:.c=$(OBJEXT))

NUTTXOBJS = $(AOBJS) $(COBJS)
HOSTOBJS = $(HOSTSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(HOSTSRCS)
OBJS = $(AOBJS) $(COBJS) $(HOSTOBJS)

# Override in Make.defs if linker is not 'ld'

ifneq ($(CONFIG_HOST_MACOS),y)
  ARCHSCRIPT += -T nuttx.ld
  LDSTARTGROUP ?= -Wl,--start-group
  LDENDGROUP ?= -Wl,--end-group
endif

# Determine which NuttX libraries will need to be linked in
# Most are provided by LINKLIBS on the MAKE command line

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
	$(Q) $(MAKE) -C board libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

# A partially linked object containing only NuttX code (no interface to host OS)
# Change the names of most symbols that conflict with libc symbols.
# Generate the final NuttX binary by linking the host-specific objects with the NuttX
# specific objects (with munged names)

# C++ global objects are constructed before main get executed, but it isn't a good
# point for simulator because NuttX doesn't finish the kernel initialization yet.
# So we have to skip the standard facilities and do the construction by ourself.
# But how to achieve the goal?
# 1.Command linker generate the default script(-verbose)
# 2.Replace __init_array_start/__init_array_end with _sinit/_einit
# 3.Append __init_array_start = .; __init_array_end = .;
# Step 2 let nxtask_startup find objects need to construct
# Step 3 cheat the host there is no object to construct
# Note: the destructor can be fixed in the same way.

nuttx-names.dat: nuttx-names.in
	$(call PREPROCESS, nuttx-names.in, nuttx-names.dat)

nuttx$(EXEEXT): libarch$(LIBEXT) board/libboard$(LIBEXT) $(LINKOBJS) $(HOSTOBJS) nuttx-names.dat
	$(Q) echo "LD:  nuttx$(EXEEXT)"
	$(Q) $(LD) -r $(LDLINKFLAGS) $(RELPATHS) $(EXTRA_LIBPATHS) -o nuttx.rel $(REQUIREDOBJS) $(LDSTARTGROUP) $(RELLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
	$(Q) $(OBJCOPY) --redefine-syms=nuttx-names.dat nuttx.rel
	$(Q) $(CC) $(CCLINKFLAGS) -Wl,-verbose 2>&1 | \
	     sed -e '/====/,/====/!d;//d' -e 's/__executable_start/_stext/g' -e 's/__init_array_start/_sinit/g' \
	         -e 's/__init_array_end/_einit/g' -e 's/__fini_array_start/_sfini/g' -e 's/__fini_array_end/_efini/g' >nuttx.ld
	$(Q) echo "__init_array_start = .; __init_array_end = .; __fini_array_start = .; __fini_array_end = .;" >>nuttx.ld
	$(if $(CONFIG_HAVE_CXX),\
	$(Q) "$(CXX)" $(CCLINKFLAGS) $(LIBPATHS) $(ARCHSCRIPT) -o $(TOPDIR)/$@ nuttx.rel $(HOSTOBJS) $(STDLIBS),\
	$(Q) "$(CC)" $(CCLINKFLAGS) $(LIBPATHS) $(ARCHSCRIPT) -o $(TOPDIR)/$@ nuttx.rel $(HOSTOBJS) $(STDLIBS))
	$(Q) $(NM) $(TOPDIR)/$@ | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		sort > $(TOPDIR)/System.map

# This is part of the top-level export target

export_startup: board/libboard$(LIBEXT) up_head.o $(HOSTOBJS) nuttx-names.dat
	cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup
	cp nuttx-names.dat ${EXPORT_DIR}/libs
	echo main NXmain >> ${EXPORT_DIR}/libs/nuttx-names.dat

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(HOSTSRCS:.c=.ddh)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board depend ; \
	fi
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

context::

clean_context::

clean:
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board clean ; \
	fi
	$(call DELFILE, nuttx.ld)
	$(call DELFILE, nuttx.rel)
	$(call DELFILE, nuttx-names.dat)
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board distclean ; \
	fi
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, hostfs.h)

-include Make.dep
