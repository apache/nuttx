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

INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)
INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)chip
INCLUDES += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)sched

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

NUTTX = $(call CONVERT_PATH,$(TOPDIR)$(DELIM)nuttx$(EXEEXT))

# Determine which objects are required in the link.The
# up_head object normally draws in all that is needed, but
# there are a fews that must be included because they
# are called only from the host OS-specific logic(HOSTOBJS)

LINKOBJS = up_head$(OBJEXT)
REQUIREDOBJS = $(LINKOBJS)

ifeq ($(CONFIG_HOST_X86_64),y)
ifeq ($(CONFIG_SIM_M32),y)
  ASRCS += up_vfork_x86.S
else
  ASRCS += up_vfork_x86_64.S
endif
else ifeq ($(CONFIG_HOST_X86),y)
  ASRCS += up_vfork_x86.S
else ifeq ($(CONFIG_HOST_ARM),y)
  ASRCS += up_vfork_arm.S
else ifeq ($(CONFIG_HOST_ARM64),y)
  ASRCS += up_vfork_arm64.S
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS  = up_initialize.c up_idle.c up_interruptcontext.c up_initialstate.c
CSRCS += up_createstack.c up_usestack.c up_releasestack.c up_stackframe.c
CSRCS += up_unblocktask.c up_blocktask.c up_releasepending.c
CSRCS += up_exit.c up_schedulesigaction.c
CSRCS += up_heap.c up_uart.c up_assert.c up_nputs.c
CSRCS += up_copyfullstate.c
CSRCS += up_sigdeliver.c

ifeq ($(CONFIG_SCHED_BACKTRACE),y)
CSRCS += up_backtrace.c
endif

ifeq ($(CONFIG_ARCH_HAVE_VFORK),y)
ifeq ($(CONFIG_SCHED_WAITPID),y)
CSRCS += up_vfork.c
endif
endif

VPATH = :sim
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  VPATH += :sim/win
else
  VPATH += :sim/posix
endif
DEPPATH = $(patsubst %,--dep-path %,$(subst :, ,$(VPATH)))

CFLAGS += -fvisibility=default
HOSTCFLAGS = $(ARCHWARNINGS) $(ARCHOPTIMIZATION) \
   $(ARCHCFLAGS) $(HOSTINCLUDES) $(EXTRAFLAGS) -D__SIM__ -pipe \
   -fvisibility=default

HOSTCFLAGS += ${INCDIR_PREFIX}$(ARCH_SRCDIR)
HOSTCFLAGS += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)chip
ifeq ($(CONFIG_HOST_MACOS),y)
  HOSTCFLAGS += -Wno-deprecated-declarations
endif

HOSTSRCS = up_hostirq.c up_hostmemory.c up_hosttime.c up_simuart.c up_hostmisc.c
STDLIBS += -lpthread
ifeq ($(CONFIG_HOST_MACOS),y)
ifeq ($(CONFIG_HAVE_CXXINITIALIZE),y)
  # Note: up_macho_init.c is not in CSRCS because it's picky about
  # the place in the object list for linking. Namely, its constructor
  # should be the first one in the executable.
  HEADSRC = up_macho_init.c
endif
else
  STDLIBS += -lrt
endif

ifneq ($(CONFIG_LIBM),y)
  STDLIBS += -lm
endif

ifeq ($(CONFIG_SIM_UBSAN),y)
  STDLIBS += -lubsan
endif

ifeq ($(CONFIG_STACK_COLORATION),y)
  CSRCS += up_checkstack.c
endif

ifeq ($(CONFIG_SPINLOCK),y)
  HOSTSRCS += up_testset.c
endif

ifeq ($(CONFIG_SMP),y)
  CSRCS += up_smpsignal.c up_cpuidlestack.c
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
else ifeq ($(CONFIG_SIM_FRAMEBUFFER),y)
  CSRCS += up_framebuffer.c
endif

ifeq ($(CONFIG_SIM_X11FB),y)
  HOSTSRCS += up_x11framebuffer.c
  STDLIBS += -lX11 -lXext
ifeq ($(CONFIG_SIM_TOUCHSCREEN),y)
  CSRCS += up_touchscreen.c
  HOSTSRCS += up_x11eventloop.c
else ifeq ($(CONFIG_SIM_AJOYSTICK),y)
  CSRCS += up_ajoystick.c
  HOSTSRCS += up_x11eventloop.c
else ifeq ($(CONFIG_SIM_BUTTONS),y)
  HOSTSRCS += up_x11eventloop.c
endif

ifeq ($(CONFIG_SIM_KEYBOARD),y)
  CSRCS += up_keyboard.c
endif
endif

ifeq ($(CONFIG_FS_FAT),y)
  CSRCS += up_blockdevice.c up_deviceimage.c
  STDLIBS += -lz
endif

ifeq ($(CONFIG_ARCH_ROMGETC),y)
  CSRCS += up_romgetc.c
endif

ifeq ($(CONFIG_SIM_NETDEV_TAP),y)
  CSRCS += up_netdriver.c
ifneq ($(CONFIG_WINDOWS_CYGWIN),y)
  HOSTSRCS += up_tapdev.c
else # CONFIG_WINDOWS_CYGWIN != y
  HOSTSRCS += up_wpcap.c
  STDLIBS = /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a
endif # CONFIG_WINDOWS_CYGWIN != y
else ifeq ($(CONFIG_SIM_NETDEV_VPNKIT),y)
  CSRCS += up_netdriver.c
  HOSTSRCS += up_vpnkit.c
  VPATH += :sim/vpnkit
  HOSTSRCS += protocol.c negotiate.c
endif

ifeq ($(CONFIG_SIM_NETUSRSOCK),y)
  HOSTSRCS += up_usrsock_host.c
  CSRCS += up_usrsock.c
endif

ifeq ($(CONFIG_SIM_HCISOCKET),y)
  HOSTSRCS += up_hcisocket_host.c
  CSRCS += up_hcisocket.c
endif

ifeq ($(CONFIG_SIM_I2CBUS_LINUX),y)
  HOSTSRCS += up_i2cbuslinux.c
endif

ifeq ($(CONFIG_SIM_SPI_LINUX),y)
  HOSTSRCS += up_spilinux.c
endif

ifeq ($(CONFIG_RPTUN),y)
  CSRCS += up_rptun.c
endif

ifeq ($(CONFIG_SIM_SOUND_ALSA),y)
  CSRCS += up_alsa.c
  STDLIBS += -lasound
endif

ifeq ($(CONFIG_SIM_HOSTFS),y)
  HOSTSRCS += up_hostfs.c

hostfs.h: $(TOPDIR)/include/nuttx/fs/hostfs.h
	@echo "CP:  $<"
	$(Q) cp $< $@

up_hostfs.c: hostfs.h
endif

COBJS = $(CSRCS:.c=$(OBJEXT))

NUTTXOBJS = $(AOBJS) $(COBJS)
HOSTOBJS = $(HOSTSRCS:.c=$(OBJEXT))
HEADOBJ = $(HEADSRC:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(HOSTSRCS)
OBJS = $(AOBJS) $(COBJS) $(HOSTOBJS)

$(foreach lib,$(notdir $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))), \
  $(foreach elib,$(EXTRA_LIBS), \
    $(if $(filter $(notdir $(elib)),$(lib)), \
      $(eval NAMEFULL_LIBS+=$(elib)), \
      $(if $(filter $(notdir $(elib)),$(patsubst lib%$(LIBEXT),-l%,$(lib))), \
        $(eval NAMESPEC_LIBS+=$(elib)) \
       ) \
     ) \
   ) \
 )

EXTRA_LIBS := $(filter-out $(NAMEFULL_LIBS) $(NAMESPEC_LIBS),$(EXTRA_LIBS))
EXTRA_LIBS += $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))

# Override in Make.defs if linker is not 'ld'

ifneq ($(CONFIG_HOST_MACOS),y)
  ARCHSCRIPT += nuttx.ld
  LDSTARTGROUP ?= --start-group
  LDENDGROUP ?= --end-group
endif

# Additional rules for system call wrapper

ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
  EXTRALINKCMDS += @$(TOPDIR)/syscall/syscall_wraps.ldcmd
endif

LDFLAGS += $(addprefix -T,$(call CONVERT_PATH,$(ARCHSCRIPT)))

# Determine which NuttX libraries will need to be linked in
# Most are provided by LINKLIBS on the MAKE command line

RELLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
RELPATHS += -L"$(TOPDIR)/staging"

# Add the board-specific library and directory

RELPATHS += -L board
RELLIBS += -lboard

# Make targets begin here

all: up_head$(OBJEXT) libarch$(LIBEXT)

.PHONY: export_startup clean distclean cleanrel depend

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(HOSTOBJS) $(HEADOBJ): %$(OBJEXT): %.c
	$(Q) echo "CC:  $<"
	$(Q) "$(CC)" -c $(HOSTCFLAGS) $< -o $@

# The architecture-specific library

libarch$(LIBEXT): $(NUTTXOBJS)
	$(call ARCHIVE, $@, $^)

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
# 2.Move input sections out of .init_array/.fini_array into new .sinit/.einit
#   output sections
# 3.Replace __init_array_start/__init_array_end with _sinit/_einit
# 4.Append __init_array_start = .; __init_array_end = .;
# Step 2 cheat the host there is no object to construct (glibc >= 2.34)
# Step 3 let nxtask_startup find objects need to construct
# Step 4 cheat the host there is no object to construct (glibc < 2.34)
# Note: the destructor can be fixed in the same way.

nuttx-names.dat: nuttx-names.in
	$(call PREPROCESS, nuttx-names.in, nuttx-names.dat)

define LINK_ALLSYMS
	$(Q)$(TOPDIR)/tools/mkallsyms.py $(NUTTX) allsyms.tmp
	$(Q) $(call COMPILE, -x c allsyms.tmp, allsyms$(OBJEXT))
	$(if $(CONFIG_HAVE_CXX),\
	$(Q) "$(CXX)" $(CFLAGS) $(LDFLAGS) -o $(NUTTX) \
	      $(HEADOBJ) nuttx.rel $(HOSTOBJS) $(STDLIBS) allsyms$(OBJEXT),\
	$(Q) "$(CC)" $(CFLAGS) $(LDFLAGS) -o $(NUTTX) \
	      $(HEADOBJ) nuttx.rel $(HOSTOBJS) $(STDLIBS) allsyms$(OBJEXT))
	$(Q) $(call DELFILE, allsyms.tmp allsyms$(OBJEXT))
endef

# Note: Use objcopy for Linux because for some reasons visibility=hidden
# stuff doesn't work there as we expect.
# Note: _stext stuff is for CONFIG_CXX_INITIALIZE_SINIT, which in not
# necessary for macOS.
nuttx$(EXEEXT): libarch$(LIBEXT) board/libboard$(LIBEXT) $(HEADOBJ) $(LINKOBJS) $(HOSTOBJS) nuttx-names.dat
	$(Q) echo "LD:  nuttx$(EXEEXT)"
	$(Q) $(LD) -r $(LDLINKFLAGS) $(RELPATHS) $(EXTRA_LIBPATHS) $(EXTRALINKCMDS) \
	     -o nuttx.rel $(REQUIREDOBJS) $(LDSTARTGROUP) $(RELLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
ifneq ($(CONFIG_HOST_MACOS),y)
	$(Q) $(OBJCOPY) --redefine-syms=nuttx-names.dat nuttx.rel
	$(Q) $(CC) $(CFLAGS) -Wl,-verbose 2>&1 | \
	     sed -e '/====/,/====/!d;//d' -e 's/__executable_start/_stext/g' \
	         -e 's/^\(\s\+\)\(\.init_array\)/\1\2 : { }\n\1.sinit/g' \
	         -e 's/^\(\s\+\)\(\.fini_array\)/\1\2 : { }\n\1.einit/g' \
	         -e 's/__init_array_start/_sinit/g' -e 's/__init_array_end/_einit/g' \
	         -e 's/__fini_array_start/_sfini/g' -e 's/__fini_array_end/_efini/g' >nuttx.ld
	$(Q) echo "__init_array_start = .; __init_array_end = .; __fini_array_start = .; __fini_array_end = .;" >>nuttx.ld
endif
ifneq ($(CONFIG_ALLSYMS),y)
	$(if $(CONFIG_HAVE_CXX),\
	$(Q) "$(CXX)" $(CFLAGS) $(LDFLAGS) -o $(TOPDIR)/$@ $(HEADOBJ) nuttx.rel $(HOSTOBJS) $(STDLIBS),\
	$(Q) "$(CC)" $(CFLAGS) $(LDFLAGS) -o $(TOPDIR)/$@ $(HEADOBJ) nuttx.rel $(HOSTOBJS) $(STDLIBS))
else
	$(Q) # Link and generate default table
	$(Q) $(if $(wildcard $(shell echo $(NUTTX))),,$(call LINK_ALLSYMS, $@))
	$(Q) # Extract all symbols
	$(Q) $(call LINK_ALLSYMS, $^)
	$(Q) # Extract again since the table offset may changed
	$(Q) $(call LINK_ALLSYMS, $^)
endif
	$(Q) $(NM) $(TOPDIR)/$@ | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		sort > $(TOPDIR)/System.map

# This is part of the top-level export target

export_startup: up_head.o $(HOSTOBJS) nuttx-names.dat
	cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup
	cp nuttx-names.dat ${EXPORT_DIR}/libs

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(HOSTSRCS:.c=.ddh)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

config.h: $(TOPDIR)/include/nuttx/config.h
	@echo "CP:  $<"
	$(Q) cp $< $@

.depend: Makefile config.h $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board depend ; \
	fi
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

context::

clean:
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board clean ; \
	fi
	$(call DELFILE, nuttx.ld)
	$(call DELFILE, nuttx.rel)
	$(call DELFILE, nuttx-names.dat)
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean:: clean
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board distclean ; \
	fi
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, config.h)
	$(call DELFILE, hostfs.h)

-include Make.dep
