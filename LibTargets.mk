############################################################################
# LibTargets.mk
#
#   Copyright (C) 2007-2012, 2014 Gregory Nutt. All rights reserved.
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

# Archive targets.  The target build sequence will first create a series of
# libraries, one per configured source file directory.  The final NuttX
# execution will then be built from those libraries.  The following targets
# build those libraries.
#
# Possible kernel-mode builds

libc$(DELIM)libkc$(LIBEXT): context
	$(Q) $(MAKE) -C libc TOPDIR="$(TOPDIR)" libkc$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libkc$(LIBEXT): libc$(DELIM)libkc$(LIBEXT)
	$(Q) install libc$(DELIM)libkc$(LIBEXT) lib$(DELIM)libkc$(LIBEXT)

libnx$(DELIM)libknx$(LIBEXT): context
	$(Q) $(MAKE) -C libnx TOPDIR="$(TOPDIR)" libknx$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libknx$(LIBEXT): libnx$(DELIM)libknx$(LIBEXT)
	$(Q) install libnx$(DELIM)libknx$(LIBEXT) lib$(DELIM)libknx$(LIBEXT)

mm$(DELIM)libkmm$(LIBEXT): context
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libkmm$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libkmm$(LIBEXT): mm$(DELIM)libkmm$(LIBEXT)
	$(Q) install mm$(DELIM)libkmm$(LIBEXT) lib$(DELIM)libkmm$(LIBEXT)

$(ARCH_SRC)$(DELIM)libkarch$(LIBEXT): context
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libkarch$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libkarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libkarch$(LIBEXT)
	$(Q) install $(ARCH_SRC)$(DELIM)libkarch$(LIBEXT) lib$(DELIM)libkarch$(LIBEXT)


sched$(DELIM)libsched$(LIBEXT): context
	$(Q) $(MAKE) -C sched TOPDIR="$(TOPDIR)" libsched$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libsched$(LIBEXT): sched$(DELIM)libsched$(LIBEXT)
	$(Q) install sched$(DELIM)libsched$(LIBEXT) lib$(DELIM)libsched$(LIBEXT)

net$(DELIM)libnet$(LIBEXT): context
	$(Q) $(MAKE) -C net TOPDIR="$(TOPDIR)" libnet$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libnet$(LIBEXT): net$(DELIM)libnet$(LIBEXT)
	$(Q) install net$(DELIM)libnet$(LIBEXT) lib$(DELIM)libnet$(LIBEXT)

configs$(DELIM)libconfigs$(LIBEXT): context
	$(Q) $(MAKE) -C configs TOPDIR="$(TOPDIR)" libconfigs$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libconfigs$(LIBEXT): configs$(DELIM)libconfigs$(LIBEXT)
	$(Q) install configs$(DELIM)libconfigs$(LIBEXT) lib$(DELIM)libconfigs$(LIBEXT)

crypto$(DELIM)libcrypto$(LIBEXT): context
	$(Q) $(MAKE) -C crypto TOPDIR="$(TOPDIR)" libcrypto$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libcrypto$(LIBEXT): crypto$(DELIM)libcrypto$(LIBEXT)
	$(Q) install crypto$(DELIM)libcrypto$(LIBEXT) lib$(DELIM)libcrypto$(LIBEXT)

fs$(DELIM)libfs$(LIBEXT): context
	$(Q) $(MAKE) -C fs TOPDIR="$(TOPDIR)" libfs$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libfs$(LIBEXT): fs$(DELIM)libfs$(LIBEXT)
	$(Q) install fs$(DELIM)libfs$(LIBEXT) lib$(DELIM)libfs$(LIBEXT)

drivers$(DELIM)libdrivers$(LIBEXT): context
	$(Q) $(MAKE) -C drivers TOPDIR="$(TOPDIR)" libdrivers$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libdrivers$(LIBEXT): drivers$(DELIM)libdrivers$(LIBEXT)
	$(Q) install drivers$(DELIM)libdrivers$(LIBEXT) lib$(DELIM)libdrivers$(LIBEXT)

binfmt$(DELIM)libbinfmt$(LIBEXT): context
	$(Q) $(MAKE) -C binfmt TOPDIR="$(TOPDIR)" libbinfmt$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libbinfmt$(LIBEXT): binfmt$(DELIM)libbinfmt$(LIBEXT)
	$(Q) install binfmt$(DELIM)libbinfmt$(LIBEXT) lib$(DELIM)libbinfmt$(LIBEXT)

graphics$(DELIM)libgraphics$(LIBEXT): context
	$(Q) $(MAKE) -C graphics TOPDIR="$(TOPDIR)" libgraphics$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libgraphics$(LIBEXT): graphics$(DELIM)libgraphics$(LIBEXT)
	$(Q) install graphics$(DELIM)libgraphics$(LIBEXT) lib$(DELIM)libgraphics$(LIBEXT)

audio$(DELIM)libaudio$(LIBEXT): context
	$(Q) $(MAKE) -C audio TOPDIR="$(TOPDIR)" libaudio$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libaudio$(LIBEXT): audio$(DELIM)libaudio$(LIBEXT)
	$(Q) install audio$(DELIM)libaudio$(LIBEXT) lib$(DELIM)libaudio$(LIBEXT)

# Special case

syscall$(DELIM)libstubs$(LIBEXT): context
	$(Q) $(MAKE) -C syscall TOPDIR="$(TOPDIR)" libstubs$(LIBEXT) # KERNEL=y EXTRADEFINES=$(KDEFINE)

lib$(DELIM)libstubs$(LIBEXT): syscall$(DELIM)libstubs$(LIBEXT)
	$(Q) install syscall$(DELIM)libstubs$(LIBEXT) lib$(DELIM)libstubs$(LIBEXT)

# Possible user-mode builds

libc$(DELIM)libuc$(LIBEXT): context
	$(Q) $(MAKE) -C libc TOPDIR="$(TOPDIR)" libuc$(LIBEXT) KERNEL=n

lib$(DELIM)libuc$(LIBEXT): libc$(DELIM)libuc$(LIBEXT)
	$(Q) install libc$(DELIM)libuc$(LIBEXT) lib$(DELIM)libuc$(LIBEXT)

libnx$(DELIM)libunx$(LIBEXT): context
	$(Q) $(MAKE) -C libnx TOPDIR="$(TOPDIR)" libunx$(LIBEXT) KERNEL=n

lib$(DELIM)libunx$(LIBEXT): libnx$(DELIM)libunx$(LIBEXT)
	$(Q) install libnx$(DELIM)libunx$(LIBEXT) lib$(DELIM)libunx$(LIBEXT)

mm$(DELIM)libumm$(LIBEXT): context
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libumm$(LIBEXT) KERNEL=n

lib$(DELIM)libumm$(LIBEXT): mm$(DELIM)libumm$(LIBEXT)
	$(Q) install mm$(DELIM)libumm$(LIBEXT) lib$(DELIM)libumm$(LIBEXT)

$(ARCH_SRC)$(DELIM)libuarch$(LIBEXT): context
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libuarch$(LIBEXT) KERNEL=n

lib$(DELIM)libuarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libuarch$(LIBEXT)
	$(Q) install $(ARCH_SRC)$(DELIM)libuarch$(LIBEXT) lib$(DELIM)libuarch$(LIBEXT)

libxx$(DELIM)libcxx$(LIBEXT): context
	$(Q) $(MAKE) -C libxx TOPDIR="$(TOPDIR)" libcxx$(LIBEXT) KERNEL=n

lib$(DELIM)libcxx$(LIBEXT): libxx$(DELIM)libcxx$(LIBEXT)
	$(Q) install libxx$(DELIM)libcxx$(LIBEXT) lib$(DELIM)libcxx$(LIBEXT)

$(APPDIR)$(DELIM)libapps$(LIBEXT): context
	$(Q) $(MAKE) -C $(APPDIR) TOPDIR="$(TOPDIR)" libapps$(LIBEXT) KERNEL=n

lib$(DELIM)libapps$(LIBEXT): $(APPDIR)$(DELIM)libapps$(LIBEXT)
	$(Q) install $(APPDIR)$(DELIM)libapps$(LIBEXT) lib$(DELIM)libapps$(LIBEXT)

syscall$(DELIM)libproxies$(LIBEXT): context
	$(Q) $(MAKE) -C syscall TOPDIR="$(TOPDIR)" libproxies$(LIBEXT) KERNEL=n

lib$(DELIM)libproxies$(LIBEXT): syscall$(DELIM)libproxies$(LIBEXT)
	$(Q) install syscall$(DELIM)libproxies$(LIBEXT) lib$(DELIM)libproxies$(LIBEXT)

# Possible non-kernel builds

libc$(DELIM)libc$(LIBEXT): context
	$(Q) $(MAKE) -C libc TOPDIR="$(TOPDIR)" libc$(LIBEXT)

lib$(DELIM)libc$(LIBEXT): libc$(DELIM)libc$(LIBEXT)
	$(Q) install libc$(DELIM)libc$(LIBEXT) lib$(DELIM)libc$(LIBEXT)

libnx$(DELIM)libnx$(LIBEXT): context
	$(Q) $(MAKE) -C libnx TOPDIR="$(TOPDIR)" libnx$(LIBEXT)

lib$(DELIM)libnx$(LIBEXT): libnx$(DELIM)libnx$(LIBEXT)
	$(Q) install libnx$(DELIM)libnx$(LIBEXT) lib$(DELIM)libnx$(LIBEXT)

mm$(DELIM)libmm$(LIBEXT): context
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libmm$(LIBEXT)

lib$(DELIM)libmm$(LIBEXT): mm$(DELIM)libmm$(LIBEXT)
	$(Q) install mm$(DELIM)libmm$(LIBEXT) lib$(DELIM)libmm$(LIBEXT)

$(ARCH_SRC)$(DELIM)libarch$(LIBEXT): context
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libarch$(LIBEXT)

lib$(DELIM)libarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libarch$(LIBEXT)
	$(Q) install $(ARCH_SRC)$(DELIM)libarch$(LIBEXT) lib$(DELIM)libarch$(LIBEXT)
