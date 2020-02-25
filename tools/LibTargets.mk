############################################################################
# tools/LibTargets.mk
#
#   Copyright (C) 2007-2012, 2014, 2019 Gregory Nutt. All rights reserved.
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

libs$(DELIM)libc$(DELIM)libkc$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C libs$(DELIM)libc TOPDIR="$(TOPDIR)" libkc$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libkc$(LIBEXT): libs$(DELIM)libc$(DELIM)libkc$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libnx$(DELIM)libknx$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C libs$(DELIM)libnx TOPDIR="$(TOPDIR)" libknx$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libknx$(LIBEXT): libs$(DELIM)libnx$(DELIM)libknx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

mm$(DELIM)libkmm$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libkmm$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libkmm$(LIBEXT): mm$(DELIM)libkmm$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

$(ARCH_SRC)$(DELIM)libkarch$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libkarch$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libkarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libkarch$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

pass1$(DELIM)libpass1$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C pass1 TOPDIR="$(TOPDIR)" libpass1$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libpass1$(LIBEXT): pass1$(DELIM)libpass1$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

sched$(DELIM)libsched$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C sched TOPDIR="$(TOPDIR)" libsched$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libsched$(LIBEXT): sched$(DELIM)libsched$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

net$(DELIM)libnet$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C net TOPDIR="$(TOPDIR)" libnet$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libnet$(LIBEXT): net$(DELIM)libnet$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

boards$(DELIM)libboards$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C boards TOPDIR="$(TOPDIR)" libboards$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libboards$(LIBEXT): boards$(DELIM)libboards$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

crypto$(DELIM)libcrypto$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C crypto TOPDIR="$(TOPDIR)" libcrypto$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libcrypto$(LIBEXT): crypto$(DELIM)libcrypto$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

fs$(DELIM)libfs$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C fs TOPDIR="$(TOPDIR)" libfs$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libfs$(LIBEXT): fs$(DELIM)libfs$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

drivers$(DELIM)libdrivers$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C drivers TOPDIR="$(TOPDIR)" libdrivers$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libdrivers$(LIBEXT): drivers$(DELIM)libdrivers$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

binfmt$(DELIM)libbinfmt$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C binfmt TOPDIR="$(TOPDIR)" libbinfmt$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libbinfmt$(LIBEXT): binfmt$(DELIM)libbinfmt$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

graphics$(DELIM)libgraphics$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C graphics TOPDIR="$(TOPDIR)" libgraphics$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libgraphics$(LIBEXT): graphics$(DELIM)libgraphics$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

audio$(DELIM)libaudio$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C audio TOPDIR="$(TOPDIR)" libaudio$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libaudio$(LIBEXT): audio$(DELIM)libaudio$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

video$(DELIM)libvideo$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C video TOPDIR="$(TOPDIR)" libvideo$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libvideo$(LIBEXT): video$(DELIM)libvideo$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

wireless$(DELIM)libwireless$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C wireless TOPDIR="$(TOPDIR)" libwireless$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libwireless$(LIBEXT): wireless$(DELIM)libwireless$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

$(ARCH_SRC)$(DELIM)libarch$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libarch$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libarch$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libdsp$(DELIM)libdsp$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C libs$(DELIM)libdsp TOPDIR="$(TOPDIR)" libdsp$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libdsp$(LIBEXT): libs$(DELIM)libdsp$(DELIM)libdsp$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

openamp$(DELIM)libopenamp$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C openamp TOPDIR="$(TOPDIR)" libopenamp$(LIBEXT) KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libopenamp$(LIBEXT): openamp$(DELIM)libopenamp$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

# Special case

syscall$(DELIM)libstubs$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C syscall TOPDIR="$(TOPDIR)" libstubs$(LIBEXT) # KERNEL=y EXTRADEFINES=$(KDEFINE)

staging$(DELIM)libstubs$(LIBEXT): syscall$(DELIM)libstubs$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

# Possible user-mode builds

libs$(DELIM)libc$(DELIM)libuc$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libc TOPDIR="$(TOPDIR)" libuc$(LIBEXT) KERNEL=n

staging$(DELIM)libuc$(LIBEXT): libs$(DELIM)libc$(DELIM)libuc$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libnx$(DELIM)libunx$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libnx TOPDIR="$(TOPDIR)" libunx$(LIBEXT) KERNEL=n

staging$(DELIM)libunx$(LIBEXT): libs$(DELIM)libnx$(DELIM)libunx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

mm$(DELIM)libumm$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libumm$(LIBEXT) KERNEL=n

staging$(DELIM)libumm$(LIBEXT): mm$(DELIM)libumm$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

$(ARCH_SRC)$(DELIM)libuarch$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libuarch$(LIBEXT) KERNEL=n

staging$(DELIM)libuarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libuarch$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libxx$(DELIM)libxx$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libxx TOPDIR="$(TOPDIR)" libxx$(LIBEXT) KERNEL=n

staging$(DELIM)libxx$(LIBEXT): libs$(DELIM)libxx$(DELIM)libxx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

$(APPDIR)$(DELIM)libapps$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C $(APPDIR) TOPDIR="$(TOPDIR)" KERNEL=n

staging$(DELIM)libapps$(LIBEXT): $(APPDIR)$(DELIM)libapps$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

syscall$(DELIM)libproxies$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C syscall TOPDIR="$(TOPDIR)" libproxies$(LIBEXT) KERNEL=n

staging$(DELIM)libproxies$(LIBEXT): syscall$(DELIM)libproxies$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

# Possible non-kernel builds

libs$(DELIM)libc$(DELIM)libc$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libc TOPDIR="$(TOPDIR)" libc$(LIBEXT)

staging$(DELIM)libc$(LIBEXT): libs$(DELIM)libc$(DELIM)libc$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libnx$(DELIM)libnx$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libnx TOPDIR="$(TOPDIR)" libnx$(LIBEXT)

staging$(DELIM)libnx$(LIBEXT): libs$(DELIM)libnx$(DELIM)libnx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

mm$(DELIM)libmm$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C mm TOPDIR="$(TOPDIR)" libmm$(LIBEXT)

staging$(DELIM)libmm$(LIBEXT): mm$(DELIM)libmm$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)
