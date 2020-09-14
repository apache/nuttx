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
	$(Q) $(MAKE) -C libs$(DELIM)libc libkc$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libkc$(LIBEXT): libs$(DELIM)libc$(DELIM)libkc$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libnx$(DELIM)libknx$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C libs$(DELIM)libnx libknx$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libknx$(LIBEXT): libs$(DELIM)libnx$(DELIM)libknx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

mm$(DELIM)libkmm$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C mm libkmm$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libkmm$(LIBEXT): mm$(DELIM)libkmm$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

$(ARCH_SRC)$(DELIM)libkarch$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C $(ARCH_SRC) libkarch$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libkarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libkarch$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

pass1$(DELIM)libpass1$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C pass1 libpass1$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libpass1$(LIBEXT): pass1$(DELIM)libpass1$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

sched$(DELIM)libsched$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C sched libsched$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libsched$(LIBEXT): sched$(DELIM)libsched$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

net$(DELIM)libnet$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C net libnet$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libnet$(LIBEXT): net$(DELIM)libnet$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

boards$(DELIM)libboards$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C boards libboards$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libboards$(LIBEXT): boards$(DELIM)libboards$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

crypto$(DELIM)libcrypto$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C crypto libcrypto$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libcrypto$(LIBEXT): crypto$(DELIM)libcrypto$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

fs$(DELIM)libfs$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C fs libfs$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libfs$(LIBEXT): fs$(DELIM)libfs$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

drivers$(DELIM)libdrivers$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C drivers libdrivers$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libdrivers$(LIBEXT): drivers$(DELIM)libdrivers$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

external$(DELIM)libexternal$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C external libexternal$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libexternal$(LIBEXT): external$(DELIM)libexternal$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

binfmt$(DELIM)libbinfmt$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C binfmt libbinfmt$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libbinfmt$(LIBEXT): binfmt$(DELIM)libbinfmt$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

graphics$(DELIM)libgraphics$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C graphics libgraphics$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libgraphics$(LIBEXT): graphics$(DELIM)libgraphics$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

audio$(DELIM)libaudio$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C audio libaudio$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libaudio$(LIBEXT): audio$(DELIM)libaudio$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

video$(DELIM)libvideo$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C video libvideo$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libvideo$(LIBEXT): video$(DELIM)libvideo$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

wireless$(DELIM)libwireless$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C wireless libwireless$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libwireless$(LIBEXT): wireless$(DELIM)libwireless$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

openamp$(DELIM)libopenamp$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C openamp libopenamp$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libopenamp$(LIBEXT): openamp$(DELIM)libopenamp$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

syscall$(DELIM)libstubs$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C syscall libstubs$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libstubs$(LIBEXT): syscall$(DELIM)libstubs$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

syscall$(DELIM)libwraps$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C syscall libwraps$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"

staging$(DELIM)libwraps$(LIBEXT): syscall$(DELIM)libwraps$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

# Special case

ifeq ($(CONFIG_BUILD_FLAT),y)
$(ARCH_SRC)$(DELIM)libarch$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C $(ARCH_SRC) libarch$(LIBEXT) EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)"
else
$(ARCH_SRC)$(DELIM)libarch$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C $(ARCH_SRC) libarch$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"
endif

staging$(DELIM)libarch$(LIBEXT): $(ARCH_SRC)$(DELIM)libarch$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

# Possible user-mode builds

ifeq ($(CONFIG_BUILD_FLAT),y)
libs$(DELIM)libc$(DELIM)libc$(LIBEXT): pass2dep
else
libs$(DELIM)libc$(DELIM)libc$(LIBEXT): pass1dep
endif
	$(Q) $(MAKE) -C libs$(DELIM)libc libc$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libc$(LIBEXT): libs$(DELIM)libc$(DELIM)libc$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

ifeq ($(CONFIG_BUILD_FLAT),y)
libs$(DELIM)libnx$(DELIM)libnx$(LIBEXT): pass2dep
else
libs$(DELIM)libnx$(DELIM)libnx$(LIBEXT): pass1dep
endif
	$(Q) $(MAKE) -C libs$(DELIM)libnx libnx$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libnx$(LIBEXT): libs$(DELIM)libnx$(DELIM)libnx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

ifeq ($(CONFIG_BUILD_FLAT),y)
mm$(DELIM)libmm$(LIBEXT): pass2dep
else
mm$(DELIM)libmm$(LIBEXT): pass1dep
endif
	$(Q) $(MAKE) -C mm libmm$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libmm$(LIBEXT): mm$(DELIM)libmm$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libxx$(DELIM)libxx$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C libs$(DELIM)libxx libxx$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libxx$(LIBEXT): libs$(DELIM)libxx$(DELIM)libxx$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

libs$(DELIM)libdsp$(DELIM)libdsp$(LIBEXT): pass2dep
	$(Q) $(MAKE) -C libs$(DELIM)libdsp libdsp$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libdsp$(LIBEXT): libs$(DELIM)libdsp$(DELIM)libdsp$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

ifeq ($(CONFIG_BUILD_FLAT),y)
$(APPDIR)$(DELIM)libapps$(LIBEXT): pass2dep
else
$(APPDIR)$(DELIM)libapps$(LIBEXT): pass1dep
endif
	$(Q) $(MAKE) -C $(APPDIR) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libapps$(LIBEXT): $(APPDIR)$(DELIM)libapps$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)

syscall$(DELIM)libproxies$(LIBEXT): pass1dep
	$(Q) $(MAKE) -C syscall libproxies$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

staging$(DELIM)libproxies$(LIBEXT): syscall$(DELIM)libproxies$(LIBEXT)
	$(Q) $(call INSTALL_LIB,$<,$@)
