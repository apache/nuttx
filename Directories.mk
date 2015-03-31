############################################################################
# Directories.mk
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

# All add-on directories.
#
# NUTTX_ADDONS is the list of directories built into the NuttX kernel.
# USER_ADDONS is the list of directories that will be built into the user
#   application

NUTTX_ADDONS :=
USER_ADDONS :=

# In the protected build, the applications in the apps/ directory will be
# into the userspace; in the flat build, the applications will b built into
# the kernel space.  But in the kernel build, the applications will not be
# built at all by this Makefile.

ifeq ($(CONFIG_BUILD_PROTECTED),y)
USER_ADDONS += $(APPDIR)
else
ifneq ($(CONFIG_BUILD_KERNEL),y)
NUTTX_ADDONS += $(APPDIR)
endif
endif

# Lists of build directories.
#
# FSDIRS depend on file descriptor support; NONFSDIRS do not (except for parts
#   of FSDIRS).  We will exclude FSDIRS from the build if file descriptor
#   support is disabled
# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# USERDIRS - When NuttX is build is a monolithic kernel, this provides the
#   list of directories that must be built
# OTHERDIRS - These are directories that are not built but probably should
#   be cleaned to prevent garbage from collecting in them when changing
#   configurations.

NONFSDIRS = sched configs $(ARCH_SRC) $(NUTTX_ADDONS)
FSDIRS = fs drivers binfmt
CONTEXTDIRS = $(APPDIR)
USERDIRS =
OTHERDIRS = lib

ifeq ($(CONFIG_BUILD_PROTECTED),y)

USERDIRS += libc mm $(USER_ADDONS)
ifeq ($(CONFIG_HAVE_CXX),y)
USERDIRS += libxx
endif

else
ifeq ($(CONFIG_BUILD_KERNEL),y)

USERDIRS += libc mm
ifeq ($(CONFIG_HAVE_CXX),y)
USERDIRS += libxx
endif

else

NONFSDIRS += libc mm
OTHERDIRS += $(USER_ADDONS)
ifeq ($(CONFIG_HAVE_CXX),y)
NONFSDIRS += libxx
else
OTHERDIRS += libxx
endif

endif
endif

ifeq ($(CONFIG_LIB_SYSCALL),y)
NONFSDIRS += syscall
CONTEXTDIRS += syscall
USERDIRS += syscall
else
OTHERDIRS += syscall
endif

ifeq ($(CONFIG_NX),y)
NONFSDIRS += graphics libnx
CONTEXTDIRS += graphics libnx
else
OTHERDIRS += graphics libnx
endif

ifeq ($(CONFIG_AUDIO),y)
NONFSDIRS += audio
else
OTHERDIRS += audio
endif

# CLEANDIRS are the directories that will clean in.  These are
#   all directories that we know about.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED or
#   CONFIG_BUILD_KERNEL), then this holds only the directories containing
#   kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED),
#   then this holds only the directories containing user files. If
#   CONFIG_BUILD_KERNEL is selected, then applications are not build at all.

CLEANDIRS   = $(NONFSDIRS) $(FSDIRS) $(USERDIRS) $(OTHERDIRS)
KERNDEPDIRS = $(NONFSDIRS)
USERDEPDIRS = $(USERDIRS)

# Add file system directories to KERNDEPDIRS (they are already in CLEANDIRS)

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifeq ($(CONFIG_NET),y)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
KERNDEPDIRS += fs
endif
KERNDEPDIRS += drivers
endif
else
KERNDEPDIRS += $(FSDIRS)
endif

# Add networking directories to KERNDEPDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
KERNDEPDIRS += net
endif
CLEANDIRS += net

ifeq ($(CONFIG_CRYPTO),y)
KERNDEPDIRS += crypto
endif
CLEANDIRS += crypto
