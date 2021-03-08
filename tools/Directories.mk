############################################################################
# tools/Directories.mk
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

# Lists of build directories.
#
# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# CLEANDIRS are the directories that the clean target will executed in.
#   These are all directories that we know about.
# CCLEANDIRS are directories that the clean_context target will execute in.
#   The clean_context target "undoes" the actions of the context target.
#   Only directories known to require cleaning are included.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED or
#   CONFIG_BUILD_KERNEL), then this holds only the directories containing
#   kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED),
#   then this holds only the directories containing user files. If
#   CONFIG_BUILD_KERNEL is selected, then applications are not build at all.

CLEANDIRS :=
CCLEANDIRS := boards $(APPDIR) graphics
KERNDEPDIRS :=
USERDEPDIRS :=

# In the protected build, the applications in the apps/ directory will be
# into the userspace; in the flat build, the applications will be built into
# the kernel space.  But in the kernel build, the applications will not be
# built at all by this Makefile.

ifeq ($(CONFIG_BUILD_PROTECTED),y)
USERDEPDIRS += $(APPDIR)
else ifneq ($(CONFIG_BUILD_KERNEL),y)
KERNDEPDIRS += $(APPDIR)
else
CLEANDIRS += $(APPDIR)
endif

KERNDEPDIRS += sched drivers boards $(ARCH_SRC)
KERNDEPDIRS += fs binfmt

ifeq ($(EXTERNALDIR),external)
  KERNDEPDIRS += external
endif

CONTEXTDIRS = boards drivers fs $(APPDIR) $(ARCH_SRC)
CLEANDIRS += pass1

ifeq ($(CONFIG_BUILD_FLAT),y)

KERNDEPDIRS += libs$(DELIM)libc mm
ifeq ($(CONFIG_HAVE_CXX),y)
KERNDEPDIRS += libs$(DELIM)libxx
else
CLEANDIRS += libs$(DELIM)libxx
endif

else

USERDEPDIRS += libs$(DELIM)libc mm
ifeq ($(CONFIG_HAVE_CXX),y)
USERDEPDIRS += libs$(DELIM)libxx
else
CLEANDIRS += libs$(DELIM)libxx
endif

endif

ifeq ($(CONFIG_LIB_SYSCALL),y)
CONTEXTDIRS += syscall
USERDEPDIRS += syscall
else
ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
CONTEXTDIRS += syscall
USERDEPDIRS += syscall
else
CLEANDIRS += syscall
endif
endif

CONTEXTDIRS += libs$(DELIM)libc
ifeq ($(CONFIG_HAVE_CXX),y)
CONTEXTDIRS += libs$(DELIM)libxx
endif

ifeq ($(CONFIG_NX),y)
KERNDEPDIRS += graphics
CONTEXTDIRS += graphics
else
CLEANDIRS += graphics
endif

ifeq ($(CONFIG_NXFONTS),y)
ifeq ($(CONFIG_BUILD_FLAT),y)
KERNDEPDIRS += libs$(DELIM)libnx
else
USERDEPDIRS += libs$(DELIM)libnx
endif
CONTEXTDIRS += libs$(DELIM)libnx
else
CLEANDIRS += libs$(DELIM)libnx
endif

ifeq ($(CONFIG_AUDIO),y)
KERNDEPDIRS += audio
else
CLEANDIRS += audio
endif

ifeq ($(CONFIG_VIDEO),y)
KERNDEPDIRS += video
else
CLEANDIRS += video
endif

ifeq ($(CONFIG_WIRELESS),y)
KERNDEPDIRS += wireless
else
CLEANDIRS += wireless
endif

ifeq ($(CONFIG_LIBDSP),y)
KERNDEPDIRS += libs$(DELIM)libdsp
else
CLEANDIRS += libs$(DELIM)libdsp
endif

# Add networking directories to KERNDEPDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
KERNDEPDIRS += net
else
CLEANDIRS += net
endif

ifeq ($(CONFIG_CRYPTO),y)
KERNDEPDIRS += crypto
else
CLEANDIRS += crypto
endif

ifeq ($(CONFIG_OPENAMP),y)
KERNDEPDIRS += openamp
CONTEXTDIRS += openamp
else
CLEANDIRS += openamp
endif

ifeq ($(CONFIG_MM_TLSF_MANAGER),y)
KERNDEPDIRS += mm
CONTEXTDIRS += mm
else
CLEANDIRS += mm
endif

CLEANDIRS += $(KERNDEPDIRS) $(USERDEPDIRS)
