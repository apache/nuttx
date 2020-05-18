############################################################################
# tools/zds/Makefile
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

TOPDIR ?= $(CURDIR)/..
include $(TOPDIR)/Make.defs

# Define HOSTCC on the make command line if it differs from these defaults
# Define HOSTCFLAGS with -g on the make command line to build debug versions

HOSTOS = ${shell uname -o 2>/dev/null || uname -s 2>/dev/null || echo "Other"}

ifeq ($(HOSTOS),MinGW)

# In the Windows native environment, the MinGW GCC compiler is used

HOSTCC ?= mingw32-gcc.exe
HOSTCFLAGS ?= -O2 -Wall -Wstrict-prototypes -Wshadow -I.
HOSTCFLAGS += -DHOST_NATIVE=1

else

# Must be Cygwin or MYS2 on Windows (Can't be Linux, macOS, BSD, ..)
#
# GCC or clang is assumed in all other POSIX environments.
# strtok_r is used in some tools, but does not seem to be available in
# the MinGW environment.

HOSTCC ?= cc
HOSTCFLAGS ?= -O2 -Wall -Wstrict-prototypes -Wshadow -I.
HOSTCFLAGS += -DHAVE_STRTOK_C=1 -DHOST_CYGWIN=1

endif

# Targets

all: zdsar.exe
default: all
.PHONY: clean

# Add CFLAGS=-g on the make command line to build debug versions

CFLAGS = -O2 -Wall -Wstrict-prototypes -Wshadow -I.
CFLAGS += -DHAVE_STRTOK_C=1

# zdsar - Wrapper for the ZDS-II librarian

zdsar.exe: zdsar.c
	$(Q) $(HOSTCC) $(HOSTCFLAGS) -o zdsar.exe zdsar.c

clean:
	@rm -f *.o *.a *.dSYM *~ .*.swp
	@rm -f zdsar zdsar.exe
