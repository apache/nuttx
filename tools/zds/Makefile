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

# Targets

all: zdsar.exe
default: all
.PHONY: clean

# Add CFLAGS=-g on the make command line to build debug versions

CFLAGS = -O2 -Wall -Wstrict-prototypes -Wshadow
CFLAGS += -DHAVE_STRTOK_C=1

# zdsar - Wrapper for the ZDS-II librarian

zdsar.exe: zdsar.c
	$(Q) $(HOSTCC) $(HOSTCFLAGS) -o zdsar.exe zdsar.c

clean:
	@rm -f *.o *.a *.dSYM *~ .*.swp
	@rm -f zdsar zdsar.exe
