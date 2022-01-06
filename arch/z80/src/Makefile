############################################################################
# arch/z80/src/Makefile
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

# Makefile fragments

include $(TOPDIR)/Make.defs
include chip/Make.defs

# Compiler-Dependent Make:  SDCC, Clang, or ZiLOG ZDS-II

COMPILER = ${shell basename "$(CC)"}
ifeq ($(patsubst %-clang,clang,$(COMPILER)),clang)

    include Makefile.clang

else ifeq ($(COMPILER),sdcc)

  # Check for SDCC native windows build

  ifeq ($(CONFIG_WINDOWS_NATIVE),y)

    # SDCC Windows native build

    include Makefile.sdccw
  else

    # SDCC in a POSIX environment (Linux, macOS, or Cygwin/MSYS)

    include Makefile.sdccl
  endif
else

  # Check for ZDSII native windows build

  ifeq ($(CONFIG_WINDOWS_NATIVE),y)

    # ZDSII Windows native build

    include Makefile.zdsiiw
  else

    # ZDSII in a POSIX environment (Cygwin/MSYS)

    include Makefile.zdsiil
  endif
endif
