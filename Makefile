############################################################################
# Makefile
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

# Check if the system has been configured

ifeq ($(wildcard .config),)
.DEFAULT default:
	@echo "NuttX has not been configured!"
	@echo "To configure the project:"
	@echo "  tools/configure.sh <config>"
	@echo "For a list of available configurations:"
	@echo "  tools/configure.sh -L"
else
include .config

# Build any necessary tools needed early in the build.
# incdir - Is needed immediately by all Make.defs file.

TOPDIR := ${shell echo $(CURDIR) | sed -e 's/ /\\ /g'}
DUMMY  := ${shell $(MAKE) -C tools -f Makefile.host incdir \
          INCDIR="$(TOPDIR)/tools/incdir.sh"}

# Include the correct Makefile for the selected architecture.

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
include tools/Makefile.win
else
include tools/Makefile.unix
endif
endif
