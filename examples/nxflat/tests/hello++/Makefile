############################################################################
# examples/nxflat/tests/hello/Makefile
#
#   Copyright (C) 2009 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

-include $(TOPDIR)/.config	# Current configuration
-include $(TOPDIR)/Make.defs	# Basic make info
include ../Make.defs		# NXFLAT make info

BIN1			= hello++1
BIN2			= hello++2
BIN3			= hello++3
#BIN4			= hello++4

SRCS1			= $(BIN1).c
OBJS1			= $(SRCS1:.c=.o)

SRCS2			= $(BIN2).c
OBJS2			= $(SRCS2:.c=.o)

SRCS3			= $(BIN3).c
OBJS3			= $(SRCS3:.c=.o)

#SRCS4			= $(BIN4).c
#OBJS4			= $(SRCS4:.c=.o)

CXXOBJS			= $(OBJS1) $(OBJS2) $(OBJS3) # $(OBJS4)

LIBSTDC_STUBS_DIR	= $(TOPDIR)/libxx
LIBSTDC_STUBS_LIB	= $(LIBSTDC_STUBS_DIR)/liblibxx.a

all: $(BIN1) $(BIN2) $(BIN3) # $(BIN4)

$(CXXOBJS): %.o: %.cpp
	$(NXFLATCXX) -c $(NXFLATCXXFLAGS) $< -o $@

# This contains libstdc++ stubs to that you can build C++ code
# without actually have libstdc++

$(LIBSTDC_STUBS_LIB):
	$(MAKE) -C $(LIBSTDC_STUBS_DIR) TOPDIR=$(TOPDIR)

# BIN1 and BIN2 link just like C code because they contain no
# static constructors.  BIN1 is equivalent to a C hello world;
# BIN2 contains a class that implements hello world, but it is
# not statically initialized.

$(BIN1): $(OBJS1)
	$(NXFLATLD) -o $@ $^

$(BIN2): $(OBJS2) $(LIBSTDC_STUBS_LIB)
	$(NXFLATLD) -o $@ $^

# BIN3 and BIN4 require that we include --cxx in the xflat-ld command.
# This will instruct xflat-ld that we want it to put togethe the correct
# startup files to handle the C++ static initializers.
#
# BIN3 is equivalent to BIN2 except that is uses static initializers

$(BIN3): $(OBJS3) $(LIBSTDC_STUBS_LIB)
	$(XFLATLD) --cxx -o $@ $^

# BIN4 is similar to BIN3 except that it uses the streams code from libstdc++
#
# NOTE:  libstdc++ is not available for XFLAT as of this writing
#
#$(BIN4): $(OBJS4) $(LIBSTDC_STUBS_LIB)
#	$(XFLATLD) --cxx -o $@ $^

clean: 
	rm -f $(BIN1) $(BIN2) $(BIN3) $(BIN4) *.o *~ core

user_install: $(BIN1) $(BIN2) $(BIN3) # $(BIN4)
	install -D $(BIN1) $(ROMFS_DIR)/$(BIN1)
	install -D $(BIN2) $(ROMFS_DIR)/$(BIN2)
	install -D $(BIN3) $(ROMFS_DIR)/$(BIN3)
#	install -D $(BIN4) $(ROMFS_DIR)/$(BIN4)







