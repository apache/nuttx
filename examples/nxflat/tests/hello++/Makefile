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
ALL_BIN			= $(BIN1) $(BIN2) $(BIN3) $(BIN4)

R1SRCS1			= $(BIN1).c
R1OBJS1			= $(R1SRCS1:.c=.o)
R2SRC1			= $(BIN1)-thunk.S
R2OBJ1			= $(R2SRC1:.S=.o)

R1SRCS2			= $(BIN2).c
R1OBJS2			= $(R1SRCS2:.c=.o)
R2SRC2			= $(BIN2)-thunk.S
R2OBJ2			= $(R2SRC2:.S=.o)

R1SRCS3			= $(BIN3).c
R1OBJS3			= $(R1SRCS3:.c=.o)
R2SRC3			= $(BIN3)-thunk.S
R2OBJ3			= $(R2SRC3:.S=.o)

#R1SRCS4		= $(BIN4).c
#R1OBJS4			= $(R1SRCS4:.c=.o)
#R2SRC4			= $(BIN4)-thunk.S
#R2OBJ4			= $(R2SRC4:.S=.o)

DERIVED			= $(R2SRC1) $(R2SRC2) $(R2SRC3) $(R2SRC4)

R1CXXOBJS		= $(R1OBJS1) $(R1OBJS2) $(R1OBJS3) # $(R1OBJS4)
R2AOBJS			= $(R2OBJ1) $(R2OBJ2) $(R2OBJ3) # $(R2OBJ4)

LIBSTDC_STUBS_DIR	= $(TOPDIR)/libxx
LIBSTDC_STUBS_LIB	= $(LIBSTDC_STUBS_DIR)/liblibxx.a

all: $(BIN1) $(BIN2) $(BIN3) # $(BIN4)

$(R1CXXOBJS): %.o: %.cpp
	$(NXFLATCXX) -c $(NXFLATCXXFLAGS) $< -o $@

$(R2AOBJS): %.o: %.S
	$(NXFLATCC) -c $(NXFLATCFLAGS) $< -o $@

# This contains libstdc++ stubs to that you can build C++ code
# without actually having libstdc++

$(LIBSTDC_STUBS_LIB):
	$(MAKE) -C $(LIBSTDC_STUBS_DIR) TOPDIR=$(TOPDIR)

# BIN1 and BIN2 link just like C code because they contain no
# static constructors.  BIN1 is equivalent to a C hello world;
# BIN2 contains a class that implements hello world, but it is
# not statically initialized.

$(BIN1).r1: $(R1OBJS1)
	$(NXFLATLD) -r $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC1): $(BIN1).r1
	$(MKNXFLAT) -o $@ $^

$(BIN1).r2: $(R2OBJ1)
	$(NXFLATLD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS1) $(R2OBJ1)

$(BIN1): $(BIN1).r2
	touch $(BIN1) # For now

$(BIN2).r1: $(R1OBJS2) $(LIBSTDC_STUBS_LIB)
	$(NXFLATLD) -r $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC2): $(BIN2).r1
	$(MKNXFLAT) -o $@ $^

$(BIN2).r2: $(R2OBJ2)
	$(NXFLATLD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS2) $(R2OBJ2)

(BIN2): $(BIN2).r2
	touch $(BIN2) # For now

# BIN3 and BIN4 require that we include --cxx in the xflat-ld command.
# This will instruct xflat-ld that we want it to put togethe the correct
# startup files to handle the C++ static initializers.
#
# BIN3 is equivalent to BIN2 except that is uses static initializers

$(BIN3).r1: $(R1OBJS3) $(LIBSTDC_STUBS_LIB)
	$(NXFLATLD) -r $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC3): $(BIN3).r1
	$(MKNXFLAT) -o $@ $^

$(BIN3).r2: $(R2OBJ3)
	$(NXFLATLD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS3) $(R2OBJ3)

$(BIN3): $(BIN3).r2
	touch $(BIN3) # For now

# BIN4 is similar to BIN3 except that it uses the streams code from libstdc++
#
# NOTE:  libstdc++ is not available for XFLAT as of this writing
#
#$(BIN4).r1: $(R1OBJS4) $(LIBSTDC_STUBS_LIB)
#	$(NXFLATLD) -r $(NXFLATLDFLAGS1) -o $@ $^
#
#$(R2SRC4): $(BIN4).r1
#	$(MKNXFLAT) -o $@ $^
#
#$(BIN4).r2: $(R2OBJ4)
#	$(NXFLATLD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS4) $(R2OBJ4)
#
#$(BIN4): $(BIN4).r2
#	touch $(BIN4) # For now

clean: 
	rm -f $(ALL_BIN) $(DERIVED) *.o *.r1 *.r2 *~ .*.swp core

install: $(ALL_BIN)
	install -D $(BIN1) $(ROMFS_DIR)/$(BIN1)
	install -D $(BIN2) $(ROMFS_DIR)/$(BIN2)
	install -D $(BIN3) $(ROMFS_DIR)/$(BIN3)
#	install -D $(BIN4) $(ROMFS_DIR)/$(BIN4)







