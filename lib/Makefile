############################################################################
# lib/Makefile
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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
###########################################################################

-include $(TOPDIR)/Make.defs
include stdio/Make.defs
include string/Make.defs

ASRCS = 
AOBJS = $(ASRCS:.S=$(OBJEXT))

MISC_SRCS = lib_init.c lib_filesem.c
ifneq ($(CONFIG_NFILE_STREAMS),0)
MISC_SRCS += lib_streamsem.c
endif

CTYPE_SRCS = 

STDLIB_SRCS = lib_abs.c lib_imaxabs.c lib_labs.c lib_llabs.c lib_rand.c lib_qsort.c

MATH_SRCS = lib_rint.c lib_fixedmath.c lib_b16sin.c lib_b16cos.c

UNISTD_SRCS = lib_getopt.c lib_getoptargp.c lib_getoptindp.c lib_getoptoptp.c
ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_DISABLE_ENVIRON),y)
UNISTD_SRCS +=  lib_chdir.c lib_getcwd.c
endif
endif

TIME_SRCS = lib_mktime.c lib_gmtime.c lib_gmtimer.c lib_strftime.c \
		  lib_calendar2utc.c lib_daysbeforemonth.c lib_isleapyear.c

NET_SRCS = lib_htons.c lib_htonl.c lib_inetntoa.c lib_etherntoa.c

LIBGEN_SRCS = lib_basename.c lib_dirname.c

REGEX_SRCS = lib_match.c

CRC_SRCS = lib_crc32.c

SQ_SRCS = sq_addlast.c sq_addfirst.c sq_addafter.c \
		  sq_rem.c sq_remlast.c sq_remfirst.c sq_remafter.c

DQ_SRCS = dq_addlast.c dq_addfirst.c dq_addafter.c dq_addbefore.c \
		  dq_rem.c dq_remlast.c dq_remfirst.c

DBG_SRCS = lib_dbg.c lib_dumpbuffer.c

CSRCS = $(MISC_SRCS) $(STRING_SRCS) $(CTYPE_SRCS) $(STDIO_SRCS) $(STDLIB_SRCS) \
		  $(MATH_SRCS) $(UNISTD_SRCS) $(TIME_SRCS) $(NET_SRCS) $(LIBGEN_SRCS) \
		  $(REGEX_SRCS) $(CRC_SRCS) $(SQ_SRCS) $(DQ_SRCS) $(DBG_SRCS) 
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ROOTDEPPATH	= --dep-path .
STDIODEPPATH = --dep-path stdio
STRINGDEPPATH = --dep-path string
VPATH = stdio:string

BIN = liblib$(LIBEXT)

all: $(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(STRINGDEPPATH) $(STDIODEPPATH) \
	  $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
