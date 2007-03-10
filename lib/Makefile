############################################################
# Makefile
#
#   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
############################################################

-include $(TOPDIR)/Make.defs

MKDEP 		= $(TOPDIR)/tools/mkdeps.sh

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))

MISC_SRCS	= lib_init.c lib_streamsem.c lib_filesem.c
STRING_SRCS	= lib_memset.c lib_memcpy.c lib_memcmp.c lib_memmove.c \
		  lib_strcpy.c lib_strncpy.c lib_strcmp.c \
		  lib_strlen.c lib_strdup.c lib_strtol.c lib_strchr.c \
		  lib_strtok.c lib_strtokr.c
CTYPE_SRCS	= 
STDIO_SRCS	= lib_fopen.c lib_fclose.c \
		  lib_fread.c lib_libfread.c lib_fgetc.c lib_fgets.c lib_gets.c \
		  lib_fwrite.c lib_libfwrite.c lib_fflush.c \
		  lib_fputc.c lib_puts.c lib_fputs.c \
		  lib_ungetc.c \
		  lib_printf.c lib_vprintf.c lib_fprintf.c  lib_rawprintf.c lib_lowprintf.c \
		  lib_vfprintf.c lib_sprintf.c lib_libsprintf.c lib_vsprintf.c \
		  lib_libvsprintf.c lib_stdstream.c lib_memstream.c \
		  lib_rawstream.c lib_lowstream.c lib_nullstream.c \
		  lib_sscanf.c
STDLIB_SRCS	= lib_getenv.c lib_rand.c
MATH_SRCS	= lib_rint.c
SQ_SRCS		= sq_addlast.c sq_addfirst.c sq_addafter.c \
		  sq_rem.c sq_remlast.c sq_remfirst.c sq_remafter.c
DQ_SRCS		= dq_addlast.c dq_addfirst.c dq_addafter.c dq_addbefore.c \
		  dq_rem.c dq_remlast.c dq_remfirst.c

CSRCS		= $(MISC_SRCS) $(STRING_SRCS) $(CTYPE_SRCS) $(STDIO_SRCS) \
		  $(STDLIB_SRCS) $(MATH_SRCS) $(SQ_SRCS) $(DQ_SRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= liblib$(LIBEXT)

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(CC) -c $(CFLAGS) -D__ASSEMBLY__ $< -o $@

$(COBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BIN):	$(OBJS)
	( for obj in $(OBJS) ; do \
		$(AR) $@ $${obj} || \
			{ echo "$(AR) $@ $obj FAILED!" ; exit 1 ; } ; \
	done ; )

.depend: Makefile $(SRCS)
	$(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	touch $@

depend: .depend

clean:
	rm -f $(BIN) *.o *.rel *.asm *.lst *.sym *.adb *~

distclean: clean
	rm -f Make.dep .depend

-include Make.dep
