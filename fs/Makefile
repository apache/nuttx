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

MKDEP	= $(TOPDIR)/tools/mkdeps.sh

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= fs_open.c fs_close.c fs_read.c fs_write.c fs_ioctl.c fs_dup.c \
		  fs_opendir.c fs_closedir.c fs_readdir.c fs_readdirr.c \
		  fs_seekdir.c fs_telldir.c fs_rewinddir.c fs_fsync.c fs_files.c \
		  fs_inode.c fs_inodefind.c fs_inodereserve.c \
		  fs_inoderemove.c fs_registerdriver.c fs_unregisterdriver.c \
		  fs_inodeaddref.c fs_inoderelease.c
ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
CSRCS		+= fs_registerblockdriver.c fs_unregisterblockdriver.c \
		  fs_mount.c fs_umount.c fs_unlink.c fs_mkdir.c fs_rmdir.c \
		  fs_rename.c
ifeq ($(CONFIG_FS_FAT),y)
CSRCS		+= fs_fat32.c fs_fat32util.c
endif
endif
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libfs$(LIBEXT)

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
