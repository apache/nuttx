############################################################################
# configs/ea3131/locked/Makefile
#
#   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Make.defs

ifeq ($(WINTOOL),y)
  # Windows-native toolchains
  PASS1_LIBPATHS	= ${shell for path in $(LINKLIBS); do dir=`dirname $(TOPDIR)/$$path`;echo "-L\"`cygpath -w $$dir`\"";done}
  PASS1_LDSCRIPT	= -T "${shell cygpath -w $(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/locked/ld-locked.inc}"
else
  # Linux/Cygwin-native toolchain 
  PASS1_LIBPATHS	= $(addprefix -L$(TOPDIR)/,$(dir $(LINKLIBS)))
  PASS1_LDSCRIPT	= -T$(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/locked/ld-locked.inc
endif

PASS1_LDFLAGS	= -r $(PASS1_LDSCRIPT)
PASS1_LDLIBS	= $(patsubst lib%,-l%,$(basename $(notdir $(LINKLIBS))))
PASS1_LIBGCC	= "${shell $(CC) -print-libgcc-file-name}"

all: locked.r

ld-locked.inc: mklocked.sh $(TOPDIR)/.config
	@echo "MK:  ld-locked.inc"
	@./mklocked.sh "$(TOPDIR)"

locked.r: ld-locked.inc $(OBJS)
	@echo "LD:  locked.r"
	@$(LD) -o $@ $(PASS1_LDFLAGS) $(PASS1_LIBPATHS) --start-group $(PASS1_LDLIBS) --end-group -u os_start $(PASS1_LIBGCC) 

.depend:

depend: .depend

clean:
	@rm -f locked.r *~ .*.swp

distclean: clean
	@rm -f ld-locked.inc

