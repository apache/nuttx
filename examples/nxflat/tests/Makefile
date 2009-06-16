############################################################################
# examples/nxflat/Makefile
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

TESTDIRS	:= errno hello hello++ longjmp mutex pthread signal task

define TEST_template
$(1)_$(2):
	$(MAKE) -C $(1) $(3)
endef

$(foreach TEST, $(TESTDIRS), $(eval $(call TEST_template,$(TEST),build, all)))
$(foreach TEST, $(TESTDIRS), $(eval $(call TEST_template,$(TEST),clean,clean)))
$(foreach TEST, $(TESTDIRS), $(eval $(call TEST_template,$(TEST),install,install)))

all: build
.PHONY: all build clean user_install root_install

build: $(foreach TEST, $(TESTDIRS), $(TEST)_build)

clean: $(foreach TEST, $(TESTDIRS), $(TEST)_clean)

install: $(foreach TEST, $(TESTDIRS), $(TEST)_install)

