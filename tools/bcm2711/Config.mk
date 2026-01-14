############################################################################
# tools/bcm2711/Config.mk
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

# These are the macros that will be used in the NuttX make system to compile
# and assemble source files and to insert the resulting object files into an
# archive. These replace the default definitions at tools/Config.mk

# POSTBUILD -- Perform post build operations

# Post build logic for the Raspberry Pi 4B

ifeq ($(CONFIG_ARCH_BOARD_RASPBERRYPI_4B),y)

CONFIG_TXT = config.txt

define POSTBUILD
	$(Q)echo "Generating $(CONFIG_TXT)";
	$(Q)echo "kernel=nuttx.bin" > $(CONFIG_TXT);
	$(Q)echo "arm_64bit=1" >> $(CONFIG_TXT);
	$(Q)echo "core_freq_min=500" >> $(CONFIG_TXT);
	$(if $(CONFIG_RPI4B_DEBUG_BOOT),$(Q)echo "uart_2ndstage=1" >> $(CONFIG_TXT);)
endef

endif
