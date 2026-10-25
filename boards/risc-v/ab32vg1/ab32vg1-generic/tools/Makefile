############################################################################
# boards/risc-v/ab32vg1/ab32vg1-generic/tools/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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

# Host tool to load and flash NuttX over the AB32VG1 mask ROM UART loader.
# The flash programming stub and the stage-1 boot loader run on the
# AB32VG1; they are cross-compiled and embedded in the host tool.
#
#   make                 build ab32vg1_dl
#   make CROSSDEV=...    RISC-V toolchain prefix (default riscv-none-elf-)

CC       ?= cc
CROSSDEV ?= riscv-none-elf-
HOSTCFLAGS ?= -O2 -Wall -Wextra

TARGET_CFLAGS = -march=rv32imc -mabi=ilp32 -Os -ffreestanding -nostdlib \
                -fno-builtin -Wall -Wno-array-bounds -I target \
                -Wl,--no-warn-rwx-segments

COMMON = target/spiflash.c target/util.c target/target.h

all: ab32vg1_dl

target/stub.elf: target/stub_start.S target/stub.c target/stub.h \
                 target/stub.ld $(COMMON)
	$(CROSSDEV)gcc $(TARGET_CFLAGS) -T target/stub.ld -o $@ \
	  target/stub_start.S target/stub.c target/spiflash.c target/util.c

target/boot.elf: target/boot_start.S target/boot.c target/boot.ld $(COMMON)
	$(CROSSDEV)gcc $(TARGET_CFLAGS) -T target/boot.ld -o $@ \
	  target/boot_start.S target/boot.c target/spiflash.c target/util.c

%.bin: %.elf
	$(CROSSDEV)objcopy -O binary $< $@

# Embed a binary as a C array (portable: od + sed)

define BIN2C
	@echo "/* Generated from $(1), do not edit */" > $(3)
	@echo "static const unsigned char $(2)[] =" >> $(3)
	@echo "{" >> $(3)
	@od -An -v -tx1 $(1) | sed -e 's/  */ /g' -e 's/^ //' -e '/^$$/d' \
	  -e 's/\([0-9a-f][0-9a-f]\)/0x\1,/g' -e 's/^/  /' >> $(3)
	@echo "};" >> $(3)
endef

ab32vg1_stub.h: target/stub.bin
	$(call BIN2C,$<,g_ab32vg1_stub,$@)

ab32vg1_boot.h: target/boot.bin
	$(call BIN2C,$<,g_ab32vg1_boot,$@)

ab32vg1_dl: ab32vg1_dl.c ab32vg1_stub.h ab32vg1_boot.h target/stub.h
	$(CC) $(HOSTCFLAGS) -o $@ ab32vg1_dl.c

clean:
	rm -f ab32vg1_dl ab32vg1_stub.h ab32vg1_boot.h
	rm -f target/*.elf target/*.bin

.PHONY: all clean
