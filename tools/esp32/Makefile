############################################################################
# tools/esp32/Makefile
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

ESP_PATH  = ${HOME}/esp

IDF_PATH  = ${ESP_PATH}/esp-idf
QEMU_PATH = ${ESP_PATH}/qemu

QEMU_BIN = ${QEMU_PATH}/xtensa-softmmu/qemu-system-xtensa
QEMU_LINK = ${IDF_PATH}/tools/qemu-system-xtensa

BOOTLOADER_BIN = ${IDF_PATH}/hello_world/build/bootloader/bootloader.bin
PARTITION_TABLE_BIN = ${IDF_PATH}/hello_world/build/partition_table/partition-table.bin

all: ${IDF_PATH} ${BOOTLOADER_BIN} ${PARTITION_TABLE_BIN} ${QEMU_BIN} ${QEMU_LINK}
default: all
.PHONY: clean esp-idf-hello-world

# Add CFLAGS=-g on the make command line to build debug versions

CFLAGS = -O2 -Wall

${IDF_PATH}:
	mkdir -p ${ESP_PATH}
	echo "WARNING:this directory can be automatically removed" > ${ESP_PATH}/README
	cd ${ESP_PATH} && \
		ls esp-idf || git clone -b v4.1 --recursive https://github.com/espressif/esp-idf.git && \
	cd ${IDF_PATH} && \
		./install.sh

${IDF_PATH}/hello_world:
	cp -r ${IDF_PATH}/examples/get-started/hello_world ${IDF_PATH}/hello_world

${IDF_PATH}/hello_world/build: SHELL:=/usr/bin/env bash
${IDF_PATH}/hello_world/build: ${IDF_PATH}/hello_world
	cd ${IDF_PATH} && \
		. export.sh && \
		cd hello_world && \
		idf.py menuconfig && \
		echo "CONFIG_ESPTOOLPY_FLASHMODE_DOUT=y" >> sdkconfig && \
		idf.py build

${BOOTLOADER_BIN}: ${IDF_PATH}/hello_world/build

${PARTITION_TABLE_BIN}: ${IDF_PATH}/hello_world/build

${QEMU_BIN}:
	cd ${ESP_PATH} && \
		git clone https://github.com/espressif/qemu && \
		cd qemu && \
		./configure --target-list=xtensa-softmmu \
		    --enable-debug --enable-sanitizers \
		    --disable-strip --disable-user \
		    --disable-capstone --disable-vnc \
		    --disable-sdl --disable-gtk && \
		make -j8

${QEMU_LINK}: ${QEMU_BIN}
	ln -srf ${QEMU_BIN} ${QEMU_LINK}

clean:
	@rm -rf ${HOME}/.espressif
	@grep "WARNING:this directory can be automatically removed" ${ESP_PATH}/README > /dev/null 2> /dev/null && rm -rf ${ESP_PATH}; echo
