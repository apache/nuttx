############################################################################
# tools/esp32c3/Makefile
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

BOOTLOADER_BIN = ${IDF_PATH}/hello_world/build/bootloader/bootloader.bin
PARTITION_TABLE_BIN = ${IDF_PATH}/hello_world/build/partition_table/partition-table.bin

all: ${IDF_PATH} ${BOOTLOADER_BIN} ${PARTITION_TABLE_BIN}
default: all
.PHONY: clean esp-idf-hello-world

# Add CFLAGS=-g on the make command line to build debug versions

CFLAGS = -O2 -Wall

${IDF_PATH}:
	@mkdir -p ${ESP_PATH}
	@echo "WARNING:this directory can be automatically removed" > ${ESP_PATH}/README
	@cd ${ESP_PATH} && \
		ls esp-idf 1>/dev/null 2>&1 || git clone -b v4.3-beta1 --recursive https://github.com/espressif/esp-idf.git && \
		cd ${IDF_PATH} && \
		./install.sh

${IDF_PATH}/hello_world:
	@cp -r ${IDF_PATH}/examples/get-started/hello_world ${IDF_PATH}/hello_world

${IDF_PATH}/hello_world/build: SHELL:=/usr/bin/env bash
${IDF_PATH}/hello_world/build: ${IDF_PATH}/hello_world
	@cd ${IDF_PATH} && \
		. export.sh && \
		cd hello_world && \
		idf.py set-target esp32c3 && \
		idf.py menuconfig && \
		echo "CONFIG_ESPTOOLPY_FLASHMODE_DOUT=y" >> sdkconfig && \
		idf.py build

${BOOTLOADER_BIN}: ${IDF_PATH}/hello_world/build

${PARTITION_TABLE_BIN}: ${IDF_PATH}/hello_world/build

clean:
	@rm -rf ${HOME}/.espressif
	@grep "WARNING:this directory can be automatically removed" ${ESP_PATH}/README 1>/dev/null 2>&1 && rm -rf ${ESP_PATH}; echo
