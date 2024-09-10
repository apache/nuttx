#!/usr/bin/env bash
############################################################################
# tools/build-globals.sh
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
#
# Script to create modlib_global.S which contains a structure define 
# the API names and addresses we will export for resolving symbols in
# dynamic loaded shared objects. Typically these are libc APIs.

#
# Symbols to ignore within the NuttX libraries
#
FILTER="^lib_low|^FUNCTION|^STUB|^__start|^_vect|^arm_|^arp_|^bch|^binfmt|^blake|^block_|^cdcacm|^chksum|^clock_|^close_|^crypto_|^devif_|^devnull|^devuran|^devzero|^emerg|^epoll_|^elf_|^_dbgR|^dq_|^env_|^file_|^files_|^fs_|^ftl_|^g_|^get_|^group_|^global|^hcom|^i2c_|^inode_|^iob_|^irq_|^kmm_|^lfs_|^lib_|^local_|^mm_|^modlib_|^mpu_|^mq_|^nglobals|^net_|^netdev_|^nx|^pipecommon|^posix_spawn_file|^psock_|^ramlog|^rammap|^readline_|^register_|^sched_|^sockfd|^spawn_|^sq_|^stm32|^symtab_|^syslog_|^syslogstream|^task_|^tcp_|^timer_|^uart_|^ub[12]|^udp_|^umm_|^umount|^unload_|^unregister|^up_|^usb|^usrsock_|^watchdog|^wd_|globalNames$|nglobals$|global_table$|^\.l"

if [ -z "${NM}" ]; then
	NM="nm"
fi

#
# Extract symbols from the runtime
#
SYMS=`cat System.map | awk '{print $3}' | sort | grep -Ev ${FILTER}`
SYM=(${SYMS})
GLOBALS="libs/libc/modlib/modlib_globals.S"

#
# Generate the modlib_xxxx_globals.S file
#
cat >${GLOBALS} <<__EOF__
#ifdef __CYGWIN__
#  define SYMBOL(s) _##s
#  define GLOBAL .global
#  define SECTION .data
	.macro GLOBAL ep
	.global	SYMBOL(\ep)
	.type	SYMBOL(\ep), "object"
	.endm
	.macro SIZE ep
	.endm
#elif defined(__ELF__)
#  define SYMBOL(s) s
#  define SECTION .data
	.macro GLOBAL ep
	.global	SYMBOL(\ep)
	.type	SYMBOL(\ep), "object"
	.endm
	.macro SIZE ep
	.size	SYMBOL(\ep), . - SYMBOL(\ep)
	.endm
#else
#  define SYMBOL(s) _##s
#  define SECTION .section __DATA,__data
	.macro GLOBAL ep
	.private_extern SYMBOL(\ep)
	.globl 	SYMBOL(\ep)
	.endm
	.macro SIZE ep
	.endm
#endif

#if __SIZEOF_POINTER__ == 8
	.macro globalEntry index, ep
	.quad	.l\index
	.quad	\ep
	.endm
# define ALIGN 8
#else
	.macro globalEntry index, ep
	.long	.l\index
	.long	\ep
	.endm
# define ALIGN 4
#endif
#ifdef __ARM_ARCH_ISA_THUMB2
# ifdef __ARM_ARCH_7M__
	.arch armv7e-m
# elif defined ___ARM_ARCH 8
	.arch armv8-m.base
#endif
#ifdef __ARM_ASM_SYNTAX_UNIFIED__
	.syntax	unified
#endif
	.thumb
#endif
	.data
	.align	ALIGN
	GLOBAL	globalNames

SYMBOL(globalNames):
__EOF__

for ((i = 0; i < ${#SYM[@]}; i++))
do
	echo ".l${i}:	.string	\"${SYM[$i]}\"" >>${GLOBALS}
done

cat >>${GLOBALS} <<__EOF__
	SIZE	globalNames

	.align	ALIGN
	GLOBAL	nglobals
SYMBOL(nglobals):
	.word	${#SYM[@]}
	SIZE	nglobals

	.align	ALIGN
	GLOBAL	global_table
SYMBOL(global_table):
__EOF__

for ((i = 0; i < ${#SYM[@]}; i++))
do
	echo "	globalEntry ${i}, ${SYM[$i]}" >>${GLOBALS}
done

cat >>${GLOBALS} <<__EOF__ 
	SIZE	global_table
__EOF__

echo "${#SYM[@]} symbols defined"
