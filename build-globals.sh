#!/bin/bash
#
# Script to create modlib_global.S which contains a structure define 
# the API names and addresses we will export for resolving symbols in
# dynamic loaded shared objects. Typically these are libc APIs.

#
# Find an entrypoint using a binary search
#
findep()
{
	CHECK=$1
	SIZE=${#SYM[@]}
	L=0
	R=$((SIZE - 1))
	OLDL=99999999
	OLDR=99999999
	while [ ${L} -le ${R} ]
	do
		T=$(( L + R ))
		M=$(( T / 2 ))
		N=$(( T % 2 ))
		M=$(( M - N ))
		if [ ${SYM[${M}]} \< ${CHECK} ]; then
			L=$(( M + 1 ))
		elif [ ${SYM[${M}]} = ${CHECK} ]; then
			return 1
		else
			R=$(( M - 1 ))
		fi
		if [ ${OLDL} -eq ${L} -a ${OLDR} -eq ${R} ]; then
			: return 0
		fi
		OLDL=${L}
		OLDR=${R}
	done
	return 0
}

#
# Extract entrypoints from a library after applying a filter to
# exclude those we aren't interested in.
#
getEP() 
{
	for ((i = 0; i < ${#OBJ[@]}; i++))
	do
		if [ -f staging/${OBJ[$i]} ]; then
			FUNCS=`${NM} -g --defined-only staging/${OBJ[$i]} 2>/dev/null | grep "^0" | awk '{print $3}' | sort | grep -Ev ${FILTER}`
			FUNC=(${FUNCS})
			for ((j = 0; j < ${#FUNC[@]}; j++))
			do
				findep ${FUNC[$j]}
				if [ $? -eq 1 ]; then
					EP[${I_EP}]=${FUNC[$j]}
					I_EP=$((I_EP + 1))
				fi
			done
		fi
	done 
}
	
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
I_EP=0

#
# Libraries to be searched
#
OBJS="libsched.a libdrivers.a libconfigs.a libstubs.a libkc.a libkmm.a libkarch.a libpass1.a libnet.a libcrypto.a libfs.a libbinfmt.a libxx.a libuc.a libumm.a libuarch.a libapps.a"
OBJ=(${OBJS})

#
# Perform the extraction from the libraries
#
getEP
EPS=`printf '%s\n' "${EP[@]}" | sort -u`
EP=(${EPS})

#
# Generate the modlib_xxxx_globals.S file
#
cat >${GLOBALS} <<__EOF__
#ifdef __CYGWIN__
#  define SYMBOL(s) _##s
#  define WEAK .weak
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
#  define WEAK .weak
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
#  define WEAK .weak_definition
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
	WEAK	SYMBOL(\ep)
	.quad	.l\index
	.quad	\ep
	.endm
# define ALIGN 8
#else
	.macro globalEntry index, ep
	WEAK	SYMBOL(\ep)		
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

for ((i = 0; i < ${#EP[@]}; i++))
do
	echo ".l${i}:	.string	\"${EP[$i]}\"" >>${GLOBALS}
done

cat >>${GLOBALS} <<__EOF__
	SIZE	globalNames 

	.align	ALIGN
	GLOBAL	nglobals
SYMBOL(nglobals):	
	.word	${#EP[@]}
	SIZE	nglobals

	.align	ALIGN
	GLOBAL	global_table
SYMBOL(global_table):
__EOF__

for ((i = 0; i < ${#EP[@]}; i++))
do
	echo "	globalEntry ${i}, ${EP[$i]}" >>${GLOBALS}
done

cat >>${GLOBALS} <<__EOF__ 
	SIZE	global_table
__EOF__

echo "${#EP[@]} symbols defined"
