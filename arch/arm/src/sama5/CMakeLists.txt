############################################################################
# arch/arm/sama5/CMakeLists.txt
#
#   Copyright (C) 2013-2014, 2016, 2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
############################################################################

# The vector table is the "head" object, i.e., the one that must forced into
# the link in order to draw in all of the other components

HEAD_ASRC  = arm_vectortab.S)

if("${CONFIG_BUILD_KERNEL}" STREQUAL "y")
STARTUP_OBJS = crt0$(OBJEXT)
endif()

# Force the start-up logic to be at the beginning of the .text to simplify
# debug.

if("${CONFIG_PAGING}" STREQUAL "y")
set(CMN_ASRCS)
list(APPEND CMN_ASRCS arm_pghead.S)
else()
set(CMN_ASRCS)
list(APPEND CMN_ASRCS arm_head.S)
endif()

# Common assembly language files

list(APPEND CMN_ASRCS arm_vectors.S arm_fpuconfig.S arm_fullcontextrestore.S)
list(APPEND CMN_ASRCS arm_saveusercontext.S arm_vectoraddrexcptn.S arm_vfork.S)
list(APPEND CMN_ASRCS arm_testset.S arm_fetchadd.S)
list(APPEND CMN_ASRCS cp15_coherent_dcache.S cp15_invalidate_dcache.S)
list(APPEND CMN_ASRCS cp15_clean_dcache.S cp15_flush_dcache.S cp15_invalidate_dcache_all.S)

# Configuration dependent assembly language files

# Common C source files

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_initialize.c up_interruptcontext.c up_exit.c up_createstack.c)
list(APPEND CMN_CSRCS up_releasestack.c up_usestack.c up_vfork.c up_puts.c up_mdelay.c)
list(APPEND CMN_CSRCS up_stackframe.c up_udelay.c up_modifyreg8.c up_modifyreg16.c)
list(APPEND CMN_CSRCS up_modifyreg32.c)

list(APPEND CMN_CSRCS arm_assert.c arm_blocktask.c arm_copyfullstate.c arm_dataabort.c)
list(APPEND CMN_CSRCS arm_doirq.c arm_initialstate.c arm_mmu.c arm_prefetchabort.c)
list(APPEND CMN_CSRCS arm_releasepending.c arm_reprioritizertr.c)
list(APPEND CMN_CSRCS arm_schedulesigaction.c arm_sigdeliver.c arm_syscall.c)
list(APPEND CMN_CSRCS arm_unblocktask.c arm_undefinedinsn.c)

# Configuration dependent C files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_ARMV7A_L2CC_PL310}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_l2cc_pl310.c)
endif()

if("${CONFIG_PAGING}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_allocpage.c arm_checkmapping.c arm_pginitialize.c)
list(APPEND CMN_CSRCS arm_va2pte.c)
endif()

if("${CONFIG_BUILD_KERNEL}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c arm_signal_dispatch.c)
endif()

if("${CONFIG_ARCH_ADDRENV}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv.c arm_addrenv_utils.c arm_pgalloc.c)
if("${CONFIG_ARCH_STACK_DYNAMIC}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_ustack.c)
endif()
if("${CONFIG_ARCH_KERNEL_STACK}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_kstack.c)
endif()
if("${CONFIG_MM_SHM}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_shm.c)
endif()
endif()

if("${CONFIG_MM_PGALLOC}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_physpgaddr.c)
if("${CONFIG_ARCH_PGPOOL_MAPPING}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_virtpgaddr.c)
endif()
endif()

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_coherent_dcache.c)
elseif("${CONFIG_MODULE}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_coherent_dcache.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS arm_savefpu.S arm_restorefpu.S)
list(APPEND CMN_CSRCS arm_copyarmstate.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

# SAMA5-specific assembly language files


# SAMA5-specific C source files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sam_allocateheap.c sam_boot.c sam_clockconfig.c sam_irq.c)
list(APPEND CHIP_CSRCS sam_lowputc.c sam_memories.c sam_memorymap.c sam_pck.c)
list(APPEND CHIP_CSRCS sam_pio.c sam_pmc.c sam_sckc.c sam_serial.c sam_serialinit.c)

# Configuration dependent C and assembly language files

if("${CONFIG_MM_PGALLOC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_pgalloc.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_SAMA5_DMAC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dmac.c)
else()
if("${CONFIG_SAMA5_DMAC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dmac.c)
endif()
endif()

if("${CONFIG_SAMA5_XDMAC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_xdmac.c)
else()
if("${CONFIG_SAMA5_XDMAC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_xdmac.c)
endif()
endif()

if("${CONFIG_SAMA5_PIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_pioirq.c)
endif()

if("${CONFIG_SAMA5_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_rtc.c)
endif()

if("${CONFIG_SAMA5_WDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_wdt.c)
endif()

if("${CONFIG_SAMA5_DBGU}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dbgu.c)
endif()

if("${CONFIG_SAMA5_TRNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_trng.c)
endif()

if("${CONFIG_SAMA5_FLEXCOM_USART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_flexcom_serial.c)
endif()

if("${CONFIG_SAMA5_SPI0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
else()
if("${CONFIG_SAMA5_SPI1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
else()
endif()
endif()

if("${CONFIG_SAMA5_SSC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ssc.c)
else()
if("${CONFIG_SAMA5_SSC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ssc.c)
else()
endif()
endif()

if("${CONFIG_SAMA5_LCDC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_lcd.c)
endif()

if("${CONFIG_SAMA5_UHPHS}" STREQUAL "y")
if("${CONFIG_SAMA5_OHCI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ohci.c)
endif()
if("${CONFIG_SAMA5_EHCI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ehci.c)
endif()
endif()

if("${CONFIG_SAMA5_UDPHS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_udphs.c)
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
if("${CONFIG_USBHOST_TRACE}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_usbhost.c)
else()
if("${CONFIG_DEBUG_USB}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_usbhost.c)
endif()
endif()
endif()

if("${CONFIG_SAMA5_HSMCI0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_hsmci.c sam_hsmci_clkdiv.c)
else()
if("${CONFIG_SAMA5_HSMCI1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_hsmci.c sam_hsmci_clkdiv.c)
else()
if("${CONFIG_SAMA5_HSMCI2}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_hsmci.c sam_hsmci_clkdiv.c)
endif()
endif()
endif()

if("${CONFIG_NET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ethernet.c)
if("${CONFIG_SAMA5_EMACA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_emaca.c)
endif()
if("${CONFIG_SAMA5_EMACB}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_emacb.c)
endif()
if("${CONFIG_SAMA5_GMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_gmac.c)
endif()
endif()

if("${CONFIG_SAMA5_CAN0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_can.c)
else()
if("${CONFIG_SAMA5_CAN1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_can.c)
endif()
endif()

if("${CONFIG_SAMA5_TWI0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twi.c)
else()
if("${CONFIG_SAMA5_TWI1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twi.c)
else()
if("${CONFIG_SAMA5_TWI2}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twi.c)
endif()
endif()
endif()

if("${CONFIG_SAMA5_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_adc.c)
if("${CONFIG_SAMA5_TSD}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tsd.c)
endif()
endif()

if("${CONFIG_SAMA5_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_pwm.c)
endif()

if("${CONFIG_SAMA5_HAVE_TC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tc.c)
if("${CONFIG_SAMA5_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_oneshot.c sam_oneshot_lowerhalf.c)
endif()
if("${CONFIG_SAMA5_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_freerun.c)
endif()
if("${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tickless.c)
endif()
endif()

if("${CONFIG_SAMA5_EBICS0_NAND}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
else()
if("${CONFIG_SAMA5_EBICS1_NAND}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
else()
if("${CONFIG_SAMA5_EBICS2_NAND}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
else()
if("${CONFIG_SAMA5_EBICS3_NAND}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
endif()
endif()
endif()
endif()
