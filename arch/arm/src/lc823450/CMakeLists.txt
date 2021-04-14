############################################################################
# arch/arm/src/lc823450/CMakeLists.txt
#
#   Copyright 2014, 2015, 2016, 2017, 2018 Sony Video & Sound Products Inc.
#   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
#   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
#   Author: Yasuhiro Osaki <Yasuhiro.Osaki@jp.sony.com>
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

HEAD_ASRC =

CMN_UASRCS =
CMN_UCSRCS =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_mdelay.c up_udelay.c up_exit.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_memfault.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_systemreset.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c up_doirq.c up_hardfault.c)
list(APPEND CMN_CSRCS up_svcall.c up_vfork.c up_trigger_irq.c)

# list(APPEND CMN_CSRCS up_dwt.c)

list(APPEND CMN_CSRCS up_stackframe.c)
list(APPEND CMN_ASRCS up_exception.S)
list(APPEND CMN_CSRCS up_vectors.c)

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

if("${CONFIG_ARCH_MEMCPY}" STREQUAL "y")
	list(APPEND CMN_ASRCS memcpy-armv7m.S)
endif()

if("${CONFIG_ARCH_MEMMOVE}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_memmove.c)
endif()

if("${CONFIG_DEBUG_STACK}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()


set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS lc823450_allocateheap2.c lc823450_start.c lc823450_irq.c lc823450_timer.c)
list(APPEND CHIP_CSRCS lc823450_lowputc.c lc823450_serial.c lc823450_clockconfig.c)
list(APPEND CHIP_CSRCS lc823450_syscontrol.c lc823450_gpio.c)

# Configuration-dependent LC823450 files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_idle.c)
endif()

if("${CONFIG_WATCHDOG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_wdt.c)
endif()

if("${CONFIG_LC823450_SPIFI), y)
	list(APPEND CHIP_CSRCS lc823450_spifi2.c)
endif()

if("${CONFIG_LC823450_SDIF), y)
	list(APPEND CHIP_CSRCS lc823450_sdc.c)
list(APPEND CHIP_CSRCS lc823450_sddrv_dep.c)
endif()

if("${CONFIG_PWM), y)
	list(APPEND CHIP_CSRCS lc823450_pwm.c)
endif()

if("${CONFIG_I2C), y)
	list(APPEND CHIP_CSRCS lc823450_i2c.c)
endif()

if("${CONFIG_SPI), y)
	list(APPEND CHIP_CSRCS lc823450_spi.c)
endif()

if("${CONFIG_ARCH_DMA), y)
	list(APPEND CMN_CSRCS lc823450_dma.c)
endif()

if("${CONFIG_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_rtc.c)
endif()

if("${CONFIG_FS_EVFAT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_evfat.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_usbdev.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_adc.c)
endif()

if("${CONFIG_LC823450_IPL2}" STREQUAL "y")
if("${CONFIG_LC823450_SPIFLASH_BOOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_spif_ipl2.c)
else()
	list(APPEND CHIP_CSRCS lc823450_ipl2.c)
endif()
endif()

if("${CONFIG_DVFS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_dvfs2.c)
list(APPEND CHIP_CSRCS lc823450_procfs_dvfs.c)
endif()

if("${CONFIG_PM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_pminitialize.c)
list(APPEND CHIP_CSRCS lc823450_sleep.c)
endif()

if("${CONFIG_LC823450_SPIFI_RAMFAT), y)
	list(APPEND CHIP_CSRCS lc823450_ramfat.c)
endif()

if("${CONFIG_SMP), y)
	list(APPEND CHIP_CSRCS lc823450_cpuidlestack.c)
list(APPEND CHIP_CSRCS lc823450_cpuindex.c)
list(APPEND CHIP_CSRCS lc823450_cpupause.c)
list(APPEND CHIP_CSRCS lc823450_cpustart.c)
list(APPEND CHIP_CSRCS lc823450_testset.c)
endif()

if("${CONFIG_LC823450_SDRAM), y)
	list(APPEND CHIP_CSRCS lc823450_sdram.c)
endif()

if("${CONFIG_LC823450_MTD}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_mtd.c)
list(APPEND CHIP_CSRCS lc823450_mmcl.c)
endif()

if("${CONFIG_LC823450_I2S0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_i2s.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lc823450_userspace.c)
endif()

if("${CONFIG_ARM_MPU}" STREQUAL "y")
CMN_CSRCS  += up_mpu.c)
list(APPEND CHIP_CSRCS lc823450_mpuinit2.c)
endif()
