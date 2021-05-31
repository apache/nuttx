############################################################################
# arch/sim/src/CMakeLists.txt
#
#   Copyright (C) 2007, 2008, 2011-2012, 2014, 2016, 2018 Gregory Nutt. All
#     rights reserved.
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

# Initialize empty variables to hold internal (emulated guest) and external
# (host) source files

set(SRCS)
set(HOSTSRCS)

set(HOST_INCLUDE_DIRS)
set(STDLIBS pthread)

# common guest sources

list(APPEND SRCS
      up_initialize.c up_idle.c up_interruptcontext.c up_initialstate.c
      up_createstack.c up_usestack.c up_releasestack.c up_stackframe.c
      up_unblocktask.c up_blocktask.c up_releasepending.c
      up_reprioritizertr.c up_exit.c up_schedulesigaction.c
      up_heap.c up_uart.c up_assert.c up_copyfullstate.c up_sigdeliver.c
	  up_hostmisc.c up_backtrace.c
)

if(X86_64)
  if(CONFIG_SIM_M32)
    list(APPEND SRCS up_vfork32.S)
  else()
#    list(APPEND SRCS up_vfork_x86_64.S)
  endif()
elseif(X86)
  list(APPEND SRCS up_vfork32.S)
elseif(ARM)
  list(APPEND SRCS up_vfork_arm.S)
endif()

if (CONFIG_ARCH_HAVE_VFORK AND CONFIG_SCHED_WAITPID)
  list(APPEND SRCS up_vfork.c)
endif()

if (CONFIG_SCHED_INSTRUMENTATION AND NOT CONFIG_SCHED_INSTRUMENTATION_BUFFER)
  list(APPEND SRCS up_schednote.c)
endif()

if (CONFIG_ONESHOT)
  list(APPEND SRCS up_oneshot.c)
endif()

if (CONFIG_RTC_DRIVER)
  list(APPEND SRCS up_rtc.c)
endif()

if (CONFIG_SIM_LCDDRIVER)
  list(APPEND SRCS up_lcd.c)
elseif(CONFIG_SIM_FRAMEBUFFER)
  list(APPEND SRCS up_framebuffer.c)
endif()

if (CONFIG_STACK_COLORATION)
  list(APPEND SRCS up_checkstack.c)
endif()

if (CONFIG_SIM_IOEXPANDER)
  list(APPEND SRCS up_ioexpander.c)
endif()

if (CONFIG_SIM_SPIFLASH)
  list(APPEND SRCS up_spiflash.c)
endif()

if (CONFIG_SIM_QSPIFLASH)
  list(APPEND SRCS up_qspiflash.c)
endif()

if (CONFIG_FS_FAT)
  list(APPEND SRCS up_blockdevice.c up_deviceimage.c)
  list(APPEND STDLIBS z)
endif()

if (APPLE)
  if (NOT CONFIG_LIBCXX)
    list(APPEND STDLIBS c++abi)
  endif()
else()
  list(APPEND STDLIBS rt)
endif()

if (CONFIG_ARCH_ROMGETC)
  list(APPEND SRCS up_romgetc.c)
endif()

if (CONFIG_SIM_MOTOR_FOC)
  list(APPEND SRCS up_foc.c)
endif()

if (CONFIG_RPTUN)
  list(APPEND SRCS up_rptun.c)
endif()

if (CONFIG_SIM_SOUND_ALSA)
  list(APPEND SRCS up_alsa.c)
  list(APPEND STDLIBS asound)
endif()

# host sources ###############################################################

list(APPEND HOSTSRCS up_hostirq.c up_hostmemory.c up_hosttime.c up_simuart.c)

if (CONFIG_SPINLOCK)
  list(APPEND HOSTSRCS up_testset.c)
endif()

if (CONFIG_SMP)
  list(APPEND SRCS up_smpsignal.c up_cpuidlestack.c)
  list(APPEND HOSTSRCS up_simsmp.c)

  list(APPEND HOST_DEFINITIONS -DCONFIG_SMP=1 -DCONFIG_SMP_NCPUS=${CONFIG_SMP_NCPUS})
  if (CONFIG_SIM_WALLTIME)
    list(APPEND HOST_DEFINITIONS CONFIG_SIM_WALLTIME=1)
  endif()
  if (CONFIG_SCHED_INSTRUMENTATION)
    list(APPEND HOST_DEFINITIONS CONFIG_SCHED_INSTRUMENTATION=1)
  endif()
endif()

if (CONFIG_SIM_X11FB)
  if (CONFIG_SIM_X11NOSHM)
    list(APPEND HOST_DEFINITIONS CONFIG_SIM_X11NOSHM=1)
  endif()

  list(APPEND HOSTSRCS up_x11framebuffer.c)
  list(APPEND STDLIBS X11 Xext)

  if (CONFIG_SIM_TOUCHSCREEN)
    list(APPEND SRCS up_touchscreen.c)
    list(APPEND HOST_DEFINITIONS CONFIG_SIM_TOUCHSCREEN=1)
    list(APPEND HOSTSRCS up_x11eventloop.c)
  elseif (CONFIG_SIM_AJOYSTICK)
    list(APPEND SRCS up_ajoystick.c)
    list(APPEND HOST_DEFINITIONS CONFIG_SIM_AJOYSTICK=1)
    list(APPEND HOSTSRCS up_x11eventloop.c)
  elseif (CONFIG_SIM_BUTTONS)
    list(APPEND HOST_DEFINITIONS CONFIG_SIM_BUTTONS=1)
    list(APPEND HOSTSRCS up_x11eventloop.c)
  endif()
endif()

if (CONFIG_SIM_NETDEV_TAP)
  list(APPEND SRCS up_netdriver.c)
  list(APPEND HOST_DEFINITIONS NETDEV_BUFSIZE=${CONFIG_NET_ETH_PKTSIZE})

  if (NOT CYGWIN)
    list(APPEND HOSTSRCS up_tapdev.c)

    if (CONFIG_SIM_NET_BRIDGE)
      list(APPEND HOST_DEFINITIONS
            -DCONFIG_SIM_NET_BRIDGE
            -DCONFIG_SIM_NET_BRIDGE_DEVICE=\"${CONFIG_SIM_NET_BRIDGE_DEVICE}\"
      )
    endif()
    if (CONFIG_SIM_NET_HOST_ROUTE)
      list(APPEND HOST_DEFINITIONS CONFIG_SIM_NET_HOST_ROUTE)
    endif()
  else() # CYGWIN != y
    list(APPEND HOSTSRCS up_wpcap.c)
    list(APPEND STDLIBS /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a)
  endif() # CONFIG_WINDOWS_CYGWIN != y
elseif (CONFIG_SIM_NETDEV_VPNKIT)
  list(APPEND SRCS up_netdriver.c)
  list(APPEND HOST_DEFINITIONS CONFIG_SIM_NETDEV_VPNKIT_PATH=\"${CONFIG_SIM_NETDEV_VPNKIT_PATH}\")
  list(APPEND HOSTSRCS up_vpnkit.c vpnkit/protocol.c vpnkit/negotiate.c)
endif()

if (CONFIG_SIM_HCISOCKET)
  list(APPEND HOSTSRCS up_hcisocket_host.c)
  list(APPEND SRCS up_hcisocket.c)
endif()

if(CONFIG_SIM_BTUART)
  list(APPEND HOSTSRCS up_hcisocket_host.c)
  list(APPEND SRCS up_btuart.c)
endif()

if(CONFIG_I2C_RESET)
  list(APPEND HOST_DEFINITIONS CONFIG_I2C_RESET=1)
endif()

if (CONFIG_SIM_I2CBUS_LINUX)
  list(APPEND HOSTSRCS up_i2cbuslinux.c)
endif()

if (CONFIG_FS_HOSTFS AND NOT CONFIG_FS_HOSTFS_RPMSG)
  list(APPEND HOSTSRCS up_hostfs.c)
  list(APPEND HOST_DEFINITIONS CONFIG_NAME_MAX=${CONFIG_NAME_MAX})

  configure_file(${NUTTX_DIR}/include/nuttx/fs/hostfs.h ${CMAKE_CURRENT_BINARY_DIR}/hostfs.h COPYONLY)
  target_include_directories(nuttx PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()

target_sources(up_head PUBLIC up_head.c)
target_sources(arch PRIVATE ${SRCS})
target_sources(nuttx PRIVATE ${HOSTSRCS})

target_link_libraries(nuttx PUBLIC ${STDLIBS})
target_compile_definitions(nuttx PRIVATE ${HOST_DEFINITIONS})
