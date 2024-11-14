# ##############################################################################
# arch/sim/src/sim/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# Initialize empty variables to hold internal (emulated guest) and external
# (host) source files

set(SRCS)
set(HOSTSRCS)

set(HOST_INCLUDE_DIRS)
set(STDLIBS pthread)

if(CONFIG_COVERAGE_TOOLCHAIN)
  list(APPEND STDLIBS gcov)
endif()

list(APPEND HOST_DEFINITIONS -D__SIM__)

# common guest sources

list(
  APPEND
  SRCS
  sim_initialize.c
  sim_idle.c
  sim_cpuinfo.c
  sim_doirq.c
  sim_initialstate.c
  sim_createstack.c
  sim_usestack.c
  sim_releasestack.c
  sim_stackframe.c
  sim_exit.c
  sim_schedulesigaction.c
  sim_switchcontext.c
  sim_heap.c
  sim_uart.c
  sim_copyfullstate.c
  sim_sigdeliver.c
  sim_registerdump.c
  sim_saveusercontext.c
  sim_tcbinfo.c
  sim_sectionheap.c
  sim_checkhostfstypes.c)

if(CONFIG_HOST_X86_64)
  if(CONFIG_SIM_M32)
    list(APPEND SRCS sim_fork_x86.S)
  else()
    list(APPEND SRCS sim_fork_x86_64.S)
  endif()
elseif(CONFIG_HOST_X86)
  list(APPEND SRCS sim_fork_x86.S)
elseif(CONFIG_HOST_ARM)
  list(APPEND SRCS sim_fork_arm.S)
elseif(CONFIG_HOST_ARM64)
  list(APPEND SRCS sim_fork_arm64.S)
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS sim_backtrace.c)
endif()

if(CONFIG_ARCH_HAVE_FORK)
  list(APPEND SRCS sim_fork.c)
endif()

if(CONFIG_ONESHOT)
  list(APPEND SRCS sim_oneshot.c)
endif()

if(CONFIG_RTC_DRIVER)
  list(APPEND SRCS sim_rtc.c)
endif()

if(CONFIG_SIM_LCDDRIVER)
  list(APPEND SRCS sim_lcd.c)
elseif(CONFIG_SIM_FRAMEBUFFER)
  list(APPEND SRCS sim_framebuffer.c)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS sim_checkstack.c)
endif()

if(CONFIG_FS_FAT)
  list(APPEND SRCS sim_blockdevice.c sim_deviceimage.c)
  list(APPEND STDLIBS z)
endif()

if(APPLE)
  if(NOT CONFIG_LIBCXX)
    list(APPEND STDLIBS c++abi)
  endif()
else()
  list(APPEND STDLIBS rt)
endif()

if(CONFIG_LIBM_TOOLCHAIN)
  list(APPEND STDLIBS m)
endif()

if(CONFIG_RPMSG_VIRTIO)
  list(APPEND SRCS sim_rpmsg_vritio.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS sim_rptun.c)
endif()

if(CONFIG_SIM_SOUND_ALSA)
  list(APPEND SRCS posix/sim_alsa.c)
  list(APPEND SRCS sim_offload.c)
  list(APPEND STDLIBS asound)
  list(APPEND STDLIBS mad)
endif()

# host sources ###############################################################

list(
  APPEND
  HOSTSRCS
  sim_hostirq.c
  sim_hostmemory.c
  sim_hostmisc.c
  sim_hosttime.c
  sim_hostuart.c)

# Note: sim_macho_init.c is picky about the place in the object list for
# linking. Namely, its constructor should be the first one in the executable.
# For now, we are just assuming no other files in HOSTSRCS provide constructors.
if(CONFIG_HOST_MACOS)
  if(CONFIG_HAVE_CXXINITIALIZE)
    list(APPEND HOSTSRCS sim_macho_init.c)
  endif()
endif()

if(CONFIG_SIM_CAMERA_V4L2)
  list(APPEND HOSTSRCS sim_host_v4l2.c)
  list(APPEND SRCS sim_camera.c)
  list(APPEND STDLIBS v4l2)
endif()

if(CONFIG_SIM_VIDEO_DECODER)
  list(APPEND SRCS sim_decoder.c)
  list(APPEND SRCS sim_openh264dec.c)
endif()

if(CONFIG_SIM_VIDEO_ENCODER)
  list(APPEND SRCS sim_encoder.c)
  list(APPEND SRCS sim_x264encoder.c)
endif()

if(CONFIG_SPINLOCK)
  list(APPEND HOSTSRCS sim_testset.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS sim_smpsignal.c sim_cpuidlestack.c)
endif()

if(CONFIG_ARCH_HAVE_MULTICPU AND NOT WIN32)
  list(APPEND HOSTSRCS sim_hostsmp.c)
endif()

if(CONFIG_SIM_X11FB)
  list(APPEND HOSTSRCS sim_x11framebuffer.c)
  if(APPLE)
    find_package(X11 REQUIRED)
    if(X11_FOUND)
      target_include_directories(nuttx PRIVATE ${X11_INCLUDE_DIR})
      target_link_libraries(nuttx PRIVATE ${X11_LIBRARIES})
    endif()
  else()
    list(APPEND STDLIBS X11 Xext)
  endif()

  if(CONFIG_SIM_TOUCHSCREEN)
    list(APPEND SRCS sim_touchscreen.c)
    list(APPEND HOSTSRCS sim_x11eventloop.c)
  elseif(CONFIG_SIM_AJOYSTICK)
    list(APPEND SRCS sim_ajoystick.c)
    list(APPEND HOSTSRCS sim_x11eventloop.c)
  elseif(CONFIG_SIM_BUTTONS)
    list(APPEND HOSTSRCS sim_x11eventloop.c)
  endif()

  if(CONFIG_SIM_KEYBOARD)
    list(APPEND SRCS sim_keyboard.c)
  endif()
endif()

if(CONFIG_SIM_NETDEV_TAP)
  list(APPEND SRCS sim_netdriver.c)

  if(NOT CYGWIN)
    list(APPEND HOSTSRCS sim_tapdev.c)

  else() # CYGWIN != y
    list(APPEND HOSTSRCS sim_wpcap.c)
    list(APPEND STDLIBS /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a)
  endif() # CONFIG_WINDOWS_CYGWIN != y
elseif(CONFIG_SIM_NETDEV_VPNKIT)
  list(APPEND SRCS sim_netdriver.c)
  list(APPEND HOST_DEFINITIONS
       CONFIG_SIM_NETDEV_VPNKIT_PATH=\"${CONFIG_SIM_NETDEV_VPNKIT_PATH}\")
  list(APPEND HOSTSRCS sim_vpnkit.c vpnkit/sim_protocol.c
       vpnkit/sim_negotiate.c)
endif()

if(CONFIG_SIM_NETUSRSOCK)
  list(APPEND HOSTSRCS sim_hostusrsock.c)
  list(APPEND SRCS sim_usrsock.c)
endif()

if(CONFIG_SIM_HCISOCKET)
  list(APPEND HOSTSRCS sim_hosthcisocket.c)
  list(APPEND SRCS sim_hcisocket.c)
endif()

if(CONFIG_SIM_I2CBUS_LINUX)
  list(APPEND HOSTSRCS sim_linuxi2c.c)
endif()

if(CONFIG_SIM_SPI_LINUX)
  list(APPEND HOSTSRCS sim_linuxspi.c)
endif()

if(CONFIG_SIM_USB_DEV)
  list(APPEND SRCS sim_usbdev.c)
  if(CONFIG_SIM_USB_RAW_GADGET)
    list(APPEND HOSTSRCS sim_rawgadget.c)
  endif()
endif()

if(CONFIG_SIM_USB_HOST)
  list(APPEND SRCS sim_usbhost.c)
  if(CONFIG_SIM_LIBUSB)
    list(APPEND HOSTSRCS sim_libusb.c)
    list(APPEND STDLIBS usb-1.0)
  endif()
endif()

list(APPEND HOSTSRCS sim_hostfs.c)
list(APPEND HOST_DEFINITIONS CONFIG_NAME_MAX=${CONFIG_NAME_MAX})

configure_file(${NUTTX_DIR}/include/nuttx/fs/hostfs.h
               ${CMAKE_CURRENT_BINARY_DIR}/hostfs.h COPYONLY)
configure_file(${CMAKE_BINARY_DIR}/include/nuttx/config.h
               ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
target_include_directories(nuttx PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_include_directories(nuttx PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_include_directories(sim_head PUBLIC ${NUTTX_DIR}/sched)
target_sources(sim_head PUBLIC sim_head.c sim_doirq.c)

target_include_directories(arch PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_sources(arch PRIVATE ${SRCS})
target_compile_options(arch PRIVATE -fvisibility=default)

if(WIN32)
  set(HOSTDIR win)
else()
  set(HOSTDIR posix)
  target_link_libraries(nuttx PUBLIC ${STDLIBS})
endif()

set(WINHOSTSRCS)
foreach(hostsrc ${HOSTSRCS})
  list(APPEND WINHOSTSRCS ${HOSTDIR}/${hostsrc})
endforeach()

set(HOSTSRCS ${WINHOSTSRCS})

target_sources(nuttx PRIVATE ${HOSTSRCS})

get_target_property(HOST_COMPILE_OPTIONS nuttx COMPILE_OPTIONS)
if(HOST_COMPILE_OPTIONS)
  foreach(remove_item IN LISTS SIM_NO_HOST_OPTIONS)
    list(REMOVE_ITEM HOST_COMPILE_OPTIONS ${remove_item})
  endforeach()
  set_target_properties(nuttx PROPERTIES COMPILE_OPTIONS
                                         "${HOST_COMPILE_OPTIONS}")
endif()

target_compile_definitions(nuttx PRIVATE ${HOST_DEFINITIONS})
