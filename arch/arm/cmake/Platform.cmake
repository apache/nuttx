# configure target processor

if("${CONFIG_ARCH_CORTEXM0}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
elseif("${CONFIG_ARCH_CORTEXM3}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
elseif("${CONFIG_ARCH_CORTEXM4}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
elseif("${CONFIG_ARCH_CORTEXM7}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m7)
else()
  message(FATAL_ERROR "processor not set")
endif()

include(platforms/${CMAKE_SYSTEM_NAME}-${TOOLCHAIN_PREFIX}-gcc-${CMAKE_SYSTEM_PROCESSOR})
