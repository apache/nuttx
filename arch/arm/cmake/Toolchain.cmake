# arm-none-eabi-gcc toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(TOOLCHAIN_PREFIX arm-none-eabi)
set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

# needed for test compilation
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# compiler tools
# TODO: move to Toolchain-tools?
foreach(tool nm ld objcopy ranlib strip)
	string(TOUPPER ${tool} TOOL)
	find_program(CMAKE_${TOOL} ${TOOLCHAIN_PREFIX}-${tool} REQUIRED)
endforeach()

include(Toolchain-tools)

