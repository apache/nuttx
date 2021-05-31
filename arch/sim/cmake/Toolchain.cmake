if (APPLE)
  find_program(CMAKE_C_ELF_COMPILER x86_64-elf-gcc)
  find_program(CMAKE_CXX_ELF_COMPILER x86_64-elf-g++)
endif()

include(Toolchain-tools)
