add_subdirectory(${CONFIG_ARCH_CHIP})

# Include directories (before system ones) as PUBLIC so that it can be
# exposed to libboard
target_include_directories(arch BEFORE PUBLIC ${CONFIG_ARCH_CHIP} common)

if (NOT CONFIG_BUILD_FLAT)
  target_include_directories(arch_interface BEFORE PUBLIC ${CONFIG_ARCH_CHIP} common)
endif()
