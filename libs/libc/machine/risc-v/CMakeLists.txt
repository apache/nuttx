add_subdirectory(common)

if (CONFIG_ARCH_RV64GC)
  add_subdirectory(rv64)
endif()

if (CONFIG_ARCH_RV32IM)
  add_subdirectory(rv32)
endif()
