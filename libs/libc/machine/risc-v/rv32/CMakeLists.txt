if (CONFIG_RISCV_MEMCPY)
  target_sources(c PRIVATE arch_memcpy.S)
endif()
