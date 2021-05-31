set(ARCH_SUBDIR)

if (CONFIG_ARCH_ARMV7A)         # ARMv7-A
  set(ARCH_SUBDIR armv7-a)
elseif (CONFIG_ARCH_ARMV7R)     # ARMv7-R
  set(ARCH_SUBDIR armv7-r)
elseif (CONFIG_ARCH_ARMV7M)     # ARMv7-M
  set(ARCH_SUBDIR armv7-m)
elseif (CONFIG_ARCH_ARMV8M)     # ARMv8-M
  set(ARCH_SUBDIR armv8-m)
elseif (CONFIG_ARCH_ARMV6M)     # ARMv6-M
  set(ARCH_SUBDIR armv6-m)
else()                          # ARM9, ARM7TDMI, etc.
  set(ARCH_SUBDIR arm)
endif()

add_subdirectory(src)
