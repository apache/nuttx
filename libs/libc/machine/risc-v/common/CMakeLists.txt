set(SRCS)

if (CONFIG_LIBC_ARCH_ELF)
  list(APPEND SRCS arch_elf.c)
endif()

if (CONFIG_ARCH_SETJMP_H)
  list(APPEND SRCS arch_setjmp.S)
endif()

target_sources(c PRIVATE ${SRCS})
