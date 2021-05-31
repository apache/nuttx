set(USRCS)
set(SRCS
  arm_saveusercontext.S
  arm_fullcontextrestore.S
  arm_switchcontext.S
  arm_testset.S
  arm_fetchadd.S
  vfork.S)

if (CONFIG_ARMV7M_LAZYFPU)
  list(APPEND SRCS arm_lazyexception.S)
else()
  list(APPEND SRCS arm_exception.S)
endif()

if (CONFIG_ARCH_FPU)
  list(APPEND SRCS arm_fpu.S)
endif()

target_sources(arch PRIVATE ${SRCS})

if (CONFIG_BUILD_PROTECTED)
  target_sources(arch_interface PRIVATE arm_signal_handler.S)
endif()

