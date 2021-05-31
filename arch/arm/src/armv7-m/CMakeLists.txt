# TODO: do not use config to detect this
if (CONFIG_ARCH_TOOLCHAIN_IAR)
  add_subdirectory(iar)
else()
  add_subdirectory(gnu)
endif()

set(SRCS
  arm_assert.c arm_blocktask.c arm_copyfullstate.c arm_hardfault.c
  arm_initialstate.c arm_memfault.c arm_releasepending.c arm_reprioritizertr.c
  arm_schedulesigaction.c arm_sigdeliver.c  arm_svcall.c arm_systemreset.c
  arm_trigger_irq.c arm_unblocktask.c arm_doirq.c )

if (CONFIG_STM32_TICKLESS_SYSTICK)
  list(APPEND SRCS arm_systick.c)
endif()


if (CONFIG_ARMV7M_STACKCHECK)
  list(APPEND SRCS arm_stackcheck.c)
endif()

list(APPEND SRCS arm_vectors.c)

if (CONFIG_ARCH_RAMVECTORS)
  list(APPEND SRCS arm_ramvec_initialize.c arm_ramvec_attach.c)
endif()

if (CONFIG_ARM_MPU)
  list(APPEND SRCS arm_mpu.c)
endif()

if (CONFIG_ARCH_FPU)
  list(APPEND SRCS arm_copyarmstate.c)
endif()

if (CONFIG_ARMV7M_ITMSYSLOG)
  list(APPEND SRCS arm_itm_syslog.c)
endif()

if (CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS arm_signal_dispatch.c)
endif()

target_sources(arch PRIVATE ${SRCS})
