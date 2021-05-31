set(SRCS
  arm_createstack.c
  arm_exit.c arm_initialize.c
  arm_interruptcontext.c arm_modifyreg8.c
  arm_modifyreg16.c arm_modifyreg32.c
  arm_releasestack.c
  arm_stackframe.c
  arm_udelay.c arm_usestack.c
  arm_vfork.c)

if (NOT CONFIG_TIMER_ARCH)
  list(APPEND SRCS arm_mdelay.c)
endif()

if (CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS arm_task_start.c arm_pthread_start.c)
endif()

if (CONFIG_STACK_COLORATION)
  list(APPEND SRCS arm_checkstack.c)
endif()

target_sources(arch PRIVATE ${SRCS})
