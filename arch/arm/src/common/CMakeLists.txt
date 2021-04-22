
set(CSRCS)

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CSRCS arm_task_start.c arm_pthread_start.c
	)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CSRCS arm_checkstack.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CSRCS arm_idle.c)
endif()

list(APPEND CSRCS
	arm_allocateheap.c
	arm_createstack.c
	#arm_etherstub.c
	arm_exit.c
	arm_hostfs.c
	arm_initialize.c
	arm_interruptcontext.c
	arm_lowputs.c
	arm_mdelay.c
	arm_modifyreg16.c
	arm_modifyreg32.c
	arm_modifyreg8.c
	arm_puts.c
	arm_releasestack.c
	arm_semi_syslog.c
	arm_stackframe.c
	arm_udelay.c
	arm_usestack.c
	arm_vfork.c
)

target_sources(arch PRIVATE ${CSRCS})
