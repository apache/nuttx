
set(CSRCS)

if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
	list(APPEND CSRCS arm_stackcheck.c)
endif()

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CSRCS gnu/arm_lazyexception.S)
else()
	list(APPEND CSRCS gnu/arm_exception.S)
endif()

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CSRCS arm_ramvec_initialize.c arm_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CSRCS arm_mpu.c)
	if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
		list(APPEND CSRCS arm_signal_dispatch.c)
		list(APPEND CSRCS gnu/arm_signal_handler.S)
	endif()
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CSRCS gnu/arm_fpu.S)
	list(APPEND CSRCS arm_copyarmstate.c)
endif()

if("${CONFIG_ARMV7M_ITMSYSLOG}" STREQUAL "y")
	list(APPEND CSRCS arm_itm_syslog.c)
endif()

if("${CONFIG_ARMV7M_DCACHE}" STREQUAL "y")
	list(APPEND CSRCS arm_cache.c)
endif()

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CSRCS arm_cache.c)
elseif("${CONFIG_MODULE}" STREQUAL "y")
	list(APPEND CSRCS arm_cache.c)
endif()

set(ASRC_DIR "gnu")
if("${CONFIG_ARCH_TOOLCHAIN_IAR}" STREQUAL "y")
	set(ASRC_DIR "iar")
endif()

list(APPEND CSRCS
	${ASRC_DIR}/arm_saveusercontext.S
	${ASRC_DIR}/arm_fullcontextrestore.S
	${ASRC_DIR}/arm_switchcontext.S
	${ASRC_DIR}/arm_testset.S
	${ASRC_DIR}/arm_fetchadd.S
	${ASRC_DIR}/vfork.S

	arm_assert.c
	arm_blocktask.c
	arm_copyfullstate.c
	arm_doirq.c
	arm_hardfault.c
	arm_initialstate.c
	arm_memfault.c
	arm_releasepending.c
	arm_reprioritizertr.c
	arm_schedulesigaction.c
	arm_sigdeliver.c
	arm_svcall.c
	arm_systemreset.c
	arm_trigger_irq.c
	arm_unblocktask.c
	arm_vectors.c
)

target_include_directories(arch PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_sources(arch PRIVATE ${CSRCS})
