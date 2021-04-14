
set(CSRCS)

list(APPEND CSRCS up_exception.S)

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
		list(APPEND CSRCS up_signal_dispatch.c)
		list(APPEND CSRCS up_signal_handler.S)
	endif()
endif()

list(APPEND CSRCS
	up_saveusercontext.S
	up_fullcontextrestore.S
	up_switchcontext.S
	vfork.S

	up_assert.c
	up_blocktask.c
	up_copyfullstate.c
	up_doirq.c
	up_hardfault.c
	up_initialstate.c
	up_releasepending.c
	up_reprioritizertr.c
	up_schedulesigaction.c
	up_sigdeliver.c
	up_svcall.c
	up_systemreset.c
	up_unblocktask.c
	up_vectors.c
)
set_parent_srcs(${CSRCS})
