if (CONFIG_DRIVERS_RC)
  set(SRCS lirc_dev.c)

  if (CONFIG_RC_DUMMY)
    list(APPEND SRCS dummy.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
