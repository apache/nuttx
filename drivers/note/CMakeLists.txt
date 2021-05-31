set(SRCS)

if (CONFIG_DRIVER_NOTE)
  list(APPEND SRCS note_driver.c)
endif()

if (CONFIG_DRIVER_NOTERAM)
  list(APPEND SRCS noteram_driver.c)
endif()

if (CONFIG_DRIVER_NOTECTL)
  list(APPEND SRCS notectl_driver.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
