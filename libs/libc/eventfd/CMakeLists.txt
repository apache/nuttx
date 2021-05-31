if (CONFIG_EVENT_FD)
  target_sources(c PRIVATE lib_eventfd.c)
endif()
