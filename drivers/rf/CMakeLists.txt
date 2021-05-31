if (CONFIG_DRIVERS_RF)
  set(SRCS)

  if (CONFIG_SPI)
    if (CONFIG_RF_DAT31R5SP)
      list(APPEND SRCS dat-31r5-sp.c)
    endif()
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
