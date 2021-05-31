if (CONFIG_MOTOR_FOC)
  target_sources(drivers PRIVATE foc_dev.c)
endif()
