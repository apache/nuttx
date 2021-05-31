if (CONFIG_OPENAMP)
  nuttx_add_kernel_library(openamp)

  set(OPENAMP_VERSION 2020.04.0)

  include(openamp.cmake)
  include(libmetal.cmake)
endif()
