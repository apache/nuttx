if (CONFIG_OPENAMP)
  # get libmetal

  FetchContent_Declare(libmetal
    URL https://github.com/OpenAMP/libmetal/archive/v${OPENAMP_VERSION}.zip
    PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/0001-system-nuttx-change-clock_systimespec-to-clock_systi.patch)
  FetchContent_Populate(libmetal)
  FetchContent_GetProperties(libmetal SOURCE_DIR LIBMETAL_SOURCE_DIR)

  # process libmetal headers

  if (CONFIG_ARCH STREQUAL sim)
    set(LIBMETAL_ARCH x86_64)
  elseif (CONFIG_ARCH STREQUAL risc-v)
    set(LIBMETAL_ARCH riscv)
  else()
    set(LIBMETAL_ARCH ${CONFIG_ARCH})
  endif()

  set(PROJECT_VER_MAJOR 0)
  set(PROJECT_VER_MINOR 1)
  set(PROJECT_VER_PATCH 0)
  set(PROJECT_VER 0.1.0)
  set(PROJECT_SYSTEM nuttx)
  set(PROJECT_PROCESSOR ${LIBMETAL_ARCH})
  set(PROJECT_MACHINE ${CONFIG_ARCH_CHIP})
  set(PROJECT_SYSTEM_UPPER nuttx)
  set(PROJECT_PROCESSOR_UPPER ${LIBMETAL_ARCH})
  set(PROJECT_MACHINE_UPPER ${CONFIG_ARCH_CHIP})

  file(GLOB OPENMETAL_HEADERS RELATIVE ${LIBMETAL_SOURCE_DIR}/lib
    ${LIBMETAL_SOURCE_DIR}/lib/compiler/gcc/*.h
    ${LIBMETAL_SOURCE_DIR}/lib/processor/${LIBMETAL_ARCH}/*.h
    ${LIBMETAL_SOURCE_DIR}/lib/system/nuttx/*.h
    ${LIBMETAL_SOURCE_DIR}/lib/*.h)

  foreach(header ${OPENMETAL_HEADERS})
    configure_file(${LIBMETAL_SOURCE_DIR}/lib/${header} ${CMAKE_BINARY_DIR}/include/metal/${header})
  endforeach()

  set_property(TARGET nuttx APPEND PROPERTY NUTTX_KERNEL_INCLUDE_DIRECTORIES)

  # include sources

  set(SRCS
    lib/system/nuttx/condition.c lib/system/nuttx/device.c
    lib/system/nuttx/init.c lib/system/nuttx/io.c
    lib/system/nuttx/irq.c lib/system/nuttx/shmem.c
    lib/system/nuttx/time.c lib/device.c lib/dma.c lib/init.c lib/io.c
    lib/irq.c lib/log.c lib/shmem.c lib/version.c)

  list(TRANSFORM SRCS PREPEND ${LIBMETAL_SOURCE_DIR}/)
  target_sources(openamp PRIVATE ${SRCS})
  target_compile_definitions(openamp PRIVATE -DMETAL_INTERNAL)
endif()
