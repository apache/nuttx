set(PLATFORM_FLAGS -mcpu=cortex-m4 -mthumb)

if (CONFIG_ARCH_FPU)
  list(APPEND PLATFORM_FLAGS -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
else()
  list(APPEND PLATFORM_FLAGS -mfloat-abi=soft)
endif()

add_compile_options(${PLATFORM_FLAGS})

# TODO: this should be possible to be done in arch-independant way
# but the compile options are only finalized during build stage
# as they use generator expressions. Thus, I cannot seem to use them
# during config stage to invoke the compiler and get the path
# I resorted to just passing the platform flags right here.

execute_process(
  COMMAND ${CMAKE_C_COMPILER} ${PLATFORM_FLAGS} --print-libgcc-file-name
  OUTPUT_VARIABLE LIBGCC_FILENAME)
get_filename_component(LIBGCC_FILEPATH ${LIBGCC_FILENAME} PATH)

