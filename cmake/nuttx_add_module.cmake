function(nuttx_add_module target)
  # Use ELF capable toolchain, by building manually and overwriting the non-elf output
  if (CMAKE_C_ELF_COMPILER)
    add_library(${target} ${ARGN})

    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_C_ELF_COMPILER} $<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_LINK_OPTIONS> $<TARGET_FILE:${target}> -o ${target}
      COMMAND_EXPAND_LISTS)
  else()
    add_executable(${target} ${ARGN})
    target_link_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_LINK_OPTIONS>>)
  endif()

  set_property(TARGET ${target} PROPERTY ELF_BINARY ${CMAKE_CURRENT_BINARY_DIR}/${target})

  nuttx_add_library_internal(${target})

  target_compile_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_COMPILE_OPTIONS>>)
  target_compile_definitions(${target} PRIVATE
    $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_DEFINITIONS>>
    $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_DEFINITIONS>>)

  install(TARGETS ${target})
endfunction()
