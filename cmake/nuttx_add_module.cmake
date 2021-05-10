function(nuttx_add_module target)
  add_executable(target ${ARGN})

  target_link_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,MODULE_LINK_OPTIONS>>)
  target_compile_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,MODULE_COMPILE_OPTIONS>>)
  target_compile_definitions(${target} PRIVATE
    $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,MODULE_DEFINITIONS>>
    $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_DEFINITIONS>>)
endfunction()
