# Parse CSV file to get expected output source files for each target
file(STRINGS syscall.csv SYSCALLS)
list(TRANSFORM SYSCALLS REPLACE "^\"([^,]+)\",.+" "\\1")
list(TRANSFORM SYSCALLS APPEND ".c")

if (CONFIG_LIB_SYSCALL)
  add_subdirectory(proxies)
  add_subdirectory(stubs)

  target_sources(stubs PRIVATE syscall_stublookup.c)
endif()

# TODO: should CONFIG_SCHED_INSTRUMENTATION_SYSCALL depend on CONFIG_LIB_SYSCALL?
if (CONFIG_SCHED_INSTRUMENTATION_SYSCALL)
  if (CONFIG_LIB_SYSCALL)
    target_sources(proxies PRIVATE syscall_names.c)
  else()
    target_sources(proxies PRIVATE syscall_names.c)
  endif()
endif()

if (CONFIG_SCHED_INSTRUMENTATION_SYSCALL)
  add_subdirectory(wraps)
endif()
