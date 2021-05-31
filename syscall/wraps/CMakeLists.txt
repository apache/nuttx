nuttx_add_kernel_library(wraps)

list(TRANSFORM SYSCALLS PREPEND "${CMAKE_CURRENT_BINARY_DIR}/WRAPS_")

# generate files
execute_process(
  OUTPUT ${SYSCALLS}
  COMMAND ${CMAKE_BINARY_DIR}/bin/mksyscall -w ${CMAKE_CURRENT_LIST_DIR}/../syscall.csv
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS nuttx_host_tools)

# TODO:
#SYSCALLWRAPS = syscall_wraps.ldcmd
#$(Q) $(CPP) $(CPPFLAGS) $(SYSCALLWRAPS:.ldcmd=.h) | \
#  sed -e '1,/WRAPOPTSTARTS/d' -e '/^#/d' > $(SYSCALLWRAPS)

# add sources to target
target_sources(wraps PRIVATE ${SYSCALLS})
