nuttx_add_kernel_library(stubs)

list(TRANSFORM SYSCALLS PREPEND "${CMAKE_CURRENT_BINARY_DIR}/STUB_")

# generate files
add_custom_command(
  OUTPUT ${SYSCALLS}
  COMMAND ${CMAKE_BINARY_DIR}/bin/mksyscall -s ${CMAKE_CURRENT_LIST_DIR}/../syscall.csv
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS nuttx_host_tools)

# add sources to target
target_sources(stubs PRIVATE ${SYSCALLS})

