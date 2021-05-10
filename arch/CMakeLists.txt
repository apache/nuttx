# declare arch lib
nuttx_add_kernel_library(arch)

target_include_directories(arch PRIVATE ${CMAKE_SOURCE_DIR}/sched)

# include corresponding arch subdirectory
add_subdirectory(${CONFIG_ARCH})
