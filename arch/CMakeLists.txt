# Declare arch library. In contrast to other user/kernel pairs built in
# KERNEL mode, on arch/ the "user" portion uses a different set of sources
# than the kernel portion. To keep things simple the naming k<lib> is not
# used. Instead, the "user" portion is named "arch_interface".

nuttx_add_kernel_library(arch)
target_include_directories(arch PRIVATE ${CMAKE_SOURCE_DIR}/sched)

if (NOT CONFIG_BUILD_FLAT)
  nuttx_add_system_library(arch_interface)
  target_include_directories(arch_interface PRIVATE ${CMAKE_SOURCE_DIR}/sched)
endif()

# TODO: move this higher up
#ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
#  EXTRALINKCMDS += @$(TOPDIR)/syscall/syscall_wraps.ldcmd
#endif

# include corresponding arch subdirectory
add_subdirectory(${CONFIG_ARCH})
