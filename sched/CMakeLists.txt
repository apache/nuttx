
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

nuttx_add_kernel_library(sched)

add_subdirectory(clock)
add_subdirectory(environ)
add_subdirectory(group)
add_subdirectory(init)
add_subdirectory(irq)
add_subdirectory(module)
add_subdirectory(mqueue)
add_subdirectory(paging)
add_subdirectory(pthread)
add_subdirectory(sched)
add_subdirectory(semaphore)
add_subdirectory(signal)
add_subdirectory(task)
add_subdirectory(timer)
add_subdirectory(wdog)
add_subdirectory(wqueue)
add_subdirectory(tls)

target_include_directories(sched INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
