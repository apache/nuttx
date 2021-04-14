
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SRCS)

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


nuttx_add_library(sched ${SRCS})
target_include_directories(sched INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
