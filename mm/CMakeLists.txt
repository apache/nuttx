
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SRCS)

#add_subdirectory(bin)
add_subdirectory(iob)
#add_subdirectory(kbin)
#add_subdirectory(kmm_heap)
add_subdirectory(mm_gran)
add_subdirectory(mm_heap)
#add_subdirectory(shm)
#add_subdirectory(ubin)
add_subdirectory(umm_heap)


nuttx_add_library(mm ${SRCS})
target_include_directories(mm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
