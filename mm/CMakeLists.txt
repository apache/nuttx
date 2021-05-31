nuttx_add_kernel_library(mm SPLIT)

add_subdirectory(bin)
add_subdirectory(circbuf)
add_subdirectory(iob)
add_subdirectory(kbin)
add_subdirectory(kmm_heap)
add_subdirectory(mm_gran)
add_subdirectory(mm_heap)
add_subdirectory(shm)
add_subdirectory(umm_heap)

target_include_directories(mm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
