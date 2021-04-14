
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SRCS)

add_subdirectory(nx)
#add_subdirectory(nxfonts) # TODO: generation
add_subdirectory(nxglib)
add_subdirectory(nxmu)
add_subdirectory(nxtk)

nuttx_add_library(nx ${SRCS})
target_include_directories(nx INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
