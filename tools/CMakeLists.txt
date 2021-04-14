# Configure project
cmake_minimum_required(VERSION 3.16)
project(NuttX-tools LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

message(STATUS "NuttX Host Tools")

add_library(cfg cfgdefine.c)

add_executable(mkconfig mkconfig.c)
target_link_libraries(mkconfig cfg)
install(TARGETS mkconfig DESTINATION bin)

add_executable(mkversion mkversion.c)
target_link_libraries(mkversion cfg)
install(TARGETS mkversion DESTINATION bin)

add_executable(nxstyle nxstyle.c)
install(TARGETS nxstyle DESTINATION bin)
