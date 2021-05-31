# Configure project
cmake_minimum_required(VERSION 3.16)
project(nuttx_tools LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

message(STATUS "NuttX Host Tools")

# set basic warnings

add_compile_options(-Wall)

# configure according to platform

if (MSYS)
  add_compile_definitions(CONFIG_WINDOWS_NATIVE=y)
else()
  # GCC or clang is assumed in all other POSIX environments
  # (Linux, Cygwin, MSYS2, macOS).
  # strtok_r is used in some tools, but does not seem to be available in
  # the MinGW environment.

  add_compile_definitions(HAVE_STRTOK_C=1)

  if (CYGWIN)
    add_compile_definitions(HOST_CYGWIN=1)
  endif()
endif()

# define targets

add_library(cfg cfgdefine.c)

add_executable(mkconfig mkconfig.c)
target_link_libraries(mkconfig cfg)
install(TARGETS mkconfig DESTINATION bin)

add_executable(mkversion mkversion.c)
target_link_libraries(mkversion cfg)
install(TARGETS mkversion DESTINATION bin)

add_library(csvparser csvparser.c)

add_executable(mksyscall mksyscall.c)
target_link_libraries(mksyscall PRIVATE csvparser)
install(TARGETS mksyscall DESTINATION bin)

#add_executable(nxstyle nxstyle.c)
#install(TARGETS nxstyle DESTINATION bin)

#add_executable(gencromfs gencromfs.c)
#install(TARGETS gencromfs DESTINATION bin)
