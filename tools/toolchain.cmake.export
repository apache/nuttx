set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(NUTTX 1)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(NUTTX_PATH ${CMAKE_CURRENT_LIST_DIR}/..)
include(${NUTTX_PATH}/scripts/target.cmake)

set(LINKER_SCRIPT ${NUTTX_PATH}/scripts/${LDNAME})

set(CMAKE_C_FLAGS "${ARCHCPUFLAGS} ${ARCHCFLAGS} -D__NuttX__")
set(CMAKE_CXX_FLAGS "${ARCHCPUFLAGS} ${ARCHCXXFLAGS} -D__NuttX__")

set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${NUTTX_PATH}/include
                                         ${NUTTX_PATH}/arch/chip)

set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
    ${NUTTX_PATH}/include/${NUTTX_CXX} ${NUTTX_PATH}/include
    ${NUTTX_PATH}/arch/chip)

file(GLOB STARTUP_OBJS ${NUTTX_PATH}/startup/*)

add_compile_options(-nostdlib)
add_compile_options(-ffunction-sections -fdata-sections)

# same entry used for all build modes in crt0.c and arch/.../xxx_start.c

set(ENTRY_NAME "__start")

set(CMAKE_C_LINK_EXECUTABLE
    "<CMAKE_LINKER> ${LDFLAGS} --entry=${ENTRY_NAME} -T${LINKER_SCRIPT} <OBJECTS> ${STARTUP_OBJS} -o <TARGET> <LINK_LIBRARIES> -L${NUTTX_PATH}/libs --start-group ${LDLIBS} ${EXTRA_LIBS} --end-group"
)
set(CMAKE_CXX_LINK_EXECUTABLE
    "<CMAKE_LINKER> ${LDFLAGS} --entry=${ENTRY_NAME} -T${LINKER_SCRIPT} <OBJECTS> ${STARTUP_OBJS} -o <TARGET> <LINK_LIBRARIES> -L${NUTTX_PATH}/libs --start-group ${LDLIBS} ${EXTRA_LIBS} --end-group"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
