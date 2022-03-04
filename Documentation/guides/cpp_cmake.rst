.. include:: /substitutions.rst
.. _cpp_cmake:

C++ Example using CMake
=======================

In some situations, developers intend to implement software using the NuttX platform in
a previously set hardware and configuration where it is not possible or allowed to make
changes. In such situations, less contact with the operating source tree is better, where
it is only used for the application.

Some approaches are possible to do that today:

* https://cwiki.apache.org/confluence/display/NUTTX/Building+NuttX+with+Applications+Outside+of+the+Source+Tree
* https://www.programmersought.com/article/61604062421/

We have been seen the increase of the use of C++ language in embedded systems application. And
CMake (https://www.cmake.org) is the preferred build system used to build C++ projects. NuttX
support C++ based projects.

Using the 'build as a library' procedure of NuttX, it is possible to build NuttX
applications using C++ language and also the cmake build tool.

This document will show how to reimplement the hellocpp project using this cmake.

Preparation
-----------

#. Base NuttX compilation changes

    For this example, load the configuration 'stm32f4discovery:testlibcxx' for building

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh stm32f4discovery:testlibcxx

    In menuconfig, the main points to be changed on a typical NuttX configuration are the following:

    * Set RTOS Features -> Tasks and Scheduling -> Application entry point to 'hellocpp_main'

    * Build NuttX and generate the export 

    .. code-block:: console

       $ make export

Creating the project
--------------------

#. Create your project file structure

    The project structure is organized as follow:

    .. code-block:: console

       hellocpp/
       hellocpp/CMakeLists.txt
       hellocpp/cmake/stm32f4discovery.cmake
       hellocpp/nuttx-export-10.0.1/
       hellocpp/src/CMakeLists.txt
       hellocpp/src/main.cpp
       hellocpp/src/HelloWorld.h
       hellocpp/src/HelloWorld.cpp

    The directory 'nuttx-export-10.0.1' is the unzipped content from the file created during
    make export procedure done before.

#. File contents

* hellocpp/CMakeLists.txt

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.2...3.15)

    project(HelloCpp
            VERSION 1.0
            DESCRIPTION "Hello world C++ NuttX"
    )

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    # set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_C_STANDARD 99)

    set(NUTTX_PATH "${CMAKE_SOURCE_DIR}/nuttx-export-10.0.1")

    include(cmake/stm32f4discovery.cmake)

    set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -fno-builtin -Wall -Wshadow -Wundef -fno-strict-aliasing -Os")
    set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -D_DEBUG -D_LIBCPP_BUILD_STATIC -D_LIBCPP_NO_EXCEPTIONS ")
    set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -fno-exceptions -fcheck-new -fno-rtti -pedantic ")
    set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -nostdinc++")

    set(AC_DEFINES "${AC_DEFINES} -DCONFIG_WCHAR_BUILTIN")

    include_directories(
            src
            ${NUTTX_PATH}/include
            ${NUTTX_PATH}/include/libcxx
            ${NUTTX_PATH}/arch/chip
    )

    set(EXE_NAME hellocpp)

    set(CMAKE_CXX_FLAGS     "${AC_HW_FLAGS} ${AC_DEFINES} ${AC_COMMON_FLAGS} ${AC_CXX_EXTRA_FLAGS}")
    if (PARAM_DEBUG)
        set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -g")
    endif()

    set(CMAKE_SKIP_RPATH ON)
    set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_LINKER} ${AC_LINKER_FLAGS} -o ${EXE_NAME}.elf <OBJECTS> <LINK_LIBRARIES>")

    set(BUILD_SHARED_LIBS OFF)

    add_subdirectory(src)

* hellocpp/cmake/stm32f4discovery.cmake

.. code-block:: cmake
    
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(MCU_LINKER_SCRIPT "${NUTTX_PATH}/scripts/ld.script")

    set(COMPILER_PREFIX arm-none-eabi-)

    # cmake-format: off
    set(CMAKE_C_COMPILER    ${COMPILER_PREFIX}gcc)
    set(CMAKE_CXX_COMPILER  ${COMPILER_PREFIX}g++)
    set(CMAKE_AR            ${COMPILER_PREFIX}ar)
    set(CMAKE_RANLIB        ${COMPILER_PREFIX}ranlib)
    set(CMAKE_LINKER        ${COMPILER_PREFIX}ld)
    set(CMAKE_ASM_COMPILER  ${COMPILER_PREFIX}gcc)
    set(CMAKE_OBJCOPY       ${COMPILER_PREFIX}objcopy)
    set(CMAKE_OBJDUMP       ${COMPILER_PREFIX}objdump)
    set(CMAKE_SIZE          ${COMPILER_PREFIX}size)

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

    set(AC_HW_FLAGS         "-mcpu=cortex-m4 -mthumb -mfloat-abi=soft ")
    set(AC_HW_FLAGS         "${AC_HW_FLAGS} -isystem ${NUTTX_PATH}/include")
    set(AC_HW_FLAGS         "${AC_HW_FLAGS} -pipe")

    set(AC_LINKER_FLAGS     "--entry=__start -nostartfiles -nodefaultlibs -T${MCU_LINKER_SCRIPT}")

* hellocpp/src/CMakeLists.txt

.. code-block:: cmake
   
    set(HEADER_FILES
            HelloWorld.h
    )

    set(SOURCE_FILES
            HelloWorld.cpp
    )

    link_directories(${EXE_NAME} ${NUTTX_PATH}/libs)

    add_executable(${EXE_NAME} ${SOURCE_FILES} main.cpp ${HEADER_FILES})

    add_custom_command(
            TARGET ${EXE_NAME}
            POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} ARGS -S -O binary ${CMAKE_BINARY_DIR}/${EXE_NAME}.elf ${CMAKE_BINARY_DIR}/${EXE_NAME}.bin
    )

    target_link_libraries(${EXE_NAME} --start-group)

    target_link_libraries(${EXE_NAME} sched)
    target_link_libraries(${EXE_NAME} drivers)
    target_link_libraries(${EXE_NAME} boards)
    target_link_libraries(${EXE_NAME} c)
    target_link_libraries(${EXE_NAME} mm)
    target_link_libraries(${EXE_NAME} arch)
    target_link_libraries(${EXE_NAME} xx)
    target_link_libraries(${EXE_NAME} apps)
    target_link_libraries(${EXE_NAME} fs)
    target_link_libraries(${EXE_NAME} binfmt)
    target_link_libraries(${EXE_NAME} board)
    target_link_libraries(${EXE_NAME} gcc)
    target_link_libraries(${EXE_NAME} supc++)

    target_link_libraries(${EXE_NAME} --end-group)

* hellocpp/src/main.cpp

.. code-block:: c++

    #include "HelloWorld.h"
    #include <nuttx/config.h>

    extern "C"
    {
            int hellocpp_main(void)
            {

                    CHelloWorld *pHelloWorld = new CHelloWorld();
                    pHelloWorld->HelloWorld();

                    CHelloWorld helloWorld;
                    helloWorld.HelloWorld();

                    delete pHelloWorld;
                    return 0;
            }
    }

* hellocpp/src/HelloWorld.h

.. code-block:: c++

    #ifndef HELLOWORLD_H_
    #define HELLOWORLD_H_

    #include "nuttx/config.h"

    class CHelloWorld
    {
            public:
                    CHelloWorld();
                    ~CHelloWorld();
                    bool HelloWorld(void);
            private:
                    int mSecret;
    };

    #endif

* hellocpp/src/HelloWorld.cpp

.. code-block:: c++

    #include <cstdio>
    #include <string>

    #include "HelloWorld.h"

    CHelloWorld::CHelloWorld() {
            mSecret = 42;
            std::printf("Constructor: mSecret=%d\n",mSecret);
    }

    CHelloWorld::~CHelloWorld() {

    }

    bool CHelloWorld::HelloWorld(void) {
            std::printf("HelloWorld: mSecret=%d\n",mSecret);

            std::string sentence = "Hello";
            std::printf("TEST=%s\n",sentence.c_str());

            if (mSecret == 42) {
                    std::printf("CHelloWorld: HelloWorld: Hello, world!\n");
                    return true;
            }
            else {
                    std::printf("CHelloWorld: HelloWorld: CONSTRUCTION FAILED!\n");
                    return false;
            }
    }

Building
--------

To launch build, you use the cmake procedure:

.. code-block:: console

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

And finally a bin file will be created to be loaded on the board.
