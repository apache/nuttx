.. include:: /substitutions.rst
.. _cpp_cmake:

=======================
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
===========

#. Base NuttX compilation changes

    For this example, load the configuration 'stm32f4discovery:nsh' for building (Linux host)

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -l stm32f4discovery:nsh

    See :ref:`quickstart/compiling_make:Initialize Configuration` for more information about configure.sh tool.

    In menuconfig, the main points to be changed on a typical NuttX configuration are the following:

    * RTOS Features -> Tasks and Scheduling -> Application entry point to 'main'
    * Library Routines -> Have C++ compiler
    * Library Routines -> Have C++ initialization -> C++ Library -> Toolchain C++ support (you can also choose the basic version or the LLVM one)
    * Library Routines -> Have C++ initialization -> C++ Library -> C++ low level library select -> GNU low level libsupc++
    * Library Routines -> Language standard -> choose the version you want - for this example we will use "c++17"
    * Library Routines -> Enable Exception Support -> to enable to support C++ exceptions - for this example we will select it
    * Library Routines -> Enable RTTI Support -> to enable to support C++ RTTI features (like dynamic_cast()/typeid()) - for this example we will not enable it


    Build NuttX and generate the export

    .. code-block:: console

       $ make export

Creating the project
====================

#. Create your project file structure

    The project structure is organized as follow:

    .. code-block:: console

       hellocpp/
       hellocpp/CMakeLists.txt
       hellocpp/nuttx-export-12.10.0/
       hellocpp/main.cpp
       hellocpp/HelloWorld.h
       hellocpp/HelloWorld.cpp

    The directory 'nuttx-export-12.10.0' is the unzipped content from the file created during
    make export procedure done before.

#. File contents

* hellocpp/CMakeLists.txt

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.12...3.31)

    project(HelloCpp
            VERSION 1.0
            DESCRIPTION "Hello world C++ NuttX"
    )

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    set(SOURCE_FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/HelloWorld.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    )

    set(EXE_NAME "hello")
    add_executable(${EXE_NAME} ${SOURCE_FILES})

    add_custom_command(
            TARGET ${EXE_NAME}
            POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} ARGS -S -O binary ${CMAKE_BINARY_DIR}/${EXE_NAME} ${CMAKE_BINARY_DIR}/${EXE_NAME}.bin


* hellocpp/main.cpp

.. code-block:: c++

    #include <memory>

    #include "HelloWorld.h"

    extern "C" int main(int, char*[])
    {
        auto pHelloWorld = std::make_shared<CHelloWorld>();
        pHelloWorld->HelloWorld();

        CHelloWorld helloWorld;
        helloWorld.HelloWorld();

        return 0;
    }


* hellocpp/HelloWorld.h

.. code-block:: c++

    #ifndef HELLOWORLD_H
    #define HELLOWORLD_H

    class CHelloWorld
    {
    public:
        CHelloWorld();
        ~CHelloWorld() = default;

        bool HelloWorld();

    private:
        int mSecret;
    };

    #endif

* hellocpp/HelloWorld.cpp

.. code-block:: c++

    #include <cstdio>
    #include <string>

    #include "HelloWorld.h"

    CHelloWorld::CHelloWorld()
    {
        mSecret = 42;
        std::printf("Constructor: mSecret=%d\n",mSecret);
    }


    bool CHelloWorld::HelloWorld()
    {
        std::printf("HelloWorld: mSecret=%d\n",mSecret);

        std::string sentence = "Hello";
        std::printf("TEST=%s\n",sentence.c_str());

        if (mSecret == 42)
        {
            std::printf("CHelloWorld: HelloWorld: Hello, world!\n");
            return true;
        }
        else
        {
            std::printf("CHelloWorld: HelloWorld: CONSTRUCTION FAILED!\n");
            return false;
        }
    }


Building
========

To launch build, you use the cmake procedure:

.. code-block:: console

    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_TOOLCHAIN_FILE=../nuttx-export-12.10.0/scripts/toolchain.cmake
    $ make

Two binaries are generated: an elf one - useful for debug purpose - and a binary one to be flashed on the board
