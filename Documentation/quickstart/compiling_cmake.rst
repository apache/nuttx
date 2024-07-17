====================
Compiling with CMake
====================

Initialize Configuration with CMake
===================================

The first step is to initialize NuttX configuration for a given board, based on
a pre-existing configuration. To list all supported configurations you can do:

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -L | less

The output is in the format ``<board name>:<board configuration>``. You will see that
generally all boards support the ``nsh`` configuration which is a good starting point
since it enables booting into the interactive command line
:doc:`/applications/nsh/index`.

To choose a configuration you pass the ``<board name>:<board configuration>`` such as:

    .. code-block:: console

       $ cd nuttx
       $ cmake -B build -DBOARD_CONFIG=stm32f4discovery:nsh -GNinja

The ``-B build`` tells what is the build directory.

You can then customize this configuration by using the menu based
configuration system with:

.. code-block:: console

   $ cd nuttx
   $ cmake --build build -t menuconfig 

Modifying the configuration is covered in :doc:`configuring`.

Build NuttX with CMake
======================

We can now build NuttX. To do so, you can simply run:

  .. code-block:: console

     $ cd nuttx
     $ cmake --build build 

The build will complete by generating the binary outputs
inside ``build/`` directory. Typically this includes the ``nuttx``
ELF file (suitable for debugging using ``gdb``) and a ``nuttx.bin``
file that can be flashed to the board.

To clean the build, you can do:

  .. code-block:: console

     $ cmake --build build -t clean

Out-of-tree building
====================

Key benefit of CMake is the out-of-tree building, which allows one to have different build folders for different configs, very proper if one need check multiple configs for the same codebase.  Out-of-tree means above ``build`` folders can be out of Nuttx source tree.

Suppose ``$NUTTX_DIR`` is the nuttx source tree, we can use temporary folder for a particular target config as shown below.

  .. code-block:: console

    $ echo $NUTTX_DIR
    /home/user/Projects/Nuttx/nuttx
    $ mkdir -p ~/tmp/rv32/nsh
    $ cd ~/tmp/rv32/nsh
    # Make sure a proper toolchain is in your $PATH
    $ riscv64-unknown-elf-gcc -v
    $ cmake $NUTTX_DIR -DBOARD_CONFIG=rv-virt:nsh -GNinja
    -- Initializing NuttX
    --   Board:  rv-virt
    --   Config: nsh
    --   Appdir: /home/yf/Projects/Nuttx/apps
    -- The C compiler identification is GNU 10.2.0
    -- The CXX compiler identification is GNU 10.2.0
    -- The ASM compiler identification is GNU
    -- Found assembler: /usr/bin/riscv64-unknown-elf-gcc
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/yf/tmp/rv32/nsh
    $ ninja
    $ size nuttx
       text    data      bss      dec      hex  filename
     167411      365    11568   179344    2bc90  nuttx

This approach works for FLAT configs now and PROTECTED configs soon if needed CMake scripts are available already.

Building KERNEL configs
=======================

We can use CMake to build the kernel image for KERNEL configs now, assuming apps ROMFS is prepared using the makefile system. If the development focus is kernel side and apps don't change often, then CMake can help us achieve out-of-tree build if your device's CMake scripts are ready. Let's take ``canm230`` device as an example:

  .. code-block:: console

    $ echo $NUTTX_DIR
    /home/user/Projects/Nuttx/nuttx
    $ mkdir -p ~/tmp/k230/nsbi
    # copy the romfs_boot.c to build folder
    $ cp romfs_boot.c ~/tmp/k230/nsbi
    $ cd ~/tmp/k230/nsbi
    $ ls -l
    total 976
    -rw-rw-r-- 1 yf yf 997843 Jul 15 06:23 romfs_boot.c
    $ cmake $NUTTX_DIR -DBOARD_CONFIG=canmv230:nsbi -GNinja
    -- Initializing NuttX
    --   Board:  canmv230
    --   Config: nsbi
    --   Appdir: /home/yf/Projects/Nuttx/apps
    -- The C compiler identification is GNU 10.2.0
    -- The CXX compiler identification is GNU 10.2.0
    -- The ASM compiler identification is GNU
    -- Found assembler: /usr/bin/riscv64-unknown-elf-gcc
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/yf/tmp/k230/nsbi
    $ ninja
    $ size nuttx
      text     data      bss      dec      hex  filename
    281671      609    37496   319776    4e120  nuttx

Note that for QEMU targets, we can directly use the apps binary on host folder via ``hostfs`` in QEMU.

So even apps side CMake support is not ready, we still can enjoy CMake for kernel build with KERNEL configs.
