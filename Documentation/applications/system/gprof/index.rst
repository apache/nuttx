=============================
``gprof`` GNU Profile tool
=============================

GNU Profile (gprof) is a performance analysis tool that helps developers
identify code bottlenecks and optimize their programs.
It provides detailed information about the execution time and call
frequency of functions within a program.

gprof can be used to:

1. Detect performance bottlenecks in your code
2. Identify which functions consume the most execution time
3. Analyze the call graph of your program
4. Help prioritize optimization efforts

Usage
=====

QEMU example
------------
For this example, we're using **QEMU** and **aarch64-none-elf-gcc** with the **qemu-armv8a** board.

1. Configure ``./tools/configure.sh -E qemu-armv8a:nsh`` and make sure ``CONFIG_SYSTEM_GPROF`` and ``CONFIG_PROFILE_MINI`` are enabled
2. Build ``make -j``
3. Launch qemu::

    qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -chardev stdio,id=con,mux=on -serial chardev:con \
      -mon chardev=con,mode=readline -semihosting -kernel ./nuttx

4. Mount hostfs for saving data later::

    nsh> mount -t hostfs -o fs=. /mnt

5. Start profiling::

    nsh> gprof start

6. Do some test and stop profiling::

    nsh> gprof stop

7. Dump profiling data::

    nsh> gprof dump /mnt/gmon.out

8. Analyze the data on host using gprof tool::

    $ aarch64-none-elf-gprof nuttx gmon.out -b

.. note:: The saved file format complies with the standard gprof format.
  For detailed instructions on gprof command usage, please refer to the GNU gprof manual:
  https://ftp.gnu.org/old-gnu/Manuals/gprof-2.9.1/html_mono/gprof.html

Example output::

    $ aarch64-none-elf-gprof nuttx gmon.out -b
    Flat profile:

    Each sample counts as 0.001 seconds.
      %   cumulative   self              self     total
     time   seconds   seconds    calls   s/call   s/call  name
     75.58     12.44    12.44    12462     0.00     0.00  up_idle
     24.30     16.44     4.00        4     1.00     1.00  up_ndelay
      0.05     16.45     0.01      177     0.00     0.00  pl011_txint
      0.02     16.45     0.00       35     0.00     0.00  uart_readv

This output shows the performance profile of the program,
including execution time and call counts for each function.
The flat profile table provides a quick overview of where the program spends most of its time.
This information can be used to identify performance bottlenecks and optimize critical parts of the code.

Real board example
------------------
Let take **esp32s3-devkit** as an example.

Test the flat profile
~~~~~~~~~~~~~~~~~~~~~
1. Configure ``./tools/configure.sh -E esp32s3-devkit:nsh`` and make sure these items are enabled::

    # for gprof
    CONFIG_PROFILE_MINI=y
    CONFIG_SYSTEM_GPROF=y

    # save and transfer data
    CONFIG_FS_TMPFS=y
    CONFIG_SYSTEM_YMODEM=y

2. Build and flash ``make flash ESPTOOL_PORT=/dev/ttyUSB0 -j``
3. Run ``minicom -D /dev/ttyUSB0 -b 115200`` to connect to the board
4. Start profiling::

    nsh> gprof start

    # do some test here, such as ostest

    nsh> gprof stop
    nsh> gprof dump /tmp/gmon.out
    nsh> sb /tmp/gmon.out

5. Receive the file on PC, and analyze the data on host::

    $ cp nuttx nuttx_prof
    $ xtensa-esp32s3-elf-objcopy -I elf32-xtensa-le --rename-section .flash.text=.text nuttx_prof
    $ xtensa-esp32s3-elf-gprof nuttx_prof gmon.out

Test the call graph profile
~~~~~~~~~~~~~~~~~~~~~~~~~~~
1. Add compiler option ``-pg`` to the component, such as ostest Makefile, like: ``CFLAGS += -pg``
2. Enable the configuration item ``CONFIG_FRAME_POINTER``

The other steps are the same as the flat profile.
