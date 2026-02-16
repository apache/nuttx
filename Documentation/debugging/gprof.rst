============================
GNU gprof Profiling Tool
============================

Overview
--------

gprof is a performance profiling tool that helps developers analyze runtime performance,
identify performance hotspots, and understand function call relationships in their programs.
NuttX integrates gprof support through sampling and function call tracing to generate
detailed performance analysis reports.

Features
--------

gprof provides the following key capabilities:

1. **Flat Profile**: Displays execution time distribution across functions
2. **Call Graph**: Shows call relationships and time distribution between functions
3. **Function Statistics**: Provides detailed metrics including call counts, cumulative time, and self time

Configuration
-------------

Required Configuration Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To enable gprof functionality, add the following options to your kernel configuration::

    CONFIG_FRAME_POINTER=y
    CONFIG_PROFILE_MINI=y
    CONFIG_SYSTEM_GPROF=y

Optional Configuration
~~~~~~~~~~~~~~~~~~~~~~

- ``CONFIG_PROFILE_ALL=y``: Enables full profiling with call graph information

Configuration Details
~~~~~~~~~~~~~~~~~~~~~

- ``CONFIG_FRAME_POINTER``: Enables frame pointer for stack unwinding
- ``CONFIG_PROFILE_MINI``: Enables lightweight profiling based on timer sampling, recording only function execution time
- ``CONFIG_SYSTEM_GPROF``: Enables the gprof command-line tool
- ``CONFIG_PROFILE_ALL``: Enables complete function call graph analysis (optional, increases performance overhead). Records function call relationships. If you only need call graph analysis for specific modules, you can skip this option and instead add the ``-pg`` compiler flag to the module's Makefile or CMakeLists.txt

Basic Usage
-----------

Example: CoreMark Performance Analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following demonstrates profiling the CoreMark benchmark test.

**Step 1: Configure and Build**::

    $ ./tools/configure.sh qemu-armv7a/nsh
    $ make -j
    $ qemu-system-arm -cpu cortex-a7 -nographic \
     -machine virt,virtualization=off,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx -s | tee gprof.log

**Step 2: Run Profiling**::

    nsh> gprof start
    nsh> coremark
    nsh> gprof stop
    nsh> gprof dump /tmp/gmon.out
    nsh> hexdump /tmp/gmon.out

**Step 3: Analyze on Host**::

    $ grep -E "^[0-9a-f]+:" gprof.log | xxd -r > gmon.out
    $ arm-none-eabi-gprof nuttx gmon.out -b

    Flat profile:

    Each sample counts as 0.001 seconds.
      %   cumulative   self              self     total
     time   seconds   seconds    calls  Ts/call  Ts/call  name
     41.90     16.93    16.93                             up_idle
     20.61     25.26     8.33                             core_state_transition
      5.21     27.36     2.11                             core_list_find
      4.61     29.22     1.86                             core_list_reverse
      4.49     31.04     1.81                             core_bench_list
      3.64     34.18     1.47                             matrix_mul_matrix
      3.16     35.46     1.28                             coremark_crcu8
      ...

**Interpreting the Results:**

- ``up_idle`` accounts for 41.90% of execution time, indicating the system spends most time in idle state
- ``core_state_transition`` consumes 20.61%, representing the most time-intensive function in CoreMark
- Other performance hotspots include list operations (``core_list_find``, ``core_list_reverse``) and matrix operations (``matrix_mul_matrix``)

Example: Call Graph Analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following demonstrates call graph analysis with ``CONFIG_PROFILE_ALL`` enabled.

**Step 1: Configure and Build**::

    $ ./tools/configure.sh mps2-an500/nsh
    $ make -j
    $ qemu-system-arm -M mps2-an500 -cpu cortex-m7 -nographic -kernel ./nuttx -s | tee gprof.log

**Step 2: Run Profiling**::

    nsh> gprof start
    nsh> sleep 1
    nsh> gprof stop
    nsh> gprof dump /tmp/gmon.out
    nsh> hexdump /tmp/gmon.out

**Step 3: Analyze on Host**::

    $ grep -E "^[0-9a-f]+:" gprof.log | xxd -r > gmon.out
    $ arm-none-eabi-gprof nuttx/nuttx gmon.out -b
    Call graph

    granularity: each sample hit covers 4 byte(s) for 0.10% of 1.00 seconds

    index % time    self  children    called     name
    -----------------------------------------------
                    0.00    0.00    2066/2066        irq_dispatch [9]
    [5]      0.0    0.00    0.00    2066         perf_gettime [5]
                    0.00    0.00    2066/2066        up_perf_gettime [6]
    -----------------------------------------------
                    0.00    0.00    2066/2066        perf_gettime [5]
    [6]      0.0    0.00    0.00    2066         up_perf_gettime [6]
    -----------------------------------------------
                    0.00    0.00    1007/2017        systick_interrupt [21]
                    0.00    0.00    1010/2017        systick_getstatus [13]
    [7]      0.0    0.00    0.00    2017         systick_is_running [7]
    -----------------------------------------------

**Interpreting the Call Graph:**

The example above shows the complete call chain::

    irq_dispatch [9]
      └─> perf_gettime [5]
            └─> up_perf_gettime [6]

For detailed call graph output interpretation, refer to the gprof manual: https://sourceware.org/binutils/docs/gprof/Call-Graph.html

Profiling Individual Modules
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you prefer not to enable ``CONFIG_PROFILE_ALL`` system-wide (to reduce performance overhead),
you can profile specific modules by adding the ``-pg`` compiler flag to the module's build configuration.

**Adding -pg to Makefile**::

    # Enable -pg for the entire directory
    CFLAGS += -pg

**Adding -pg to CMakeLists.txt**::

    target_compile_options(mymodule PRIVATE -pg)

This approach allows precise profiling of critical modules while maintaining overall system performance.

Recovering Data from Serial Console
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you cannot directly export files from the device, you can recover the data through
serial console xxd output and reconstruct it on the host:

**Step 1: Display hexadecimal data on device**::

    nsh> hexdump /tmp/gmon.out

**Step 2:** Save the serial console output to a log file (e.g., ``gprof.log``)

**Step 3: Convert xxd output to binary using xxd -r**::

    $ grep -E "^[0-9a-f]+:" gprof.log | xxd -r > gmon.out

**Step 4: Analyze the converted file with gprof**::

    $ arm-none-eabi-gprof nuttx/nuttx gmon.out -b

Real Board Examples
-------------------

QEMU aarch64 Example
~~~~~~~~~~~~~~~~~~~~

This example uses **QEMU** and **aarch64-none-elf-gcc** with the **qemu-armv8a** board.

**Step 1: Configure and Build**::

    $ ./tools/configure.sh -E qemu-armv8a:nsh
    # Ensure CONFIG_SYSTEM_GPROF and CONFIG_PROFILE_MINI are enabled
    $ make -j

**Step 2: Launch QEMU**::

    $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -chardev stdio,id=con,mux=on -serial chardev:con \
      -mon chardev=con,mode=readline -semihosting -kernel ./nuttx

**Step 3: Mount hostfs for saving data**::

    nsh> mount -t hostfs -o fs=. /mnt

**Step 4: Run profiling**::

    nsh> gprof start
    # Do some test here
    nsh> gprof stop
    nsh> gprof dump /mnt/gmon.out

**Step 5: Analyze on host**::

    $ aarch64-none-elf-gprof nuttx gmon.out -b

ESP32-S3 Example with Ymodem Transfer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This example demonstrates profiling on **esp32s3-devkit** with data transfer via Ymodem.

**Step 1: Configure and Build**::

    $ ./tools/configure.sh -E esp32s3-devkit:nsh
    # Enable the following options:
    # CONFIG_PROFILE_MINI=y
    # CONFIG_SYSTEM_GPROF=y
    # CONFIG_FS_TMPFS=y
    # CONFIG_SYSTEM_YMODEM=y
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0 -j

**Step 2: Connect to board**::

    $ minicom -D /dev/ttyUSB0 -b 115200

**Step 3: Run profiling on device**::

    nsh> gprof start
    # Do some test here, such as ostest
    nsh> gprof stop
    nsh> gprof dump /tmp/gmon.out
    nsh> sb /tmp/gmon.out

**Step 4: Receive file and analyze on host**::

    # Receive file via Ymodem in minicom, then:
    $ cp nuttx nuttx_prof
    $ xtensa-esp32s3-elf-objcopy -I elf32-xtensa-le --rename-section .flash.text=.text nuttx_prof
    $ xtensa-esp32s3-elf-gprof nuttx_prof gmon.out

.. note:: For Xtensa targets, the ``objcopy --rename-section`` step is required
   because the text section has a different name (``.flash.text``).

Important Notes
---------------

- ``CONFIG_PROFILE_ALL`` significantly increases performance overhead and memory usage. Enable only when call graph analysis is required.
- For simulator environments, use ``CONFIG_SIM_PROFILE`` to enable gprof functionality.
- On ARM Cortex-M v6/v7/v8, the Flat Profile functionality has limitations and requires modification of ``_vectors`` to capture the thread PC pointer during interrupts.

References
----------

- GNU gprof Manual: https://sourceware.org/binutils/docs/gprof/
