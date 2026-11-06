====================================
Profile-Guided Optimization (PGO)
====================================

Overview
========

Profile-Guided Optimization (PGO) is a compiler optimization technique that uses
runtime profiling data to generate more efficient code. The optimization process
consists of two phases:

1. **Instrumentation Phase**: Build with profiling instrumentation to collect
   runtime execution data
2. **Optimization Phase**: Rebuild using the collected profile data to generate
   optimized code

PGO can significantly improve performance by:

- Optimizing hot code paths based on actual runtime behavior
- Improving branch prediction and code layout
- Reducing instruction cache misses
- Enabling better inlining decisions

Configuration
=============

Kconfig Options
---------------

To enable PGO, you need to configure the following options:

1. Enable code coverage for all modules:

   .. code-block:: kconfig

      CONFIG_COVERAGE_ALL=y

2. Select PGO mode instead of source-based coverage:

   .. code-block:: kconfig

      CONFIG_COVERAGE_PGO=y

3. Enable the gcov system utility to dump profile data:

   .. code-block:: kconfig

      CONFIG_SYSTEM_GCOV=y

These options can be set via ``menuconfig``:

.. code-block:: console

   $ make menuconfig

Navigate to:

.. code-block:: text

   Toolchain Library Support
     -> Enable code coverage for all module
         Coverage mode (Profile Guided Optimization)

   Application Configuration
     -> System Libraries and NSH Add-Ons
       -> gcov tool

Supported Toolchains
--------------------

Currently, PGO support is primarily implemented for **Clang/LLVM** and **GCC** toolchains.

Usage Workflow
==============

Phase 1: Collect Profile Data
------------------------------

1. **Configure with PGO instrumentation:**

   .. code-block:: console

      $ make menuconfig
      # Enable CONFIG_COVERAGE_ALL, CONFIG_COVERAGE_PGO, and CONFIG_SYSTEM_GCOV

2. **Build the system:**

   For CMake-based builds:

   .. code-block:: console

      $ cmake -B build -S nuttx/ -DBOARD_CONFIG=<your_board>
      $ cmake --build build

3. **Run the application and dump profile data:**

   Execute your application with representative workloads. The instrumented
   binary will collect profiling data internally. After the workload completes,
   use the ``gcov`` command to dump the profile data:

   .. code-block:: console

      $ # In NuttX shell (nsh)
      nsh> gcov -d /data/default.profraw
      # Or specify the output directory
      nsh> gcov -d /tmp/default.profraw

   The profile data will be written to the specified location. If using a
   filesystem like hostfs, you can directly access it from the host. Otherwise,
   use file transfer methods (TFTP, serial, etc.) to retrieve it.

4. **Transfer profile data to build directory:**

   Copy the ``default.profraw`` file from the target to your build directory:

   .. code-block:: console

      $ # If using hostfs, simply copy from mounted directory
      $ cp /path/to/hostfs/default.profraw .

      $ # Or use other file transfer methods
      $ scp target:/data/default.profraw .
      $ # or via serial, TFTP, etc.

5. **Verify profile data:**

   Check that ``default.profraw`` has been successfully transferred to the
   build directory:

   .. code-block:: console

      $ ls -lh default.profraw

Phase 2: Build with Profile Data
---------------------------------

1. **Keep the profile data:**

   Ensure ``default.profraw`` from Phase 1 is present in the build directory.
   The file must be transferred from the target device to your host build
   directory (e.g., via hostfs, serial transfer, or other methods).

2. **Rebuild:**

   Simply rebuild the project. The build system will automatically:

   - Detect the presence of ``default.profraw``
   - Merge profile data using ``llvm-profdata``
   - Generate ``default.profdata``
   - Reconfigure the build with ``-fprofile-use=<path>/default.profdata``
   - Compile with optimizations based on profile data

   For CMake:

   .. code-block:: console

      $ llvm-profdata merge *.profraw -o default.profdata
      $ cmake --build build
      # CMake will automatically reconfigure and rebuild with profile data

3. **Deploy optimized binary:**

   The resulting binary is now optimized based on the collected profile data.

Implementation Details
======================

CMake Implementation
--------------------

For CMake-based builds, the PGO logic is implemented in
``nuttx/cmake/nuttx_pgo.cmake``. The ``nuttx_add_pgo_flags()`` function:

1. Checks for ``${CMAKE_BINARY_DIR}/default.profraw``
2. If found, executes: ``llvm-profdata merge -output=default.profdata *.profraw``
3. Adds ``-fprofile-use=<path>`` if merge succeeds
4. Otherwise, adds ``-fprofile-generate`` for data collection

Profile Data Format
-------------------

- **`.profraw`**: Raw profile data generated during program execution
- **`.profdata`**: Indexed profile data created by ``llvm-profdata merge``

The ``.profdata`` format is used by the compiler for optimization.

Best Practices
==============

Representative Workloads
------------------------

For best results, run workloads that are representative of actual usage patterns:

- Execute all critical code paths
- Include typical input data
- Run for sufficient duration to capture steady-state behavior
- Include edge cases if they're common in production

Iterative Optimization
----------------------

You can iterate the PGO process multiple times:

1. Build with ``-fprofile-generate``
2. Run and collect new profile data
3. Rebuild with ``-fprofile-use``
4. If needed, collect profile data from the optimized binary
5. Rebuild again with updated profile data

Profile Data Management
-----------------------

- Store profile data (``default.profraw``) separately for different configurations
- Use version control to track profile data for reproducible builds
- Update profile data when workload patterns change significantly

Clean Builds
------------

To start fresh:

.. code-block:: console

   $ rm -f default.profraw default.profdata
   $ make clean
   $ make

Limitations
===========

- **Cross-compilation**: Profile data must be collected on target or simulator
- **Binary Size**: Instrumented binaries are larger and slower
- **Profile Validity**: Profile data may become stale as code changes
- **GCC Concurrency Issues**: When using GCC for PGO, the ``__gcov_indirect_call``
  global variable has critical concurrency issues in both SMP and single-CPU
  environments. GCC directly injects code to access this variable without protection
  mechanisms, leading to race conditions in SMP systems and data corruption from
  interrupt handlers in single-CPU systems. Consider using Clang/LLVM PGO instead,
  as it does not rely on global variables for indirect call profiling

Examples
========

Example: CMake Build with Hostfs
---------------------------------

.. code-block:: console

   # Phase 1: Collect profile data
   $ cmake -B build -DBOARD_CONFIG=qemu-armv8a:nsh
   $ cmake --build build
   $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 -kernel build/nuttx -semihosting
   # In NuttX shell:
   # nsh> mount -t hostfs -o fs=/path/to/host/build /mnt
   # nsh> <run your workload>
   # nsh> gcov -d /mnt/default.profraw
   # Exit QEMU - profile data is now in build directory

   # Phase 2: Optimized build
   $ cmake --build build
   # Build system detects default.profraw and rebuilds with optimizations
   $ # Deploy optimized build/nuttx.bin

See Also
========

- LLVM PGO Documentation: https://clang.llvm.org/docs/UsersManual.html#profile-guided-optimization
