==================
``gcov`` gcov tool
==================

gcov is a tool for testing code coverage.
After the program is run, you can view the line coverage, function coverage,
and branch coverage of each file.

Support
-------

The current system supports four code coverage detection implementations:

1. GCC native implementation
2. CLANG native implementation
3. GCC coverage nuttx mini version
4. CLANG coverage nuttx mini version

The following table shows the specific differences between the four implementations
ends 24.11.26：

Support            GCC-native  CLANG-native     GCC-nuttx-mini      CLANG-nuttx-mini
Compiler version:     ALL          ALL        GCC 13.2 and below   CLANG 17.0 and below

Program coverage statistics support:

Main Program           √            √                 √                     √
Interrupt Program      ×            √                 √                     √

Architecture Support:

    arm                √            √                 √                     √
    arm64              √            √                 √
    riscv              √            √
    X86_X64            √            √
    xtensa             √            √
    sim                √            √

Usage
=====

App-Usage
---------
Usage::

    gcov [-d path] [-t strip] [-r] [-h]

Where:

  -d dump the coverage, path is the path to the coverage file, the default output is to stdout
  -t strip the path prefix number
  -r reset the coverage
  -h show this text and exits.

Examples of applicable platforms
--------------------------------

1. SIM platform usage

  1. Please enable the following config

    # Support instrumentation of all codes in sim
    1. CONFIG_COVERAGE_TOOLCHAIN=y

    # Enable instrumentation
    2. CONFIG_COVERAGE_ALL=y

    # Enable gcov app
    3. CONFIG_SYSTEM_GCOV=y

  2. Compile and run
     ```
     $ After the code is compiled, a *.gcno file with the same name will be generated next to the *.o file.
     $ After the compilation is completed, run the code to be tested, and exit the sim after the execution is completed
     ```

  3. Run the gcov app
     ```
     $ gcov -d path_to_gcno_file
     ```

  4. Check whether the generation is successful
     ```
     Execute the following command in the project root directory to check whether there is a gcda file generated (code coverage data)
     find ./ -name "*.gcno"
     find ./ -name "*.gcda"
     ```

  5. Examples:
     ```
     $ ./tools/configure.sh sim:nsh
     nsh: poweroff
     $ ./tools/gcov.py -e nuttx -s .
     Then open the report link printed by the tool
     (file://<abs path>/gcov/report/index.html) with your browser
     ```

     ``-a`` defaults to the current directory and ``-o`` to ``gcov``.

2. Applicable to device

  Due to differences in implementation methods, the device side is divided into GCC and CLANG

  1. GCC

    1. Please enable the following config

      # Recommended to use nuttx mini version
      CONFIG_COVERAGE_MINI=y

      # Enable gcov app
      CONFIG_SYSTEM_GCOV=y

      # Please add different compilation options in makefile according to the compiler
      CFLAGS += -fprofile-arcs -ftest-coverage -fno-inline

    2. Run the gcov app
       ```
       $ gcov -d path_to_gcno_file
       ```

    3. Export data

      After running the code on the device,
      execute the gcov -d /tmp/gcov command in the nuttx command line
      to save the generated data to the file system.
      You need to use your method to export the file to the host

    4. Generate Report

      1. Install the tool

        sudo apt install lcov

      2. Generate report

        Run the following command to generate coverage report

        # The toolchain (gcc or clang) and the target architecture are
        # detected from the ELF passed with -e, so the same command works
        # on sim and on cross targets. -s is the directory holding the
        # \*.gcno files (the build directory), -a the exported data.

        # sim / arm / other targets
        ./tools/gcov.py -e nuttx -s . -a path_to_exported_data

        # If the sources were built in another tree, map the recorded
        # paths, e.g. -b /build/root=/home/me/nuttx

    5. Impact and precautions
       ```
       1. Before using the tools/gcov.py tool, you need to ensure that *.gcno and *.gcda files exist
       2. If *.gcno does not exist, recompile the code after distclean.
       3. If *.gcda does not exist, please check and use poweroff to exit Vela normally
       ```

  2. CLANG

    # There is a ready-made defconfig in NXboards/arm/mps/mps3-an547/configs/gcov
    # which can be used for reference

    1. Please enable the following config

      # Recommended to use nuttx mini version
      CONFIG_COVERAGE_MINI=y

      # Enable gcov app
      CONFIG_SYSTEM_GCOV=y

      # Please add different compilation options in makefile according to the compiler
      CFLAGS += -fprofile-instr-generate -fcoverage-mapping

    2. Modify the linker script

      Please find the corresponding storage location in the link
      script for the following data：

      For detailed examples, please refer to boards/arm/mps/mps3-an547/scripts/flash.ld

      .. code-block:: none

          __llvm_prf_names : {
              __start__llvm_prf_names = .;
              KEEP (*(__llvm_prf_names))
              __end__llvm_prf_names = .;
          }

          __llvm_prf_data : {
              __start__llvm_prf_data = .;
              KEEP (*(__llvm_prf_data))
              __end__llvm_prf_data = .;
          }

          __llvm_prf_vnds : {
              __start__llvm_prf_vnds = .;
              KEEP (*(__llvm_prf_vnds))
              __end__llvm_prf_vnds = .;
          }

          __llvm_prf_cnts : {
              __start__llvm_prf_cnts = .;
              KEEP (*(__llvm_prf_cnts))
              __end__llvm_prf_cnts = .;
          }


    3. Run the gcov app

      ```
      $ gcov -d path_to_gcno_file
      ```

    4. Export data

    5. Generate Report

      The same command as for GCC produces the report: the ELF given with
      -e tells gcov.py that the data is clang source-based coverage, the
      exported \*.profraw files are passed with -a, and llvm-profdata and
      llvm-cov are driven automatically.

      ```
      $ ./tools/gcov.py -e nuttx -a path_to_exported_data
      ```

      Pass ``--llvm-html`` to render with ``llvm-cov show`` instead of
      genhtml, which keeps region-level highlighting at the cost of a
      report that differs from the GCC one.
