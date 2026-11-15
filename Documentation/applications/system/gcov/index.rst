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

        # By default, it is generated in the root directory of the vela project. Add parameters to specify the report generation location
        # The -t parameter specifies the gcov version, which needs to match the gcc version

        # sim
        ./tools/gcov.sh -t gcov-13

        # arm platform
        ./tools/gcov.sh -t arm-none-eabi-gcov

    5. Impact and precautions
       ```
       1. Before using the .tools/gcov.sh tool, you need to ensure that *.gcno and *.gcda files exist
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

      Please execute the following command, where

      1. xxxfile: the file for exporting data on the device
      2. xxxelf: the ELF file corresponding to the device

      # Convert the exported coverage data file
      llvm-profdata merge -sparse xxxfile -o result.profdata

      # Generate a visualization html file
      llvm-cov show -format=html xxxelf -instr-profile=result.profdata -output-dir=./coverage/html
