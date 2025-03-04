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

Build
-----

Enable the following configuration in NuttX::

  CONFIG_SYSTEM_GPROF

Using in NuttX
--------------

1. Start profiling::

     nsh> gprof start

2. Stop profiling::

     nsh> gprof stop

3. Dump profiling data::

     nsh> gprof dump /tmp/gmon.out

Analyzing on Host
-----------------

1. Pull the profiling data to host::

     adb pull /tmp/gmon.out ./gmon.out

2. Analyze the data using gprof tool::

     The saved file format complies with the standard gprof format.
     For detailed instructions on gprof command usage, please refer to the GNU gprof manual:
     https://ftp.gnu.org/old-gnu/Manuals/gprof-2.9.1/html_mono/gprof.html

     arm-none-eabi-gprof ./nuttx/nuttx gmon.out -b

     Example output:

     ```
     arm-none-eabi-gprof nuttx/nuttx gmon.out -b
     Flat profile:

     Each sample counts as 0.001 seconds.
       %   cumulative   self              self     total
      time   seconds   seconds    calls   s/call   s/call  name
      66.41      3.55     3.55       43     0.08     0.08  sdelay
      33.44      5.34     1.79       44     0.04     0.04  delay
       0.07      5.34     0.00                             up_idle
       0.04      5.34     0.00                             nx_start
       0.02      5.34     0.00                             fdtdump_main
       0.02      5.34     0.00                             nxsem_wait
       0.00      5.34     0.00        1     0.00     5.34  hello_main
       0.00      5.34     0.00        1     0.00     0.00  singal_handler

     ```

     This output shows the performance profile of the program,
     including execution time and call counts for each function.
     The flat profile table provides a quick overview of where the program spends most of its time.
     In this example, `sdelay` and `delay` functions consume the majority of execution time.
     This information can be used to identify performance bottlenecks and optimize critical parts of the code.
