================================================
``sd_bench`` SD card or mount point bench test
================================================

Performs bench mark testing on SD card or other mount points using the file system layer.

A single test run.

- Sequentially writes blocks of bytes to a test file on the device until the test duration elapses.
- Optionally, the number of bytes written are read back and verified.

The following runtime options are available::

  sdbench: [-b] [-r] [-d] [-k] [-s] [-a] [-v]
    -b   Block size per write (1-65536), default 512
    -r   Number of runs (1-10000), default 5
    -d   Max duration of a test (ms) (1-60000), default 2000
    -k   Keep test file when finished, default false
    -s   Call fsync after each block, false calls fsync
         only at the end of each run, default false
    -a   Test performance on aligned data, default false
    -v   Verify data and block number, default true

An example of a completed test::

  nsh> sdbench
  Using block size = 512 bytes, sync = false

  Testing Sequential Write Speed...
    Run  1:    345.9 KB/s, max write time: 156.907 ms (3.2 KB/s), fsync: 259.687 ms
    Run  2:    378.8 KB/s, max write time: 30.273 ms (16.5 KB/s), fsync: 240.832 ms
    Run  3:    372.1 KB/s, max write time: 37.630 ms (13.3 KB/s), fsync: 261.005 ms
    Run  4:    341.7 KB/s, max write time: 186.352 ms (2.7 KB/s), fsync: 240.875 ms
    Run  5:    375.6 KB/s, max write time: 37.785 ms (13.2 KB/s), fsync: 250.928 ms
    Avg   :    362.8 KB/s, 3.999 MB written.

  Testing Sequential Read Speed...
    Run  1:    636.5 KB/s, max read/verify time: 54.1180 ms (9.2 KB/s)
    Run  2:    648.9 KB/s, max read/verify time: 54.0520 ms (9.3 KB/s)
    Run  3:    663.2 KB/s, max read/verify time: 43.5360 ms (11.5 KB/s)
    Run  4:    721.8 KB/s, max read/verify time: 11.7640 ms (42.5 KB/s)
    Avg   :    652.6 KB/s, 3.999 MB and verified


The following Kconfig options can be used to configure the application at compile time.

- ``CONFIG_TESTING_SD_BENCH`` - Enable the SD benchmark testing utility.
- ``CONFIG_TESTING_SD_BENCH_PROGNAME`` - The name of the program registered with nsh.
- ``CONFIG_TESTING_SD_BENCH_PRIORITY`` - The priority of the task.
- ``CONFIG_TESTING_SD_BENCH_STACKSIZE`` - The stacksize of the task.
- ``CONFIG_TESTING_SD_BENCH_DEVICE`` - The mountpoint to run sdbench on.
