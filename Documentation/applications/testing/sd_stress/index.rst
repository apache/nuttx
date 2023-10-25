================================================
``sd_stress`` SD card or mount point stress test
================================================

Performs stress testing on SD card or other mount points using the file system layer.

A single test run.

- Creates a staging directory
- Creates multiple files in this directory. Writing, reading and verifying a set of bytes from each one.
- Renames the staging directory.
- Remove the created files from the renamed directory.
- Remove the renamed directory.

The following runtime options are available::

  nsh> sdstress -h
  Stress test on a mount point
  sdstress: [-r] [-b] [-f]
    -r   Number of runs (1-10000), default 32
    -b   Number of bytes (1-10000), default 4096
    -f   Number of files (1-999), default 64


An example of a completed test::

  nsh> sdstress -b 4096 -f 32 -r 5
  Start stress test with 32 files, 4096 bytes and 5 iterations.
  iteration 0 took 4063.445 ms: OK
  iteration 1 took 4158.073 ms: OK
  iteration 2 took 4216.130 ms: OK
  iteration 3 took 4295.138 ms: OK
  iteration 4 took 4352.903 ms: OK
  Test OK: Average time: 4217.138 ms

The following Kconfig options can be used to configure the application at compile time.

- ``CONFIG_TESTING_SD_STRESS`` - Enable the stress test utility.
- ``CONFIG_TESTING_SD_STRESS_PROGNAME`` - The name of the program registered with nsh.
- ``CONFIG_TESTING_SD_STRESS_PRIORITY`` - The priority of the task.
- ``CONFIG_TESTING_SD_STRESS_STACKSIZE`` - The stacksize of the task.
- ``CONFIG_TESTING_SD_STRESS_STACKSIZE`` - The mountpoint of the filesystem to test.
