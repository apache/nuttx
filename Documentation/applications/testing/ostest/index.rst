.. _ostest:

==================
``ostest`` OS test
==================

This is the NuttX _qualification_ suite. It attempts to exercise a broad set of
OS functionality. Its coverage is not very extensive as of this writing, but it
is used to qualify each NuttX release.

The behavior of the ``ostest`` can be modified with the following settings in the
``boards/<arch>/<chip>/<board>/configs/<config>/defconfig`` file:

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the OS test example as an NSH built-in
    application.
- ``CONFIG_TESTING_OSTEST_LOOPS`` – Used to control the number of executions of
    the test. If undefined, the test executes one time. If defined to be zero,
    the test runs forever.

- ``CONFIG_TESTING_OSTEST_STACKSIZE`` – Used to create the ostest task. Default is
    ``8192``.
- ``CONFIG_TESTING_OSTEST_NBARRIER_THREADS`` – Specifies the number of threads to
    create in the barrier test. The default is 8 but a smaller number may be
    needed on systems without sufficient memory to start so many threads.

- ``CONFIG_TESTING_OSTEST_RR_RANGE`` – During round-robin scheduling test two
    threads are created. Each of the threads searches for prime numbers below
    this value, one number at a time, and starts again at 1 at the end of the
    range. The time for one number must be much shorter than
    ``CONFIG_RR_INTERVAL``. Allowed values ``[1; 32767]``, default ``30000``.
    A slow board can set this value lower in its defconfig.

- ``CONFIG_TESTING_OSTEST_RR_RUNS`` – The number of time slices of each thread
    in the round-robin scheduling test. The test takes about
    ``2 * CONFIG_TESTING_OSTEST_RR_RUNS * CONFIG_RR_INTERVAL`` milliseconds. It
    passes when the two threads had the same number of time slices, plus or
    minus one. Allowed values ``[1; 32767]``, default ``10``.
