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
    threads are created. Each of the threads searches for prime numbers in the
    configurable range, doing that configurable number of times. This value
    specifies the end of search range and together with number of runs allows to
    configure the length of this test – it should last at least a few tens of
    seconds. Allowed values ``[1; 32767]``, default ``10000``.

- ``CONFIG_TESTING_OSTEST_RR_RUNS`` – During round-robin scheduling test two
    threads are created. Each of the threads searches for prime numbers in the
    configurable range, doing that configurable number of times.
