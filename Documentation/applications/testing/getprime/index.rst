======================
``getprime`` benchmark
======================

This application is used to benchmark processing time using multithreading. Each
thread that is spawned will find all prime numbers in the range of 1 to 10,000.

To use ``getprime``, pass the number of threads you'd like to run as the only
argument. After the threads finish processing, the duration of the test is
printed to the console.

.. code:: console

   nsh> getprime 3
   Set thread priority to 10
   Set thread policy to SCHED_RR
   Start thread #0
   Start thread #1
   Start thread #2
   thread #0 started, looking for primes < 10000, doing 10 run(s)
   thread #1 started, looking for primes < 10000, doing 10 run(s)
   thread #2 started, looking for primes < 10000, doing 10 run(s)
   thread #0 finished, found 1230 primes, last one was 9973
   thread #1 finished, found 1230 primes, last one was 9973
   thread #2 finished, found 1230 primes, last one was 9973
   Done
   getprime took 89040 msec

This program can be used to see performance differences between boards,
single-core vs SMP, thread scheduling settings, etc.
