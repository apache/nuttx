=======
Testing
=======

The ``pthread_mutex_perf`` is a simple performance test to validate if some
specific kernel modification impacted the pthread_mutex_trylock().

Basically the test will run a busy-wait 1000000 trying to acquire a mutex
that is already locked. Then it will calculate the total time and the
average time, repeating this cycle 10 times.

This test is not intended to confirm that pthread mutex is working, for that
it is better to use ostest. Its goal is to verify impact in the kernel that
could be considered a regression.

