``serloop`` Serial Loopback
===========================

This is a mindlessly simple loopback test on the console. Useful for testing new
serial drivers. Configuration options include:

- ``CONFIG_EXAMPLES_SERLOOP_BUFIO`` – Use C buffered I/O (``getchar``/``putchar``) vs.
   raw console I/O (read/read).
