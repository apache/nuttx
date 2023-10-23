=========================
``random`` Random Numbers
=========================

This is a very simply test of ``/dev/random``. It simple collects random numbers
and displays them on the console.

Prerequistes:

- ``CONFIG_DEV_RANDOM`` – Support for ``/dev/random`` must be enabled in order to
  select this example.

Configuration:

- ``CONFIG_EXAMPLES_RANDOM`` – Enables the ``/dev/random`` test.
- ``CONFIG_EXAMPLES_MAXSAMPLES`` – This is the size of the ``/dev/random`` I/O
  buffer in units of 32-bit samples. Careful! This buffer is allocated on the
  stack as needed! Default ``64``.
- ``CONFIG_EXAMPLES_NSAMPLES`` – When you execute the ``rand`` command, a number of
  samples ranging from ``1`` to ``EXAMPLES_MAXSAMPLES`` may be specified. If no
  argument is specified, this is the default number of samples that will be
  collected and displayed. Default ``8``.
