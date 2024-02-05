===============================
``qencoder`` Quadrature Encoder
===============================

This example is a simple test of a Quadrature Encoder driver. It simply reads
positional data from the encoder and prints it.,

This test depends on these specific QE/NSH configurations settings (your
specific PWM settings might require additional settings).

- ``CONFIG_SENSORS_QENCODER`` – Enables quadrature encoder support (upper-half
  driver).
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the QE test as an NSH built-in function.
  Default: Built as a standalone program.

Additional configuration options will mostly likely be required for the board-
specific lower-half driver. See the documentation page for your board.


Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_QENCODER_DEVPATH`` – The path to the QE device. Default:
  ``/dev/qe0``.
- ``CONFIG_EXAMPLES_QENCODER_NSAMPLES`` – This number of samples is collected and
  the program terminates. Default: Samples are collected indefinitely.
- ``CONFIG_EXAMPLES_QENCODER_DELAY`` – This value provides the delay (in
  milliseconds) between each sample. Default: ``100`` milliseconds.
