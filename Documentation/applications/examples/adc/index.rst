=====================
``adc`` Read from ADC
=====================

A mindlessly simple test of an ADC devices. It simply reads from the ADC device
and dumps the data to the console forever.

This test depends on these specific ADC/NSH configurations settings (your
specific ADC settings might require additional settings).

- ``CONFIG_ADC`` – Enabled ADC support.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the ADC test as an NSH built-in function.
  Default: Built as a standalone program.

Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_ADC_DEVPATH`` – The default path to the ADC device. Default:
  ``/dev/adc0``.
- ``CONFIG_EXAMPLES_ADC_NSAMPLES`` – This number of samples is collected and the
  program terminates. Default: Samples are collected indefinitely.
- ``CONFIG_EXAMPLES_ADC_GROUPSIZE`` – The number of samples to read at once.
  Default: ``4``.

