``i2cchar`` Transfer Through I2C
================================

A mindlessly simple test of an I2C driver. It reads an write garbage data to the
I2C transmitter and/or received as fast possible.

This test depends on these specific I2S/AUDIO/NSH configurations settings (your
specific I2S settings might require additional settings).

- ``CONFIG_I2S`` – Enabled I2S support.
- ``CONFIG_AUDIO`` – Enabled audio support.
- ``CONFIG_DRIVERS_AUDIO`` – Enable audio device support.
- ``CONFIG_AUDIO_I2SCHAR`` – Enabled support for the I2S character device.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the I2S test as an NSH built-in function.
  Default: Built as a standalone program.

Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_I2SCHAR`` – Enables the I2C test.

- ``CONFIG_EXAMPLES_I2SCHAR_DEVPATH`` – The default path to the ADC device.
  Default: ``/dev/i2schar0``.

- ``CONFIG_EXAMPLES_I2SCHAR_TX`` – This should be set if the I2S device supports a
  transmitter.

- ``CONFIG_EXAMPLES_I2SCHAR_TXBUFFERS`` – This is the default number of audio
  buffers to send before the TX transfers terminate. When both TX and RX
  transfers terminate, the task exits (and, if an NSH builtin, the ``i2schar``
  command returns). This number can be changed from the NSH command line.

- ``CONFIG_EXAMPLES_I2SCHAR_TXSTACKSIZE`` – This is the stack size to use when
  starting the transmitter thread. Default ``1536``.

- ``CONFIG_EXAMPLES_I2SCHAR_RX`` – This should be set if the I2S device supports a
  transmitter.

- ``CONFIG_EXAMPLES_I2SCHAR_RXBUFFERS`` – This is the default number of audio
  buffers to receive before the RX transfers terminate. When both TX and RX
  transfers terminate, the task exits (and, if an NSH builtin, the ``i2schar``
  command returns). This number can be changed from the NSH command line.

- ``CONFIG_EXAMPLES_I2SCHAR_RXSTACKSIZE`` – This is the stack size to use when
  starting the receiver thread. Default ``1536``.

- ``CONFIG_EXAMPLES_I2SCHAR_BUFSIZE`` – The size of the data payload in one audio
  buffer. Applies to both TX and RX audio buffers.

- ``CONFIG_EXAMPLES_I2SCHAR_DEVINIT`` – Define if architecture-specific I2S device
  initialize is available. If defined, the platform specific code must provide a
  function ``i2schar_devinit()`` that will be called each time that this test
  executes. Not available in the kernel build mode.
