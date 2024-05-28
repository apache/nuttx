========================
Avia Semiconductor HX711
========================

Driver contributed by Michał Łyszczek.

HX711 is a 24bit ADC (Analog Digital Converter) designed for weight scales.
This chip can be very slow. With internal oscillator and RATE pin pulled
down, it outputs only 10 samples per second. To not hog down CPU, driver
uses interrupt to detect when chip is ready. This will make read(2) blocking,
but system can do whatever it needs before chip is ready. Because of that
driver does not fully follow ADC API, but rather standard character device
(read only).

Values from tensometer can be easily read from shell with ``cat`` command

.. code-block::

   cat /dev/hxx711_0

Altough it may be better to dump values with example ``hx711`` program,
since ``cat`` will just read until the end of time, and if ctrl+c is
not working, it will steal shell forever.

-------
reading
-------

Reading is done by calling standard, posix, read(2) function. Only one value
can be returned with single call to read(2). But an averaging function can
be enabled, so that driver will read N samples, average them, and then will
return single averaged value.

This function accepts two types of buffer.

If buffer is of size ``sizeof(int32_t)`` a int32 value will be stored in
a buffer. If buffer size of bigger than ``sizeof(int32_t)`` function will
store string representation of values in passed buffer.

Simple code to read and print value may look like this

.. code-block:: C

   int fd;
   fd = open("/dev/hx711_0", O_RDONLY);

   for (; ; )
     {
       int32_t value;
       value = read(fd, &value, sizeof(value));
       printf("Read: %"PRIi32"\n", value);
     }

-----
ioctl
-----

Since this chip (and driver) is designed for weight scale, kernel driver
can provide some processing to make life easier for userspace code. These
functions are implemented via ioctl(2) commands. In practice, non of these
can be used, but if you just open driver and read it, you will get raw
values from hx711 chip, which you will have to process yourself. If your
needs are more standard, it's better to use kernel processing.

HX711_SET_AVERAGE
-----------------

.. code-block:: C

   unsigned average = 5;
   ioctl(fd, HX711_SET_AVERAGE, average);

Driver will read this number of samples from hx711 and will return average
value of them all. To avoid corrupted data due to integer overflow, max
average value that can be set is 225. If you need to average more values
you will need to write your own code for that.

HX711_SET_CHANNEL
-----------------

.. code-block:: C

   char channel = 'a';
   ioctl(fd, HX711_SET_CHANNEL, channel);

HX711 has 2 channels, A and B, which can be swapped as necessary. Driver
automatically performs dummy read, so that next call to read(2) will return
value from new channel. When you switch to channel 'B', driver automatically
changes gain to 32 (the only possible value). Going back to 'A' will set
gain to 128.

HX711_SET_GAIN
--------------

.. code-block:: C

   unsigned char gain = 128;
   ioctl(fd, HX711_SET_GAIN, gain);

Set gain. Channel 'A' supports gain "128" and "64". Channel 'B' has only
one gain option - 32.

HX711_SET_VAL_PER_UNIT
----------------------

.. code-block:: C

   int val_per_unit = 813;
   ioctl(fd, HX711_SET_VAL_PER_UNIT, val_per_unit);

Driver can perform calculations so that you can read physical values like
grams, ounce or pounds, or your own artificial unit. You just need to specify
what value from tensometer coresponds to one unit.

Say you have tensometer that has max value of 1'000'000. Value 100'000 means
1kg and sensor is fully linear. If you want to get readings in kg, you would
set ``val_per_unit`` to 100'000. If you wanted output in grams, it would be
value of 100. To have tenths of grams precision, you would set it to 10.
Driver does not care about unit, you just pick one and stick to it.

Note that driver can only return integers, so if you set it to return unit
of kg, you will only get 1, 2, 3kg... and you won't be able to sense 0.5kg
or 1.5kg. For that you would have to set value to 10'000, and driver would
return you values of 15 (for 1.5kg) or 0.5 (for 0.5kg).

HX711_TARE
----------

.. code-block:: C

  float precision = 0.1;
  ioctl(fd, HX711_TARE, &precision);

Every scale needs a tare function. Driver polls hx711 for some time, and if
it detects that scale is stable state, ioctl(2) will return with success,
and next read(2) call will take new tare value into consideration when
returning readings. Scale is assumed to be stable when several consecutive
readings are (min-max values) are within specified precition.

If ``HX711_SET_VAL_PER_UNIT`` was set prior to this, you can pass value
in your unit. If you configured driver to work with grams, you can set
this value to 0.1 (gram) or 5 (gram).

If driver cannot get stable reading within some time, it will return with
ETIME errno set.

Important note, make sure you have set correct sign before taring, or
else you will double your tare value instead of zeroing it!

HX711_SIGN
----------

.. code-block:: C

   int sign = -1;
   ioctl(fd, HX711_SIGN, &sign);

If values from drivers go lower when mass on scale goes higher you can swap
the sign. This may be necessary when tensometer was installed upside down.

---------------------
hx711 example program
---------------------

There is also companion program in Application Configuration ---> Examples
called ``HX711 driver example``. Main purpose of this is to show how to
use the driver, but it also is a very good tool for quickly debuging chip
from the shell, as it can dump readings and set all options.

.. code-block::

   -h       print this help message
   -d<path> path to hx711 device, default: /dev/hx711_0
   -t<prec> tares the scale with specified precision, might take few seconds to complete.
            If you set value per unit, precision is in units, otherwise it's raw values.
            If units are used, float can be passed like 0.1
   -v<val>  value read that coresponds to one unit. This value has to be
            calibrated first before it's known
   -s       reverse sign, if values decreses when mass increases, pass this
   -D       dumps current device settings (like, average, channel, gain etc.)
   -a<avg>  set how many samples should be averaged before returning value,
            values [1..225] are valid
   -c<chan> set channel to read (either 'a' or 'b' is valid)
   -g<gain> set adc gain, for channel 'a' 64 and 128 are valid,
            for channel 'b', only 64 is valid
   -r<num>  read this number of samples before exiting, samples will be printed
            on stdout as string, one sample per line

   Set values are persistant, as in once set they are stored in driver and
   will be applied during execution of this program.

   If you specify only <-a|-c|-g|-v|-t> without -r, program will set new parameters
   and exit. You can later call program again only with -r option to read
   samples with previously set values. You can also pass all of them in one call

   To test if you require CONFIG_ADC_HX711_ADD_DELAY option set, run as:
        hx711 -a225 -r128
   This will load hx711 chip long enough to show any possible errors due to
   lack of added delay.

   Program executes in order: set options, tare, dump, run, so if you specify all
   options, new settings will be applied, then new settings will be printed
   and at the end program will tare the scale and print samples

   Examples:

   Set hx711 settings for first chip and exit:
        hx711 -a32 -ca -g64

   Dump chip settings from different chip
        hx711 -d/dev/hx711_2 -D

   Read 10 samples with previously set hx711 settings
        hx711 -r10

   Change channel and read 32 samples (average setting won't change):
        hx711 -cb -r32

   Set value per unit, to get output in grams, and then tare with 10g precision
        hx711 -v 813 -t 10

