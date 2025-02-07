===========================
``accel_calib`` Application
===========================

This application is intended for streaming delimited accelerometer data to the
console for consumption by the user's host machine. This allows the use of
calibration tools like `Magneto
<https://sites.google.com/view/sailboatinstruments1/a-download-magneto-v1-2?authuser=0>`_
and `FreeIMU
<https://varesano.net/freeimu-magnetometer-and-accelerometer-calibration-gui-alpha-version-out/>`_
for NuttX boards that have on-board accelerometers.

This application works for any accelerometer that implements the :doc:`uorb
</components/drivers/special/sensors/sensors_uorb>` interface.

Usage
=====

Call the program with the name of the uORB accelerometer topic you wish to
calibrate, and you'll see the output start streaming to the console.

Note that measurements are listed in XYZ order.

.. code-block:: console

    nsh> accel_calib sensor_accel0
    Sampling frequency is 50Hz
    0.013160        -0.253640       9.955389
    0.015553        -0.245266       9.961371
    0.027517        -0.253640       9.984103
    ...

While the application is running, you can read the stream or record it using
your host based serial monitor.

Configuration Options
=====================

``CONFIG_ACCEL_CALIB_SAMPLING_FREQ``
------------------------------------

This option allows you to select a sampling frequency for the accelerometer in
Hz. By default it is 50Hz.

``CONFIG_ACCEL_CALIB_DELIM``
------------------------------------

This option allows you to select the delimeter to be used between measurements.
By default this is a tab, but you can select something like a comma for CSV
style records.
