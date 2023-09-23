=================
Nordic Thingy:52
=================

.. tags:: arch:arm, chip:nrf52, part:nrf52832, vendor:nordic

The `Thingy:52 (PCA20020) <https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-52>`_
is a prototyping platform build around the nRF52832 from Nordic.

Peripheral Support
==================

================================== ======= ==========================
Peripheral                         Support NOTES
================================== ======= ==========================
UART                               Yes
Buttons                            No
LEDs                               Yes     Lightwell RGB LED (SX1509)
NFC                                No
IMU (MPU-9250)                     No
Low power accelerometer (LIS2DH12) No
Pressure and temperature (LPS22HB) Yes     uORB, ``sensor_baro0``
Humidity and temperature (HTS221)  Yes     uORB, ``sensor_humi0``,
                                           ``sensor_ambient_temp0``
Color sensor (BH1745NUC)           Yes     uORB, ``sensor_rgb0``
Gas sensor (CCS811)                Yes     uORB, ``sensor_co20``,
                                           ``sensor_tvoc0``
Microphone (MP34DB02)              No
Speaker                            No
Battery monitoring                 Yes     SAADC AIN4 (P0.28)
================================== ======= ==========================

Serial Console
==============

At default serial console is available with Segger RTT driver.
For access to the RTT console use this commands::

  JLinkGDBServer -if SWD -device stm32h743zi -speed 16000
  sudo socat -d -d PTY,link=/dev/ttyRTT0,raw,ignoreeof TCP:127.0.0.1:19021,reuseaddr
  minicom -D /dev/ttyRTT0

An alternative option is to use the P4 connector and connect an external UART converter:

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
P0.02 UART0 TX   P4 header
P0.03 UART0 RX   P4 header
===== ========== ==========

For this you need to select the following options in your configuration::

  CONFIG_NRF52_UART0=y
  CONFIG_UART0_SERIAL_CONSOLE=y

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh thingy52:<subdir>

Where <subdir> is one of the following:

nsh
----

Basic NuttShell configuration (console enabled on Segger RTT channel).

thingy_nimble
-------------

NuttShell configuration with the nimBLE host stack, the SX1509 IO expander
and LED driver, the buttons and GPIO examples and the on-board uORB sensors
(HTS221, LPS22HB, BH1745NUC and CCS811) enabled.

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug probe.
