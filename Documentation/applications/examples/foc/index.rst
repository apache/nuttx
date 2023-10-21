``foc`` FOC motor controller example
====================================

The main purpose of this example is to provide a universal template to
implement the motor controller based on the kernel-side FOC device and
the application-side FOC library.

At the moment, this example implements a simple open-loop velocity controller.

Hardware setup
--------------

This example has not yet implemented any mechanism to protect the
powered device. This means that there is no overtemeprature
protection, no overcurrent protection and no overvoltage protection.

Make sure that you power the device properly and provide current
limits on your own so as not to break your hardware.

Configuration
-------------

The FOC PI current controller parameters can be obtained from the given
equations::

  Kp = ccb * Ls;
  pp = Rs / Ls;
  Ki = pp * Kp * T;

where:
  - Kp  - PI proportional coefficient
  - Ki  - PI integral coefficient
  - Rs  - average phase serial resistance
  - Ls  - average phase serial inductance
  - pp  - pole plant
  - ccb - current control bandwidth
  - T   - sampling period

Sample parameters for some commercially available motors
--------------------------------------------------------

========================== == ========== ========= ======= =====
Motor model                p  Rs         Ls        i_max   v_max
========================== == ========== ========= ======= =====
Odrive D6374 150KV         7  0.0254 Ohm 8.73 uH   ?       ?
Linix 45ZWN24-40           2  0.5 Ohm    0.400 mH  2.34A   24V
Bull-Running BR2804-1700   7  0.11 Ohm   0.018 mH  1.2A    12V
iPower GBM2804H-100T       7  5.29 Ohm   1.05 mH   0.15A   12V
========================== == ========== ========= ======= =====

* Odrive D6374 150KV

====== =========== ===== ======== ======
f_PWM  f_notifier  ccb   Kp       Ki
====== =========== ===== ======== ======
20kHz  10kHz       1000  0.0087   0.0025
====== =========== ===== ======== ======


* Linix 45ZWN24-40 (PMSM motor dedicated for NXP FRDM-MC-LVMTR kit)

====== =========== ===== ======== ======
f_PWM  f_notifier  ccb   Kp       Ki
====== =========== ===== ======== ======
10kHz  5kHz        1000  0.4      0.1
====== =========== ===== ======== ======

* Bull-Running BR2804-1700 kV (motor provided with the ST P-NUCLEO-IHM07 kit)

====== =========== ===== ======== ======
f_PWM  f_notifier  ccb   Kp       Ki
====== =========== ===== ======== ======
20kHz  10kHz       200   0.036    0.022
====== =========== ===== ======== ======


* iPower GBM2804H-100T (gimbal motor provided with the ST P-NUCLEO-IHM03 kit)

====== =========== ===== ======== ======
f_PWM  f_notifier  ccb   Kp       Ki
====== =========== ===== ======== ======
20kHz  10kHz       TODO  TODO     TODO
====== =========== ===== ======== ======
