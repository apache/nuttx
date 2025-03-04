=================
Thermal Framework
=================

Thermal Framework is a subsystem in the kernel that provides interfaces for thermal management of devices. It is designed to monitor the temperature of various components and adjust their operating conditions to prevent overheating. The framework is responsible for controlling the cooling devices, such as fans and heatsinks, to maintain the temperature within safe limits. It provides a unified thermal management interface that can be used by different thermal drivers. The drivers can be implemented by hardware vendors to support their specific thermal management requirements.

Brief
=====
1. Support Zone, Cooling Device and Governor
    - ``Zone``: Responsible for monitoring the temperature of the specified area, obtains the temperature through the temperature sensor, and the sensor driver returns the temperature through callback function.
    - ``Cooling Device``: A cooling device is a device that can reduce the temperature by using resources such as cpufreq, fan, etc. The cpufreq cooling device driver is preset to simplify CPU frequency modulation temperature control.
    - ``Governor``: For temperature control, you can use the preset or custom registered one, preset "step_wise" governor:
        - When the temperature of the "Zone" reaches the temperature trip point, and the temperature change trend rises or stabilizes ("step_wise" obtains the value of the corresponding "Zone" every 20ms [``CONFIG_THERMAL_DUMMY_POLLING_DELAY=200``, ``CONFIG_USEC_PER_TICK=100``]), the current temperature equals to OR greater than the last obtained temperature value), improve the state of the "Cooling Device" (trigger the cooling operation executed by the corresponding state, Through ``set_state``).
        - When the temperature of the zone is lower than the temperature trip point, and the temperature trend is steadily decreasing, the state of the "Cooling Device" is reduced.

#. Support three types of temperatures trip points: ``THERMAL_NORMAL``, ``THERMAL_HOT`` and ``THERMAL_CRITICAL``
    - ``NORMAL``: When the device temperature reaches temperature of this trip point,  control needs to be started. If current temperature equals to the temperature obtained last time, the temperature control level("cooling state") is maintained. When it is greater than, "cooling state" increased.
    - ``HOT``: When the device temperature reaches this trip point, stricter temperature control (such as resource limiting) is required. If the current temperature is equals to OR greater than temperature obtained last time, the temperature control level is increased.
    - ``CRITICAL``: Shut down / restart the device.

#. ProcFS node supported, used for debugging, and we can get the binding info between "Zone", "Cooling Decice", "Trip" and "Governor", temperature value and cooling state; (for example, "Zone Device"(temperature sensor): ``/proc/thermal/cpu_thermal``), and write 0 or 1 to turn off or on the "Zone Device";

Device Driver
=============
1. Cooling Device
    Device providers should provide ``struct thermal_cooling_device_ops_s`` instance and private data (optional, for example, ``struct dummy_cooling_device_s``). Please ref to drivers/thermal/thermal_dummy.c::

      static const struct thermal_cooling_device_ops_s g_dummy_fan0_ops =
      {
        .set_state     = dummy_cdev_set_state,
        .get_state     = dummy_cdev_get_state,
        .get_max_state = dummy_cdev_get_max_state,
      };

      static struct dummy_cooling_device_s g_dummy_fan0_data =
      {
        .cur_state = 0,
        .max_state = 16,
      };

      int thermal_dummy_init(void)
      {
        FAR struct thermal_cooling_device_s *cdev;
        FAR struct thermal_zone_device_s *zdev;
        ...

        /* Cooling Device */
        cdev = thermal_cooling_device_register("fan0", &g_dummy_fan0_data,
                                               &g_dummy_fan0_ops);
        ...
      }

#. Zone Device
    The following instances need to be defined:
        - ``struct thermal_zone_device_ops_s``: Get temperature, set temperature window(optional)
        - ``struct thermal_zone_params_s``: Describe zone, governor and cooling-maps
        - ``dummy_zone_device_s`` (private, optional): For temperature and trends

    For example, drivers/thermal/thermal_dummy.c::

      /* Zone Device */
      zdev = thermal_zone_device_register("cpu-thermal", &g_dummy_zone,
                                          &g_dummy_zone_ops, &g_dummy_params);

      static const struct thermal_zone_device_ops_s g_dummy_zone_ops =
      {
        .get_temp  = dummy_zdev_get_temp,
        .set_trips = dummy_zdev_set_trips,
      };

      static struct dummy_zone_device_s g_dummy_zone =
      {
        .temperature = 45,
        .raising = true,
      };

#. Testing / Debuging
    - Disable Zone Device: ``echo 0 > /proc/thermal/cpu-thermal``
    - Get binding info::

        nsh> cat /proc/thermal/cpu-thermal
        z:cpu-thermal t:77 t:1 h:16 l:0 c:fan0 s:7|7
        z:cpu-thermal t:77 t:1 h:3 l:3 c:cpufreq s:3|3
        z:cpu-thermal t:77 t:2 h:2 l:0 c:cpufreq s:3|2

Board Customization
===================
The binding relationship between Trip, Cooling Device, Governor and Zone is shown in thermal_dummy.c. It is expected that the vendor adapter will provide the hardware related initial under ``CONFIG_ARCH_BOARD_CUSTOM_DIR`` for product customization, as described in the following comments and structures:
::

  /* thermal-zones {
   *   "cpu-thermal" {
   *     polling-delay : CONFIG_THERMAL_DUMMY_POLLING_DELAY;
   *     passive-delay : CONFIG_THERMAL_DUMMY_PASSIVE_DELAY;
   *     governor      : "step_wise";
   *
   *     trips {
   *       "cpu_crit"   { 90, 10, THERMAL_CRITICAL };
   *       "cpu_alert1" { 70, 10, THERMAL_HOT };
   *       "cpu_alert0" { 60, 10, THERMAL_NORMAL };
   *     };
   *
   *     cooling-maps {
   *       "cpu_alert0" {
   *         { "cpu0", THERMAL_NO_LIMIT, 3 };
   *       };
   *       "cpu_alert1" {
   *         { "cpu0", THERMAL_NO_LIMIT, 3 };
   *         { "fan0", THERMAL_NO_LIMIT, THERMAL_NO_LIMIT };
   *       };
   *       "cpu_crit" {
   *         { NULL, THERMAL_NO_LIMIT, THERMAL_NO_LIMIT };
   *       };
   *     };
   *   };
   * };
   */

  static const struct thermal_zone_trip_s g_dummy_trips[] =
  {
    {.name = "cpu_crit",   .temp = 90, .hyst = 10, .type = THERMAL_CRITICAL},
    {.name = "cpu_alert1", .temp = 70, .hyst = 10, .type = THERMAL_NORMAL},
    {.name = "cpu_alert0", .temp = 60, .hyst = 10, .type = THERMAL_NORMAL},
  };

  static const struct thermal_zone_map_s g_dummy_maps[] =
  {
    {
      .trip_name = "cpu_alert1",
      .cdev_name = "cpufreq",
      .low    = 3,
      .high   = THERMAL_NO_LIMIT,
      .weight = 20
    },
    {
      .trip_name = "cpu_alert1",
      .cdev_name = "fan0",
      .low    = THERMAL_NO_LIMIT,
      .high   = THERMAL_NO_LIMIT,
      .weight = 20
    },
    {
      .trip_name = "cpu_alert0",
      .cdev_name = "cpufreq",
      .low    = THERMAL_NO_LIMIT,
      .high   = 2,
      .weight = 20
    },
  };

  static const struct thermal_zone_params_s g_dummy_params =
  {
    .gov_name = "step_wise",
    .polling_delay = CONFIG_THERMAL_DUMMY_POLLING_DELAY,
    .trips = g_dummy_trips,
    .num_trips = nitems(g_dummy_trips),
    .maps = g_dummy_maps,
    .num_maps = nitems(g_dummy_maps),
  };
