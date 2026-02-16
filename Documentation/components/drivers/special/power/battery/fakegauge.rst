===============
 Battery Gauge
===============

Battery gauge driver for NuttX that measurement battery data (voltage,
current, capacity, temperature, and charging status) for charger and
healthd in production.

Fake Gauge
==========

A fake battery gauge driver for NuttX that simulates battery data (voltage,
current, capacity, temperature, and charging status) for testing and
development purposes. it provides mock battery data without requiring physical
battery hardware.It generates random values within predefined ranges and
updates them periodically, making it useful for:

  - Testing battery monitoring applications
  - Developing power management features
  - Debugging battery-related logic without real hardware

Features
========

The Fake gauge simulates key battery parameters.

  Voltage (mV):
  - Voltage ranges: 4000mV to 4200mV

  Current (mA):
  - Current ranges: -100mA to 500mA
  - Current resolution: 1mA

  Capacity (%):
  - Capacity ranges: 0% to 100%
  - Capacity resolution: 1%

  Temperature (0.1°C):
  - Temperature resolution: 0.1°C

  Charging Status:
  - Charging
  - Discharging
  - Not charging

  Periodic data updates (default: 5 seconds)
  Compatible with NuttX battery gauge framework.


Usage
=====

  File Information
  - Path: drivers/power/battery/battery_fakegauge.c
  - License: Apache License 2.0

  Dependencies
  - NuttX operating system
  - NuttX battery gauge framework (nuttx/power/battery_gauge.h)
  - NuttX work queue for periodic updates

  Configuration
  - Enable the fake gauge driver in the NuttX configuration file (CONFIG_BATTERY_FAKEGAUGE=y)
  - Configure the update interval in seconds (CONFIG_BATTERY_FAKEGAUGE_UPDATE_INTERVAL)

This driver is intended for development and testing only, not for production use with
real batteries.
