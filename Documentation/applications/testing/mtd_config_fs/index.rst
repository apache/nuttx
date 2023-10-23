=========================================
``mtd_nvs`` MTD non-volatile storage Test
=========================================

This is a test for MTD non-volatile storage. MTD non-volatile storage was originally
implemented in Zephyr by Laczen. We made several modification to the original design.
The main purpose of those modification was:

1. support C-string key in nvs API(Original design only support uint16_t as key)
2. Meanwhile achieve better performance by limiting flash read times(Theoratically
   better than Zephyr subsys/settings, which is based on original NVS).

Options:
- ``CONFIG_TESTING_FAILSAFE_MTD_CONFIG`` – Enable the test.
- ``CONFIG_TESTING_FAILSAFE_MTD_CONFIG_VERBOSE`` – Verbose output.

EXAMPLE::
  mtdconfig_fs_test -m /dev/config  – Test MTD NVS on /dev/config
  mtdconfig_fs_test -h              – Get help message
