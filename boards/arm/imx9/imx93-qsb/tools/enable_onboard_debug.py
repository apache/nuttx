#!/usr/bin/env python3
############################################################################
# boards/arm/imx9/imx93-qsb/tools/enable_onboard_debug.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

from pyftdi.i2c import I2cController

i2c = I2cController()
i2c.configure("ftdi://ftdi:4232h/2")

slave_addr = 0x21  # IO expander address
port = i2c.get_port(slave_addr)


def set_bit(value, bit):
    return value | (1 << bit)


def clear_bit(value, bit):
    return value & ~(1 << bit)


def read_modify_write(port, reg_addr, bit, set_high=True):
    current = port.read_from(reg_addr, 1)[0]
    new_val = set_bit(current, bit) if set_high else clear_bit(current, bit)
    if new_val != current:
        port.write_to(reg_addr, bytes([new_val]))
    return new_val


# Set bit 4 high in register 0x03 (rc_sel high)
read_modify_write(port, 0x03, bit=4, set_high=True)

# Clear bit 4 in register 0x07 (configure rc_sel as output)
read_modify_write(port, 0x07, bit=4, set_high=False)

i2c.terminate()
