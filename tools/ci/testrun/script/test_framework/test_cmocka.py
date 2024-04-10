#!/usr/bin/python3
# encoding: utf-8

import os

import pytest

pytestmark = [pytest.mark.common, pytest.mark.rv_virt]

cmocka_list_start = "cmocka_list_start"
cmocka_list_end = "cmocka_list_end"
cmocka_test_start = "cmocka_test_start"
cmocka_test_end = "cmocka_test_end"


@pytest.mark.run(order=1)
def test_cmocka(p):
    if p.board == "sim":
        os.mkdir("./test")
        ret = p.sendCommand("mount -t hostfs -o fs=./test /data")
    if p.board == "rv-virt":
        ret = p.sendCommand("mount -t vfat /dev/virtblk0 /data")

    p.sendCommand(f"echo {cmocka_list_start}")
    p.sendCommand("cmocka --list", "Cmocka Test Completed")
    p.sendCommand(f"echo {cmocka_list_end}")

    p.sendCommand(f"echo {cmocka_test_start}")
    ret = p.sendCommand(
        "cmocka --skip test_case_posix_timer|test_case_oneshot|write_default|read_default|burst_test|gpiotest01|"
        "test_playback.*|test_interaction.*|test_stress.*|test_capture.*",
        "Cmocka Test Completed",
        timeout=1200,
    )
    p.sendCommand(f"echo {cmocka_test_end}")

    if p.board == "sim":
        os.rmdir("./test")

    assert ret == 0
