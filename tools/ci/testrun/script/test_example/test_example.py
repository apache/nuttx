#!/usr/bin/python3
# encoding: utf-8
import pytest

pytestmark = [pytest.mark.common, pytest.mark.qemu]


def test_hello(p):
    ret = p.sendCommand("hello", "Hello, World!!")
    assert ret == 0


def test_helloxx(p):
    ret = p.sendCommand("helloxx", "Hello, World!!")
    assert ret == 0


def test_pipe(p):
    p.sendCommand("umount /tmp")
    ret = p.sendCommand("pipe", "redirect_reader: Returning success", 60)
    assert ret == 0
    p.sendCommand("\n")
    p.sendCommand("mount -t tmpfs /tmp")


def test_popen(p):
    ret = p.sendCommand("popen", "Calling pclose()")
    assert ret == 0


def test_usrsocktest(p):
    ret = p.sendCommand("usrsocktest", "FAILED:0", 60)
    assert ret == 0
