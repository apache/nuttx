#!/usr/bin/python3
# encoding: utf-8
import pytest

pytestmark = [pytest.mark.common, pytest.mark.qemu]
do_not_support = ["sabre-6quad", "rv-virt", "rv-virt64", "esp32c3-devkit", "bl602evb"]


def test_ostest(p):
    ret = p.sendCommand("ostest", "Exiting with status", 300)
    assert ret == 0


def test_mm(p):
    if p.board in do_not_support:
        pytest.skip("unsupported at {}".format(p.board))
    ret = p.sendCommand("mm", "TEST COMPLETE", 120)
    assert ret == 0


def test_cxxtest(p):
    if p.board in do_not_support:
        pytest.skip("unsupported at {}".format(p.board))
    ret = p.sendCommand("cxxtest", "Test std::map")
    assert ret == 0


def test_scanftest(p):
    if p.board in do_not_support:
        pytest.skip("unsupported at {}".format(p.board))
    ret = p.sendCommand("scanftest", "Test #25")
    assert ret == 0


def test_getprime(p):
    if p.board in ["rv-virt", "rv-virt64"]:
        pytest.skip("unsupported at {}".format(p.board))
    ret = p.sendCommand("getprime", "getprime took")
    assert ret == 0


@pytest.mark.run(order=-2)
def test_fs_test(p):
    if p.board in do_not_support:
        pytest.skip("unsupported at {}".format(p.board))
    fstest_dir = "{}/{}_fstest".format(p.fs, p.core)
    p.sendCommand("mkdir %s" % fstest_dir)
    ret = p.sendCommand("fstest -n 10 -m %s" % fstest_dir, "Final memory usage", 2000)
    p.sendCommand("ls %s" % fstest_dir)
    p.sendCommand("rmdir %s" % fstest_dir)
    assert ret == 0


@pytest.mark.run(order=-1)
def test_psram_test(p):
    if p.board in do_not_support:
        pytest.skip("unsupported at {}".format(p.board))
    if p.sendCommand("ls /", "tmp/") == 0:
        ret = p.sendCommand("fstest -n 10 -m /tmp", "Final memory usage", 500)
        p.sendCommand("ls /tmp")
        assert ret == 0
