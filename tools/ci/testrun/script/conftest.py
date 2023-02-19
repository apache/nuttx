#!/usr/bin/env python3
# encoding: utf-8

import pytest
from utils.common import connectNuttx


def pytest_addoption(parser):
    parser.addoption(
        "-D",
        action="store",
        default=None,
        help="specify device, for example: /dev/ttyUSB0",
    )
    parser.addoption(
        "-B", action="store", default="sim", help="specify board, for example: sim"
    )
    parser.addoption(
        "-P",
        action="store",
        default=None,
        help="specify vela path, for example: /home/root/vela",
    )
    parser.addoption(
        "-F",
        action="store",
        default="/data",
        help="specify filesystem, for example: /data or /tmp",
    )
    parser.addoption(
        "-L",
        action="store",
        default=None,
        help="specify log path, for example: /home/root/vela/logs",
    )
    parser.addoption("-O", action="store", default=None, help="specify ota version")
    parser.addoption(
        "-S", action="store_true", default=False, help="enable sudo as run sim"
    )
    parser.addoption(
        "-C", action="store_true", default=False, help="enable pre-checkin run"
    )
    parser.addoption(
        "-U",
        action="store",
        default=None,
        help="specify core: ap, audio, cp, sensor, tee",
    )
    parser.addoption(
        "-M",
        action="store",
        default="minicom",
        help="serial open method:serial or minicom",
    )
    parser.addoption(
        "-R",
        action="store",
        default="sim",
        help="specify the target type: target|qemu|sim|module, default is sim",
    )


@pytest.fixture(scope="session")
def get_option(pytestconfig):
    dev = pytestconfig.getoption("-D")
    board = pytestconfig.getoption("-B")
    vela_path = pytestconfig.getoption("-P")
    fs = pytestconfig.getoption("-F")
    log_path = pytestconfig.getoption("-L")
    ota_version = pytestconfig.getoption("-O")
    sudo = pytestconfig.getoption("-S")
    ci = pytestconfig.getoption("-C")
    core = pytestconfig.getoption("-U")
    method = pytestconfig.getoption("-M")
    target = pytestconfig.getoption("-R")
    yield dev, board, vela_path, fs, log_path, ota_version, core, sudo, ci, method, target


@pytest.fixture(scope="session", name="p")
def connect_nuttx_session(get_option):
    (
        dev,
        board,
        vela_path,
        fs,
        log_path,
        ota_version,
        core,
        sudo,
        ci,
        method,
        target,
    ) = get_option
    print(get_option)
    p = connectNuttx(
        board, vela_path, dev, log_path, fs, ota_version, core, sudo, ci, method, target
    )
    p.setup()
    yield p
    p.cleanup()


@pytest.fixture(scope="function", autouse=True)
def do_free_ps(request, p):
    if "disable_autouse" in request.keywords:
        yield
    else:
        yield
        p.sendCommand("free", "total", flag=">")
        p.sendCommand("ps", "PID", flag=">")
        p.sendCommand("ls %s" % p.fs, flag=">")


@pytest.fixture(scope="function", name="pp")
def connect_nuttx_function(get_option):
    (
        dev,
        board,
        vela_path,
        fs,
        log_path,
        ota_version,
        core,
        sudo,
        ci,
        method,
        target,
    ) = get_option
    print(get_option)
    p = connectNuttx(
        board, vela_path, dev, log_path, fs, ota_version, core, sudo, ci, method, target
    )
    p.setup()
    yield p
    # p.cleanup()
