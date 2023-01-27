# Requirements
- Python 3.6 or above
- pexpect + minicom
- pytest
```bash
sudo apt-get install minicom
cd env
pip3 install -r requirements.txt
```
# Pytest Original Parameter useage refer to help
# Customized Parameters
```bash
pytest
    -D  specify device, for example: /dev/ttyUSB0
    -B  specify board, for example: sim,best1600_ep
    -P  specify nuttx path
    -F  specify filesystem, for example: /data
    -L  specify log path
    -O  specify ota version, for example: VELA-2.0
    -S  enable sudo as run sim
    -C  enable pre-checkin run
    -U  specify core: ap, audio, cp, sensor, tee
    -M  serial or minicom
    -R  specify target type: target|sim|qemu|module

```
# Example
```bash
cd script
# testsuite
pytest test_[testsuite].py -D /dev/ttyUSBx -B <board> -L <logpath> -F <filesystem folder> -P <nuttx path> -R <target|sim|qemu|module>
# mark
pytest -m <mark_name> ./ -D /dev/ttyUSBx -B <board> -L <logpath> -F <filesystem folder> -P <nuttx path> -R <target|sim|qemu|module> --json=<logpath>/pytest.json
# sim
pytest -m sim ./ -B sim -P <nuttx path> -L <logpath> -F <filesystem folder> -P <nuttx path> -R <target|sim|qemu|module> --json=<logpath>/pytest.json
# stress test
pytest -m miwear ./ -B miwear -P <nuttx path> -L <logpath> -F <filesystem path> --json=<logpath>/pytest.json --count=100 --repeat-scope=session
```
# Generate Report
```bash
cd utils/report
python3 report_gen.py -l <json log path> -b <branch>
```
