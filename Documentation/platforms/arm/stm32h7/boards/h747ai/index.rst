==============
Generic H747AI
==============

Assumes the following:

    * 48 MHz crystal oscillator for HSE

Compiling
---------

Configure and build using:

    cmake -B build/h747ai_cm7_rptun -DBOARD_CONFIG=h747ai:nsh_cm7_rptun -GNinja
    cmake -B build/h747ai_cm4_rptun -DBOARD_CONFIG=h747ai:nsh_cm4_rptun -GNinja

    cmake --build build/h747ai_cm7_rptun
    cmake --build build/h747ai_cm4_rptun

Testing
-------

Connect indirectly to the CM4 via the CM7 using:

    nsh-cm7> cu -l /dev/ttyproxy
    NuttShell (NSH) NuttX-3.6.1
    nsh-cm4>

Debugging
---------

Debug using:

	{
		"name": "Debug CM7 - ST-Link",
		"cwd": "${workspaceFolder}",
		"type": "cortex-debug",
		"executable": "${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
		"loadFiles": [
			"${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
			"${workspaceFolder}/build/h747ai_cm4_rptun/nuttx"
		],
		"request": "launch",
		"servertype": "stlink",
		"gdbPath": "/usr/bin/gdb-multiarch",
		"device": "STM32H747AI",
		"interface": "swd",
		"runToEntryPoint": "main",
		"svdFile": "${workspaceFolder}/STM32H747_CM7.svd",
		"v1": false,
		"showDevDebugOutput": "both",
		"serverArgs": [
			"-l",
			"1",
			"-m",
			"0",
			"-k",
			"-t",
			"-s"
		]
	},
	{
		"name": "Debug CM4 - ST-Link",
		"cwd": "${workspaceFolder}",
		"type": "cortex-debug",
		"executable": "${workspaceFolder}/build/h747ai_cm4_rptun/nuttx",
		"loadFiles": [
			"${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
			"${workspaceFolder}/build/h747ai_cm4_rptun/nuttx"
		],
		"request": "launch",
		"servertype": "stlink",
		"gdbPath": "/usr/bin/gdb-multiarch",
		"device": "STM32H747AI",
		"interface": "swd",
		"runToEntryPoint": "main",
		"svdFile": "${workspaceFolder}/STM32H747_CM4.svd",
		"v1": false,
		"showDevDebugOutput": "both",
		"serverArgs": [
			"-l",
			"1",
			"-m",
			"3",
			"-k",
			"-t",
			"-s"
		]
	},
	{
		"name": "Debug CM7+CM4 - ST-Link",
		"cwd": "${workspaceFolder}",
		"type": "cortex-debug",
		"executable": "${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
		"loadFiles": [
			"${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
			"${workspaceFolder}/build/h747ai_cm4_rptun/nuttx"
		],
		"request": "launch",
		"servertype": "stlink",
		"gdbPath": "/usr/bin/gdb-multiarch",
		"device": "STM32H747AI",
		"interface": "swd",
		"serialNumber": "",
		"svdFile": "${workspaceFolder}/STM32H747_CM7.svd",
		"v1": false,
		"showDevDebugOutput": "both",
		"serverArgs": [
			"-l",
			"1",
			"-m",
			"0",
			"-k",
			"-t",
			"-s"
		],
		"chainedConfigurations": {
			"enabled": true,
			"waitOnEvent": "postInit",
			"detached": true,
			"delayMs": 5000,
			"lifecycleManagedByParent": true,
			"launches": [
				{
					"name": "Attach CM4 - ST-Link",
					"folder": "${workspaceFolder}"
				}
			]
		}
	},
	{
		"name": "Attach CM7 - ST-Link",
		"cwd": "${workspaceFolder}",
		"type": "cortex-debug",
		"executable": "${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
		"loadFiles": [
			"${workspaceFolder}/build/h747ai_cm7_rptun/nuttx",
			"${workspaceFolder}/build/h747ai_cm4_rptun/nuttx"
		],
		"request": "attach",
		"servertype": "stlink",
		"gdbPath": "/usr/bin/gdb-multiarch",
		"device": "STM32H747AI",
		"interface": "swd",
		"runToEntryPoint": "main",
		"svdFile": "${workspaceFolder}/STM32H747_CM7.svd",
		"v1": false,
		"showDevDebugOutput": "both",
		"serverArgs": [
			"-l",
			"1",
			"-m",
			"0",
			"-k",
			"-t",
			"-s"
		]
	},
	{
		"name": "Attach CM4 - ST-Link",
		"cwd": "${workspaceFolder}",
		"type": "cortex-debug",
		"executable": "${workspaceFolder}/build/h747ai_cm4_rptun/nuttx",
		"request": "attach",
		"servertype": "stlink",
		"gdbPath": "/usr/bin/gdb-multiarch",
		"device": "STM32H747AI",
		"interface": "swd",
		"serialNumber": "",
		"svdFile": "${workspaceFolder}/STM32H747_CM4.svd",
		"v1": false,
		"showDevDebugOutput": "both",
		"serverArgs": [
			"-l",
			"1",
			"-m",
			"3",
			"-t",
			"-s"
		]
	}

Note that the M7 is core 0 (`-m 0`) and the M4 is core 3 (`-m 3`).
