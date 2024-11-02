@echo off

rem 安装离线工具包中的python:
rem 1. 检查工具包目录是否存在python
rem 2. 删除指定的python工具解压目录，并创建
rem 3. 解压python工具，到指定目录

set SCRIPT_EXIT_CODE=0
set "COMMAND_START_DIR=%cd%"
set "ZIP_EXE=%cd%\..\dist\7zr.exe"

set "COMMAND_START_DIR=%COMMAND_START_DIR%\..\dist"
if not exist "%COMMAND_START_DIR%\" (
    echo 目录:%COMMAND_START_DIR%找不到，请重新解压工具包！！！
    echo. 
    set SCRIPT_EXIT_CODE=1
    goto :_ByeBye
)
rem 这里可能需要根据需求适配，人力不足无法适配，抱歉！
set "PYTHON_ZIP=%COMMAND_START_DIR%\idf-python-3.11.2-embed-win64.7z"
if not exist %PYTHON_ZIP% (
    echo 文件:%PYTHON_ZIP%找不到，请重新解压工具包！！！
    echo. 
    set SCRIPT_EXIT_CODE=1
    goto :_ByeBye
)

set "COMMAND_START_DIR=%cd%\..\.boss"
if exist "%COMMAND_START_DIR%\" (
    rmdir /s /q "%COMMAND_START_DIR%"
)
mkdir %COMMAND_START_DIR%

if not exist %ZIP_EXE% (
    echo 文件:%ZIP_EXE%找不到，请重新解压工具包！！！
    set SCRIPT_EXIT_CODE=1
    goto :_ByeBye
)

%ZIP_EXE% x %PYTHON_ZIP% -o%COMMAND_START_DIR% -y

set COMMAND_START_DIR=
set PYTHON_ZIP=
set ZIP_EXE=

:_ByeBye
exit /b %SCRIPT_EXIT_CODE%
