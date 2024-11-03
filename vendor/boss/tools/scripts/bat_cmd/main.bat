::
:: 声明：该文件通过ESP项目改写而来，详细请访问https://docs.espressif.com/projects/esp-idf。
:: ‌Declaration‌: This file has been rewritten using ESP. For more details, please 
::              visit https://docs.espressif.com/projects/esp-idf.
::

@echo off

set "BAT_TOOLS_DIR=%~dp0"
for %%i in ("%cd%\..") do set "ROOT_TOOLS_DIR=%%~fi"

call %BAT_TOOLS_DIR%\check_python_env.cmd
if %errorlevel% equ 0 (
    echo. 
    echo python env check SUCCESS!
    echo. 
    goto :_GOON
)

echo 开始自动安装离线python压缩包
call %BAT_TOOLS_DIR%\install_python_zip.cmd
if %errorlevel% equ 0 (
    echo. 
    echo install python zip SUCCESS!
    echo. 
)
set Path="%ROOT_TOOLS_DIR%\.boss\idf-python-3.11.2-embed-win64\";%PATH%
call %BAT_TOOLS_DIR%\check_python_env.cmd
if %errorlevel% equ 0 (
    echo python env check SUCCESS!
    echo. 
    goto :_GOON
)
:_ByeBye
pause
exit

:_GOON
python --version
echo. 
echo Extract C/C++ env
python %cd%\idf-tools.py extract --files %ROOT_TOOLS_DIR%\dist\xtensa-esp-elf-13.2.0_20240530-x86_64-w64-mingw32_hotfix.zip %ROOT_TOOLS_DIR%\.boss

echo. 
echo Install python env
python %cd%\idf-tools.py install --python-env

echo. 
pause
