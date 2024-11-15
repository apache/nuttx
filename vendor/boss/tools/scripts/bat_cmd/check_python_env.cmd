::
:: 声明：该文件通过ESP项目改写而来，详细请访问https://docs.espressif.com/projects/esp-idf。
:: ‌Declaration‌: This file has been rewritten using ESP. For more details, please 
::              visit https://docs.espressif.com/projects/esp-idf.
::
@echo off

if defined MSYSTEM (
	echo This .bat file is for Windows CMD.EXE shell only.
	goto :__end
)

:: 要求检查：
::  1. 需要安装python工具；

set SCRIPT_EXIT_CODE=0

set MISSING_REQUIREMENTS=
python.exe --version >NUL 2>NUL
if %errorlevel% neq 0 (
    set SCRIPT_EXIT_CODE=%errorlevel%
    set "MISSING_REQUIREMENTS=%MISSING_REQUIREMENTS%  python"
)

if not "%MISSING_REQUIREMENTS%" == "" goto :__error_missing_requirements

goto :__end

:__error_missing_requirements
    echo. 
    echo Error^: The following tools are not installed in your environment.
    echo. 
    echo %MISSING_REQUIREMENTS%
    echo. 
    echo Please use the tools we provide and make sure to utilize them correctly.
    echo. 
    goto :__end

:__end
set MISSING_REQUIREMENTS=
exit /b %SCRIPT_EXIT_CODE%
