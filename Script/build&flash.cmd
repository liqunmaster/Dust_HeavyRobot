@echo off
setlocal

set "WEST=D:\MCU\zephyr\zephyrproject\.venv\Scripts\west.exe"
set "HPM_WORKSPACE=%~dp0.."
set "HPM_ELF=%HPM_WORKSPACE%\build\zephyr\Dust_HeavyRobot.elf"
set "HPM_OPENOCD=D:\MCU\hpm\tools\openocd\openocd.exe"
set "HPM_OPENOCD_TCL=D:\MCU\hpm\tools\openocd\tcl"
set "HPM_BOARD_CFG=%HPM_WORKSPACE%\Board\hpm5361\hpm5361.cfg"

echo [1/2] Incremental build Zephyr hpm5361icb for on-chip flash ...
pushd "D:\MCU\zephyr\zephyrproject"
"%WEST%" build -b hpm5361icb -s "%HPM_WORKSPACE%" -d "%HPM_WORKSPACE%\build"
set "BUILD_RESULT=%ERRORLEVEL%"
popd
if not "%BUILD_RESULT%"=="0" exit /b %BUILD_RESULT%

if not exist "%HPM_ELF%" (
    echo ERROR: ELF not found: %HPM_ELF%
    exit /b 1
)

echo [2/2] Use HPMicro official OpenOCD with HPM5361 board config ...
"%HPM_OPENOCD%" ^
    -s "%HPM_OPENOCD_TCL%" ^
    -f "%HPM_BOARD_CFG%" ^
    -c "adapter speed 500" ^
    -c "program {%HPM_ELF%} verify reset exit"
if errorlevel 1 exit /b 1

echo DONE. HPMicro official programmer verified the firmware in on-chip flash.
exit /b 0
