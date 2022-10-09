@echo off
:loop
echo Cボタンを押しながら、デバイスを接続してください。
wait_bootloader.exe
rem mphidflash-1.8-win-32.exe -v 04d8 -p e72f >NUL
rem IF %ERRORLEVEL% == 3 goto WaitLoop
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -s -w MD22USB_1050.hex
echo.
echo 治具を取り外してください。
pause
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -r
vercheck.exe -m 10 -s 50
if ERRORLEVEL 2 goto passed
goto failed

:passed
echo.
echo.
echo == FFU PASSED ==
echo デバイスを取り外してください
echo.
pause
goto end

:failed
echo.
echo.
echo FAILED FAILED FAILED FAILED FAILED FAILED
echo デバイスを取り外し、再実行してください。
echo.
pause
goto end

:end

