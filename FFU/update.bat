@echo off
:loop
echo C�{�^���������Ȃ���A�f�o�C�X��ڑ����Ă��������B
wait_bootloader.exe
rem mphidflash-1.8-win-32.exe -v 04d8 -p e72f >NUL
rem IF %ERRORLEVEL% == 3 goto WaitLoop
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -s -w MD22USB_1050.hex
echo.
echo ��������O���Ă��������B
pause
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -r
vercheck.exe -m 10 -s 50
if ERRORLEVEL 2 goto passed
goto failed

:passed
echo.
echo.
echo == FFU PASSED ==
echo �f�o�C�X�����O���Ă�������
echo.
pause
goto end

:failed
echo.
echo.
echo FAILED FAILED FAILED FAILED FAILED FAILED
echo �f�o�C�X�����O���A�Ď��s���Ă��������B
echo.
pause
goto end

:end

