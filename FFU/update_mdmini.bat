@echo off
cls
echo = ���K�h��mini����Firmware�X�V ==================================
echo MD2mini2USB��Firmware�X�V�����{���܂��B
echo �X�V���̓P�[�u�������O�����肵�Ȃ��悤�ɂ��肢���܂��B
echo.
echo �R���g���[��C�{�^���������Ȃ���A�f�o�C�X���Đڑ����Ă��������B
echo �f�o�C�X���F��������{�^�������𗣂��Ă��������B
echo.
wait_bootloader.exe
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -s -w .\forMD2mini\md22usb_0142.hex
echo.
echo.
echo MD2mini2USB��Firmware�X�V���������܂����B
echo �f�o�C�X�����O���Ă��������B
echo.
echo �R���g���[���̏�Ԃɂ���Ă͐ݒ�̏��������K�v�ł��B
echo ���������������ꍇ��B�{�^���������Ȃ���f�o�C�X��ڑ����Ă��������B
pause

