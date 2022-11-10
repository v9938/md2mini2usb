@echo off
cls
echo = メガドラmini向けFirmware更新 ==================================
echo MD2mini2USBのFirmware更新を実施します。
echo 更新中はケーブルを取り外したりしないようにお願いします。
echo.
echo コントローラCボタンを押しながら、デバイスを再接続してください。
echo デバイスが認識したらボタンから手を離してください。
echo.
wait_bootloader.exe
mphidflash-1.8-win-32.exe -v 04d8 -p e72f -s -w .\forMD2mini\md22usb_0142.hex
echo.
echo.
echo MD2mini2USBのFirmware更新が完了しました。
echo デバイスを取り外してください。
echo.
echo コントローラの状態によっては設定の初期化が必要です。
echo 動きがおかしい場合はBボタンを押しながらデバイスを接続してください。
pause

