# vl53l0x_driver

This repository is ROS node to use the VL53L0X written in C++.

C++ code for ROS node

in reference to the pololu's repository https://github.com/pololu/vl53l0x-arduino (Use code:single.ino,VL53L0X.cpp,VL53L0X.h) 

use "i2c-dev.h" header instead of "Wire.h"(Arduino I2C Library)

# 使用したボード(Use boards)
今回使用したのは以下の2つのボードです
・vl53l0x(Poloru)

(・https://www.pololu.com/product/2490,http://store

・shopping.yahoo.co.jp/suzakulab/pololu-2490.html)

・Digispark USB Development Board

(・http://digistump.com/products/1

・https://www.amazon.co.jp/HiLetgo-Digispark-Kickstarter-ATTINY85-%E8%B6%85%E5%B0%8F%E5%9E%8BArduino%E4%BA%92%E6%8F%9B/dp/B00VM5M4W4/ref=cm_cr_arp_d_product_top?ie=UTF8)

# Digisparkの設定(Setting of Digispark)
・概要
i2c_tiny_usb-on-Little-WireをDigisparkに書き込み、Digisparkをi2cポートとして使う
use Digispark as i2c port by writing i2c_tiny_usb-on-Little-Wire to Disispark

※Windowsで行います(Windows10で利用可能)
※on Windows(Windows10 available)

1.ドライバのインストール(Install driver)
https://github.com/digistump/DigistumpArduino/releases/download/1.6.7/Digistump.Drivers.zip

2.i2c_tiny_usb-on-Little-Wireのバイナリ(.hex)をダウンロード(Download binary(.hex) of i2c_tiny_usb-on-Little-Wire)
https://github.com/nopdotcom/i2c_tiny_usb-on-Little-Wire/releases/download/v1.3/main.zip

3.DigistumpのSource ForgeからWindows用ツールキットをダウンロード(Download tool-kit for Windows in Digistump's Source Forge)
https://sourceforge.net/projects/digistump/files/DigisparkArduino-Win32-1.0.4-May19.zip/download

使うのは上の中の書き込みツール(using write-tool) micronucleus.exe
Digispark-Arduino-1.0.4\hardware\tools\avr\bin\micronucleus.exe

4.環境変数(PATH)に書き込みツールmicronucleus.exeの場所を追加(add address of write-tool(micronucles.exe) to PATH(environment variables) )

5.コマンドプロンプトで書き込みツールを実行(run write-tool in cmd.exe(command prompt))
C:\にツールとmain.hexを置いた場合
C:\> micronucleus.exe --run main.hex
とコマンドを打ってから、一度 DigiSpark を USB コネクタから抜いて挿しなおすと書き込まれる(after run write-tool command,once unplugging and plugging,so write binary(main.hex) to Digispark)

6.connect pins

Digispark | vl53l0x(Poloru)

 P2 SCK  -> SCL

 P0 SDA  -> SDA

 5V      -> VIN
 
 GND     -> GND 

7.Linux環境に接続してデバイスとしての認識確認、i2cのデバイス確認等(plug  Digispark to Linux, and check the recognition of the device as i2c)

command hint

・dmesg | grep i2c

[37355.430634] usb 4-1.2: Product: i2c-tiny-usb on littleWire

[37355.431529] i2c-tiny-usb 4-1.2:1.0: version 2.01 found at bus 004 address 113

[37355.433207] i2c i2c-7: connected i2c-tiny-usb device


・sudo apt-get install i2c-tools

・sudo i2cdetect -y 7
     
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f

00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 

10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 

30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

70: -- -- -- -- -- -- -- --

・sudo i2cdump -y 7 0x29

・i2cget,i2cset,etc...


# without sudo(command to set permission to access to a i2c-bus)
sudo usermod -a -G i2c $USER

