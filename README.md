# SHISEIGYO-3 N1

<p align="center">
  <img src="images/main.jpg" width="600">
</p>

リアクションホイール倒立振子モジュール  
ESP32ベースのリアルタイム制御・ブラウザ調整対応。

## Links

- 使用方法  
  https://homemadegarbage.com/reactionwheel84

- Shop  
  https://shop.homemadegarbage.com/product/s-3-n1/

## Development Environment

- Arduino IDE: ver. 1.8.19
- ESP32 board package: ver. 2.0.13

## Required Libraries

### MPU6050
MPU6050 : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

I2Cdev : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev

### Kalman Filter Library
Version: 1.0.2 https://github.com/TKJElectronics/KalmanFilter

### Adafruit GFX Library
Version: 1.10.1 https://github.com/adafruit/Adafruit-GFX-Library

### Adafruit SSD1306
Version: 2.4.1 https://github.com/adafruit/Adafruit_SSD1306

## MPU6050 のオフセット調整

MPU6050 のオフセット値はセンサ個体ごとに異なるため、使用するモジュールに合わせて調整してください。

オフセット調整には、jrowberg/i2cdevlib の `IMU_Zero` サンプルを使用します。

https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/IMU_Zero/IMU_Zero.ino

MPU6050 を水平で安定した場所に置いて `IMU_Zero.ino` を実行し、出力された6軸分のオフセット値を本プログラム内に設定します。

```cpp
mpu.setXAccelOffset(...);
mpu.setYAccelOffset(...);
mpu.setZAccelOffset(...);
mpu.setXGyroOffset(...);
mpu.setYGyroOffset(...);
mpu.setZGyroOffset(...);
```

## Features

- リアクションホイール倒立制御
- ブラウザベース調整UI
- Wi-Fi APモード
- パラメータ保存機能
- GetUp機能
- ブレーキ調整UI

## License

MIT License
