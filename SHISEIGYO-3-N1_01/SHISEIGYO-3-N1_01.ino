#include <Kalman.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "BrowserUI.h"
#include "driver/pcnt.h"


WebServer server(80);

const char ssid[] = "SHISEIGYO-3-N1";  // SSID
const char pass[] = "password";   // password

const IPAddress ip(192, 168, 11, 33);      // IPアドレス
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク


Preferences preferences;


#define GetUpBt 2
#define ModeBt 12

#define PULSE_INPUT_PIN_L 32
#define PULSE_CTRL_PIN_L 33
#define PWM_pinL 27
#define brakeL 19
#define rote_pinL 23
#define servoL 4

#define PULSE_INPUT_PIN_R 39
#define PULSE_CTRL_PIN_R 36
#define PWM_pinR 25
#define brakeR 17
#define rote_pinR 16
#define servoR 13

#define PULSE_INPUT_PIN_C 35
#define PULSE_CTRL_PIN_C 34
#define PWM_pinC 26
#define brakeC 18
#define rote_pinC 5
#define servoC 14

#define CH_L 0
#define CH_R 1
#define CH_C 2
#define CH_ServoL 13
#define CH_ServoR 14
#define CH_ServoC 15


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float theta_X = 0.0, theta_Y = 0.0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

unsigned long oldTime = 0, loopTime, nowTime;
float dt;

Kalman kalmanX, kalmanY;
float kalAngleX, kalAngleY, kalAngleDotX, kalAngleDotY;

int16_t enc_countL = 0, enc_countR = 0, enc_countC = 0;
float theta_LRdotWheel;

int Cok = 0, LRok = 0;
int GetUpcntC = 0;

float MtL, MtR, MtC, MtY, MtX;
float AjC = 0.0, AjLR = 0.0;
int pwmDutyL, pwmDutyR, pwmDutyC;

int GetUpX = 0, GetUpY = 0;
int GetUpBtState = 0;

float getUpDeg = -5.0;
int brakeMode = 0;


int Mode = 1;
String modeString;

volatile int ModeBtState = 0;
volatile unsigned long lastModeInterrupt = 0;


int YawRotOn = 0;
float rotYaw = 0.0, yawCorr = 0.0;
float KpYawRot = 0.1;

float Kp = 26.0, Kd = 40.0, Kw = 3.0, Ka = 0.4;
int loopDelay = 3;


float rotMaxC = 100, rotMaxLR = 50;
int shiftCL = 400, shiftCR = 400, shiftL = 120, shiftR = 120;
int delayC = 250, delayLR = 110;
int offsetC = 0, offsetL = 0, offsetR = 0;



//加速度センサから傾きデータ取得 [deg]
void get_theta() {
  mpu.getAcceleration(&ax, &ay, &az);
  
  theta_X = atan2(1.0 * ay , az) * RAD_TO_DEG;
  theta_Y = atan2(-1.0 * ax , az) * RAD_TO_DEG;
}

//角速度取得 [deg/s]
void get_gyro_data() {
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = gx / 131.072;
  gyroY = gy / 131.072;
  gyroZ = gz / 131.072;
}


//起き上がりY
void getupY(){
  digitalWrite(brakeC, HIGH);
  GetUpY = 1;
  AjC = 0.0;
  AjLR = 0.0;
  
  //回転方向
  if(kalAngleY < 0.0){
    digitalWrite(rote_pinC, HIGH);
    getUpDeg = -5.0;
    Cok = 8;
  }else{
    digitalWrite(rote_pinC, LOW);
    getUpDeg = 5.0;
    Cok = 9;
  }

  int rotMaxCpwm = 1023 * (1.0 - rotMaxC / 100.0);
  for(int i = 1023; i >= rotMaxCpwm; i--){
    ledcWrite(CH_C, i);
    delay(2);
  }
  ledcWrite(CH_C, rotMaxCpwm);
  delay(500);

  digitalWrite(brakeC, LOW);
  if(Cok == 8){
    ledcWrite(CH_ServoC, usToDuty(1500 + offsetC - shiftCR));
  }else{
    ledcWrite(CH_ServoC, usToDuty(1500 + offsetC + shiftCL));
  }
  delay(delayC);

  ledcWrite(CH_ServoC, usToDuty(1500 + offsetC));
}


uint32_t usToDuty(uint32_t us) {
  return (uint32_t)((us / 20000.0) * 65535);
}


void GetUp() {
  GetUpBtState = 1;
}


void ModeOnOff() {
  unsigned long now = millis();

  if(now - lastModeInterrupt > 200){
    ModeBtState = 1;
    lastModeInterrupt = now;
  }
}


//Core0
void disp(void *pvParameters) {
  Wire1.begin(0, 15); //SDA,SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();

  beginBrowserUI(ssid, pass, ip, subnet);
  
  for (;;){
    handleBrowserClient();
    disableCore0WDT();
    
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    
    display.setCursor(0,0);            
    display.println(kalAngleX, 1);
    display.setCursor(50,0);            
    display.println(kalAngleY, 1);
    
    display.setCursor(0,9);   
    if(Mode){
      display.println("Point inverted");          
    }else{
      display.println("Side inverted");
    }

    int LineC = map(kalAngleY, 20, -20, 0, 127);
    if(LineC >= 0 && LineC < 128){
      if(abs(kalAngleY) <= 1.0){
        display.fillRect(LineC-2, 16, 5, display.height()-1, SSD1306_WHITE); //(左上x, 左上y, 幅, 高さ, 線の色
      }else{
        display.drawLine(LineC, 16, LineC, display.height()-1, SSD1306_WHITE); //始点x, 始点y, 終点x, 終点y, 線の色
      }
    }
    
    int LineL = map(kalAngleX, 8, -8, 16, 63);
    if(LineL >= 16 && LineL < 64){
      if(abs(kalAngleX) <= 1.0){
        display.fillRect(0, LineL - 2 , display.width()-1, 5, SSD1306_WHITE); 
      }else{
        display.drawLine(0, LineL, display.width()-1, LineL, SSD1306_WHITE);
      }
    }
      
    display.display(); 
    if(LRok != 9) delay(30);
    display.clearDisplay();

    
    //起き上がりX
    if(GetUpX == 1){
      digitalWrite(brakeL, HIGH);
      digitalWrite(brakeR, HIGH);
      
      //回転方向
      digitalWrite(rote_pinL, HIGH);
      digitalWrite(rote_pinR, LOW);
      
      int rotMaxLRpwm = 1023 * (1.0 - rotMaxLR / 100.0);
      for(int i = 1023; i >= rotMaxLRpwm; i--){
        ledcWrite(CH_L, i);
        ledcWrite(CH_R, i);
        delay(5);
      }
      ledcWrite(CH_L, rotMaxLRpwm);
      ledcWrite(CH_R, rotMaxLRpwm);
      delay(500);

      digitalWrite(brakeL, LOW);
      digitalWrite(brakeR, LOW);
      
      ledcWrite(CH_ServoL, usToDuty(1500 + offsetL + shiftL));
      ledcWrite(CH_ServoR, usToDuty(1500 + offsetR - shiftR));

      LRok = 9;
      Mode = 1; //点倒立モード
      GetUpX = 0;
      getUpDeg = -5.0;
      
      delay(delayLR);
    
      ledcWrite(CH_ServoL, usToDuty(1500 + offsetL));
      ledcWrite(CH_ServoR, usToDuty(1500 + offsetR));
    }

    //起き上がりX軸
    if(LRok == 9 && dt < 1.0){
      if(kalAngleX > getUpDeg){
        digitalWrite(brakeL, HIGH);
        digitalWrite(brakeR, HIGH);
        LRok = 2;
      }
    }
  }
}

void setup() {
  pinMode(GetUpBt, INPUT_PULLUP);
  pinMode(ModeBt, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);


  pinMode(brakeL, OUTPUT);
  pinMode(brakeR, OUTPUT);
  pinMode(brakeC, OUTPUT);
  pinMode(rote_pinL, OUTPUT);
  pinMode(rote_pinR, OUTPUT);
  pinMode(rote_pinC, OUTPUT);

  digitalWrite(brakeL, LOW);
  digitalWrite(brakeR, LOW);
  digitalWrite(brakeC, LOW);

  attachInterrupt(GetUpBt, GetUp, FALLING);
  attachInterrupt(ModeBt, ModeOnOff, FALLING);

  Serial.begin(115200);

  pcnt_config_t pcnt_config_A_L;   
        pcnt_config_A_L.pulse_gpio_num = PULSE_INPUT_PIN_L;
        pcnt_config_A_L.ctrl_gpio_num = PULSE_CTRL_PIN_L;
        pcnt_config_A_L.lctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_A_L.hctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_A_L.channel = PCNT_CHANNEL_0;
        pcnt_config_A_L.unit = PCNT_UNIT_0;
        pcnt_config_A_L.pos_mode = PCNT_COUNT_INC;
        pcnt_config_A_L.neg_mode = PCNT_COUNT_DEC;

  pcnt_config_t pcnt_config_B_L;   
        pcnt_config_B_L.pulse_gpio_num = PULSE_CTRL_PIN_L;
        pcnt_config_B_L.ctrl_gpio_num = PULSE_INPUT_PIN_L;
        pcnt_config_B_L.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_B_L.hctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_B_L.channel = PCNT_CHANNEL_1;
        pcnt_config_B_L.unit = PCNT_UNIT_0;
        pcnt_config_B_L.pos_mode = PCNT_COUNT_INC;
        pcnt_config_B_L.neg_mode = PCNT_COUNT_DEC;

  pcnt_unit_config(&pcnt_config_A_L);
  pcnt_unit_config(&pcnt_config_B_L);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);//カウンタ初期化


  pcnt_config_t pcnt_config_A_R;   
        pcnt_config_A_R.pulse_gpio_num = PULSE_INPUT_PIN_R;
        pcnt_config_A_R.ctrl_gpio_num = PULSE_CTRL_PIN_R;
        pcnt_config_A_R.lctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_A_R.hctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_A_R.channel = PCNT_CHANNEL_0;
        pcnt_config_A_R.unit = PCNT_UNIT_1;
        pcnt_config_A_R.pos_mode = PCNT_COUNT_INC;
        pcnt_config_A_R.neg_mode = PCNT_COUNT_DEC;

  pcnt_config_t pcnt_config_B_R;   
        pcnt_config_B_R.pulse_gpio_num = PULSE_CTRL_PIN_R;
        pcnt_config_B_R.ctrl_gpio_num = PULSE_INPUT_PIN_R;
        pcnt_config_B_R.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_B_R.hctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_B_R.channel = PCNT_CHANNEL_1;
        pcnt_config_B_R.unit = PCNT_UNIT_1;
        pcnt_config_B_R.pos_mode = PCNT_COUNT_INC;
        pcnt_config_B_R.neg_mode = PCNT_COUNT_DEC;

  pcnt_unit_config(&pcnt_config_A_R);
  pcnt_unit_config(&pcnt_config_B_R);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);//カウンタ初期化


  pcnt_config_t pcnt_config_A_C;   
        pcnt_config_A_C.pulse_gpio_num = PULSE_INPUT_PIN_C;
        pcnt_config_A_C.ctrl_gpio_num = PULSE_CTRL_PIN_C;
        pcnt_config_A_C.lctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_A_C.hctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_A_C.channel = PCNT_CHANNEL_0;
        pcnt_config_A_C.unit = PCNT_UNIT_2;
        pcnt_config_A_C.pos_mode = PCNT_COUNT_INC;
        pcnt_config_A_C.neg_mode = PCNT_COUNT_DEC;

  pcnt_config_t pcnt_config_B_C;   
        pcnt_config_B_C.pulse_gpio_num = PULSE_CTRL_PIN_C;
        pcnt_config_B_C.ctrl_gpio_num = PULSE_INPUT_PIN_C;
        pcnt_config_B_C.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_B_C.hctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_B_C.channel = PCNT_CHANNEL_1;
        pcnt_config_B_C.unit = PCNT_UNIT_2;
        pcnt_config_B_C.pos_mode = PCNT_COUNT_INC;
        pcnt_config_B_C.neg_mode = PCNT_COUNT_DEC;

  pcnt_unit_config(&pcnt_config_A_C);
  pcnt_unit_config(&pcnt_config_B_C);
  pcnt_counter_pause(PCNT_UNIT_2);
  pcnt_counter_clear(PCNT_UNIT_2);//カウンタ初期化

  pcnt_counter_resume(PCNT_UNIT_0);//カウンタ初期化
  pcnt_counter_resume(PCNT_UNIT_1);//カウンタ初期化
  pcnt_counter_resume(PCNT_UNIT_2);//カウンタ初期化


  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  mpu.setXAccelOffset(-5946);
  mpu.setYAccelOffset(-2558);
  mpu.setZAccelOffset(2113);
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(-10);
  mpu.setZGyroOffset(11);

  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);


  preferences.begin("parameter", false);

  //パラメータ初期値取得  
  shiftCL = preferences.getInt("shiftCL", shiftCL);
  shiftCR = preferences.getInt("shiftCR", shiftCR);
  shiftL = preferences.getInt("shiftL", shiftL);
  shiftR = preferences.getInt("shiftR", shiftR);
  offsetC = preferences.getInt("offsetC", offsetC);
  offsetL = preferences.getInt("offsetL", offsetL);
  offsetR = preferences.getInt("offsetR", offsetR);
  delayC = preferences.getInt("delayC", delayC);
  delayLR = preferences.getInt("delayLR", delayLR);

  KpYawRot = preferences.getFloat("KpYawRot", KpYawRot);
  
  Kp = preferences.getFloat("Kp", Kp);
  Kd = preferences.getFloat("Kd", Kd);
  Kw = preferences.getFloat("Kw", Kw);
  Ka = preferences.getFloat("Ka", Ka);
  loopDelay = preferences.getInt("loopDelay", loopDelay);
  rotMaxC = preferences.getFloat("rotMaxC", rotMaxC);
  rotMaxLR = preferences.getFloat("rotMaxLR", rotMaxLR);


  ledcSetup(CH_L, 20000, 10);
  ledcAttachPin(PWM_pinL, CH_L);
  ledcSetup(CH_R, 20000, 10);
  ledcAttachPin(PWM_pinR, CH_R);
  ledcSetup(CH_C, 20000, 10);
  ledcAttachPin(PWM_pinC, CH_C);

  ledcSetup(CH_ServoL, 50, 16);
  ledcAttachPin(servoL, CH_ServoL);
  ledcWrite(CH_ServoL, usToDuty(1500 + offsetL));
  ledcSetup(CH_ServoR, 50, 16);
  ledcAttachPin(servoR, CH_ServoR);
  ledcWrite(CH_ServoR, usToDuty(1500 + offsetR));
  ledcSetup(CH_ServoC, 50, 16);
  ledcAttachPin(servoC, CH_ServoC);
  ledcWrite(CH_ServoC, usToDuty(1500 + offsetC));
  delay(500); 
  

  //ディスプレイ表示 タスク
  xTaskCreatePinnedToCore(
    disp
    ,  "disp"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);
   
}


void loop() {  
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  
  dt = (float)loopTime / 1000000.0; //sec
  
  //モータの角速度算出
  pcnt_get_counter_value(PCNT_UNIT_0, &enc_countL);
  float theta_LdotWheel = -1.0 * float(enc_countL) * 0.9 / dt;
  pcnt_counter_clear(PCNT_UNIT_0);

  pcnt_get_counter_value(PCNT_UNIT_1, &enc_countR);
  float theta_RdotWheel = 1.0 * float(enc_countR) * 0.9 / dt;
  pcnt_counter_clear(PCNT_UNIT_1);

  pcnt_get_counter_value(PCNT_UNIT_2, &enc_countC);
  float theta_CdotWheel = -1.0 * float(enc_countC) * 0.9 / dt;
  pcnt_counter_clear(PCNT_UNIT_2);
  

  get_theta();
  get_gyro_data();
    
  //カルマンフィルタ 姿勢 傾き
  kalAngleX = kalmanX.getAngle(theta_X, gyroX, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, gyroY, dt);
  
  //カルマンフィルタ 姿勢 角速度
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();


  //GETUPボタンプッシュ時の処理
  if(GetUpBtState){
    GetUpBtState = 0;
    GetUpcntC = 0;

    if((Cok == 0 || Cok == 8 || Cok == 9) && (fabs(kalAngleY) > 40.0)){
      AjC = 0.0;
      AjLR = 0.0;
      Mode = 0; //辺倒立モード
      getupY();
    }


    if(Cok == 2 && (fabs(kalAngleY) < 2.0) && (kalAngleX < -30.0)){
      AjLR = 0.0;
      GetUpX = 1;
    }
  }


  //MODEボタンプッシュ時の処理
  if(ModeBtState){
    ModeBtState = 0;
    Mode = !Mode;
  }


  //ブレーキ
  if((Cok == 2 || (Cok == 1 && GetUpcntC > 100)) && fabs(kalAngleY) > 20.0){
    digitalWrite(brakeC, LOW);
    digitalWrite(brakeL, LOW);
    digitalWrite(brakeR, LOW);
    Cok = 0;
    LRok = 0;
    AjC = 0.0;
    AjLR = 0.0;
    YawRotOn = 0;
    yawCorr = 0.0;
  }
  
  if(LRok == 2 && fabs(kalAngleX) > 20.0){
    digitalWrite(brakeL, LOW);
    digitalWrite(brakeR, LOW);
    if(Cok != 0){
      digitalWrite(brakeC, LOW);
      Cok = 0;
      AjC = 0.0;
    }
    LRok = 0;
    AjLR = 0.0;
    YawRotOn = 0;
    yawCorr = 0.0;
  }


  //起き上がりY軸
  if(Cok == 8 && dt < 1.0){
    if(kalAngleY > getUpDeg){
      digitalWrite(brakeC, HIGH); //ブレーキ解除
      theta_CdotWheel = 0.0;
      Cok = 1;
    }
  }
  if(Cok == 9 && dt < 1.0){
    if(kalAngleY < getUpDeg){
      digitalWrite(brakeC, HIGH); //ブレーキ解除
      theta_CdotWheel = 0.0;
      Cok = 1;
    }
  }

          
  //モータ起動
  if(Mode == 1){ //点倒立モード
    if ((Cok == 0 && fabs(kalAngleY) < 1.0) && (LRok == 0 && fabs(kalAngleX) < 1.0)){
      Cok = 2;
      LRok = 2;
      digitalWrite(brakeC, HIGH); //ブレーキ解除
      digitalWrite(brakeL, HIGH); //ブレーキ解除
      digitalWrite(brakeR, HIGH); //ブレーキ解除
    }
  }else{ //辺倒立モード
    if (Cok == 0 && fabs(kalAngleY) < 1.0 && kalAngleX < -30.0){
      Cok = 2;
      digitalWrite(brakeC, HIGH); //ブレーキ解除
    }
  }
  

  //Kd = 80, Kw半減, Ka = 0
  if (Cok == 1){
    MtY = Kp * kalAngleY / 90.0 + 80.0 * kalAngleDotY / 500.0 + Kw * theta_CdotWheel / 20000.0;
    GetUpcntC++;
    if(GetUpcntC > 750){
      Cok = 2;
    }
  }
  
  if (Cok == 2){
    GetUpY = 0;
    AjC +=  Ka * theta_CdotWheel / 1000000.0;
    MtY = Kp * (kalAngleY + AjC) / 90.0 + Kd * kalAngleDotY / 500.0 + Kw * theta_CdotWheel / 10000.0;
  }
  

  if (LRok == 2){
    theta_LRdotWheel = (theta_LdotWheel + theta_RdotWheel) / 2.0;
    AjLR +=  Ka * theta_LRdotWheel / 1000000.0;
    MtX = Kp * (kalAngleX + AjLR) / 90.0 + Kd * kalAngleDotX / 500.0 + Kw * theta_LRdotWheel / 10000.0;
  }


  //自転ON
  if(rotYaw != 0.0){
    YawRotOn = 1;
    yawCorr = KpYawRot / 100.0 * (rotYaw - gyroZ);
  }

  //自転OFF
  //if(fabs(yawCorr) < 0.000005 && YawRotOn == 1 && ){
  if(fabs(yawCorr) < 0.0005 && YawRotOn == 1){
    YawRotOn = 0;
    yawCorr = 0.0;
  }

  
  MtC = constrain(MtY + yawCorr, -1.0, 1.0);
  pwmDutyC = 1023 * (1.0 - fabs(MtC));
  
  MtL = constrain(float(0.5 * MtY - 0.866* MtX + yawCorr), -1.0, 1.0);
  pwmDutyL = 1023 * (1.0 - fabs(MtL));
  
  MtR = constrain(float(0.5 * MtY + 0.866* MtX + yawCorr), -1.0, 1.0);
  pwmDutyR = 1023 * (1.0 - fabs(MtR));
  
        
  //回転方向 
  if(Cok == 1 || Cok == 2){
    if(MtC < 0.0){
      digitalWrite(brakeC, HIGH);
      digitalWrite(rote_pinC, LOW);
      ledcWrite(CH_C, pwmDutyC);
    }else{
      digitalWrite(brakeC, HIGH);
      digitalWrite(rote_pinC, HIGH);
      ledcWrite(CH_C, pwmDutyC);
    }
  }
  
  if (LRok == 2){ 
    if(MtL < 0.0){
      digitalWrite(brakeL, HIGH);
      digitalWrite(rote_pinL, HIGH);
      ledcWrite(CH_L, pwmDutyL);
    }else{
      digitalWrite(brakeL, HIGH);
      digitalWrite(rote_pinL, LOW);
      ledcWrite(CH_L, pwmDutyL);
    }
    
    if(MtR < 0.0){
      digitalWrite(brakeR, HIGH);
      digitalWrite(rote_pinR, HIGH);
      ledcWrite(CH_R, pwmDutyR);
    }else{
      digitalWrite(brakeR, HIGH);
      digitalWrite(rote_pinR, LOW);
      ledcWrite(CH_R, pwmDutyR);
    }
  }

  if(Cok == 1 || Cok == 8 || Cok == 9 || LRok == 9){
    //delay無し
  }else{
    delay(loopDelay);
  }
}
