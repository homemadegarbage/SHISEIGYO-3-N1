#include "BrowserUI.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>


extern WebServer server;
extern Preferences preferences;

extern int Mode;
extern int GetUpBtState;
extern int brakeMode;
extern String modeString;

extern float Kp;
extern float Kd;
extern float Kw;
extern float Ka;
extern int loopDelay;

extern float rotMaxLR;
extern float rotMaxC;
extern int offsetC;
extern int offsetL;
extern int offsetR;
extern int shiftCL;
extern int shiftCR;
extern int shiftL;
extern int shiftR;
extern int delayC;
extern int delayLR;

extern float rotYaw;
extern float KpYawRot;

extern uint32_t usToDuty(uint32_t us);

constexpr int CH_ServoL_BROWSER = 13;
constexpr int CH_ServoR_BROWSER = 14;
constexpr int CH_ServoC_BROWSER = 15;

static void handleRoot();
static void handleBrake();
static void handleBackToMain();
static void handleMode();
static void handleGetUp();
static void rotYawm();
static void rotYawp();
static void KpYawRotM();
static void KpYawRotP();
static void handleshiftCLM();
static void handleshiftCLP();
static void handleshiftCRM();
static void handleshiftCRP();
static void handleshiftLM();
static void handleshiftLP();
static void handleshiftRM();
static void handleshiftRP();
static void handleloopDelayCM();
static void handleloopDelayCP();
static void handleloopDelayLRM();
static void handleloopDelayLRP();
static void handleoffsetCM();
static void handleoffsetCP();
static void handleoffsetLM();
static void handleoffsetLP();
static void handleoffsetRM();
static void handleoffsetRP();
static void handleKpM();
static void handleKpP();
static void handleKdM();
static void handleKdP();
static void handleKwM();
static void handleKwP();
static void handleKaM();
static void handleKaP();
static void handleloopDelayM();
static void handleloopDelayP();
static void handleRotMaxCm();
static void handleRotMaxCp();
static void handleRotMaxLRm();
static void handleRotMaxLRp();

void beginBrowserUI(const char* ssid, const char* pass, const IPAddress& ip, const IPAddress& subnet) {
  WiFi.softAP(ssid, pass);           // SSIDとパスワードを設定する
  delay(100);                        // AP起動直後の設定失敗を避けるために少し待つ
  WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクを設定する

  server.on("/", handleRoot);

  server.on("/brake", handleBrake);
  server.on("/backToMain", handleBackToMain);

  server.on("/mode", handleMode);
  server.on("/GetUp", handleGetUp);

  server.on("/rotYawm", rotYawm);
  server.on("/rotYawp", rotYawp);
  server.on("/KpYawRotM", KpYawRotM);
  server.on("/KpYawRotP", KpYawRotP);

  server.on("/shiftCLM", handleshiftCLM);
  server.on("/shiftCLP", handleshiftCLP);
  server.on("/shiftCRM", handleshiftCRM);
  server.on("/shiftCRP", handleshiftCRP);
  server.on("/shiftLM", handleshiftLM);
  server.on("/shiftLP", handleshiftLP);
  server.on("/shiftRM", handleshiftRM);
  server.on("/shiftRP", handleshiftRP);

  server.on("/offsetCM", handleoffsetCM);
  server.on("/offsetCP", handleoffsetCP);
  server.on("/offsetLM", handleoffsetLM);
  server.on("/offsetLP", handleoffsetLP);
  server.on("/offsetRM", handleoffsetRM);
  server.on("/offsetRP", handleoffsetRP);

  server.on("/delayCM", handleloopDelayCM);
  server.on("/delayCP", handleloopDelayCP);
  server.on("/delayLRM", handleloopDelayLRM);
  server.on("/delayLRP", handleloopDelayLRP);

  server.on("/KpM", handleKpM);
  server.on("/KpP", handleKpP);
  server.on("/KdM", handleKdM);
  server.on("/KdP", handleKdP);
  server.on("/KwM", handleKwM);
  server.on("/KwP", handleKwP);
  server.on("/KaM", handleKaM);
  server.on("/KaP", handleKaP);

  server.on("/loopDelayM", handleloopDelayM);
  server.on("/loopDelayP", handleloopDelayP);

  server.on("/rotMaxCm", handleRotMaxCm);
  server.on("/rotMaxCp", handleRotMaxCp);
  server.on("/rotMaxLRm", handleRotMaxLRm);
  server.on("/rotMaxLRp", handleRotMaxLRp);

  server.begin();
}

void handleBrowserClient() {
  server.handleClient();
}

//ブラウザ表示
static void handleRoot() {
  brakeMode = 0;
  
  if(Mode){
    modeString = "点倒立";
  }else{
    modeString = "辺倒立";
  }
  
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp += "<head>";
  temp += "<meta charset=\"utf-8\">";
  temp += "<title>SHISEIGYO-3 N1</title>";
  temp += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp += "<style>";
  temp += ".container{max-width: 500px;margin: auto;text-align: center;font-size: 1.2rem;}";
  temp += "span,.pm{display: inline-block;border: 1px solid #ccc;width: 50px;height: 30px;vertical-align: middle;margin-bottom: 20px;}";
  temp += "span{width: 120px;}";
  temp += "button{width: 100px;height: 40px;font-weight: bold;margin-bottom: 10px;}";
  temp += ".btn-row{display: flex;justify-content: center;gap: 10px;}";
  temp += "</style></head><body><div class=\"container\">";


  temp += "<a class='button' href='/brake'>ブレーキ調整</a><br><br>";

  temp += "<div class=\"btn-row\">";
  temp += "<button type=\"button\"><a href=\"/mode\">" + modeString + "</a></button>"; //倒立モード
  temp += "<button type=\"button\"><a href=\"/GetUp\">GetUp</a></button>"; //起き上がりボタン
  temp += "</div><br>";

  //rotYaw
  temp += "自転<br>";
  temp += "<a class=\"pm\" href=\"/rotYawm\">-</a>";
  temp += "<span>" + String(rotYaw) + "</span>";
  temp += "<a class=\"pm\" href=\"/rotYawp\">+</a><br>";

  //KpYawRot
  temp += "自転Pパラメータ<br>";
  temp += "<a class=\"pm\" href=\"/KpYawRotM\">-</a>";
  temp += "<span>" + String(KpYawRot) + "</span>";
  temp += "<a class=\"pm\" href=\"/KpYawRotP\">+</a><br>";
  

  //Kp
  temp += "Kp<br>";
  temp += "<a class=\"pm\" href=\"/KpM\">-</a>";
  temp += "<span>" + String(Kp) + "</span>";
  temp += "<a class=\"pm\" href=\"/KpP\">+</a><br>";

  //Kd
  temp += "Kd<br>";
  temp += "<a class=\"pm\" href=\"/KdM\">-</a>";
  temp += "<span>" + String(Kd) + "</span>";
  temp += "<a class=\"pm\" href=\"/KdP\">+</a><br>";

  //Kw
  temp += "Kw<br>";
  temp += "<a class=\"pm\" href=\"/KwM\">-</a>";
  temp += "<span>" + String(Kw) + "</span>";
  temp += "<a class=\"pm\" href=\"/KwP\">+</a><br>";

  //Ka
  temp += "Ka<br>";
  temp += "<a class=\"pm\" href=\"/KaM\">-</a>";
  temp += "<span>" + String(Ka) + "</span>";
  temp += "<a class=\"pm\" href=\"/KaP\">+</a><br>";

  //loopDelay
  temp += "loopDelay [msec]<br>";
  temp += "<a class=\"pm\" href=\"/loopDelayM\">-</a>";
  temp += "<span>" + String(loopDelay) + "</span>";
  temp += "<a class=\"pm\" href=\"/loopDelayP\">+</a><br>";

   
  temp += "</div>";
  temp += "</body>";
  server.send(200, "text/HTML", temp);

}


static void handleBrake() {
  brakeMode = 1;
  
  String html ="<!DOCTYPE html><html lang='ja'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Brake</title>";
  html += "<style>";
  html += ".container{margin:auto;text-align:center;font-size:1.2rem;}";
  html += "span,.pm{display:inline-block;border:1px solid #ccc;width:50px;height:30px;vertical-align:middle;margin-bottom:8px;}";
  html += "span{width:120px;}";
  html += ".column-3{max-width:330px;margin:auto;text-align:center;display:flex;justify-content:space-between;flex-wrap:wrap;}"; 
  html += "button{width: 100px;height: 40px;font-weight: bold;margin-bottom: 0px;}";
  html += ".btn-row{display: flex;justify-content: center;gap: 10px;}";
  html += "</style></head><body><div class='container'>";

  html += "<div class=\"btn-row\">";
  html += "<button type=\"button\"><a href=\"/backToMain\">戻る</a></button>"; //戻るボタン
  html += "<button type=\"button\"><a href=\"/GetUp\">GetUp</a></button>"; //起き上がりボタン
  html += "</div><br>";


  //Rot Max C
  html += "Rot Max C [%]<br>";
  html += "<a class=\"pm\" href=\"/rotMaxCm\">-</a>";
  html += "<span>" + String(rotMaxC) + "</span>";
  html += "<a class=\"pm\" href=\"/rotMaxCp\">+</a><br>";

  //Rot Max LR
  html += "Rot Max LR [%]<br>";
  html += "<a class=\"pm\" href=\"/rotMaxLRm\">-</a>";
  html += "<span>" + String(rotMaxLR) + "</span>";
  html += "<a class=\"pm\" href=\"/rotMaxLRp\">+</a><br>";

  
  //shiftCL
  html += "shiftCL<br>";
  html += "<a class=\"pm\" href=\"/shiftCLM\">-</a>";
  html += "<span>" + String(shiftCL) + "</span>";
  html += "<a class=\"pm\" href=\"/shiftCLP\">+</a><br>";

  //shiftCR
  html += "shiftCR<br>";
  html += "<a class=\"pm\" href=\"/shiftCRM\">-</a>";
  html += "<span>" + String(shiftCR) + "</span>";
  html += "<a class=\"pm\" href=\"/shiftCRP\">+</a><br>";

  //shiftL
  html += "shiftL<br>";
  html += "<a class=\"pm\" href=\"/shiftLM\">-</a>";
  html += "<span>" + String(shiftL) + "</span>";
  html += "<a class=\"pm\" href=\"/shiftLP\">+</a><br>";

  //shiftR
  html += "shiftR<br>";
  html += "<a class=\"pm\" href=\"/shiftRM\">-</a>";
  html += "<span>" + String(shiftR) + "</span>";
  html += "<a class=\"pm\" href=\"/shiftRP\">+</a><br>";
  

  //delayC
  html += "delayC [msec]<br>";
  html += "<a class=\"pm\" href=\"/delayCM\">-</a>";
  html += "<span>" + String(delayC) + "</span>";
  html += "<a class=\"pm\" href=\"/delayCP\">+</a><br>";

  //delayLR
  html += "delayLR [msec]<br>";
  html += "<a class=\"pm\" href=\"/delayLRM\">-</a>";
  html += "<span>" + String(delayLR) + "</span>";
  html += "<a class=\"pm\" href=\"/delayLRP\">+</a><br>";
   

  //offsetC
  html += "offsetC<br>";
  html += "<a class=\"pm\" href=\"/offsetCM\">-</a>";
  html += "<span>" + String(offsetC) + "</span>";
  html += "<a class=\"pm\" href=\"/offsetCP\">+</a><br>";

  //offsetL
  html += "offsetL<br>";
  html += "<a class=\"pm\" href=\"/offsetLM\">-</a>";
  html += "<span>" + String(offsetL) + "</span>";
  html += "<a class=\"pm\" href=\"/offsetLP\">+</a><br>";

  //offsetR
  html += "offsetR<br>";
  html += "<a class=\"pm\" href=\"/offsetRM\">-</a>";
  html += "<span>" + String(offsetR) + "</span>";
  html += "<a class=\"pm\" href=\"/offsetRP\">+</a><br>";


  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

static void handleBackToMain() {
  handleRoot(); // トップページを再表示
}


static void handleMode() {
  if(Mode == 0){
    Mode = 1;
  }else{
    Mode = 0;
  }
  handleRoot();
}

static void handleGetUp() {
  GetUpBtState = 1;
  if(brakeMode){
    handleBrake();
  }else{
    handleRoot();
  }
}


static void rotYawm() {
  if (rotYaw >= -35) {
    rotYaw -= 5;
  }
  handleRoot();
}
static void rotYawp() {
  if (rotYaw <= 35) {
    rotYaw += 5;
  }
  handleRoot();
}

static void KpYawRotM() {
  if (KpYawRot >= 0) {
    KpYawRot -= 0.1;
    preferences.putFloat("KpYawRot", KpYawRot);
  }
  handleRoot();
}
static void KpYawRotP() {
  if (KpYawRot <= 100) {
    KpYawRot += 0.1;
    preferences.putFloat("KpYawRot", KpYawRot);
  }
  handleRoot();
}

static void handleshiftCLM() {
  if(shiftCL > 0){
    shiftCL -= 5;
    preferences.putInt("shiftCL", shiftCL);
  }
  handleBrake();
}
static void handleshiftCLP() {
  if(shiftCL <= 1000){
    shiftCL += 5;
    preferences.putInt("shiftCL", shiftCL);
  }
  handleBrake();
}

static void handleshiftCRM() {
  if(shiftCR > 0){
    shiftCR -= 5;
    preferences.putInt("shiftCR", shiftCR);
  }
  handleBrake();
}
static void handleshiftCRP() {
  if(shiftCR <= 1000){
    shiftCR += 5;
    preferences.putInt("shiftCR", shiftCR);
  }
  handleBrake();
}


static void handleshiftLM() {
  if(shiftL > 0){
    shiftL -= 5;
    preferences.putInt("shiftL", shiftL);
  }
  handleBrake();
}
static void handleshiftLP() {
  if(shiftL <= 1000){
    shiftL += 5;
    preferences.putInt("shiftL", shiftL);
  }
  handleBrake();
}

static void handleshiftRM() {
  if(shiftR > 0){
    shiftR -= 5;
    preferences.putInt("shiftR", shiftR);
  }
  handleBrake();
}
static void handleshiftRP() {
  if(shiftR <= 1000){
    shiftR += 5;
    preferences.putInt("shiftR", shiftR);
  }
  handleBrake();
}


static void handleloopDelayCM() {
  if(delayC > 0){
    delayC -= 10;
    preferences.putInt("delayC", delayC);
  }
  handleBrake();
}
static void handleloopDelayCP() {
  if(delayC <= 2000){
    delayC += 10;
    preferences.putInt("delayC", delayC);
  }
  handleBrake();
}

static void handleloopDelayLRM() {
  if(delayLR > 0){
    delayLR -= 10;
    preferences.putInt("delayLR", delayLR);
  }
  handleBrake();
}
static void handleloopDelayLRP() {
  if(delayLR <= 2000){
    delayLR += 10;
    preferences.putInt("delayLR", delayLR);
  }
  handleBrake();
}


static void handleoffsetCM() {
  if(offsetC >  -1000){
    offsetC -= 5;
    preferences.putInt("offsetC", offsetC);
    ledcWrite(CH_ServoC_BROWSER, usToDuty(1500 + offsetC));
  }
  handleBrake();
}
static void handleoffsetCP() {
  if(offsetC <= 1000){
    offsetC += 5;
    preferences.putInt("offsetC", offsetC);
    ledcWrite(CH_ServoC_BROWSER, usToDuty(1500 + offsetC));
  }
  handleBrake();
}

static void handleoffsetLM() {
  if(offsetL >  -1000){
    offsetL -= 5;
    preferences.putInt("offsetL", offsetL);
    ledcWrite(CH_ServoL_BROWSER, usToDuty(1500 + offsetL));
  }
  handleBrake();
}
static void handleoffsetLP() {
  if(offsetL <= 1000){
    offsetL += 5;
    preferences.putInt("offsetL", offsetL);
    ledcWrite(CH_ServoL_BROWSER, usToDuty(1500 + offsetL));
  }
  handleBrake();
}

static void handleoffsetRM() {
  if(offsetR >  -1000){
    offsetR -= 5;
    preferences.putInt("offsetR", offsetR);
    ledcWrite(CH_ServoR_BROWSER, usToDuty(1500 + offsetR));
  }
  handleBrake();
}
static void handleoffsetRP() {
  if(offsetR <= 1000){
    offsetR += 5;
    preferences.putInt("offsetR", offsetR);
    ledcWrite(CH_ServoR_BROWSER, usToDuty(1500 + offsetR));
  }
  handleBrake();
}


static void handleKpM() {
  if(Kp >  0){
    Kp -= 1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}
static void handleKpP() {
  if(Kp <= 100){
    Kp += 1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}

static void handleKdM() {
  if(Kd >  0){
    Kd -= 1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}
static void handleKdP() {
  if(Kd <= 100){
    Kd += 1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}

static void handleKwM() {
  if(Kw >  0){
    Kw -= 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}
static void handleKwP() {
  if(Kw <= 30){
    Kw += 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}

static void handleKaM() {
  if(Ka > -30){
    Ka -= 0.1;
    preferences.putFloat("Ka", Ka);
  }
  handleRoot();
}
static void handleKaP() {
  if(Ka <= 30){
    Ka += 0.1;
    preferences.putFloat("Ka", Ka);
  }
  handleRoot();
}

static void handleloopDelayM() {
  if(loopDelay > 0){
    loopDelay -= 1;
    preferences.putInt("loopDelay", loopDelay);
  }
  handleRoot();
}
static void handleloopDelayP() {
  if(loopDelay <= 100){
    loopDelay += 1;
    preferences.putInt("loopDelay", loopDelay);
  }
  handleRoot();
}

static void handleRotMaxCm() {
  if(rotMaxC > 0.0){
    rotMaxC -= 1.0;
    preferences.putFloat("rotMaxC", rotMaxC);
  }
  handleBrake();
}
static void handleRotMaxCp() {
  if(rotMaxC < 100){
    rotMaxC += 1.0;
    preferences.putFloat("rotMaxC", rotMaxC);
  }
  handleBrake();
}

static void handleRotMaxLRm() {
  if(rotMaxLR > 0.0){
    rotMaxLR -= 1.0;
    preferences.putFloat("rotMaxLR", rotMaxLR);
  }
  handleBrake();
}
static void handleRotMaxLRp() {
  if(rotMaxLR < 100.0){
    rotMaxLR += 1.0;
    preferences.putFloat("rotMaxLR", rotMaxLR);
  }
  handleBrake();
}
