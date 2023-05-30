#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_intr_alloc.h"
#include "m3508.h"
#include <math.h>

// PS4 Controller
#define PS4_ADDR "C8:F0:9E:51:65:8C" // red

// エアシリンダー
#define OPEN_PIN 13 // 1
#define UPDOWN_PIN 12 // 2
#define RING_PIN 14 // 3
#define BRIDGE_PIN 27 // 4

#define DIR_PIN 21

#define rx 4
#define tx 5
#define CAN_SPEED 1000E3

TaskHandle_t thp[3];

int8_t headerByte = 0x55; // 0x55 = 85
int8_t addressByte = 0x00; // 0x00 = 0
int8_t commandByte1 = 0x00; // 昇降機構
int8_t commandByte2 = 0x00; // ローラ速度設定
int8_t commandByte3 = 0x00; // ローラ速度増減
int8_t commandByte4 = 0x00; // 自動昇降
int8_t checksum = 0x00; // 0x00 = 0

int8_t updwn_cmd = 0; // Up:1, Down:-1, Stop:0
int8_t rollr_cmd = 0;
int8_t rollr_spd_cmd = 0;
int8_t auto_updwn_cmd = 0; // 0 or 1
bool prev_bttn_state[10] = {false, false, false, false, false, false, false, false, false, false}; // Share, Options, Left, Right, Touchpad, R1, ○, △, □, ×

int core0a_free_stack = 0;
int core0b_free_stack = 0;
int core1b_free_stack = 0;
int core1m_free_stack = 0;

float l_x = 0.0; // 左スティックのX軸
float l_y = 0.0; // 左スティックのY軸
float r_x = 0.0; // 右スティックのX軸
float r_y = 0.0; // 右スティックのY軸

float lstick_x = l_x;
float lstick_y = l_y;
float rstick_x = r_x; 
float rstick_y = r_y; 

float sin45 = 0.7071;
float cos45 = 0.7071;
float length = 0.5; // ロボットの車輪間距離(m)

int16_t current_data[4] = {0, 0, 0, 0}; // 送信するデータ
uint8_t send_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 送信するデータ

int16_t mangle[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M3508の8個分の角度値(16bit)
int16_t mrpm[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M3508の8個分の回転数(16bit)
int16_t mtorque[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M3508の8個分のトルク値(16bit)

int id[4] = {0x201, 0x202, 0x203, 0x204}; // M3508のID
int num = 0;

// PID制御用
float Kp = 140; // 比例ゲイン(元180) (RRなし：100)
float Ki = 0.0; // 積分ゲイン
float Kd = 0.0; // 微分ゲイン

float prev_time = 0; // 前回の時間

int16_t target_rpm[4] = {0, 0, 0, 0}; // 目標回転数
int16_t error[4] = {0, 0, 0, 0}; // 偏差
int16_t integral[4] = {0, 0, 0, 0}; // 積分
int16_t derivative[4] = {0, 0, 0, 0}; // 微分
int16_t prev_error[4] = {0, 0, 0, 0}; // 前回の偏差

float front_left_speed = 0.0;
float front_right_speed = 0.0;
float rear_left_speed = 0.0;
float rear_right_speed = 0.0;

void Core0a(void *args); // PS4 Controller
void Core0b(void *args);
void Core1b(void *args);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(OPEN_PIN, OUTPUT);
  pinMode(UPDOWN_PIN, OUTPUT);
  pinMode(RING_PIN, OUTPUT);
  pinMode(BRIDGE_PIN, OUTPUT);

  // CAN
  while (!can_init(rx, tx, CAN_SPEED));

  PS4.begin(PS4_ADDR);
  Serial.println("> PS4: Started");
  Serial.println("> PS4: Press PS button to connect");
  while (!PS4.isConnected()) {
    Serial.println("> PS4: Connecting...");
    delay(1000);
  }
  
  Serial.println("> PS4: Connected");
  PS4.setLed(255, 255, 255);
  PS4.setRumble(200, 200);
  PS4.sendToController();
  delay(1000);
  PS4.setRumble(0, 0);
  PS4.sendToController();

  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); // (タスク名, タスクのサイズ, パラメータ, 優先度, タスクハンドル, コア番号)
  xTaskCreatePinnedToCore(Core0b, "Core0b", 4096, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(Core1b, "Core1b", 4096, NULL, 3, &thp[2], 1); 
}

void loop() {
  // コントローラ入力から，各車輪の回転速度を計算
  front_left_speed = lstick_x*(-cos45) + lstick_y*(-sin45) + rstick_x*length;
  front_right_speed = lstick_x*(-sin45) + lstick_y*cos45 + rstick_x*length; 
  rear_left_speed = lstick_x*sin45 + lstick_y*(-cos45) + rstick_x*length;
  rear_right_speed = lstick_x*cos45 + lstick_y*sin45 + rstick_x*length;

  target_rpm[0] = (int16_t)front_left_speed;
  target_rpm[1] = (int16_t)front_right_speed;
  target_rpm[2] = (int16_t)rear_left_speed;
  target_rpm[3] = (int16_t)rear_right_speed;

  float dt = millis() - prev_time;
  prev_time = millis();

  // PID制御(速度制御)
  for (int i = 0; i < 4; i++){
      error[i] = target_rpm[i] - mrpm[i]; // 誤差(目標値 - 現在値)
      integral[i] += error[i] * dt; // 積分
      derivative[i] = (error[i] - prev_error[i]) / dt; // 微分
      prev_error[i] = error[i]; // 前回の誤差を保存
  }

  // 電流値の計算(-10000 ~ 10000)
  for (int i = 0; i < 4; i++){
      current_data[i] = (Kp * error[i] + Ki * integral[i] + Kd * derivative[i]); // PID出力
      current_data[i] = constrain(current_data[i], -10000, 10000);
  }
  m3508_read_data(id[num], mangle, mrpm, mtorque);
  num = (num + 1) % 4;
  // 送信するデータを作成
  m3508_make_data(current_data, send_data);
  // 送信
  m3508_send_data(send_data);
  core1m_free_stack = uxTaskGetStackHighWaterMark(NULL);
  delay(2);
}

void Core0a(void *args) {
  Serial.println("> Core0a: Started");
  while (1) {
    if (PS4.LatestPacket() && PS4.isConnected()) {
      // 移動・回転
      l_x = PS4.LStickX() / 127.0;
      l_y = PS4.LStickY() / 127.0;
      r_x = PS4.RStickX() / 127.0;
      // r_y = PS4.RStickY() / 127.0;

      if (l_x < 0.1 && l_x > -0.1) l_x = 0;
      if (l_y < 0.1 && l_y > -0.1) l_y = 0;
      if (r_x < 0.1 && r_x > -0.1) r_x = 0;

      l_x = -l_x * abs(l_x);
      l_y = l_y * abs(l_y);
      r_x = r_x * abs(r_x);

      // 速度半減
      if (PS4.L1()){
          l_x *= 0.6;
          l_y *= 0.6;
          r_x *= 0.6;
      }
      if (PS4.Up()){
          updwn_cmd = 1;
      }
      if (PS4.Down()){
          updwn_cmd = -1;
      }
      if (!PS4.Up() && !PS4.Down()){
          updwn_cmd = 0;
      }
      if (PS4.Circle() && prev_bttn_state[6] == false) {
        if (PS4.L1()){
          auto_updwn_cmd = 1;
        } else {
          rollr_cmd = 1;
        }
      }
      if (PS4.Triangle() && prev_bttn_state[7] == false) {
        if (PS4.L1()){
          auto_updwn_cmd = 2;
        } else {
          rollr_cmd = 2;
        }
      }
      if (PS4.Square() && prev_bttn_state[8] == false) {
        if (PS4.L1()){
          auto_updwn_cmd = 3;
        } else {
          rollr_cmd = 3;
        }
      }
      if (PS4.Cross() && prev_bttn_state[9] == false) { // 全て停止
        if (PS4.L1()){
          auto_updwn_cmd = 4;
        } else {
          updwn_cmd = 0;
          rollr_cmd = 0;
          rollr_spd_cmd = 0;
        }
      }
      if (PS4.Left() && prev_bttn_state[2] == false){
        rollr_spd_cmd -= 1;
      }
      if (PS4.Right() && prev_bttn_state[3] == false){
        rollr_spd_cmd += 1;
      }
      // 回収機構（開閉）デフォルト：開
      if (PS4.Share() && prev_bttn_state[0] == false){
        digitalWrite(OPEN_PIN, !(digitalRead(OPEN_PIN)));
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      // 回収機構（上下）条件：リング射出機構が閉じている，昇降機構が0度，回収機構が開いている
      if (PS4.Options() && prev_bttn_state[1] == false && digitalRead(RING_PIN) == false){
        if (digitalRead(UPDOWN_PIN) == true && digitalRead(OPEN_PIN) == true) {
          digitalWrite(UPDOWN_PIN, LOW);
        } else {
          digitalWrite(UPDOWN_PIN, HIGH);
        }
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      // リング射出（回収機構が上がっている）
      if (PS4.R1() && prev_bttn_state[5] == false && digitalRead(UPDOWN_PIN) == false){
        digitalWrite(RING_PIN, !(digitalRead(RING_PIN)));
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      // 橋渡し
      if (PS4.Touchpad() && prev_bttn_state[4] == 0) {
        digitalWrite(BRIDGE_PIN, !(digitalRead(BRIDGE_PIN))); // タッチパッドのとき，橋渡し
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      prev_bttn_state[0] = PS4.Share();
      prev_bttn_state[1] = PS4.Options();
      prev_bttn_state[2] = PS4.Left();
      prev_bttn_state[3] = PS4.Right();
      prev_bttn_state[4] = PS4.Touchpad();
      prev_bttn_state[5] = PS4.R1();
      prev_bttn_state[6] = PS4.Circle();
      prev_bttn_state[7] = PS4.Triangle();
      prev_bttn_state[8] = PS4.Square();
      prev_bttn_state[9] = PS4.Cross();
    } else {
      updwn_cmd = 0;
      rollr_cmd = 0;
      rollr_spd_cmd = 0;
      l_x = 0;
      l_y = 0;
      r_x = 0;
    }
    lstick_x = l_x * 200;
    lstick_y = l_y * 200;
    rstick_x = r_x * 200;
    delay(10);
    core0a_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Core0b(void *args) {
  Serial.println("> Core0b: Started");
  while (1) {
    commandByte1 = updwn_cmd;
    commandByte2 = rollr_cmd;
    commandByte3 = rollr_spd_cmd;
    commandByte4 = auto_updwn_cmd;
    checksum = headerByte + addressByte + commandByte1 + commandByte2 + commandByte3 + commandByte4;
    Serial2.write(headerByte);
    Serial2.write(addressByte);
    Serial2.write(commandByte1);
    Serial2.write(commandByte2);
    Serial2.write(commandByte3);
    Serial2.write(commandByte4);
    Serial2.write(checksum);
    Serial2.flush();
    auto_updwn_cmd = 0;
    delay(10);
    core0b_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Core1b(void *args) {
  Serial.println("> Core1b: Started");
  while (1) {
    // Serial.print(core0a_free_stack);
    // Serial.print(", ");
    // Serial.print(core0b_free_stack);
    // Serial.print(", ");
    // Serial.print(core1b_free_stack);
    // Serial.print(", ");
    // Serial.print(core1m_free_stack);
    // mrpmを表示
    Serial.print(String(lstick_x) + ", " + String(lstick_y) + ", " + String(rstick_x));
    Serial.print(" | ");
    Serial.print(String(mrpm[0]) + ", " + String(mrpm[1]) + ", " + String(mrpm[2]) + ", " + String(mrpm[3]));
    // Serial.print(" | ");
    // Serial.print(String(mtorque[0]) + ", " + String(mtorque[1]) + ", " + String(mtorque[2]) + ", " + String(mtorque[3]));
    Serial.println();

    delay(10); 
    core1b_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}

/*
白：0C:B8:15:D8:64:76
青：0C:B8:15:C5:1C:C4
黒：60:8C:4A:71:05:E2
黒2：0C:B8:15:F8:3C:D8
赤：C8:F0:9E:51:65:8C
*/

