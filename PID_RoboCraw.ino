#include "MotorController.h"
#include <Arduino.h>
#include <Servo.h>


const int power_supply_pin = 10;
#define PIN_MOTOR_L 5
#define PIN_MOTOR_R 4
#define PIN_ENCODER_L_A 3
#define PIN_ENCODER_L_B 19
#define PIN_ENCODER_R_A 2
#define PIN_ENCODER_R_B 18
#define PWM_OUT_PIN0  5
#define PWM_OUT_PIN1  4

Servo myservo0, myservo1;
unsigned long current_time = 0, prev_time_10ms = 0, prev_time_100ms;
String reciev_str, send_str;
#define MOTOR_NUM 2
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
MotorController motor_controllers[2];


float L_KP = 1.0;   // 標準ESC
float L_KI = 0.06;  // 標準ESC
float L_KD = 0.1;   // 標準ESC

float R_KP = 1.0;     // 標準ESC
float R_KI = 0.06;    // 標準ESC
float R_KD = 0.1;     // 標準ESC

float L_LPF = 0.00;   // 標準ESC
float R_LPF = 0.00;   // 標準ESC

//追記(2022/11/30 吉田)
float R_WHEEL_RADIUS = 0.0645 ; //車輪の半径(m)
float L_WHEEL_RADIUS = 0.0645 ; //車輪の半径(m)
float CURVATURE = 0.2 ; //曲率(曲線の曲がり具合)
float LINEAR_RATIO = 0.55 ;
float ANGULAR_RATIO = 0.25 ;
float TREAD = 0.455 ; //タイヤ間の距離(m)

int split(String data, char delimiter, String *dst) {
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if (tmp == delimiter) {
      index++;
      if (index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

void leftEncHandler() {
  motor_controllers[MOTOR_LEFT].updateEnc();
}

void rightEncHandler() {
  motor_controllers[MOTOR_RIGHT].updateEnc();
}

void setup() {
  Serial.begin(115200);
  pinMode(power_supply_pin, OUTPUT); //エンコーダーの給電用
  digitalWrite(power_supply_pin, HIGH);
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効

  // LEFT
  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, true);
  // RIGHT
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, true);

  delay(10);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);
  // 外部割込みピンは直接レジスタをたたく。（Arduinoコンパイラの場合、同じブロックであるD4~D7も反応してしまう）

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_controllers[i].driveMotor();
  }

  delay(500); // 10msでもOKでした。確実に回すために設定（気持ち）
}

void send_speed()
{

  //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。開発用
  //Serial.print(motor_controllers[MOTOR_LEFT].rpm_); // 制御量を見るため。開発用
  //Serial.print("\n");
  //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。開発用
  //Serial.println(motor_controllers[MOTOR_RIGHT].rpm_);
  //Serial.print("0.0");  // カウンタが見たいだけ。あとで戻す

}

void job_10ms()
{
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_controllers[i].driveMotor();
  }

}

void job_100ms()
{
  send_speed();
}

void kinematics(String *linear_angular, int *cmd_rpm) {
  float linear_speed_ = linear_angular[0].toFloat();
  float angular_speed_ = linear_angular[1].toFloat();
  if (linear_speed_ > 3) {
    linear_speed_ = 3;
  }
  if (angular_speed_ > 2) {
    angular_speed_ = 1;
  }
  float new_right_velocity = 0.0; //更新速度(right)
  float new_left_velocity = 0.0; //更新速度(left)
  float right_velocity = 0.0;
  float left_velocity = 0.0;
  float l_rpm = 0.0;
  float r_rpm = 0.0;
  //各駆動輪の速度
  new_right_velocity = -(linear_speed_ * LINEAR_RATIO + angular_speed_ * ANGULAR_RATIO * (1 + (TREAD / 2) * CURVATURE)) ;
  new_left_velocity = linear_speed_ * LINEAR_RATIO - angular_speed_ * ANGULAR_RATIO * (1 + (TREAD / 2) * CURVATURE) ;
  if (right_velocity != new_right_velocity || left_velocity != new_left_velocity) {
    right_velocity = new_right_velocity ;
    left_velocity = new_left_velocity ;
  }
  //velocity to rpm変換
  cmd_rpm[0] = (right_velocity / R_WHEEL_RADIUS) * (60 / (2 * PI)) ;
  cmd_rpm[1] = (left_velocity / L_WHEEL_RADIUS) * (60 / (2 * PI)) ;
}

void recieve_cmd() {
  Serial.println("receive_cmd"); //文字列型で価を受け取る(例)"100,100"
  reciev_str = Serial.readStringUntil('\n');
  if (reciev_str.length() > 0) {
    String sp_reciev_str[2];
    split(reciev_str, ',', sp_reciev_str); //コンマで左右のrpmを区別する
    int cmd_rpm[2] ;
    kinematics(sp_reciev_str, cmd_rpm);
    for (int i = 0 ; i < 2 ; i ++ ) {
      Serial.println(cmd_rpm[i]);
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(cmd_rpm[i]);
    }
  }
}



void loop()
{
  current_time = micros();  // オーバーフローまで約40分

  // 10msJob:drive_motor,calc_rpm
  if (current_time - prev_time_10ms > 10000) { // TODO 10秒で1msくらいズレる
    job_10ms();
    prev_time_10ms = current_time;
  }
  // 100msJob:print_rpm
  if (current_time - prev_time_100ms > 100000) { // TODO 1秒で1msくらいズレる
    job_100ms();
    prev_time_100ms = current_time;
  }

  if (Serial.available() > 0) {
    recieve_cmd();
  }
}
