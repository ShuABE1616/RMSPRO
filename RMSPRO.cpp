/*
Project Name    : RMSPRO
File Name       : RMSPRO.cpp
Creation Date   : c++
author          : Shu ABE,Hibiki SHINOHARA,Yuri HIRATA
update date     : 2023/8/7

Copyright © <2023> RobomatchProject

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "RMSPRO.h"
#include <avr/io.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <TimerOne.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define MT1fr 3
#define MT1ba 9
#define MT2fr 10
#define MT2ba 5 
#define frn   1
#define bac   2
#define IR1   7
#define IR2   6
#define IR3   4
#define IR4   8
#define LINE1 14
#define BTN   2
#define OUTPUT_READABLE_YAWPITCHROLL

volatile static bool Ready_to_start;
volatile int Latest_azim;
volatile double Deg_mpu;
volatile double Degree;
volatile double Median_x;
volatile double Median_y;
volatile double Scale;
volatile int Raw_data[3];
volatile static int _ir1;
volatile static int _ir2;
volatile static int _ir3;
volatile static int _ir4;
volatile bool Calib;
volatile bool az_check_flag = false;

///////////////
/*コンストラクタ*/
////////////////
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


/////////////////////////////////////////////////
/*タイマー割り込み関数 2040マイクロ秒周期で呼び出される*/
/////////////////////////////////////////////////
void RMSPRO::timerISR(void){

  static long count = 0;

  //タイマ割り込みの停止
  Timer1.detachInterrupt();

  /*一定周期ごとに関数を実行*/
  /*タイミングが重なった場合上から優先で実行される*/
  if((count % 5) == 0){//約100Hz
    RMSPRO::azimUpdate();
  }
  else if((count % 23) == 0){//約20Hz
    RMSPRO::irUpdate();
  }
  else{
    /*DO NOTHING*/
  }

  count++;

  /*タイマー割り込みの再開*/
  Timer1.attachInterrupt(timerISR); 
}

//コンストラクタ
RMSPRO::RMSPRO(void)
{
  Ready_to_start = false;
  Median_x = 0;
  Median_y = 0;
  Scale = 1;
  Calib = false;
  Latest_azim = 0;
}

//初期化
void RMSPRO::init(void)
{
  pinMode(MT1fr, OUTPUT);
  pinMode(MT1ba, OUTPUT);
  pinMode(MT2fr, OUTPUT);
  pinMode(MT2ba, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011011;
  ICR1 = 255;
  pinMode(LINE1, INPUT);
  pinMode(BTN, INPUT);
  Serial.begin(9600);
 
      
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif

      // initialize device
      mpu.initialize();

      // verify connection
      mpu.testConnection();

      // load and configure the DMP
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
      
      }
    
    

  /*タイマ割り込み開始*/
  Timer1.initialize(); //2040マイクロ秒周期でタイマ割込みが入る
  Timer1.attachInterrupt(timerISR); 

}


/////////////////////モーターの出力を設定/////////////////////
//beginner mode
void RMSPRO::m(int left, int right)
{
  int mt_state[2];//回転状況
  int mt_power[4];//モーターの出力
  int spd[2] = {left, right};//モーターの速度
  int i;//for文用

  for (i = 0; i < 2; i++)//モーターキャパ越え防止
  {
    if(spd[i] > 255)//モーターの速度を255以上にしない
    {
      spd[i] = 255;
    }
    else if(spd[i] < -255)//モーターの速度を-255以下にしない
    {
      spd[i] = -255;
    }
  }

  for (i = 0; i < 2; i++)//モーターの回転状況を確認
  {
    if (spd[i] > 0)//正転
    {
      mt_power[i * 2] = spd[i];
      mt_power[i * 2 + 1] = 0;
      if (mt_state[i] == frn)
      {
        mt_state[i] = bac;
      }
      mt_state[i] = 1;
    }

    else if (spd[i] < 0)//逆転
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = -spd[i];
      if (mt_state[i] == bac)
      {
        mt_state[i] = frn;
      }
    }

    else//停止
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = 0;
      mt_state[i] = 0;
    }
  }
  analogWrite(MT1fr, mt_power[0]);
  analogWrite(MT1ba, mt_power[1]);
  analogWrite(MT2fr, mt_power[2]);
  analogWrite(MT2ba, mt_power[3]);
}

//advanced mode
void RMSPRO::motor(int left, int right)
{
  int mt_state[2];//回転状況
  int mt_power[4];//モーターの出力
  int spd[2] = {left, right};//モーターの速度
  int i;//for文用

  for (i = 0; i < 2; i++)//モーターキャパ越え防止
  {
    if(spd[i] > 255)//モーターの速度を255以上にしない
    {
      spd[i] = 255;
    }
    else if(spd[i] < -255)//モーターの速度を-255以下にしない
    {
      spd[i] = -255;
    }
  }

  for (i = 0; i < 2; i++)//モーターの回転状況を確認
  {
    if (spd[i] > 0)//正転
    {
      mt_power[i * 2] = spd[i];
      mt_power[i * 2 + 1] = 0;
      if (mt_state[i] == frn)
      {
        mt_state[i] = bac;
      }
      mt_state[i] = 1;
    }

    else if (spd[i] < 0)//逆転
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = -spd[i];
      if (mt_state[i] == bac)
      {
        mt_state[i] = frn;
      }
    }

    else//停止
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = 0;
      mt_state[i] = 0;
    }
  }
  analogWrite(MT1fr, mt_power[0]);
  analogWrite(MT1ba, mt_power[1]);
  analogWrite(MT2fr, mt_power[2]);
  analogWrite(MT2ba, mt_power[3]);
}

/////////////////////モーターの出力を設定end/////////////////////

////////////////////////ボールセンサーの値を返す////////////////////////
//beginner mode
//ボールセンサーの値を返す


int RMSPRO::b(DIR dir)
{
  int ret = 0;
  switch (dir)
  {
    case front:
      ret = _ir1;
      break;
    case right:
      ret = _ir2;
      break;
    case back:
      ret = _ir3;
      break;
    case left:
      ret = _ir4;
      break;
    default:
      ret = 0;
      break;
  }
  return ret;
}

int RMSPRO::b(DIR2 dir)
{
  int ret = 0;
  switch (dir)
  {
    case fr:
      ret = _ir1;
      break;
    case ri:
      ret = _ir2;
      break;
    case ba:
      ret = _ir3;
      break;
    case le:
      ret = _ir4;
      break;
    default:
      ret = 0;
      break;
  }
  return ret;
}

int RMSPRO::b(number num)
{
  int ret = 0;
  switch (num)
  {
    case d1:
      ret = _ir1;
      break;
    case d2:
      ret = _ir2;
      break;
    case d3:
      ret = _ir3;
      break;
    case d4:
      ret = _ir4;
      break;
    default:
      ret = 0;
      break;
  }
  return ret;
}


//advanced mode
//ボールセンサーの値を返す

int RMSPRO::ball(void)
{
   int ret = 0;
  ret = _ir1;
  return ret;
}

//ボールセンサーの値を返す
int RMSPRO::ball(DIR2 dir)
{
  int ret = 0;
  switch (dir)
  {
  case front:
    ret = _ir1;
    break;
  case right:
    ret = _ir2;
    break;
  case back:
    ret = _ir3;
    break;
  case left:
    ret = _ir4;
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

int RMSPRO::ball(DIR dir)
{
  int ret = 0;
  switch (dir)
  {
  case front:
    ret = _ir1;
    break;
  case right:
    ret = _ir2;
    break;
  case back:
    ret = _ir3;
    break;
  case left:
    ret = _ir4;
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

int RMSPRO::ball(number num)
{
  int ret = 0;
  switch (num)
  {
  case d1:
    ret = _ir1;
    break;
  case d2:
    ret = _ir2;
    break;
  case d3:
    ret = _ir3;
    break;
  case d4:
    ret = _ir4;
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}
////////////////////////ボールセンサーの値を返すend////////////////////////

////////////////////////ボールセンサーの計算////////////////////////
void RMSPRO::irUpdate(void)
{

  _ir1 = 0;
  _ir2 = 0;
  _ir3 = 0;
  _ir4 = 0;

  for (int i = 0; i < 500; i++)
  {
    if (digitalRead(IR1) == LOW)
    {
      _ir1++;
    }
    if (digitalRead(IR2) == LOW)
    {
      _ir2++;
    }
    if (digitalRead(IR3) == LOW)
    {
      _ir3++;
    }
    if (digitalRead(IR4) == LOW)
    {
      _ir4++;
    }
  }
}
////////////////////////ボールセンサーの計算end////////////////////////

////////////////////////ラインセンサーの値を返す////////////////////////
//beginner mode
int RMSPRO::l(void)
{
  int val = 0;
  val = analogRead(LINE1);
  return val;
}

//advanced mode
int RMSPRO::line(void)
{
  int val = 0;
  val = analogRead(LINE1);
  return val;
}
////////////////////////ラインセンサーの値を返すend////////////////////////



////////////////////////センサモニタ////////////////////////


//beginner mode
void RMSPRO::srmo(void)
{
  Serial.print("(");
  Serial.print(bt());
  Serial.print(",");
  Serial.print(b(d1));
  Serial.print(",");
  Serial.print(b(d2));
  Serial.print(",");
  Serial.print(b(d3));
  Serial.print(",");
  Serial.print(b(d4));
  Serial.print(",");
  Serial.print(l());
  Serial.print(")");
}


//advanced mode
void RMSPRO::sensorMonitor(void)
{
  Serial.print("(");
  Serial.print(btn());
  Serial.print(",");
  Serial.print(ball(d1));
  Serial.print(",");
  Serial.print(ball(d2));
  Serial.print(",");
  Serial.print(ball(d3));
  Serial.print(",");
  Serial.print(ball(d4));
  Serial.print(",");
  Serial.print(line());
  Serial.print(")");
}
//(ボタン,ボール前(d1),ボール右(d2),ボール後ろ(d3),ボール左(d4),ライン)
////////////////////////センサモニタend////////////////////////

////////////////////////ボタンの値を返す////////////////////////
//beginner mode
int RMSPRO::bt(void)
{
  int val = 0;
  val = digitalRead(BTN);
  return Ready_to_start;
  return val;
}


//advanced mode
int RMSPRO::btn(void)
{
  int val = 0;
  val = digitalRead(BTN);
  return Ready_to_start;
}


////////////////////////ボタンの値を返すend////////////////////////

////////////////////////ジャイロセンサーの値を返す////////////////////////
//ジャイロの計算
void RMSPRO::azimUpdate(void)
{
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

    //I2C割り込みの許可
    interrupts();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Degree = ypr[0] * 180/M_PI;
    }

    //I2C割り込み禁止
    noInterrupts();  
    az_check_flag = false;
  
}

int RMSPRO::dir(void)
{
  int ret = 0;
  ret = Degree - Latest_azim;
  while(ret > 179)
  {
    ret -= 360;
  }
  while(ret < -179)
  {
    ret += 360;
  }
  return ret;
}
