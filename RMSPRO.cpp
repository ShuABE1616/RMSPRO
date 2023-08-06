/*
Project Name    : RMSPRO
File Name       : RMSPRO.cpp
Creation Date   : c++
author          : Shu ABE,Yuri Hirata
update date     : 2023/8/2

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
#include <TimerOne.h>

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

volatile static int _ir1;
volatile static int _ir2;
volatile static int _ir3;
volatile static int _ir4;
volatile static bool Ready_to_start;

//タイマー割り込み
void RMSPRO::timerISR(void)
{
   static long count = 0;

  //タイマ割り込みの停止
  Timer1.detachInterrupt();
  if((count % 2) == 0)
  {
    RMSPRO::irUpdate();
  }
  else
  {
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
  Timer1.initialize();
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


////////////////////////ボールセンサーの値を返す////////////////////////
//beginner mode
//ボールセンサーの値を返す
/*int RMSPRO::b(void)
{
   int ret = 0;
  ret = _ir1;
  return ret;
}

//ボールセンサーの値を返す
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
}*/

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
  Serial.print("ボール1:");
  Serial.print(b());
  Serial.print("  ");
  Serial.print("ライン1:");
  Serial.print(analogRead(LINE1));
  Serial.print("  ");
  Serial.print("ボタン:");
  Serial.print(digitalRead(BTN));
  Serial.print("  ");
  Serial.println();
}

//advanced mode

void RMSPRO::sensorMonitor(void)
{
  Serial.print("ボール1:");
  Serial.print(b());
  Serial.print("  ");
  Serial.print("ライン1:");
  Serial.print(analogRead(LINE1));
  Serial.print("  ");
  Serial.print("ボタン:");
  Serial.print(digitalRead(BTN));
  Serial.print("  ");
  Serial.println();
}
////////////////////////センサモニタend////////////////////////

////////////////////////ボタンの値を返す////////////////////////
//beginner mode
int RMSPRO::bt(void)
{
  int val = 0;
  val = digitalRead(BTN);
  return val;
}

//advanced mode
int RMSPRO::btn(void)
{
  int val = 0;
  val = digitalRead(BTN);
  return val;
}
////////////////////////advanced mode end////////////////////////