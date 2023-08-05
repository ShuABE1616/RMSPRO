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

//コンストラクタ
RMSPRO::RMSPRO(void){
}

void RMSPRO::timerISR(void){

  static long count = 0;

  //タイマ割り込みの停止
  Timer1.detachInterrupt();

  /*一定周期ごとに関数を実行*/
  /*タイミングが重なった場合上から優先で実行される*/
  if((count % 23) == 0){//約20Hz
    RMSPRO::irUpdate();
  }
  
  else{
    /*DO NOTHING*/
  }

  count++;

  /*タイマー割り込みの再開*/
  Timer1.attachInterrupt(timerISR); 
}

/////////////////////ボールセンサー/////////////////////
int RMSPRO::b(void){
  int ret = 0;
  ret = _ir1;
  return ret;
}

int RMSPRO::b(DIRECTION dir){
  int ret = 0;
  switch (dir)
  {
  case front :
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


RMSPRO::init(void){
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(LINE1, INPUT);
  pinMode(BTN, INPUT);
  Serial.begin(9600);
  Timer1.initialize();
  Timer1.attachInterrupt(timerISR);
} 