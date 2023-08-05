volatile static int _ir1;
volatile static int _ir2;
volatile static int _ir3;
volatile static int _ir4;

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
  case　front :
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
