#include <RMSPRO.h>

RMSPRO r;//ここのrはなんでもいいです

void setup()
{
    Serial.begin(9600);
    r.init();
    //上二つはライブラリを使うときに必要です
    //一度だけ実行するものはここに書きます
}
void loop()
{
  Serial.print(r.b(d1));
  Serial.print(":");
  Serial.print(r.b(d2));
  Serial.print(":");
  Serial.print(r.b(d3));
  Serial.print(":");
  Serial.println(r.b(d4));
  //繰り返し実行するものはここに書きます 
}


/*書き方はArduinoのプログラムと同じです。
ライブラリを使うときは、r.を先につけてください。
3行目のrはなんでもいいですが、変更した際は先に付けるrも変更したものに変更してください。
*/

/*
ライブラリの関数一覧
r.motor(left,right); //モーターを動かす。leftとrightには-255~255の値を入れる。正の値で前進、負の値で後退。
r.sensorMonitor(); //センサーの値をシリアルモニターに表示する。
r.ball(); //前のボールセンサーの値を返す。0~500の値を返す。
()の中に入るもの
方向(front)(right)(back)(left)(fr)(ri)(ba)(le)
番号(d1)(d2)(d3)(d4)
r.line(); //ラインセンサーの値を返す。0~1023の値を返す。
r.btn(); //ボタンの値を返す。highかlowの値を返す。0か1の値を返すがわかりにくくなるので気をつけて。
r.sensormonitor//シリアルモニタにセンサーの値を出力します。(ボタン,ボール前(d1),ボール右(d2),ボール後ろ(d3),ボール左(d4),ライン)です。
*/