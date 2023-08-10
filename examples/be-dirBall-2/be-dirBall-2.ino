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
    //繰り返し実行するものはここに書きます
  Serial.print(r.b(front));
  Serial.print(":");
  Serial.print(r.b(right));
  Serial.print(":");
  Serial.print(r.b(back));
  Serial.print(":");
  Serial.println(r.b(left));
}


/*書き方はArduinoのプログラムと同じです。
ライブラリを使うときは、r.を先につけてください。
3行目のrはなんでもいいですが、変更した際は先に付けるrも変更したものに変更してください。
*/

/*
ライブラリの関数一覧
r.m(left,right); //モーターを動かす。leftとrightには-255~255の値を入れる。正の値で前進、負の値で後退。
r.srmo(); //センサーの値をシリアルモニターに表示する。
r.b(); //前のボールセンサーの値を返す。0~500の値を返す。()の中に１〜４の数字を入れることもできます。
r.b(FRONT); //前のボールセンサーの値を返す。0~500の値を返す。
r.b(RIGHT); //右のボールセンサーの値を返す。0~500の値を返す。
r.b(BACK); //後ろのボールセンサーの値を返す。0~500の値を返す。
r.b(LEFT); //左のボールセンサーの値を返す。0~500の値を返す。
r.l(); //ラインセンサーの値を返す。0~1023の値を返す。
r.bt(); //ボタンの値を返す。highかlowの値を返す。0か1の値を返すがわかりにくくなるので気をつけて。
*/