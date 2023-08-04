typedef enum //ラインセンサとボールセンサの方向指定で使用
{
	front,
	right,
	back,
	left,
} DIRECTION;


/*TakiroboF1クラス*/
class RMSPRO
{
public:
    RMSPRO(void);
    void init(void);
    int b(void);
    int b(DIRECTION dir);
    
private: 
    static void irUpdate(void);
    static void timerISR(void);
};
#endif