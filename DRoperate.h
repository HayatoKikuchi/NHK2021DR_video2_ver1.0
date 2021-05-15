// DRに関連するクラス
// 旧DRBlue.h

#ifndef DROPERATE_h
#define DROPERATE_h

#include <Arduino.h>
#include "define.h"
#include "Button_Encorder.h"
#include "lpms_me1Peach.h"
#include "phaseCounterPeach.h"
#include "LCDclass.h"
#include "Button_Encorder.h"
#include "PIDclass.h"

#define _PUSHED     1
#define _RELEASED   2

struct expand_value
{
  bool flag_pahse1;
  bool flag_pahse2;
};

class DRoperate
{
public:
    /*********** 変数宣言 ***********/
    coords position; //自己位置(x,y,z)

    /*********** 関数宣言 ***********/
    DRoperate(lpms_me1 *_lpms, phaseCounter *_enc1, phaseCounter *_enc2);
    void updateRobotPosition(void); //自己推定を行う
    void updateRoboAngle(void); //ロボットの姿勢(角度)のみ取得
    void setPosition(double x, double y); //自己位置の初期化
    void setAngleOfset(double z_rad);
    void init_positon(void); //汎用基板の基本的なセットアップを行う
    void DRsetup(void); //DRに関するセットアップを行う
    void allOutputLow(void); //全てのデジタル出力をLOWにする
    void LEDblink(byte pin, int times, int interval); //LEDを点滅させる
    void RGB_led(int period); //フルカラーLEDを奇麗に光らせる

    coords getPosition();
    coords getVelocity();

private:
    /****自己位置推定用の変数とクラス****/
    lpms_me1 *lpms;
    phaseCounter *enc1;
    phaseCounter *enc2;
    double encX_rad , encX  ,pre_encX;
    double encY_rad , encY , pre_encY;
    double x_axis_prime, y_axis_prime;
    double Angle_ofset;
    coords prePosition;
    coords robotvelocity;
};

class PIDsetting
{
public:
    PIDsetting(PID *_pid, myLCDclass *_LCD, Encorder *_encorder);

    void init();
    void task(bool flag_500ms,bool up, bool down,char moji[],bool flag);
private:
    PID *pid;
    myLCDclass *LCD;
    Encorder *encorder;
    bool flag_lcd;
};

#endif