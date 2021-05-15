// ゲームコントローラのジョイスティックデータから，
// 指令速度を計算するクラス
// 作成日：2019年12月30日
// 作成者：上野祐樹
// 編集者：菊池隼(2021/02/10)

#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include <Arduino.h>
#include "define.h"
#include "PIDclass.h"

#define JOY_DEADBAND    ( 5 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( PI / 3.0 * 2.0 )

#define JOYCONMODE      1
#define JOYCONVELPID    2
#define JOYCONPOSIPID   3
#define POSITIONPID     4

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    double refKakudo;
    double tmpPx, tmpPy;

    /*********** 関数宣言 ***********/
    ManualControl(PID *_pidvel, PID *_pidposi);
    
    coords getGlobalVel(unsigned int JoyX, unsigned int JoyY, unsigned int JoyZ);
    coords getLocalVel(double refVx, double refVy, double refVz);
    coords getVel_max(double vel_x, double vel_y, double vel_z);
    coords_4 getCmd(double refVx, double refVy, double refVz);
    double getOmegaCMD(double conZ_or_position,double maxomega, double _robotAngle, int mode); // タイマー割込みで使用
    bool setRefAngle(double angle_deg); //目標角度を設定，angle[deg]

private:
    PID *pidvel;
    PID *pidposi;

    double robotAngle;
    double refAngle;

    int path_num;
    int mode;
    int max_pathnum;

    double conv_length;
    double conv_tnum;

    bool mode_changed;
    bool init_done;

    double tan, per, rot;
};

#endif