// ゲームコントローラのジョイスティックデータから，
// 指令速度を計算するクラス
// 作成日：2019年12月30日
// 作成者：上野祐樹

#include "ManualControl.h"
#include "Filter.h"
#include "PIDclass.h"

Filter velX_filter(INT_TIME);
Filter velY_filter(INT_TIME);
Filter velZ_filter(INT_TIME);

ManualControl::ManualControl(PID *_pidvel, PID *_pidposi)
{
  pidvel = _pidvel;
  pidposi = _pidposi;
  velX_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
  velY_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
  velZ_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
}

coords ManualControl::getGlobalVel(unsigned int JoyX, unsigned int JoyY, unsigned int JoyZ){
  int joyraw;
  coords rawV, refV;

  // ジョイスティックから指令速度を計算する
  joyraw = (int)JoyX - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.x = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
  }else{
    rawV.x = 0.0;
  }
  refV.x = velX_filter.LowPassFilter(rawV.x);

  joyraw = (int)JoyY - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.y = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
  }else{
    rawV.y = 0.0;
  }
  refV.y = velY_filter.LowPassFilter(rawV.y);

  joyraw = (int)JoyZ - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.z = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXANGVEL;
  }else{
    rawV.z = 0.0;
  }
  refV.z = velZ_filter.LowPassFilter(rawV.z);
  //refV.z = rawV.z;

  return refV;
}

double ManualControl::getOmegaCMD(double conZ_or_position,double maxomega, double _robotAngle, int mode)
{
  robotAngle = _robotAngle;
  double robot_omega;
  static double pre_robotAngle;
  robot_omega = (robotAngle - pre_robotAngle)/INT_TIME;
  pre_robotAngle = robotAngle;
  double refOmega;

  switch (mode)
  {
  case JOYCONMODE:
    refOmega = conZ_or_position;
    break;
  
  case JOYCONVELPID:
    refOmega = pidposi->getCmd(conZ_or_position,robot_omega,maxomega);
    break;
  
  case JOYCONPOSIPID:
    refAngle += conZ_or_position * 0.01;
    refOmega = pidvel->getCmd(pidposi->getCmd(refAngle,robotAngle,maxomega),robot_omega,maxomega);
    break;
  
  case POSITIONPID:
    refOmega = pidvel->getCmd(pidposi->getCmd(conZ_or_position,robotAngle,maxomega),robot_omega,maxomega);
    break;

  default:
    refOmega = 0.0;
    break;
  }

  return refOmega;
}

bool ManualControl::setRefAngle(double angle_deg)
{
  refAngle = radians(angle_deg);
  return true;
}

coords ManualControl::getLocalVel(double vel_x, double vel_y, double vel_z){
  coords refVel;
  refVel.x = +vel_x*cos(robotAngle) + vel_y*sin(robotAngle);
  refVel.y = -vel_x*sin(robotAngle) + vel_y*cos(robotAngle);
  refVel.z = vel_z;

  return refVel;
}

coords ManualControl::getVel_max(double vel_x, double vel_y, double vel_z){
  coords refV;
  double vel;
  vel = sqrt(vel_x*vel_x + vel_y*vel_y);
  if(vel <= 0.0){
    refV.x = vel_x;
    refV.y = vel_y;
  }else if(fabs(vel_x) > fabs(vel_y)){
    refV.x = vel_x*fabs(vel_x) / vel;
    refV.y = vel_y*fabs(vel_x) / vel;
  }else if(fabs(vel_x) <= vel_y){
    refV.x = vel_x*fabs(vel_y) / vel;
    refV.y = vel_y*fabs(vel_y) / vel;
  }
  refV.z = vel_z;
  
  return refV;
}

coords_4 ManualControl::getCmd(double refVx, double refVy, double refVz){

  //**ロボットの正面をx軸の正方向として第n象限のモータをMnとする**//

  /* 
    *      X
    *      ^
    *   M1 | M4
    * Y<---+----
    *   M2 | M3
    *      |
  */

  coords_4 refOmega;
  refOmega.i   = (refVx - refVy - refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.ii  = (refVx + refVy - refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.iii = (refVx - refVy + refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.iv  = (refVx + refVy + refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;

  coords_4 VelCmd;
  VelCmd.i   = refOmega.i   * _4RES_2PI_WHEEL;
  VelCmd.ii  = refOmega.ii  * _4RES_2PI_WHEEL;
  VelCmd.iii = refOmega.iii * _4RES_2PI_WHEEL;
  VelCmd.iv  = refOmega.iv  * _4RES_2PI_WHEEL;

  return VelCmd;
}