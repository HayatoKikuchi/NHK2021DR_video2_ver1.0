// 旧DRBlue.cpp
#include "DRoperate.h"

DRoperate::DRoperate(lpms_me1 *_lpms, phaseCounter *_enc1, phaseCounter *_enc2)
{
  lpms = _lpms;
  enc1 = _enc1;
  enc2 = _enc2;

  pre_encX = 0.0;
  pre_encY = 0.0;
  Angle_ofset = 0.0;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;
}

PIDsetting::PIDsetting(PID *_pid, myLCDclass *_LCD, Encorder *_encorder)
{
  pid = _pid;
  LCD = _LCD;
  encorder = _encorder;
  flag_lcd = true;
}

/****自己位置推定の関数****/
void DRoperate::updateRobotPosition()
{
  encX_rad = -1*(double)enc1->getCount() * _2PI_RES4;
  encY_rad = (double)enc2->getCount() * _2PI_RES4;

  encX = RADIUS_X * encX_rad;
  encY = RADIUS_Y * encY_rad;
  x_axis_prime = encX - pre_encX;
  y_axis_prime = encY - pre_encY;

  position.z = (double)lpms->get_z_angle() + Angle_ofset;
  position.x += x_axis_prime*cos(position.z) - y_axis_prime*sin(position.z);
  position.y += x_axis_prime*sin(position.z) + y_axis_prime*cos(position.z);

  pre_encX = encX;
  pre_encY = encY;
}

void DRoperate::updateRoboAngle()
{
  position.z = (double)lpms->get_z_angle() + Angle_ofset;
}

void DRoperate::setAngleOfset(double z_rad)
{
  Angle_ofset = z_rad;
}

void DRoperate::setPosition(double x, double y) // x[m]，y[m]
{
  position.x = x;
  position.y = y;
}

coords DRoperate::getPosition()
{
  return position;
}

void DRoperate::DRsetup()
{   
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ

  pinMode(PIN_SUPPORT_LEFT,OUTPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_USER, OUTPUT);
}

void DRoperate::init_positon()
{
  //LPMS-ME1の初期化
  if(lpms->init() == 1){
    analogWrite(PIN_LED_RED,0);
    LEDblink(PIN_LED_GREEN, 3, 100);  // 初期が終わった証拠にブリンク
    analogWrite(PIN_LED_GREEN,255);
    delay(100);
  }
  
  enc1->init();
  enc2->init();
}

// LEDをチカチカさせるための関数
void DRoperate::LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void DRoperate::RGB_led(int period){
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  count += period; // ここで光る周期を変えられる(はず)
  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else{
    count = 0;
  }
}

void DRoperate::allOutputLow()
{
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, LOW);
  digitalWrite(PIN_LED_4, LOW);
  digitalWrite(PIN_LED_ENC, LOW);

  digitalWrite(PIN_SUPPORT_LEFT, LOW);
}

void PIDsetting::init()
{
  flag_lcd = true;
}

void PIDsetting::task(bool flag_500ms,bool up, bool down,char moji[],bool flag)
{
  static int pid_setting_mode;
  static double kp, ki, kd;
  static bool init_kp, init_ki, init_kd;
  
  if(flag)
  {
    if(flag_lcd)
    { 
      LCD->clear_display();
      LCD->write_str(moji,LINE_1,1);
      flag_lcd = false;
      init_kp = true;
      kp = pid->Kp; ki = pid->Ki; kd = pid->Kd;
      pid_setting_mode = 1;
    }

    if(down)   pid_setting_mode++;
    else if(up) pid_setting_mode--;
    if(pid_setting_mode == 0) pid_setting_mode = 3;
    else if(pid_setting_mode == 4) pid_setting_mode = 1;

    switch (pid_setting_mode)
    {
    case 1:
      init_ki = true;
      init_kd = true;
      if(init_kp)
      { 
        LCD->write_str("Kp ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * pid->Kp));
        init_kp = false;
      }
      kp = 0.1*(double)encorder->getEncCount();
      if(flag_500ms)
      {
        LCD->write_str("          ",LINE_3,4);
        LCD->write_double(pid->Kp,LINE_3,4);
      }
      break;
    
    case 2:
      init_kp = true;
      init_kd = true;
      if(init_ki)
      {
        LCD->write_str("Ki ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * pid->Ki));
        init_ki = false;
      }
      ki = 0.1*(double)encorder->getEncCount();
      if(flag_500ms)
      {
        LCD->write_str("          ",LINE_3,4);
        LCD->write_double(pid->Ki,LINE_3,4);
      }
      break;
    
    case 3:
      init_kp = true;
      init_ki = true;
      if(init_kd)
      {
        LCD->write_str("Kd ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * pid->Kd));
        init_kd = false;
      }
      kd = 0.1*(double)encorder->getEncCount();
      if(flag_500ms)
      {
        LCD->write_str("          ",LINE_3,4);
        LCD->write_double(pid->Kd,LINE_3,4);
      }
      break;

    default:
      break;
    }
    pid->setPara(kp,ki,kd);
  }
  else
  {
    PIDsetting::init();
  }
}