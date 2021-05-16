// Button.hを勝手に変更した

#ifndef _BUTTON_ENCORDER_H_INCLUDED
#define _BUTTON_ENCORDER_H_INCLUDED

#include <Arduino.h>

#define DIP1_CON 0x01
#define DIP2 0x02
#define DIP3_POSITION_PID 0x04
#define DIP4_SETTING 0x08

#define ON 1
#define OFF 2

class Button{
  public:
  Button(int xpin_num);

  bool button_rise();
  bool button_fall();
  bool button_changed();
  bool get_button_state();
  int  get_count();
  void set_count(int setNum);

  private:
  int counts;
  int pin_num;
  bool pre_button_state;
  bool initial_flag;
};

class DipSW{
  public:
  DipSW();

  int getDipState();

  /*
  * 指定されたDipSwotchがONかOFFを指定し状態の真偽を返す
  *
  * @param one_dip DIP1~DIP4を選択．マクロ名は自分で変更可
  * @param on_or_off DipSwitchがONかOFFを選択
  * 
  */
  bool getBool(int one_dip, int on_or_off);

  private:
  int dipNum;
};

class Encorder{
  public:
  
  Encorder();

  int getEncCount();
  void setEncCount(int setnum);
  int enc_count;
  double count;

  private:

  bool flag_enc;
  int encA, pre_encA, encB, pre_encB;
};

#endif