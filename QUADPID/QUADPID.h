#ifndef QUADPID_h
#define QUADPID_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class QUADPID
{
  public:
    //constructor
    QUADPID();
    //methods
    float minMax(float value, float min_value, float max_value);
    float pid(float e, float es, float ed, float kp, float ki, float kd);
    void pidAll();
    void motorMix(float throttle);
    void getGains(float kp[3], float ki[3], float kd[3]);
    void getControlConstraints(float minControl, float maxControl);
    void errosCalc(float references[3], float states[3]);
    void toMotors(int m1, int m2, int m3, int m4);
    void reset(float throttle);
    //fields
    float maxControl;
    float minControl;
    float kp[3];
    float ki[3]; 
    float kd[3];
    float e[3] = {0, 0, 0};
    float esum[3] = {0, 0, 0};
    float ed[3] = {0, 0, 0};
    float epast[3] = {0, 0, 0};
    float pulse_length_esc1;
    float pulse_length_esc2;
    float pulse_length_esc3;
    float pulse_length_esc4;
    float uroll;
    float upitch;
    float uyaw;
};

#endif
