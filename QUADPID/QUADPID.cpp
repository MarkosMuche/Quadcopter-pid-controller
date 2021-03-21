#include "Arduino.h"
#include "QUADPID.h"
#define ROLL 0
#define PITCH 1
#define YAW 2

QUADPID ::QUADPID() {
}
void QUADPID::pidAll() {
  float eyaw = this->e[YAW];
  float esumyaw = this->esum[YAW];
  float edyaw = this->ed[YAW];
  float kpyaw = this->kp[YAW];
  float kiyaw = this->ki[YAW];
  float kdyaw = this->kd[YAW];
  
  float eroll = this->e[ROLL];
  float esumroll = this->esum[ROLL];
  float edroll = this->ed[ROLL];
  float kproll = this->kp[ROLL];
  float kiroll = this->ki[ROLL];
  float kdroll = this->kd[ROLL];

  float epitch = this->e[PITCH];
  float esumpitch = this->esum[PITCH];
  float edpitch = this->ed[PITCH];
  float kppitch = this->kp[PITCH];
  float kipitch = this->ki[PITCH];
  float kdpitch = this->kd[PITCH];

  this->uroll = this->pid(eroll, esumroll , edroll , kproll, kiroll, kdroll );
  this->upitch = this->pid(epitch, esumpitch , edpitch , kppitch, kipitch, kdpitch );
  this->uyaw = this->pid(eyaw, esumyaw , edyaw , kpyaw, kiyaw, kdyaw );
}
void QUADPID::errosCalc(float references[3], float states[3]) {
  // Calculate current angle errors
  this->e[YAW]   = states[YAW]  -    references[YAW];
  this->e[PITCH] = states[PITCH] - references[PITCH];
  this->e[ROLL]  = states[ROLL]  - references[ROLL];

  // Calculate sum of angle errors : Integral coefficients
  this->esum[YAW]   += this->e[YAW];
  this->esum[PITCH] += this->e[PITCH];
  this->esum[ROLL]  += this->e[ROLL];

  // Keep values in acceptable range
  this->esum[YAW]   = this->minMax(this->esum[YAW],   -400 / this->ki[YAW],   400 / this->ki[YAW]);
  this->esum[PITCH] = this->minMax(this->esum[PITCH], -400 / this->ki[PITCH], 400 / this->ki[PITCH]);
  this->esum[ROLL]  = this->minMax(this->esum[ROLL],  -400 / this->ki[ROLL],  400 / this->ki[ROLL]);

  // Calculate angle error delta : Derivative coefficients
  this->ed[YAW]   = this->e[YAW]   - this->epast[YAW];
  this->ed[PITCH] = this->e[PITCH] - this->epast[PITCH];
  this->ed[ROLL]  = this->e[ROLL]  - this->epast[ROLL];

  // Save current angle error as epast for next time
  this->epast[YAW]   = this->e[YAW];
  this->epast[PITCH] = this->e[PITCH];
  this->epast[ROLL]  = this->e[ROLL];
}
void QUADPID::motorMix(float throttle) {
  this->pulse_length_esc1 = throttle - this->uroll + this->upitch - this->uyaw;
  this->pulse_length_esc2 = throttle + this->uroll + this->upitch + this->uyaw;
  this->pulse_length_esc3 = throttle + this->uroll - this->upitch - this->uyaw;
  this->pulse_length_esc4 = throttle - this->uroll - this->upitch + this->uyaw;
  this->pulse_length_esc1 = this->minMax(this->pulse_length_esc1, 1015, 2000);
  this->pulse_length_esc2 = this->minMax(this->pulse_length_esc2, 1015, 2000);
  this->pulse_length_esc3 = this->minMax(this->pulse_length_esc3, 1015, 2000);
  this->pulse_length_esc4 = this->minMax(this->pulse_length_esc4, 1015, 2000);
}
// this may cause a problem with the arguments (m1,m2,m3,m4)
void QUADPID::toMotors(int m1, int m2, int m3, int m4) {
  float looptimer;
  float looptime = micros();
  float timer1 = looptime + this->pulse_length_esc1;
  float timer2 = looptime + this->pulse_length_esc2;
  float timer3 = looptime + this->pulse_length_esc3;
  float timer4 = looptime + this->pulse_length_esc4;
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, HIGH);

  while (digitalRead(m1) == 1 || digitalRead(m2) == 1 || digitalRead(m3) == 1 || digitalRead(m4) == 1) {
    looptimer = micros();
    if (timer1 <= looptimer) digitalWrite(m1, LOW);
    if (timer2 <= looptimer) digitalWrite(m2, LOW);
    if (timer3 <= looptimer) digitalWrite(m3, LOW);
    if (timer4 <= looptimer) digitalWrite(m4, LOW);
  }
}
void QUADPID::getGains(float kp[3], float ki[3], float kd[3]) {
  this->kp[ROLL] = kp[ROLL];
  this->ki[ROLL] = ki[ROLL];
  this->kd[ROLL] = kd[ROLL];
  this->kp[PITCH] = kp[PITCH];
  this->ki[PITCH] = ki[PITCH];
  this->kd[PITCH] = kd[PITCH];
  this->kp[YAW] = kp[YAW];
  this->ki[YAW] = ki[YAW];
  this->kd[YAW] = kd[YAW];
}
void QUADPID::getControlConstraints(float minControl, float maxControl) {
  this->minControl = minControl;
  this->maxControl = maxControl;
}
void QUADPID::reset(float throttle) {

  this->e[YAW]   = 0;
  this->e[PITCH] = 0;
  this->e[ROLL]  = 0;

  this->esum[YAW]   = 0;
  this->esum[PITCH] = 0;
  this->esum[ROLL]  = 0;

  this->epast[YAW]   = 0;
  this->epast[PITCH] = 0;
  this->epast[ROLL]  = 0;
  throttle = 1011;
}
// local functions
float QUADPID::minMax(float value, float min_value, float max_value) {
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }

  return value;
}
float QUADPID::pid(float e, float esum, float ed, float kp, float ki, float kd) {
  float u = 0;
  // PID = e.Kp + ∫e.Ki + Δe.Kd
  u = kp * e + kd * ed + ki * esum;
  u = this->minMax(u, -minControl, maxControl);
  return u;
}
