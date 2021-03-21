
#include <KF.h>
#include "QUADPID.h"
#include "MPU6050.h"

float sensorReturn[6];
boolean started = 0;
float minControl = -400;
float maxControl = 400;
float FREQ = 250;
float period = (1.0 / FREQ) ;
const int m1 = 4;
const int m2 = 5;
const int m3 = 6;
const int m4 = 7;
#define ROLL 0
#define PITCH 1
#define YAW 2
// global variables
float references[3] = {0, 0, 0};
float throttle = 1000;
QUADPID ctrlr;
KF kf;
MPU6050 mpu;

// kalman initialization
float xroll[2] = {0.1, 0.1};
float Proll[4] = {0.1 , 0, 0, 0.1};

float xpitch[2] = {0.1, 0.1};
float Ppitch[4] = {0.1 , 0, 0, 0.1};

float xyaw = {0};

float  A[4] = {1, 0.1, 0 , 1};
float B[2] = {0 , 1};
float C[2] = {1, 0};
float Q[4] = {1, 0, 0, 10};
float R = 1;

float kp[3] = {1, 1, 1};
float ki[3] = {1, 1, 1};
float kd[3] = {1, 1, 1};

void setup() {
  kf.getABC(A, B, C);
  kf.getQR(Q, R);
  ctrlr.getGains(kp, ki, kd);
  ctrlr.getControlConstraints(minControl, maxControl);
  mpu.setupSensor();
}

void loop() {

  fromKeyboard();
  if (started == 1) {
    float *p;
    p = mpu.readSensor(sensorReturn);
    float rolldGyro = *(p);
    float pitchdGyro = *(p + 1);
    float yawdGyro = *(p + 2);
    float rollAcc = *(p + 3);
    float pitchAcc = *(p + 4);
    float xyaw = *(p + 5);
    kf.kalman(xroll, Proll, rolldGyro, rollAcc);
    kf.kalman(xpitch, Ppitch, pitchdGyro, pitchAcc);

    // error calculations
    float states[3] = {xroll[0], xpitch[0], xyaw};
    ctrlr.errosCalc(references, states);
    ctrlr.pidAll();

    // motor mix
    ctrlr.motorMix(throttle);
    ctrlr.toMotors(m1, m2, m3, m4);
  }
  else {
    // reset
    ctrlr.reset(throttle);
    throttle = 1011;
  }
}

void fromKeyboard() {

  if (Serial.available()) {
    int data = Serial.read();
    switch (data) {
      case 48 : throttle += 5;           // 0
        break;
      case 49 : throttle -= 5; // 1
        break;
      case 50 : references[ROLL] += 1;   // 2
        break;
      case 51 : references[ROLL] -= 1;  // 3
        break;
      case 52 : started = started * (-1);      // 4
        break;
    }
  }

}
