#ifndef __PID_H__
#define __PID_H__

#include "src/FpF.hpp"
#include "src/PID/PID_v1.h"
#include "DCMotor.h"

typedef enum {
  CL_OPEN_LOOP = 0,
  CL_POSITION,
  CL_VELOCITY,
} cl_control;

typedef enum {
  TARGET_VELOCITY = 0,
  TARGET_POSITION
} cl_target;

class PIDWrapper {

  public:
    PIDWrapper(float& inputpos, float& inputvelo, DCMotor* motor, int index, int periodms_pos, int periodms_velo);

    void setGains(float kp, float ki, float kd) {
      if (this->mode == CL_VELOCITY) {
        pid_velo->SetTunings(kp, ki, kd);
      }
      if (this->mode == CL_POSITION) {
        pid_pos->SetTunings(kp, ki, kd);
      }
      run();
    };

    void resetGains();

    void setControlMode(cl_control mode) {
      this->mode = mode;
      run();
    };

    void setSetpoint(cl_target control_target, float target) {
      if (control_target == TARGET_VELOCITY) {
        this->targetvelo = target;
      } else if (control_target == TARGET_POSITION) {
        this->targetpos = target;
      }
      run();
    };

    void setMaxAcceleration(float maxAccel) {
      this->maxAcceleration = maxAccel;
      run();
    };

    void setMaxVelocity(float maxVelocity) {
      this->maxVelocity = maxVelocity;
      run();
    };

    void setLimits(int16_t minDuty, int16_t maxDuty) {
      if (mode == CL_POSITION) {
        pid_pos->SetOutputLimits((float)minDuty, (float)maxDuty);
      }
      if (mode == CL_VELOCITY) {
        pid_velo->SetOutputLimits((float)minDuty, (float)maxDuty);
      }
      run();
    };

    void run() {
      pid_velo->SetMode(AUTOMATIC);
      pid_pos->SetMode(AUTOMATIC);
    };

    void stop() {
      pid_velo->SetMode(MANUAL);
      pid_pos->SetMode(MANUAL);
    };

    cl_control mode = CL_VELOCITY;
    float targetpos = 0.0;
    float targetvelo = 0.0;
    float maxAcceleration;
    float maxVelocity;
    int maxDuty = 100;
    int minDuty = 0;
    float actualDuty;
    float velocmd = 0.0f;
    PID* pid_velo;
    PID* pid_pos;
    DCMotor* motor;
};

#endif
