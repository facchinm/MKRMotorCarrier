#include "PID.h"
#include "Events.h"

static PIDWrapper* obj[2] = {NULL, NULL};

//extern DCMotor dcmotors[2];// necessary for workaround below

void calculatePID_wrapper(void* arg) {
  static float prevvelocmd[2] = {0.0f, 0.0f};

  for (int i = 0; i < 2; i++) {

    if (obj[i]->mode == CL_POSITION) {
      if (obj[i]->pid_pos->Compute()) {
        //slew limit velocity command with max accel
        float curvelocmd = obj[i]->velocmd;
        if ((prevvelocmd[i] - curvelocmd) > obj[i]->maxAcceleration) curvelocmd = prevvelocmd[i] - obj[i]->maxAcceleration;//limit decel
        if ((curvelocmd - prevvelocmd[i]) > obj[i]->maxAcceleration) curvelocmd = prevvelocmd[i] + obj[i]->maxAcceleration;//limit accel
        //copy position PID output to velocity PID setpoint
        obj[i]->targetvelo = curvelocmd;
        //save curvelocmd for next iteration
        prevvelocmd[i] = curvelocmd;
      }
    }

    if (obj[i]->pid_velo->Compute()) {
      int dutyout = int32_t(obj[i]->actualDuty);
      //obj[i]->motor->setDuty(dutyout);  not working so using line below instead

      //deadzone compensation
      if (dutyout > 0) dutyout += 13;
      if (dutyout < 0) dutyout -= 13;
      obj[i]->motor->setDuty(dutyout);
    }
  }
}

static float KP_DEFAULT, KI_DEFAULT, KD_DEFAULT;

PIDWrapper::PIDWrapper(float& inputpos, float& inputvelo, DCMotor* motor, int index, int periodms_velo , int periodms_pos) {

  KP_DEFAULT = 2.0f;
  KI_DEFAULT = 0.1f;
  KD_DEFAULT = 0.0f;

  pid_pos = new PID(&inputpos, &velocmd, &targetpos, KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, DIRECT);
  pid_velo = new PID(&inputvelo, &actualDuty, &targetvelo, KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, DIRECT);
  pid_pos->SetSampleTime(periodms_pos);
  pid_velo->SetSampleTime(periodms_velo);
  pid_pos->SetOutputLimits(float((int16_t)-30), float((int16_t)30));  // position pid can only command +/- max_velo
  pid_velo->SetOutputLimits(float((int16_t)-100), float((int16_t)100)); // velocity pid can only command +/- 100 PWM duty cycle

  run();

  this->motor = motor;
  this->motor->pid = this;
  obj[index] = this;
  if (index == 0) {
    // recalculate every millisecond
    registerTimedEvent(calculatePID_wrapper, this, 0);
  }
}

void PIDWrapper::resetGains() {
  cl_control prev_mode = this->mode;
  this->mode = CL_VELOCITY;
  setGains(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT);
  this->mode = CL_POSITION;
  setGains(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT);
  this->mode = prev_mode;
  run();
};
