#include "src/Encoder/Encoder.h"

class EncoderWrapper {
  public:
    EncoderWrapper(int pinA, int pinB, int index);
    void getRawCount();
    void getOverflowUnderflow();
    void getCountPerSecond();
    void resetCounter(long value);
    int read();
    void setIrqOnCount(long value) {
      targetCount = value;
      irqCountEnabled = true;
    }
    void setIrqOnVelocity(long value) {
      if (value != 0) {
        targetVelocity = (float)(value & 0xFFFFFF);
        //irqRatio = float(value >> 24) / 100.0f;
        irqVelocityEnabled = true;
      } else {
        irqVelocityEnabled = false;
      }
    }
    bool underflow;
    bool overflow;
    float velocity;
    float position;
    float irqRatio = 2.0f;
    bool irqCountEnabled = false;
    bool irqVelocityEnabled = false;
    int targetCount = -1;
    float targetVelocity = -1.0;
  private:
    Encoder* enc;
};
