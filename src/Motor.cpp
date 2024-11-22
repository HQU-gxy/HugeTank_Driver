#include <ulog.h>
#include "Motor.h"

Motor::Motor(uint8_t alarm_pin,
             uint8_t enable_pin,
             uint8_t direction_pin,
             TimerCfg feedback_timer,
             TimerCfg pwm_timer)
    : alarmPin(alarm_pin),
      enablePin(enable_pin),
      directionPin(direction_pin),
      feedbackChannel(feedback_timer.channel),
      pwmChannel(pwm_timer.channel)
{
  pinMode(alarmPin, INPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  pwmTimer = std::make_shared<HardwareTimer>(pwm_timer.timer);
  pwmTimer->setPWM(pwm_timer.channel, pwm_timer.pin, 400, 0);

  feedbackTimer = std::make_shared<HardwareTimer>(feedback_timer.timer);
  feedbackTimer->setMode(feedback_timer.channel, TIMER_INPUT_CAPTURE_RISING, feedback_timer.pin);

  feedbackTimer->setPrescaleFactor(PRESCALE_FACTOR);
  feedbackTimer->setOverflow(OVERFLOW_VALUE);
  feedbackTimer->attachInterrupt(feedback_timer.channel, std::bind(&Motor::feedbackInputCallback, this));
  feedbackTimer->attachInterrupt([this]
                                 {rolloverCompareCount++;
                                  if (rolloverCompareCount > 1)
                                    freqMeasured = 0; });

  feedbackTimer->resume();

  timerFreq = feedbackTimer->getTimerClkFreq() / PRESCALE_FACTOR;
  ULOG_DEBUG("Timer freq: %d Hz", timerFreq);
}

void Motor::setDirection(bool direction)
{
  digitalWrite(directionPin, direction);
}

void Motor::setSpeed(int32_t speed)
{
  if (speed < 0)
  {
    setDirection(1);
    speed = -speed;
  }
  else
  {
    setDirection(0);
  }

  setDuty(speed);
}

void Motor::feedbackInputCallback()
{
  currentCapture = feedbackTimer->getCaptureCompare(feedbackChannel);
  /* frequency computation */
  if (currentCapture > lastCapture)
  {
    freqMeasured = timerFreq / (currentCapture - lastCapture);
  }
  else if (currentCapture <= lastCapture)
  {
    // It's an overflow
    freqMeasured = timerFreq / (OVERFLOW_VALUE + currentCapture - lastCapture);
  }
  lastCapture = currentCapture;
  rolloverCompareCount = 0;
}

Motor::~Motor()
{
}
