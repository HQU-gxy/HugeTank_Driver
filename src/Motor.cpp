#include <ulog.h>
#include "Motor.h"

#include <STM32FreeRTOS.h>

#include "config.h"

void fuckPID(TimerHandle_t shit)
{

  auto motor = static_cast<Motor *>(pvTimerGetTimerID(shit));
  if (!motor)
  {
    ULOG_ERROR("Fuck PID is called without a Motor pointer passed in");
    return;
  }

  // An incredibly shitty PID implementation
  static int32_t lastError = 0;
  static int32_t lastLastError = 0;
  static int32_t lastOutput = 0;

  int32_t currentError = motor->targetFreq - motor->freqMeasured;
  int32_t output = lastOutput + motor->PID_KP * (currentError - lastError) + motor->PID_KI * currentError + motor->PID_KD * (currentError - 2 * lastError + lastLastError);

  if (output < motor->MIN_DUTY)
    output = motor->MIN_DUTY;
  else if (output > motor->MAX_DUTY)
    output = motor->MAX_DUTY;

  if (motor->targetFreq == 0)
    output = 0;
  motor->setDuty(output);

  lastOutput = output;
  lastError = currentError;
  lastLastError = lastError;

#ifdef PID_TUNING
  struct __attribute__((packed))
  {
    uint8_t header = 0x69;
    uint32_t freq;
    int32_t error;
    int32_t output;
    char shit[4] = {'s', 'h', 'i', 't'};
  } pidStatus{
      .freq = motor->freqMeasured,
      .error = currentError,
      .output = output,
  };
  Serial2.write(reinterpret_cast<char *>(&pidStatus), sizeof(pidStatus));
#endif
}

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
  auto PIDTimer = xTimerCreate("PID", pdMS_TO_TICKS(PID_PERIOD), pdTRUE, static_cast<void *>(this), fuckPID);
  xTimerStart(PIDTimer, 0);
  ULOG_DEBUG("PID Timer started");
}

void Motor::setSpeed(float speed)
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

  targetFreq = speed * SPEED_SCALE;
  if (targetFreq < MIN_TARGET_FREQ)
  {
    targetFreq = 0;
  }
}

void Motor::feedbackInputCallback()
{
  currentCapture = feedbackTimer->getCaptureCompare(feedbackChannel);
  freqMeasured = timerFreq / (OVERFLOW_VALUE * rolloverCompareCount + currentCapture - lastCapture);

  lastCapture = currentCapture;
  rolloverCompareCount = 0;
}

Motor::~Motor()
{
}
