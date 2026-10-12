#include "motors.h"

static float    g_left = 0.0f, g_right = 0.0f;
static uint32_t g_lastCmdMs = 0;
static bool     g_stopped = true;

// ---------------------------------------------------- compatibilidade de core
// A API do LEDC mudou entre arduino-esp32 2.x e 3.x.
static void pwmSetup(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, channel);
#else
  ledcSetup(channel, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

static void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// ---------------------------------------------------------------- helpers
static uint32_t dutyFromSpeed(float magnitude) {
  if (magnitude <= 0.001f) return 0;
  if (magnitude > 1.0f) magnitude = 1.0f;
  // Mapeia (0,1] -> [MIN_DUTY, MAX_DUTY], pulando a zona morta do motor.
  return (uint32_t)(MOTOR_MIN_DUTY + magnitude * (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY));
}

static void driveChannel(uint8_t pin1, uint8_t ch1, uint8_t pin2, uint8_t ch2,
                         float speed, bool invert) {
  if (invert) speed = -speed;

  uint32_t duty = dutyFromSpeed(fabsf(speed));
  if (duty == 0) {                 // roda livre
    pwmWrite(pin1, ch1, 0);
    pwmWrite(pin2, ch2, 0);
  } else if (speed > 0) {          // frente
    pwmWrite(pin1, ch1, duty);
    pwmWrite(pin2, ch2, 0);
  } else {                         // tras
    pwmWrite(pin1, ch1, 0);
    pwmWrite(pin2, ch2, duty);
  }
}

// ---------------------------------------------------------------- API
void motorsBegin() {
  pwmSetup(MOTOR_L_IN1, MOTOR_CH_L_IN1);
  pwmSetup(MOTOR_L_IN2, MOTOR_CH_L_IN2);
  pwmSetup(MOTOR_R_IN1, MOTOR_CH_R_IN1);
  pwmSetup(MOTOR_R_IN2, MOTOR_CH_R_IN2);
  motorsStop();
}

void motorsSetSpeeds(float left, float right) {
  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  driveChannel(MOTOR_L_IN1, MOTOR_CH_L_IN1, MOTOR_L_IN2, MOTOR_CH_L_IN2,
               left, MOTOR_L_INVERT);
  driveChannel(MOTOR_R_IN1, MOTOR_CH_R_IN1, MOTOR_R_IN2, MOTOR_CH_R_IN2,
               right, MOTOR_R_INVERT);

  g_left = left;
  g_right = right;
  g_lastCmdMs = millis();
  g_stopped = false;
}

void motorsStop() {
  pwmWrite(MOTOR_L_IN1, MOTOR_CH_L_IN1, 0);
  pwmWrite(MOTOR_L_IN2, MOTOR_CH_L_IN2, 0);
  pwmWrite(MOTOR_R_IN1, MOTOR_CH_R_IN1, 0);
  pwmWrite(MOTOR_R_IN2, MOTOR_CH_R_IN2, 0);
  g_left = g_right = 0.0f;
  g_stopped = true;
}

bool motorsWatchdogTick() {
  if (g_stopped) return false;
  if (millis() - g_lastCmdMs > MOTOR_WATCHDOG_MS) {
    motorsStop();
    return true;
  }
  return false;
}

float motorsLastLeft()  { return g_left; }
float motorsLastRight() { return g_right; }
