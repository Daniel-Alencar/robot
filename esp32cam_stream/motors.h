/*
 * Acionamento de dois motores DC (robo diferencial) no ESP32-CAM.
 *
 * Esquema: PWM sinal-magnitude nas duas entradas de direcao de cada canal.
 * Nao usa pino de enable, entao gasta apenas 2 GPIOs por motor -- o que e
 * tudo que o ESP32-CAM tem de sobra depois da camera.
 *
 * Compativel com L298N (jumpers ENA/ENB fechados), TB6612 (PWMA/PWMB em 3V3),
 * DRV8833, MX1508 e L9110S.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// --------------------------------------------------------------- pinos
// GPIOs 12-15 sao os do cartao SD (nao usado aqui). NAO use GPIO 0 (XCLK da
// camera) nem GPIO 16 (chip select da PSRAM).
//
// ATENCAO: GPIO 12 e strapping pin (MTDI). Se estiver em nivel alto durante o
// boot, o ESP32 tenta tensao de flash errada e nao inicializa. Coloque um
// resistor de 10k entre GPIO 12 e GND. Se a placa ainda assim nao bootar com o
// driver conectado, troque MOTOR_L_IN1 para GPIO 2.
#define MOTOR_L_IN1   12
#define MOTOR_L_IN2   13
#define MOTOR_R_IN1   15
#define MOTOR_R_IN2   14

// --------------------------------------------------------------- PWM
// O driver da camera usa LEDC_TIMER_0 / LEDC_CHANNEL_0 para o XCLK, por isso os
// motores ficam em canais altos e em outro timer.
#define MOTOR_PWM_FREQ   20000   // 20 kHz: acima da faixa audivel
#define MOTOR_PWM_BITS   8       // duty 0..255
#define MOTOR_CH_L_IN1   4
#define MOTOR_CH_L_IN2   5
#define MOTOR_CH_R_IN1   6
#define MOTOR_CH_R_IN2   7

// Motor pequeno nao sai do lugar com duty baixo. |velocidade| em (0,1] e
// mapeado para [MOTOR_MIN_DUTY, MOTOR_MAX_DUTY] para compensar a zona morta.
// Ajuste MIN subindo ate o robo comecar a girar com o menor comando.
#define MOTOR_MIN_DUTY   60
#define MOTOR_MAX_DUTY   255

// Inverta se uma roda girar ao contrario do esperado (alternativa a trocar os
// fios no driver).
#define MOTOR_L_INVERT   false
#define MOTOR_R_INVERT   false

// Sem comando novo dentro desse prazo, os motores param. E a protecao contra
// o Python travar, fechar ou o WiFi cair com o robo em movimento.
#define MOTOR_WATCHDOG_MS 400

void motorsBegin();

// Velocidades normalizadas em [-1, 1]. Positivo = para frente.
void motorsSetSpeeds(float left, float right);

// Corta o acionamento (roda livre) e zera o comando corrente.
void motorsStop();

// Chame com frequencia no loop(): para os motores se o comando expirou.
// Retorna true no instante em que o watchdog dispara.
bool motorsWatchdogTick();

float motorsLastLeft();
float motorsLastRight();

#endif  // MOTORS_H
