# ESP32-CAM → streaming MJPEG → visualizador Python

O ESP32-CAM sobe um servidor HTTP na rede WiFi e publica os frames da câmera como
MJPEG (`multipart/x-mixed-replace`). O script Python conecta nesse endpoint e exibe
as imagens numa janela OpenCV.

```
ESP32-CAM  --(WiFi)-->  http://<ip>/stream  --(HTTP)-->  viewer.py (OpenCV)
```

## Arquivos

| Arquivo | Função |
| --- | --- |
| `esp32cam_stream/esp32cam_stream.ino` | firmware do ESP32-CAM (Arduino) |
| `viewer.py` | visualizador no PC |

## 1. Firmware

1. Arduino IDE → **Preferências → URLs adicionais**:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   e instale o pacote **esp32** em *Gerenciador de Placas*.
2. Abra `esp32cam_stream/esp32cam_stream.ino` e edite:
   ```cpp
   const char *WIFI_SSID = "SUA_REDE";
   const char *WIFI_PASS = "SUA_SENHA";
   ```
   Use uma rede **2.4 GHz** — o ESP32 não enxerga 5 GHz.
3. Configure em *Tools*:
   - Board: **AI Thinker ESP32-CAM**
   - Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**
   - PSRAM: **Enabled**
4. Gravação (placa sem USB): ligue **IO0 ao GND** antes de energizar para entrar em
   modo bootloader, grave, depois remova o jumper e dê reset.
5. Abra o Serial Monitor a **115200 baud**. Ele imprime o IP:
   ```
   IP: 192.168.0.42
   Stream: http://192.168.0.42/stream
   ```

Endpoints disponíveis:

- `/` — página HTML com o vídeo (teste rápido no navegador)
- `/stream` — MJPEG contínuo (usado pelo Python)
- `/capture` — um único JPEG

## 2. Visualizador no PC

```bash
pip install opencv-python requests numpy
python3 viewer.py 192.168.0.42
```

Outras formas:

```bash
python3 viewer.py http://192.168.0.42/stream     # URL completa
python3 viewer.py 192.168.0.42 --scale 1.5       # janela maior
python3 viewer.py 192.168.0.42 --save-dir ./caps # pasta dos snapshots
python3 viewer.py 192.168.0.42 --headless        # só estatísticas, sem janela
```

Teclas: **q** ou **ESC** sai, **s** salva o frame atual em JPEG.

Se o stream cair (WiFi instável, reset da placa), o script reconecta sozinho a cada
2 segundos.

## Ajustes de qualidade / taxa de quadros

Em `initCamera()`:

```cpp
config.frame_size   = FRAMESIZE_VGA;  // QVGA 320x240 · VGA 640x480 · SVGA 800x600 · UXGA 1600x1200
config.jpeg_quality = 12;             // 0 = melhor imagem (mais bytes) ... 63 = pior
```

Regra prática: **menor resolução + `jpeg_quality` maior = mais fps**. VGA com
qualidade 12 costuma dar ~20–25 fps numa rede boa; UXGA cai para poucos fps.

Imagem invertida? Troque em `initCamera()`:

```cpp
s->set_vflip(s, 1);    // espelha na vertical
s->set_hmirror(s, 1);  // espelha na horizontal
```

LED branco da placa (GPIO 4) como iluminação — em `setup()`:
`digitalWrite(LED_FLASH_GPIO, HIGH);`

## Problemas comuns

| Sintoma | Causa provável |
| --- | --- |
| `Falha ao iniciar camera: 0x105` | cabo flat mal encaixado ou alimentação fraca |
| Reinícios (brownout) durante o stream | fonte 5 V fraca — use ≥ 2 A; evite alimentar pelo pino 3V3 |
| Conecta ao WiFi mas o stream travado | `WiFi.setSleep(false)` é obrigatório (já está no código) |
| Python não conecta | PC e ESP32 na mesma rede/VLAN; teste o IP no navegador primeiro |
| Stream com ~1 fps | sinal WiFi fraco ou resolução alta demais |
| Só um cliente de cada vez | o handler é bloqueante; feche o navegador ao usar o Python |

---

# Parte 2 — Navegação autônoma do robô diferencial

O PC faz a percepção e o planejamento; o ESP32-CAM publica o vídeo e executa as
velocidades que recebe. Nenhuma inteligência fica embarcada.

```
ESP32-CAM ──MJPEG :80──▶ perception.py ──▶ planner.py ──┐
    ▲                                                   │
    └──────────── UDP :3333  "V <esq> <dir>" ◀───────────┘
    │
    └──▶ motors.cpp ──▶ driver de motor ──▶ rodas
```

| Arquivo | Função |
| --- | --- |
| `esp32cam_stream/motors.{h,cpp}` | PWM dos dois motores, zona morta, watchdog |
| `esp32cam_stream/esp32cam_stream.ino` | câmera + stream + receptor UDP |
| `mjpeg.py` | leitor do stream MJPEG (compartilhado) |
| `perception.py` | detecção de espaço livre → varredura de distâncias |
| `planner.py` | varredura → velocidades das rodas |
| `link.py` | canal UDP de controle |
| `navigator.py` | laço principal, HUD, armamento, segurança |

## Ligação do driver de motor

Só sobram 4 GPIOs utilizáveis depois da câmera. O esquema usa **PWM
sinal-magnitude**: duas entradas de direção por motor, sem pino de enable.

| Sinal | GPIO | L298N | TB6612 | DRV8833 / MX1508 |
| --- | --- | --- | --- | --- |
| Esquerda A | 12 | IN1 | AIN1 | IN1 |
| Esquerda B | 13 | IN2 | AIN2 | IN2 |
| Direita A | 15 | IN3 | BIN1 | IN3 |
| Direita B | 14 | IN4 | BIN2 | IN4 |

- **L298N:** mantenha os jumpers de ENA e ENB fechados (enable fixo em 5 V).
- **TB6612:** ligue PWMA e PWMB em 3V3 e STBY em 3V3.
- GND do driver e GND do ESP32 **têm de ser comuns**.

⚠️ **GPIO 12 é strapping pin (MTDI).** Se estiver em nível alto no boot, o ESP32
seleciona tensão de flash errada e não inicializa. Coloque um resistor de **10 kΩ
entre GPIO 12 e GND**. Se mesmo assim não bootar com o driver ligado, troque
`MOTOR_L_IN1` para GPIO 2 em `motors.h`.

⚠️ **Nunca alimente os motores pela saída 5 V do ESP32-CAM.** Use uma fonte
separada (ou bateria) para o driver; o pico de partida dos motores derruba o
regulador e reinicia a placa no meio da navegação.

Pinos proibidos: GPIO 0 (XCLK), 16 (PSRAM), 1/3 (serial), e todos os da câmera.

## Montagem da câmera

A percepção assume plano de chão, então a pose da câmera entra na conta:

- **Altura:** meça do chão até a lente (`--cam-height`, em metros).
- **Inclinação:** aponte **10 a 15° para baixo** (`--cam-tilt`). Esse valor é
  crítico. Com a lente a 12 cm, 20° de inclinação fazem a imagem inteira cobrir
  apenas 0,12 a 0,5 m de chão — perto demais para desviar em tempo. Com 12°, a
  mesma imagem cobre de ~0,16 m a mais de 2 m.
- O piso deve ocupar aproximadamente os dois terços inferiores da imagem.

## Calibração em três passos

**1. Distância.** Ponha uma caixa a 50 cm exatos à frente do robô e rode:

```bash
python3 navigator.py 10.0.0.200 --dry-run
```

Leia `frente` no HUD. Se marcar menos que o real, diminua `--cam-tilt`; se marcar
mais, aumente. Ajuste 1–2° por vez. `--cam-height` desloca a escala inteira.

**2. Zona morta dos motores.** Em `motors.h`, suba `MOTOR_MIN_DUTY` (começa em
60) até as rodas girarem com o menor comando sem travar. Teste manualmente:

```bash
python3 -c "
from link import MotorLink
l = MotorLink('10.0.0.200'); print(l.ping())
print(l.set_speeds(0.25, 0.25)); import time; time.sleep(1); print(l.stop())"
```

Se uma roda girar ao contrário, use `MOTOR_L_INVERT` / `MOTOR_R_INVERT`.

**3. Limiar de travamento.** Observe `diff` no HUD com o robô parado e com ele
andando. Ponha `--stuck-diff` entre os dois valores (padrão 2.0).

## Rodando

```bash
# 1. Sempre primeiro: observe sem acionar nada
python3 navigator.py 10.0.0.200 --dry-run

# 2. Com o robô suspenso no ar, rodas livres, confira os sentidos
python3 navigator.py 10.0.0.200 --max-speed 0.25

# 3. No chão, devagar
python3 navigator.py 10.0.0.200 --max-speed 0.35
```

Teclas: **ESPAÇO** arma/desarma (começa **desarmado**), **b** força recuo,
**q** sai parando os motores.

## Camadas de segurança

1. O robô começa desarmado; nada se move sem ESPAÇO.
2. Watchdog no firmware: sem comando válido por 400 ms, os motores param.
3. Thread de envio a 10 Hz, desacoplada da taxa de quadros — o watchdog não
   dispara por engasgo do vídeo.
4. Frame velho (>0,5 s) faz a thread mandar parada em vez do último comando.
5. Queda do stream ou do WiFi desarma e para.
6. `finally` garante parada em exceção, Ctrl+C ou fim do programa.
7. Percepção com baixa confiança é tratada como obstáculo.

Mesmo assim: **mantenha a mão no cabo da bateria nos primeiros testes.**

## Ajuste de comportamento

Em `planner.py`, `PlannerConfig`:

| Parâmetro | Efeito |
| --- | --- |
| `v_cruise` | velocidade em reta |
| `stop_dist` | a que distância para e gira (0,28 m) |
| `clear_dist` | folga para voltar a andar; histerese contra oscilação |
| `turn_penalty` | quanto o desvio do centro é penalizado ao escolher direção |
| `slew_per_s` | suavidade da aceleração; menor = menos derrapagem |
| `min_confidence` | abaixo disso a percepção é considerada cega |

Em `perception.py`, `FreeSpaceDetector`: `chroma_k` e `luma_k` (tolerância de cor
e brilho do piso), `edge_thresh` (sensibilidade a bordas), `run_length` (pixels
seguidos para confirmar obstáculo).

Piso estampado gerando falsos obstáculos? Suba `chroma_k` e `edge_thresh`.
Obstáculos sendo ignorados? Baixe os dois.

## O que este sistema não faz

Isto é um navegador **reativo local**: ele desvia do que vê agora. Honestamente:

- **Não há mapa nem destino.** O robô vagueia evitando obstáculos. Não sabe onde
  esteve e pode circular na mesma área. Navegação com objetivo exige SLAM.
- **Sem odometria.** Nenhum encoder, então as velocidades são malha aberta: o
  comando 0,4 não corresponde a uma velocidade conhecida em m/s.
- **Distâncias aproximadas**, dependentes da calibração e da hipótese de chão
  plano. Desnível, rampa ou tapete grosso quebram a conta.
- **Câmera monocular não mede profundidade de verdade.** Tudo vem da hipótese de
  plano de chão.
- **Cegueiras da percepção:** vidro, obstáculos da cor do piso, objetos mais
  baixos que a borda inferior do campo de visão, buracos e degraus para baixo
  (um degrau parece piso). Piso estampado e sombras duras geram falso positivo.
- **Parede lisa de frente** é o pior caso: a faixa de referência passa a ser a
  própria parede e tudo parece livre. Só o detector de travamento pega isso,
  depois de já ter encostado.
- **Latência.** Com compressão JPEG em software no GC2145 o laço roda a poucos
  Hz; a ~6 fps e 0,2 m/s o robô anda ~3 cm entre decisões. Ande devagar.

Se precisar de confiabilidade de verdade, o próximo passo não é melhorar a
visão: é somar **dois ou três sensores ultrassônicos (HC-SR04) ou um VL53L0X**
na frente e usar a câmera como complemento. Um sensor de 15 reais resolve
diretamente os casos em que a câmera monocular é cega por princípio.

## Alternativa: profundidade por rede neural

Se quiser substituir a heurística de piso por estimativa de profundidade
monocular aprendida (MiDaS, Depth Anything), o ponto de troca é
`FreeSpaceDetector.process()`: basta produzir o mesmo objeto `Scan`. Roda no PC,
onde há GPU/CPU sobrando. Dá distâncias relativas melhores em cena complexa, mas
continua sem escala métrica absoluta — e o custo é latência, que num laço de
controle a 6 Hz já é o recurso mais escasso.
