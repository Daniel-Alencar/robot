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
