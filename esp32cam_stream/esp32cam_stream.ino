/*
 * ESP32-CAM -> streaming MJPEG na rede WiFi
 *
 * Endpoints:
 *   http://<ip>/         pagina HTML simples com o video
 *   http://<ip>/stream   stream MJPEG (multipart/x-mixed-replace)  <- usado pelo Python
 *   http://<ip>/capture  um unico frame JPEG
 *
 * Placa: "AI Thinker ESP32-CAM" (Tools > Board > ESP32 Arduino)
 * Partition: "Huge APP (3MB No OTA)"  |  PSRAM: Enabled
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "motors.h"
#include "esp_http_server.h"
#include "esp_timer.h"

// ----------------------------------------------------------------- WiFi
const char *WIFI_SSID = "Ponto de Acesso";
const char *WIFI_PASS = "RO081056";

// ------------------------------------------------- Pinos AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_FLASH_GPIO     4   // LED branco da placa

// ----------------------------------------------------------------- MJPEG
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t server = NULL;

// ----------------------------------------------------------------- camera
// Qualidade usada quando o JPEG tem de ser feito em software (frame2jpg).
static const int SW_JPEG_QUALITY = 80;   // 1..100

static bool g_jpegNative = false;        // sensor comprime em hardware?

static void fillPins(camera_config_t &config) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.grab_mode = CAMERA_GRAB_LATEST;   // sempre o frame mais recente
}

static bool initCamera() {
  const bool psram = psramFound();

  // --- Tentativa 1: JPEG em hardware (OV2640, OV3660, OV5640) ---
  camera_config_t config = {};
  fillPins(config);
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location  = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.frame_size   = psram ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
  config.jpeg_quality = psram ? 12 : 15;   // 0 (melhor) .. 63 (pior)
  config.fb_count     = psram ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);

  if (err == ESP_OK) {
    g_jpegNative = true;
  } else if (err == ESP_ERR_NOT_SUPPORTED) {
    // --- Tentativa 2: sensor sem JPEG (OV7670, GC0308...) -> RGB565 ---
    // O frame cru e convertido para JPEG em software no handler.
    Serial.println("Sensor sem JPEG em hardware; usando RGB565 + compressao em software.");
    esp_camera_deinit();

    camera_config_t raw = {};
    fillPins(raw);
    raw.xclk_freq_hz = 10000000;          // OV7670 nao e estavel a 20 MHz
    raw.pixel_format = PIXFORMAT_RGB565;
    raw.fb_location  = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    raw.frame_size   = psram ? FRAMESIZE_QVGA : FRAMESIZE_QQVGA;  // 320x240 / 160x120
    raw.jpeg_quality = 12;                // ignorado em RGB565
    raw.fb_count     = psram ? 2 : 1;

    err = esp_camera_init(&raw);
    if (err == ESP_OK) g_jpegNative = false;
  }

  if (err != ESP_OK) {
    Serial.printf("Falha ao iniciar camera: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Sensor PID 0x%04x | JPEG em hardware: %s\n",
                s->id.PID, g_jpegNative ? "sim" : "nao");

  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  if (s->id.PID == OV2640_PID) {
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
  }
  return true;
}

// ----------------------------------------------------------------- handlers
static esp_err_t stream_handler(httpd_req_t *req) {
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  int64_t last = esp_timer_get_time();
  uint32_t frames = 0;

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Captura falhou");
      res = ESP_FAIL;
      break;
    }

    size_t len = 0;
    uint8_t *buf = NULL;
    bool converted = false;

    if (fb->format == PIXFORMAT_JPEG) {
      len = fb->len;
      buf = fb->buf;
    } else {
      converted = frame2jpg(fb, SW_JPEG_QUALITY, &buf, &len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!converted) { res = ESP_FAIL; break; }
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      int n = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
      res = httpd_resp_send_chunk(req, part, n);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)buf, len);
    }

    if (fb)             esp_camera_fb_return(fb);
    else if (converted) free(buf);

    if (res != ESP_OK) break;   // cliente desconectou

    if (++frames % 30 == 0) {
      int64_t now = esp_timer_get_time();
      Serial.printf("~%.1f fps\n", 30.0 * 1000000.0 / (now - last));
      last = now;
    }
  }

  return res;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t res;
  if (fb->format == PIXFORMAT_JPEG) {
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  } else {
    uint8_t *buf = NULL;
    size_t len = 0;
    bool ok = frame2jpg(fb, SW_JPEG_QUALITY, &buf, &len);
    esp_camera_fb_return(fb);
    if (!ok) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    res = httpd_resp_send(req, (const char *)buf, len);
    free(buf);
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  static const char page[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<title>ESP32-CAM</title>"
    "<style>body{margin:0;background:#111;display:flex;align-items:center;"
    "justify-content:center;height:100vh}img{max-width:100%;max-height:100%}</style>"
    "</head><body><img src=\"/stream\"></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, strlen(page));
}

static void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port    = 80;
  config.ctrl_port      = 32768;
  config.max_uri_handlers = 4;
  config.stack_size     = 8192;

  httpd_uri_t uri_index   = { "/",        HTTP_GET, index_handler,   NULL };
  httpd_uri_t uri_stream  = { "/stream",  HTTP_GET, stream_handler,  NULL };
  httpd_uri_t uri_capture = { "/capture", HTTP_GET, capture_handler, NULL };

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &uri_index);
    httpd_register_uri_handler(server, &uri_stream);
    httpd_register_uri_handler(server, &uri_capture);
    Serial.println("Servidor HTTP iniciado na porta 80");
  } else {
    Serial.println("Falha ao iniciar servidor HTTP");
  }
}

// ----------------------------------------------------------------- controle UDP
// Protocolo de texto, uma mensagem por datagrama:
//   "V <esq> <dir>"  velocidades em [-1,1]        -> responde "OK <esq> <dir> <rssi>"
//   "S"              parada imediata              -> responde "OK 0 0 <rssi>"
//   "P"              ping/keepalive               -> responde "PONG <uptime_ms> <rssi>"
// UDP e sem conexao: nao compete com o stream HTTP e nao acumula fila quando a
// rede engasga -- comando atrasado e comando descartado, que e o que queremos
// num laco de velocidade.
#define CONTROL_PORT 3333

static WiFiUDP udp;
static char    udpBuf[64];

static void handleControlPackets() {
  int size = udp.parsePacket();
  while (size > 0) {
    int n = udp.read(udpBuf, sizeof(udpBuf) - 1);
    if (n < 0) n = 0;
    udpBuf[n] = '\0';

    char reply[64];
    reply[0] = '\0';

    if (udpBuf[0] == 'V') {
      float l = 0, r = 0;
      if (sscanf(udpBuf + 1, "%f %f", &l, &r) == 2) {
        motorsSetSpeeds(l, r);
        snprintf(reply, sizeof(reply), "OK %.2f %.2f %d",
                 motorsLastLeft(), motorsLastRight(), WiFi.RSSI());
      } else {
        snprintf(reply, sizeof(reply), "ERR formato");
      }
    } else if (udpBuf[0] == 'S') {
      motorsStop();
      snprintf(reply, sizeof(reply), "OK 0.00 0.00 %d", WiFi.RSSI());
    } else if (udpBuf[0] == 'P') {
      snprintf(reply, sizeof(reply), "PONG %lu %d", (unsigned long)millis(), WiFi.RSSI());
    } else {
      snprintf(reply, sizeof(reply), "ERR comando");
    }

    if (reply[0]) {
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t *)reply, strlen(reply));
      udp.endPacket();
    }

    size = udp.parsePacket();   // esvazia a fila: só o último comando importa
  }
}

// ----------------------------------------------------------------- setup/loop
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(200);

  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);   // flash desligado

  // Motores primeiro: garante PWM em zero antes de qualquer outra coisa, para
  // o robo nao arrancar durante a inicializacao da camera/WiFi.
  motorsBegin();

  if (!initCamera()) {
    Serial.println("Reiniciando em 5s...");
    delay(5000);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // essencial para streaming estavel
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Stream: http://%s/stream\n", WiFi.localIP().toString().c_str());

  startServer();

  udp.begin(CONTROL_PORT);
  Serial.printf("Controle UDP escutando na porta %d\n", CONTROL_PORT);
}

void loop() {
  handleControlPackets();

  if (motorsWatchdogTick()) {
    Serial.println("Watchdog: sem comando, motores parados.");
  }

  // Reconecta se o WiFi cair; o servidor HTTP roda em tarefas proprias.
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 2000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      motorsStop();              // nunca siga em movimento sem link
      Serial.println("WiFi caiu, reconectando...");
      WiFi.reconnect();
    }
  }

  delay(2);   // laco em ~500 Hz: latencia baixa para os comandos
}
