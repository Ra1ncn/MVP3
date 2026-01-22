// xiao_car_udp_ledcAttach.ino
// Xiao car: connect to microMVP_AP, receive UDP broadcast (33 bytes),
// parse slot for CAR_ID, drive motors via PHASE/ENABLE using ledcAttach.

#include <WiFi.h>
#include <WiFiUdp.h>

// -------- WiFi --------
static const char* WIFI_SSID = "microMVP_AP";
static const char* WIFI_PASS = "12345678";

// -------- UDP --------
static const uint16_t UDP_PORT = 9001;
WiFiUDP udp;

// -------- Car ID --------
static const int CAR_ID = 1;   // <<< make sure this matches your car
static const int CARS_PER_LEVEL = 10;
static const int CAR_LEVEL = (CAR_ID - 1) / CARS_PER_LEVEL;
static const int CAR_INDEX = CAR_ID - CAR_LEVEL * CARS_PER_LEVEL; // 1..10

// -------- Motor pins (your confirmed mapping) --------
const int APHASE = D3;
const int AENBL  = D2;   // PWM
const int BPHASE = D1;
const int BENBL  = D0;   // PWM

// -------- PWM config --------
static const uint32_t PWM_FREQ = 20000;
static const uint8_t  PWM_RES  = 8;       // bits, duty 0..255

// -------- Calibration --------
static const int LEFT_MAX  = 255;  // scale 0..127 -> 0..LEFT_MAX
static const int RIGHT_MAX = 255;

// -------- Safety --------
static const uint32_t CMD_TIMEOUT_MS = 200;
static uint32_t lastCmdMs = 0;

// -------- Reconnect throttling --------
static uint32_t lastReconnectTryMs = 0;
static const uint32_t RECONNECT_PERIOD_MS = 1500; // 防止疯狂 begin()

// ---------- helpers ----------
static inline int decodeThrust(uint8_t b) {
  int mag = (int)(b & 0x7F);
  int sgn = (b & 0x80) ? -1 : 1;
  return mag * sgn; // -127..127
}

static inline int dutyFromThrust(int thrustAbs, int maxDuty) {
  int d = (thrustAbs * maxDuty) / 127;
  if (d < 0) d = 0;
  if (d > 255) d = 255;
  return d;
}

// -------- motor control --------
void setupMotor() {
  pinMode(APHASE, OUTPUT);
  pinMode(BPHASE, OUTPUT);

  if (!ledcAttach(AENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach AENBL failed");
  }
  if (!ledcAttach(BENBL, PWM_FREQ, PWM_RES)) {
    Serial.println("[CAR] ledcAttach BENBL failed");
  }

  digitalWrite(APHASE, LOW);
  digitalWrite(BPHASE, LOW);

  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

void runMotors(int rightThrust, int leftThrust) {
  digitalWrite(BPHASE, (rightThrust >= 0) ? LOW : HIGH);
  digitalWrite(APHASE, (leftThrust  >= 0) ? LOW : HIGH);

  int rabs = abs(rightThrust);
  int labs = abs(leftThrust);

  int rduty = dutyFromThrust(rabs, RIGHT_MAX);
  int lduty = dutyFromThrust(labs, LEFT_MAX);

  ledcWrite(BENBL, rduty);
  ledcWrite(AENBL, lduty);
}

void stopMotors() {
  ledcWrite(AENBL, 0);
  ledcWrite(BENBL, 0);
}

// -------- WiFi/UDP --------
void setupUDP() {
  udp.begin(UDP_PORT);
  Serial.printf("[CAR] UDP listen :%u\n", UDP_PORT);
}

void connectWiFiBlocking() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  Serial.printf("[CAR] Connecting to WiFi SSID=%s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    if (millis() - t0 > 15000) {
      Serial.println("[CAR] WiFi connect waiting...");
      t0 = millis();
    }
  }
  Serial.print("[CAR] WiFi connected, IP=");
  Serial.println(WiFi.localIP());
}

// ====== Scheme A: event-driven reconnect + UDP rebuild ======
#if defined(ESP32)
static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("[CAR][WiFi] DISCONNECTED, reason=%d\n",
                  (int)info.wifi_sta_disconnected.reason);

    stopMotors();

    // 节流：避免某些情况下疯狂 begin 导致更不稳定
    uint32_t now = millis();
    if (now - lastReconnectTryMs < RECONNECT_PERIOD_MS) return;
    lastReconnectTryMs = now;

    // 更“干净”的重连方式（比只 WiFi.begin 更稳）
    WiFi.disconnect(true, true);
    delay(50);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.printf("[CAR][WiFi] GOT_IP=%s\n", WiFi.localIP().toString().c_str());

    // 关键：重建 UDP 监听（拿到 IP 后做）
    udp.stop();
    delay(10);
    udp.begin(UDP_PORT);
    Serial.printf("[CAR] UDP re-listen :%u\n", UDP_PORT);

    // 防止刚重连就被 timeout 逻辑误判
    lastCmdMs = millis();
  }
}
#endif

// -------- main --------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== Xiao Car UDP Receiver ===");
  Serial.printf("[CAR] CAR_ID=%d CAR_LEVEL=%d CAR_INDEX=%d\n", CAR_ID, CAR_LEVEL, CAR_INDEX);

  setupMotor();

  // register WiFi events (Scheme A)
#if defined(ESP32)
  WiFi.onEvent(onWiFiEvent);
#endif

  connectWiFiBlocking();
  setupUDP();

  lastCmdMs = millis();
}

void loop() {
  // 如果掉线，parsePacket 也不会有意义；让事件回调去负责重连
  if (WiFi.status() == WL_CONNECTED) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      uint8_t buf[64];
      int n = udp.read(buf, sizeof(buf));

      if (n == 33) {
        uint8_t level = buf[0];
        if ((int)level == CAR_LEVEL) {
          const uint8_t* payload = &buf[1];

          int rightPos = 3 * CAR_INDEX;
          int leftPos  = 3 * CAR_INDEX - 1;

          if (rightPos >= 0 && rightPos < 32 && leftPos >= 0 && leftPos < 32) {
            uint8_t rc = payload[rightPos];
            uint8_t lc = payload[leftPos];

            int rightThrust = decodeThrust(rc);
            int leftThrust  = decodeThrust(lc);

            runMotors(rightThrust, leftThrust);
            lastCmdMs = millis();
          }
        }
      }
    }
  }

  // Safety stop if command stream lost
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  delay(1);
}
