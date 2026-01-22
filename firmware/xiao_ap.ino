// xiao_ap_bridge.ino
// Xiao_AP: USB Serial -> WiFi SoftAP -> UDP broadcast raw bytes
// Frame in from PC (Serial):
//   0xAA 0x55 | level(1) | len(1=32) | payload(32) | checksum(1)
// checksum = XOR(level, len, payload...)
// UDP out (broadcast):
//   [level(1)] + payload(32)  => 33 bytes

#include <WiFi.h>
#include <WiFiUdp.h>

// --------------------
// WiFi SoftAP settings
// --------------------
static const char* AP_SSID = "microMVP_AP";
static const char* AP_PASS = "12345678";   // >= 8 chars

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);
IPAddress UDP_BROADCAST(192, 168, 4, 255);

// --------------------
// UDP settings
// --------------------
static const uint16_t UDP_PORT = 9001;  // AP -> Cars
WiFiUDP udp;

// --------------------
// Serial settings
// --------------------
static const uint32_t SERIAL_BAUD = 115200;

// --------------------
// Frame format
// --------------------
static const uint8_t HDR0 = 0xAA;
static const uint8_t HDR1 = 0x55;
static const uint8_t PAYLOAD_LEN_EXPECTED = 32;

// Parser state machine
enum RxState {
  WAIT_HDR0,
  WAIT_HDR1,
  WAIT_LEVEL,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CKSUM
};

static RxState state = WAIT_HDR0;

static uint8_t rx_level = 0;
static uint8_t rx_len = 0;
static uint8_t rx_payload[PAYLOAD_LEN_EXPECTED];
static uint8_t rx_idx = 0;
static uint8_t rx_cksum = 0;

static inline uint8_t crc8_xor_update(uint8_t c, uint8_t b) {
  return (uint8_t)(c ^ b);
}

void setupSoftAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[AP] softAP start: ");
  Serial.println(ok ? "OK" : "FAIL");

  Serial.print("[AP] IP: ");
  Serial.println(WiFi.softAPIP());
}

void setupUDP() {
  // Note: for sending broadcast only, begin() is optional,
  // but it's fine to begin on UDP_PORT (and can help debugging).
  udp.begin(UDP_PORT);
  Serial.print("[UDP] broadcast port: ");
  Serial.println(UDP_PORT);
}

void udpBroadcastLevelPayload(uint8_t level, const uint8_t* payload32) {
  uint8_t out[1 + PAYLOAD_LEN_EXPECTED];
  out[0] = level;
  memcpy(out + 1, payload32, PAYLOAD_LEN_EXPECTED);

  udp.beginPacket(UDP_BROADCAST, UDP_PORT);
  udp.write(out, sizeof(out));
  udp.endPacket();
}

// Feed one byte into the serial frame parser.
// Returns true if a complete valid frame was assembled and broadcasted.
bool feedByte(uint8_t b) {
  switch (state) {
    case WAIT_HDR0:
      if (b == HDR0) state = WAIT_HDR1;
      break;

    case WAIT_HDR1:
      if (b == HDR1) {
        state = WAIT_LEVEL;
      } else {
        state = WAIT_HDR0;
      }
      break;

    case WAIT_LEVEL:
      rx_level = b;
      rx_cksum = 0;
      rx_cksum = crc8_xor_update(rx_cksum, rx_level);
      state = WAIT_LEN;
      break;

    case WAIT_LEN:
      rx_len = b;
      rx_cksum = crc8_xor_update(rx_cksum, rx_len);

      // We only accept fixed 32B payloads (matches your comm.py)
      if (rx_len != PAYLOAD_LEN_EXPECTED) {
        state = WAIT_HDR0;
        break;
      }
      rx_idx = 0;
      state = WAIT_PAYLOAD;
      break;

    case WAIT_PAYLOAD:
      rx_payload[rx_idx++] = b;
      rx_cksum = crc8_xor_update(rx_cksum, b);
      if (rx_idx >= rx_len) {
        state = WAIT_CKSUM;
      }
      break;

    case WAIT_CKSUM:
      // b is checksum byte from PC
      if (b == rx_cksum) {
        // valid frame -> UDP broadcast
        udpBroadcastLevelPayload(rx_level, rx_payload);
        // (optional) debug:
        // Serial.printf("[OK] level=%u len=%u\n", rx_level, rx_len);
        state = WAIT_HDR0;
        return true;
      } else {
        // checksum fail -> resync
        // Serial.println("[ERR] checksum");
        state = WAIT_HDR0;
      }
      break;
  }
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("=== Xiao_AP USB->UDP Bridge ===");
  setupSoftAP();
  setupUDP();

  Serial.println("[INFO] Expect framed serial: AA 55 level len(32) payload(32) cksum(xor)");
  Serial.println("[INFO] UDP broadcast: [level(1)] + payload(32) to 192.168.4.255:9001");
}

void loop() {
  // Read serial bytes and parse frames
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    feedByte(b);
  }

  // Optional: small yield
  delay(1);
}
