/*
 * Lidar bridge: the ESP32 reads the robot vacuum's lidar (TX line, 115200 baud, 3.3 V) and sends
 * the raw bytes over Wi-Fi to the browser, through a WebSocket on port 81. The browser decodes
 * them (src/core/lidar/camsense.ts), exactly as with the USB-serial adapter.
 *
 * Wiring: lidar TX → GPIO16 (RX2), lidar GND → GND. The ESP32 is powered by USB for now and the
 * lidar by the robot's own board (both GNDs must be connected).
 *
 * Build and flash (see README): arduino-cli compile/upload --fqbn esp32:esp32:esp32
 */
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#include "secrets.h"  // WIFI_SSID, WIFI_PASSWORD

constexpr int LIDAR_RX = 16;
constexpr uint32_t LIDAR_BAUD = 115200;
constexpr int LED = 2;  // the blue LED of the DevKit: blinks while lidar data flows
constexpr const char *HOSTNAME = "lidar";  // → ws://lidar.local:81

WebSocketsServer ws(81);

// bytes are grouped into one message every 20 ms (~200 bytes at the lidar's rate)
uint8_t buf[2048];
size_t len = 0;
uint32_t lastSend = 0;
uint32_t bytesSince = 0;
uint32_t lastReport = 0;
uint32_t lastWifiCheck = 0;

void onEvent(uint8_t client, WStype_t type, uint8_t *, size_t) {
  if (type == WStype_CONNECTED) Serial.printf("browser %u connected\n", client);
  if (type == WStype_DISCONNECTED) Serial.printf("browser %u disconnected\n", client);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  // no power saving: it adds up to ~100 ms of latency to every message
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
    digitalWrite(LED, !digitalRead(LED));
  }
  Serial.printf("\nConnected: ws://%s:81  (or ws://%s.local:81)\n", WiFi.localIP().toString().c_str(), HOSTNAME);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  connectWifi();
  MDNS.begin(HOSTNAME);
  ws.begin();
  ws.onEvent(onEvent);
}

void loop() {
  ws.loop();
  while (Serial2.available() && len < sizeof(buf)) buf[len++] = Serial2.read();
  uint32_t now = millis();
  if (len && (now - lastSend >= 20 || len >= sizeof(buf) / 2)) {
    ws.broadcastBIN(buf, len);
    bytesSince += len;
    len = 0;
    lastSend = now;
    digitalWrite(LED, (now / 100) % 2);
  }
  if (now - lastSend > 500) digitalWrite(LED, LOW);
  if (now - lastReport >= 5000) {
    Serial.printf("lidar %u bytes/s · %u browser(s) · Wi-Fi %d dBm · %s\n", bytesSince / 5, ws.connectedClients(), WiFi.RSSI(),
                  WiFi.localIP().toString().c_str());
    bytesSince = 0;
    lastReport = now;
  }
  if (now - lastWifiCheck >= 5000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
