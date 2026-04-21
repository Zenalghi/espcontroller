#include <Arduino.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =========================================================
// 1. KONFIGURASI HARDWARE (OLED, SENSOR, RELAY, TOMBOL)
// =========================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Tombol (BOOT Button bawaan ESP32 DevKit = GPIO 0)
#define BUTTON_PIN 0

Adafruit_AHTX0 aht;
bool aht_status = false;
float room_temp = 0.0;
float room_hum = 0.0;

// Pin Relay (26, 25, 33, 32) - TIPE ACTIVE LOW
const int RELAY_PINS[4] = {26, 25, 33, 32};
bool relayState[4] = {false, false, false, false};

// =========================================================
// 2. KONFIGURASI JARINGAN & MQTT
// =========================================================
const char *mqtt_server = "broker.mqtt.cool";
const int mqtt_port = 1883;
const char *mqtt_prefix = "bms_panel/260216";

WiFiClient espClient;
PubSubClient mqtt(espClient);
WiFiManager wm;

// =========================================================
// 3. PARAMETER MODEL BATERAI (EKSTRAKSI DARI CSV FILE)
// =========================================================
const float Q_AH = 20.798555;
const float Q_COULOMB = Q_AH * 3600.0;

const int LUT_OCV_SIZE = 10;
const float lut_soc_ocv[LUT_OCV_SIZE] = {
    0.0, 0.098849, 0.212431, 0.326001, 0.439511,
    0.553234, 0.667017, 0.780667, 0.894377, 1.0};
const float lut_ocv[LUT_OCV_SIZE] = {
    2.655, 3.194, 3.223, 3.253, 3.282,
    3.288, 3.289, 3.297, 3.326, 3.537};

const int LUT_ECM_SIZE = 9;
const float lut_soc_ecm[LUT_ECM_SIZE] = {
    0.0, 0.090902, 0.204618, 0.318054, 0.431697,
    0.545421, 0.659070, 0.772787, 0.886430};
const float lut_r0[LUT_ECM_SIZE] = {
    0.006050, 0.002800, 0.002800, 0.002899, 0.002700,
    0.002400, 0.002899, 0.002199, 0.002800};
const float lut_r1[LUT_ECM_SIZE] = {
    0.009500, 0.002506, 0.002207, 0.002212, 0.002372,
    0.002436, 0.002374, 0.002345, 0.002684};
const float lut_c1[LUT_ECM_SIZE] = {
    11281.15, 20591.86, 24841.48, 15061.40, 20897.75,
    19607.70, 15177.97, 16580.74, 24189.08};

// =========================================================
// 4. TUNING NOISE PARAMETER
// =========================================================
const float Q_NOISE_00 = 1e-5;
const float Q_NOISE_11 = 1e-4;
const float R_NOISE = 1e-4;

// =========================================================
// 5. VARIABEL GLOBAL IPC, STATE ESTIMATION, & UI
// =========================================================
volatile bool portalActive = false;
volatile bool requestPortalOpen = false;
volatile bool requestPortalClose = false;

volatile int currentPage = 1;
const int MAX_PAGES = 6;

struct BMS_Data
{
  float voltage = 0.0;
  float current = 0.0;
  float power = 0.0;
  float bat_temp1 = 0.0;
  float mos_temp = 0.0;
  float cells_v[8] = {0};
  float wire_res[8] = {0};
  float avg_cell_v = 0.0;
  float max_cell_v = 0.0;
  float min_cell_v = 0.0;
  float delta_v = 0.0;
} bmsData;

float soc_cc = 0.9414;
float ekf_x[2] = {0.9414, 0.0};
float ekf_P[2][2] = {{0.01, 0.0}, {0.0, 0.01}};
float v_pred_last = 0.0;
float dt_last = 0.0;

unsigned long last_mqtt_time = 0;
bool is_first_run = true;
bool lastWiFiState = false;

// =========================================================
// 6. FUNGSI MATEMATIKA: INTERPOLASI & EKF
// =========================================================
float interpolate1D(float x, const float *x_data, const float *y_data, int size)
{
  if (x <= x_data[0])
    return y_data[0];
  if (x >= x_data[size - 1])
    return y_data[size - 1];
  for (int i = 0; i < size - 1; i++)
  {
    if (x >= x_data[i] && x <= x_data[i + 1])
    {
      float t = (x - x_data[i]) / (x_data[i + 1] - x_data[i]);
      return y_data[i] + t * (y_data[i + 1] - y_data[i]);
    }
  }
  return y_data[0];
}

float get_dOCV_dSOC(float soc)
{
  float delta = 0.001;
  float s_high = min(soc + delta, 1.0f);
  float s_low = max(soc - delta, 0.0f);
  float ocv_high = interpolate1D(s_high, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float ocv_low = interpolate1D(s_low, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float derivative = (ocv_high - ocv_low) / (s_high - s_low);
  return max(derivative, 0.000001f);
}
// =========================================================
// ENGINE EKF DENGAN VERBOSE SERIAL DEBUGGING
// =========================================================
void runEKFStep(float I_meas, float V_meas, float dt)
{
  // 1. Integrasi Coulomb Counting
  soc_cc = constrain(soc_cc - (I_meas * dt / Q_COULOMB), 0.0, 1.0);

  float soc_prev = constrain(ekf_x[0], 0.0, 1.0);
  float vc1_prev = ekf_x[1];

  // 2. Evaluasi Parameter Dinamis ECM
  float R0 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r0, LUT_ECM_SIZE), 0.0001f);
  float R1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r1, LUT_ECM_SIZE), 0.0001f);
  float C1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_c1, LUT_ECM_SIZE), 1.0f);
  float tau = max(R1 * C1, 0.000001f);

  // TAHAP PREDIKSI (A PRIORI)
  float soc_pred = constrain(soc_prev - (I_meas * dt / Q_COULOMB), 0.0, 1.0);
  float alpha = (dt > 0) ? exp(-dt / tau) : 1.0f;
  float vc1_pred = (alpha * vc1_prev) + (R1 * (1.0f - alpha) * I_meas);

  ekf_x[0] = soc_pred;
  ekf_x[1] = vc1_pred;

  // Prediksi Matriks Kovariansi P_pred
  float P_pred[2][2];
  P_pred[0][0] = ekf_P[0][0] + Q_NOISE_00;
  P_pred[0][1] = ekf_P[0][1] * alpha;
  P_pred[1][0] = ekf_P[1][0] * alpha;
  P_pred[1][1] = (alpha * alpha * ekf_P[1][1]) + Q_NOISE_11;

  // TAHAP UPDATE (A POSTERIORI)
  float OCV_pred = interpolate1D(soc_pred, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float dOCV_dSOC = get_dOCV_dSOC(soc_pred);

  float V_pred = OCV_pred - vc1_pred - (I_meas * R0);
  v_pred_last = V_pred;

  // Matriks Jacobian (H)
  float h0 = dOCV_dSOC;
  float h1 = -1.0f;

  // Inovasi & Kalman Gain (K)
  float S = (h0 * h0 * P_pred[0][0]) + (h0 * h1 * P_pred[0][1]) +
            (h1 * h0 * P_pred[1][0]) + (h1 * h1 * P_pred[1][1]) + R_NOISE;

  float K[2];
  K[0] = ((P_pred[0][0] * h0) + (P_pred[0][1] * h1)) / S;
  K[1] = ((P_pred[1][0] * h0) + (P_pred[1][1] * h1)) / S;

  // Koreksi State
  float error = V_meas - V_pred;
  ekf_x[0] = constrain(ekf_x[0] + (K[0] * error), 0.0, 1.0);
  ekf_x[1] = ekf_x[1] + (K[1] * error);

  // Joseph Form Update
  float I_KH[2][2];
  I_KH[0][0] = 1.0f - (K[0] * h0);
  I_KH[0][1] = -(K[0] * h1);
  I_KH[1][0] = -(K[1] * h0);
  I_KH[1][1] = 1.0f - (K[1] * h1);

  float Temp[2][2];
  Temp[0][0] = I_KH[0][0] * P_pred[0][0] + I_KH[0][1] * P_pred[1][0];
  Temp[0][1] = I_KH[0][0] * P_pred[0][1] + I_KH[0][1] * P_pred[1][1];
  Temp[1][0] = I_KH[1][0] * P_pred[0][0] + I_KH[1][1] * P_pred[1][0];
  Temp[1][1] = I_KH[1][0] * P_pred[0][1] + I_KH[1][1] * P_pred[1][1];

  ekf_P[0][0] = Temp[0][0] * I_KH[0][0] + Temp[0][1] * I_KH[0][1] + (K[0] * K[0] * R_NOISE);
  ekf_P[0][1] = Temp[0][0] * I_KH[1][0] + Temp[0][1] * I_KH[1][1] + (K[0] * K[1] * R_NOISE);
  ekf_P[1][0] = Temp[1][0] * I_KH[0][0] + Temp[1][1] * I_KH[0][1] + (K[1] * K[0] * R_NOISE);
  ekf_P[1][1] = Temp[1][0] * I_KH[1][0] + Temp[1][1] * I_KH[1][1] + (K[1] * K[1] * R_NOISE);

  // ==============================================================
  // CETAK KALKULASI EKF KE SERIAL MONITOR UNTUK DEMONSTRASI
  // ==============================================================
  Serial.println("\n┌──────────────────────────────────────────────┐");
  Serial.println("│          MATRIKS KALKULASI EKF AKTIF         │");
  Serial.println("├──────────────────────────────────────────────┤");
  Serial.printf("│ 1. Input Sensor : V_meas = %.3f V | I = %.2f A\n", V_meas, I_meas);
  Serial.printf("│ 2. Open-Loop CC : SOC_CC = %.2f %%\n", soc_cc * 100);
  Serial.printf("│ 3. State Pred   : V_pred = %.3f V | SOC = %.2f %%\n", V_pred, soc_pred * 100);
  Serial.printf("│ 4. Error (Inov) : V_meas - V_pred = %+.4f V\n", error);
  Serial.printf("│ 5. Kalman Gain  : K[0] = %.5f | K[1] = %.5f\n", K[0], K[1]);
  Serial.printf("│ 6. Output Final : SOC_EKF Terkoreksi = %.2f %%\n", ekf_x[0] * 100);
  Serial.println("└──────────────────────────────────────────────┘");
}

// =========================================================
// 7. FUNGSI MQTT PUBLISH (STATE & CALCULATION)
// =========================================================
void publishRelayState()
{
  if (!mqtt.connected())
    return;
  JsonDocument doc;
  doc["relay_1"] = relayState[0] ? "ON" : "OFF";
  doc["relay_2"] = relayState[1] ? "ON" : "OFF";
  doc["relay_3"] = relayState[2] ? "ON" : "OFF";
  doc["relay_4"] = relayState[3] ? "ON" : "OFF";

  char buffer[200];
  serializeJson(doc, buffer);
  String topic = String(mqtt_prefix) + "/state/relays";
  mqtt.publish(topic.c_str(), buffer, true);
}

void publishComputedData(float dt)
{
  if (!mqtt.connected())
    return;
  JsonDocument doc;
  doc["soc_cc"] = soc_cc * 100.0;
  doc["soc_ekf"] = ekf_x[0] * 100.0;
  doc["v_pred"] = v_pred_last;
  doc["avg_cell_v"] = bmsData.avg_cell_v;
  doc["dt"] = dt;

  char buffer[200];
  serializeJson(doc, buffer);
  String topic = String(mqtt_prefix) + "/data/calc";
  mqtt.publish(topic.c_str(), buffer, false);
}

void setRelay(int index, bool state)
{
  if (relayState[index] != state)
  {
    relayState[index] = state;
    digitalWrite(RELAY_PINS[index], state ? LOW : HIGH);
    publishRelayState();
  }
}

// =========================================================
// 8. MQTT CALLBACK (PARSING -> MATH -> PUBLISH -> PROTECT)
// =========================================================
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String msg;
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];
  String topicStr = String(topic);

  for (int i = 0; i < 4; i++)
  {
    String cmdTopic = String(mqtt_prefix) + "/switch/relay_" + String(i + 1) + "/command";
    if (topicStr == cmdTopic)
    {
      if (msg == "ON" && i < 3 && ekf_x[0] < 0.20)
      {
        Serial.println("[PROTEKSI] Ditolak! SOC Baterai di bawah 20%");
        return;
      }
      setRelay(i, (msg == "ON"));
      return;
    }
  }

  if (topicStr == String(mqtt_prefix) + "/data/main")
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (!error)
    {
      bmsData.voltage = doc["voltage"] | 0.0;
      bmsData.bat_temp1 = doc["bat_temp1"] | 0.0;
      bmsData.mos_temp = doc["mos_temp"] | 0.0;
      bmsData.power = doc["power"] | 0.0;
      bmsData.current = doc["current"] | 0.0;

      float sum_cells = 0;
      float max_v = 0.0;
      float min_v = 5.0;

      JsonArray cells = doc["cells_v"];
      JsonArray wire_res = doc["wire_res"];

      for (int i = 0; i < 8; i++)
      {
        float cv = cells[i] | 0.0;
        bmsData.cells_v[i] = cv;
        bmsData.wire_res[i] = wire_res[i] | 0.0;

        sum_cells += cv;
        if (cv > max_v)
          max_v = cv;
        if (cv < min_v && cv > 0.1)
          min_v = cv;
      }

      bmsData.avg_cell_v = sum_cells / 8.0;
      bmsData.max_cell_v = max_v;
      bmsData.min_cell_v = min_v;
      bmsData.delta_v = max_v - min_v;

      unsigned long now = millis();
      float dt = 0.0;
      if (is_first_run)
      {
        dt = 1.0;
        is_first_run = false;
      }
      else
      {
        dt = 1.0;
      }
      last_mqtt_time = now;
      dt_last = dt;

      runEKFStep(bmsData.current, bmsData.avg_cell_v, dt);
      publishComputedData(dt);

      if (ekf_x[0] < 0.20)
      {
        if (relayState[0] || relayState[1] || relayState[2])
        {
          Serial.println("\n⚠️ [WARNING] SOC < 20%! MEMUTUS BEBAN SECARA OTOMATIS!");
          setRelay(1, false);
          setRelay(2, false);
          setRelay(3, false);
        }
      }
    }
  }
}

void reconnectMQTT()
{
  String clientId = "espbms-" + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str()))
  {
    for (int i = 1; i <= 4; i++)
    {
      mqtt.subscribe((String(mqtt_prefix) + "/switch/relay_" + String(i) + "/command").c_str());
    }
    mqtt.subscribe((String(mqtt_prefix) + "/data/main").c_str());
    publishRelayState();
  }
}

// =========================================================
// 9. TUGAS CORE 0: JARINGAN & MQTT (BACKGROUND)
// =========================================================
void TaskNetwork(void *pvParameters)
{
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("Re", "12345679"))
  {
    delay(3000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("esp-bms-ekf");
  ArduinoOTA.begin();

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  unsigned long lastMqttReconnectAttempt = 0;

  for (;;)
  {
    bool currentWiFiState = (WiFi.status() == WL_CONNECTED);
    if (requestPortalOpen)
    {
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal("esp-setup", "12345679");
      requestPortalOpen = false;
      portalActive = true;
    }
    if (requestPortalClose)
    {
      wm.stopConfigPortal();
      requestPortalClose = false;
      portalActive = false;
    }

    if (portalActive)
    {
      wm.process();
    }
    else if (currentWiFiState)
    {
      ArduinoOTA.handle();
      if (!mqtt.connected())
      {
        if (millis() - lastMqttReconnectAttempt > 5000)
        {
          lastMqttReconnectAttempt = millis();
          reconnectMQTT();
        }
      }
      else
      {
        mqtt.loop();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =========================================================
// 10. TUGAS CORE 1: LAYAR MULTI-PAGE OLED 128x64
// =========================================================
void bacaSensorAHT()
{
  if (aht_status)
  {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    room_temp = temp.temperature;
    room_hum = humidity.relative_humidity;
  }
}

void updateLayar()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (portalActive)
  {
    display.setCursor(0, 0);
    display.print("==== SETUP MODE ====");
    display.drawLine(0, 10, 128, 10, WHITE); // Garis bawah judul

    display.setCursor(0, 16);
    display.print("WiFi: esp-setup");
    display.setCursor(0, 26);
    display.print("Pass: 12345679");
    display.setCursor(0, 36);
    display.print("IP  : 192.168.4.1");
  }
  else if (currentPage == 1)
  {
    // Halaman 1: Main Dashboard
    display.setCursor(0, 0);
    display.print("=== MAIN DASHBOARD ===");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf("Volt : %.1fV | %.1fA", bmsData.voltage, bmsData.current);
    display.setCursor(0, 26);
    display.printf("SOC  : CC:%.0f%% EKF:%.0f%%", soc_cc * 100, ekf_x[0] * 100);
    display.setCursor(0, 36);
    display.printf(" Relay1:%-2s| Relay2:%s", relayState[0] ? "ON" : "X", relayState[1] ? "ON" : "X");
    display.setCursor(0, 46);
    display.printf(" Relay3:%-2s| Relay4:%s", relayState[2] ? "ON" : "X", relayState[3] ? "ON" : "X");
    display.setCursor(0, 56);
    char wifiSym = (WiFi.status() == WL_CONNECTED) ? 'V' : 'X';
    char mqttSym = mqtt.connected() ? 'V' : 'X';
    display.printf(" WiFi [%c] | MQTT [%c]", wifiSym, mqttSym);
  }
  else if (currentPage == 2)
  {
    // Halaman 2: Cell Diagnostics
    display.setCursor(0, 0);
    display.print("== CELL DIAGNOSTIC ==");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf("Avg Cell : %.3f V", bmsData.avg_cell_v);
    display.setCursor(0, 26);
    display.printf("Max Cell : %.3f V", bmsData.max_cell_v);
    display.setCursor(0, 36);
    display.printf("Min Cell : %.3f V", bmsData.min_cell_v);
    display.setCursor(0, 46);
    display.printf("Delta(dV): %.3f V", bmsData.delta_v);
    display.setCursor(0, 56);
    display.printf("Temp : MOS:%.1f B:%.1f", bmsData.mos_temp, bmsData.bat_temp1);
  }
  else if (currentPage == 3)
  {
    // Halaman 3: Environment & Power
    display.setCursor(0, 0);
    display.print("==== ENV & POWER ====");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf("Power  : %.0f W", bmsData.power);
    display.setCursor(0, 26);
    display.printf("Room T : %.1f C", room_temp);
    display.setCursor(0, 36);
    display.printf("Room H : %.1f %%", room_hum);
    display.setCursor(0, 46);
    display.print(aht_status ? "AHT10  : OK" : "AHT10  : ERROR");
  }
  else if (currentPage == 4)
  {
    // Halaman 4: Network & System
    display.setCursor(0, 0);
    display.print("=== NETWORK & SYS ===");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf("WiFi : %s", WiFi.status() == WL_CONNECTED ? WiFi.SSID().c_str() : "Disconnected");
    display.setCursor(0, 26);
    display.print("IP   : ");
    display.print(WiFi.localIP());
    display.setCursor(0, 36);
    display.printf("EKF dt: %.2f sec", dt_last);
    display.setCursor(0, 46);
    display.print("OTA  : Ready");
  }
  else if (currentPage == 5)
  {
    // Halaman 5: Voltase 8 Sel Baterai
    display.setCursor(0, 0);
    display.print("=== CELL VOLTAGES ===");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf(" V1:%.3f   V2:%.3f", bmsData.cells_v[0], bmsData.cells_v[1]);
    display.setCursor(0, 26);
    display.printf(" V3:%.3f   V4:%.3f", bmsData.cells_v[2], bmsData.cells_v[3]);
    display.setCursor(0, 36);
    display.printf(" V5:%.3f   V6:%.3f", bmsData.cells_v[4], bmsData.cells_v[5]);
    display.setCursor(0, 46);
    display.printf(" V7:%.3f   V8:%.3f", bmsData.cells_v[6], bmsData.cells_v[7]);
  }
  else if (currentPage == 6)
  {
    // Halaman 6: Resistansi (Wire Res) 8 Sel Baterai
    display.setCursor(0, 0);
    display.print("=== WIRE RESISTOR ===");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setCursor(0, 16);
    display.printf(" R1:%.3f   R2:%.3f", bmsData.wire_res[0], bmsData.wire_res[1]);
    display.setCursor(0, 26);
    display.printf(" R3:%.3f   R4:%.3f", bmsData.wire_res[2], bmsData.wire_res[3]);
    display.setCursor(0, 36);
    display.printf(" R5:%.3f   R6:%.3f", bmsData.wire_res[4], bmsData.wire_res[5]);
    display.setCursor(0, 46);
    display.printf(" R7:%.3f   R8:%.3f", bmsData.wire_res[6], bmsData.wire_res[7]);
  }

  display.display();
}

// Cetak isi OLED ke Serial Monitor
void printOledToSerial()
{
  Serial.println("\n----------------------------------------");
  if (portalActive)
  {
    Serial.println("==== SETUP MODE ====");
    Serial.println("WiFi: esp-setup");
    Serial.println("Pass: 12345679");
    Serial.println("IP  : 192.168.4.1");
  }
  else if (currentPage == 1)
  {
    Serial.println("=== MAIN DASHBOARD ===");
    Serial.printf("Volt : %.1fV | %.1fA\n", bmsData.voltage, bmsData.current);
    Serial.printf("SOC  : CC:%.0f%% EKF:%.0f%%\n", soc_cc * 100, ekf_x[0] * 100);

    // Relay dibagi menjadi 2 baris untuk Serial Monitor
    Serial.printf("Relay1:%s | Relay2:%s\n", relayState[0] ? "ON " : "OFF", relayState[1] ? "ON " : "OFF");
    Serial.printf("Relay3:%s | Relay4:%s\n", relayState[2] ? "ON " : "OFF", relayState[3] ? "ON " : "OFF");

    // WiFi & MQTT digabung dalam 1 baris
    char wifiSym = (WiFi.status() == WL_CONNECTED) ? 'V' : 'X';
    char mqttSym = mqtt.connected() ? 'V' : 'X';
    Serial.printf("WiFi [%c] | MQTT [%c]\n", wifiSym, mqttSym);
  }
  else if (currentPage == 2)
  {
    Serial.println("== CELL DIAGNOSTIC ==");
    Serial.printf("Avg Cell : %.3f V\n", bmsData.avg_cell_v);
    Serial.printf("Max Cell : %.3f V\n", bmsData.max_cell_v);
    Serial.printf("Min Cell : %.3f V\n", bmsData.min_cell_v);
    Serial.printf("Delta(dV): %.3f V\n", bmsData.delta_v);
    Serial.printf("Temp : MOS:%.1f B:%.1f\n", bmsData.mos_temp, bmsData.bat_temp1);
  }
  else if (currentPage == 3)
  {
    Serial.println("==== ENV & POWER ====");
    Serial.printf("Power  : %.0f W\n", bmsData.power);
    Serial.printf("Room T : %.1f C\n", room_temp);
    Serial.printf("Room H : %.1f %%\n", room_hum);
    Serial.printf("AHT10  : %s\n", aht_status ? "OK" : "ERROR");
  }
  else if (currentPage == 4)
  {
    Serial.println("=== NETWORK & SYS ===");
    Serial.printf("WiFi : %s\n", WiFi.status() == WL_CONNECTED ? WiFi.SSID().c_str() : "Disconnected");
    Serial.print("IP   : ");
    Serial.println(WiFi.localIP());
    Serial.printf("EKF dt: %.2f sec\n", dt_last);
    Serial.println("OTA  : Ready");
  }
  else if (currentPage == 5)
  {
    Serial.println("=== CELL VOLTAGES ===");
    Serial.printf("V1:%.3f   V2:%.3f\n", bmsData.cells_v[0], bmsData.cells_v[1]);
    Serial.printf("V3:%.3f   V4:%.3f\n", bmsData.cells_v[2], bmsData.cells_v[3]);
    Serial.printf("V5:%.3f   V6:%.3f\n", bmsData.cells_v[4], bmsData.cells_v[5]);
    Serial.printf("V7:%.3f   V8:%.3f\n", bmsData.cells_v[6], bmsData.cells_v[7]);
  }
  else if (currentPage == 6)
  {
    Serial.println("=== WIRE RESISTOR ===");
    Serial.printf("R1:%.3f   R2:%.3f\n", bmsData.wire_res[0], bmsData.wire_res[1]);
    Serial.printf("R3:%.3f   R4:%.3f\n", bmsData.wire_res[2], bmsData.wire_res[3]);
    Serial.printf("R5:%.3f   R6:%.3f\n", bmsData.wire_res[4], bmsData.wire_res[5]);
    Serial.printf("R7:%.3f   R8:%.3f\n", bmsData.wire_res[6], bmsData.wire_res[7]);
  }
  Serial.println("----------------------------------------");
}

void cekTombolSmart()
{
  static unsigned long waktuTekan = 0;
  static bool sedangDitekan = false;
  static bool longPressTriggered = false;

  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (!sedangDitekan)
    {
      waktuTekan = millis();
      sedangDitekan = true;
      longPressTriggered = false;
    }
    else if ((millis() - waktuTekan > 5000) && !longPressTriggered)
    {
      longPressTriggered = true;
      Serial.println("[TOMBOL] Long Press Terdeteksi -> Toggle WiFi Portal");
      if (!portalActive)
        requestPortalOpen = true;
      else
        requestPortalClose = true;
    }
  }
  else
  {
    if (sedangDitekan)
    {
      if (!longPressTriggered && (millis() - waktuTekan < 1000))
      {
        currentPage++;
        if (currentPage > MAX_PAGES)
          currentPage = 1;
        Serial.printf("\n[TOMBOL] Short Press -> Pindah Halaman %d\n", currentPage);
        updateLayar();
        printOledToSerial();
      }
      sedangDitekan = false;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < 4; i++)
  {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED Gagal!");
    for (;;)
      ;
  }
  if (aht.begin())
    aht_status = true;

  xTaskCreatePinnedToCore(TaskNetwork, "TaskNet", 10000, NULL, 1, NULL, 0);
}

void loop()
{
  static unsigned long waktuTerakhir = 0;
  if (millis() - waktuTerakhir >= 500)
  {
    waktuTerakhir = millis();
    bacaSensorAHT();
    updateLayar();
  }

  cekTombolSmart();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}