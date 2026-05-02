#include <Arduino.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

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
float room_temp = 25.0;
float room_hum = 80.0;

// Pin Relay (26, 25, 33, 32) - TIPE ACTIVE LOW
const int RELAY_PINS[4] = {26, 25, 33, 32};
bool relayState[4] = {false, false, false, false};

// =========================================================
// 2. KONFIGURASI JARINGAN, MQTT & FIREBASE
// =========================================================
const char *mqtt_server = "broker.mqtt.cool";
const int mqtt_port = 1883;
const char *mqtt_prefix = "bms_panel/2602165";

WiFiClient espClient;
PubSubClient mqtt(espClient);
WiFiManager wm;

// --- KONFIGURASI FIREBASE ---
#define FIREBASE_URL "bmsv1-f5b30-default-rtdb.asia-southeast1.firebasedatabase.app"
#define API_KEY "AIzaSyBQampK8P14r7gqOH9NDjuE9pAOE9WpB24"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

// =========================================================
// 3. PARAMETER MODEL BATERAI (Cubic Spline 21 Titik)
// =========================================================
const float Q_AH = 20.798555;
const float Q_COULOMB = Q_AH * 3600.0;

// Diperbarui menjadi 21 titik (interval 5%) dari Cubic Spline Ground Truth
const int LUT_OCV_SIZE = 21;
const float lut_soc_ocv[LUT_OCV_SIZE] = {
    0.00, 0.05, 0.10, 0.15, 0.20,
    0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.65, 0.70,
    0.75, 0.80, 0.85, 0.90, 0.95, 1.00};

// Nilai OCV disesuaikan agar selalu memiliki gradien positif (monotonik naik)
// Sangat penting agar dOCV/dSOC (Matriks H) tidak pernah bernilai nol
const float lut_ocv[LUT_OCV_SIZE] = {
    2.655, 3.050, 3.194, 3.210, 3.220,
    3.232, 3.245, 3.258, 3.270, 3.282,
    3.285, 3.287, 3.288, 3.289, 3.291,
    3.294, 3.300, 3.310, 3.331, 3.385, 3.537};

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
const float Q_NOISE_00 = 1e-7; // Sangat percaya pada Coulomb Counting
const float Q_NOISE_11 = 1e-5; // Toleransi untuk dinamika polarisasi ECM
const float R_NOISE = 5e-2;    // Abaikan lonjakan tegangan di area flat LiFePO4

// =========================================================
// 5. VARIABEL GLOBAL IPC, STATE ESTIMATION, & UI
// =========================================================
volatile bool portalActive = false;
volatile bool requestPortalOpen = false;
volatile bool requestPortalClose = false;
volatile bool ota_updating = false;
volatile int ota_progress_percent = 0;

// === FLAG UNTUK MENJAGA STABILITAS MQTT BUFFER ===
volatile bool flag_run_ekf = false;
volatile bool flag_publish_relay = false;
volatile bool flag_send_firebase = false;
volatile bool flag_soc_critical_sent = false;

// Variabel Global untuk Menampilkan Performa di Layar
volatile float perf_ekf_time_ms = 0.0;
volatile float perf_ram_used_pct = 0.0;
volatile uint32_t perf_ram_used_bytes = 0;
volatile uint32_t perf_ram_total_bytes = 0;

volatile int currentPage = 1;
const int MAX_PAGES = 7;

struct BMS_Data
{
  float voltage = 0.0;
  float current = 0.0;
  float power = 0.0;
  float mos_temp = 0.0;
  float bat_temp1 = 0.0;
  float bat_temp2 = 0.0;
  float cells_v[8] = {0};
  float wire_res[8] = {0};
  float avg_cell_v = 0.0;
  float max_cell_v = 0.0;
  float min_cell_v = 0.0;
  float delta_v = 0.0;
} bmsData;

// --- SETTING INISIALISASI SOC AWAL ---
// Ubah nilai ini sesuai tujuan pengujian skripsi Anda:
// 0.0000 -> Skenario 1 (Charging) jika ingin CC & EKF mulai benar dari 0%
// 1.0000 -> Skenario 2 (Discharge) jika mulai dari baterai penuh 100%
// 0.9414 -> Skenario 3 Atau Tes Uji Konvergensi (Salah tebak awal)(Urban Load)
float soc_cc = 0.9414;
float ekf_x[2] = {0.9414, 0.0};
//--------------------------------------------------
float ekf_P[2][2] = {{0.01, 0.0}, {0.0, 0.01}};
float v_pred_last = 0.0;
float dt_last = 0.0;

unsigned long last_mqtt_time = 0;
unsigned long last_firebase_time = 0;
const unsigned long FIREBASE_INTERVAL = 60000; // Delay Firebase 60 Detik

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
  // KALKULASI UTILISASI MEMORI DAN CPU
  // ==============================================================
  uint32_t ram_total = ESP.getHeapSize();
  uint32_t ram_used = ram_total - ESP.getFreeHeap();
  float ram_pct = (ram_used / (float)ram_total) * 100.0;

  uint32_t flash_used = ESP.getSketchSize();
  uint32_t flash_total = flash_used + ESP.getFreeSketchSpace();
  float flash_pct = (flash_used / (float)flash_total) * 100.0;

  // Beban prosesor dihitung dari eksekusi EKF sebelumnya dibagi cycle time (1 detik = 1000 ms)
  float cpu_load = (perf_ekf_time_ms / 1000.0) * 100.0;

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
  Serial.println("├──────────────────────────────────────────────┤");
  Serial.println("│   [ UTILISASI MEMORI & BEBAN PROSESOR ]      │");
  Serial.printf("│ - Waktu EKF (Loop sblmnya): %.3f ms\n", perf_ekf_time_ms);
  Serial.printf("│ - Beban CPU (Load)        : %.4f %%\n", cpu_load);
  Serial.printf("│ - SRAM (Heap) Used        : %.2f %% (%u B)\n", ram_pct, ram_used);
  Serial.printf("│ - Flash Memory Used       : %.2f %% (%u B)\n", flash_pct, flash_used);
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
  mqtt.publish(topic.c_str(), buffer, false);
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
    // Kita panggil flag agar dikirim saat buffer jaringan aman
    flag_publish_relay = true;
  }
}

// =========================================================
// 8. MQTT CALLBACK
// =========================================================
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String msg;
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];
  String topicStr = String(topic);

  // --- KONTROL RELAY & PROTEKSI ---
  for (int i = 0; i < 4; i++)
  {
    String cmdTopic = String(mqtt_prefix) + "/switch/relay_" + String(i + 1) + "/command";
    if (topicStr == cmdTopic)
    {
      // Proteksi SOC < 20% HANYA UNTUK RELAY 2, 3, dan 4 (indeks 1, 2, 3)
      if (msg == "ON" && i > 0 && ekf_x[0] < 0.20)
      {
        Serial.println("[PROTECTION] Denied! Battery SOC below 20%. Only Relay 1 is allowed.");
        return;
      }
      setRelay(i, (msg == "ON"));
      return;
    }
  }

  // --- GANTI HALAMAN OLED VIA MQTT ---
  String pageCmdTopic = String(mqtt_prefix) + "/display/page/command";
  if (topicStr == pageCmdTopic)
  {
    if (!portalActive && !ota_updating) // Hanya bisa ganti jika tidak sedang setup/OTA
    {
      if (msg == "NEXT")
      {
        currentPage = currentPage + 1;
        if (currentPage > MAX_PAGES)
          currentPage = 1;
        Serial.printf("\n[MQTT] Move Page -> %d\n", currentPage);
      }
      else
      {
        int p = msg.toInt();
        if (p >= 1 && p <= MAX_PAGES)
        {
          currentPage = p;
          Serial.printf("\n[MQTT] Jump to Page -> %d\n", currentPage);
        }
      }
    }
    else
    {
      Serial.println("\n[MQTT] Change Page rejected (In Setup/OTA Mode)");
    }
    return;
  }

  // --- TERIMA DATA HiL ZKETECH ---
  if (topicStr == String(mqtt_prefix) + "/data/main")
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (!error)
    {
      bmsData.voltage = doc["voltage"] | 0.0;
      bmsData.bat_temp1 = doc["bat_temp1"] | 0.0;
      bmsData.bat_temp2 = doc["bat_temp2"] | 0.0;
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

      // Beri sinyal ke TaskNetwork agar menjalankan EKF
      // setelah proses penerimaan (buffer read) MQTT benar-benar tuntas.
      flag_run_ekf = true;
    }
  }
}

void reconnectMQTT()
{
  String clientId = "espbms-" + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str()))
  {
    Serial.println("\n[MQTT] Terhubung ke Broker!");
    for (int i = 1; i <= 4; i++)
    {
      mqtt.subscribe((String(mqtt_prefix) + "/switch/relay_" + String(i) + "/command").c_str());
    }
    // --- Subscribe ke topik ganti halaman OLED ---
    mqtt.subscribe((String(mqtt_prefix) + "/display/page/command").c_str());
    mqtt.subscribe((String(mqtt_prefix) + "/data/main").c_str());
    flag_publish_relay = true;
  }
  else
  {
    Serial.print("[MQTT] Gagal konek, rc=");
    Serial.println(mqtt.state());
  }
}

// =========================================================
// 9. TUGAS CORE 0: JARINGAN (MQTT & FIREBASE)
// =========================================================
void TaskNetwork(void *pvParameters)
{
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("Re", "12345679"))
  {
    delay(3000);
    ESP.restart();
  }

  // --- KONFIGURASI FIREBASE ---
  config.api_key = API_KEY;
  config.database_url = FIREBASE_URL;

  // ====================================================================================
  // SOLUSI ERROR CONFIGURATION_NOT_FOUND / INVALID_EMAIL
  // Karena Rules Realtime Database di console Anda sudah di-set ke TRUE (Publik),
  // kita menginstruksikan library untuk melakukan Bypass/Test Mode agar tidak
  // perlu melakukan pendaftaran Email/Anonymous authentication yang membuat error.
  // ====================================================================================
  config.signer.test_mode = true;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  signupOK = true; // Langsung di-flag true karena test mode aktif
  Serial.println("[FIREBASE] Mode Publik (Test Mode) Aktif. Mengabaikan Otentikasi.");

  // --- KONFIGURASI ARDUINO OTA ---
  ArduinoOTA.setHostname("esp-bms-ekf");

  ArduinoOTA.onStart([]()
                     { ota_updating = true; ota_progress_percent = 0; });
  ArduinoOTA.onEnd([]()
                   { delay(500); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { ota_progress_percent = (progress / (total / 100)); });
  ArduinoOTA.onError([](ota_error_t error)
                     { ota_updating = false; });

  ArduinoOTA.begin();

  // Membesarkan Buffer dan Waktu Tunggu MQTT agar lebih kebal lag
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);
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

      if (!ota_updating)
      {
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
          // mqtt.loop() bertugas menerima pesan masuk
          mqtt.loop();

          // === EKSEKUSI EKF & PENGUKURAN PERFORMA ===
          if (flag_run_ekf)
          {
            flag_run_ekf = false; // Reset flag

            unsigned long now = millis();
            float dt = 1.0;
            if (!is_first_run && last_mqtt_time > 0)
            {
              // Anda bisa pakai dt asli di sini jika ingin waktu yg akurat
              // dt = (now - last_mqtt_time) / 1000.0;
            }
            is_first_run = false;
            last_mqtt_time = now;
            dt_last = dt;

            // 1. Pengukuran Beban Prosesor (Waktu Eksekusi EKF)
            unsigned long ekf_start_time = micros();
            runEKFStep(bmsData.current, bmsData.avg_cell_v, dt);
            unsigned long ekf_end_time = micros();

            perf_ekf_time_ms = (ekf_end_time - ekf_start_time) / 1000.0;

            // 2. Pengukuran Utilisasi Memori SRAM (Heap)
            perf_ram_total_bytes = ESP.getHeapSize();
            perf_ram_used_bytes = perf_ram_total_bytes - ESP.getFreeHeap();
            perf_ram_used_pct = (perf_ram_used_bytes / (float)perf_ram_total_bytes) * 100.0;

            publishComputedData(dt);

            // LOGIKA FIREBASE: Kirim jika drop < 20% (Emergency Override)
            if (ekf_x[0] < 0.20)
            {
              if (relayState[1] || relayState[2] || relayState[3])
              {
                Serial.println("\n⚠️ [WARNING] SOC < 20%! AUTOMATICALLY CUT OFF LOAD 2, 3, AND 4!");
                setRelay(1, false);
                setRelay(2, false);
                setRelay(3, false);
              }
              if (!flag_soc_critical_sent)
              {
                flag_send_firebase = true;
                flag_soc_critical_sent = true;
              }
            }
            else
            {
              flag_soc_critical_sent = false;
            }
          }

          if (flag_publish_relay)
          {
            flag_publish_relay = false;
            publishRelayState();
          }

          // === LOGIKA DELAY FIREBASE 60 DETIK ===
          if (millis() - last_firebase_time >= FIREBASE_INTERVAL)
          {
            last_firebase_time = millis();
            flag_send_firebase = true;
          }

          // === PROSES KIRIM KE FIREBASE ===
          if (flag_send_firebase && signupOK && Firebase.ready())
          {
            flag_send_firebase = false; // Reset bendera

            FirebaseJson json;
            json.set("voltage", bmsData.voltage);
            json.set("current", bmsData.current);
            json.set("power", bmsData.power);
            json.set("soc_ekf", ekf_x[0] * 100.0);
            json.set("soc_cc", soc_cc * 100.0);
            json.set("avg_cell_v", bmsData.avg_cell_v);
            json.set("max_cell_v", bmsData.max_cell_v);
            json.set("min_cell_v", bmsData.min_cell_v);
            json.set("delta_v", bmsData.delta_v);
            json.set("mos_temp", bmsData.mos_temp);
            json.set("bat_temp1", bmsData.bat_temp1);
            json.set("bat_temp2", bmsData.bat_temp2);
            json.set("room_temp", room_temp);
            json.set("room_hum", room_hum);

            // Menulis ke path /bms_realtime (Akan me-replace data lama, cocok untuk Dashboard)
            if (Firebase.RTDB.setJSON(&fbdo, "/bms_realtime", &json))
            {
              Serial.println("[FIREBASE] Data berhasil dikirim (Interval 60s / Darurat)!");
            }
            else
            {
              Serial.printf("[FIREBASE] Gagal kirim: %s\n", fbdo.errorReason().c_str());
            }
          }
        }
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
    // room_temp = temp.temperature;
    // room_hum = humidity.relative_humidity;
  }
}

// Menggambar Layar saat proses OTA berlangsung
void drawOTAScreen(int percent)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("==== OTA UPDATE ====");
  display.drawLine(0, 10, 128, 10, WHITE);
  display.setCursor(0, 20);
  display.printf("Downloading: %d %%", percent);
  display.drawRect(14, 37, 100, 10, WHITE);
  display.fillRect(14, 37, percent, 10, WHITE);
  display.setCursor(0, 56);
  display.print("Please wait...");
  display.display();
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
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(0, 16);
    display.print("WiFi: esp-setup");
    display.setCursor(0, 26);
    display.print("Pass: 12345679");
    display.setCursor(0, 36);
    display.print("IP  : 192.168.4.1");
    // Data Sensor AHT10
    display.setCursor(0, 46);
    display.printf("Temp: %.1f C", room_temp);
    display.setCursor(0, 56);
    display.printf("Hum : %.1f %%", room_hum);
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
    display.printf(" WiFi [%c] | MQTT [%c]", (WiFi.status() == WL_CONNECTED) ? 'V' : 'X', mqtt.connected() ? 'V' : 'X');
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
    display.printf("Max:%.3f Min:%.3f", bmsData.max_cell_v, bmsData.min_cell_v);
    display.setCursor(0, 36);
    display.printf("Delta(dV): %.3f V", bmsData.delta_v);
    display.setCursor(0, 46);
    display.printf("Temp MOS : %.1f C", bmsData.mos_temp);
    display.setCursor(0, 56);
    display.printf("Bat T1:%.1f T2:%.1f", bmsData.bat_temp1, bmsData.bat_temp2);
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
    display.print("IP   :");
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
  // HALAMAN 7: FORMULASI MASALAH
  else if (currentPage == 7)
  {
    display.setCursor(0, 0);
    display.print("== PERFORMA SISTEM ==");
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(0, 16);
    display.printf("EKF Time: %.3f ms", perf_ekf_time_ms);
    display.setCursor(0, 26);
    display.printf("RAM Used: %.1f %%", perf_ram_used_pct);
    display.setCursor(0, 36);
    display.printf("RAM: %u B", perf_ram_used_bytes);
    display.setCursor(0, 46);
    display.printf("Max: %u B", perf_ram_total_bytes);
    display.setCursor(0, 56);
    display.print(signupOK ? "Firebase: ONLINE" : "Firebase: OFFLINE");
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
    Serial.printf("Temp: %.1f C\n", room_temp);
    Serial.printf("Hum : %.1f %%\n", room_hum);
  }
  else if (currentPage == 1)
  {
    Serial.println("=== MAIN DASHBOARD ===");
    Serial.printf("Volt : %.1fV | %.1fA\n", bmsData.voltage, bmsData.current);
    Serial.printf("SOC  : CC:%.0f%% EKF:%.0f%%\n", soc_cc * 100, ekf_x[0] * 100);
    Serial.printf("Relay1:%s | Relay2:%s\n", relayState[0] ? "ON " : "OFF", relayState[1] ? "ON " : "OFF");
    Serial.printf("Relay3:%s | Relay4:%s\n", relayState[2] ? "ON " : "OFF", relayState[3] ? "ON " : "OFF");
    char wifiSym = (WiFi.status() == WL_CONNECTED) ? 'V' : 'X';
    char mqttSym = mqtt.connected() ? 'V' : 'X';
    Serial.printf("WiFi [%c] | MQTT [%c]\n", wifiSym, mqttSym);
  }
  else if (currentPage == 2)
  {
    Serial.println("== CELL DIAGNOSTIC ==");
    Serial.printf("Avg Cell : %.3f V\n", bmsData.avg_cell_v);
    Serial.printf("Max:%.3f Min:%.3f\n", bmsData.max_cell_v, bmsData.min_cell_v);
    Serial.printf("Delta(dV): %.3f V\n", bmsData.delta_v);
    Serial.printf("Temp MOS : %.1f C\n", bmsData.mos_temp);
    Serial.printf("Bat T1:%.1f C | T2:%.1f C\n", bmsData.bat_temp1, bmsData.bat_temp2);
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
    Serial.printf("WiFi :%s\n", WiFi.status() == WL_CONNECTED ? WiFi.SSID().c_str() : "Disconnected");
    Serial.print("IP   :");
    Serial.println(WiFi.localIP());
    Serial.printf("EKF dt:%.2f sec\n", dt_last);
    Serial.println("OTA  :Ready");
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
  else if (currentPage == 7)
  {
    Serial.println("== PERFORMA SISTEM ==");
    Serial.printf("EKF Time: %.3f ms\n", perf_ekf_time_ms);
    Serial.printf("RAM Used: %.1f %%\n", perf_ram_used_pct);
    Serial.printf("RAM Used: %u B\n", perf_ram_used_bytes);
    Serial.printf("RAM Total: %u B\n", perf_ram_total_bytes);
    Serial.printf("Firebase: %s\n", signupOK ? "ONLINE" : "OFFLINE");
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
      Serial.println("[BUTTON] Long Press Detected -> Toggle WiFi Portal");
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
        // Kunci navigasi halaman jika portal aktif ATAU OTA berjalan
        if (!portalActive && !ota_updating)
        {
          currentPage = currentPage + 1;
          if (currentPage > MAX_PAGES)
            currentPage = 1;
          Serial.printf("\n[BUTTON] Short Press -> Change Page %d\n", currentPage);
          updateLayar();
          printOledToSerial();
        }
        else
        {
          Serial.println("\n[BUTTON] Short Press ignored because it is in Setup/OTA Mode");
        }
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
    Serial.println("OLED Failed!");
    for (;;)
      ;
  }
  if (aht.begin())
    aht_status = true;

  // DINAIIKKAN MENJADI 15000 bytes KARENA FIREBASE BUTUH MEMORI BESAR
  xTaskCreatePinnedToCore(TaskNetwork, "TaskNet", 15000, NULL, 1, NULL, 0);
}

void loop()
{
  static unsigned long waktuTerakhir = 0;
  static int last_drawn_percent = -1;

  // Jika sedang OTA, alihkan fokus UI ke Progress Bar OTA (DIJALANKAN DI CORE 1)
  if (ota_updating)
  {
    if (ota_progress_percent != last_drawn_percent)
    {
      last_drawn_percent = ota_progress_percent;
      drawOTAScreen(ota_progress_percent);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return; // Keluar dari loop agar tidak update halaman lain
  }

  if (millis() - waktuTerakhir >= 500)
  {
    waktuTerakhir = millis();
    bacaSensorAHT();
    updateLayar();
  }

  cekTombolSmart();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}