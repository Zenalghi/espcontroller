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
// 1. KONFIGURASI HARDWARE
// =========================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define BUTTON_PIN 0

Adafruit_AHTX0 aht;
bool aht_status = false;

// Pin Relay (26, 25, 33, 32) - TIPE ACTIVE HIGH
const int RELAY_PINS[4] = {26, 25, 33, 32};
bool relayState[4] = {false, false, false, false}; // false = OFF, true = ON

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
// 3. VARIABEL GLOBAL (IPC & EKF DATA)
// =========================================================
volatile bool portalActive = false;
volatile bool requestPortalOpen = false;
volatile bool requestPortalClose = false;

// Wadah Penampung Data Jikong (Persiapan Rumus EKF Skripsi)
struct BMS_EKF_Data
{
  float voltage = 0.0;
  float current = 0.0;
  float bat_temp1 = 0.0;
  float cells_v[8] = {0};
} bmsData;

// Pelacak status koneksi untuk Serial Monitor
bool lastWiFiState = false;
bool lastMQTTState = false;

// =========================================================
// 4. FUNGSI LAYAR OLED
// =========================================================
String teksBerjalan(String teks, int batasKarakter = 21)
{
  if (teks.length() <= batasKarakter)
    return teks;
  int kecepatan = 300;
  int indeks = (millis() / kecepatan) % (teks.length() + 3);
  String teksGabung = teks + "   " + teks;
  return teksGabung.substring(indeks, indeks + batasKarakter);
}

void updateLayar(String baris1, String baris2 = "", String baris3 = "", String baris4 = "")
{
  display.clearDisplay();

  String ahtStr = "";
  if (aht_status)
  {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    ahtStr = String((int)temp.temperature) + "C|" + String((int)humidity.relative_humidity) + "%";
  }
  else
  {
    ahtStr = "[X] AHT";
  }

  int16_t x_pos = SCREEN_WIDTH - (ahtStr.length() * 6);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setTextWrap(false);

  display.setCursor(0, 0);
  display.print(baris1);

  display.setCursor(x_pos, 0);
  display.print(ahtStr);

  if (baris2 != "")
  {
    display.setCursor(0, 8);
    display.print(baris2);
  }
  if (baris3 != "")
  {
    display.setCursor(0, 16);
    display.print(baris3);
  }
  if (baris4 != "")
  {
    display.setCursor(0, 24);
    display.print(baris4);
  }

  display.display();
}

void pantauSistem()
{
  static unsigned long waktuTerakhir = 0;
  const long jedaUpdate = 300;

  if (millis() - waktuTerakhir >= jedaUpdate)
  {
    waktuTerakhir = millis();

    if (portalActive)
    {
      updateLayar("Portal Active!", "WiFi: esp-setup", "Pass: 12345679", "IP  : 192.168.4.1");
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
      String ipStr = WiFi.localIP().toString();
      String teksWiFi = teksBerjalan("WiFi: " + WiFi.SSID());
      String teksMQTT = mqtt.connected() ? "MQTT: Connected" : "MQTT: Connecting...";

      updateLayar("Ready!", teksWiFi, teksMQTT, "IP  : " + ipStr);
    }
    else
    {
      updateLayar("Disconnected!", "Waiting for WiFi...", teksBerjalan("Searching..."), "System Active");
    }
  }
}

// =========================================================
// 5. FUNGSI KONTROL RELAY & MQTT
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

  char buffer[256];
  serializeJson(doc, buffer);

  String topic = String(mqtt_prefix) + "/state/relays";
  mqtt.publish(topic.c_str(), buffer, true); // Retained message

  Serial.println("\n[MQTT] Status Relay berhasil di-publish:");
  Serial.println(buffer);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String msg;
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];
  String topicStr = String(topic);

  // A. MENANGKAP PERINTAH RELAY (4 Channel)
  for (int i = 0; i < 4; i++)
  {
    String cmdTopic = String(mqtt_prefix) + "/switch/relay_" + String(i + 1) + "/command";
    if (topicStr == cmdTopic)
    {
      Serial.print("\n[MQTT] Perintah Relay Diterima: ");
      Serial.println(msg);

      if (msg == "ON")
      {
        relayState[i] = true;
        Serial.printf("[RELAY] Hardware Relay %d dihidupkan (Aktif HIGH)\n", i + 1);
      }
      else if (msg == "OFF")
      {
        relayState[i] = false;
        Serial.printf("[RELAY] Hardware Relay %d dimatikan (LOW)\n", i + 1);
      }

      // Eksekusi ke Hardware (Active HIGH: true = HIGH, false = LOW)
      digitalWrite(RELAY_PINS[i], relayState[i] ? HIGH : LOW);

      // Update status ke MQTT
      publishRelayState();
      return;
    }
  }

  // B. MENANGKAP DATA JIKONG UNTUK EKF
  if (topicStr == String(mqtt_prefix) + "/data/main")
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (!error)
    {
      bmsData.voltage = doc["voltage"] | 0.0;
      bmsData.current = doc["current"] | 0.0;
      bmsData.bat_temp1 = doc["bat_temp1"] | 0.0;

      JsonArray cells = doc["cells_v"];
      for (int i = 0; i < 8; i++)
      {
        bmsData.cells_v[i] = cells[i] | 0.0;
      }

      // Print ini hanya untuk memastikan data masuk, bisa dikomen nanti kalau dirasa spam
      Serial.println("\n[BMS DATA] Sinkronisasi Memory EKF Sukses:");
      Serial.printf(" -> Volt : %.2f V | Amp: %.2f A | Suhu: %.1f C\n", bmsData.voltage, bmsData.current, bmsData.bat_temp1);
    }
    else
    {
      Serial.print("\n[BMS DATA] Gagal parsing JSON EKF: ");
      Serial.println(error.c_str());
    }
  }
}

void reconnectMQTT()
{
  String clientId = "espmain-" + String(random(0xffff), HEX);
  Serial.print("[MQTT] Mencoba connect ke broker... ");

  if (mqtt.connect(clientId.c_str()))
  {
    Serial.println("BERHASIL!");

    // Subscribe Topik Relay
    for (int i = 1; i <= 4; i++)
    {
      String topic = String(mqtt_prefix) + "/switch/relay_" + String(i) + "/command";
      mqtt.subscribe(topic.c_str());
      Serial.println("[MQTT] Subscribe: " + topic);
    }

    // Subscribe Topik Data BMS (Untuk EKF)
    String dataTopic = String(mqtt_prefix) + "/data/main";
    mqtt.subscribe(dataTopic.c_str());
    Serial.println("[MQTT] Subscribe: " + dataTopic);

    // Kirim status awal relay agar dashboard sinkron
    publishRelayState();
  }
  else
  {
    Serial.print("GAGAL, state=");
    Serial.print(mqtt.state());
    Serial.println(" -> Coba lagi nanti.");
  }
}

// =========================================================
// 6. TUGAS CORE 0: NETWORK & BACKGROUND PROCESS
// =========================================================
void TaskNetwork(void *pvParameters)
{
  Serial.println("[CORE 0] Task Network dimulai.");
  wm.setConfigPortalTimeout(180);

  // Memblokir Core 0 sampai WiFi connect (TIDAK MEMENGARUHI CORE 1 / OLED)
  Serial.println("[WIFI] Memulai WiFiManager... (Menunggu koneksi dari router)");
  bool res = wm.autoConnect("esp-setup", "12345679");
  if (!res)
  {
    Serial.println("[WIFI] Timeout! Gagal terhubung ke WiFi. ESP akan Restart...");
    delay(3000);
    ESP.restart();
  }

  // Konfigurasi OTA & MQTT setelah WiFi terhubung
  ArduinoOTA.setHostname("esp-yoyo");
  ArduinoOTA.begin();
  Serial.println("[OTA] Service OTA jarak jauh (esp-yoyo) siap digunakan.");

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  unsigned long lastMqttReconnectAttempt = 0;

  // Loop abadi khusus Core 0
  for (;;)
  {
    // Tracker Status WiFi
    bool currentWiFiState = (WiFi.status() == WL_CONNECTED);
    if (currentWiFiState != lastWiFiState)
    {
      if (currentWiFiState)
      {
        Serial.print("\n[WIFI] Terhubung! IP Address: ");
        Serial.println(WiFi.localIP());
      }
      else
      {
        Serial.println("\n[WIFI] Terputus dari jaringan! Mencoba menyambung kembali...");
      }
      lastWiFiState = currentWiFiState;
    }

    // Cek Perintah Buka/Tutup Portal dari Tombol Fisik
    if (requestPortalOpen)
    {
      Serial.println("\n[WIFI] Mode AP Aktif. Membuka Portal WiFiManager (esp-setup)...");
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal("esp-setup", "12345679");
      requestPortalOpen = false;
      portalActive = true;
    }
    if (requestPortalClose)
    {
      Serial.println("\n[WIFI] Menutup Portal WiFiManager secara paksa.");
      wm.stopConfigPortal();
      requestPortalClose = false;
      portalActive = false;
    }

    // Eksekusi Network Routine
    if (portalActive)
    {
      wm.process(); // Melayani web server Captive Portal
    }
    else
    {
      if (currentWiFiState)
      {
        ArduinoOTA.handle();

        // Logic non-blocking untuk MQTT reconnect
        if (!mqtt.connected())
        {
          if (lastMQTTState)
          {
            Serial.println("\n[MQTT] Terputus dari broker MQTT!");
            lastMQTTState = false;
          }
          if (millis() - lastMqttReconnectAttempt > 5000)
          {
            lastMqttReconnectAttempt = millis();
            reconnectMQTT();
          }
        }
        else
        {
          if (!lastMQTTState)
          {
            lastMQTTState = true;
          }
          mqtt.loop();
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Wajib agar Core 0 tidak crash/watchdog error
  }
}

// =========================================================
// 7. CORE 1: HARDWARE INIT & MAIN LOOP
// =========================================================
void cekTombolReset()
{
  static unsigned long waktuTekan = 0;
  static bool sedangDitekan = false;

  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (!sedangDitekan)
    {
      waktuTekan = millis();
      sedangDitekan = true;
      Serial.println("[TOMBOL] Tombol ditekan, mulai menghitung mundur 5 detik...");
    }
    else if (millis() - waktuTekan > 5000)
    {
      Serial.println("[TOMBOL] 5 Detik terlampaui! Mengirim instruksi ke Core 0.");
      if (!portalActive)
        requestPortalOpen = true;
      else
        requestPortalClose = true;

      // Mengunci eksekusi sampai tombol fisik dilepas
      while (digitalRead(BUTTON_PIN) == LOW)
      {
        vTaskDelay(10);
      }
      Serial.println("[TOMBOL] Dilepas. Lanjut proses.");
      sedangDitekan = false;
    }
  }
  else
  {
    if (sedangDitekan)
    {
      Serial.println("[TOMBOL] Dibatalkan (dilepas sebelum 5 detik).");
    }
    sedangDitekan = false;
  }
}
void setup()
{
  Serial.begin(115200);
  delay(1000); // Beri jeda Serial Terminal agar tulisan awal tidak terpotong

  Serial.println("\n\n========================================");
  Serial.println("[SYSTEM] BOOTING ESPMAIN (Mode Dual Core FreeRTOS)");
  Serial.println("========================================");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // INIT RELAY (ACTIVE HIGH PROTECTION)
  Serial.println("[RELAY] Menyiapkan perlindungan pin (Active HIGH)...");
  for (int i = 0; i < 4; i++)
  {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW); // Paksa pin jadi 0V (OFF) sebelum jadi OUTPUT
  }

  // =========================================================
  // FIX I2C: KUNCI JALUR AGAR OLED & AHT10 TIDAK BERTABRAKAN
  // =========================================================
  Serial.println("[I2C] Mengaktifkan jalur SDA/SCL secara manual...");
  Wire.begin();
  Wire.setClock(100000); // Kunci kecepatan di 100kHz (Aman untuk semua sensor)
  delay(100);            // Beri nafas sedikit untuk driver I2C
  // =========================================================

  Serial.print("[OLED] Inisialisasi layar SSD1306... ");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("GAGAL! (Cek kabel SDA/SCL OLED)");
    for (;;)
      ; // Kunci sistem kalau OLED rusak/kabel putus
  }
  Serial.println("SUKSES.");

  Serial.print("[AHT10] Inisialisasi sensor suhu I2C... ");
  if (!aht.begin())
  {
    Serial.println("TIDAK DITEMUKAN! (Cek kabel AHT10)");
    aht_status = false;
  }
  else
  {
    Serial.println("SUKSES.");
    aht_status = true;
  }

  updateLayar("Starting...", "Initializing", "FreeRTOS Engine", "Please Wait");

  // MENJALANKAN CORE 0 (TUGAS JARINGAN)
  Serial.println("[CORE 1] Mendelegasikan tugas WiFi & MQTT ke Core 0...");
  xTaskCreatePinnedToCore(
      TaskNetwork, /* Fungsi Task */
      "TaskNet",   /* Nama Task */
      10000,       /* Ukuran Memory Stack */
      NULL,        /* Parameter */
      1,           /* Prioritas Task */
      NULL,        /* Handle */
      0);          /* Ditanam ke Core 0 */

  Serial.println("[CORE 1] Masuk ke Main Loop (Mengurus Animasi UI & Hardware).\n");
}

void loop()
{
  pantauSistem();
  cekTombolReset();
  vTaskDelay(10 / portTICK_PERIOD_MS); // Wajib agar Core 1 tidak kena Watchdog Trigger
}