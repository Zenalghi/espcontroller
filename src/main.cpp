#include <Arduino.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>

// Konfigurasi Layar OLED 0.91"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define BUTTON_PIN 0

// Objek AHT10 dan variabel status
Adafruit_AHTX0 aht;
bool aht_status = false;

// --- VARIABEL GLOBAL BARU ---
WiFiManager wm;            // Pindahkan ke global agar bisa diakses di semua void
bool portalActive = false; // Penanda status portal sedang buka/tutup
// ----------------------------

String teksBerjalan(String teks, int batasKarakter = 21)
{
  if (teks.length() <= batasKarakter)
  {
    return teks; // Kalau pendek, tampilkan normal
  }

  // Kecepatan geser: 300ms per karakter. Semakin kecil angka, semakin ngebut.
  int kecepatan = 300;
  int indeks = (millis() / kecepatan) % (teks.length() + 3);

  // Tambahkan spasi kosong sebagai jeda sebelum teks mengulang dari awal
  String teksGabung = teks + "   " + teks;

  // Potong teks sesuai batas layar
  return teksGabung.substring(indeks, indeks + batasKarakter);
}

// Fungsi pembantu untuk update teks di layar OLED
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
  const long jedaUpdate = 300; // <--- UBAH JADI 300ms AGAR ANIMASI MULUS

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

      // BUNGKUS TEKS YANG PANJANG DENGAN FUNGSI teksBerjalan()
      String teksWiFi = teksBerjalan("WiFi: " + WiFi.SSID());
      String teksIP = teksBerjalan("IP  : " + ipStr);

      updateLayar(
          "Ready!",
          teksWiFi,
          teksIP,
          "OTA Standby...");
    }
    else
    {
      String teksMencari = teksBerjalan("Searching: " + WiFi.SSID());
      updateLayar("Disconnected!", "Waiting for WiFi...", teksMencari, "System Active");
    }
  }
}

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
    }
    else if (millis() - waktuTekan > 5000)
    { // Jika tepat ditahan 5 detik
      if (!portalActive)
      {
        // 1. Tampilkan di layar DULUAN agar kita tahu 5 detik sudah pas
        updateLayar("", "Opening Portal...", "Please Release", "Button...");

        // 2. Baru proses buka WiFi di background
        wm.setConfigPortalBlocking(false);
        wm.startConfigPortal("esp-setup", "12345679");
        portalActive = true;
      }
      else
      {
        // Sama, tampilkan notif DULUAN
        updateLayar("", "Portal Closed!", "Please Release", "Button...");

        wm.stopConfigPortal();
        portalActive = false;
      }

      // 3. Kunci program di sini sampai tombol benar-benar dilepas
      while (digitalRead(BUTTON_PIN) == LOW)
      {
        delay(10);
      }

      sedangDitekan = false;

      // Catatan: Setelah tombol dilepas, layar akan otomatis kembali
      // ke tampilan utama dalam waktu maksimal 2 detik
      // (mengikuti jedaUpdate di fungsi pantauSistem).
    }
  }
  else
  {
    sedangDitekan = false; // Tombol dilepas
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("OLED Init Failed"));
    for (;;)
      ;
  }

  if (!aht.begin())
  {
    Serial.println("AHT10 Not Found!");
    aht_status = false;
  }
  else
  {
    aht_status = true;
  }

  updateLayar("WiFi...?",
              "WiFi: esp-setup",
              "Pass: 12345679",
              "IP  : 192.168.4.1");

  // Konfigurasi awal WiFiManager (saat baru nyala tetap pakai mode blocking standar)
  wm.setConfigPortalTimeout(180);
  bool res = wm.autoConnect("esp-setup", "12345679");

  if (!res)
  {
    updateLayar("", "Timeout", "ESP Restarting...");
    delay(3000);
    ESP.restart();
  }

  String ipStr = WiFi.localIP().toString();
  Serial.println("IP Address: " + ipStr);

  // ===================== SETUP OTA =====================
  ArduinoOTA.setHostname("esp-yoyo");
  ArduinoOTA.onStart([]()
                     { updateLayar("", "OTA Starting..."); });
  ArduinoOTA.onEnd([]()
                   { updateLayar("", "OTA Complete!", "Rebooting..."); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    unsigned int percent = (progress / (total / 100));
    updateLayar("","OTA Updating...", "Progress: " + String(percent) + "%"); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    updateLayar("","OTA FAILED!");
    delay(3000); });
  ArduinoOTA.begin();

  // Tahan layar info sebentar
  delay(2000);
}

void loop()
{
  ArduinoOTA.handle();

  // PENTING: Jika portal sedang aktif, izinkan WiFiManager memproses request web
  if (portalActive)
  {
    wm.process();
  }

  pantauSistem();
  cekTombolReset();
}