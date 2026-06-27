# Informasi Arsitektur Kode `main.cpp`

File `main.cpp` ini adalah program utama untuk ESP32 yang difungsikan sebagai Battery Management System (BMS) Controller. Program ini dirancang dengan memanfaatkan fitur **Dual-Core** dari ESP32 menggunakan **FreeRTOS** agar performanya stabil dan antarmuka (layar) tidak macet saat terjadi proses jaringan yang berat.

Berikut adalah penjelasan pembagian tugas antara Core 0 dan Core 1, serta penggunaan FreeRTOS dalam kode ini.

---

## 1. Pembagian Tugas (Dual Core)

ESP32 memiliki dua inti prosesor (Core 0 dan Core 1). Pada lingkungan Arduino, secara default fungsi `setup()` dan `loop()` berjalan di Core 1, sedangkan WiFi/Radio stack berjalan di background pada Core 0. 

Namun pada kode ini, kita secara eksplisit membuat task FreeRTOS baru bernama `TaskNetwork` yang dipaku (pinned) ke **Core 0**. 

### 💻 **Core 0: Task Jaringan & Kalkulasi Berat (Backend)**
Core 0 didedikasikan untuk menangani semua operasi jaringan, protokol komunikasi, dan komputasi matematis EKF. Hal ini diatur di dalam fungsi `TaskNetwork()`.
Tugas yang berjalan di Core 0 meliputi:
- **Konektivitas WiFi & Portal Setup:** Diatur oleh `WiFiManager`.
- **Sinkronisasi Waktu (NTP):** Mengambil waktu dari internet untuk timestamp.
- **Komunikasi MQTT:** Menerima (subscribe) data tegangan, arus, suhu dari BMS Jikong dan mengirim (publish) hasil kalkulasi SoC serta status Relay.
- **Komputasi Extended Kalman Filter (EKF):** Kalkulasi SoC (State of Charge) baterai yang kompleks dan melibatkan operasi matriks, dijalankan ketika data terbaru masuk dari MQTT.
- **Integrasi Firebase:** Mengirim log status baterai ke Realtime Database dan History.
- **Over-The-Air (OTA) Update:** Menangani proses penerimaan firmware baru secara wireless via `ArduinoOTA`.

*Kode pembuatan Task (berada di `setup()`):*
```cpp
// TaskNet dialokasikan stack memory 15000 byte dan dipaku ke Core 0
xTaskCreatePinnedToCore(TaskNetwork, "TaskNet", 15000, NULL, 1, NULL, 0);
```
*(Catatan: Memory 15000 byte diberikan karena library Firebase dan SSL/TLS sangat boros memori saat mengirim data).*

### 🖥️ **Core 1: Antarmuka & Sensor Eksternal (Frontend)**
Core 1 menjalankan fungsi standar `loop()` dan `setup()`. Tugas utamanya adalah menangani antarmuka perangkat keras agar tetap responsif bagi pengguna, tanpa terpengaruh oleh delay jaringan (misal saat koneksi WiFi putus).
Tugas yang berjalan di Core 1 meliputi:
- **Pembaruan Layar OLED 128x64:** Menggambar User Interface (UI), menu multi-page, dan animasi progres bar saat OTA Update (Fungsi `updateLayar()` dan `drawOTAScreen()`). Layar diperbarui secara berkala setiap 500 ms.
- **Pembacaan Sensor Ruangan (AHT10):** Mengambil data suhu dan kelembaban ruangan (Fungsi `bacaSensorAHT()`).
- **Pembacaan Smart Button:** Mengecek penekanan tombol untuk berganti halaman (Short Press) atau membuka/menutup portal WiFi (Long Press 5 detik). (Fungsi `cekTombolSmart()`).

---

## 2. Inter-Process Communication (IPC) via FreeRTOS
Karena Core 0 dan Core 1 berjalan secara paralel (bersamaan), mereka perlu berkomunikasi satu sama lain. Komunikasi ini (IPC) dilakukan menggunakan **Variabel Global Volatile** yang bertindak sebagai jembatan (flag) antar-Core:

1. **`volatile bool portalActive`, `requestPortalOpen`, `requestPortalClose`**:
   - Jika tombol ditahan (oleh Core 1), Core 1 akan mengubah variabel `requestPortalOpen = true`. 
   - Core 0 yang memantau variabel ini akan menghentikan loop-nya sebentar untuk membuka WiFiManager Portal.
2. **`volatile bool ota_updating`, `ota_progress_percent`**:
   - Saat Core 0 menerima pembaruan firmware dari WiFi, ia akan mengubah `ota_updating = true` dan mengupdate persen progres.
   - Core 1 membaca flag tersebut dan langsung membekukan UI normal, beralih menampilkan layar `drawOTAScreen(percent)` sehingga progres download terlihat di layar OLED.
3. **`volatile bool flag_run_ekf`, `flag_publish_relay`, `flag_send_firebase`**:
   - Sebagai trigger agar komputasi jaringan/EKF tidak jalan terus-menerus melainkan berdasarkan *event* (kejadian), menjaga CPU load agar tidak 100%.

---

## Kesimpulan Desain
Dengan menggunakan **FreeRTOS Dual-Core Architecture**, sistem mendapatkan keuntungan berikut:
- **Responsive UI:** Layar OLED tidak akan pernah nge-*lag* atau nge-*freeze* meskipun MQTT sedang putus koneksi (reconnecting) atau Firebase sedang mengirim data (yang biasanya memblokir eksekusi selama hitungan detik).
- **Stabilitas Komputasi:** Kalkulasi matriks *Extended Kalman Filter* (EKF) bisa dijalankan secara intensif di Core 0 tanpa membuat pembacaan tombol dan update layar terhambat.
- **Manajemen Memori:** Alokasi memori Firebase dipisah secara independen di task network.
