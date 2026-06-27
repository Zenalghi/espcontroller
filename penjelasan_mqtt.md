# Arsitektur Komunikasi MQTT pada BMS ESP32

Dokumen ini menjelaskan bagaimana protokol **MQTT** (*Message Queuing Telemetry Transport*) diimplementasikan pada file `main.cpp` untuk keperluan pertukaran data dua arah secara *real-time*.

---

## 1. Konfigurasi Broker MQTT
- **Server:** `broker.emqx.io` (Public Broker)
- **Port:** `1883`
- **Topic Prefix:** `bms_panel/2602165`
- **Client ID:** Dihasilkan secara acak unik menggunakan MAC Address ESP32 (`espbms-<MAC><Random>`) agar tidak terjadi bentrok sesi (session collision) di public broker.
- **Buffer & Keep-Alive:** Diatur khusus ke 512 Bytes (BufferSize) dan 120 Detik (KeepAlive) untuk menahan data JSON yang panjang dan lag jaringan internet.

---

## 2. MQTT Subscribe (Menerima Data & Perintah)
ESP32 mendengarkan (*subscribe*) topik-topik berikut untuk menerima perintah dari pengguna atau data dari BMS Jikong.

### a. Kontrol Relay (Beban)
- **Topik:** `bms_panel/2602165/switch/relay_{1,2,3,4}/command`
- **Payload:** `"ON"` atau `"OFF"`
- **Fungsi Khusus (Proteksi Sistem):** Saat ESP32 menerima perintah `"ON"` untuk Relay 2, 3, atau 4, sistem akan mengecek nilai SoC (EKF). **Jika SoC baterai < 20%**, maka perintah akan **ditolak** demi keamanan baterai. (Hanya Relay 1 yang boleh menyala di bawah 20%).

### b. Kontrol Layar OLED
- **Topik:** `bms_panel/2602165/display/page/command`
- **Payload:** `"NEXT"` atau angka `"1"` sampai `"7"`
- **Fungsi Khusus:** Digunakan untuk mengubah halaman layar OLED via jaringan (remote). Perintah diabaikan jika ESP32 sedang berada di mode Setup WiFi (Portal) atau sedang melakukan pembaruan firmware (OTA).

### c. Inisialisasi SoC Bawaan (Auto-Calibration)
- **Topik:** `bms_panel/2602165/data/soc_bawaan`
- **Payload (JSON):** `{"soc_jk": 95.0}`
- **Fungsi Khusus:** Hanya berjalan satu kali (flag `is_soc_initialized`). Saat data SoC bawaan Jikong masuk, EKF dan Coulomb Counting akan direset menyamai nilai Jikong sebagai titik awal (kalibrasi otomatis) sebelum EKF berjalan sendiri.

### d. Data Mentah BMS (Input EKF)
- **Topik:** `bms_panel/2602165/data/main`
- **Payload (JSON):** Memuat nilai tegangan (`voltage`), arus (`current`), daya, suhu baterai/MOS, serta nilai array 8 sel tegangan (`cells_v`) dan resistansinya (`wire_res`).
- **Fungsi Khusus:** Saat data ini masuk secara utuh, sistem menyalakan tanda `flag_run_ekf = true`, yang akan mengeksekusi perhitungan Kalman Filter di Core 0 pada putaran sistem berikutnya.

---

## 3. MQTT Publish (Mengirim Data & Status)
ESP32 mengirimkan (*publish*) data ke luar sistem agar dapat dibaca oleh platform Dashboard atau sistem otomasi (seperti Home Assistant).

### a. Status Relay Aktual
- **Topik:** `bms_panel/2602165/state/relays`
- **Payload (JSON):** `{"relay_1":"ON", "relay_2":"OFF", ...}`
- **Trigger:** Dikirim setiap kali ada perubahan status relay.

### b. Hasil Kalkulasi Sistem (EKF & CC)
- **Topik:** `bms_panel/2602165/data/calc`
- **Payload (JSON):** Berisi hasil tebakan matematika ESP32, seperti:
  - `soc_ekf` (Tebakan pintar Closed-Loop)
  - `soc_cc` (Akumulasi murni Open-Loop)
  - `v_pred` (Tegangan yang diprediksi)
  - `avg_cell_v` (Rata-rata voltase per sel)
  - `dt` (Interval waktu antar paket, digunakan untuk debugging).
- **Trigger:** Dikirim tepat setelah EKF selesai dieksekusi.

---

## 4. Keandalan Sistem (Resilience)
- **Auto-Reconnect:** Jika sambungan ke EMQX putus, ESP32 akan melakukan `reconnectMQTT()` secara berkala setiap 5 detik dengan non-blocking (tidak membuat sistem hang).
- **Flag-based Execution:** Payload MQTT tidak langsung dieksekusi di dalam fungsi *callback* secara mentah-mentah (hal ini rawan menyebabkan *stack overflow* pada ESP32). Kode menggunakan pengaman berupa variabel global (`flag_run_ekf`, `flag_publish_relay`) sehingga eksekusi berat dilempar ke Loop Core 0 secara aman.
