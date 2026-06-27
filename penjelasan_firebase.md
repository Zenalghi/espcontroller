# Arsitektur Database Firebase pada BMS ESP32

Dokumen ini menguraikan bagaimana **Firebase Realtime Database (RTDB)** digunakan dalam file `main.cpp` untuk bertindak sebagai pencatat *history* awan (cloud logger) dan monitor *real-time*.

---

## 1. Konfigurasi & Koneksi Firebase
- **URL Database:** `bmsv1-f5b30-default-rtdb.asia-southeast1.firebasedatabase.app`
- **Test Mode Aktif:**
  - Terjadi masalah klasik pada ESP32 dimana *login auth* (Email/Anonymous) kerap memunculkan error `CONFIGURATION_NOT_FOUND` atau `INVALID_EMAIL`.
  - Kode menanganinya dengan fitur **Test Mode** (`config.signer.test_mode = true`). Fitur ini melewati seluruh proses otentikasi. Ini aman dilakukan asalkan *Rules* di Console Firebase telah diatur sebagai `true` (akses publik).

---

## 2. Sinkronisasi Waktu Internet (NTP)
Karena ESP32 tidak memiliki baterai RTC (*Real-Time Clock*) internal yang menyala saat mati listrik, ia harus mengambil waktu kalender asli dari internet agar data *History* di Firebase punya label waktu yang valid.
- **Server NTP:** `pool.ntp.org` dan `time.nist.gov`.
- **Zona Waktu:** Di-set ke UTC+7 (Waktu Indonesia Barat) menggunakan offset detik `7 * 3600`.
- **Fungsi:** Menyediakan jam (Timestamp) dan tanggal (untuk membuat nama folder log harian).

---

## 3. Mode Pengiriman Data
ESP32 menggunakan pengatur waktu milidetik (`millis()`) agar bisa mengirim data ke Firebase dalam dua ritme waktu yang berbeda tanpa menggunakan perintah `delay()` yang dapat membuat layar macet.

### a. Pengiriman Realtime (Setiap 60 Detik)
- **Interval:** 1 Menit (`FIREBASE_INTERVAL = 60000`).
- **Lokasi Path:** `/bms_realtime`
- **Metode:** Menggunakan fungsi **`setJSON()`**.
- **Perilaku:** Fungsi ini akan selalu menimpa (overwrite) data sebelumnya di Node `/bms_realtime`. Digunakan khusus agar antarmuka web/aplikasi ponsel bisa memantau kondisi baterai terkini.
- **Trigger Ekstra (Darurat):** Jika sewaktu-waktu SoC (EKF) mendadak turun hingga di bawah 20%, ESP32 akan menyalakan saklar `flag_send_firebase = true` seketika itu juga. Pengiriman segera dilakukan tanpa menunggu siklus 1 menit demi pelaporan status genting.

### b. Perekaman Jejak (History) (Setiap 15 Menit)
- **Interval:** 15 Menit (`FIREBASE_HISTORY_INTERVAL = 15 * 60 * 1000`).
- **Lokasi Path:** `/bms_history/YYYY-MM-DD` (Tanggal spesifik dibangkitkan dari NTP).
- **Metode:** Menggunakan fungsi **`pushJSON()`**.
- **Perilaku:** Alih-alih menimpa data, fungsi `push` akan menciptakan ID acak (unik) dan menempelkan tumpukan data baru ke bawah data lama. Sangat cocok untuk keperluan pembuatan grafik log data harian. 

---

## 4. Paket Data (Payload JSON)
Semua variabel dikemas ke dalam objek `FirebaseJson` sebelum diunggah dalam satu kali tembakan ke awan (untuk menghemat *bandwidth* kuota). 

Data yang diunggah meliputi:
1. **Timestamp Khusus:** `timestamp` ("HH:MM:SS" - didapat dari NTP).
2. **Kalkulasi & Daya:** `soc_ekf`, `soc_cc`, `voltage`, `current`, `power`.
3. **Diagnostik Kesehatan Baterai:** `avg_cell_v`, `max_cell_v`, `min_cell_v`, `delta_v` (ketidakseimbangan sel).
4. **Keamanan Thermal (Suhu):** `mos_temp`, `bat_temp1`, `bat_temp2` (Dari Jikong).
5. **Lingkungan Ruang (Environment):** `room_temp`, `room_hum` (Dari sensor I2C AHT10 bawaan ESP32).

---

## 5. Hubungan dengan FreeRTOS
Sama halnya dengan komputasi MQTT dan EKF, semua rutinitas sinkronisasi Firebase dieksekusi secara eksklusif di dalam `TaskNetwork` pada **Core 0**. Mengapa?

Proses SSL/TLS *(Handshake kriptografi yang aman)* yang digunakan oleh Firebase untuk mengamankan data terenkripsi HTTPS memakan siklus kerja prosesor secara masif (bisa *hang* selama 1-3 detik per pengiriman). Dengan mengisolasinya di **Core 0**, Core 1 tetap bisa dengan leluasa membaca sensor AHT10 dan menggambar menu di layar OLED 128x64 tanpa ada gejala tampilan grafis terhenti/putus-putus.
