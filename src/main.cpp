// =============================================================================
// PROGRAM  : BMS (Battery Management System) Controller
// PLATFORM : ESP32 DevKit V1
// DESKRIPSI: Program ini berjalan di ESP32 yang berperan sebagai "otak" dari
//            sistem manajemen baterai. Tugas utamanya adalah:
//            1. Menerima data mentah dari BMS Jikong via MQTT
//            2. Menghitung estimasi SoC (State of Charge / Kondisi Baterai)
//               menggunakan algoritma Extended Kalman Filter (EKF)
//            3. Menampilkan data ke layar OLED 128x64
//            4. Mengirimkan data ke server Firebase (Cloud)
//            5. Mengontrol 4 buah Relay sebagai pemutus daya otomatis
// =============================================================================

// --- LIBRARY (Pustaka) yang digunakan ---
// Arduino.h      : Library inti ESP32/Arduino (wajib ada)
// WiFiManager.h  : Mengelola koneksi WiFi via portal web (tidak perlu hardcode password)
// ArduinoOTA.h   : Update firmware ESP32 tanpa kabel (Over-The-Air), cukup via jaringan WiFi
// Wire.h         : Protokol komunikasi I2C (dipakai OLED & sensor AHT)
// Adafruit_GFX.h : Library grafis dasar untuk menggambar teks/garis di layar
// Adafruit_SSD1306.h : Driver khusus untuk layar OLED SSD1306 128x64
// Adafruit_AHTX0.h   : Driver sensor suhu & kelembapan AHT10/AHT20
// PubSubClient.h : Library untuk protokol MQTT (kirim-terima pesan ke broker)
// ArduinoJson.h  : Mem-parse (membaca) dan membuat data format JSON
// Firebase_ESP_Client.h : Library resmi untuk komunikasi ke Firebase Google
// time.h         : Library standar C untuk mendapatkan waktu dari internet (NTP)
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
#include <time.h>

// =========================================================
// 1. KONFIGURASI HARDWARE (OLED, SENSOR, RELAY, TOMBOL)
// =========================================================
// SCREEN_WIDTH / SCREEN_HEIGHT: Resolusi layar OLED dalam piksel
// OLED_RESET = -1 berarti tidak pakai pin reset fisik, reset dilakukan via software
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
// Membuat objek 'display' yang akan kita gunakan untuk menggambar ke layar OLED
// Parameter: (lebar, tinggi, objek_I2C, pin_reset)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BUTTON_PIN = GPIO 0 adalah tombol BOOT bawaan ESP32.
// Kita "bajak" fungsinya untuk navigasi halaman OLED & buka portal WiFi.
#define BUTTON_PIN 0

// Objek untuk membaca sensor suhu & kelembapan AHT10
Adafruit_AHTX0 aht;
bool aht_status = false; // Status true = sensor AHT berhasil dideteksi, false = tidak ada / error
float room_temp = 25.0;  // Suhu ruangan (default 25°C sebelum sensor dibaca pertama kali)
float room_hum = 80.0;   // Kelembapan ruangan (default 80% RH)

// RELAY_PINS: Nomor pin GPIO yang terhubung ke modul relay.
// TIPE ACTIVE LOW artinya:
//   - digitalWrite(pin, LOW)  --> Relay AKTIF / Arus MENGALIR
//   - digitalWrite(pin, HIGH) --> Relay MATI  / Arus TERPUTUS
const int RELAY_PINS[4] = {26, 25, 33, 32};
// relayState: Menyimpan kondisi logis setiap relay (true = ON, false = OFF)
bool relayState[4] = {false, false, false, false};

// =========================================================
// 2. KONFIGURASI JARINGAN, MQTT & FIREBASE
// =========================================================
// MQTT adalah protokol pesan ringan berbasis Publish/Subscribe.
// ESP32 kita berlangganan (subscribe) ke topik tertentu untuk menerima data BMS Jikong,
// dan mem-publish hasil kalkulasi EKF ke topik lain.
// "broker.emqx.io" adalah server MQTT publik gratis milik EMQX.
// const char *mqtt_server = "broker.mqtt.cool"; // (broker lama, dinonaktifkan)
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // Port standar MQTT (tanpa enkripsi)
// mqtt_prefix: Awalan unik untuk semua topik MQTT kita, agar tidak bentrok dengan perangkat lain
const char *mqtt_prefix = "bms_panel/2602165";

// espClient : Objek koneksi TCP/IP via WiFi yang akan digunakan MQTT
// mqtt      : Objek utama untuk semua operasi MQTT (publish, subscribe, dll)
// wm        : Objek WiFiManager untuk manajemen koneksi WiFi
WiFiClient espClient;
PubSubClient mqtt(espClient);
WiFiManager wm;

// --- KONFIGURASI FIREBASE ---
// Firebase Realtime Database digunakan sebagai cloud storage untuk:
// 1. Dashboard Realtime (/bms_realtime): Data terkini yang selalu ditimpa setiap 60 detik
// 2. Riwayat Historis (/bms_history/YYYY-MM-DD): Data yang diakumulasi setiap 15 menit
#define FIREBASE_URL "bmsv1-f5b30-default-rtdb.asia-southeast1.firebasedatabase.app"
#define API_KEY "AIzaSyBQampK8P14r7gqOH9NDjuE9pAOE9WpB24"

FirebaseData fbdo;   // Objek untuk operasi baca/tulis ke Firebase
FirebaseAuth auth;   // Objek untuk otentikasi (login) ke Firebase
FirebaseConfig config; // Objek untuk menyimpan konfigurasi (URL, API Key) Firebase
bool signupOK = false; // Flag penanda apakah koneksi Firebase sudah siap digunakan

// =========================================================
// 3. PARAMETER MODEL BATERAI (Cubic Spline 21 Titik)
// =========================================================
// Q_AH: Kapasitas Nominal Baterai dalam Ampere-Hour (Ah)
const float Q_AH = 20.798555;
// Q_COULOMB: Kapasitas dikonversi menjadi Coulomb (Ampere-detik).
// Digunakan dalam rumus SoC karena 'dt' (delta time) bersatuan detik (1 Ah = 3600 Coulomb).
const float Q_COULOMB = Q_AH * 3600.0;

// LUT_OCV_SIZE: Jumlah titik pada tabel pemetaan (Lookup Table) OCV vs SoC
const int LUT_OCV_SIZE = 21;
// lut_soc_ocv: Array nilai SoC (sumbu X) dari 0% (0.0) hingga 100% (1.0) dengan interval 5%
const float lut_soc_ocv[LUT_OCV_SIZE] = {
    0.00, 0.05, 0.10, 0.15, 0.20,
    0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.65, 0.70,
    0.75, 0.80, 0.85, 0.90, 0.95, 1.00};

// lut_ocv: Array nilai Tegangan Open Circuit (OCV) yang berkorespondensi dengan array lut_soc_ocv di atas.
// Disesuaikan agar nilainya selalu naik (monotonik) supaya turunan matematisnya tidak pernah nol.
const float lut_ocv[LUT_OCV_SIZE] = {
    2.6550, 3.0269, 3.1972, 3.2391, 3.2261,
    3.2242, 3.2424, 3.2625, 3.2758, 3.2835,
    3.2871, 3.2880, 3.2878, 3.2884, 3.2917,
    3.2958, 3.2973, 3.3039, 3.3353, 3.4122, 3.5370};

// LUT_ECM_SIZE: Jumlah titik pada tabel parameter model sirkuit ekuivalen (ECM 1-RC Thevenin)
const int LUT_ECM_SIZE = 9;
// lut_soc_ecm: Array SoC untuk pemetaan parameter R0, R1, C1
const float lut_soc_ecm[LUT_ECM_SIZE] = {
    0.0, 0.090902, 0.204618, 0.318054, 0.431697,
    0.545421, 0.659070, 0.772787, 0.886430};
// lut_r0: Array nilai Resistansi Internal Ohmic (R0) dalam satuan Ohm
const float lut_r0[LUT_ECM_SIZE] = {
    0.006050, 0.002800, 0.002800, 0.002899, 0.002700,
    0.002400, 0.002899, 0.002199, 0.002800};
// lut_r1: Array nilai Resistansi Polarisasi (R1) dalam satuan Ohm (Bagian dari sirkuit RC)
const float lut_r1[LUT_ECM_SIZE] = {
    0.009500, 0.002506, 0.002207, 0.002212, 0.002372,
    0.002436, 0.002374, 0.002345, 0.002684};
// lut_c1: Array nilai Kapasitansi Polarisasi (C1) dalam satuan Farad (Bagian dari sirkuit RC)
const float lut_c1[LUT_ECM_SIZE] = {
    11281.15, 20591.86, 24841.48, 15061.40, 20897.75,
    19607.70, 15177.97, 16580.74, 24189.08};

// =========================================================
// 4. TUNING NOISE PARAMETER (Matriks Q dan R)
// =========================================================
// Q_NOISE (Process Noise Covariance): Seberapa besar EKF percaya pada MODEL matematisnya sendiri.
//   Nilai BESAR = EKF tidak percaya model, lebih bergantung pada sensor.
//   Nilai KECIL = EKF percaya modelnya, lambat bereaksi terhadap perubahan sensor.
//   Q_NOISE_00 = noise untuk SoC (nilai kecil = asumsi SoC berubah pelan/stabil)
//   Q_NOISE_11 = noise untuk Vc1 (nilai besar = tegangan kapasitor bisa berubah cepat)
const float Q_NOISE_00 = 2e-6f;
const float Q_NOISE_11 = 1e-1f;

// R_BASE (Measurement Noise Covariance): Seberapa besar EKF percaya pada SENSOR tegangan.
//   Nilai BESAR = EKF tidak percaya sensor, filter lebih halus tapi lambat koreksi.
//   Nilai KECIL = EKF sangat percaya sensor, koreksi agresif.
const float R_BASE = 1e-4f;

// REST_CURRENT_THRESH: Batas arus (Ampere) untuk mendeteksi bahwa baterai sedang 'istirahat' (tidak ada beban/pengisian)
const float REST_CURRENT_THRESH = 0.05f; // A - di bawah ini = rest
// REST_SETTLE_S: Berapa detik arus harus di bawah threshold secara TERUS-MENERUS untuk dikonfirmasi sebagai 'confirmed rest'
const int REST_SETTLE_S = 30;            // detik konfirmasi rest sebelum R_REST aktif
// R_REST: Nilai R yang dipakai saat baterai sedang istirahat.
// Saat istirahat, tegangan sensor sangat akurat (= OCV sejati), jadi EKF dipercaya lebih agresif.
const float R_REST = 1e-4f;              // agresif saat confirmed rest (sama R_BASE)

// =========================================================
// 5. VARIABEL GLOBAL IPC, STATE ESTIMATION, & UI
// =========================================================
// "volatile" dipakai karena variabel-variabel ini diakses dari 2 Core ESP32 secara bersamaan.
// Kata kunci ini memberitahu compiler agar TIDAK meng-cache nilai variabel tersebut,
// sehingga setiap akses selalu membaca dari memori asli (mencegah bug antar-core).

// portalActive      : true = Sedang menampilkan portal konfigurasi WiFi, mode normal dibekukan
// requestPortalOpen : Sinyal dari Core 1 (tombol) ke Core 0 (jaringan) untuk BUKA portal
// requestPortalClose: Sinyal dari Core 1 (tombol) ke Core 0 (jaringan) untuk TUTUP portal
// ota_updating      : true = Firmware sedang di-update via OTA, semua fungsi lain dibekukan
// ota_progress_percent : Persentase (0-100) progress download firmware OTA untuk ditampilkan di OLED
volatile bool portalActive = false;
volatile bool requestPortalOpen = false;
volatile bool requestPortalClose = false;
volatile bool ota_updating = false;
volatile int ota_progress_percent = 0;

// === FLAG (BENDERA) KOMUNIKASI ANTAR TASK ===
// Pola ini digunakan agar proses yang butuh waktu lama (EKF, Firebase) tidak berjalan
// DI DALAM fungsi mqttCallback (yang dipanggil saat data MQTT diterima).
// Jika dijalankan langsung di callback, bisa menyebabkan buffer MQTT overflow/hang.
// Solusinya: set flag = true di callback, lalu eksekusi sesungguhnya dilakukan di loop utama.
volatile bool flag_run_ekf = false;         // Sinyal: ada data MQTT baru, jalankan EKF!
volatile bool flag_publish_relay = false;    // Sinyal: ada perubahan relay, kirim statusnya via MQTT
volatile bool flag_send_firebase = false;    // Sinyal: waktunya kirim data realtime ke Firebase
volatile bool flag_soc_critical_sent = false;// Penanda: sudah kirim notifikasi SOC kritis, jangan kirim ulang
volatile bool is_first_ekf_sent = false;     // Penanda: apakah EKF pertama sudah dikirim ke Firebase

// Variabel performa sistem yang ditampilkan di Halaman 7 OLED
volatile float perf_ekf_time_ms = 0.0;       // Waktu eksekusi EKF dalam milidetik
volatile float perf_ram_used_pct = 0.0;      // Persentase penggunaan RAM (Heap)
volatile uint32_t perf_ram_used_bytes = 0;   // RAM yang terpakai dalam Bytes
volatile uint32_t perf_ram_total_bytes = 0;  // Total RAM yang tersedia

// currentPage: Halaman OLED yang sedang ditampilkan (mulai dari halaman 1 saat boot)
volatile int currentPage = 1;
const int MAX_PAGES = 7; // Total ada 7 halaman di layar OLED

// Struct BMS_Data: Menampung data mentah yang datang dari MQTT (BMS Jikong)
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

// --- VARIABEL UNTUK COULOMB COUNTING (CC) ---
// Coulomb Counting adalah metode sederhana menghitung SoC hanya dari arus yang masuk/keluar.
// Rumus: SoC_baru = SoC_lama - (Arus * Waktu / Kapasitas_Total)
// Kelemahan: Bisa 'drift' (meleset) karena tidak ada koreksi dari sensor tegangan.
// soc_cc diinisialisasi ke 0.9414 = 94.14% (nilai sementara sebelum dapat data dari Jikong)
float soc_cc = 0.9414;

// --- VARIABEL UNTUK EXTENDED KALMAN FILTER (EKF) ---
// EKF adalah metode yang LEBIH CANGGIH dari Coulomb Counting.
// EKF menggabungkan dua sumber informasi:
//   1. Prediksi dari model (seperti Coulomb Counting)
//   2. Koreksi dari sensor tegangan fisik
// Hasilnya adalah estimasi SoC yang jauh lebih akurat & tahan terhadap drift.

// ekf_x[2]: State Vector (Vektor Status) = "kondisi baterai saat ini menurut EKF"
// ekf_x[0] = Estimasi SoC (0.0 - 1.0, setara 0% - 100%)
// ekf_x[1] = Estimasi Vc1 (Tegangan kapasitor RC dalam model Thevenin, dalam Volt)
float ekf_x[2] = {0.9414, 0.0};

// ekf_P[2][2]: Error Covariance Matrix (Matriks Kovarian Error / Matriks Keraguan).
// Ini adalah UKURAN KETIDAKPASTIAN EKF terhadap estimasinya sendiri.
// Nilai besar = EKF ragu terhadap estimasinya (akan lebih percaya sensor).
// Nilai kecil = EKF yakin terhadap estimasinya (akan kurang terpengaruh oleh sensor).
// Matriks P akan otomatis diperbarui setiap siklus menggunakan Joseph Form Update.
// P[0][0] = Ketidakpastian pada SoC | P[1][1] = Ketidakpastian pada Vc1
float ekf_P[2][2] = {{0.1, 0.0}, {0.0, 0.01}};

// v_pred_last: Menyimpan hasil prediksi tegangan terakhir (untuk ditampilkan ke Serial Monitor)
float v_pred_last = 0.0;
// dt_last: Menyimpan selisih waktu (detik) antar 2 siklus EKF terakhir (untuk tampilan OLED)
float dt_last = 0.0;

// Variabel pendeteksi kondisi 'rest' (baterai tidak dibebani / tidak diisi)
int rest_counter_s = 0;      // Penghitung berapa detik arus sudah mendekati nol secara terus-menerus
bool in_confirmed_rest = false; // true = baterai DIPASTIKAN sedang istirahat

// Timer untuk mengatur kapan data dikirim ke Firebase (dalam milidetik)
unsigned long last_mqtt_time = 0;              // Waktu terakhir data MQTT diterima
unsigned long last_firebase_time = 0;          // Waktu terakhir data dikirim ke /bms_realtime
unsigned long last_firebase_history_time = 0;  // Waktu terakhir data dikirim ke /bms_history
const unsigned long FIREBASE_INTERVAL = 60000; // Interval kirim data realtime = 60 detik
const unsigned long FIREBASE_HISTORY_INTERVAL = 15 * 60 * 1000; // Interval simpan history = 15 menit
volatile bool flag_send_history = false; // Sinyal: waktunya simpan data history ke Firebase

bool is_first_run = true;    // true selama belum pernah ada satu pun data MQTT yang diterima
bool lastWiFiState = false;  // Menyimpan status WiFi siklus sebelumnya (untuk deteksi perubahan)

// FLAG AUTO-CALIBRATION:
// EKF dan CC hanya boleh berjalan SETELAH mendapat nilai SoC awal dari BMS Jikong.
// Tanpa nilai awal yang benar, hasil kalkulasi EKF tidak akan akurat sejak awal.
// is_soc_initialized diset true setelah menerima topik MQTT '/data/soc_bawaan' dari Jikong.
bool is_soc_initialized = false;

// =========================================================
// 6. FUNGSI MATEMATIKA: INTERPOLASI & EKF
// =========================================================

// interpolate1D: Interpolasi Linear 1 Dimensi
// Fungsi ini mencari nilai Y yang sesuai untuk sebuah nilai X di antara 2 titik pada tabel.
// Analogi: Bayangkan tabel konversi suhu. Jika suhu 92°C tidak ada di tabel, tapi 90°C dan 95°C ada,
// fungsi ini akan MENEBAK nilai antara keduanya secara proporsional (linear).
//
// Parameter:
//   x      : Nilai X yang ingin dicari Y-nya (misal: SoC saat ini)
//   x_data : Array nilai X di tabel (misal: lut_soc_ocv)
//   y_data : Array nilai Y di tabel (misal: lut_ocv)
//   size   : Jumlah titik dalam tabel
// Return: Nilai Y hasil interpolasi
float interpolate1D(float x, const float *x_data, const float *y_data, int size)
{
  // Jika X lebih kecil dari data terkecil, kembalikan nilai pertama (boundary bawah)
  if (x <= x_data[0])
    return y_data[0];
  // Jika X lebih besar dari data terbesar, kembalikan nilai terakhir (boundary atas)
  if (x >= x_data[size - 1])
    return y_data[size - 1];
  // Cari di interval mana X berada
  for (int i = 0; i < size - 1; i++)
  {
    if (x >= x_data[i] && x <= x_data[i + 1])
    {
      // Hitung faktor interpolasi 't' (0.0 = di titik kiri, 1.0 = di titik kanan)
      float t = (x - x_data[i]) / (x_data[i + 1] - x_data[i]);
      // Rumus interpolasi linear: Y = Y_kiri + t * (Y_kanan - Y_kiri)
      return y_data[i] + t * (y_data[i + 1] - y_data[i]);
    }
  }
  return y_data[0]; // Fallback jika tidak ketemu (seharusnya tidak pernah sampai sini)
}

// get_dOCV_dSOC: Menghitung turunan (slope/kemiringan) kurva OCV terhadap SoC menggunakan Metode Numerik.
// Dalam matematika EKF, ini disebut elemen MATRIKS JACOBIAN (H).
//
// Mengapa turunan ini penting?
//   - Kurva OCV-SoC tidak linear! Di daerah SoC tertentu, kemiringannya sangat landai (datar).
//   - Jika kemiringan kecil (kurva datar), perubahan SoC hanya menghasilkan perubahan tegangan kecil.
//     Artinya sensor tegangan menjadi TIDAK SENSITIF, sehingga EKF harus kurang mempercayai sensor.
//   - Jika kemiringan besar (kurva curam), sensor tegangan sangat informatif, EKF bisa lebih percaya.
//
// Metode: Pendekatan beda-hingga (Finite Difference) / Diferensiasi Numerik
//   dOCV/dSOC ≈ (OCV(soc+h) - OCV(soc-h)) / (2h)
float get_dOCV_dSOC(float soc)
{
  soc = constrain(soc, 0.0f, 1.0f); // Pastikan SoC tidak keluar dari rentang 0.0 - 1.0
  float h = 0.005f; // Langkah kecil untuk aproksimasi turunan (h = 0.5% SoC)
  float soc_lo = max(soc - h, 0.0f); // Titik bawah (dengan batas minimal 0.0)
  float soc_hi = min(soc + h, 1.0f); // Titik atas (dengan batas maksimal 1.0)
  float dSOC = soc_hi - soc_lo;      // Selisih SoC antara dua titik
  if (dSOC < 1e-6f) // Hindari pembagian dengan nol
    return 0.0f;
  // Hitung kemiringan kurva OCV: (OCV_atas - OCV_bawah) / (SoC_atas - SoC_bawah)
  return (interpolate1D(soc_hi, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE) - interpolate1D(soc_lo, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE)) / dSOC;
}

// =========================================================
// ENGINE EKF (INTI DARI PERHITUNGAN BATERAI)
// =========================================================
// runEKFStep: Menjalankan SATU SIKLUS LENGKAP Extended Kalman Filter.
// Fungsi ini dipanggil setiap kali ada data baru dari MQTT (kurang lebih tiap 1 detik).
//
// Parameter:
//   I_meas  : Arus baterai yang terukur sensor (Ampere). (+) = discharge, (-) = charge.
//   V_meas  : Rata-rata tegangan per sel yang terukur sensor (Volt).
//   dt      : Delta Time = selisih waktu sejak pemanggilan terakhir (detik).
//
// Alur Kerja EKF dalam 2 tahap utama:
//   [TAHAP 1 - PREDIKSI]  : EKF menebak kondisi baterai berikutnya berdasarkan model fisik.
//   [TAHAP 2 - KOREKSI]   : EKF memperbaiki tebakannya berdasarkan selisih antara
//                           tegangan yang diprediksi vs yang diukur sensor fisik.
void runEKFStep(float I_meas, float V_meas, float dt)
{
  // -------------------------------------------------------------
  // METODE 1: COULOMB COUNTING (PENGUKURAN OPEN-LOOP)
  // Menghitung SoC hanya berdasarkan rumus integrasi arus: SoC_baru = SoC_lama - (Arus * waktu / Kapasitas Total)
  // -------------------------------------------------------------
  soc_cc = constrain(soc_cc - (I_meas * dt / Q_COULOMB), 0.0, 1.0);

  // -------------------------------------------------------------
  // METODE 2: EXTENDED KALMAN FILTER (PENGUKURAN CLOSED-LOOP)
  // -------------------------------------------------------------

  // soc_prev, vc1_prev: Menyalin hasil estimasi dari perhitungan siklus lalu untuk dijadikan bahan tebakan.
  float soc_prev = constrain(ekf_x[0], 0.0, 1.0);
  float vc1_prev = ekf_x[1];

  // R0, R1, C1: Parameter Equivalent Circuit Model baterai aktual.
  // Nilainya berubah-ubah secara dinamis tergantung pada berapa SoC baterai saat ini (dicari via interpolasi).
  float R0 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r0, LUT_ECM_SIZE), 0.0001f);
  float R1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r1, LUT_ECM_SIZE), 0.0001f);
  float C1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_c1, LUT_ECM_SIZE), 1.0f);

  // tau (τ): Konstanta waktu untuk komponen RC. Menggambarkan seberapa cepat tegangan polarisasi terbentuk/hilang.
  float tau = max(R1 * C1, 0.000001f);

  // === TAHAP 1: PREDIKSI (A PRIORI / TEBAKAN AWAL) ============================

  // soc_pred: EKF menebak nilai SoC berikutnya menggunakan rumus Coulomb Counting.
  float soc_pred = constrain(soc_prev - (I_meas * dt / Q_COULOMB), 0.0, 1.0);
  // alpha & vc1_pred: EKF menebak nilai tegangan kapasitor (Vc1) berikutnya berdasarkan laju peluruhan RC.
  float alpha = (dt > 0) ? expf(-dt / tau) : 1.0f;
  float vc1_pred = (alpha * vc1_prev) + (R1 * (1.0f - alpha) * I_meas);

  // Memasukkan sementara tebakan awal ke dalam State Vector.
  ekf_x[0] = soc_pred;
  ekf_x[1] = vc1_pred;

  // Prediksi Covariance P (decoupled)
  float P_pred[2][2];
  P_pred[0][0] = ekf_P[0][0] + Q_NOISE_00;
  P_pred[0][1] = 0.0f;
  P_pred[1][0] = 0.0f;
  P_pred[1][1] = (alpha * alpha * ekf_P[1][1]) + Q_NOISE_11;

  float dOCV_dSOC = get_dOCV_dSOC(soc_pred);

  // Dynamic R (R_dynamic): Noise pengukuran yang bisa berubah-ubah secara dinamis.
  // Ide utamanya: Kepercayaan EKF terhadap sensor tegangan disesuaikan dengan kondisi baterai.
  //
  // Aturannya:
  //   - Saat REST TERKONFIRMASI: EKF sangat percaya sensor (R kecil = agresif)
  //     karena saat istirahat V_meas ≈ OCV sejati, jadi sangat informatif.
  //   - Saat arus kecil tapi belum confirmed rest:
  //     R disesuaikan dengan kemiringan kurva OCV (dOCV_dSOC).
  //     Kurva curam → sensor informatif → R kecil → koreksi agresif.
  //     Kurva datar  → sensor tidak informatif → R besar → koreksi halus.
  //   - Saat arus besar (baterai diisi/dibebani):
  //     Sama seperti atas tapi lebih sensitif (epsilon lebih kecil = R bisa lebih kecil).
  float R_dynamic;
  if (in_confirmed_rest)
  {
    R_dynamic = R_REST; // Mode paling percaya sensor (baterai benar-benar istirahat)
  }
  else if (abs(I_meas) < 0.05f)
  {
    // Arus nyaris nol, tapi belum 30 detik: kepercayaan proporsional dengan kemiringan OCV
    R_dynamic = R_BASE / (abs(dOCV_dSOC) + 1e-3f);
  }
  else
  {
    // Ada arus signifikan: sensor tetap diperhitungkan, tapi lebih waspada terhadap drop tegangan
    R_dynamic = R_BASE / (abs(dOCV_dSOC) + 1e-4f);
  }
  // Batasi R_dynamic agar tidak terlalu kecil (0.0001) atau terlalu besar (10.0)
  R_dynamic = constrain(R_dynamic, 0.0001f, 10.0f);

  // === TAHAP 2: KOREKSI (A POSTERIORI / PERBAIKAN TEBAKAN) ==========================

  // OCV_pred: Cari nilai OCV (tegangan idealnya baterai jika tidak ada beban/arus)
  // yang berkoresponden dengan nilai SoC tebakan dari Tahap 1.
  float OCV_pred = interpolate1D(soc_pred, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);

  // V_pred: EKF menghitung berapa tegangan yang SEHARUSNYA dibaca sensor jika tebakan EKF benar.
  // Rumus Model Sirkuit Ekuivalen (ECM) Thevenin 1-RC:
  //   V_terminal = OCV - Vc1 - (I * R0)
  //   V_terminal = Tegangan terminal (yang dibaca sensor)
  //   OCV        = Tegangan sumber ideal baterai (Open Circuit Voltage)
  //   Vc1        = Drop tegangan di kapasitor (efek polarisasi baterai)
  //   I * R0     = Drop tegangan di resistor ohmik internal (langsung proporsional dengan arus)
  float V_pred = OCV_pred - vc1_pred - (I_meas * R0);
  v_pred_last = V_pred; // Simpan untuk keperluan tampilan/debug

  // Matriks H (h0, h1): Linearisasi dari fungsi pengukuran non-linear.
  // Ini adalah baris Matriks Jacobian yang menghubungkan State Space ke pengukuran.
  //   h0 = ∂V_terminal/∂SoC = dOCV/dSOC (seberapa besar perubahan SoC mempengaruhi tegangan)
  //   h1 = ∂V_terminal/∂Vc1 = -1 (tegangan kapasitor mengurangi tegangan terminal)
  float h0 = fabsf(dOCV_dSOC) + 1e-4f; // +1e-4f agar tidak pernah nol (mencegah K=0)
  float h1 = -1.0f;

  // S (Innovation Covariance): Total ketidakpastian dari tebakan pengukuran.
  // S = H * P_pred * H^T + R
  // Semakin besar S, semakin tidak yakin EKF terhadap pengukuran → Kalman Gain mengecil.
  float S = (h0 * h0 * P_pred[0][0]) + (h0 * h1 * P_pred[0][1]) +
            (h1 * h0 * P_pred[1][0]) + (h1 * h1 * P_pred[1][1]) + R_dynamic;
  if (S < 1e-9f) // Pastikan S tidak pernah nol (mencegah pembagian dengan nol)
    S = 1e-9f;

  // K (Kalman Gain): BOBOT KOREKSI - inti dari seluruh algoritma EKF!
  // K = P_pred * H^T / S
  // K[0] = Kalman Gain untuk SoC (seberapa banyak error tegangan mengkoreksi SoC)
  // K[1] = Kalman Gain untuk Vc1 (seberapa banyak error tegangan mengkoreksi Vc1)
  // Jika K besar (mendekati 1): EKF sangat percaya sensor, koreksi agresif.
  // Jika K kecil (mendekati 0): EKF kurang percaya sensor, koreksi minimal.
  float K[2];
  K[0] = ((P_pred[0][0] * h0) + (P_pred[0][1] * h1)) / S;
  K[1] = ((P_pred[1][0] * h0) + (P_pred[1][1] * h1)) / S;

  // Innovation (innov): Selisih antara tegangan yang BENAR-BENAR diukur sensor (V_meas)
  // vs tegangan yang DIPREDIKSI model (V_pred).
  // Ini adalah 'sinyal koreksi' yang akan dipakai untuk memperbaiki estimasi EKF.
  // innov > 0 artinya SoC aktual lebih tinggi dari perkiraan (EKF perlu naikkan SoC)
  // innov < 0 artinya SoC aktual lebih rendah dari perkiraan (EKF perlu turunkan SoC)
  float innov = V_meas - V_pred;

  // --- SOFT DEADBAND (1mV) + CORRECTION CAP (10% SoC per langkah) ---
  // Ini adalah dua lapisan perlindungan agar EKF tidak membuat koreksi yang terlalu drastis.

  // 1. SOFT DEADBAND: Jika error tegangan sangat kecil (< 1mV), mungkin itu hanya noise sensor.
  //    Daripada mengabaikan sepenuhnya (hard deadband), kita perkecil gain-nya secara proporsional
  //    (soft deadband). Ini mencegah SoC bergoyang liar akibat noise kecil.
  float K0_eff = K[0]; // K0_eff = Kalman Gain efektif (bisa dikurangi oleh deadband)
  const float DEADBAND = 0.001f;      // 1 mV = ambang batas noise minimum
  const float MAX_CORRECTION = 0.10f; // Batas maksimum koreksi SoC = 10% per siklus

  if (fabsf(innov) < DEADBAND)
  {
    // Kurangi gain secara linear sesuai besarnya innovasi (0 mV → gain=0, 1 mV → gain penuh)
    K0_eff *= (fabsf(innov) / DEADBAND);
  }

  // 2. CORRECTION CAP: Batasi perubahan SoC maksimal 10% dalam satu siklus.
  //    Ini mencegah EKF 'loncat' jauh tiba-tiba akibat pembacaan tegangan yang anomali (noise spike).
  float soc_correction = K0_eff * innov; // Hitung besarnya koreksi SoC
  if (soc_correction > MAX_CORRECTION)   // Jika terlalu besar positif, batasi
    soc_correction = MAX_CORRECTION;
  if (soc_correction < -MAX_CORRECTION)  // Jika terlalu besar negatif, batasi
    soc_correction = -MAX_CORRECTION;

  // Terapkan koreksi ke State Vector:
  // SoC_baru = SoC_prediksi + K0_efektif * innovasi  (dibatasi 0.0-1.0)
  ekf_x[0] = max(0.0f, min(1.0f, ekf_x[0] + soc_correction));
  // Vc1_baru = Vc1_prediksi + K1 * innovasi (tanpa deadband/cap)
  ekf_x[1] += K[1] * innov;

  // Batasi Vc1 agar tidak lari ke nilai tidak fisik (maksimal ±0.5V)
  // Tegangan drop kapasitor tidak mungkin melebihi 0.5V untuk baterai LiFePO4
  if (ekf_x[1] > 0.5f)
    ekf_x[1] = 0.5f;
  if (ekf_x[1] < -0.5f)
    ekf_x[1] = -0.5f;

  // Memperbarui Matriks Kovarian Error (P) menggunakan JOSEPH FORM UPDATE.
  // Rumus standar: P_baru = (I - K*H) * P_pred * (I - K*H)^T + K*R*K^T
  //
  // Mengapa Joseph Form dan bukan rumus sederhana (P = (I-KH)*P)?
  // Karena Joseph Form secara matematis lebih NUMERICALLY STABLE (stabil secara numerik)
  // dan menjamin matriks P selalu SIMETRIS dan POSITIF DEFINIT (nilai diagonal selalu > 0).
  // Ini sangat penting untuk mencegah EKF dari 'divergence' (meledak atau meliarkan diri).

  // Hitung (I - K*H): Matriks pembobot untuk pembaruan P
  float I_KH[2][2];
  I_KH[0][0] = 1.0f - (K0_eff * h0); // 1 - K0*H0
  I_KH[0][1] = -(K0_eff * h1);       // -K0*H1
  I_KH[1][0] = -(K[1] * h0);         // -K1*H0
  I_KH[1][1] = 1.0f - (K[1] * h1);   // 1 - K1*H1

  // Hitung (I-KH) * P_pred sebagai intermediate result
  float Temp[2][2];
  Temp[0][0] = I_KH[0][0] * P_pred[0][0] + I_KH[0][1] * P_pred[1][0];
  Temp[0][1] = I_KH[0][0] * P_pred[0][1] + I_KH[0][1] * P_pred[1][1];
  Temp[1][0] = I_KH[1][0] * P_pred[0][0] + I_KH[1][1] * P_pred[1][0];
  Temp[1][1] = I_KH[1][0] * P_pred[0][1] + I_KH[1][1] * P_pred[1][1];

  // Hitung P_baru = Temp * (I-KH)^T + K*R*K^T (Joseph Form lengkap)
  ekf_P[0][0] = Temp[0][0] * I_KH[0][0] + Temp[0][1] * I_KH[0][1] + (K0_eff * K0_eff * R_dynamic);
  ekf_P[0][1] = Temp[0][0] * I_KH[1][0] + Temp[0][1] * I_KH[1][1] + (K0_eff * K[1] * R_dynamic);
  ekf_P[1][0] = Temp[1][0] * I_KH[0][0] + Temp[1][1] * I_KH[0][1] + (K[1] * K0_eff * R_dynamic);
  ekf_P[1][1] = Temp[1][0] * I_KH[1][0] + Temp[1][1] * I_KH[1][1] + (K[1] * K[1] * R_dynamic);

  // Paksa matriks P agar tetap SIMETRIS: rata-rata P[0][1] dan P[1][0]
  // (Dalam teori identik, tapi akumulasi floating point bisa membuatnya sedikit berbeda)
  ekf_P[0][1] = (ekf_P[0][1] + ekf_P[1][0]) * 0.5f;
  ekf_P[1][0] = ekf_P[0][1];
  // Pastikan diagonal selalu positif (minimum 1e-10) agar tidak menjadi negatif akibat rounding error
  ekf_P[0][0] = max(ekf_P[0][0], 1e-10f);
  ekf_P[1][1] = max(ekf_P[1][1], 1e-10f);

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
  Serial.printf("│ 4. Error (Inov) : V_meas - V_pred = %+.4f V\n", innov);
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

// publishRelayState: Mem-publish status ke-4 relay ke topik MQTT.
// Fungsi ini dipanggil setiap kali ada perubahan status relay,
// agar dashboard (HP/PC) bisa menampilkan status terkini.
// Format JSON yang dikirim: {"relay_1": "ON", "relay_2": "OFF", ...}
void publishRelayState()
{
  if (!mqtt.connected()) // Jangan coba publish jika tidak terhubung ke broker
    return;
  JsonDocument doc;                              // Buat dokumen JSON kosong
  doc["relay_1"] = relayState[0] ? "ON" : "OFF"; // Isi data relay
  doc["relay_2"] = relayState[1] ? "ON" : "OFF";
  doc["relay_3"] = relayState[2] ? "ON" : "OFF";
  doc["relay_4"] = relayState[3] ? "ON" : "OFF";

  char buffer[200];           // Buffer array of char untuk menampung string JSON
  serializeJson(doc, buffer); // Konversi objek JSON menjadi string teks
  String topic = String(mqtt_prefix) + "/state/relays"; // Topik tujuan
  mqtt.publish(topic.c_str(), buffer, false); // Kirim! (false = tidak retained)
}

// publishComputedData: Mem-publish hasil kalkulasi ESP32 (output EKF) ke MQTT.
// Ini adalah data yang DI-HITUNG oleh ESP32, berbeda dengan data mentah dari Jikong.
// Data ini dapat dibaca oleh dashboard atau sistem lain yang subscribe ke topik ini.
// Berisi: SoC dari CC, SoC dari EKF, tegangan prediksi, rata-rata sel, delta waktu.
void publishComputedData(float dt)
{
  if (!mqtt.connected())
    return;
  JsonDocument doc;
  doc["soc_cc"]     = soc_cc * 100.0;        // SoC Coulomb Counting dalam persen (%)
  doc["soc_ekf"]    = ekf_x[0] * 100.0;     // SoC Extended Kalman Filter dalam persen (%)
  doc["v_pred"]     = v_pred_last;           // Tegangan yang diprediksi EKF (Volt)
  doc["avg_cell_v"] = bmsData.avg_cell_v;    // Rata-rata tegangan per sel (Volt)
  doc["dt"]         = dt;                    // Delta time siklus ini (detik)

  char buffer[200];
  serializeJson(doc, buffer);
  String topic = String(mqtt_prefix) + "/data/calc"; // Topik khusus untuk data kalkulasi
  mqtt.publish(topic.c_str(), buffer, false);
}

// setRelay: Fungsi untuk mengubah status relay secara AMAN.
// Tidak langsung memanipulasi pin; hanya mengubah jika ada perubahan (menghindari write berulang).
// Setelah berubah, set flag agar statusnya dikirim via MQTT (tidak langsung dari sini).
//
// Parameter:
//   index : Nomor relay (0=relay1, 1=relay2, 2=relay3, 3=relay4)
//   state : true = Aktifkan relay | false = Matikan relay
void setRelay(int index, bool state)
{
  if (relayState[index] != state) // Hanya bertindak jika ada PERUBAHAN status
  {
    relayState[index] = state;
    // ACTIVE LOW: LOW  = relay aktif (arus mengalir), HIGH = relay mati (arus terputus)
    digitalWrite(RELAY_PINS[index], state ? LOW : HIGH);
    // Set flag agar status baru dipublish via MQTT pada siklus loop berikutnya (saat buffer aman)
    flag_publish_relay = true;
  }
}

// =========================================================
// 8. MQTT CALLBACK (FUNGSI PENERIMA PESAN)
// =========================================================
// mqttCallback: Fungsi ini dipanggil OTOMATIS oleh library PubSubClient setiap kali
// ada pesan MQTT yang masuk dari broker (untuk topik yang sudah kita subscribe).
//
// PENTING: Jangan lakukan operasi lambat (kalkulasi berat, Firebase) DI SINI!
// Cukup terima data, simpan ke variabel global, lalu set flag.
// Eksekusi berat dilakukan di TaskNetwork loop agar tidak memblokir buffer MQTT.
//
// Parameter:
//   topic   : Nama topik MQTT dari pesan yang masuk (string C)
//   payload : Isi pesan (array byte, belum ada null-terminator)
//   length  : Panjang payload dalam byte
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

  // --- TERIMA DATA INISIALISASI SOC DARI JIKONG ---
  String socInitTopic = String(mqtt_prefix) + "/data/soc_bawaan";
  if (topicStr == socInitTopic)
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (!error && !is_soc_initialized)
    {
      float soc_jk = doc["soc_jk"] | -1.0;
      if (soc_jk >= 0.0)
      {
        // Set nilai CC dan EKF sesuai dengan SOC asli bawaan Jikong (dibagi 100 agar skala 0.0 - 1.0)
        soc_cc = soc_jk / 100.0;
        ekf_x[0] = soc_cc;
        ekf_x[1] = 0.0;
        ekf_P[0][0] = 0.01f;
        ekf_P[0][1] = 0.0f;
        ekf_P[1][0] = 0.0f;
        ekf_P[1][1] = 0.001f;
        is_soc_initialized = true;
        Serial.println("\n=================================================");
        Serial.printf("[INIT] AUTO-CALIBRATION BERHASIL!\n");
        Serial.printf("SOC Awal EKF & CC di-set ke: %.1f %% dari Jikong\n", soc_jk);
        Serial.println("=================================================");
      }
    }
    return;
  }

  // --- TERIMA DATA UTAMA BMS (TEGANGAN, ARUS, SUHU, SEL) DARI JIKONG ---
  // Topik ini adalah "jantung" sistem. Setiap data yang masuk di sini akan
  // memicu satu siklus kalkulasi EKF.
  if (topicStr == String(mqtt_prefix) + "/data/main")
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg); // Parse string JSON menjadi objek

    if (!error) // Hanya proses jika JSON valid (tidak ada error parsing)
    {
      // Simpan data utama ke struct bmsData
      bmsData.voltage   = doc["voltage"]   | 0.0; // Tegangan total pack baterai (Volt)
      bmsData.bat_temp1 = doc["bat_temp1"] | 0.0; // Suhu sensor baterai 1 (°C)
      bmsData.bat_temp2 = doc["bat_temp2"] | 0.0; // Suhu sensor baterai 2 (°C)
      bmsData.mos_temp  = doc["mos_temp"]  | 0.0; // Suhu MOSFET pada BMS Jikong (°C)
      bmsData.power     = doc["power"]     | 0.0; // Daya sesaat (Watt = Volt * Ampere)
      bmsData.current   = doc["current"]   | 0.0; // Arus baterai (Ampere)

      // Variabel bantu untuk mencari statistik tegangan sel secara real-time
      float sum_cells = 0;  // Akumulasi jumlah tegangan semua sel
      float max_v = 0.0;    // Tegangan sel tertinggi
      float min_v = 5.0;    // Tegangan sel terendah (diinisialisasi tinggi agar mudah dicari nilai min)

      JsonArray cells    = doc["cells_v"];  // Array JSON berisi tegangan 8 sel (Volt)
      JsonArray wire_res = doc["wire_res"]; // Array JSON berisi resistansi kabel 8 sel (mΩ)

      // Iterasi melalui semua 8 sel baterai
      for (int i = 0; i < 8; i++)
      {
        float cv = cells[i] | 0.0;      // Baca tegangan sel ke-i (default 0 jika tidak ada)
        bmsData.cells_v[i]  = cv;       // Simpan tegangan sel
        bmsData.wire_res[i] = wire_res[i] | 0.0; // Simpan resistansi kabel sel

        sum_cells += cv;                  // Akumulasi untuk hitung rata-rata
        if (cv > max_v) max_v = cv;       // Update max jika lebih tinggi
        if (cv < min_v && cv > 0.1)       // Update min jika lebih rendah (abaikan sel 0V = tidak terbaca)
          min_v = cv;
      }

      // Hitung statistik akhir dari data 8 sel
      bmsData.avg_cell_v = sum_cells / 8.0; // Rata-rata tegangan per sel (dipakai sebagai V_meas di EKF)
      bmsData.max_cell_v = max_v;           // Tegangan sel tertinggi
      bmsData.min_cell_v = min_v;           // Tegangan sel terendah
      bmsData.delta_v    = max_v - min_v;   // Delta-V = selisih tertinggi vs terendah (indikator imbalance)

      // PENTING: Jangan panggil runEKFStep() langsung di sini!
      // Set flag saja, biarkan TaskNetwork loop yang mengeksekusinya.
      // Ini untuk mencegah watchdog timeout dan buffer overflow MQTT.
      flag_run_ekf = true;
    }
  }
}

// reconnectMQTT: Mencoba (re)koneksi ke broker MQTT.
// Dipanggil jika koneksi MQTT terputus (dicek tiap 5 detik di loop TaskNetwork).
void reconnectMQTT()
{
  // Buat Client ID yang UNIK agar tidak bertabrakan dengan device lain di broker publik.
  // Kombinasi: prefix tetap + 4 byte bawah MAC Address + angka acak
  String clientId = "espbms-" + String((uint32_t)ESP.getEfuseMac(), HEX) + String(random(0xffff), HEX);

  if (mqtt.connect(clientId.c_str())) // Coba koneksi ke broker
  {
    Serial.println("\n[MQTT] Terhubung ke Broker!");

    // Subscribe ke semua topik yang perlu kita dengarkan:
    // 1. Topik perintah relay (4 relay, 1 topik per relay)
    for (int i = 1; i <= 4; i++)
    {
      mqtt.subscribe((String(mqtt_prefix) + "/switch/relay_" + String(i) + "/command").c_str());
    }
    // 2. Topik perintah ganti halaman OLED dari dashboard
    mqtt.subscribe((String(mqtt_prefix) + "/display/page/command").c_str());
    // 3. Topik data utama BMS dari Jikong (tegangan, arus, sel, suhu)
    mqtt.subscribe((String(mqtt_prefix) + "/data/main").c_str());
    // 4. Topik inisialisasi SoC pertama kali dari Jikong
    mqtt.subscribe((String(mqtt_prefix) + "/data/soc_bawaan").c_str());

    // Setelah terhubung, langsung publish status relay terkini agar dashboard up-to-date
    flag_publish_relay = true;
  }
  else
  {
    // Jika gagal, cetak kode error MQTT ke Serial Monitor untuk debugging
    // Kode error: -1=MQTT_CONNECTION_REFUSED, -2=MQTT_CONNECTION_TIMEOUT, dst.
    Serial.print("[MQTT] Gagal konek, rc=");
    Serial.println(mqtt.state());
  }
}

// =========================================================
// 9. TUGAS CORE 0: JARINGAN (MQTT & FIREBASE)
// =========================================================
// TaskNetwork: Fungsi ini berjalan sebagai FreeRTOS Task di CORE 0 ESP32.
// ESP32 memiliki 2 core (dual-core):
//   - CORE 0: Menjalankan TaskNetwork (jaringan, WiFi, MQTT, Firebase, EKF)
//   - CORE 1: Menjalankan loop() bawaan Arduino (layar OLED, tombol, sensor AHT)
// Pembagian ini memastikan tampilan OLED tidak lag meskipun jaringan sedang sibuk.
void TaskNetwork(void *pvParameters)
{
  wm.setConfigPortalTimeout(180); // Portal WiFi akan tertutup otomatis setelah 3 menit jika didiamkan
  if (!wm.autoConnect("esp-setup", "12345679"))
  {
    Serial.println("[WIFI] Gagal connect dan timeout hit, restart ESP32...");
    delay(3000);
    ESP.restart();
  }

  // --- SINKRONISASI WAKTU INTERNET (NTP) UNTUK TIMESTAMP FIREBASE ---
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // Waktu Indonesia Barat (UTC+7)
  Serial.println("[NTP] Sinkronisasi waktu internet...");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo, 5000))
  {
    Serial.println("[NTP] Menunggu waktu sinkron...");
  }
  Serial.println("[NTP] Waktu berhasil disinkronisasi!");

  // --- KONFIGURASI FIREBASE ---
  config.api_key = API_KEY;
  config.database_url = FIREBASE_URL;

  // ====================================================================================
  // SOLUSI ERROR CONFIGURATION_NOT_FOUND / INVALID_EMAIL
  // Karena Rules Realtime Database di console Anda sudah di-set ke TRUE (Publik),
  // kita menginstruksikan library untuk melakukan Bypass/Test Mode agar tidak
  // perlu melakukan pendaftaran Email/Anonymous authentication yang membuat error.
  // ====================================================================================
  config.signer.test_mode = true; // Melewati enkripsi otentikasi login email

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

  // Membesarkan Buffer dan Waktu Tunggu MQTT agar kebal lag (120 Detik)
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(120);
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
          // mqtt.loop() WAJIB dipanggil secara rutin!
          // Fungsinya: menerima pesan masuk dari broker dan memanggil mqttCallback.
          // Tanpa ini, ESP32 tidak akan pernah "mendengar" pesan MQTT yang datang.
          mqtt.loop();

          // === EKSEKUSI EKF & PENGUKURAN PERFORMA ===
          // Flag ini di-set true oleh mqttCallback saat data '/data/main' baru diterima.
          if (flag_run_ekf)
          {
            flag_run_ekf = false; // Reset flag agar tidak dieksekusi dua kali

            // Guard: EKF hanya boleh berjalan setelah nilai SoC awal dari Jikong diketahui.
            // Ini memastikan EKF dimulai dari kondisi yang benar, bukan tebakan awal yang mungkin salah.
            if (!is_soc_initialized)
            {
              Serial.println("[WAIT] Menunggu inisialisasi SOC dari Jikong BMS...");
              continue; // Lewati siklus ini, tunggu sampai SoC terima dari Jikong
            }

            unsigned long now = millis(); // Waktu saat ini dalam milidetik
            float dt = 1.0;              // Default delta time = 1 detik (fallback)

            // --- KALKULASI DELTA TIME (dt) AKTUAL ---
            // Mengapa tidak pakai dt=1.0 saja? Karena network bisa lag!
            // Jika pesan MQTT terlambat 2 detik, tapi kita hitung dt=1 detik,
            // maka Coulomb Counting akan salah hitung (kehilangan 1 detik data arus).
            // Solusi: Ukur waktu nyata antar pesan.
            if (!is_first_run && last_mqtt_time > 0)
            {
              dt = (now - last_mqtt_time) / 1000.0; // Konversi milidetik ke detik
            }
            is_first_run = false;  // Tandai bahwa ini bukan lagi iterasi pertama
            last_mqtt_time = now;  // Simpan waktu sekarang untuk kalkulasi dt berikutnya
            dt_last = dt;          // Simpan untuk ditampilkan di OLED halaman 4

            // --- DETEKSI STATUS REST (BATERAI ISTIRAHAT) ---
            // Jika arus secara terus-menerus mendekati nol, baterai dianggap "istirahat".
            // Status ini penting untuk EKF: saat istirahat, V_meas = OCV sejati → koreksi lebih akurat.
            if (fabsf(bmsData.current) < REST_CURRENT_THRESH)
            {
              rest_counter_s += (int)dt;              // Akumulasi berapa detik arus < threshold
              if (rest_counter_s >= REST_SETTLE_S)    // Jika sudah > 30 detik...
                in_confirmed_rest = true;             // ...maka KONFIRMASI: baterai istirahat!
            }
            else
            {
              rest_counter_s = 0;         // Reset penghitung karena arus terdeteksi kembali
              in_confirmed_rest = false;  // Status istirahat dibatalkan
            }

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

            // Kirim EKF pertama kali langsung ke realtime dan history Firebase
            if (!is_first_ekf_sent)
            {
              is_first_ekf_sent = true;
              flag_send_firebase = true;
              flag_send_history = true;
              last_firebase_time = millis();
              last_firebase_history_time = millis();
              Serial.println("[FIREBASE] Menandai pengiriman EKF pertama kali ke Realtime & History.");
            }

            // LOGIKA PROTEKSI: Putus daya darurat jika SOC < 20%
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

          // === LOGIKA DELAY FIREBASE 60 DETIK (REALTIME) ===
          if (millis() - last_firebase_time >= FIREBASE_INTERVAL)
          {
            last_firebase_time = millis();
            flag_send_firebase = true;
          }

          // === LOGIKA DELAY FIREBASE 15 MENIT (HISTORY) ===
          if (millis() - last_firebase_history_time >= FIREBASE_HISTORY_INTERVAL)
          {
            last_firebase_history_time = millis();
            flag_send_history = true;
          }

          // === PROSES KIRIM KE FIREBASE ===
          if ((flag_send_firebase || flag_send_history) && signupOK && Firebase.ready())
          {
            bool sendRealtime = flag_send_firebase;
            bool sendHistory = flag_send_history;
            flag_send_firebase = false;
            flag_send_history = false;

            // Mengambil waktu internet
            struct tm timeinfo;
            if (!getLocalTime(&timeinfo))
            {
              Serial.println("[FIREBASE] Gagal dapat waktu NTP, lewati pengiriman.");
              continue;
            }

            // Membuat nama folder berdasarkan Tanggal
            char datePath[30];
            strftime(datePath, sizeof(datePath), "/bms_history/%Y-%m-%d", &timeinfo);

            // Menyimpan jam spesifik pada baris data
            char timeString[20];
            strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);

            FirebaseJson json;
            json.set("timestamp", timeString);

            // Data Utama BMS & EKF
            json.set("voltage", bmsData.voltage);
            json.set("current", bmsData.current);
            json.set("power", bmsData.power);
            json.set("soc_ekf", ekf_x[0] * 100.0);
            json.set("soc_cc", soc_cc * 100.0);

            // Data Diagnostik Sel (Battery Health)
            json.set("avg_cell_v", bmsData.avg_cell_v);
            json.set("max_cell_v", bmsData.max_cell_v);
            json.set("min_cell_v", bmsData.min_cell_v);
            json.set("delta_v", bmsData.delta_v);

            // Data Suhu (Safety & Environment)
            json.set("mos_temp", bmsData.mos_temp);
            json.set("bat_temp1", bmsData.bat_temp1);
            json.set("bat_temp2", bmsData.bat_temp2);
            json.set("room_temp", room_temp);
            json.set("room_hum", room_hum);

            // Aksi 1: Menulis status terkini ke Dashboard Realtime
            if (sendRealtime)
            {
              Firebase.RTDB.setJSON(&fbdo, "/bms_realtime", &json);
            }

            // Aksi 2: Merekam jejak data (akumulatif/tidak menimpa data lama)
            if (sendHistory)
            {
              if (Firebase.RTDB.pushJSON(&fbdo, datePath, &json))
              {
                Serial.printf("[FIREBASE] Data History tersimpan ke %s\n", datePath);
              }
              else
              {
                Serial.printf("[FIREBASE] Gagal simpan History: %s\n", fbdo.errorReason().c_str());
              }
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
// Bagian ini berjalan di CORE 1 (melalui loop() Arduino).
// Bertanggung jawab atas antarmuka pengguna (UI) di layar OLED:
//   - Menggambar 7 halaman informasi yang berbeda
//   - Membaca tombol untuk navigasi halaman
//   - Membaca sensor suhu AHT10
//   - Menampilkan progress bar saat OTA update

// bacaSensorAHT: Membaca data dari sensor suhu & kelembapan AHT10 (jika ada).
// Dipanggil setiap 500ms dari loop() utama.
void bacaSensorAHT()
{
  if (aht_status) // Hanya baca jika sensor berhasil dideteksi saat setup()
  {
    sensors_event_t humidity, temp; // Objek untuk menampung hasil pembacaan sensor
    aht.getEvent(&humidity, &temp); // Ambil data dari sensor (mengisi objek di atas)
    room_temp = temp.temperature;            // Simpan suhu dalam °Celsius
    room_hum  = humidity.relative_humidity;  // Simpan kelembapan dalam %RH
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
    display.printf("SOC  : %.0f%%", ekf_x[0] * 100);
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
    display.printf("BMS Temp MOS : %.1f C", bmsData.mos_temp);
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
    // display.setCursor(0, 26);
    // display.printf("Room T : %.1f C", room_temp);
    // display.setCursor(0, 36);
    // display.printf("Room H : %.1f %%", room_hum);
    // display.setCursor(0, 46);
    // display.print(aht_status ? "AHT10  : OK" : "AHT10  : ERROR");
  }
  else if (currentPage == 4)
  {
    // Halaman 4: Network & System
    display.setCursor(0, 0);
    display.print("====== NETWORK ======");
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(0, 16);
    display.printf("WiFi : %s", WiFi.status() == WL_CONNECTED ? WiFi.SSID().c_str() : "Disconnected");
    display.setCursor(0, 26);
    display.print("IP   :");
    display.print(WiFi.localIP());
    // display.setCursor(0, 36);
    // display.printf("EKF dt: %.2f sec", dt_last);
    // display.setCursor(0, 46);
    // display.print("OTA  : Ready");
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

// cekTombolSmart: Membaca tombol BOOT dengan logika debounce dan long press.
// Karena hanya ada 1 tombol, dibedakan 2 fungsi:
//   - SHORT PRESS (< 1 detik) : Ganti halaman OLED ke halaman berikutnya
//   - LONG PRESS  (> 5 detik) : Buka/tutup portal konfigurasi WiFi
//
// 'static' pada variabel lokal = nilainya BERTAHAN antar pemanggilan fungsi (seperti variabel global tapi lokal)
void cekTombolSmart()
{
  static unsigned long waktuTekan = 0;       // Waktu tombol pertama kali ditekan (milidetik)
  static bool sedangDitekan = false;         // Flag apakah tombol saat ini dalam kondisi ditekan
  static bool longPressTriggered = false;    // Flag agar long press hanya terpicu SEKALI per tekan

  if (digitalRead(BUTTON_PIN) == LOW) // LOW = tombol ditekan (INPUT_PULLUP: aktif low)
  {
    if (!sedangDitekan) // Jika ini adalah AWAL tekan (transisi dari HIGH ke LOW)
    {
      waktuTekan = millis();       // Catat waktu mulai tekan
      sedangDitekan = true;
      longPressTriggered = false;
    }
    else if ((millis() - waktuTekan > 5000) && !longPressTriggered) // Sudah > 5 detik & belum terpicu
    {
      longPressTriggered = true; // Tandai agar tidak terpicu berulang
      Serial.println("[BUTTON] Long Press Detected -> Toggle WiFi Portal");
      // Kirim sinyal ke Core 0 untuk buka/tutup portal WiFi
      if (!portalActive)
        requestPortalOpen = true;  // Minta portal WiFi dibuka
      else
        requestPortalClose = true; // Minta portal WiFi ditutup
    }
  }
  else // Tombol DILEPAS (transisi dari LOW ke HIGH)
  {
    if (sedangDitekan) // Hanya proses jika sebelumnya memang ditekan
    {
      if (!longPressTriggered && (millis() - waktuTekan < 1000)) // Short press = dilepas < 1 detik
      {
        // Hanya ganti halaman jika tidak sedang dalam mode portal/OTA
        if (!portalActive && !ota_updating)
        {
          currentPage = currentPage + 1;    // Maju ke halaman berikutnya
          if (currentPage > MAX_PAGES)      // Jika sudah di halaman terakhir, kembali ke halaman 1
            currentPage = 1;
          Serial.printf("\n[BUTTON] Short Press -> Change Page %d\n", currentPage);
          updateLayar();         // Langsung update tampilan OLED
          printOledToSerial();   // Cetak juga ke Serial Monitor untuk debug
        }
        else
        {
          Serial.println("\n[BUTTON] Short Press ignored because it is in Setup/OTA Mode");
        }
      }
      sedangDitekan = false; // Reset status tekan
    }
  }
}

// setup: Fungsi inisialisasi Arduino yang dipanggil SATU KALI saat ESP32 pertama menyala.
// Urutan inisialisasi penting: hardware harus siap sebelum fungsi lain berjalan.
void setup()
{
  Serial.begin(115200); // Inisialisasi komunikasi Serial dengan baud rate 115200 bps untuk debug

  // Konfigurasi pin tombol: INPUT_PULLUP = aktifkan resistor pull-up internal.
  // Tanpa pull-up, pin akan floating (tidak stabil antara HIGH/LOW).
  // Dengan pull-up: pin = HIGH saat tombol dilepas, LOW saat tombol ditekan.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Inisialisasi 4 pin relay sebagai OUTPUT dan set kondisi awal = HIGH.
  // HIGH = Relay MATI (karena ACTIVE LOW). Penting: pastikan relay OFF dulu saat boot
  // agar tidak ada beban yang menyala sebelum sistem siap.
  for (int i = 0; i < 4; i++)
  {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH); // ACTIVE LOW: HIGH = Relay OFF
  }

  // Inisialisasi bus I2C dengan kecepatan 100kHz (Standard Mode).
  // I2C digunakan oleh OLED dan sensor AHT10 secara berbagi bus yang sama.
  Wire.begin();
  Wire.setClock(100000); // Frekuensi clock I2C: 100kHz
  delay(100);            // Beri waktu komponen I2C untuk siap

  // Inisialisasi layar OLED SSD1306:
  // SSD1306_SWITCHCAPVCC = Gunakan pompa tegangan internal (tidak perlu VCC eksternal untuk layar)
  // 0x3C = Alamat I2C default modul OLED SSD1306 (bisa juga 0x3D tergantung hardware)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED Failed!"); // Jika OLED tidak ditemukan, berhenti total
    for (;;)                        // Loop tanpa henti (infinite loop) = ESP32 freeze
      ;
  }

  // Coba inisialisasi sensor AHT10. Jika berhasil, set flag untuk aktifkan pembacaan rutin.
  if (aht.begin())
    aht_status = true;

  // Buat FreeRTOS Task untuk TaskNetwork dan jalankan di CORE 0.
  // Stack size 15000 bytes (besar karena Firebase membutuhkan banyak memori untuk buffer HTTP).
  // Priority = 1 (default). NULL = tidak ada parameter. NULL = tidak perlu handle task.
  xTaskCreatePinnedToCore(
    TaskNetwork, // Fungsi yang akan dijalankan sebagai task
    "TaskNet",   // Nama task (untuk debugging dengan monitor FreeRTOS)
    15000,       // Ukuran stack (bytes). Firebase butuh minimal ~10KB!
    NULL,        // Parameter yang dikirim ke task (tidak dipakai)
    1,           // Prioritas task (1 = normal)
    NULL,        // Handle task (tidak dipakai)
    0            // Nomor Core: 0 = jalankan di Core 0
  );
}

// loop: Fungsi utama Arduino yang berjalan TERUS-MENERUS di CORE 1.
// Bertanggung jawab atas UI (OLED) dan pembacaan tombol.
// CATATAN: loop() berjalan di Core 1, sementara TaskNetwork berjalan di Core 0.
void loop()
{
  static unsigned long waktuTerakhir = 0;  // Waktu terakhir OLED diperbarui
  static int last_drawn_percent = -1;      // Progress OTA terakhir yang digambar (untuk deteksi perubahan)

  // --- PRIORITAS: Mode OTA Update ---
  // Jika sedang OTA, HENTIKAN semua fungsi normal dan tampilkan hanya progress bar OTA.
  // Ini agar OTA tidak terganggu dan pengguna tahu update sedang berjalan.
  if (ota_updating)
  {
    // Update layar OTA hanya jika persentase berubah (menghindari redraw yang tidak perlu)
    if (ota_progress_percent != last_drawn_percent)
    {
      last_drawn_percent = ota_progress_percent;
      drawOTAScreen(ota_progress_percent); // Gambar tampilan progress bar
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // Yield ke sistem untuk 20ms
    return; // Keluar dari loop() agar tidak menjalankan kode di bawah ini
  }

  // --- Pembaruan OLED & Sensor: Dilakukan setiap 500ms ---
  // Tidak perlu lebih sering karena data BMS hanya diperbarui ~1 detik sekali via MQTT.
  if (millis() - waktuTerakhir >= 500)
  {
    waktuTerakhir = millis();
    bacaSensorAHT(); // Baca sensor suhu & kelembapan ruangan
    updateLayar();   // Gambar ulang halaman OLED yang sedang aktif
  }

  // Cek status tombol setiap iterasi loop (tidak dibatasi timer)
  // agar responsif dan tidak melewatkan short press yang singkat
  cekTombolSmart();

  // Yield selama 10ms: Memberikan waktu kepada sistem operasi (FreeRTOS)
  // untuk menjalankan task lain yang mungkin butuh giliran di Core 1.
  // Tanpa ini, loop bisa berjalan terlalu cepat dan tidak efisien.
  vTaskDelay(10 / portTICK_PERIOD_MS);
}