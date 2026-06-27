# Panduan Rumus Matematika (EKF & Coulomb Counting) pada BMS ESP32

File ini merangkum langkah-langkah komputasi dan persamaan matematika (dalam format LaTeX) yang digunakan di dalam kode `main.cpp` untuk mengestimasi SoC (*State of Charge*) baterai secara *real-time*.

---

## 1. Interpolasi Linear (Lookup Table)
Kode ini menggunakan **1D Linear Interpolation** (Fungsi `interpolate1D`) untuk mencari nilai $OCV$, $R_0$, $R_1$, dan $C_1$ pada titik SoC tertentu di antara dua titik data dalam tabel.

Jika kita mencari nilai $y$ untuk nilai $x$ yang berada di antara titik $(x_0, y_0)$ dan $(x_1, y_1)$, rumus yang digunakan adalah:

$$ y = y_0 + \left( \frac{x - x_0}{x_1 - x_0} \right) (y_1 - y_0) $$

---

## 2. Coulomb Counting (Open-Loop Estimation)
Metode penghitungan SoC secara langsung dengan mengakumulasikan arus yang keluar atau masuk. 

$$ SoC_{cc}(t) = SoC_{cc}(t-1) - \frac{I_{meas} \cdot \Delta t}{Q_{total}} $$

**Dimana:**
- $I_{meas}$ = Arus yang terukur (Ampere). Dalam logika kode, positif = discharge, negatif = charge.
- $\Delta t$ = Selisih waktu atau interval sampling (detik).
- $Q_{total}$ = Kapasitas total baterai dalam satuan *Coulomb* (Ampere-detik).

---

## 3. Extended Kalman Filter (EKF) - Metode Closed-Loop

EKF menggunakan pemodelan **Equivalent Circuit Model (ECM) 1-RC Thevenin**. Ada dua *State* (vektor tebakan) yang dijaga oleh EKF:
- $x_0 = SoC$ (Estimasi State of Charge)
- $x_1 = V_{c1}$ (Tegangan pada komponen RC kapasitor polarisasi)

### Tahap 1: Prediksi (A Priori)
Tahap ini menebak status baterai sebelum pengukuran tegangan asli (sensor) dimasukkan.

**a. Prediksi State Vector ($x$)**

$$ SoC_{pred} = SoC_{prev} - \frac{I_{meas} \cdot \Delta t}{Q_{total}} $$

Menghitung efek peluruhan tegangan pada kapasitor ($\tau$ adalah konstanta waktu $R_1 \times C_1$):
$$ \alpha = e^{-\frac{\Delta t}{\tau}} $$

$$ V_{c1, pred} = \left( \alpha \cdot V_{c1, prev} \right) + \left( R_1 \cdot (1 - \alpha) \cdot I_{meas} \right) $$

**b. Prediksi Error Covariance Matrix ($P$)**
Matriks $P$ menggambarkan seberapa **ragu** (uncertainty) sistem terhadap tebakan state-nya. Untuk mengurangi beban CPU ESP32, model ini disederhanakan (*decoupled*).

$$ P_{00, pred} = P_{00, prev} + Q_{noise, 00} $$
$$ P_{11, pred} = (\alpha^2 \cdot P_{11, prev}) + Q_{noise, 11} $$

**Dimana:** $Q_{noise}$ adalah matriks tuning proses (proses *noise*).

---

### Tahap 2: Menghitung Jacobian OCV (H Matrix)
Karena OCV tidak linear terhadap SoC, EKF memerlukan **Jacobian** (turunan pertama) OCV terhadap SoC di titik tersebut untuk melinearisasi persamaan. Kode menggunakan *Central Difference Method* (h = 0.005):

$$ \frac{dOCV}{dSoC} \approx \frac{OCV(SoC + h) - OCV(SoC - h)}{2h} $$

Matriks Pengukuran ($H$) untuk ECM 1-RC:
$$ H = \begin{bmatrix} \left| \frac{dOCV}{dSoC} \right| & -1 \end{bmatrix} $$

---

### Tahap 3: Koreksi (A Posteriori)
Di tahap ini, EKF mengukur seberapa meleset tebakannya dengan membandingkannya terhadap sensor tegangan fisik, lalu mengoreksi kembali tebakannya.

**a. Menebak Tegangan Sensor (Prediksi V)**
$$ V_{pred} = OCV(SoC_{pred}) - V_{c1, pred} - (I_{meas} \cdot R_0) $$

**b. Menghitung Error / Inovasi ($\tilde{y}$)**
$$ \tilde{y} = V_{meas} - V_{pred} $$

**c. Menghitung Covariance Inovasi ($S$)**
$$ S = \left( H \cdot P_{pred} \cdot H^T \right) + R_{dynamic} $$
*(Dimana $R_{dynamic}$ adalah noise sensor yang berubah tergantung tingkat kemiringan kurva).*

**d. Menghitung Kalman Gain ($K$)**
Kalman gain menentukan seberapa besar EKF harus percaya pada tebakan awalnya ($P$) vs percaya pada sensor ($S$).
$$ K = \frac{P_{pred} \cdot H^T}{S} $$

Dalam implementasinya untuk array:
$$ K_0 = \frac{P_{00, pred} \cdot H_0 + P_{01, pred} \cdot H_1}{S} $$
$$ K_1 = \frac{P_{10, pred} \cdot H_0 + P_{11, pred} \cdot H_1}{S} $$

**e. Mengoreksi State Vector ($x$)**
$$ x_{new} = x_{pred} + \left( K \cdot \tilde{y} \right) $$

**f. Mengoreksi Error Covariance Matrix ($P$) - Joseph Form**
Agar matriks $P$ selalu stabil (positif definitif) meski ada *floating-point error* dari mikrokontroler, kode menggunakan pembaruan **Joseph Form**:
$$ I_{KH} = I - K \cdot H $$
$$ P_{new} = \left( I_{KH} \cdot P_{pred} \cdot I_{KH}^T \right) + \left( K \cdot R_{dynamic} \cdot K^T \right) $$

---

## Ringkasan Alur EKF di ESP32:
1. Baca Arus & Tegangan dari Jikong BMS.
2. Prediksi SoC pakai **Coulomb Counting** (*Tahap 1*).
3. Bandingkan **Tegangan Asli** vs **Tegangan Tebakan Matematika** (*Tahap 3a, 3b*).
4. Hitung **Kalman Gain** (*Tahap 3d*).
5. Tarik/Koreksi nilai SoC agar sesuai dengan kurva OCV baterai (*Tahap 3e*).
6. Ulangi di siklus (detik) berikutnya.

---

## 4. Output Final (Nilai SoC yang Dikirim ke Firebase)

Persentase SoC (*State of Charge*) akhir yang dikirimkan ke Firebase dan ditampilkan pada layar OLED merupakan **elemen pertama** dari Vektor Status ($x$) yang telah selesai dikoreksi pada Tahap 3e, yaitu $x_{new, 0}$ (atau $SoC_{new}$).

Karena kalkulasi EKF menghasilkan nilai skala rasio (0.0 hingga 1.0), nilai tersebut dikonversi menjadi persentase dengan rumus:

$$ SoC_{Final, \%} = SoC_{new} \times 100\% $$
