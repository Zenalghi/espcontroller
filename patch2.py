import os

filepath = r'c:\Users\zenaj\Documents\Courses\Sms 8\espcontroller\src\main.cpp'
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# Chunk 5: is_soc_initialized ekf_P matrix update
old_soc_init = '''      if (soc_jk >= 0.0)
      {
        // Set nilai CC dan EKF sesuai dengan SOC asli bawaan Jikong (dibagi 100 agar skala 0.0 - 1.0)
        soc_cc = soc_jk / 100.0;
        ekf_x[0] = soc_cc;
        is_soc_initialized = true;'''
new_soc_init = '''      if (soc_jk >= 0.0)
      {
        // Set nilai CC dan EKF sesuai dengan SOC asli bawaan Jikong (dibagi 100 agar skala 0.0 - 1.0)
        soc_cc = soc_jk / 100.0;
        ekf_x[0] = soc_cc;
        ekf_x[1] = 0.0;
        ekf_P[0][0] = 0.01f;
        ekf_P[0][1] = 0.0f;
        ekf_P[1][0] = 0.0f;
        ekf_P[1][1] = 0.001f;
        is_soc_initialized = true;'''
content = content.replace(old_soc_init, new_soc_init)

# Chunk 6: TaskNetwork rest detection
old_task = '''            is_first_run = false;
            last_mqtt_time = now;
            dt_last = dt;

            // 1. Pengukuran Beban Prosesor (Waktu Eksekusi EKF)'''
new_task = '''            is_first_run = false;
            last_mqtt_time = now;
            dt_last = dt;

            // --- Rest detection: count consecutive seconds of near-zero current ---
            if (fabsf(bmsData.current) < REST_CURRENT_THRESH)
            {
              rest_counter_s += (int)dt;
              if (rest_counter_s >= REST_SETTLE_S)
                in_confirmed_rest = true;
            }
            else
            {
              rest_counter_s = 0;
              in_confirmed_rest = false;
            }

            // 1. Pengukuran Beban Prosesor (Waktu Eksekusi EKF)'''
content = content.replace(old_task, new_task)

# Chunk 7: runEKFStep
old_ekf = '''void runEKFStep(float I_meas, float V_meas, float dt)
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

  // tau (\u03C4): Konstanta waktu untuk komponen RC. Menggambarkan seberapa cepat tegangan polarisasi terbentuk/hilang.
  float tau = max(R1 * C1, 0.000001f);

  // === TAHAP 1: PREDIKSI (A PRIORI / TEBAKAN AWAL) ============================

  // soc_pred: EKF menebak nilai SoC berikutnya menggunakan rumus Coulomb Counting.
  float soc_pred = constrain(soc_prev - (I_meas * dt / Q_COULOMB), 0.0, 1.0);
  // alpha & vc1_pred: EKF menebak nilai tegangan kapasitor (Vc1) berikutnya berdasarkan laju peluruhan RC.
  float alpha = (dt > 0) ? exp(-dt / tau) : 1.0f;
  float vc1_pred = (alpha * vc1_prev) + (R1 * (1.0f - alpha) * I_meas);

  // Memasukkan sementara tebakan awal ke dalam State Vector.
  ekf_x[0] = soc_pred;
  ekf_x[1] = vc1_pred;

  // P_pred: Prediksi Matriks Keraguan. Keraguan sistem akan SELALU BERTAMBAH di tahap tebakan
  // karena masuknya nilai 'Q_NOISE' (seiring berjalannya waktu, sistem makin tidak yakin tebakannya benar).
  float P_pred[2][2];
  P_pred[0][0] = ekf_P[0][0] + Q_NOISE_00;
  P_pred[0][1] = ekf_P[0][1] * alpha;
  P_pred[1][0] = ekf_P[1][0] * alpha;
  P_pred[1][1] = (alpha * alpha * ekf_P[1][1]) + Q_NOISE_11;

  // R_eff: Logika Noise Adaptif (Menentukan nilai sensor mana yang sedang bisa dipercaya)
  float R_eff;
  if (fabsf(I_meas) < 0.05f)
  {
    R_eff = R_NOISE_REST; // Arus nyaris 0, V = OCV asli, sensor tegangan sangat dipercaya.
  }
  else if (I_meas < 0.0f)
  {
    R_eff = R_NOISE_CHARGE; // Arus masuk, tegangan semu naik, sensor kurang dipercaya.
  }
  else
  {
    R_eff = R_NOISE_DISCHARGE; // Arus keluar, lumayan bisa diandalkan.
  }

  // === TAHAP 2: KOREKSI (A POSTERIORI / PERBAIKAN TEBAKAN) ==========================

  // OCV_pred: Mencari nilai Open Circuit Voltage berdasar tebakan SoC awal (dari lookup table OCV).
  float OCV_pred = interpolate1D(soc_pred, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  // dOCV_dSOC: Meminta nilai kemiringan kurva (gradient) saat ini.
  float dOCV_dSOC = get_dOCV_dSOC(soc_pred);

  // V_pred: EKF menebak TEGANGAN FISIK sensor.
  // Rumus ECM = Tegangan murni (OCV) dikurangi drop kapasitor (Vc1) dikurangi drop resistor ohmik (I*R0)
  float V_pred = OCV_pred - vc1_pred - (I_meas * R0);
  v_pred_last = V_pred;

  // Matriks H (h0 & h1): Matriks Jacobian observasi, menjembatani perhitungan State (SoC) menjadi Prediksi Sensor (V_pred).
  float h0 = dOCV_dSOC;
  float h1 = -1.0f;

  // S (Covariance Inovasi): Mengukur total semua keraguan (Keraguan Tebakan + Noise Sensor R_eff).
  float S = (h0 * h0 * P_pred[0][0]) + (h0 * h1 * P_pred[0][1]) +
            (h1 * h0 * P_pred[1][0]) + (h1 * h1 * P_pred[1][1]) + R_eff;

  // K (Kalman Gain): INTI DARI ALGORITMA EKF.
  // Matriks ini memutuskan "Siapa yang lebih bisa dipercaya? Hitungan Coulomb Counting atau Sensor Tegangan?".
  float K[2];
  K[0] = ((P_pred[0][0] * h0) + (P_pred[0][1] * h1)) / S;
  K[1] = ((P_pred[1][0] * h0) + (P_pred[1][1] * h1)) / S;

  // error (Inovasi Tegangan): Selisih antara Tegangan aktual dari sensor dikurangi Tebakan Tegangan EKF (V_pred).
  float error = V_meas - V_pred;

  // FINAL KOREKSI: EKF memperbaiki SoC tebakan awalnya.
  // Rumus: SoC Baru = SoC Tebakan + (Kalman Gain * Error Tegangan)
  ekf_x[0] = constrain(ekf_x[0] + (K[0] * error), 0.0, 1.0);
  ekf_x[1] = ekf_x[1] + (K[1] * error);

  // I_KH & Temp: Variabel matrix pembantu untuk kalkulasi Joseph Form Update.
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

  // ekf_P (Joseph Form Update): Memperbarui Matriks Keraguan (P).
  // Setelah EKF mengoreksi status menggunakan sensor tegangan, "Keraguannya" (P) akan menurun (menjadi lebih yakin untuk putaran/loop selanjutnya).
  ekf_P[0][0] = Temp[0][0] * I_KH[0][0] + Temp[0][1] * I_KH[0][1] + (K[0] * K[0] * R_eff);
  ekf_P[0][1] = Temp[0][0] * I_KH[1][0] + Temp[0][1] * I_KH[1][1] + (K[0] * K[1] * R_eff);
  ekf_P[1][0] = Temp[1][0] * I_KH[0][0] + Temp[1][1] * I_KH[0][1] + (K[1] * K[0] * R_eff);
  ekf_P[1][1] = Temp[1][0] * I_KH[1][0] + Temp[1][1] * I_KH[1][1] + (K[1] * K[1] * R_eff);

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
  Serial.println("\\n+----------------------------------------------+");
  Serial.println("¦          MATRIKS KALKULASI EKF AKTIF         ¦");
  Serial.println("+----------------------------------------------¦");
  Serial.printf("¦ 1. Input Sensor : V_meas = %.3f V | I = %.2f A\\n", V_meas, I_meas);
  Serial.printf("¦ 2. Open-Loop CC : SOC_CC = %.2f %%\\n", soc_cc * 100);
  Serial.printf("¦ 3. State Pred   : V_pred = %.3f V | SOC = %.2f %%\\n", V_pred, soc_pred * 100);
  Serial.printf("¦ 4. Error (Inov) : V_meas - V_pred = %+.4f V\\n", error);
  Serial.printf("¦ 5. Kalman Gain  : K[0] = %.5f | K[1] = %.5f\\n", K[0], K[1]);
  Serial.printf("¦ 6. Output Final : SOC_EKF Terkoreksi = %.2f %%\\n", ekf_x[0] * 100);
  Serial.println("+----------------------------------------------¦");
  Serial.println("¦   [ UTILISASI MEMORI & BEBAN PROSESOR ]      ¦");
  Serial.printf("¦ - Waktu EKF (Loop sblmnya): %.3f ms\\n", perf_ekf_time_ms);
  Serial.printf("¦ - Beban CPU (Load)        : %.4f %%\\n", cpu_load);
  Serial.printf("¦ - SRAM (Heap) Used        : %.2f %% (%u B)\\n", ram_pct, ram_used);
  Serial.printf("¦ - Flash Memory Used       : %.2f %% (%u B)\\n", flash_pct, flash_used);
  Serial.println("+----------------------------------------------+");
}'''

new_ekf = '''void runEKFStep(float I_meas, float V_meas, float dt)
{
  soc_cc = constrain(soc_cc - (I_meas * dt / Q_COULOMB), 0.0, 1.0);

  float soc_prev = constrain(ekf_x[0], 0.0, 1.0);
  float vc1_prev = ekf_x[1];

  float R0 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r0, LUT_ECM_SIZE), 0.0001f);
  float R1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_r1, LUT_ECM_SIZE), 0.0001f);
  float C1 = max(interpolate1D(soc_prev, lut_soc_ecm, lut_c1, LUT_ECM_SIZE), 1.0f);
  float tau = max(R1 * C1, 0.000001f);

  float soc_pred = constrain(soc_prev - (I_meas * dt / Q_COULOMB), 0.0, 1.0);
  float alpha = (dt > 0) ? expf(-dt / tau) : 1.0f;
  float vc1_pred = (alpha * vc1_prev) + (R1 * (1.0f - alpha) * I_meas);

  ekf_x[0] = soc_pred;
  ekf_x[1] = vc1_pred;

  // Prediksi Covariance P (decoupled)
  float P_pred[2][2];
  P_pred[0][0] = ekf_P[0][0] + Q_NOISE_00;
  P_pred[0][1] = 0.0f;
  P_pred[1][0] = 0.0f;
  P_pred[1][1] = (alpha * alpha * ekf_P[1][1]) + Q_NOISE_11;

  float OCV_pred = interpolate1D(soc_pred, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float dOCV_dSOC = get_dOCV_dSOC(soc_pred);

  float V_pred = OCV_pred - vc1_pred - (I_meas * R0);
  v_pred_last = V_pred;

  // Matriks H (h0 & h1): Matriks Jacobian observasi
  float h0 = fabsf(dOCV_dSOC) + 1e-4f;
  float h1 = -1.0f;

  // Dynamic R: trust measurement proportional to OCV slope steepness
  float R_dynamic;
  if (in_confirmed_rest)
  {
    R_dynamic = R_REST;
  }
  else if (abs(I_meas) < 0.05f)
  {
    R_dynamic = R_BASE / (abs(dOCV_dSOC) + 1e-3f);
  }
  else
  {
    R_dynamic = R_BASE / (abs(dOCV_dSOC) + 1e-4f);
  }

  // Clamp R_dynamic to prevent numerical issues
  R_dynamic = constrain(R_dynamic, 0.0001f, 10.0f);

  float S = (h0 * h0 * P_pred[0][0]) + (h0 * h1 * P_pred[0][1]) +
            (h1 * h0 * P_pred[1][0]) + (h1 * h1 * P_pred[1][1]) + R_dynamic;
  if (S < 1e-9f)
    S = 1e-9f;

  float K[2];
  K[0] = ((P_pred[0][0] * h0) + (P_pred[0][1] * h1)) / S;
  K[1] = ((P_pred[1][0] * h0) + (P_pred[1][1] * h1)) / S;

  float innov = V_meas - V_pred;

  // --- SOFT DEADBAND (1mV) + CORRECTION CAP (10% SoC per step) ---
  float K0_eff = K[0];
  const float DEADBAND = 0.001f; // 1 mV
  const float MAX_CORRECTION = 0.10f; // maks 10% SoC per langkah

  if (fabsf(innov) < DEADBAND)
  {
    K0_eff *= (fabsf(innov) / DEADBAND);
  }

  // Apply correction with cap
  float soc_correction = K0_eff * innov;
  if (soc_correction > MAX_CORRECTION) soc_correction = MAX_CORRECTION;
  if (soc_correction < -MAX_CORRECTION) soc_correction = -MAX_CORRECTION;

  // Koreksi State x = x + correction (capped)
  ekf_x[0] = max(0.0f, min(1.0f, ekf_x[0] + soc_correction));
  ekf_x[1] += K[1] * innov;

  // Cap Vc1
  if (ekf_x[1] > 0.5f)
    ekf_x[1] = 0.5f;
  if (ekf_x[1] < -0.5f)
    ekf_x[1] = -0.5f;

  // Update Covariance P (Joseph Form)
  float I_KH[2][2];
  I_KH[0][0] = 1.0f - (K0_eff * h0);
  I_KH[0][1] = -(K0_eff * h1);
  I_KH[1][0] = -(K[1] * h0);
  I_KH[1][1] = 1.0f - (K[1] * h1);

  float Temp[2][2];
  Temp[0][0] = I_KH[0][0] * P_pred[0][0] + I_KH[0][1] * P_pred[1][0];
  Temp[0][1] = I_KH[0][0] * P_pred[0][1] + I_KH[0][1] * P_pred[1][1];
  Temp[1][0] = I_KH[1][0] * P_pred[0][0] + I_KH[1][1] * P_pred[1][0];
  Temp[1][1] = I_KH[1][0] * P_pred[0][1] + I_KH[1][1] * P_pred[1][1];

  ekf_P[0][0] = Temp[0][0] * I_KH[0][0] + Temp[0][1] * I_KH[0][1] + (K0_eff * K0_eff * R_dynamic);
  ekf_P[0][1] = Temp[0][0] * I_KH[1][0] + Temp[0][1] * I_KH[1][1] + (K0_eff * K[1] * R_dynamic);
  ekf_P[1][0] = Temp[1][0] * I_KH[0][0] + Temp[1][1] * I_KH[0][1] + (K[1] * K0_eff * R_dynamic);
  ekf_P[1][1] = Temp[1][0] * I_KH[1][0] + Temp[1][1] * I_KH[1][1] + (K[1] * K[1] * R_dynamic);

  // Symmetry enforcement & positivity floor
  ekf_P[0][1] = (ekf_P[0][1] + ekf_P[1][0]) * 0.5f;
  ekf_P[1][0] = ekf_P[0][1];
  ekf_P[0][0] = max(ekf_P[0][0], 1e-10f);
  ekf_P[1][1] = max(ekf_P[1][1], 1e-10f);

  uint32_t ram_total = ESP.getHeapSize();
  uint32_t ram_used = ram_total - ESP.getFreeHeap();
  float ram_pct = (ram_used / (float)ram_total) * 100.0;

  uint32_t flash_used = ESP.getSketchSize();
  uint32_t flash_total = flash_used + ESP.getFreeSketchSpace();
  float flash_pct = (flash_used / (float)flash_total) * 100.0;

  float cpu_load = (perf_ekf_time_ms / 1000.0) * 100.0;

  Serial.println("\\n+----------------------------------------------+");
  Serial.println("¦          MATRIKS KALKULASI EKF AKTIF         ¦");
  Serial.println("+----------------------------------------------¦");
  Serial.printf("¦ 1. Input Sensor : V_meas = %.3f V | I = %.2f A\\n", V_meas, I_meas);
  Serial.printf("¦ 2. Open-Loop CC : SOC_CC = %.2f %%\\n", soc_cc * 100);
  Serial.printf("¦ 3. State Pred   : V_pred = %.3f V | SOC = %.2f %%\\n", V_pred, soc_pred * 100);
  Serial.printf("¦ 4. Error (Inov) : V_meas - V_pred = %+.4f V\\n", innov);
  Serial.printf("¦ 5. Kalman Gain  : K[0] = %.5f | K[1] = %.5f\\n", K[0], K[1]);
  Serial.printf("¦ 6. Output Final : SOC_EKF Terkoreksi = %.2f %%\\n", ekf_x[0] * 100);
  Serial.println("+----------------------------------------------¦");
  Serial.println("¦   [ UTILISASI MEMORI & BEBAN PROSESOR ]      ¦");
  Serial.printf("¦ - Waktu EKF (Loop sblmnya): %.3f ms\\n", perf_ekf_time_ms);
  Serial.printf("¦ - Beban CPU (Load)        : %.4f %%\\n", cpu_load);
  Serial.printf("¦ - SRAM (Heap) Used        : %.2f %% (%u B)\\n", ram_pct, ram_used);
  Serial.printf("¦ - Flash Memory Used       : %.2f %% (%u B)\\n", flash_pct, flash_used);
  Serial.println("+----------------------------------------------+");
}'''
content = content.replace(old_ekf, new_ekf)

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)
print('Phase 2 patched successfully.')
