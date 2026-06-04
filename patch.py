import os

filepath = r'c:\Users\zenaj\Documents\Courses\Sms 8\espcontroller\src\main.cpp'
with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# Chunk 1: lut_ocv
old_lut = '''const float lut_ocv[LUT_OCV_SIZE] = {
    2.655, 3.050, 3.194, 3.210, 3.220,
    3.232, 3.245, 3.258, 3.270, 3.282,
    3.285, 3.287, 3.288, 3.289, 3.291,
    3.294, 3.300, 3.310, 3.331, 3.385, 3.537};'''

new_lut = '''const float lut_ocv[LUT_OCV_SIZE] = {
    2.6550, 3.0269, 3.1972, 3.2391, 3.2261,
    3.2242, 3.2424, 3.2625, 3.2758, 3.2835,
    3.2871, 3.2880, 3.2878, 3.2884, 3.2917,
    3.2958, 3.2973, 3.3039, 3.3353, 3.4122, 3.5370};'''
content = content.replace(old_lut, new_lut)

# Chunk 2: Noise parameters
old_noise = '''// Matriks Q (Process Noise Covariance): Menunjukkan seberapa "Tidak Percaya" sistem pada model tebakannya sendiri.
// Q_NOISE_00: Noise proses untuk tebakan SoC. Nilai 1e-7 sangat kecil, artinya kita SANGAT percaya pada hitungan Arus (Coulomb Counting).
const float Q_NOISE_00 = 1e-7;
// Q_NOISE_11: Noise proses untuk tegangan kapasitor (Vc1). Memberikan sedikit toleransi kelenturan pada dinamika model.
const float Q_NOISE_11 = 5e-4;

// Matriks R (Measurement Noise Covariance): Menunjukkan seberapa "Tidak Percaya" sistem pada sensor Tegangan.
// Semakin besar nilai R, EKF akan semakin mengabaikan tegangan saat mengoreksi SoC (Mencegah lonjakan koreksi berlebih).
// R_NOISE_CHARGE: Nilai besar (skeptis) karena saat di-charge tegangan sering palsu akibat resistansi.
const float R_NOISE_CHARGE = 0.035;
// R_NOISE_DISCHARGE: Nilai sedang, sensor tegangan cukup bisa diandalkan saat pengosongan.
const float R_NOISE_DISCHARGE = 0.004;
// R_NOISE_REST: Nilai sangat kecil (Sangat percaya sensor), karena saat arus nol, tegangan terminal = tegangan murni OCV.
const float R_NOISE_REST = 0.0008;'''

new_noise = '''const float Q_NOISE_00 = 2e-6f;
const float Q_NOISE_11 = 1e-1f;
const float R_BASE = 1e-4f;

const float REST_CURRENT_THRESH = 0.05f; // A - di bawah ini = rest
const int REST_SETTLE_S = 30;            // detik konfirmasi rest sebelum R_REST aktif
const float R_REST = 1e-4f;             // agresif saat confirmed rest (sama R_BASE)'''
content = content.replace(old_noise, new_noise)

# Chunk 3: Rest variables
old_rest = '''// dt_last: Menyimpan selisih waktu antar perhitungan
float dt_last = 0.0;'''
new_rest = '''// dt_last: Menyimpan selisih waktu antar perhitungan
float dt_last = 0.0;

// Rest detection state
int rest_counter_s = 0;
bool in_confirmed_rest = false;'''
content = content.replace(old_rest, new_rest)

# Chunk 4: get_dOCV_dSOC
old_docv = '''// get_dOCV_dSOC: Mencari turunan matematis dari kurva OCV terhadap SoC (Matriks Jacobian)
// Tujuan: Memberitahu EKF seberapa curam kurva tegangan baterai pada titik SoC saat ini.
// Semakin curam kurvanya, EKF akan semakin sensitif mengoreksi SoC.
float get_dOCV_dSOC(float soc)
{
  float delta = 0.001; // Menggeser sedikit (0.1%) ke depan dan belakang untuk mencari kemiringan
  float s_high = min(soc + delta, 1.0f);
  float s_low = max(soc - delta, 0.0f);
  float ocv_high = interpolate1D(s_high, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float ocv_low = interpolate1D(s_low, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE);
  float derivative = (ocv_high - ocv_low) / (s_high - s_low); // Rumus gradien (y2-y1)/(x2-x1)
  return max(derivative, 0.05f);                              // Kemiringan minimum dicegah bernilai nol agar EKF tidak lumpuh
}'''
new_docv = '''// get_dOCV_dSOC: Mencari turunan matematis dari kurva OCV terhadap SoC (Matriks Jacobian)
float get_dOCV_dSOC(float soc)
{
  soc = constrain(soc, 0.0f, 1.0f);
  float h = 0.005f;
  float soc_lo = max(soc - h, 0.0f);
  float soc_hi = min(soc + h, 1.0f);
  float dSOC = soc_hi - soc_lo;
  if (dSOC < 1e-6f)
    return 0.0f;
  return (interpolate1D(soc_hi, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE) - interpolate1D(soc_lo, lut_soc_ocv, lut_ocv, LUT_OCV_SIZE)) / dSOC;
}'''
content = content.replace(old_docv, new_docv)

# Write back
with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)
print('Phase 1 patched successfully.')
