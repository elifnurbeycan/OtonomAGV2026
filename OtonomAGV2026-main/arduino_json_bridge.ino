#include <Wire.h>

// IMU data structure for ROS 2
struct ImuData { float ax, ay, az, gx, gy, gz; };

// ============================================================
// DONANIM PIN TANIMLARI (Orijinal robot kablolamanız)
// ============================================================

// Sol motor (M1)
#define PIN_L_RPWM 10
#define PIN_L_LPWM 11
#define PIN_L_R_EN 22
#define PIN_L_L_EN 23

// Sağ motor (M2)
#define PIN_R_RPWM 9
#define PIN_R_LPWM 6
#define PIN_R_R_EN 24
#define PIN_R_L_EN 25

// Enkoder Sol
#define PIN_ENC_L_A  2   // Kesme pini (INT0)
#define PIN_ENC_L_B  4   // Dijital pin

// Enkoder Sağ
#define PIN_ENC_R_A  3   // Kesme pini (INT1)
#define PIN_ENC_R_B  7   // Dijital pin

// IMU I2C Adresleri
#define ADXL_ADDR 0x53
#define GYRO_ADDR 0x68

// ADXL345 & ITG3200 Kayıtçılar (Registers)
#define ADXL_BW_RATE     0x2C
#define ADXL_DATA_FORMAT 0x31
#define ADXL_POWER_CTL   0x2D
#define ADXL_DATAX0      0x32
#define GYRO_GX_H        0x1D
#define GYRO_PWR_MGM     0x3E
#define GYRO_SMPLRT_DIV  0x15
#define GYRO_DLPF_FS     0x16
#define GYRO_INT_CFG     0x17

// ============================================================
// SABİTLER & KALİBRASYON DEĞERLERİ
// ============================================================
#define BAUD_RATE      115200
#define LOOP_HZ        20
#define LOOP_MS        (1000 / LOOP_HZ)
#define CMD_TIMEOUT_MS 2000

// Ivmeölçer kalibrasyon offset/scale (Raporunuzdan alındı)
float offsetX = -0.015;
float offsetY =  0.015;
float offsetZ =  0.035;
float scaleX = 0.985;
float scaleY = 0.966;
float scaleZ = 1.005;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// ============================================================
// GLOBAL DURUM
// ============================================================
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

unsigned long last_cmd_ms = 0;
int cmd_left  = 0;
int cmd_right = 0;

char rx_buf[128];
uint8_t rx_idx = 0;

// ============================================================
// YARDIMCI I2C FONKSİYONLARI
// ============================================================
void writeByte(byte addr, byte reg, byte data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

int16_t read16_LE(byte addr, byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)2);
  int16_t value = 0;
  if (Wire.available() >= 2) {
    byte lowByte = Wire.read();
    byte highByte = Wire.read();
    value = (int16_t)((highByte << 8) | lowByte);
  }
  return value;
}

int16_t read16_BE(byte addr, byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)2);
  int16_t value = 0;
  if (Wire.available() >= 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    value = (int16_t)((highByte << 8) | lowByte);
  }
  return value;
}

// ============================================================
// ENKODER KESME SERVİSLERİ (ISR)
// ============================================================
void isr_enc_l() {
  if (digitalRead(PIN_ENC_L_B) == HIGH) enc_left--;
  else enc_left++;
}

void isr_enc_r() {
  if (digitalRead(PIN_ENC_R_B) == HIGH) enc_right--;
  else enc_right++;
}

// ============================================================
// MOTOR SÜRÜCÜ KONTROLÜ
// ============================================================
void motor_set_left(int val) {
  val = constrain(val, -255, 255);
  int pwm = map(abs(val), 0, 255, 0, 799);
  if (val > 0) {
    digitalWrite(PIN_L_LPWM, LOW);
    OCR1B = pwm;
  } else if (val < 0) {
    digitalWrite(PIN_L_LPWM, HIGH);
    OCR1B = 799 - pwm;
  } else {
    digitalWrite(PIN_L_LPWM, LOW);
    OCR1B = 0;
  }
}

void motor_set_right(int val) {
  val = constrain(val, -255, 255);
  int pwm = map(abs(val), 0, 255, 0, 799);
  if (val > 0) {
    digitalWrite(PIN_R_LPWM, LOW);
    OCR1A = pwm;
  } else if (val < 0) {
    digitalWrite(PIN_R_LPWM, HIGH);
    OCR1A = 799 - pwm;
  } else {
    digitalWrite(PIN_R_LPWM, LOW);
    OCR1A = 0;
  }
}

void motors_stop() {
  motor_set_left(0);
  motor_set_right(0);
}

// ============================================================
// IMU BAŞLATMA VE OKUMA
// ============================================================
void imu_init() {
  // ADXL345 Kurulumu
  writeByte(ADXL_ADDR, ADXL_BW_RATE, 0x0A);       // 100 Hz
  writeByte(ADXL_ADDR, ADXL_DATA_FORMAT, 0x08);  // full resolution ±2g
  writeByte(ADXL_ADDR, ADXL_POWER_CTL, 0x08);    // ölçüm modu
  
  // ITG3200 Kurulumu
  writeByte(GYRO_ADDR, GYRO_PWR_MGM, 0x00);
  delay(50);
  writeByte(GYRO_ADDR, GYRO_SMPLRT_DIV, 0x07);
  writeByte(GYRO_ADDR, GYRO_DLPF_FS, 0x18);
  writeByte(GYRO_ADDR, GYRO_INT_CFG, 0x00);
}

void calibrate_gyro() {
  const int samples = 500;
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    sumX += read16_BE(GYRO_ADDR, GYRO_GX_H);
    sumY += read16_BE(GYRO_ADDR, GYRO_GX_H + 2);
    sumZ += read16_BE(GYRO_ADDR, GYRO_GX_H + 4);
    delay(2);
  }
  gyroOffsetX = sumX / (float)samples;
  gyroOffsetY = sumY / (float)samples;
  gyroOffsetZ = sumZ / (float)samples;
}

bool imu_read(ImuData &d) {
  // ADXL345 Okuma
  int16_t ax_raw = read16_LE(ADXL_ADDR, ADXL_DATAX0);
  int16_t ay_raw = read16_LE(ADXL_ADDR, ADXL_DATAX0 + 2);
  int16_t az_raw = read16_LE(ADXL_ADDR, ADXL_DATAX0 + 4);

  float ax_g = (ax_raw * 0.0039 - offsetX) * scaleX;
  float ay_g = (ay_raw * 0.0039 - offsetY) * scaleY;
  float az_g = (az_raw * 0.0039 - offsetZ) * scaleZ;

  d.ax = ax_g * 9.80665; // m/s^2
  d.ay = ay_g * 9.80665;
  d.az = az_g * 9.80665;

  // ITG3200 Okuma
  int16_t gx_raw = read16_BE(GYRO_ADDR, GYRO_GX_H);
  int16_t gy_raw = read16_BE(GYRO_ADDR, GYRO_GX_H + 2);
  int16_t gz_raw = read16_BE(GYRO_ADDR, GYRO_GX_H + 4);

  // deg/s -> rad/s dönüşümü (ITG3200 14.375 LSB per dps)
  float gx_dps = (gx_raw - gyroOffsetX) / 14.375;
  float gy_dps = (gy_raw - gyroOffsetY) / 14.375;
  float gz_dps = (gz_raw - gyroOffsetZ) / 14.375;

  d.gx = gx_dps * (3.14159265f / 180.0f); // rad/s
  d.gy = gy_dps * (3.14159265f / 180.0f);
  d.gz = gz_dps * (3.14159265f / 180.0f);

  return true;
}

// ============================================================
// SERİ HABERLEŞME
// ============================================================
void process_line(char *line) {
  char *p_left = strstr(line, "\"left\"");
  char *p_right = strstr(line, "\"right\"");
  
  if (p_left && p_right) {
    char *val_left = strchr(p_left, ':');
    char *val_right = strchr(p_right, ':');
    
    if (val_left && val_right) {
      cmd_left  = constrain(atoi(val_left + 1), -255, 255);
      cmd_right = constrain(atoi(val_right + 1), -255, 255);
      last_cmd_ms = millis();
    }
  }
}

void read_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_idx > 0) {
        rx_buf[rx_idx] = '\0';
        process_line(rx_buf);
        rx_idx = 0;
      }
    } else if (rx_idx < sizeof(rx_buf) - 1) {
      rx_buf[rx_idx++] = c;
    }
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(BAUD_RATE);

  // BTS7960 Enable pinlerini çıkış yap ve aktif et
  pinMode(PIN_L_R_EN, OUTPUT); digitalWrite(PIN_L_R_EN, HIGH);
  pinMode(PIN_L_L_EN, OUTPUT); digitalWrite(PIN_L_L_EN, HIGH);
  pinMode(PIN_R_R_EN, OUTPUT); digitalWrite(PIN_R_R_EN, HIGH);
  pinMode(PIN_R_L_EN, OUTPUT); digitalWrite(PIN_R_L_EN, HIGH);

  pinMode(PIN_L_RPWM, OUTPUT); pinMode(PIN_L_LPWM, OUTPUT);
  pinMode(PIN_R_RPWM, OUTPUT); pinMode(PIN_R_LPWM, OUTPUT);

  // Timer 1'i 20 kHz Fast PWM Mode 14 olarak yapılandır (Prescaler = 1, Top = 799)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 799;

  motors_stop();

  // Enkoder girişleri ve kesmeler (Orijinal kesme pinleriniz)
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_enc_r, RISING);

  Wire.begin();
  Wire.setClock(400000);
  imu_init();
  delay(200);
  calibrate_gyro();

  last_cmd_ms = millis();
  Serial.println("{\"status\":\"ready\"}");
}

void loop() {
  static unsigned long last_tx = 0;
  unsigned long now = millis();

  read_serial();

  // Watchdog kontrolü
  if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    motors_stop();
    cmd_left = cmd_right = 0;
  } else {
    motor_set_left(cmd_left);
    motor_set_right(cmd_right);
  }

  // 20 Hz Telemetri Gönderimi
  if (now - last_tx >= LOOP_MS) {
    last_tx = now;

    ImuData imu = {};
    imu_read(imu);

    noInterrupts();
    int32_t el = enc_left;
    int32_t er = enc_right;
    interrupts();

    Serial.print(F("{\"encoders\":{\"left\":"));
    Serial.print(el);
    Serial.print(F(",\"right\":"));
    Serial.print(er);
    Serial.print(F("},\"imu\":{\"accel\":{\"x\":"));
    Serial.print(imu.ax, 6);
    Serial.print(F(",\"y\":"));
    Serial.print(imu.ay, 6);
    Serial.print(F(",\"z\":"));
    Serial.print(imu.az, 6);
    Serial.print(F("},\"gyro\":{\"x\":"));
    Serial.print(imu.gx, 6);
    Serial.print(F(",\"y\":"));
    Serial.print(imu.gy, 6);
    Serial.print(F(",\"z\":"));
    Serial.print(imu.gz, 6);
    Serial.print(F("}},\"cmd_l\":"));
    Serial.print(cmd_left);
    Serial.print(F(",\"cmd_r\":"));
    Serial.print(cmd_right);
    Serial.print(F(",\"status\":\"ok\"}\n"));
  }
}
