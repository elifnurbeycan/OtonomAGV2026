# Otonom AGV (SIRIUS AMR) - ROS 2 & Arduino Serial Bridge

Bu depo, **SIRIUS AMR** projesi kapsamında Arduino Uno (ATmega328P) mikrodenetleyicisi ile ROS 2 Humble/Jazzy yüklü Pi/PC arasındaki hafifletilmiş seri haberleşme köprüsünü, odometri hesabını, IMU entegrasyonunu ve BTS7960 motor sürücüsü için ultrasonik (sessiz) motor kontrol yazılımını içerir.

---

## 🛠 Neler Değiştirildi ve Neler Yapıldı?

1. **Hafıza Optimizasyonu (RAM Koruması):**
   * Arduino Uno sadece 2KB RAM'e sahiptir. Varsayılan `ArduinoJson` kütüphanesinin dinamik hafıza (heap) yönetimi bu kısıtlı alanda çökmelere ve seri paketlerin bozulmasına sebep oluyordu.
   * `ArduinoJson` tamamen kaldırılarak yerine dinamik bellek tahsisi yapmayan (heap-free) hafif bir string parser (`strstr` ve `atoi` tabanlı) ve `F()` flash bellek makrosu kullanan doğrudan telemetri yazıcısı yazıldı. RAM kullanımı **%47'den %30'a düşürüldü**.
2. **Sağ Enkoder Yönü Düzeltmesi:**
   * İleri yönde iki tekerlek de dönerken sağ enkoderin eksi (`-`), sol enkoderin artı (`+`) sayması nedeniyle odometri düğümü robotun spin attığını sanıyordu.
   * Arduino tarafında `isr_enc_r` kesme fonksiyonu tersine çevrildi, böylece ileri sürüşte her iki enkoder de pozitif yönde artarak RViz üzerinde düzgün doğrusal hareket sağlar.
3. **BTS7960 İçin 20 kHz Ultrasonik (Sessiz) PWM:**
   * Motorlardan gelen yüksek frekanslı çınlama/ıslık sesini engellemek için **Timer 1** (Pin 9 ve 10) donanımsal olarak tam **20 kHz** frekansa (Fast PWM Mode 14) ayarlandı.
   * **Sign-Magnitude PWM** moduna geçilerek hız PWM pini (Timer 1) ve yön digital pini aracılığıyla motorlar tamamen sessiz çalıştırıldı. `millis()` ve `delay()` fonksiyonlarını yöneten Timer 0'a dokunulmadı.
4. **Fren Mekanizmalı Klavye Sürüşü:**
   * `keyboard_control.py` scripti durum kontrollü sürüşe geçirildi. Robot `w` ile ileri giderken `s` tuşuna basıldığında aniden geri gitmek yerine önce durur (fren görevi), tekrar basıldığında geri gider.

---

## 📦 Bağımlılıklar ve Gerekli Kütüphaneler

Sistemi çalıştırmadan önce aşağıdaki paketlerin ve kütüphanelerin bilgisayarınızda (veya Raspberry Pi üzerinde) kurulu olduğundan emin olun:

### 1. İşletim Sistemi ve ROS 2
* **Ubuntu 22.04 LTS** (veya uyumlu bir Linux sürümü).
* **ROS 2 Humble Desktop** (veya Jazzy) kurulu olmalıdır.

### 2. Python Kütüphaneleri (PC / Pi Tarafı)
Seri port haberleşmesi için `pyserial` kütüphanesinin yüklü olması gerekir:
```bash
sudo apt update
sudo apt install -y python3-serial
# veya pip ile yüklemek isterseniz:
pip install pyserial
```

### 3. ROS 2 Paket Bağımlılıkları
Odometri, transform (TF) ve robot durumu yayını için gerekli standart ROS 2 mesaj paketleri:
```bash
sudo apt install -y ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-robot-state-publisher
```

### 4. Arduino Tarafı (Mikrodenetleyici)
* **Donanım:** Arduino Uno (veya ATmega328P tabanlı Nano/Pro Mini).
* **Kütüphaneler:** Kodumuz **sıfır-bağımlılık (zero-dependency)** prensibiyle tamamen ham C++ ile optimize edilmiştir. `ArduinoJson` dahil **hiçbir harici kütüphaneye ihtiyaç duymaz**. Sadece Arduino IDE ile yüklü gelen dahili `Wire.h` (I2C) kütüphanesini kullanır.

### 5. Sürüm Kontrolü (Git)
GitHub işlemlerini ve kod güncellemelerini yönetmek için:
```bash
sudo apt install -y git
```

---

## 🚀 Sistem Nasıl Çalıştırılır? (Detaylı Kılavuz)

Adımları sırasıyla takip ediniz:

### 1. Arduino Kodunun Yüklenmesi
* Arduino Uno kartınızı bilgisayara USB ile bağlayın.
* Arduino IDE (veya terminalden `/snap/bin/arduino`) aracılığıyla `arduino_json_bridge.ino` dosyasını kartınıza yükleyin.

### 2. Seri Port Yetkilerinin Verilmesi (Linux)
Arduino resetlendiğinde veya USB takılıp çıkarıldığında Linux port izinlerini sıfırlar.
* Port erişimi sağlamak için terminalinizde şu komutu çalıştırıp şifrenizi girin:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  ```
* *Kalıcı Çözüm (Önerilen):* Her seferinde bu komutu yazmamak için kullanıcınızı `dialout` grubuna ekleyin (oturum kapatıp açınca aktif olur):
  ```bash
  sudo usermod -aG dialout $USER
  ```

### 3. ROS 2 Workspace Derleme (Build)
* Robotun ROS 2 workspace dizinine gidin ve projeyi derleyin:
  ```bash
  cd ~/sirius_amr_ws
  colcon build --symlink-install
  ```

### 4. Düğümlerin (Nodes) Başlatılması
* **Terminal 1:** ROS 2 launch paketini başlatarak Arduino ile bağlantıyı açın (bu komut odometriyi, robot modelini, TF eksenlerini ve IMU verisini yayınlar):
  ```bash
  cd ~/sirius_amr_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch agv_bringup bringup.launch.py
  ```

### 5. Klavye Kontrolünün Başlatılması
* **Terminal 2:** Robotu yön tuşları yerine klavye kısayolları ile sürmek için kontrol scriptini çalıştırın:
  ```bash
  cd ~/Downloads/OtonomAGV2026-main
  source /opt/ros/humble/setup.bash
  python3 keyboard_control.py
  ```
* **Klavye Kısayolları:**
  * `w` : İleri sür (geri gidiyorsa durur)
  * `s` : Geri sür / Frenle (ileri gidiyorsa durur)
  * `a` : Sola dön (sağa dönüyorsa düzeltir)
  * `d` : Sağa dön (sola dönüyorsa düzeltir)
  * `x` : Acil durdur (Tüm hızları sıfırlar)
  * `q` : Çıkış yap ve robotu durdur

### 6. RViz Üzerinde Görselleştirme
* **Terminal 3:** RViz ekranını açın:
  ```bash
  source /opt/ros/humble/setup.bash
  rviz2
  ```
* **RViz Arayüzü Ayarları:**
  1. Sol üst köşedeki **Global Options** -> **Fixed Frame** seçeneğini `map` yerine **`odom`** yapın.
  2. Sol alttaki **Add** butonuna tıklayın, listeden **RobotModel** seçin ve *OK* deyin.
  3. Sol alttaki **Add** butonuna tıklayın, listeden **TF** seçin ve *OK* deyin.
  4. Sol alttaki **Add** butonuna tıklayın, üstteki **By topic** sekmesine geçip `/odom` altındaki **Odometry** seçeneğini seçin ve *OK* deyin.
  5. Sol alttaki **Reset** butonuna tıklayarak eski verileri sıfırlayın.

Artık klavyeden robota komut verdikçe RViz üzerindeki modelin ve eksenlerin pürüzsüzce ileri/geri hareket ettiğini görebilirsiniz.
