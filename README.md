# 🛩️ SARTEK Savaşan İHA — Micro XRCE-DDS Workspace Kurulum ve Çakışma Önleme Rehberi (`sartek_siha_ws`)

Bu doküman, **`sartek_siha_ws`** çalışma alanını GitHub'dan çekecek geliştiricilerin **temiz, çakışmasız ve birebir uyumlu** bir Micro XRCE-DDS (PX4 + ROS 2 Humble) ortamı kurması için gereken ilk kurulum adımlarını, indirilmesi gereken bileşenleri ve kritik ortam kontrollerini içerir.

> [!IMPORTANT]
> **Bu Workspace'in Farkı ve Bağımsızlık Kuralı:**
> - `sartek_siha_ws`, **MAVROS kullanmaz**; PX4 ile doğrudan **Micro XRCE-DDS** (`MicroXRCEAgent` + `px4_msgs`) üzerinden haberleşir.
> - `px4_msgs` paketi ortak dış bir underlay (`~/px4_ros_ws`) yerine **doğrudan bu workspace içine (`src/px4_msgs`)** sabit sürümle eklenmiştir.
> - Sıfırdan görev bilgisayarı ve `MicroXRCEAgent` kurulumu için [INSTALLATION.md](INSTALLATION.md), Gazebo Harmonic + PX4 SITL + QGroundControl kurulumu için [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) rehberlerini takip edebilirsiniz; bu dosya ise hızlı kurulum akışı ile çakışma kontrollerini özetler.

---

## 1. İndirilmesi ve Kurulu Olması Gereken Sistem Bileşenleri

Bu workspace'i çekip çalıştıracak bilgisayarda (Ubuntu 22.04 LTS x86_64 veya Jetson/ARM64) aşağıdaki sistem geneli bileşenler kurulu olmalıdır:

| Bileşen | Beklenen Durum / Sürüm | Kurulum / Not |
|---|---|---|
| **Ubuntu** | `22.04 LTS` | Temel işletim sistemi |
| **ROS 2** | `Humble` (`/opt/ros/humble`) | `ros-humble-desktop`, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pip` |
| **RMW (DDS)** | `rmw_cyclonedds_cpp` | `sudo apt install -y ros-humble-rmw-cyclonedds-cpp` |
| **Micro-XRCE-DDS Agent** | `/usr/local/bin/MicroXRCEAgent` | `eProsima/Micro-XRCE-DDS-Agent` kaynaktan derlenmiş olmalı |
| **Gazebo** | **Harmonic** (`gz-harmonic`) | `gz-harmonic` + `ros-humble-ros-gzharmonic` (**Fortress `ros-humble-ros-gz` kurulmamalı!**) |
| **PX4-Autopilot** | `main` (`v1.18.0-beta1` / `67f5c57e9a`+) | `~/PX4-Autopilot` (Bu repo içindeki `src/px4_msgs` ile aynı sürümde olmalı) |
| **Sistem Kütüphaneleri** | `libzbar0`, `python3-pyqt5` | QR (`pyzbar`) ve arayüz bağımlılıkları |
| **Python Sabitlemeleri** | `numpy<2`, `opencv-python<4.9.0` | ROS 2 Humble `cv_bridge` ve `colcon` uyumu için (Bölüm 2.5'e bakın) |
| **QGroundControl** *(Opsiyonel)* | `~/.local/bin/qgroundcontrol` | Harici GCS izleme/kalkış için ([PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) Adım 4) |

> Bilgisayarınız tamamen sıfırsa sistem geneli kurulumlar (ROS 2 Humble, `MicroXRCEAgent`, Gazebo Harmonic, `PX4-Autopilot`, QGC) için [INSTALLATION.md](INSTALLATION.md) ve [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) dosyalarındaki adım adım yönergeleri uygulayabilirsiniz.

---

## 2. Kritik Çakışma Kontrolleri (Kurulumdan ve Derlemeden Önce!)

Yeni workspace'in eski projelerle (`px4_ros_ws`, `drone_ws`, `sartek_ws`, `multi_ws`) veya Python/Gazebo paketleriyle çakışmaması için aşağıdaki **5 kontrolü mutlaka yapın**:

### 2.1. `~/.bashrc` Kontrolü (En Kritik Kontrol)
Her yeni terminal açıldığında `~/.bashrc` otomatik çalışır. Eğer `~/.bashrc` içinde eski bir workspace (özellikle eski PX4 sürümü içeren `~/px4_ros_ws`) `source` ediliyorsa, `sartek_siha_ws` derlenirken eski workspace'in üzerine bağlanır ve yanlış `px4_msgs` mesajları yüklenir.

Kontrol edin:
```bash
grep -n "setup.bash" ~/.bashrc
env | grep -E "AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH"
```

**Olması gereken durum:**
- `~/.bashrc` içinde **yalnızca** sistem ROS 2 kurulumu bulunmalı (başka bir workspace varsa tek aktif projeniz buysa sadece `sartek_siha_ws` olabilir):
  ```bash
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
- `~/.bashrc` içinde **`px4_ros_ws`, `drone_ws`, `sartek_ws`, `multi_ws` satırları BULUNMAMALIDIR.**
- `env` çıktısında `AMENT_PREFIX_PATH` yalnızca `/opt/ros/humble` (ve bu workspace'i source ettiyseniz `/home/<kullanici>/sartek_siha_ws/install/...`) göstermelidir.

> **Çoklu Proje Kuralı:** Bilgisayarınızda birden fazla ROS 2 workspace'i duruyorsa `~/.bashrc` içine hiçbir workspace satırı yazmayın; hangi projede çalışıyorsanız sadece o terminalde `source ~/sartek_siha_ws/install/setup.bash` çalıştırın.

---

### 2.2. `RMW_IMPLEMENTATION` Eşleşmesi
Tüm terminaller (MicroXRCEAgent, Gazebo köprüsü, algoritma düğümleri) aynı DDS katmanını kullanmazsa düğümler birbirini göremez.

Kontrol edin:
```bash
echo "$RMW_IMPLEMENTATION"
# Beklenen çıktı: rmw_cyclonedds_cpp
```
Eksikse `ros-humble-rmw-cyclonedds-cpp` paketini kurup `~/.bashrc` içine `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ekleyin.

---

### 2.3. `PX4-Autopilot` ↔ `px4_msgs` Sürüm Eşleşmesi
Micro XRCE-DDS'te PX4 içindeki uORB mesaj tanımıyla ROS 2 tarafındaki `px4_msgs` tanımı **birebir eşleşmek zorundadır**. Sürüm farklı olursa `MicroXRCEAgent` bağlansa bile topic'lerden veri gelmez.

Bu workspace'te doğrulanmış eşleşme:
- **`~/PX4-Autopilot`:** `main` (`v1.18.0-beta1`, commit `67f5c57e9a` / `a6814cc7b6`)
- **`src/px4_msgs`:** `main`, commit `de6b70e54b57d8dd2198e3c8789907f0799486f8` (*Update to PX4 67f5c57e9a18d5b3d672a9932f9cb8606d7446fc*)

Kendi makinenizdeki `PX4-Autopilot` sürümünü kontrol edin:
```bash
git -C ~/PX4-Autopilot describe --tags --always
git -C ~/PX4-Autopilot log -n 3 --oneline
```
> Eğer başka bir PX4 sürümü (ör. `v1.15.x` veya daha yeni bir `main` commit'i) kullanacaksanız `src/px4_msgs` dalını/commit'ini kendi PX4 sürümünüze çekip `build/ install/ log/` silerek yeniden derlemelisiniz.

---

### 2.4. Gazebo Harmonic vs. Fortress Çakışması
ROS 2 Humble varsayılan deposundaki `ros-humble-ros-gz` **Gazebo Fortress** kurar ve Harmonic ile çakışır.

Kontrol edin:
```bash
gz sim --version
dpkg -l | grep ros-humble-ros-gz
```
**Beklenen:** `gz-harmonic` ve `ros-humble-ros-gzharmonic` kurulu olmalı; `ros-humble-ros-gz` (Fortress) kurulu **olmamalı**.

---

### 2.5. Python / `cv_bridge` / `colcon` Sürüm Kontrolü
ROS 2 Humble `cv_bridge` paketi **NumPy 1.x** ile derlenmiştir. `pip` üzerinden `numpy>=2.0`, `opencv-python>=4.9` veya `setuptools>=80` kurulması (`ultralytics` ya da `PX4-Autopilot/Tools/setup/ubuntu.sh` sonrası sık yaşanır) `cv_bridge` ve `colcon build` işlemlerini bozar.

Kontrol edin:
```bash
source /opt/ros/humble/setup.bash
python3 -c "from cv_bridge import CvBridge; import numpy as np, cv2; print('OK:', np.__version__, cv2.__version__)"
```
Hata alırsanız veya `numpy` 2.x görünüyorsa şu sabit sürümleri yükleyin:
```bash
pip3 install --user "numpy>=1.23.0,<2.0.0" "opencv-python>=4.8.0,<4.9.0" "packaging>=24.0" "setuptools>=77.0.3,<80.0.0"
```

---

## 3. GitHub'dan Çeken Kişi İçin İlk Kurulum Adımları

Yukarıdaki kontrollerden geçtikten sonra yeni bir bilgisayarda workspace'i şu sırayla kurun:

### Adım 1: Depoyu Klonlayın ve `px4_msgs` İçeriğini Doğrulayın
```bash
cd ~
git clone --recursive <GITHUB_REPO_URL> sartek_siha_ws
cd ~/sartek_siha_ws
```

> [!NOTE]
> Eğer GitHub'a pushlarken `src/px4_msgs` submodule olarak eklenmediyse klonlama sonrasında `src/px4_msgs` klasörü boş gelebilir. Böyle bir durumda PX4 sürümünüzle uyumlu `px4_msgs` commit'ini manuel olarak çekin:
> ```bash
> ls -la ~/sartek_siha_ws/src/px4_msgs
> # Klasör boşsa veya yoksa:
> rm -rf ~/sartek_siha_ws/src/px4_msgs
> git clone https://github.com/PX4/px4_msgs.git ~/sartek_siha_ws/src/px4_msgs
> git -C ~/sartek_siha_ws/src/px4_msgs checkout de6b70e54b57d8dd2198e3c8789907f0799486f8
> ```

### Adım 2: Temiz ROS 2 Ortamında Bağımlılıkları Yükleyin ve Derleyin
Başka workspace'lerin ortam değişkenlerine karışmaması için sadece `/opt/ros/humble/setup.bash` yükleyerek derleyin:
```bash
source /opt/ros/humble/setup.bash
cd ~/sartek_siha_ws

# (İlk kez rosdep kullanıyorsanız: sudo rosdep init && rosdep update)
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source ~/sartek_siha_ws/install/setup.bash
```

### Adım 3: Derleme ve Yol Doğrulaması
`px4_msgs` paketinin başka bir workspace'ten değil, doğrudan `~/sartek_siha_ws` içinden okunduğunu doğrulayın:
```bash
ros2 pkg prefix px4_msgs
# Beklenen çıktı: /home/<kullanici>/sartek_siha_ws/install/px4_msgs

python3 -c "from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand; print('PYTHON_IMPORT_OK')"
```

---

## 4. SARTEK Simülasyonu (`sartek_sim`) ve Micro XRCE-DDS Uçtan Uca Test

### A. Tek Komutla SARTEK Simülasyonu (Önerilen)
[src/sartek_sim](src/sartek_sim) paketi `PX4-Autopilot` kaynak kodunu değiştirmeden (`PX4_GZ_STANDALONE=1`) Gazebo Harmonic (`yunuseli.sdf`), kameralı uçak (`advanced_plane_cam`), PX4 SITL, `ros_gz_image` kamera köprüsü ve `MicroXRCEAgent`'ı tek seferde başlatır:

```bash
pkill -9 -f px4; pkill -9 -f gz; pkill -9 -f MicroXRCEAgent
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 launch sartek_sim sim.launch.py
```

**Başlıca Launch Parametreleri:**
- `headless:=true` → GUI açmadan simülasyon çalıştırır
- `start_px4:=false` / `start_agent:=false` / `start_bridge:=false` → bileşenleri ayrı yönetmek için kapatılabilir
- `px4_dir:=$HOME/PX4-Autopilot` → farklı PX4 konumu vermek için

**Doğrulama (İkinci Terminalde):**
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 topic list | grep -E "camera|/fmu/"
ros2 topic echo /camera/image_raw --no-arr --once
ros2 topic echo /fmu/out/vehicle_status_v5 --once
```
*(Kamera görüntüsü 640×640 `rgb8` formatında `/camera/image_raw` üzerinden gelir; OpenCV/YOLO tarafında `cv_bridge` ile `desired_encoding="bgr8"` olarak okuyabilirsiniz.)*

### B. Yalın PX4 Testi (3 Terminalde Manuel)
Sadece PX4 + Micro XRCE-DDS bağlantısını `sartek_sim` olmadan test etmek isterseniz:

```bash
# Önce kalıntı süreçleri temizleyin
pkill -9 -f px4; pkill -9 -f gz; pkill -9 -f MicroXRCEAgent
```

- **Terminal 1 — Micro XRCE-DDS Agent:**
  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```
  *(Gerçek uçuş bilgisayarında UART/USB bağlantısı için: `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`)*

- **Terminal 2 — PX4 SITL + Gazebo Harmonic:**
  ```bash
  cd ~/PX4-Autopilot
  make px4_sitl gz_advanced_plane
  ```
  *(Hızlı çoklu-rotor testi için `gz_x500` de kullanılabilir.)*

- **Terminal 3 — ROS 2 Topic Doğrulama:**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/sartek_siha_ws/install/setup.bash
  ros2 topic list | grep /fmu/
  ```

### ⚠️ PX4 v1.18+ Versiyonlu Topic İsimleri ve Önemli Kurallar
Mevcut `PX4-Autopilot` (`main` / `v1.18.0-beta1`) sürümünde `uxrce_dds_client`, `MESSAGE_VERSION != 0` olan mesajların topic adına `_v<SÜRÜM>` eki ekler:
- `/fmu/out/vehicle_status_v5` (`VehicleStatus`)
- `/fmu/out/vehicle_local_position_v1` (`VehicleLocalPosition`)
- `/fmu/out/vehicle_global_position` (`VehicleGlobalPosition`)
- `/fmu/out/vehicle_attitude` (`VehicleAttitude`)
- `/fmu/out/battery_status_v1` (`BatteryStatus`)
- `/fmu/out/airspeed_validated_v1` (`AirspeedValidated`)
- `/fmu/out/home_position_v2` (`HomePosition`)
- `/fmu/out/vehicle_command_ack_v2` (`VehicleCommandAck`)
- `/fmu/in/offboard_control_mode` (`OffboardControlMode`)
- `/fmu/in/trajectory_setpoint` (`TrajectorySetpoint`)
- `/fmu/in/vehicle_attitude_setpoint_v1` (`VehicleAttitudeSetpoint`)
- `/fmu/in/vehicle_command_v1` (`VehicleCommand`)

> [!CAUTION]
> **Algoritma Geliştirirken 3 Altın Kural:**
> 1. **Koordinat Sistemi (NED):** PX4 `px4_msgs` **NED** (X=Kuzey, Y=Doğu, **Z=Aşağı pozitif**, yani 100 m irtifa `z = -100.0`) kullanır.
> 2. **QoS Profili:** `/fmu/out/*` topic'lerine abone olurken **`ReliabilityPolicy.BEST_EFFORT`** kullanın; aksi halde veri gelmez.
> 3. **Timestamp & Offboard:** `/fmu/in/*` mesajlarında `timestamp` alanı **mikrosaniye** (`int(node.get_clock().now().nanoseconds / 1000)`) olmalı ve Offboard mod için `OffboardControlMode` + setpoint sürekli (≥ 10–20 Hz) basılmalıdır.

---

## 5. GitHub'a Push ve Ekip İçi Çalışma Kuralları

1. **`build/`, `install/`, `log/` klasörlerini asla commit'lemeyin** ([.gitignore](.gitignore) içinde tanımlıdır).
2. **`px4_msgs` yönetimi:** Yeni projeyi GitHub'a pushlarken `src/px4_msgs`'i ayrı bir iç `.git` klasörü olarak bırakmak yerine **Git Submodule** olarak ekleyin veya ekip arkadaşlarınızın Adım 1'deki komutla aynı commit'i (`de6b70e`) çekmesini sağlayın; aksi halde `git add .` yaptığınızda `src/px4_msgs` boş bir gitlink olarak pushlanır ve diğer bilgisayarda içi boş gelir.
3. **Aynı ağda birden fazla kişi test yapıyorsa** her ekip/uçak için ayrı `ROS_DOMAIN_ID` (`export ROS_DOMAIN_ID=<id>`) kullanın; tüm terminallerin aynı `ROS_DOMAIN_ID` ve `RMW_IMPLEMENTATION` değerine sahip olduğundan emin olun.
