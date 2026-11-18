# 🛸 Sıfırdan Görev Bilgisayarı Kurulum Rehberi (`sartek_siha_ws`)

Bu doküman, hiç ROS 2 kurulmamış sıfır bir **Ubuntu 22.04 LTS** Görev Bilgisayarında (Flight Computer / Geliştirme PC'si) **`sartek_siha_ws`** için gereken tüm sistem bağımlılıklarını, Python uyumluluk sabitlemelerini ve **PX4 Micro XRCE-DDS** entegrasyonunu adım adım anlatır.

> [!IMPORTANT]
> **`sartek_siha_ws` İçin Tasarım Kuralı:**
> - Bu workspace **MAVROS kullanmaz**; PX4 ile doğrudan **Micro XRCE-DDS** (`MicroXRCEAgent` + `px4_msgs`) üzerinden haberleşir.
> - Eski kurulumlardaki harici `~/px4_ros_ws` underlay'i **kullanılmaz**. `px4_msgs` doğrudan `~/sartek_siha_ws/src/px4_msgs` içinde derlenir ve `~/.bashrc` dosyasına başka workspace eklenmez.
> - Kısa özet ve ekip içi çakışma kontrolleri için [README.md](README.md), Gazebo Harmonic + PX4 SITL + QGroundControl kurulumu için [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) dosyasına bakın.

---

## 📋 Gereksinimler

- **İşletim Sistemi:** Ubuntu 22.04 LTS Server / Desktop (64-bit x86_64 veya ARM64 / Jetson)
- **İnternet Bağlantısı:** İlk kurulum ve `px4_msgs` / ROS paketleri için aktif internet bağlantısı
- **Otopilot:** PX4-Autopilot `main` / `v1.18+` (veya bu repodaki `src/px4_msgs` ile eşleşen PX4 sürümü) yüklü Pixhawk / Cube / Holybro kartı ya da SITL

---

## 🛠️ Adım 1: ROS 2 Humble ve Temel Sistem Paketleri

```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  libzbar0 \
  python3-pyqt5

# rosdep ilk kez kullanılıyorsa:
sudo rosdep init 2>/dev/null || true
rosdep update
```

---

## 🐍 Adım 2: Python Kütüphaneleri ve Sürüm Çakışması Önleme

> [!CAUTION]
> **ROS 2 Humble Python / `cv_bridge` / `colcon` Uyumluluğu:**
> - ROS 2 Humble içindeki `ros-humble-cv-bridge` paketi **NumPy 1.x ABI** ile derlendiği için `numpy>=2.0` veya `opencv-python>=4.9` kurulduğunda `AttributeError: _ARRAY_API not found` ve `KeyError: 16` hatası oluşur.
> - `ultralytics` / `torch` kurulumu `setuptools>=77.0.3` isterken `colcon-core` `setuptools<80` ister; ayrıca Ubuntu 22.04'ün eski `packaging` (`21.3`) paketi `setuptools>=77` ile `canonicalize_version(... strip_trailing_zero=False)` hatası verir.

Önce uyumlu temel sürümleri sabitleyin:

```bash
pip3 install --user \
  "numpy>=1.23.0,<2.0.0" \
  "opencv-python>=4.8.0,<4.9.0" \
  "packaging>=24.0" \
  "setuptools>=77.0.3,<80.0.0"
```

Görüntü işleme / QR / yer istasyonu haberleşmesi için ihtiyaç duyulabilecek Python paketlerini kurarken de yukarıdaki sürüm sınırlarını koruyun:

```bash
pip3 install --user \
  "numpy>=1.23.0,<2.0.0" \
  "opencv-python>=4.8.0,<4.9.0" \
  "packaging>=24.0" \
  "setuptools>=77.0.3,<80.0.0" \
  ultralytics websockets pyserial requests pyzbar pyproj
```

*(Not: NVIDIA Jetson üzerinde PyTorch kurulacaksa standart `pip install torch` yerine NVIDIA JetPack sürümünüze uygun resmi PyTorch `.whl` dosyasını kurun.)*

Doğrulama:
```bash
source /opt/ros/humble/setup.bash
python3 -c "from cv_bridge import CvBridge; import numpy as np, cv2; print('OK:', np.__version__, cv2.__version__)"
```

---

## 🔌 Adım 3: Micro-XRCE-DDS Agent Kurulumu

PX4 uORB mesajlarını ROS 2 DDS ağına aktaran aracıyı sistem geneline (`/usr/local/bin/MicroXRCEAgent`) kurun:

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/uxrce_agent
mkdir -p /tmp/uxrce_agent/build && cd /tmp/uxrce_agent/build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
rm -rf /tmp/uxrce_agent

which MicroXRCEAgent
```

---

## ⚙️ Adım 4: `~/.bashrc` Ortam Kontrolü ve Temizliği

Yeni workspace'in eski projelerle (`px4_ros_ws`, `drone_ws`, `sartek_ws`, `multi_ws`) çakışmaması için `~/.bashrc` dosyanızı kontrol edin:

```bash
grep -n "setup.bash\|RMW_IMPLEMENTATION" ~/.bashrc
```

1. Eğer `~/.bashrc` içinde `source ~/px4_ros_ws/install/setup.bash` (veya `drone_ws`, `sartek_ws`, `multi_ws`) gibi başka bir workspace satırı varsa **kaldırın**.
2. `~/.bashrc` sonunda yalnızca sistem ROS 2 ve CycloneDDS ayarları bulunsun:
   ```bash
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
3. Değişiklikten sonra **yeni bir terminal açın** (veya `source ~/.bashrc` çalıştırın) ve `env | grep AMENT_PREFIX_PATH` çıktısının yalnızca `/opt/ros/humble` gösterdiğini doğrulayın.

---

## 📦 Adım 5: `sartek_siha_ws` İndirme, `px4_msgs` Eşleştirme ve Derleme

```bash
cd ~
git clone --recursive <GITHUB_REPO_URL> sartek_siha_ws
cd ~/sartek_siha_ws
```

Eğer `src/px4_msgs` klasörü boş geldiyse (GitHub'da submodule tanımlı değilse), hedef `PX4-Autopilot` sürümünüzle eşleşen `px4_msgs` commit'ini çekin:

```bash
# Mevcut doğrulanmış PX4-Autopilot (main / 67f5c57e9a) eşleşmesi:
rm -rf ~/sartek_siha_ws/src/px4_msgs
git clone https://github.com/PX4/px4_msgs.git ~/sartek_siha_ws/src/px4_msgs
git -C ~/sartek_siha_ws/src/px4_msgs checkout de6b70e54b57d8dd2198e3c8789907f0799486f8
```

Ardından temiz ROS 2 ortamında bağımlılıkları yükleyip derleyin:

```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/sartek_siha_ws/install/setup.bash
```

Derleme doğrulaması:
```bash
ros2 pkg prefix px4_msgs
# Beklenen çıktı: /home/<kullanici>/sartek_siha_ws/install/px4_msgs

python3 -c "from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand; print('PYTHON_IMPORT_OK')"
```

---

## 🚀 Adım 6: Otopilot Bağlantısını Başlatma ve Doğrulama

### 1. Simülatör / Ethernet (UDP) ile:
- **Terminal 1 (Micro XRCE-DDS Agent):**
  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```
- **Terminal 2 (PX4 SITL + Gazebo Harmonic):**
  *(Yeni dünya/simülasyon açmadan önce eski süreçleri temizleyin)*
  ```bash
  pkill -9 -f px4; pkill -9 -f gz; sleep 1
  cd ~/PX4-Autopilot
  make px4_sitl gz_advanced_plane
  ```
  *(Farklı Gazebo dünyası ile başlatmak için: `export PX4_GZ_WORLD=<dunya_adi>`)*

### 2. Gerçek Uçuş (Seri Port — Pixhawk / Cube TELEM2) ile:
Öncesinde kullanıcının `dialout` grubunda olduğundan ve `modemmanager` servisinin kaldırıldığından emin olun (`sudo usermod -a -G dialout "$USER" && sudo apt remove -y modemmanager`).

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 3. Topic Akışı Doğrulama (Terminal 3):
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash

ros2 topic list | grep /fmu/
# PX4 main (v1.18+) sürümünde versiyonlu topic örneği:
ros2 topic echo /fmu/out/vehicle_status_v5 --once
```

---

## 📖 Terimler: `serial /dev/ttyUSB0 921600` ve `udp 8888` Nedir?

### `serial /dev/ttyUSB0 921600`
Gerçek İHA uçuşunda Görev Bilgisayarı (Jetson / Raspberry Pi / PC) ile Pixhawk/Cube Otopilot Kartı arasındaki kablolu fiziki haberleşme ayarlarıdır:
- **`serial`:** Bağlantı tipinin Seri Kablo (UART/USB) olduğunu belirtir.
- **`/dev/ttyUSB0`:** Linux'ta takılan USB-UART dönüştürücünün aygıt adresidir (doğrudan UART pinlerinde `/dev/ttyTHS1` veya `/dev/ttyAMA0` olabilir).
- **`921600`:** Baud Rate (haberleşme hızı). PX4 uXRCE-DDS yüksek frekanslı veri aktardığı için `921600` gibi yüksek bir hız kullanılır.

### `udp 8888`
Kablo yerine Sanal Ağ (Network / UDP) üzerinden haberleşme ayarlarıdır:
- **`udp`:** Bağlantı tipinin UDP ağ protokolü olduğunu belirtir.
- **`8888`:** PX4 SITL simülatörü (veya ağ üzerinden bağlanan otopilot) ile `MicroXRCEAgent`'ın haberleştiği varsayılan UDP port numarasıdır.
