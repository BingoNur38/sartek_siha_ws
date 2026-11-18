# 🛸 Workspace'ten Bağımsız PX4-Autopilot, Gazebo Harmonic ve QGroundControl Kurulum Rehberi (`sartek_siha_ws`)

Bu rehber, **Ubuntu 22.04 LTS + ROS 2 Humble** üzerinde; **`sartek_siha_ws`** (Micro XRCE-DDS tabanlı SİHA yazılımı) ve diğer projelerle çakışmayacak şekilde aşağıdaki ortak araçları sistem geneline kurmanızı sağlar:

- **Gazebo Harmonic** + **`ros-humble-ros-gzharmonic`** (ROS 2 ↔ Gazebo kamera ve sensör köprüsü)
- **`~/PX4-Autopilot`** (PX4 SITL uçuş kontrolcüsü ve derleme araçları)
- **CycloneDDS** (`ros-humble-rmw-cyclonedds-cpp`)
- **QGroundControl (QGC)** (Yer Kontrol İstasyonu)

> [!NOTE]
> **`sartek_siha_ws` İçin Önemli Fark:**
> - Bu workspace **MAVROS kullanmaz**; PX4 ile doğrudan **Micro XRCE-DDS** (`MicroXRCEAgent` + `px4_msgs`) üzerinden haberleşir.
> - `px4_msgs` ortak harici bir underlay (`~/px4_ros_ws`) yerine **doğrudan `~/sartek_siha_ws/src/px4_msgs` içinde** tutulur; böylece diğer workspace'lerdeki eski `px4_msgs` sürümleriyle çakışma yaşanmaz.

---

## 🏗️ Genel Mimari

Bu kurulumda ortak simülatör/otopilot araçları sistem veya kullanıcı geneline, `px4_msgs` ve SİHA paketleri ise doğrudan `~/sartek_siha_ws` içine yerleştirilir:

| Bileşen | Kurulum Konumu | Kapsam |
|---|---|---|
| **ROS 2 Humble** | `/opt/ros/humble` | Sistem geneli (`~/.bashrc`) |
| **Gazebo Harmonic & Köprü** | `/usr/bin/gz`, `/opt/ros/humble` | Sistem geneli |
| **CycloneDDS** | `/opt/ros/humble` | Sistem geneli (`RMW_IMPLEMENTATION`) |
| **MicroXRCEAgent** | `/usr/local/bin/MicroXRCEAgent` | Sistem geneli |
| **`px4_msgs` & SİHA Paketleri** | `~/sartek_siha_ws` | Workspace içi (`source install/setup.bash`) |
| **PX4-Autopilot (SITL)** | `~/PX4-Autopilot` | Kullanıcı geneli (`PX4_DIR`) |
| **QGroundControl** | `~/.local/bin/qgroundcontrol` | Tüm terminallerden (`qgroundcontrol`) |

```text
  Gazebo Harmonic (gz sim)
          ▲
          │  (ros-humble-ros-gzharmonic / kamera & sensör köprüsü)
          ▼
     PX4 SITL (~/PX4-Autopilot)
      ├── uXRCE-DDS (UDP:8888) ──► MicroXRCEAgent ──► ROS 2 (/fmu/... + sartek_siha_ws/px4_msgs)
      └── MAVLink   (UDP:14550) ────────────────────► QGroundControl (GCS)
```

---

## 0. Ön Hazırlık ve Mevcut Ortam Kontrolü

Devam etmeden önce sisteminizde şunların kurulu ve çakışmasız olduğundan emin olun:

```bash
# ROS 2 ve MicroXRCEAgent kontrolü
source /opt/ros/humble/setup.bash
which ros2 MicroXRCEAgent

# ~/.bashrc içinde eski workspace (px4_ros_ws, drone_ws, sartek_ws) olmamalı!
grep -n "setup.bash\|RMW_IMPLEMENTATION" ~/.bashrc

# QR kütüphanesi (libzbar0) ve rosdep henüz kurulmadıysa:
sudo apt update && sudo apt install -y libzbar0
sudo rosdep init 2>/dev/null || true
rosdep update
```

*(Eğer `MicroXRCEAgent` henüz kurulu değilse [INSTALLATION.md](INSTALLATION.md) Adım 3'ü uygulayın.)*

---

## Adım 1: Gazebo Harmonic ve `ros-humble-ros-gzharmonic` Kurulumu

> [!IMPORTANT]
> Standart ROS 2 deposundaki `ros-humble-ros-gz` paketi **Gazebo Fortress** içindir. Güncel PX4 ve `gz-msgs10` / `gz-transport13` kullanan projeler **Gazebo Harmonic** gerektirir. İkisini karıştırmak köprü ve mesaj uyumsuzluğu doğurur.

1. Eğer önceden yanlışlıkla Fortress tabanlı `ros-humble-ros-gz*` kurulduysa temizleyin:
   ```bash
   sudo apt remove -y 'ros-humble-ros-gz*' 2>/dev/null || true
   ```

2. OSRF resmi Gazebo deposunu ekleyin:
   ```bash
   sudo apt update && sudo apt install -y curl lsb-release gnupg
   sudo mkdir -p /usr/share/keyrings
   curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | sudo gpg --yes --dearmor -o /usr/share/keyrings/osrf-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   ```

3. Gazebo Harmonic ve ROS 2 Humble köprü paketini kurun:
   ```bash
   sudo apt update
   sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic
   ```

4. Kurulumu doğrulayın:
   ```bash
   gz sim --version
   dpkg -l | grep ros-humble-ros-gz
   ```

---

## Adım 2: `~/PX4-Autopilot` Kurulumu ve İlk SITL Derlemesi

1. PX4-Autopilot deposunu ev dizininize (`~/PX4-Autopilot`) alt modülleriyle klonlayın:
   ```bash
   cd ~
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd ~/PX4-Autopilot
   ```

2. **PX4 Sürüm ↔ `px4_msgs` Eşleşmesi — ÖNEMLİ:**
   - > [!WARNING]
     > **PX4 ↔ `px4_msgs` Sürüm Eşleşmesi Zorunludur!**
     > `~/PX4-Autopilot` sürümü ile `~/sartek_siha_ws/src/px4_msgs` paketinin commit/branch'i **birebir aynı sürüm ailesinde** olmalıdır. Aksi halde `MicroXRCEAgent` bağlansa bile uORB/DDS tipleri eşleşmez ve topic verisi akmaz.
     >
     > Mevcut `sartek_siha_ws` kurulumu `PX4-Autopilot` `main` (`v1.18.0-beta1` — `67f5c57e9a` / `a6814cc7b6`) ve `px4_msgs` (`de6b70e`) ile doğrulanmıştır.
     >
     > Başka bir PX4 sürümüne geçerseniz `sartek_siha_ws` içindeki `px4_msgs` sürümünü de güncelleyip temiz derleyin:
     > ```bash
     > cd ~/sartek_siha_ws/src/px4_msgs
     > git fetch && git checkout <uyumlu_branch_veya_commit>
     > cd ~/sartek_siha_ws
     > rm -rf build/ install/ log/
     > source /opt/ros/humble/setup.bash
     > colcon build --symlink-install
     > ```

3. PX4 Ubuntu bağımlılık kurulum betiğini çalıştırın:
   - Gazebo Harmonic'i Adım 1'de kurduğunuz için `--no-sim-tools` bayrağı ile mevcut Gazebo kurulumunuzu koruyabilirsiniz.
   - Yalnızca simülasyon (SITL) kullanacaksanız `--no-nuttx` ekleyebilirsiniz; gerçek Pixhawk kartı için firmware derleyecekseniz `--no-nuttx` **eklemeyin**:
     ```bash
     cd ~/PX4-Autopilot
     # Sadece simülasyon (SITL) için:
     bash ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

     # Veya donanım firmware derleme araçları (arm-none-eabi-gcc) da dahil:
     # bash ./Tools/setup/ubuntu.sh --no-sim-tools
     ```

4. > [!CAUTION]
   > **PX4 `ubuntu.sh` Sonrası Python / ROS 2 Kontrolü:**
   > PX4'ün `ubuntu.sh` betiği kendi Python bağımlılıklarını `pip` ile kurarken `numpy` veya `setuptools` sürümünü yükseltebilir. Bu durum ROS 2 Humble `cv_bridge` (`AttributeError: _ARRAY_API`) ve `colcon build` işlemlerini bozabilir. `ubuntu.sh` bittikten sonra **mutlaka** aşağıdaki kontrolü yapın:
   > ```bash
   > source /opt/ros/humble/setup.bash
   > python3 -c "from cv_bridge import CvBridge; import numpy as np, cv2; print('OK:', np.__version__, cv2.__version__)"
   > ```
   > Eğer hata alırsanız veya `numpy >= 2` görünürse tekrar sabitleyin:
   > ```bash
   > pip3 install --user "numpy>=1.23.0,<2.0.0" "opencv-python>=4.8.0,<4.9.0" "packaging>=24.0" "setuptools>=77.0.3,<80.0.0"
   > ```

5. İlk SITL derlemesini yapıp sabit kanat simülatörünü test edin:
   ```bash
   cd ~/PX4-Autopilot
   # Sabit kanat (Fixed-wing) SİHA testi:
   make px4_sitl gz_advanced_plane

   # (İsteğe bağlı) Döner kanat (Quadrotor - x500) testi için:
   # make px4_sitl gz_x500
   ```
   Gazebo penceresi açılıp PX4 konsolunda `Ready for takeoff!` / `pxh>` göründüğünde derleme başarılıdır. Kapatmak için terminalde `Ctrl+C` (veya `pxh>` içinde `shutdown`) kullanın.

6. *(İsteğe bağlı)* Scriptlerin otomatik bulması için `~/.bashrc` içine `PX4_DIR` ekleyin:
   ```bash
   if ! grep -q "export PX4_DIR=" ~/.bashrc; then
       echo 'export PX4_DIR="$HOME/PX4-Autopilot"' >> ~/.bashrc
   fi
   source ~/.bashrc
   ```

---

## Adım 3: CycloneDDS Kurulumu (`sartek_siha_ws` için MAVROS Gerektirmez)

`sartek_siha_ws` Micro XRCE-DDS tabanlı olduğu için **MAVROS (`ros-humble-mavros`) ve GeographicLib kurulumu gerekmez.** Yalnızca tüm ROS 2 terminallerinin aynı DDS katmanında haberleşmesi için CycloneDDS kurulumu yeterlidir:

1. APT paketini kurun:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-rmw-cyclonedds-cpp
   ```

2. `~/.bashrc` içinde ayarlayın:
   ```bash
   if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
       echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
   fi
   source ~/.bashrc
   ```
   > **Not:** ROS 2 düğümlerinin birbirini görebilmesi için aynı anda çalışan tüm terminallerin aynı `RMW_IMPLEMENTATION` (ve kullanılıyorsa aynı `ROS_DOMAIN_ID`) değerine sahip olması gerekir.

*(Not: Yalnızca eski MAVROS tabanlı projelerle çalışacaksanız `ros-humble-mavros ros-humble-mavros-extras` ve `install_geographiclib_datasets.sh` gerekir; `sartek_siha_ws` içinde kullanılmaz.)*

---

## Adım 4: QGroundControl (QGC) Kurulumu

1. Seri port yetkisi verin ve portları kilitleyen `modemmanager` servisini kaldırın:
   ```bash
   sudo usermod -a -G dialout "$USER"
   sudo apt remove -y modemmanager
   ```

2. Gerekli video (GStreamer) ve AppImage (FUSE / Qt / X11) kütüphanelerini kurun:
   ```bash
   sudo apt update
   sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
       libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev
   ```

3. **Önemli:** `dialout` grup yetkisinin geçerli olması için Ubuntu oturumunuzu kapatıp yeniden açın (veya bilgisayarı yeniden başlatın).

4. QGroundControl AppImage dosyasını indirin ve her terminalden `qgroundcontrol` komutuyla çalıştırılacak şekilde `~/.local/bin/` içine koyun:
   ```bash
   mkdir -p ~/.local/bin
   wget -O ~/.local/bin/qgroundcontrol https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
   chmod +x ~/.local/bin/qgroundcontrol
   ```
   *(İndirme bağlantısı değişirse resmi [QGroundControl İndirme Sayfası](https://docs.qgroundcontrol.com/) üzerinden `QGroundControl-x86_64.AppImage` dosyasını indirebilirsiniz.)*

5. `~/.local/bin` dizininin `PATH` içinde olduğundan emin olun:
   ```bash
   if ! echo "$PATH" | grep -q "$HOME/.local/bin"; then
       echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
       source ~/.bashrc
   fi
   which qgroundcontrol
   ```

---

## Adım 5: Uçtan Uca Doğrulama (`sartek_siha_ws` + PX4 SITL)

### Terminal 1 — Micro-XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — PX4 SITL + Gazebo Harmonic
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_advanced_plane
```

### Terminal 3 — ROS 2 Topic ve `px4_msgs` Testi
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash

ros2 topic list | grep /fmu/
# PX4 main (v1.18+) için versiyonlu topic örnekleri:
ros2 topic echo /fmu/out/vehicle_status_v5 --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

### Terminal 4 — QGroundControl *(İsteğe Bağlı)*
```bash
qgroundcontrol
```
QGC açıldığında PX4 SITL aracına otomatik olarak UDP (`14550`) üzerinden bağlanıp haritada konumu göstermelidir.

---

## Adım 6: `sartek_siha_ws` Derleme ve Çakışmasız Çalışma Kuralı

1. `~/.bashrc` dosyanızda **hiçbir eski workspace (`px4_ros_ws`, `drone_ws`, `sartek_ws`) `source` edilmemelidir** — yalnızca `source /opt/ros/humble/setup.bash` bulunmalıdır.
2. `sartek_siha_ws` workspace'ini her zaman temiz ortamda derleyip aktif edin:
   ```bash
   cd ~/sartek_siha_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source ~/sartek_siha_ws/install/setup.bash
   ```
3. `px4_msgs` paketinin doğru yerden okunduğunu kontrol etmek için:
   ```bash
   ros2 pkg prefix px4_msgs
   # Beklenen çıktı: /home/<kullanici>/sartek_siha_ws/install/px4_msgs
   ```

---

## 🔧 Sık Karşılaşılan Sorunlar ve Çözümleri

| Sorun | Sebep | Çözüm |
|---|---|---|
| `MicroXRCEAgent` bağlanıyor ama `ros2 topic echo` veri göstermiyor | `PX4-Autopilot` sürümü ile `~/sartek_siha_ws/src/px4_msgs` sürümü uyuşmuyor veya topic sonundaki `_v<N>` eki farklı | `ros2 topic list \| grep /fmu/` ile gerçek topic adını kontrol edin; gerekirse `src/px4_msgs` commit'ini PX4 sürümünüzle eşleştirip yeniden derleyin |
| `ros2 topic echo` / düğümler veri almıyor (QoS veya DDS sorunu) | Farklı `RMW_IMPLEMENTATION`, farklı `ROS_DOMAIN_ID` veya `BEST_EFFORT` QoS kullanılmaması | Tüm terminallerde `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ve aynı `ROS_DOMAIN_ID` olduğunu doğrulayın; `/fmu/out/*` aboneliklerinde `BEST_EFFORT` kullanın |
| `px4_msgs` yanlış/eski workspace'ten yükleniyor | `~/.bashrc` içinde `~/px4_ros_ws/install/setup.bash` kalmış | `~/.bashrc` içinden eski workspace satırını kaldırın, yeni terminal açıp `ros2 pkg prefix px4_msgs` ile doğrulayın |
| `gz sim` açılmıyor veya siyah ekran / eklenti hatası | Yanlışlıkla `ros-humble-ros-gz` (Fortress) kurulmuş | `sudo apt remove -y ros-humble-ros-gz*` sonrası `gz-harmonic` ve `ros-humble-ros-gzharmonic` kurun |
| PX4 `ubuntu.sh` sonrası `cv_bridge` veya `colcon` çöküyor | `ubuntu.sh` `numpy>=2` veya `setuptools>=80` kurmuş | `pip3 install --user "numpy>=1.23.0,<2.0.0" "opencv-python>=4.8.0,<4.9.0" "packaging>=24.0" "setuptools>=77.0.3,<80.0.0"` çalıştırın |
| Eski simülasyon süreçleri portu kilitliyor | Önceki `px4`, `gz` veya `MicroXRCEAgent` kapanmamış | `pkill -9 -f px4; pkill -9 -f gz; pkill -9 -f MicroXRCEAgent` |
