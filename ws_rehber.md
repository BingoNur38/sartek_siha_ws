# 📘 `sartek_siha_ws` Çalışma ve Kullanım Rehberi (`ws_rehber.md`)

Bu doküman, **`sartek_siha_ws`** çalışma alanının **günlük kullanım, kontrol, başlatma ve geliştirme el kitabıdır**.  
Yeni algoritma, mesaj paketi veya görev düğümü (ör. terminalden waypoint atama, otonom devriye, QR kamikaze, hedef takip) eklendikçe bu rehber ilgili kullanım komutlarıyla güncellenecektir.

> **Diğer Dokümanlarla Farkı:**
> - [INSTALLATION.md](INSTALLATION.md): Sıfır bir görev bilgisayarına tüm sistem bağımlılıklarını kurma rehberidir.
> - [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md): Gazebo Harmonic, `PX4-Autopilot` ve QGroundControl kurulum rehberidir.
> - [README.md](README.md): GitHub'dan çekecek ekip üyeleri için ilk kurulum ve çakışma önleme özetidir.
> - **`ws_rehber.md` (bu dosya):** Kurulumu tamamlanmış workspace'te klasörlerin ne işe yaradığını, başlatma öncesi kontrolleri ve sistemi nasıl çalıştırıp doğrulayacağınızı anlatır.

---

## 1. Workspace Klasör ve Dosya Yapısı (Ne Nedir, Ne İşe Yarar?)

```text
sartek_siha_ws/
├── .gitignore
├── README.md
├── INSTALLATION.md
├── PX4_SIM_KURULUM.md
├── ws_rehber.md
├── src/                                  # Tüm ROS 2 kaynak paketlerinin bulunduğu dizin
│   ├── px4_msgs/                         # PX4 v1.18 (67f5c57e9a) uyumlu uORB <-> ROS 2 mesajları
│   ├── sartek_msgs/                      # Düğümler arası ortak mesaj/aksiyon sözleşmesi (9 msg + 1 action)
│   ├── sartek_px4_bridge/                # PX4 uXRCE-DDS <-> sartek_msgs sınır köprüsü
│   │   ├── sartek_px4_bridge/
│   │   │   ├── conversions.py            # NED<->ENU, quaternion<->Euler, QoS ve PX4 mod yardımcıları
│   │   │   ├── px4_telemetry_bridge.py   # /fmu/out/* -> TelemetryStatus + SystemHealth
│   │   │   └── px4_control_bridge.py     # MissionCommand / SafetyOverride -> /fmu/in/*
│   │   ├── launch/px4_bridge.launch.py   # İki köprü düğümünü birlikte başlatır
│   │   └── test/test_conversions.py      # Koordinat ve quaternion birim testleri
│   └── sartek_sim/                       # Gazebo Harmonic + PX4 SITL simülasyon paketi
│       ├── package.xml                   # Paket bağımlılıkları (ros_gz_sim, ros_gz_image vb.)
│       ├── setup.py / setup.cfg          # Python/ament kurulum ve data_files ayarları
│       ├── resource/sartek_sim           # ament indeks işaret dosyası
│       ├── sartek_sim/__init__.py        # Paket Python modül dizini
│       ├── launch/
│       │   └── sim.launch.py             # Simülasyon + PX4 + Kamera köprüsü + Agent tek launch
│       ├── worlds/
│       │   └── yunuseli.sdf              # Bursa Yunuseli saha dünyası (zemin + QR yerleşimi)
│       └── models/
│           ├── advanced_plane_cam/       # 640x640 ön kameralı sabit kanatlı uçak SDF modeli
│           ├── yunuseli_zemin/           # Yunuseli uydu görüntülü zemin modeli (2125m x 1082m)
│           └── qr_kodu_yuzeyi/           # Yerdeki QR hedef yüzeyi modeli (2.5m x 2.5m)
├── build/                                # colcon derleme ara dosyaları (GitHub'a gönderilmez)
├── install/                              # Derlenmiş çalıştırılabilir dosyalar ve setup.bash (GitHub'a gönderilmez)
└── log/                                  # colcon derleme logları (GitHub'a gönderilmez)
```

### Önemli Dosyaların Görevleri
- **[src/px4_msgs](src/px4_msgs):** Micro XRCE-DDS üzerinden PX4 ile konuşmak için kullanılan mesaj (`msg/`) ve servis (`srv/`) tanımları. `PX4-Autopilot` sürümünüzle (`de6b70e` ↔ `67f5c57e9a`) birebir eşleşir.
- **[src/sartek_msgs](src/sartek_msgs):** Tüm görev, algı, güvenlik ve yer istasyonu düğümlerinin ortak haberleşme dili. Görev algoritmaları `px4_msgs` yerine doğrudan buradaki mesajları (`TelemetryStatus`, `MissionCommand` vb.) kullanır.
- **[src/sartek_px4_bridge](src/sartek_px4_bridge):** Görev düğümlerini `px4_msgs`, NED/FRD eksenleri, `BEST_EFFORT` QoS ve Offboard kalp atışı kurallarından izole eden sınır katmanı (`px4_telemetry_bridge` + `px4_control_bridge`).
- **[src/sartek_sim/worlds/yunuseli.sdf](src/sartek_sim/worlds/yunuseli.sdf):** Yunuseli koordinatlarını (`lat: 40.233243, lon: 29.005583, elev: 101.0m`), zemin modelini (`0, -150, 0.05`) ve QR hedefini (`86.463, -276.128, 0.1`) içeren Gazebo Harmonic dünyası. PX4'ün `server.config` dosyasıyla çakışmaması için içinde tekrar eden `<plugin>` tanımları tutulmaz.
- **[src/sartek_sim/models/advanced_plane_cam/model.sdf](src/sartek_sim/models/advanced_plane_cam/model.sdf):** PX4 v1.18 `advanced_plane` modeli üzerine eklenmiş `front_camera` (640×640, `R8G8B8`, 30 Hz, 58° FOV) sensörünü içerir.
- **[src/sartek_sim/launch/sim.launch.py](src/sartek_sim/launch/sim.launch.py):** `PX4-Autopilot` kaynak kodunu değiştirmeden (`PX4_GZ_STANDALONE=1`) tüm simülasyon hattını doğru sırayla ayağa kaldırır.
- **[.gitignore](.gitignore):** `build/`, `install/`, `log/` gibi derleme çıktılarının ve önbelleklerin GitHub'a gönderilmesini engeller.

---

## 2. Başlatma Öncesi Gerekli Kontroller (Pre-Run Checklist)

Her çalışma oturumuna başlamadan önce çakışma yaşamamak için aşağıdaki kontrolleri yapın:

### 2.1. Eski Simülasyon ve Aracı Süreçlerini Temizleme
Önceki bir çalıştırmadan kalmış `px4`, `gz` veya `MicroXRCEAgent` süreçleri UDP/Gazebo portlarını kilitleyebilir:
```bash
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
```

### 2.2. Ortam Değişkeni ve `~/.bashrc` Çakışma Kontrolü
Terminalinizde eski workspace'lerin (`px4_ros_ws`, `sartek_ws`, `drone_ws`) yüklü olmadığından ve `RMW_IMPLEMENTATION` ayarının aynı olduğundan emin olun:
```bash
echo "RMW: $RMW_IMPLEMENTATION | DOMAIN: ${ROS_DOMAIN_ID:-0}"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
```
**Beklenen durum:**
- `RMW_IMPLEMENTATION` tüm terminallerde **`rmw_cyclonedds_cpp`** olmalıdır.
- `AMENT_PREFIX_PATH` içinde **`px4_ros_ws`, `sartek_ws`, `drone_ws` olmamalı**; yalnızca `/opt/ros/humble` ve (source ettikten sonra) `~/sartek_siha_ws/install/...` görünmelidir.

### 2.3. Workspace Yükleme ve Paket Yolu Doğrulaması
Her yeni terminalde çalıştırmadan önce ortamı yükleyin:
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash

ros2 pkg prefix px4_msgs sartek_msgs sartek_px4_bridge sartek_sim
```
Beklenen çıktı:
```text
/home/<kullanici>/sartek_siha_ws/install/px4_msgs
/home/<kullanici>/sartek_siha_ws/install/sartek_msgs
/home/<kullanici>/sartek_siha_ws/install/sartek_px4_bridge
/home/<kullanici>/sartek_siha_ws/install/sartek_sim
```

### 2.4. PX4 SITL Binary Kontrolü
`sim.launch.py`, derlenmiş PX4 SITL binary'sini (`~/PX4-Autopilot/build/px4_sitl_default/bin/px4`) kullanır. Yeni bir makinede ilk kez çalıştırıyorsanız PX4 tarafında en az bir kez derleme yapılmış olmalıdır:
```bash
ls -la ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
# Eğer yoksa:
# cd ~/PX4-Autopilot && make px4_sitl gz_advanced_plane
```

---

## 3. Sistemi Başlatma

### 3.1. Tek Komutla Tam Simülasyon (Standart Kullanım)

```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 launch sartek_sim sim.launch.py
```

**Arka planda sırayla gerçekleşen adımlar:**
1. Gazebo Harmonic `yunuseli.sdf` dünyasını (PX4 `server.config` ve `gz_plugins` ile) açar.
2. `2.0` saniye sonra `advanced_plane_cam` uçağını `(0, 0, 0.246)` konumunda dünyaya ekler (`ros_gz_sim create`).
3. Spawn tamamlandığı anda aynı anda şunları başlatır:
   - **PX4 SITL (`px4 -d`):** `PX4_GZ_STANDALONE=1`, `PX4_SYS_AUTOSTART=4008` (`advanced_plane` parametreleri) ile uçağa bağlanır.
   - **Kamera Köprüsü (`ros_gz_image`):** Gazebo `/front_camera` görüntüsünü ROS 2 `/camera/image_raw` topic'ine aktarır.
   - **MicroXRCEAgent (`udp4 -p 8888`):** PX4 uORB topic'lerini ROS 2 `/fmu/...` topic'lerine açar.

### 3.2. Faydalı Launch Parametreleri

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `headless` | `false` | `headless:=true` ile Gazebo penceresi açılmadan arka planda çalıştırır |
| `px4_dir` | `~/PX4-Autopilot` | `PX4-Autopilot` klasörünün yolu (`PX4_DIR` varsa onu alır) |
| `world_name` | `yunuseli` | `sartek_sim/worlds/` altındaki dünya dosya adı (`.sdf` olmadan) |
| `vehicle_model` | `advanced_plane_cam` | `sartek_sim/models/` altındaki uçak model adı |
| `start_px4` | `true` | `false` yapılarak yalnızca Gazebo başlatılabilir |
| `start_bridge` | `true` | `false` yapılarak kamera köprüsü devre dışı bırakılabilir |
| `start_agent` | `true` | `false` yapılarak `MicroXRCEAgent` ayrı terminalde çalıştırılabilir |
| `agent_port` | `8888` | Micro XRCE-DDS UDP portu |
| `spawn_x` / `spawn_y` / `spawn_z` / `spawn_yaw` | `0.0` / `0.0` / `0.246` / `0.0` | Uçağın başlangıç konumu (metre) ve yönelimi (radyan) |

Örnek (GUI'siz performans testi):
```bash
ros2 launch sartek_sim sim.launch.py headless:=true
```

### 3.3. QGroundControl (Opsiyonel — Harita ve Kalkış İzleme)
Simülasyon çalışırken ayrı bir terminalde QGroundControl'ü açarak uçağın harita konumunu izleyebilir ve manuel/otonom kalkış verebilirsiniz:
```bash
qgroundcontrol
```

---

## 4. Sistem Doğrulama ve İzleme Komutları

Simülasyon başladıktan ~10–15 saniye sonra **ikinci bir terminalde** (`source /opt/ros/humble/setup.bash && source ~/sartek_siha_ws/install/setup.bash` sonrası) hattın sağlıklı aktığını aşağıdaki komutlarla doğrulayın:

### 4.1. Topic Listesi Kontrolü
```bash
ros2 topic list | grep -E "^/camera/|/fmu/"
```
Temel olarak görmeniz gerekenler:
- `/camera/image_raw`
- `/fmu/out/vehicle_status_v5`
- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_global_position`
- `/fmu/out/airspeed_validated_v1`
- `/fmu/out/battery_status_v1`

### 4.2. Kamera Akışı Kontrolü
```bash
# Mesaj başlık/format kontrolü (640x640, encoding: rgb8)
ros2 topic echo /camera/image_raw --no-arr --once

# Yayın hızı (FPS) kontrolü (~30 Hz beklenir)
ros2 topic hz /camera/image_raw

# Canlı görüntüyü pencere içinde izleme
ros2 run rqt_image_view rqt_image_view
```

### 4.3. PX4 Telemetri Akışı Kontrolü (`px4_msgs`)
```bash
# Uçak tipi (2 = Fixed-Wing), arm ve nav_state durumu
ros2 topic echo /fmu/out/vehicle_status_v5 --once

# Lokal pozisyon ve hızlar (NED: X=Kuzey, Y=Doğu, Z=Aşağı negatif irtifa)
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once

# Hava hızı (Airspeed) ve Batarya durumu
ros2 topic echo /fmu/out/airspeed_validated_v1 --once
ros2 topic echo /fmu/out/battery_status_v1 --once
```

### 4.4. Gazebo Tarafı Ham Topic Kontrolü
```bash
gz topic -l | grep -E "front_camera|yunuseli|advanced_plane_cam"
```

---

## 5. Kod Değişikliği Sonrası Yeniden Derleme

`src/sartek_sim` veya ileride eklenecek yeni paketlerde değişiklik yaptığınızda temiz ortamdan derleyin:
```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/sartek_siha_ws/install/setup.bash
```

---

## 6. Ortak Mesaj Paketi (`sartek_msgs`) Sözleşmesi

Tüm görev geliştiricileri `px4_msgs` karmaşasından bağımsız olarak aşağıdaki `sartek_msgs` arayüzleri üzerinden geliştirme yapar:

| Kategori | Mesaj / Aksiyon | Amaç |
|---|---|---|
| **Telemetri & Sağlık** | `sartek_msgs/msg/TelemetryStatus` | Hız (`airspeed`, `groundspeed`, `vel_x/y/z`), ENU konum (`pos_x/y/z`, `altitude`, `altitude_amsl`), yönelim (`roll_deg`, `pitch_deg`, `yaw_deg`), GPS/Home (`latitude`, `longitude`, `home_*`), batarya ve IMU özeti |
| **Telemetri & Sağlık** | `sartek_msgs/msg/SystemHealth` | Batarya, GPS, IMU titreşimi, `is_armed`, `flight_mode`, `rc_connected`, `is_emergency` |
| **Görüntü İşleme** | `sartek_msgs/msg/QrMissionStatus` | QR kilidi, decode metni, normalize hata (`error_x/y` ∈ `[-1, 1]`), `area_ratio`, `confidence` |
| **Görüntü İşleme** | `sartek_msgs/msg/DogfightStatus` | Düşman İHA tespiti, kilit, normalize bbox/hata (`[-1, 1]`), `area_ratio`, PID gözlem değerleri |
| **Görüntü İşleme** | `sartek_msgs/msg/TargetInfo` | Genel/trafik hedef konumu (ENU), bbox ve hata bilgileri |
| **Kontrol & Görev** | `sartek_msgs/msg/MissionCommand` | `GOTO`/`ORBIT`/`ATTITUDE`/`RTL`/`LOITER`/`LAND` komutları (`target_x/y/z`, `speed`, `roll_deg`, `pitch_deg`, `yaw_deg`, `thrust`, `commander_id`) |
| **Kontrol & Görev** | `sartek_msgs/msg/SafetyOverride` | Acil durum override (`LOITER`, `RTL`, `FULL_THROTTLE`, `CLIMB`) |
| **Kontrol & Görev** | `sartek_msgs/action/ExecuteMission` | Coordinator → Görev işçisi başlatma / iptal (preemption) / geri bildirim aksiyonu |
| **Saha & Haberleşme** | `sartek_msgs/msg/NfzInfo` | Yasaklı uçuş alanı (HSS) 3D silindir koordinat ve yarıçapı |
| **Saha & Haberleşme** | `sartek_msgs/msg/GcsCommand` | Yer istasyonu komutu, NFZ listesi, rakip trafik listesi ve enjekte waypoint rotası |

**Arayüz Kontrol Komutları:**
```bash
ros2 interface list | grep sartek_msgs
ros2 interface show sartek_msgs/msg/TelemetryStatus
ros2 interface show sartek_msgs/msg/MissionCommand
```

> **Ekip İçi Standartlar:**
> - **Koordinat:** Görev düğümleri arasındaki lokal pozisyonlar **ENU** (`X=Doğu, Y=Kuzey, Z=Yukarı +`) cinsindendir; NED↔ENU çevrimi PX4 köprüsünde yapılır.
> - **Kamera Hataları:** `QrMissionStatus` ve `DogfightStatus` içindeki `error_x` / `error_y` değerleri **`[-1.0, +1.0]` normalize**, `bbox_cx/cy/w/h` değerleri **`[0.0, 1.0]` normalize** kullanılır.

---

## 7. PX4 Sınır / Uçuş Omurgası (`sartek_px4_bridge`)

Görev yazan geliştiricilerin `px4_msgs` topic'leri, NED↔ENU dönüşümü veya Offboard zaman damgalarıyla uğraşmaması için `sartek_px4_bridge` paketi iki köprü düğümü sunar.

### 7.1. Köprüyü Başlatma
Simülasyon (`ros2 launch sartek_sim sim.launch.py`) veya gerçek uçuşta `MicroXRCEAgent` çalıştıktan sonra ayrı bir terminalde başlatın:
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 launch sartek_px4_bridge px4_bridge.launch.py
```

**Launch Parametreleri:**
- `fmu_prefix:=""` → Tekli İHA'da boş bırakılır (`/fmu/...`); çoklu İHA için örn. `/uav1` verilebilir.
- `active_commander:=""` → Boşken her düğümden gelen `MissionCommand`'ı kabul eder; koordinatör devredeyken yetkili `commander_id` adı (örn. `mission_patrol`) verilebilir.

### 7.2. Köprülerin İşlevi ve Topic Haritası

| Düğüm | Dinlediği Topic'ler | Yayınladığı Topic'ler |
|---|---|---|
| **`px4_telemetry_bridge`** | `/fmu/out/vehicle_local_position_v1`, `vehicle_attitude`, `vehicle_global_position`, `airspeed_validated_v1`, `battery_status_v1`, `vehicle_status_v5`, `home_position_v2`, `vehicle_gnss`, `sensor_combined`, `failsafe_flags` | `/gorev/telemetry_status` (`TelemetryStatus`), `/gorev/system_health` (`SystemHealth`) |
| **`px4_control_bridge`** | `/gorev/mission_command` (`MissionCommand`), `/gorev/safety_override` (`SafetyOverride`), `/gorev/vehicle_command` (`std_msgs/String`), `/fmu/out/vehicle_status_v5`, `vehicle_command_ack_v2` | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_attitude_setpoint_v1`, `/fmu/in/vehicle_rates_setpoint`, `/fmu/in/vehicle_command_v1` |

> **Önemli Standartlar:**
> - `TelemetryStatus` ve `MissionCommand` içindeki lokal koordinatlar **ENU** (`X=Doğu`, `Y=Kuzey`, `Z=Yukarı +`) cinsindendir; köprü bunları PX4'e **NED** (`[Y, X, -Z]`) olarak aktarır.
> - Açılar: `roll_deg` (sağ kanat aşağı `+`), `pitch_deg` (**burun yukarı `+`**, burun aşağı dalış `-`), `yaw_deg` (`0..360°`, Kuzey'den saat yönünde pusula yönelimi).
> - `home_set` alanı PX4 ilk Arm / Home kilitlemesini yaptığında `true` olur.

### 7.3. Terminalden Hızlı Test Komutları

**1) Telemetri ve Sistem Sağlığını İzleme:**
```bash
ros2 topic echo /gorev/telemetry_status --once
ros2 topic echo /gorev/system_health --once
```

**2) Waypoint (`GOTO`) Komutu Gönderme (ENU: 150m Doğu, 200m Kuzey, 90m İrtifa):**
```bash
ros2 topic pub --once /gorev/mission_command sartek_msgs/msg/MissionCommand \
  "{command_type: 'GOTO', target_x: 150.0, target_y: 200.0, target_z: 90.0, speed: 20.0, commander_id: 'terminal'}"
```

**3) Tutum / Dalış (`ATTITUDE`) Komutu Gönderme (Örn. -25° Burun Aşağı Dalış, %20 Gaz):**
```bash
ros2 topic pub --once /gorev/mission_command sartek_msgs/msg/MissionCommand \
  "{command_type: 'ATTITUDE', roll_deg: 0.0, pitch_deg: -25.0, yaw_deg: 90.0, thrust: 0.2, commander_id: 'terminal'}"
```

**4) Terminalden Mod Değiştirme / Arm / Kalkış (`gorev/vehicle_command`):**
```bash
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'ARM'}"
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'TAKEOFF'}"
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'OFFBOARD'}"
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'LOITER'}"
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'RTL'}"
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'DISARM'}"
```

**5) Acil Durum Override Testi:**
```bash
ros2 topic pub --once /gorev/safety_override sartek_msgs/msg/SafetyOverride \
  "{override_active: true, override_mode: 'LOITER', reason: 'manuel_test'}"

# Override'ı kaldırmak için:
ros2 topic pub --once /gorev/safety_override sartek_msgs/msg/SafetyOverride \
  "{override_active: false, override_mode: '', reason: 'normal'}"
```

### 7.4. Dönüşüm Birim Testlerini Çalıştırma
```bash
cd ~/sartek_siha_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/sartek_px4_bridge/test/test_conversions.py -v
```

---

## 8. Gelecek Bölümler (Algoritmalar Eklendikçe Güncellenecek)

Yeni paketler ve görev algoritmaları geliştirildikçe bu bölüme kullanım talimatları eklenecektir:
- **8.1. Sabit Kanat Kalkış + Çoklu Waypoint Devriye ve HSS (NFZ) Kaçınma:** *(Patrol & NFZ düğümleri eklendikten sonra)*
- **8.2. QR Görüntü İşleme ve Kamikaze Dalış Görevi:** *(QR düğümleri eklendikten sonra)*
- **8.3. Dogfight / Hedef Takip ve Kaçış Görevleri:** *(Dogfight düğümleri eklendikten sonra)*
